#define COMPILE_SCRIPT /*
	set -- -O2 -ggdb -std=c++11 -pthread -Wall
	g++ "$0" -o "${0%.*}" "$@"
	exit
*/

#include <stdarg.h>
#include <stdint.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <memory>
#include <thread>

static inline uint32_t rotateRight(uint32_t val, uint32_t count) {
	return (val >> count) | (val << (32 - count));
}

class IMX233 {
	struct TickState;
public:
	static constexpr uint32_t systemMemoryBase = 0x40000000;
	enum CPUMode {
		CPU_MODE_USER = 0x10,
		CPU_MODE_FIQ = 0x11,
		CPU_MODE_IRQ = 0x12,
		CPU_MODE_SVC = 0x13,
		CPU_MODE_ABT = 0x17,
		CPU_MODE_UND = 0x1B,
		CPU_MODE_SYSTEM = 0x1F,
	};
	enum PSRBits {
		PSR_BITS_MODE = 0x1F,
		PSR_BITS_T = 1 << 5,
		PSR_BITS_F = 1 << 6,
		PSR_BITS_I = 1 << 7,
		PSR_BITS_A = 1 << 8,
		PSR_BITS_E = 1 << 9,
		PSR_BITS_J = 1 << 24,
		PSR_BITS_V = 1 << 28,
		PSR_BITS_C = 1 << 29,
		PSR_BITS_Z = 1 << 30,
		PSR_BITS_N = 1 << 31,
	};
	enum TickError {
		TICK_ERROR_NONE = 0,
		TICK_ERROR_PREFETCH_ABORT,
		TICK_ERROR_DATA_ABORT,
	};
	enum PendingOperation {
		PENDING_OPERATION_NONE = 0,
		PENDING_OPERATION_LDR_STR,
		PENDING_OPERATION_LDM_STM,
	};
public:
	IMX233() :
			registerFile(*this),
			memoryController(*this),
			systemControlCoprocessor(*this) {
		reset();
	}
	void reset() {
		registerFile.reset();
		memoryController.reset();
		systemControlCoprocessor.reset();
		currentTick.reset();
	}
	void tick() {
		switch (currentTick.pendingOperation) {
			case PENDING_OPERATION_NONE:
				currentTick.tickError = TICK_ERROR_NONE;
				registerFile.setProgramCounter(currentTick.curPC + 4);
				tickExecute();
				break;
			case PENDING_OPERATION_LDR_STR:
				tickPendingLDRSTR();
				break;
			case PENDING_OPERATION_LDM_STM:
				tickPendingLDMSTM();
				break;
		}
		if (currentTick.tickError != TICK_ERROR_NONE) {
			currentTick.pendingOperation = PENDING_OPERATION_NONE;
			dumpAndAbort("tick error occurred");
		}
		if (currentTick.pendingOperation == PENDING_OPERATION_NONE) {
			currentTick.curPC = registerFile.getProgramCounter();
		}
	}
	void tickExecute() {
		bool errorOccurred = false;
		// fetch instruction
		uint32_t encodedInst = memoryController.readWord(getPC(), &errorOccurred);
		if (errorOccurred) {
			currentTick.tickError = TICK_ERROR_PREFETCH_ABORT;
			return;
		}
		// only execute if condition passes
		bool condPass;
		unsigned int condType = encodedInst >> 28;
		switch (condType) {
			case 0:
				condPass = readCPSR() & PSR_BITS_Z;
				break;
			case 1:
				condPass = !(readCPSR() & PSR_BITS_Z);
				break;
			case 2:
				condPass = readCPSR() & PSR_BITS_C;
				break;
			case 3:
				condPass = !(readCPSR() & PSR_BITS_C);
				break;
			case 8:
				condPass = (readCPSR() & (PSR_BITS_C | PSR_BITS_Z)) == PSR_BITS_C;
				break;
			case 9:
				condPass = (readCPSR() ^ PSR_BITS_C) & (PSR_BITS_C | PSR_BITS_Z);
				break;
			case 14:
				condPass = true;
				break;
			case 15:
				dumpAndAbort("unconditional instructions unimplemented");
				break;
			default:
				dumpAndAbort("unknown condition %d", condType);
				break;
		}
		if (!condPass)
			return;
		// decode and dispatch
		unsigned int dec1 = (encodedInst >> 25) & 0x07;
		unsigned int dec2 = (encodedInst >> 20) & 0x1F;
		unsigned int dec3 = (encodedInst >> 4) & 0x0F;
		unsigned int Rn = (encodedInst >> 16) & 0x0F;
		unsigned int Rd = (encodedInst >> 12) & 0x0F;
		unsigned int Rs = (encodedInst >> 8) & 0x0F;
		unsigned int Rm = (encodedInst >> 0) & 0x0F;
		switch (dec1) {
			case 0:
				switch (dec2) {
					case 16:
					case 20:
					case 18:
					case 22:
						dumpAndAbort("decode 0.%d unknown", dec2);
						break;
					default:
						if ((dec3 & 0x09) != 0x09) {
							// data processing with register
							uint32_t shifter_operand;
							bool shifter_carry_out;
							decodeShifterOperand(encodedInst, &shifter_operand, &shifter_carry_out);
							inst_DATA(
									dec2 >> 1, /* opcode */
									dec2 & 0x01, /* S */
									Rd,
									Rn,
									shifter_operand,
									shifter_carry_out);
						} else {
							dumpAndAbort("multiply or extra L/S unimplemented");
						}
						break;
				}
				break;
			case 1:
				switch (dec2) {
					case 16:
					case 20: // undefined
						dumpAndAbort("decode 1.x undefined");
						break;
					case 18: // MSR CPSR, #immed
					case 22: // MSR SPSR, #immed
						{
							uint32_t _8_bit_immediate = encodedInst & 0xFF;
							uint32_t rotate_imm = (encodedInst >> 8) & 0x0F;
							uint32_t operand = rotateRight(_8_bit_immediate, rotate_imm * 2);
							inst_MSR(
									encodedInst & (1 << 22), /* R */
									(encodedInst >> 16) & 0x0F, /* field_mask */
									operand);
						}
						break;
					default: // data processing with immediate
						{
							uint32_t immed_8 = encodedInst & 0xFF;
							uint32_t rotate_imm = (encodedInst >> 8) & 0x0F;
							uint32_t shifter_operand = rotateRight(immed_8, rotate_imm * 2);
							bool shifter_carry_out = (rotate_imm == 0) ?
								readCPSR() & PSR_BITS_C :
								shifter_operand & (1 << 31);
							inst_DATA(
									dec2 >> 1, /* opcode */
									dec2 & 0x01, /* S */
									Rd,
									Rn,
									shifter_operand,
									shifter_carry_out);
						}
						break;
				}
				break;
			case 2: // LDR/STR immediate offset
				{
					bool P = encodedInst & (1 << 24);
					bool U = encodedInst & (1 << 23);
					bool B = encodedInst & (1 << 22);
					bool W = encodedInst & (1 << 21);
					bool L = encodedInst & (1 << 20);
					uint32_t offset_12 = encodedInst & 0xFFF;
					inst_LDR_STR(L, B, P, U, W, Rd, Rn, offset_12);
				}
				break;
			case 3: // LDR/STR register offset
				if (!(dec3 & 0x01)) {
					bool P = encodedInst & (1 << 24);
					bool U = encodedInst & (1 << 23);
					bool B = encodedInst & (1 << 22);
					bool W = encodedInst & (1 << 21);
					bool L = encodedInst & (1 << 20);
					uint32_t shifter_operand;
					bool shifter_carry_out;
					decodeShifterOperand(encodedInst, &shifter_operand, &shifter_carry_out);
					inst_LDR_STR(L, B, P, U, W, Rd, Rn, shifter_operand);
				} else {
					dumpAndAbort("unknown decode 3");
				}
				break;
			case 4: // LDM/STM
				{
					bool P = encodedInst & (1 << 24);
					bool U = encodedInst & (1 << 23);
					bool S = encodedInst & (1 << 22);
					bool W = encodedInst & (1 << 21);
					bool L = encodedInst & (1 << 20);
					uint32_t register_list = encodedInst & 0xFFFF;
					inst_LDM_STM(L, S, P, U, W, Rn, register_list);
				}
				break;
			case 5: // B/BL
				inst_B_BL(
						encodedInst & (1 << 24), /* L */
						encodedInst & 0x00FFFFFF);
				break;
			case 7:
				switch (dec2) {
					case 1: case 3: case 5: case 7:
					case 9: case 11: case 13: case 15: // MRC/CDP
						if (dec3 & 0x01) {
							// MRC
							inst_MRC(
									(encodedInst >> 8) & 0x0F, /* cp_num */
									(encodedInst >> 21) & 0x07, /* opcode_1 */
									Rd,
									Rn,
									Rm,
									(encodedInst >> 5) & 0x07 /* opcode_2 */);
						} else {
							// CDP
							dumpAndAbort("CDP unimplemented");
						}
						break;
					case 0: case 2: case 4: case 6:
					case 8: case 10: case 12: case 14: // MCR/CDP
						if (dec3 & 0x01) {
							// MCR
							inst_MCR(
									(encodedInst >> 8) & 0x0F, /* cp_num */
									(encodedInst >> 21) & 0x07, /* opcode_1 */
									Rd,
									Rn,
									Rm,
									(encodedInst >> 5) & 0x07 /* opcode_2 */);
						} else {
							// CDP
							dumpAndAbort("CDP unimplemented");
						}
						break;
					default:
						dumpAndAbort("decode 7.%d unknown", dec2);
						break;
				}
				break;
			default:
				dumpAndAbort("decode %d unknown", dec1);
				break;
		}
	}
	void tickPendingLDRSTR() {
		bool errorOccurred = false;
		auto& st = pendingOperationState.ldr_str;
		if (st.byte)
			dumpAndAbort("LDRB/STRB not implemented");
		if (st.load) {
			uint32_t data = memoryController.readWord(st.address, &errorOccurred);
			if (errorOccurred) {
				currentTick.tickError = TICK_ERROR_DATA_ABORT;
				return;
			}
			writeRegister(st.Rd, data);
		} else {
			uint32_t data = readRegister(st.Rd);
			memoryController.writeWord(st.address, data, &errorOccurred);
			if (errorOccurred) {
				currentTick.tickError = TICK_ERROR_DATA_ABORT;
				return;
			}
		}
		if (st.writeback)
			writeRegister(st.Rn, st.Rn_final);
		currentTick.pendingOperation = PENDING_OPERATION_NONE;
	}
	void tickPendingLDMSTM() {
		bool errorOccurred = false;
		auto& st = pendingOperationState.ldm_stm;
		if (!st.load)
			dumpAndAbort("STM not implemented");
		if (st.special)
			dumpAndAbort("LDM/STM special not implemented");
		uint32_t value = memoryController.readWord(st.address, &errorOccurred);
		if (errorOccurred) {
			currentTick.tickError = TICK_ERROR_DATA_ABORT;
			return;
		}
		if (st.up) {
			unsigned int Rd = __builtin_ctz(st.register_list);
			st.register_list &= ~(1 << Rd);
			writeRegister(Rd, value);
			st.address += 4;
			st.Rn_final += 4;
		} else {
			dumpAndAbort("LDM/STM down not implemented");
		}
		if (!st.register_list) {
			if (st.writeback)
				writeRegister(st.Rn, st.Rn_final);
			currentTick.pendingOperation = PENDING_OPERATION_NONE;
		}
	}
	void inst_DATA(unsigned int opcode, bool S,
			unsigned int Rd, unsigned int Rn, uint32_t shifter_operand, bool shifter_carry_out) {
		uint32_t alu_out;
		uint32_t Rn_value = readRegister(Rn);
		switch (opcode) {
			case 0:
			case 8:
				alu_out = Rn_value & shifter_operand;
				break;
			case 2:
			case 10:
				alu_out = Rn_value - shifter_operand;
				break;
			case 4:
			case 11:
				alu_out = Rn_value + shifter_operand;
				break;
			case 9:
				alu_out = Rn_value ^ shifter_operand;
				break;
			case 12:
				alu_out = Rn_value | shifter_operand;
				break;
			case 13:
				alu_out = shifter_operand;
				break;
			case 14:
				alu_out = Rn_value & ~shifter_operand;
				break;
			case 15:
				alu_out = ~shifter_operand;
				break;
			default:
				dumpAndAbort("data opcode %u unimplemented", opcode);
				break;
		}
		if (S) {
			if (Rd == 15)
				dumpAndAbort("Rd15 data and S not supported");
			uint32_t newCPSR =
				(readCPSR() & ~(PSR_BITS_N | PSR_BITS_Z | PSR_BITS_C)) |
				(alu_out & PSR_BITS_N) |
				(alu_out == 0 ? PSR_BITS_Z : 0) |
				(shifter_carry_out ? PSR_BITS_C : 0);
			bool a, b, r = alu_out & (1 << 31);
			switch (opcode) {
				case 8:
				case 9:
				case 13:
					break;
				case 4:
				case 11:
					a = Rn_value & (1 << 31);
					b = shifter_operand & (1 << 31);
					newCPSR =
						(newCPSR & ~(PSR_BITS_C | PSR_BITS_V)) |
						((a & b) | (a & !r) | (b & !r) ? PSR_BITS_C : 0) |
						((!a & !b & r) | (a & b & !r) ? PSR_BITS_V : 0);
					break;
				case 2:
				case 10:
					a = Rn_value & (1 << 31);
					b = shifter_operand & (1 << 31);
					newCPSR =
						(newCPSR & ~(PSR_BITS_C | PSR_BITS_V)) |
						((a & !b) | (a & !r) | (!b & !r) ? PSR_BITS_C : 0) |
						((!a & b & r) | (a & !b & !r) ? PSR_BITS_V : 0);
					break;
				default:
					dumpAndAbort("data opcode %u S unimplemented", opcode);
					break;
			}
			writeCPSR(newCPSR);
		}
		if ((opcode & 0x0C) != 0x08)
			writeRegister(Rd, alu_out);
	}
	void inst_B_BL(bool L, uint32_t signed_immed_24) {
		if (L)
			writeRegister(14, getPC() + 4);
		if (signed_immed_24 & (1 << 23))
			signed_immed_24 |= 0xFF << 24;
		writeRegister(15, readRegister(15) + (signed_immed_24 << 2));
	}
	void inst_LDR_STR(
			bool L, bool B, bool P, bool U, bool W,
			unsigned int Rd, unsigned int Rn, uint32_t offset_12) {
		currentTick.pendingOperation = PENDING_OPERATION_LDR_STR;
		auto& st = pendingOperationState.ldr_str;
		st.load = L;
		st.byte = B;
		st.writeback = (P == W);
		st.Rn = Rn;
		st.Rd = Rd;
		uint32_t Rn_value = readRegister(Rn);
		uint32_t offsettedAddress = U ?
			Rn_value + offset_12 :
			Rn_value - offset_12;
		st.Rn_final = offsettedAddress;
		if (P)
			st.address = offsettedAddress;
		else
			st.address = Rn_value;
		if (!P && W)
			dumpAndAbort("LDRT/STRT not implemented");
	}
	void inst_LDM_STM(
			bool L, bool S, bool P, bool U, bool W, unsigned int Rn, uint32_t register_list) {
		if (!register_list)
			return;
		currentTick.pendingOperation = PENDING_OPERATION_LDM_STM;
		auto& st = pendingOperationState.ldm_stm;
		st.load = L;
		st.special = S;
		st.up = U;
		st.writeback = W;
		st.Rn = Rn;
		st.register_list = register_list;
		uint32_t Rn_value = readRegister(Rn);
		st.Rn_final = Rn_value;
		if (U) {
			if (P) // increment before
				st.address = Rn_value + 4;
			else // increment after
				st.address = Rn_value;
		} else {
			if (P) // decrement before
				st.address = Rn_value - 4;
			else // decrement after
				st.address = Rn_value;
		}
	}
	void inst_MCR(uint32_t cp_num, uint32_t opcode_1,
			unsigned int Rd, unsigned int CRn, unsigned int CRm, unsigned int opcode_2) {
		if (cp_num != 15)
			dumpAndAbort("access to coproc other than CP15");
		if ((readCPSR() & PSR_BITS_MODE) == CPU_MODE_USER)
			dumpAndAbort("MRC in non-privileged mode");
		uint32_t data = readRegister(Rd);
		systemControlCoprocessor.write(opcode_1, CRn, CRm, opcode_2, data);
	}
	void inst_MRC(uint32_t cp_num, uint32_t opcode_1,
			unsigned int Rd, unsigned int CRn, unsigned int CRm, unsigned int opcode_2) {
		if (cp_num != 15)
			dumpAndAbort("access to coproc other than CP15");
		if ((readCPSR() & PSR_BITS_MODE) == CPU_MODE_USER)
			dumpAndAbort("MRC in non-privileged mode");
		uint32_t data = systemControlCoprocessor.read(opcode_1, CRn, CRm, opcode_2);
		if (Rd == 15) {
			uint32_t mask = 0x0F << 28;
			writeCPSR((readCPSR() & ~mask) | (data & mask));
		} else {
			writeRegister(Rd, data);
		}
	}
	void inst_MSR(bool R, uint32_t field_mask, uint32_t operand) {
		uint32_t mask =
			((field_mask & (1 << 0)) ? 0x000000FF : 0) |
			((field_mask & (1 << 1)) ? 0x0000FF00 : 0) |
			((field_mask & (1 << 2)) ? 0x00FF0000 : 0) |
			((field_mask & (1 << 3)) ? 0xFF000000 : 0);
		if (operand & 0x0FFFFF00)
			dumpAndAbort("MSR: attempted to set reserved bits");
		if (!R) {
			if (isPrivileged()) {
				if (operand & 0x00000020)
					dumpAndAbort("MSR: attempted to set non-ARM execution state");
				mask &= 0xF0000000 | 0x0000000F;
			} else {
				mask &= 0xF0000000;
			}
			writeCPSR((readCPSR() & ~mask) | (operand & mask));
		} else {
			mask &= 0xF0000000 | 0x0000000F | 0x00000020;
			writeSPSR((readSPSR() & ~mask) | (operand & mask));
		}
	}
	__attribute__((noreturn, format(printf, 2, 3)))
	void dumpAndAbort(const char *format, ...) {
		// print message line
		fprintf(stderr, "Abort: ");
		va_list args;
		va_start(args, format);
		vfprintf(stderr, format, args);
		va_end(args);
		fputc('\n', stderr);
		// dump info
		for (int i = 0; i < 15; i++)
			fprintf(stderr, "r%d = %08x\n", i, readRegister(i));
		uint32_t pc = getPC();
		bool err;
		fprintf(stderr, "pc = %08x (%08x)\n", pc, memoryController.readWord(pc, &err));
		// goodbye
		abort();
	}
	void decodeShifterOperand(uint32_t encodedInst, uint32_t *shifter_operand, bool *shifter_carry_out) {
		unsigned int dec3 = (encodedInst >> 4) & 0x0F;
		unsigned int Rs = (encodedInst >> 8) & 0x0F;
		unsigned int Rm = (encodedInst >> 0) & 0x0F;
		bool is_reg = dec3 & 0x01;
		uint32_t shift_type = (dec3 >> 1) & 0x03;
		uint32_t shift_imm;
		*shifter_carry_out = readCPSR() & PSR_BITS_C;
		*shifter_operand = readRegister(Rm);
		if (is_reg)
			shift_imm = readRegister(Rs) & 0xFF;
		else
			shift_imm = (encodedInst >> 7) & 0x1F;
		switch (shift_type) {
			case 0: // LSL
				if (shift_imm < 32) {
					*shifter_carry_out = *shifter_operand & (1 << (32 - shift_imm));
					*shifter_operand <<= shift_imm;
				} else {
					dumpAndAbort("LSL bad shift %u", shift_imm);
				}
				break;
			case 1: // LSR
				if (!is_reg && shift_imm == 0)
					shift_imm = 32;
				if (shift_imm < 32) {
					*shifter_carry_out = *shifter_operand & (1 << (shift_imm - 1));
					*shifter_operand >>= shift_imm;
				} else {
					dumpAndAbort("LSR bad shift %u", shift_imm);
				}
				break;
			default:
				dumpAndAbort("unknown shift type %d", shift_type);
				break;
		}
	}
	void allocateMemory(uint32_t size) {
		systemMemory.reset(new uint32_t[size / 4 + 1]);
		systemMemorySize = size;
	}
	void loadImage(const void *base, uint32_t size, uint32_t location) {
		if (size & 3) {
			std::cerr << "Warning: Image size is not word-aligned." << std::endl;
			size = (size + 3) &~3;
		}
		if (location & 3) {
			std::cerr << "Error: Image location is not word-aligned." << std::endl;
			exit(1);
		}
		if (location + size > systemMemorySize) {
			std::cerr << "Error: Image does not fit in allocated memory." << std::endl;
			exit(1);
		}
		uint32_t wordCount = size / 4;
		const uint32_t *srcBase = static_cast<const uint32_t *>(base);
		uint32_t *dstBase = systemMemory.get() + location / 4;
		for (uint32_t i = 0; i < wordCount; i++)
			dstBase[i] = le32toh(srcBase[i]);
	}
	bool isPrivileged() {
		return (readCPSR() & PSR_BITS_MODE) != CPU_MODE_USER;
	}
	uint32_t readCPSR() { return registerFile.readCPSR(); }
	void writeCPSR(uint32_t val) { registerFile.writeCPSR(val); }
	uint32_t readSPSR() { return registerFile.readSPSR(); }
	void writeSPSR(uint32_t val) { registerFile.writeSPSR(val); }
	uint32_t readRegister(uint32_t reg) { return registerFile.readRegister(reg); }
	void writeRegister(uint32_t reg, uint32_t val) { registerFile.writeRegister(reg, val); }
	uint32_t getPC() { return currentTick.curPC; }
	void setPC(uint32_t pc) { currentTick.curPC = pc; }
private:
	struct TickState {
		TickError tickError;
		PendingOperation pendingOperation;
		uint32_t curPC;
		void reset() {
			tickError = TICK_ERROR_NONE;
			pendingOperation = PENDING_OPERATION_NONE;
			curPC = 0;
		}
	};
	class RegisterFile {
	public:
		RegisterFile(IMX233& core) : core(core) {
			registerView[15] = &programCounter;
			for (int i = 0; i <= 7; i++)
				registerView[i] = &(nonBanked[i]);
		}
		void reset() {
			writeCPSR(CPU_MODE_SVC | PSR_BITS_F | PSR_BITS_I | PSR_BITS_A);
		}
		uint32_t readCPSR() {
			return curCPSR;
		}
		void writeCPSR(uint32_t newCPSR) {
			// check for unsupported values
			if (newCPSR & (PSR_BITS_T | PSR_BITS_J))
				core.dumpAndAbort("T or J bits enabled");
			// save CPSR
			curCPSR = newCPSR;
			// modify banks
			uint32_t newMode = newCPSR & PSR_BITS_MODE;
			if (newMode == CPU_MODE_SYSTEM)
				newMode = CPU_MODE_USER; // same registers are used
			for (int i = 8; i <= 12; i++)
				registerView[i] = &(fiqBanked[i - 8][newMode == CPU_MODE_FIQ]);
			for (int i = 13; i <= 14; i++)
				registerView[i] = &(allBanked[i - 13][newMode & 0x0F]);
			// modify SPSR view
			curSPSRView = &(storedSPSR[newMode & 0x0F]);

		}
		uint32_t readSPSR() {
			if (curSPSRView == &(storedSPSR[0]))
				core.dumpAndAbort("read from non-existent SPSR");
			return *curSPSRView;
		}
		void writeSPSR(uint32_t val) {
			if (curSPSRView == &(storedSPSR[0]))
				core.dumpAndAbort("write to non-existent SPSR");
			*curSPSRView = val;
		}
		uint32_t readRegister(uint32_t reg) {
			uint32_t *regPtr = registerView[reg];
			if (regPtr == &programCounter)
				return programCounter + 4;
			else
				return *regPtr;
		}
		void writeRegister(uint32_t reg, uint32_t val) {
			*(registerView[reg]) = val;
		}
		uint32_t getProgramCounter() {
			return programCounter;
		}
		void setProgramCounter(uint32_t pc) {
			programCounter = pc;
		}
	private:
		IMX233& core;
		uint32_t curCPSR;
		uint32_t storedSPSR[16];
		uint32_t nonBanked[8];
		uint32_t fiqBanked[5][2];
		uint32_t allBanked[2][16];
		uint32_t programCounter;
		uint32_t *registerView[16];
		uint32_t *curSPSRView;
	};
	class MemoryController {
	public:
		MemoryController(IMX233& core) : core(core) {}
		void reset() {}
		uint32_t readWord(uint32_t addr, bool *errorOccurred) {
			if (addr & 3)
				core.dumpAndAbort("readWord unaligned");
			return readWordPhysical(addr, errorOccurred);
		}
		void writeWord(uint32_t addr, uint32_t val, bool *errorOccurred) {
			if (addr & 3)
				core.dumpAndAbort("writeWord unaligned");
			writeWordPhysical(addr, val, errorOccurred);
		}
		uint32_t readWordPhysical(uint32_t addr, bool *errorOccurred) {
			if (addr >= core.systemMemoryBase && addr < core.systemMemoryBase + core.systemMemorySize) {
				return core.systemMemory[(addr - systemMemoryBase) / 4];
			}
			core.dumpAndAbort("readWordPhysical");
		}
		void writeWordPhysical(uint32_t addr, uint32_t val, bool *errorOccurred) {
			if (addr >= core.systemMemoryBase && addr < core.systemMemoryBase + core.systemMemorySize) {
				core.systemMemory[(addr - systemMemoryBase) / 4] = val;
				return;
			}
			core.dumpAndAbort("writeWordPhysical");
		}
	private:
		IMX233& core;
	};
	class SystemControlCoprocessor {
		enum ControlRegBits {
			CONTROL_REG_M = 1 << 0, // MMU enabled
			CONTROL_REG_A = 1 << 1, // strict alignment
			CONTROL_REG_SBZ = 0xfc1a0000,
			CONTROL_REG_SBO = 0x00050072,
		};
	public:
		SystemControlCoprocessor(IMX233& core) : core(core) {}
		void reset() {
			controlReg = CONTROL_REG_SBO;
			domainAccess = 0;
			translationTableBase = 0;
		}
		uint32_t read(unsigned int opcode_1, unsigned int CRn, unsigned int CRm, unsigned int opcode_2) {
			switch (CRn) {
				case 0: // ID codes
					switch (opcode_2) {
						case 0: // Main ID register
							return 0x41039200;
						default:
							core.dumpAndAbort("CP15 unknown opcode_2");
							break;
					}
					break;
				case 1: // control registers
					switch (opcode_2) {
						case 0: // primary control register
							return controlReg;
						default:
							core.dumpAndAbort("CP15 unknown opcode_2");
							break;
					}
					break;
				default:
					core.dumpAndAbort("CP15 unknown register read %d", CRn);
					break;
			}
		}
		void write(unsigned int opcode_1, unsigned int CRn, unsigned int CRm, unsigned int opcode_2, uint32_t data) {
			switch (CRn) {
				case 2:
					translationTableBase = data;
					break;
				case 3:
					domainAccess = data;
					break;
				case 7:
					fprintf(stderr, "TODO: cache management (CRn=7)\n");
					break;
				case 8:
					fprintf(stderr, "TODO: TLB management (CRn=8)\n");
					break;
				default:
					core.dumpAndAbort("CP15 unknown register write %d", CRn);
					break;
			}
		}
	private:
		IMX233& core;
		uint32_t controlReg;
		uint32_t domainAccess;
		uint32_t translationTableBase;
	};
private:
	std::unique_ptr<uint32_t[]> systemMemory;
	uint32_t systemMemorySize = 0;
	RegisterFile registerFile;
	MemoryController memoryController;
	SystemControlCoprocessor systemControlCoprocessor;
	TickState currentTick;
	union {
		struct {
			bool load;
			bool special;
			bool up;
			bool writeback;
			unsigned int Rn;
			uint32_t register_list;
			uint32_t Rn_final;
			uint32_t address;
		} ldm_stm;
		struct {
			bool load;
			bool byte;
			bool writeback;
			unsigned int Rn;
			unsigned int Rd;
			uint32_t Rn_final;
			uint32_t address;
		} ldr_str;
	} pendingOperationState;
};

std::vector<char> readFileToVector(const char *path) {
	std::ifstream stream(path);
	if (!stream) {
		return std::vector<char>();
	} else {
		std::istreambuf_iterator<char> start(stream);
		std::istreambuf_iterator<char> end;
		return std::vector<char>(start, end);
	}
}

void coreMainLoop(IMX233* coreptr) {
	IMX233& core = *coreptr;
	while (true)
		core.tick();
}

int main(int argc, char *argv[]) {

	if (argc != 3) {
		std::cerr << "Usage: " << argv[0] << " <kernel-image> <emulator.dtb>" << std::endl;
		return 1;
	}

	std::vector<char> kernelImage = readFileToVector(argv[1]);
	if (kernelImage.size() == 0) {
		std::cerr << "Error: Failed to read kernel image." << std::endl;
		return 1;
	}

	std::vector<char> deviceBlob = readFileToVector(argv[2]);
	if (deviceBlob.size() == 0) {
		std::cerr << "Error: Failed to read device blob." << std::endl;
		return 1;
	}

	// initialize core
	IMX233 core;
	core.allocateMemory(64 * 1024 * 1024);

	// load kernel image
	uint32_t kernelStart = 0x8000;
	core.loadImage(kernelImage.data(), kernelImage.size(), kernelStart);

	// load device blob
	uint32_t dtbStart = 8 * 1024 * 1024;
	if (kernelStart + kernelImage.size() > dtbStart) {
		std::cerr << "Error: Kernel image overlaps device blob." << std::endl;
		return 1;
	}
	core.loadImage(deviceBlob.data(), deviceBlob.size(), dtbStart);

	// set registers
	core.writeRegister(1, 0xFFFFFFFF);
	core.writeRegister(2, IMX233::systemMemoryBase + dtbStart);
	core.setPC(IMX233::systemMemoryBase + kernelStart);

	// start CPU in background thread
	std::thread coreThread(coreMainLoop, &core);

	// wait forever
	pause();

}
