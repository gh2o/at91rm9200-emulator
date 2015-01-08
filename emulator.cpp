#define COMPILE_SCRIPT /*
	set -- -O2 -ggdb -std=c++11 -pthread -Wall
	g++ "$0" -o "${0%.*}" "$@"
	exit
*/

#include <stdarg.h>
#include <stdint.h>
#include <unistd.h>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <vector>
#include <memory>
#include <thread>
#include <atomic>
#include <chrono>
#include <mutex>
#include <condition_variable>

static inline uint32_t rotateRight(uint32_t val, uint32_t count) {
	return (val >> count) | (val << (32 - count));
}

static inline int leftMostBit(unsigned int val) {
	return (sizeof(unsigned int) * 8 - 1) - __builtin_clz(val);
}

static inline int rightMostBit(unsigned int val) {
	return __builtin_ctz(val);
}

class ARM920T {
public:
	struct MemoryInterface;
private:
	struct TickState;
public:
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
		PENDING_OPERATION_STRH,
		PENDING_OPERATION_LDRH_LDRSH,
		PENDING_OPERATION_LDRSB,
		PENDING_OPERATION_SWP_SWPB,
	};
public:
	ARM920T(MemoryInterface& mi) :
			memoryInterface(mi) {
		reset();
	}
	void reset() {
		memoryInterface.reset(*this);
		registerFile.reset();
		memoryController.reset();
		systemControlCoprocessor.reset();
		currentTick.reset();
		irqAsserted = false;
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
			case PENDING_OPERATION_STRH:
				tickPendingSTRH();
				break;
			case PENDING_OPERATION_LDRH_LDRSH:
				tickPendingLDRHLDRSH();
				break;
			case PENDING_OPERATION_LDRSB:
				tickPendingLDRSB();
				break;
			case PENDING_OPERATION_SWP_SWPB:
				tickPendingSWPSWPB();
				break;
		}
		if (currentTick.tickError != TICK_ERROR_NONE) {
			currentTick.pendingOperation = PENDING_OPERATION_NONE;
			switch (currentTick.tickError) {
				case TICK_ERROR_DATA_ABORT:
					prepareInterrupt(0x10, CPU_MODE_ABT | PSR_BITS_I, currentTick.curPC + 8);
					break;
				default:
					dumpAndAbort("unknown tick error %d", currentTick.tickError);
					break;
			}
		} else if (currentTick.pendingOperation == PENDING_OPERATION_NONE) {
			currentTick.curPC = registerFile.getProgramCounter();
			// process interrupts
			if ((~readCPSR() & PSR_BITS_I) && irqAsserted)
				prepareInterrupt(0x18, CPU_MODE_IRQ | PSR_BITS_I, currentTick.curPC + 4);
		}
	}
	void prepareInterrupt(uint32_t vecAddr, uint32_t cpsrBits, uint32_t lrValue) {
		bool V = systemControlCoprocessor.controlReg &
			SystemControlCoprocessor::CONTROL_REG_V;
		uint32_t curCPSR = readCPSR();
		writeCPSR((curCPSR & ~PSR_BITS_MODE) | cpsrBits);
		writeSPSR(curCPSR);
		writeRegister(14, lrValue);
		setPC(V ? (vecAddr | 0xFFFF0000) : vecAddr);
	}
	void tickExecute() {
		bool errorOccurred = false;
		// fetch instruction
		uint32_t encodedInst = memoryController.readWord(getPC(), errorOccurred);
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
			case 4:
				condPass = readCPSR() & PSR_BITS_N;
				break;
			case 5:
				condPass = !(readCPSR() & PSR_BITS_N);
				break;
			case 8:
				condPass = (readCPSR() & (PSR_BITS_C | PSR_BITS_Z)) == PSR_BITS_C;
				break;
			case 9:
				condPass = (readCPSR() ^ PSR_BITS_C) & (PSR_BITS_C | PSR_BITS_Z);
				break;
			case 10:
				condPass = !!(readCPSR() & PSR_BITS_N) == !!(readCPSR() & PSR_BITS_V);
				break;
			case 11:
				condPass = !!(readCPSR() & PSR_BITS_N) != !!(readCPSR() & PSR_BITS_V);
				break;
			case 12:
				condPass = !(readCPSR() & PSR_BITS_Z) && !!(readCPSR() & PSR_BITS_N) == !!(readCPSR() & PSR_BITS_V);
				break;
			case 13:
				condPass = (readCPSR() & PSR_BITS_Z) || !!(readCPSR() & PSR_BITS_N) != !!(readCPSR() & PSR_BITS_V);
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
				if ((dec3 & 0x09) == 0x09) {
					if (dec3 & 0x06) {
						bool P = encodedInst & (1 << 24);
						bool U = encodedInst & (1 << 23);
						bool I = encodedInst & (1 << 22);
						bool W = encodedInst & (1 << 21);
						bool L = encodedInst & (1 << 20);
						bool S = encodedInst & (1 << 6);
						bool H = encodedInst & (1 << 5);
						if (!P && W)
							dumpAndAbort("bad misc L/S");
						uint32_t offset;
						if (I)
							offset = ((encodedInst >> 4) & 0xF0) | (encodedInst & 0x0F);
						else
							offset = readRegister(Rm);
						inst_misc_LDR_STR(L, S, H, U, P, W, Rd, Rn, offset);
					} else if ((dec2 & 0x18) == 0x08) {
						// SMULL/SMLAL/UMULL/UMLAL
						unsigned int RdHi = Rn;
						unsigned int RdLo = Rd;
						bool signedmult = encodedInst & (1 << 22);
						bool accumulate = encodedInst & (1 << 21);
						bool S = encodedInst & (1 << 20);
						inst_MULL_MLAL(signedmult, accumulate, S, RdLo, RdHi, Rm, Rs);
					} else if ((dec2 & 0x1C) == 0) {
						// MUL/MLA
						unsigned int mulRd = Rn;
						unsigned int mulRn = Rd;
						bool accumulate = encodedInst & (1 << 21);
						bool S = encodedInst & (1 << 20);
						inst_MUL_MLA(accumulate, S, mulRd, Rm, Rs, mulRn);
					} else if ((dec2 & 0x1B) == 0x10) {
						// SWP/SWPB
						bool byte = encodedInst & (1 << 22);
						inst_SWP_SWPB(byte, Rd, Rm, Rn);
					} else {
						dumpAndAbort("unknown decode 0a.[%d].[%d]", dec2, dec3);
					}
				} else if ((dec2 & 0x19) == 0x10) {
					switch (dec3) {
						case 0:
							{
								bool R = encodedInst & (1 << 22);
								if (dec2 & 0x02) {
									uint32_t field_mask = (encodedInst >> 16) & 0x0F;
									uint32_t operand = readRegister(Rm);
									inst_MSR(R, field_mask, operand);
								} else {
									inst_MRS(R, Rd);
								}
							}
							break;
						case 1:
							if (dec2 == 0x12) {
								inst_BX(Rm);
							} else {
								dumpAndAbort("unknown decode 0.[%d].1", dec2);
							}
							break;
						default:
							dumpAndAbort("unknown decode 0b.[%d].[%d]", dec2, dec3);
							break;
					}
				} else {
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
		if (st.byte) {
			unsigned int shift = 8 * (st.address & 3);
			uint32_t aladdr = st.address & ~3;
			if (st.load) {
				// LDRB(T)
				uint32_t data = memoryController.readWord(aladdr, st.usermode, errorOccurred);
				if (errorOccurred) {
					currentTick.tickError = TICK_ERROR_DATA_ABORT;
					return;
				}
				writeRegister(st.Rd, (data >> shift) & 0xFF);
			} else {
				// STRB(T)
				if (st.hasInjectedValue) {
					// actually store it
					memoryController.writeWord(aladdr, st.injectedValue, st.usermode, errorOccurred);
					if (errorOccurred) {
						currentTick.tickError = TICK_ERROR_DATA_ABORT;
						return;
					}
				} else {
					// load and inject
					uint32_t data = memoryController.readWord(aladdr, st.usermode, errorOccurred);
					if (errorOccurred) {
						currentTick.tickError = TICK_ERROR_DATA_ABORT;
						return;
					}
					data &= ~(0xFF << shift);
					data |= (readRegister(st.Rd) & 0xFF) << shift;
					st.hasInjectedValue = true;
					st.injectedValue = data;
					return;
				}
			}
		} else {
			if (st.load) {
				// LDR(T)
				uint32_t data = memoryController.readWord(st.address, st.usermode, errorOccurred);
				if (errorOccurred) {
					currentTick.tickError = TICK_ERROR_DATA_ABORT;
					return;
				}
				writeRegister(st.Rd, data);
			} else {
				// STR(T)
				uint32_t data = readRegister(st.Rd);
				memoryController.writeWord(st.address, data, st.usermode, errorOccurred);
				if (errorOccurred) {
					currentTick.tickError = TICK_ERROR_DATA_ABORT;
					return;
				}
			}
		}
		if (st.writeback)
			writeRegister(st.Rn, st.Rn_final);
		currentTick.pendingOperation = PENDING_OPERATION_NONE;
	}
	void tickPendingLDMSTM() {
		bool errorOccurred = false;
		auto& st = pendingOperationState.ldm_stm;
		if (st.special && !st.restoreCPSR)
			dumpAndAbort("LDM/STM special not implemented");
		unsigned int Rd;
		if (st.up)
			Rd = rightMostBit(st.register_list);
		else
			Rd = leftMostBit(st.register_list);
		if (st.load) {
			uint32_t value = memoryController.readWord(st.address, errorOccurred);
			if (errorOccurred) {
				currentTick.tickError = TICK_ERROR_DATA_ABORT;
				return;
			}
			writeRegister(Rd, value);
		} else {
			uint32_t value = readRegister(Rd);
			memoryController.writeWord(st.address, value, errorOccurred);
			if (errorOccurred) {
				currentTick.tickError = TICK_ERROR_DATA_ABORT;
				return;
			}
		}
		if (st.up) {
			st.address += 4;
			st.Rn_final += 4;
		} else {
			st.address -= 4;
			st.Rn_final -= 4;
		}
		st.register_list &= ~(1 << Rd);
		if (!st.register_list) {
			if (st.writeback)
				writeRegister(st.Rn, st.Rn_final);
			if (st.restoreCPSR)
				writeCPSR(readSPSR());
			currentTick.pendingOperation = PENDING_OPERATION_NONE;
		}
	}
	void tickPendingSTRH() {
		bool errorOccurred = false;
		auto& st = pendingOperationState.misc_ldr_str;
		unsigned int shift = 8 * (st.address & 2);
		uint32_t aladdr = st.address & ~2;
		if (st.hasInjectedValue) {
			// actually store it
			memoryController.writeWord(aladdr, st.injectedValue, errorOccurred);
			if (errorOccurred) {
				currentTick.tickError = TICK_ERROR_DATA_ABORT;
				return;
			}
		} else {
			// load and inject
			uint32_t data = memoryController.readWord(aladdr, errorOccurred);
			if (errorOccurred) {
				currentTick.tickError = TICK_ERROR_DATA_ABORT;
				return;
			}
			data &= ~(0xFFFF << shift);
			data |= (readRegister(st.Rd) & 0xFFFF) << shift;
			st.hasInjectedValue = true;
			st.injectedValue = data;
			return;
		}
		if (st.writeback)
			writeRegister(st.Rn, st.Rn_final);
		currentTick.pendingOperation = PENDING_OPERATION_NONE;
	}
	void tickPendingLDRHLDRSH() {
		bool errorOccurred = false;
		auto& st = pendingOperationState.misc_ldr_str;
		unsigned int shift = 8 * (st.address & 2);
		uint32_t aladdr = st.address & ~2;
		uint32_t data = memoryController.readWord(aladdr, errorOccurred);
		if (errorOccurred) {
			currentTick.tickError = TICK_ERROR_DATA_ABORT;
			return;
		}
		data = (data >> shift) & 0xFFFF;
		if (st.signextend && (data & 0x8000))
			data |= 0xFFFF0000;
		writeRegister(st.Rd, data);
		if (st.writeback)
			writeRegister(st.Rn, st.Rn_final);
		currentTick.pendingOperation = PENDING_OPERATION_NONE;
	}
	void tickPendingLDRSB() {
		bool errorOccurred = false;
		auto& st = pendingOperationState.misc_ldr_str;
		unsigned int shift = 8 * (st.address & 3);
		uint32_t aladdr = st.address & ~3;
		uint32_t data = memoryController.readWord(aladdr, errorOccurred);
		if (errorOccurred) {
			currentTick.tickError = TICK_ERROR_DATA_ABORT;
			return;
		}
		data = (data >> shift) & 0xFF;
		if (data & 0x80)
			data |= ~0xFF;
		writeRegister(st.Rd, data);
		if (st.writeback)
			writeRegister(st.Rn, st.Rn_final);
		currentTick.pendingOperation = PENDING_OPERATION_NONE;
	}
	void tickPendingSWPSWPB() {
		bool errorOccurred = false;
		auto& st = pendingOperationState.swp_swpb;
		if (st.byte)
			dumpAndAbort("SWPB not implemented");
		if (!st.hasTemp) {
			st.hasTemp = true;
			st.temp = memoryController.readWord(st.address, errorOccurred);
			if (errorOccurred) {
				currentTick.tickError = TICK_ERROR_DATA_ABORT;
				return;
			}
			return;
		} else {
			uint32_t Rm_value = readRegister(st.Rm);
			memoryController.writeWord(st.address, Rm_value, errorOccurred);
			if (errorOccurred) {
				currentTick.tickError = TICK_ERROR_DATA_ABORT;
				return;
			}
			writeRegister(st.Rd, st.temp);
		}
		currentTick.pendingOperation = PENDING_OPERATION_NONE;
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
			case 1:
			case 9:
				alu_out = Rn_value ^ shifter_operand;
				break;
			case 2:
			case 10:
				alu_out = Rn_value - shifter_operand;
				break;
			case 3:
				alu_out = shifter_operand - Rn_value;
				break;
			case 4:
			case 11:
				alu_out = Rn_value + shifter_operand;
				break;
			case 5:
				alu_out = Rn_value + shifter_operand + !!(readCPSR() & PSR_BITS_C);
				break;
			case 6:
				alu_out = Rn_value - shifter_operand - !(readCPSR() & PSR_BITS_C);
				break;
			case 7:
				alu_out = shifter_operand - Rn_value - !(readCPSR() & PSR_BITS_C);
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
			uint32_t newCPSR;
			if (Rd == 15) {
				newCPSR = readSPSR();
			} else {
				newCPSR =
					(readCPSR() & ~(PSR_BITS_N | PSR_BITS_Z | PSR_BITS_C)) |
					(alu_out & PSR_BITS_N) |
					(alu_out == 0 ? PSR_BITS_Z : 0) |
					(shifter_carry_out ? PSR_BITS_C : 0);
				bool a, b, r = alu_out & (1 << 31);
				switch (opcode) {
					case 0:
					case 1:
					case 8:
					case 9:
					case 12:
					case 13:
					case 14:
						break;
					case 4:
					case 5:
					case 11:
						a = Rn_value & (1 << 31);
						b = shifter_operand & (1 << 31);
						newCPSR =
							(newCPSR & ~(PSR_BITS_C | PSR_BITS_V)) |
							((a & b) | (a & !r) | (b & !r) ? PSR_BITS_C : 0) |
							((!a & !b & r) | (a & b & !r) ? PSR_BITS_V : 0);
						break;
					case 2:
					case 3:
					case 6:
					case 7:
					case 10:
						if (opcode == 3 || opcode == 7) {
							a = shifter_operand & (1 << 31);
							b = Rn_value & (1 << 31);
						} else {
							a = Rn_value & (1 << 31);
							b = shifter_operand & (1 << 31);
						}
						newCPSR =
							(newCPSR & ~(PSR_BITS_C | PSR_BITS_V)) |
							((a & !b) | (a & !r) | (!b & !r) ? PSR_BITS_C : 0) |
							((!a & b & r) | (a & !b & !r) ? PSR_BITS_V : 0);
						break;
					default:
						dumpAndAbort("data opcode %u S unimplemented", opcode);
						break;
				}
			}
			writeCPSR(newCPSR);
		}
		if ((opcode & 0x0C) != 0x08)
			writeRegister(Rd, alu_out);
	}
	void inst_MUL_MLA(bool accumulate, bool S,
			unsigned int Rd, unsigned int Rm,
			unsigned int Rs, unsigned int Rn) {
		uint32_t Rm_value = readRegister(Rm);
		uint32_t Rs_value = readRegister(Rs);
		uint32_t Rn_value = readRegister(Rn);
		uint32_t res = Rm_value * Rs_value;
		if (accumulate)
			res += Rn_value;
		writeRegister(Rd, res);
		if (S) {
			uint32_t newCPSR =
				(readCPSR() & ~(PSR_BITS_N | PSR_BITS_Z)) |
				(res & (1 << 31) ? PSR_BITS_N : 0) |
				(res == 0 ? PSR_BITS_Z : 0);
			writeCPSR(newCPSR);
		}
	}
	void inst_MULL_MLAL(bool signedmult, bool accumulate, bool S,
			unsigned int RdLo, unsigned int RdHi,
			unsigned int Rm, unsigned int Rs) {
		uint32_t RdLo_value = readRegister(RdLo);
		uint32_t RdHi_value = readRegister(RdHi);
		uint32_t Rm_value = readRegister(Rm);
		uint32_t Rs_value = readRegister(Rs);
		uint64_t adj = 0;
		if (signedmult) {
			if (Rm_value & (1 << 31))
				adj += Rs_value;
			if (Rs_value & (1 << 31))
				adj += Rm_value;
		}
		uint64_t res = (uint64_t)Rm_value * (uint64_t)Rs_value - (adj << 32);
		if (accumulate)
			res += (uint64_t)RdHi_value << 32 | (uint64_t)RdLo_value;
		writeRegister(RdLo, res);
		writeRegister(RdHi, res >> 32);
		if (S) {
			uint32_t newCPSR =
				(readCPSR() & ~(PSR_BITS_N | PSR_BITS_Z)) |
				(res & (1ULL << 63) ? PSR_BITS_N : 0) |
				(res == 0 ? PSR_BITS_Z : 0);
			writeCPSR(newCPSR);
		}
	}
	void inst_B_BL(bool L, uint32_t signed_immed_24) {
		if (L)
			writeRegister(14, getPC() + 4);
		if (signed_immed_24 & (1 << 23))
			signed_immed_24 |= 0xFF << 24;
		writeRegister(15, readRegister(15) + (signed_immed_24 << 2));
	}
	void inst_BX(unsigned int Rm) {
		uint32_t value = readRegister(Rm);
		if (value & 3)
			dumpAndAbort("BX to thumb code");
		writeRegister(15, value);
	}
	void inst_LDR_STR(
			bool L, bool B, bool P, bool U, bool W,
			unsigned int Rd, unsigned int Rn, uint32_t offset_12) {
		currentTick.pendingOperation = PENDING_OPERATION_LDR_STR;
		auto& st = pendingOperationState.ldr_str;
		st.load = L;
		st.byte = B;
		st.usermode = !P && W;
		st.writeback = !P || W;
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
		st.hasInjectedValue = false;
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
		st.restoreCPSR = L && S && (register_list & (1 << 15));
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
	void inst_misc_LDR_STR(
			bool L, bool S, bool H, bool U, bool P, bool W,
			unsigned int Rd, unsigned int Rn, uint32_t offset) {
		auto& st = pendingOperationState.misc_ldr_str;
		st.signextend = S;
		st.writeback = (P == W);
		st.Rn = Rn;
		st.Rd = Rd;
		uint32_t Rn_value = readRegister(Rn);
		uint32_t offsettedAddress = U ?
			Rn_value + offset :
			Rn_value - offset;
		st.Rn_final = offsettedAddress;
		if (P)
			st.address = offsettedAddress;
		else
			st.address = Rn_value;
		st.hasInjectedValue = false;
		if (L && H) {
			currentTick.pendingOperation = PENDING_OPERATION_LDRH_LDRSH;
		} else if (!L && S) {
			dumpAndAbort("LDRD/STRD");
		} else if (L) {
			currentTick.pendingOperation = PENDING_OPERATION_LDRSB;
		} else {
			currentTick.pendingOperation = PENDING_OPERATION_STRH;
		}
	}
	void inst_SWP_SWPB(bool byte,
			unsigned int Rd, unsigned int Rm, unsigned int Rn) {
		currentTick.pendingOperation = PENDING_OPERATION_SWP_SWPB;
		auto& st = pendingOperationState.swp_swpb;
		st.byte = byte;
		st.Rd = Rd;
		st.Rm = Rm;
		st.address = readRegister(Rn);
		st.hasTemp = false;
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
		constexpr uint32_t USER_BITS = 0xF0000000;
		constexpr uint32_t PRIV_BITS = 0x000000DF;
		uint32_t mask =
			((field_mask & (1 << 0)) ? 0x000000FF : 0) |
			((field_mask & (1 << 1)) ? 0x0000FF00 : 0) |
			((field_mask & (1 << 2)) ? 0x00FF0000 : 0) |
			((field_mask & (1 << 3)) ? 0xFF000000 : 0);
		if (operand & ~(USER_BITS | PRIV_BITS))
			dumpAndAbort("MSR: attempted to set reserved bits (%08x)", operand);
		if (!R) {
			if (isPrivileged()) {
				mask &= USER_BITS | PRIV_BITS;
			} else {
				mask &= USER_BITS;
			}
			writeCPSR((readCPSR() & ~mask) | (operand & mask));
		} else {
			mask &= USER_BITS | PRIV_BITS;
			writeSPSR((readSPSR() & ~mask) | (operand & mask));
		}
	}
	void inst_MRS(bool R, unsigned int Rd) {
		if (!R)
			writeRegister(Rd, readCPSR());
		else
			writeRegister(Rd, readSPSR());
	}
	void dumpState() {
		// dump PC
		uint32_t pc = getPC();
		bool err = false;
		uint32_t inst = memoryController.readWord(pc, err);
		if (!err)
			fprintf(stderr, "pc = %08x (%08x)\n", pc, inst);
		else
			fprintf(stderr, "pc = %08x\n", pc);
		// dump info
		for (int i = 0; i < 15; i++)
			fprintf(stderr, "r%d = %08x\n", i, readRegister(i));
		fprintf(stderr, "cpsr = %08x\n", readCPSR());
		if (readCPSR() & 0xF)
			fprintf(stderr, "spsr = %08x\n", readSPSR());
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
		// dump state
		dumpState();
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
				if (shift_imm == 0) {
					// default
				} else if (shift_imm < 32) {
					*shifter_carry_out = *shifter_operand & (1 << (32 - shift_imm));
					*shifter_operand <<= shift_imm;
				} else if (shift_imm == 32) {
					*shifter_carry_out = *shifter_operand & (1 << 0);
					*shifter_operand = 0;
				} else if (shift_imm > 32) {
					*shifter_carry_out = false;
					*shifter_operand = 0;
				}
				break;
			case 1: // LSR
				if (!is_reg && shift_imm == 0)
					shift_imm = 32;
				if (shift_imm == 0) {
					// default
				} else if (shift_imm < 32) {
					*shifter_carry_out = *shifter_operand & (1 << (shift_imm - 1));
					*shifter_operand >>= shift_imm;
				} else if (shift_imm == 32) {
					*shifter_carry_out = *shifter_operand & (1 << 31);
					*shifter_operand = 0;
				} else if (shift_imm > 32) {
					*shifter_carry_out = false;
					*shifter_operand = 0;
				}
				break;
			case 2: // ASR
				if (!is_reg && shift_imm == 0)
					shift_imm = 32;
				if (shift_imm == 0) {
					// default
				} else if (shift_imm < 32) {
					bool sign = *shifter_operand & (1 << 31);
					*shifter_carry_out = *shifter_operand & (1 << (shift_imm - 1));
					*shifter_operand >>= shift_imm;
					if (sign)
						*shifter_operand |= 0xFFFFFFFF << (32 - shift_imm);
				} else {
					bool sign = *shifter_operand & (1 << 31);
					*shifter_carry_out = sign;
					*shifter_operand = sign ? 0xFFFFFFFF : 0;
				}
				break;
			case 3:
				if (!is_reg && shift_imm == 0) { // RRX
					bool c_flag = *shifter_carry_out;
					*shifter_carry_out = *shifter_operand & (1 << 0);
					*shifter_operand = (c_flag ? (1 << 31) : 0) | (*shifter_operand >> 1);
				} else { // ROR
					uint32_t rotate_amt = shift_imm & 0x1F;
					if (shift_imm == 0) {
						// default
					} else if (rotate_amt == 0) {
						*shifter_carry_out = *shifter_operand & (1 << 31);
						// shifter_operand default
					} else {
						*shifter_carry_out = *shifter_operand & (1 << (rotate_amt - 1));
						*shifter_operand = rotateRight(*shifter_operand, rotate_amt);
					}
				}
				break;
		}
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
	bool getIRQ() { return irqAsserted; }
	void setIRQ(bool state) { irqAsserted = state; }
public:
	struct MemoryInterface {
		virtual void reset(ARM920T& core) {}
		virtual uint32_t readWordPhysical(uint32_t addr, bool& errorOccurred) = 0;
		virtual void writeWordPhysical(uint32_t addr, uint32_t val, bool& errorOccurred) = 0;
	};
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
		RegisterFile(ARM920T& core) : core(core) {
			registerView[15] = &programCounter;
			for (int i = 0; i <= 7; i++)
				registerView[i] = &(nonBanked[i]);
		}
		void reset() {
			writeCPSR(CPU_MODE_SVC | PSR_BITS_F | PSR_BITS_I);
			std::fill(std::begin(storedSPSR), std::end(storedSPSR), 0);
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
		ARM920T& core;
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
		enum PermCheck {
			PERM_CHECK_NONE,
			PERM_CHECK_READ,
			PERM_CHECK_WRITE
		};
	public:
		MemoryController(ARM920T& core) : core(core) {}
		void reset() {}
		uint32_t readWord(uint32_t addr, bool& errorOccurred) {
			return readWord(addr, false, errorOccurred);
		}
		void writeWord(uint32_t addr, uint32_t val, bool& errorOccurred) {
			writeWord(addr, val, false, errorOccurred);
		}
		uint32_t readWord(uint32_t addr, bool usermode, bool& errorOccurred) {
			if (addr & 3)
				core.dumpAndAbort("readWord unaligned");
			addr = translateAddress(addr, PERM_CHECK_READ, usermode, errorOccurred);
			if (errorOccurred)
				return 0;
			return readWordPhysical(addr, errorOccurred);
		}
		void writeWord(uint32_t addr, uint32_t val, bool usermode, bool& errorOccurred) {
			if (addr & 3)
				core.dumpAndAbort("writeWord unaligned");
			addr = translateAddress(addr, PERM_CHECK_WRITE, usermode, errorOccurred);
			if (errorOccurred)
				return;
			writeWordPhysical(addr, val, errorOccurred);
		}
		uint32_t readWordPhysical(uint32_t addr, bool& errorOccurred) {
			return core.memoryInterface.readWordPhysical(addr, errorOccurred);
		}
		void writeWordPhysical(uint32_t addr, uint32_t val, bool& errorOccurred) {
			core.memoryInterface.writeWordPhysical(addr, val, errorOccurred);
		}
		uint32_t translateAddress(uint32_t addr, PermCheck pcheck, bool usermode, bool& errorOccurred) {
			auto& scc = core.systemControlCoprocessor;
			typedef SystemControlCoprocessor SCC;
			if (!(scc.controlReg & SCC::CONTROL_REG_M))
				return addr;
			// should be set
			uint32_t newaddr = 0;
			uint32_t apbits = 0;
			// first level walk
			uint32_t desc1addr =
				(scc.translationTableBase & 0xFFFFC000) |
				((addr >> 18) & 0x3FFC);
			uint32_t desc1 = readWordPhysical(desc1addr, errorOccurred);
			if (errorOccurred) {
				recordFault(addr, 0xC, 0);
				return 0;
			}
			unsigned int desc1type = desc1 & 0x03;
			unsigned int domain = (desc1 >> 5) & 0xF;
			switch (desc1type) {
				case 0: // fault
					recordFault(addr, 0x5, domain);
					errorOccurred = true;
					return 0;
				case 2: // section
					newaddr = (desc1 & 0xFFF00000) | (addr & 0x000FFFFF);
					apbits = (desc1 >> 10) & 3;
					break;
				default:
					newaddr = translateLevel2(addr, desc1, domain, &apbits, errorOccurred);
					if (errorOccurred)
						return 0;
					break;
			}
			// check domain
			unsigned int domacc = (scc.domainAccess >> (domain * 2)) & 0x03;
			if ((domacc & 0x01) == 0) { // domain fault
				recordFault(addr, desc1type == 2 ? 0x9 : 0xB, domain);
				errorOccurred = true;
				return 0;
			}
			// check permissions if required
			if (domacc == 1) {
				bool AP1 = apbits & 2;
				bool AP0 = apbits & 1;
				bool S = scc.controlReg & SCC::CONTROL_REG_S;
				bool R = scc.controlReg & SCC::CONTROL_REG_R;
				bool isPrivileged = core.isPrivileged() && !usermode;
				bool accessAllowed;
				switch (pcheck) {
					case PERM_CHECK_NONE:
						accessAllowed = true;
						break;
					case PERM_CHECK_READ:
						if (isPrivileged)
							accessAllowed = S | R | AP1 | AP0;
						else
							accessAllowed = AP1 | (R & !AP0);
						break;
					case PERM_CHECK_WRITE:
						if (isPrivileged)
							accessAllowed = AP1 | AP0;
						else
							accessAllowed = AP1 & AP0;
						break;
					default:
						core.dumpAndAbort("unknown pcheck!");
				}
				if (!accessAllowed)
					core.dumpAndAbort("denied access to addr %08x -> %08x", addr, newaddr);
			}
			// done!
			return newaddr;
		}
		uint32_t translateLevel2(uint32_t addr, uint32_t desc1, unsigned int domain,
				uint32_t *apbitsPtr, bool& errorOccurred) {
			// should be set
			uint32_t newaddr;
			unsigned int subpage;
			// second level walk
			uint32_t desc2addr;
			if ((desc1 & 0x03) == 0x03) {
				// fine page table
				core.dumpAndAbort("fine page table");
			} else {
				desc2addr =
					(desc1 & 0xFFFFFC00) |
					((addr >> 10) & 0x03FC);
			}
			uint32_t desc2 = readWordPhysical(desc2addr, errorOccurred);
			if (errorOccurred) {
				recordFault(addr, 0xE, domain);
				return 0;
			}
			unsigned int desc2type = desc2 & 0x03;
			switch (desc2type) {
				case 0: // fault
					recordFault(addr, 0x7, domain);
					errorOccurred = true;
					return 0;
				case 2: // small pages
					newaddr = (desc2 & 0xFFFFF000) | (addr & 0x0FFF);
					subpage = (addr >> 10) & 3;
					break;
				default:
					core.dumpAndAbort("unsupported desc2 type %u", desc2type);
					break;
			}
			// done!
			*apbitsPtr = (desc2 >> (4 + 2 * subpage)) & 3;
			return newaddr;
		}
		void recordFault(uint32_t addr, uint32_t status, uint32_t domain) {
			auto& scc = core.systemControlCoprocessor;
			scc.faultStatus = (domain << 4) | status;
			scc.faultAddress = addr;
		}
	private:
		ARM920T& core;
	};
	class SystemControlCoprocessor {
		enum ControlRegBits {
			CONTROL_REG_M = 1 << 0, // MMU enabled
			CONTROL_REG_A = 1 << 1, // strict alignment
			CONTROL_REG_S = 1 << 8,
			CONTROL_REG_R = 1 << 9,
			CONTROL_REG_V = 1 << 13, // high exception vectors
			CONTROL_REG_SBZ = 0xfc1a0000,
			CONTROL_REG_SBO = 0x00050072,
			CONTROL_REG_SUPPORTED = 0x2303
		};
	public:
		SystemControlCoprocessor(ARM920T& core) : core(core) {}
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
							return 0x41129201;
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
				case 2:
					return translationTableBase;
				case 3:
					return domainAccess;
				case 5:
					return faultStatus;
				case 6:
					return faultAddress;
				default:
					core.dumpAndAbort("CP15 unknown register read %d", CRn);
					break;
			}
		}
		void write(unsigned int opcode_1, unsigned int CRn, unsigned int CRm, unsigned int opcode_2, uint32_t data) {
			switch (CRn) {
				case 1:
					switch (opcode_2) {
						case 0: // primary control register
							data |= CONTROL_REG_SBO;
							data &= ~CONTROL_REG_SBZ;
							controlReg = data;
							if (data & ~(CONTROL_REG_SBZ | CONTROL_REG_SBO | CONTROL_REG_SUPPORTED))
								core.dumpAndAbort("unsupported bits set in control register");
							break;
						default:
							core.dumpAndAbort("CP15 unknown opcode_2");
							break;
					}
					break;
				case 2:
					translationTableBase = data;
					break;
				case 3:
					domainAccess = data;
					break;
				case 7:
					// TODO: cache management
					break;
				case 8:
					// TODO: TLB management
					break;
				default:
					core.dumpAndAbort("CP15 unknown register write %d", CRn);
					break;
			}
		}
	private:
		ARM920T& core;
		uint32_t controlReg;
		uint32_t domainAccess;
		uint32_t translationTableBase;
		uint32_t faultStatus;
		uint32_t faultAddress;
		friend class ARM920T;
		friend class MemoryController;
	};
private:
	MemoryInterface& memoryInterface;
	RegisterFile registerFile{*this};
	MemoryController memoryController{*this};
	SystemControlCoprocessor systemControlCoprocessor{*this};
	TickState currentTick;
	std::atomic<bool> irqAsserted;
	union {
		struct {
			bool load;
			bool special;
			bool up;
			bool writeback;
			bool restoreCPSR;
			unsigned int Rn;
			uint32_t register_list;
			uint32_t Rn_final;
			uint32_t address;
		} ldm_stm;
		struct {
			bool load;
			bool byte;
			bool usermode;
			bool writeback;
			unsigned int Rn;
			unsigned int Rd;
			uint32_t Rn_final;
			uint32_t address;
			bool hasInjectedValue;
			uint32_t injectedValue;
		} ldr_str;
		struct {
			bool signextend;
			bool writeback;
			unsigned int Rn;
			unsigned int Rd;
			uint32_t Rn_final;
			uint32_t address;
			bool hasInjectedValue;
			uint32_t injectedValue;
		} misc_ldr_str;
		struct {
			bool byte;
			unsigned int Rd;
			unsigned int Rm;
			uint32_t address;
			bool hasTemp;
			uint32_t temp;
		} swp_swpb;
	} pendingOperationState;
};

class AT91RM9200Interface : public ARM920T::MemoryInterface {
	class Peripheral;
public:
	struct MMCCard;
	static constexpr uint32_t systemMemoryBase = 0x20000000;
public:
	AT91RM9200Interface(MMCCard& mmcCard) {
		systemPeripherals[0x0] = &interruptController;
		systemPeripherals[0x1] = &interruptController;
		systemPeripherals[0x2] = &debugUnit;
		systemPeripherals[0x3] = &debugUnit;
		systemPeripherals[0xC] = &powerManager;
		systemPeripherals[0xD] = &systemTimer;
		userPeripherals[0xD] = &mmcInterface;
		mmcInterface.setCard(mmcCard);
	}
	void reset(ARM920T& core) {
		corePtr = &core;
		for (int i = 0; i < 16; i++)
			if (systemPeripherals[i])
				systemPeripherals[i]->reset();
		for (int i = 0; i < 32; i++)
			if (userPeripherals[i])
				userPeripherals[i]->reset();
		systemInterrupt.reset();
	}
	void allocateSystemMemory(uint32_t size) {
		systemMemory.reset(new uint32_t[size / 4 + 1]);
		systemMemorySize = size;
	}
	void loadImageIntoMemory(const void *base, uint32_t size, uint32_t location) {
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
	uint32_t readWordPhysical(uint32_t addr, bool& errorOccurred) {
		uint32_t *location;
		Peripheral *peripheral;
		getAddressTarget(addr, &location, &peripheral);
		if (location) {
			return *location;
		} else if (peripheral) {
			return peripheral->readFromAddress(addr, errorOccurred);
		} else {
			fprintf(stderr, "TE read from unknown physaddr %08x\n", addr);
			errorOccurred = true;
			return 0;
		}
	}
	void writeWordPhysical(uint32_t addr, uint32_t val, bool& errorOccurred) {
		uint32_t *location;
		Peripheral *peripheral;
		getAddressTarget(addr, &location, &peripheral);
		if (location) {
			*location = val;
		} else if (peripheral) {
			peripheral->writeToAddress(addr, val, errorOccurred);
		} else {
			fprintf(stderr, "TE write from unknown physaddr %08x value %08x\n", addr, val);
			errorOccurred = true;
			return;
		}
	}
	void getAddressTarget(uint32_t addr, uint32_t **location, Peripheral **peripheral) {
		*location = nullptr;
		*peripheral = nullptr;
		if (((addr ^ systemMemoryBase) & (0xF << 28)) == 0) {
			// memory access
			uint32_t byteOffset = addr - systemMemoryBase;
			if (byteOffset < systemMemorySize)
				*location = &systemMemory[byteOffset / 4];
			else
				corePtr->dumpAndAbort("out-of-bounds memory access: %08x", addr);
		} else if ((addr & 0xFFFFF000) == 0xFFFFF000) {
			// system peripherals
			uint32_t periphId = (addr >> 8) & 0x0F;
			Peripheral *posPeriph = systemPeripherals[periphId];
			if (posPeriph)
				*peripheral = posPeriph;
			else
				corePtr->dumpAndAbort("unknown system peripheral: %08x", periphId);
		} else if ((addr & 0xFFF80000) == 0xFFF80000){
			// user peripherals
			uint32_t periphId = (addr >> 14) & 0x1F;
			Peripheral *posPeriph = userPeripherals[periphId];
			if (posPeriph)
				*peripheral = posPeriph;
			else
				corePtr->dumpAndAbort("unknown user peripheral: %08x", periphId);
		} else {
			corePtr->dumpAndAbort("could not interpret address: %08x", addr);
		}
	}
	struct MMCCard {
		virtual void doTransaction(unsigned int cmd, unsigned int arg) = 0;
	};
private:
	class SystemInterrupt {
	public:
		// IRQ line 1
		enum InterruptSource {
			INTERRUPT_SOURCE_SYSTEM_CLOCK,
		};
		SystemInterrupt(AT91RM9200Interface& intf) : intf(intf) {}
		void reset() {
			assertedSources = 0;
		}
		void setInterruptState(InterruptSource src, bool val) {
			std::lock_guard<std::recursive_mutex> guard(intf.irqMutex);
			if (val)
				assertedSources |= 1 << src;
			else
				assertedSources &= ~(1 << src);
			intf.interruptController.setInterruptState(1, assertedSources);
		}
	private:
		AT91RM9200Interface& intf;
		uint32_t assertedSources;
	};
	class Peripheral {
	public:
		Peripheral(AT91RM9200Interface& intf, uint32_t baseaddr)
			: intf(intf), baseaddr(baseaddr) {}
		uint32_t readFromAddress(uint32_t addr, bool& errorOccurred) {
			return readRegister(addr - baseaddr, errorOccurred);
		}
		void writeToAddress(uint32_t addr, uint32_t val, bool& errorOccurred) {
			return writeRegister(addr - baseaddr, val, errorOccurred);
		}
		virtual void reset() {}
	protected:
		virtual uint32_t readRegister(uint32_t addr, bool& errorOccurred) { return 0; };
		virtual void writeRegister(uint32_t addr, uint32_t val, bool& errorOccurred) {};
		ARM920T& core() { return *(intf.corePtr); }
		AT91RM9200Interface& intf;
		uint32_t baseaddr;
	};
	class DBGU : public Peripheral {
	public:
		using Peripheral::Peripheral;
		uint32_t readRegister(uint32_t addr, bool& errorOccurred) override {
			switch (addr) {
				case 0x14: // status register
					return 0x0202;
				case 0x1C: // transmit register
					return 0;
				case 0x40: // ID register
					return 0x09290781;
				case 0x44: // EXID register
					return 0;
				default:
					core().dumpAndAbort("DBGU read %02x", addr);
					break;
			}
		}
		void writeRegister(uint32_t addr, uint32_t val, bool& errorOccurred) override {
			switch (addr) {
				case 0x1C: // transmit register
					fputc(val, stdout);
					break;
				default:
					core().dumpAndAbort("DBGU write %02x", addr);
					break;
			}
		}
	};
	class AIC : public Peripheral {
	public:
		using Peripheral::Peripheral;
		void reset() override {
			enabledInterrupts = 0;
			rawInterrupts = 0;
			edgeMask = 0;
			edgeStatus = 0;
			levelMask = 0;
			std::fill(std::begin(levelNums), std::end(levelNums), 0);
			std::fill(std::begin(sourceModes), std::end(sourceModes), 0);
			std::fill(std::begin(sourceVectors), std::end(sourceVectors), 0);
			std::fill(std::begin(priorityMasks), std::end(priorityMasks), 0);
			priorityMasks[0] = -1u;
		}
		uint32_t readRegister(uint32_t addr, bool& errorOccurred) override {
			if ((addr & ~0xFF) == 0x00) {
				uint32_t irq = (addr & 0x7F) / 4;
				if (addr & 0x80)
					return sourceVectors[irq];
				else
					return sourceModes[irq];
			} else {
				switch (addr) {
					case 0x100:
						{
							uint32_t efInts = effectiveInterrupts();
							for (int lvl = 7; lvl >= 0; lvl--) {
								if (levelMask & (1 << lvl)) {
									break;
								}
								uint32_t prioEfInts = efInts & priorityMasks[lvl];
								if (prioEfInts) {
									uint32_t irqNum = rightMostBit(prioEfInts);
									levelMask |= 1 << lvl;
									levelNums[lvl] = irqNum;
									edgeStatus &= 1 << irqNum;
									updateOutput();
									break;
								}
							}
						}
						if (levelMask)
							return sourceVectors[levelNums[leftMostBit(levelMask)]];
						else
							return spuriousVector;
						break;
					case 0x108:
						if (levelMask)
							return levelNums[leftMostBit(levelMask)];
						else
							return 0;
						break;
					default:
						core().dumpAndAbort("AIC read %02x", addr);
						break;
				}
			}
		}
		void writeRegister(uint32_t addr, uint32_t val, bool& errorOccurred) override {
			if ((addr & ~0xFF) == 0x00) {
				uint32_t irq = (addr & 0x7F) / 4;
				if (addr & 0x80) {
					sourceVectors[irq] = val;
				} else {
					priorityMasks[sourceModes[irq] & 0x7] &= ~(1 << irq);
					sourceModes[irq] = val & 0x67;
					priorityMasks[sourceModes[irq] & 0x7] |= 1 << irq;
					if (val & (1 << 5))
						edgeMask |= 1 << irq;
					else
						edgeMask &= ~(1 << irq);
					updateOutput();
				}
			} else {
				switch (addr) {
					case 0x120:
						enabledInterrupts |= val;
						updateOutput();
						break;
					case 0x124:
						enabledInterrupts &= ~val;
						updateOutput();
						break;
					case 0x128:
						edgeStatus &= ~val;
						updateOutput();
						break;
					case 0x12C:
						edgeStatus |= val;
						updateOutput();
						break;
					case 0x130:
						if (levelMask)
							levelMask &= ~(1 << leftMostBit(levelMask));
						break;
					case 0x134:
						spuriousVector = val;
						break;
					case 0x138:
						if (val)
							core().dumpAndAbort("AIC has no requested debug features");
						break;
					default:
						core().dumpAndAbort("AIC write %02x", addr);
						break;
				}
			}
		}
		void setInterruptState(unsigned int irq, bool state) {
			std::lock_guard<std::recursive_mutex> guard(intf.irqMutex);
			bool oldstate = rawInterrupts & (1 << irq);
			if (state)
				rawInterrupts |= 1 << irq;
			else
				rawInterrupts &= ~(1 << irq);
			if (!oldstate && state)
				edgeStatus |= 1 << irq;
			updateOutput();
		}
	private:
		uint32_t pendingInterrupts() const {
			return (edgeStatus & edgeMask) | (rawInterrupts & ~edgeMask);
		}
		uint32_t effectiveInterrupts() const {
			return enabledInterrupts & pendingInterrupts();
		}
		void updateOutput() {
			std::lock_guard<std::recursive_mutex> guard(intf.irqMutex);
			core().setIRQ(effectiveInterrupts());
		}
	private:
		uint32_t enabledInterrupts;
		uint32_t rawInterrupts;
		uint32_t edgeMask;
		uint32_t edgeStatus;
		uint32_t levelMask;
		uint32_t levelNums[8];
		uint32_t sourceModes[32];
		uint32_t sourceVectors[32];
		uint32_t priorityMasks[8];
		uint32_t spuriousVector;
	};
	class ST : public Peripheral {
		typedef std::chrono::steady_clock time_clock;
		typedef std::chrono::duration<int64_t, std::ratio<1, 32768>> slow_ticks;
		typedef std::chrono::time_point<time_clock, slow_ticks> slow_point;
		enum {
			ST_IRQ_PITS = 1 << 0,
			ST_IRQ_ALMS = 1 << 3,
			ST_IRQ_ALL = ST_IRQ_PITS | ST_IRQ_ALMS,
		};
	public:
		using Peripheral::Peripheral;
		void reset() override {
			std::unique_lock<std::mutex> lock(timerThreadMutex);
			enabledInterrupts = 0;
			interruptStatus = 0;
			periodDuration = slow_ticks(65536);
			alarmValue = 1048576;
			realTimeDivider = 0x8000;
			realTimeCounter = 0;
			realTimeLastUpdated = slowPointNow();
			periodIntervalMark = slowPointNow();
			alarmMatchMark = slowPointNow();
			emitInterruptState();
			if (!timerThread.joinable())
				timerThread = std::thread(&ST::timerLoop, this);
		}
		uint32_t readRegister(uint32_t addr, bool& errorOccurred) override {
			uint32_t result;
			switch (addr) {
				case 0x10:
					{
						std::unique_lock<std::mutex> lock(timerThreadMutex);
						result = interruptStatus;
						interruptStatus = 0;
						emitInterruptState();
						timerThreadSignal.notify_all();
					}
					return result;
				case 0x24:
					updateRealTimeCounter(false);
					return realTimeCounter;
				default:
					core().dumpAndAbort("ST read %02x", addr);
					break;
			}
		}
		void writeRegister(uint32_t addr, uint32_t val, bool& errorOccurred) override {
			switch (addr) {
				case 0x04:
					{
						std::unique_lock<std::mutex> lock(timerThreadMutex);
						val &= 0xFFFF;
						periodDuration = slow_ticks(val ? val : 65536);
						periodIntervalMark = slowPointNow() + periodDuration;
						timerThreadSignal.notify_all();
					}
					break;
				case 0x0C:
					updateRealTimeCounter(true);
					{
						std::unique_lock<std::mutex> lock(timerThreadMutex);
						val &= 0xFFFF;
						realTimeDivider = val ? val : 65536;
					}
					updateAlarmMatchMark(alarmValue);
					break;
				case 0x14:
					if (val & ~ST_IRQ_ALL)
						core().dumpAndAbort("unsupported ST interrupts: %08x\n", val);
					enabledInterrupts |= val;
					emitInterruptState();
					break;
				case 0x18:
					enabledInterrupts &= ~val;
					emitInterruptState();
					break;
				case 0x20:
					val &= 0x0FFFFF;
					updateAlarmMatchMark(val ? val : 1048576);
					break;
				default:
					core().dumpAndAbort("ST write %02x (%08x)", addr, val);
					break;
			}
		}
		slow_point slowPointNow() {
			using std::chrono::time_point_cast;
			return time_point_cast<slow_ticks>(time_clock::now());
		}
		void updateRealTimeCounter(bool flushPartials) {
			slow_point nowTime = slowPointNow();
			uint64_t numSlowTicks = (nowTime - realTimeLastUpdated).count();
			uint32_t realTimeIncrement = numSlowTicks / realTimeDivider;
			realTimeCounter = (realTimeCounter + realTimeIncrement) & 0x0FFFFF;
			if (flushPartials)
				realTimeLastUpdated = nowTime;
			else
				realTimeLastUpdated += slow_ticks(realTimeIncrement * realTimeDivider);
		}
		void updateAlarmMatchMark(uint32_t newAlarmValue) {
			std::unique_lock<std::mutex> lock(timerThreadMutex);
			alarmValue = newAlarmValue;
			alarmMatchMark = realTimeLastUpdated +
				slow_ticks(((alarmValue - realTimeCounter) & 0x0FFFFF) * realTimeDivider);
			timerThreadSignal.notify_all();
		}
		void emitInterruptState() {
			intf.systemInterrupt.setInterruptState(
					SystemInterrupt::INTERRUPT_SOURCE_SYSTEM_CLOCK,
					enabledInterrupts & interruptStatus);
		}
		void timerLoop() {
			using std::chrono::time_point_cast;
			std::unique_lock<std::mutex> lock(timerThreadMutex);
			slow_point nowTime = slowPointNow();
			while (true) {
				if (periodIntervalMark < nowTime) {
					uint32_t periodsPassed = (nowTime - periodIntervalMark) / periodDuration;
					periodIntervalMark += periodsPassed * periodDuration;
				}
				if (interruptStatus == ST_IRQ_ALL) {
					// wait until further notice
					timerThreadSignal.wait(lock);
				} else {
					// wait until next
					slow_point nextMark = std::min({periodIntervalMark, alarmMatchMark});
					timerThreadSignal.wait_until(lock,
							time_point_cast<time_clock::duration>(nextMark));
				}
				nowTime = slowPointNow();
				if (nowTime >= periodIntervalMark) {
					interruptStatus |= ST_IRQ_PITS;
					periodIntervalMark += periodDuration;
					emitInterruptState();
				}
				if (nowTime >= alarmMatchMark) {
					interruptStatus |= ST_IRQ_ALMS;
					alarmMatchMark += slow_ticks((1 << 20) * realTimeDivider);
					emitInterruptState();
				}
			}
		}
	private:
		uint32_t enabledInterrupts;
		uint32_t interruptStatus;
		slow_ticks periodDuration;
		uint32_t alarmValue;
		uint32_t realTimeDivider;
		uint32_t realTimeCounter;
		slow_point realTimeLastUpdated;
		slow_point periodIntervalMark;
		slow_point alarmMatchMark;
		std::mutex timerThreadMutex;
		std::condition_variable timerThreadSignal;
		std::thread timerThread;
	};
	class PMC : public Peripheral {
	public:
		using Peripheral::Peripheral;
		void reset() override {
			enabledInterrupts = 0;
		}
		uint32_t readRegister(uint32_t addr, bool& errorOccurred) override {
			switch (addr) {
				case 0x68:
					return -1u;
				case 0x6C:
					return enabledInterrupts;
				default:
					core().dumpAndAbort("PMC read %04x", addr);
					break;
			}
		}
		void writeRegister(uint32_t addr, uint32_t val, bool& errorOccurred) override {
			switch (addr) {
				case 0x04:
					// TODO: disable clocks
					break;
				case 0x64:
					enabledInterrupts &= ~val;
					break;
				default:
					core().dumpAndAbort("PMC write %04x value %08x", addr, val);
					break;
			}
		}
	private:
		uint32_t enabledInterrupts;
	};
	class MCI : public Peripheral {
		enum MCIStatus{
			MCI_STATUS_CMDRDY = 1 << 0,
			MCI_STATUS_ALL = MCI_STATUS_CMDRDY,
		};
		struct MCIRequest {
			uint32_t modeRegister;
			uint32_t argumentRegister;
			uint32_t commandRegister;
		};
	public:
		using Peripheral::Peripheral;
		void reset() override {
			std::unique_lock<std::mutex> lock(mmcMutex);
			enabledInterrupts = 0;
			statusRegister = 0xC0E4;
			modeRegister = 0;
			argumentRegister = 0;
			emitInterruptState();
			if (!mmcThread.joinable())
				mmcThread = std::thread(&MCI::mmcLoop, this);
		}
		uint32_t readRegister(uint32_t addr, bool& errorOccurred) override {
			switch (addr) {
				case 0x40:
					return statusRegister;
				case 0x4C:
					return enabledInterrupts;
				case 0xFC: // version
					return 0x100;
				default:
					core().dumpAndAbort("MCI read %04x", addr);
					break;
			}
		}
		void writeRegister(uint32_t addr, uint32_t val, bool& errorOccurred) override {
			switch (addr) {
				case 0x00: // control register
					if (val & 0x80)
						reset();
					break;
				case 0x04: // mode register
					modeRegister = val;
					break;
				case 0x0C: // SD card register
					break;
				case 0x10: // argument register
					argumentRegister = val;
					break;
				case 0x14: // command register
					{
						std::unique_lock<std::mutex> lock(mmcMutex);
						if (statusRegister & MCI_STATUS_CMDRDY) {
							auto& req = currentRequest;
							req.modeRegister = modeRegister;
							req.argumentRegister = argumentRegister;
							req.commandRegister = val;
							statusRegister &= ~MCI_STATUS_CMDRDY;
							emitInterruptState();
							mmcSignal.notify_all();
						}
					}
					break;
				case 0x44:
					if (val & ~MCI_STATUS_ALL)
						core().dumpAndAbort("unsupported MCI interrupts: %08x\n", val);
					enabledInterrupts |= val;
					emitInterruptState();
					break;
				case 0x48:
					enabledInterrupts &= ~val;
					emitInterruptState();
					break;
				default:
					core().dumpAndAbort("MCI write %04x value %08x", addr, val);
					break;
			}
		}
		void emitInterruptState() {
			intf.interruptController.setInterruptState(
					10, enabledInterrupts & statusRegister);
		}
		void mmcLoop() {
			while (true) {
				MCIRequest req;
				{
					std::unique_lock<std::mutex> lock(mmcMutex);
					statusRegister |= MCI_STATUS_CMDRDY;
					emitInterruptState();
					while (statusRegister & MCI_STATUS_CMDRDY)
						mmcSignal.wait(lock);
					req = currentRequest;
				}
				mmcCard->doTransaction(
						req.commandRegister & 0x3F,
						req.argumentRegister);
			}
		}
		void setCard(MMCCard& card) {
			mmcCard = &card;
		}
	private:
		MMCCard *mmcCard;
		uint32_t enabledInterrupts;
		uint32_t statusRegister;
		uint32_t modeRegister;
		uint32_t argumentRegister;
		MCIRequest currentRequest;
		std::mutex mmcMutex;
		std::condition_variable mmcSignal;
		std::thread mmcThread;
	};
private:
	ARM920T *corePtr;
	std::unique_ptr<uint32_t[]> systemMemory;
	uint32_t systemMemorySize = 0;
	Peripheral *systemPeripherals[16] = { nullptr };
	Peripheral *userPeripherals[32] = { nullptr };
	SystemInterrupt systemInterrupt{*this};
	std::recursive_mutex irqMutex;
	AIC interruptController{*this, 0xFFFFF000};
	DBGU debugUnit{*this, 0xFFFFF200};
	MCI mmcInterface{*this, 0xFFFB4000};
	ST systemTimer{*this, 0xFFFFFD00};
	PMC powerManager{*this, 0xFFFFFC00};
};

class EmulatedCard : public AT91RM9200Interface::MMCCard {
public:
	void reset() {
	}
	void doTransaction(unsigned int cmd, unsigned int arg) {
		switch (cmd) {
			case 0:
				reset();
				break;
			default:
				fprintf(stderr, "EC unknown command %d\n", cmd);
				abort();
				break;
		}
	}
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

void coreMainLoop(ARM920T* coreptr) {
	ARM920T& core = *coreptr;
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

	// disable buffering on streams
	setvbuf(stdout, nullptr, _IONBF, 0);
	setvbuf(stderr, nullptr, _IONBF, 0);

	// initialize card
	EmulatedCard card;

	// initialize interface
	AT91RM9200Interface interface(card);
	interface.allocateSystemMemory(64 * 1024 * 1024);

	// initialize core
	ARM920T core(interface);

	// load kernel image
	uint32_t kernelStart = 0x8000;
	interface.loadImageIntoMemory(kernelImage.data(), kernelImage.size(), kernelStart);

	// load device blob
	uint32_t dtbStart = 8 * 1024 * 1024;
	if (kernelStart + kernelImage.size() > dtbStart) {
		std::cerr << "Error: Kernel image overlaps device blob." << std::endl;
		return 1;
	}
	interface.loadImageIntoMemory(deviceBlob.data(), deviceBlob.size(), dtbStart);

	// set registers
	core.writeRegister(1, 0xFFFFFFFF);
	core.writeRegister(2, AT91RM9200Interface::systemMemoryBase + dtbStart);
	core.setPC(AT91RM9200Interface::systemMemoryBase + kernelStart);

	// start CPU in background thread
	std::thread coreThread(coreMainLoop, &core);

	// wait forever
	pause();

}
