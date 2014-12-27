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

__attribute__((noreturn, format(printf, 1, 2)))
void abortWithMessage(const char *format, ...) {
	fprintf(stderr, "Abort: ");
	va_list args;
	va_start(args, format);
	vfprintf(stderr, format, args);
	va_end(args);
	fputc('\n', stderr);
	abort();
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
		PSR_BITS_E = 1 << 9
	};
	enum TickError {
		TICK_ERROR_NONE = 0,
		TICK_ERROR_PREFETCH_ABORT,
		TICK_ERROR_DATA_ABORT,
	};
public:
	IMX233() : memoryController(*this) {
		reset();
	}
	void reset() {
		registerFile.reset();
	}
	void tick() {
		TickState tickState;
		tickState.tickError = TICK_ERROR_NONE;
		tickState.nextPC = getPC() + 4;
		tickInternal(tickState);
		if (tickState.tickError != TICK_ERROR_NONE)
			abortWithMessage("tick error occurred");
		else
			setPC(tickState.nextPC);
	}
	void tickInternal(TickState& tickState) {
		bool errorOccurred = false;
		// fetch instruction
		uint32_t encodedInst = memoryController.readWord(getPC(), &errorOccurred);
		if (errorOccurred) {
			tickState.tickError = TICK_ERROR_PREFETCH_ABORT;
			return;
		}
		// only execute if condition passes
		if ((encodedInst >> 28) != 0x0E)
			abortWithMessage("unknown condition");
		bool condPass = true;
		if (!condPass)
			return;
		// decode and dispatch
		unsigned int dec1 = (encodedInst >> 25) & 0x07;
		unsigned int dec2 = (encodedInst >> 20) & 0x1F;
		unsigned int dec3 = (encodedInst >> 4) & 0x0F;
		switch (dec1) {
			case 1:
				switch (dec2) {
					case 18: // MSR CPSR, #immed
					case 22: // MSR SPSR, #immed
					{
						uint32_t _8_bit_immediate = encodedInst & 0xFF;
						uint32_t rotate_imm = (encodedInst >> 8) & 0x0F;
						uint32_t operand = rotateRight(_8_bit_immediate, rotate_imm * 2);
						inst_MSR(tickState,
								encodedInst & (1 << 22), /* R */
								(encodedInst >> 16) & 0x0F, /* field_mask */
								operand);
						break;
					}
					default:
						abortWithMessage("decode 1.%d unknown", dec2);
				}
				break;
			case 7:
				switch (dec2) {
					default:
						abortWithMessage("decode 7.%d unknown", dec2);
				}
			default:
				abortWithMessage("decode %d unknown", dec1);
		}
	}
	void inst_MSR(TickState& tickState, bool R, uint32_t field_mask, uint32_t operand) {
		uint32_t mask =
			((field_mask & (1 << 0)) ? 0x000000FF : 0) |
			((field_mask & (1 << 1)) ? 0x0000FF00 : 0) |
			((field_mask & (1 << 2)) ? 0x00FF0000 : 0) |
			((field_mask & (1 << 3)) ? 0xFF000000 : 0);
		if (operand & 0x0FFFFF00)
			abortWithMessage("MSR: attempted to set reserved bits");
		if (!R) {
			if (isPrivileged()) {
				if (operand & 0x00000020)
					abortWithMessage("MSR: attempted to set non-ARM execution state");
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
	uint32_t getPC() { return registerFile.getPC(); }
	void setPC(uint32_t val) { registerFile.setPC(val); }
private:
	struct TickState {
		TickError tickError;
		uint32_t nextPC;
	};
	class RegisterFile {
	public:
		RegisterFile() {
			registerView[15] = &programCounter;
			for (int i = 0; i <= 7; i++)
				registerView[i] = &(nonBanked[i]);
			reset();
		}
		void reset() {
			writeCPSR(CPU_MODE_SVC | PSR_BITS_F | PSR_BITS_I | PSR_BITS_A);
		}
		uint32_t readCPSR() {
			return curCPSR;
		}
		void writeCPSR(uint32_t newCPSR) {
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
				abortWithMessage("read from non-existent SPSR");
			return *curSPSRView;
		}
		void writeSPSR(uint32_t val) {
			if (curSPSRView == &(storedSPSR[0]))
				abortWithMessage("write to non-existent SPSR");
			*curSPSRView = val;
		}
		uint32_t readRegister(uint32_t reg) {
			uint32_t *regPtr = registerView[reg];
			if (regPtr == &programCounter)
				return programCounter + 8;
			else
				return *regPtr;
		}
		void writeRegister(uint32_t reg, uint32_t val) {
			*(registerView[reg]) = val;
		}
		uint32_t getPC() {
			return programCounter;
		}
		void setPC(uint32_t val) {
			programCounter = val;
		}
	private:
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
		uint32_t readWord(uint32_t addr, bool *errorOccurred) {
			return readWordPhysical(addr, errorOccurred);
		}
		void writeWord(uint32_t addr, uint32_t val, bool *errorOccurred) {
			writeWordPhysical(addr, val, errorOccurred);
		}
		uint32_t readWordPhysical(uint32_t addr, bool *errorOccurred) {
			if (addr >= core.systemMemoryBase && addr < core.systemMemoryBase + core.systemMemorySize)
				return core.systemMemory[(addr - systemMemoryBase) / 4];
			abortWithMessage("readWordPhysical");
		}
		void writeWordPhysical(uint32_t addr, uint32_t val, bool *errorOccurred) {
			if (addr >= core.systemMemoryBase && addr < core.systemMemoryBase + core.systemMemorySize)
				core.systemMemory[(addr - systemMemoryBase) / 4] = val;
			abortWithMessage("writeWordPhysical");
		}
	private:
		IMX233& core;
	};
	class SystemControlCoprocessor {
	public:
		SystemControlCoprocessor(IMX233& core) : core(core) {}
	private:
		IMX233& core;
	};
private:
	std::unique_ptr<uint32_t[]> systemMemory;
	uint32_t systemMemorySize = 0;
	RegisterFile registerFile;
	MemoryController memoryController;
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
