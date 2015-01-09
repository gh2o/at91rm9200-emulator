FLAGS := -O2 -std=c++11 -pthread -Wall

all: emulator kernel/emulator.dtb

emulator: emulator.cpp
	g++ $^ -o $@ $(FLAGS)

kernel/emulator.dtb: kernel/emulator.dts
	kernel/makedtb
