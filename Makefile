FLAGS := -O2 -std=c++11 -pthread -Wall

all: emulator emufdt.dtb

emulator: emulator.cpp
	g++ $^ -o $@ $(FLAGS)

emufdt.dtb: emufdt.dts
	cpp -x assembler-with-cpp -I kernel/source/include -nostdinc $^ | \
		./kernel/build/scripts/dtc/dtc -o $@ -O dtb -S 4096
