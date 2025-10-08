
norule:
	@echo "Available boards:"
	@echo "	l4p, Laser4 plus"
	@echo "	bp,  bluepill"
	@echo "Build options"
	@echo "	<board>, Build with debug"
	@echo "	<board>-release, Build release build"
	@echo "	<board>-program, Program generated binary"
	@echo "	<board>-dfu, Release build for DFU"
	@echo "	<board>-bootloader, Build DFU bootloader "
	@echo "	<board>-release, Build dfu image"

all: lp4-release

program: l4p-program

bootloader: l4p-bootloader

dfu: l4p-dfu

clean:
	@$(RM) -rf build

# Common variables
MAKE_TGT = $(MAKE) -C target/laser4_plus PROJECT_DIR=$(CURDIR)
MAKE_DK415 = $(MAKE) -C target/415dk PROJECT_DIR=$(CURDIR)

# Per-board settings
L4P_FLAGS = XTAL=HSE12MHZ
BP_FLAGS  = TARGET=bp BOARD=BOARD_BLUEPILL
DK415_FLAGS = TARGET=dk415 BOARD=BOARD_DK415

# Default build targets
l4p:         ; @$(MAKE_TGT) $(L4P_FLAGS)
bp:          ; @$(MAKE_TGT) $(BP_FLAGS)
dk415:       ; @$(MAKE_DK415) $(DK415_FLAGS)

# Release build
l4p-release: ; @$(MAKE_TGT) $(L4P_FLAGS) RELEASE=1
bp-release:  ; @$(MAKE_TGT) $(BP_FLAGS) RELEASE=1

# Programming
l4p-program: ; @$(MAKE_TGT) $(L4P_FLAGS) jlink-program
bp-program:  ; @$(MAKE_TGT) $(BP_FLAGS) jlink-program
dk415-program:  ; @$(MAKE_DK415) $(DK415_FLAGS) jlink-program

# DFU builds
l4p-dfu:     ; @$(MAKE_TGT) $(L4P_FLAGS) dfu
	py lib/stm32-dfu-bootloader/bin2dfu.py build/l4p/l4p.dfu build/l4p/l4p.bin 0x8001000

bp-dfu:      ; @$(MAKE_TGT) $(BP_FLAGS) dfu
	py lib/stm32-dfu-bootloader/bin2dfu.py build/bp/bp.dfu build/bp/bp.bin 0x8001000

# Bootloaders
l4p-bootloader:
	@$(MAKE) -B -C lib/stm32-dfu-bootloader CONFIG="-DENABLE_CUSTOM_DFU_BOOT -DENABLE_SAFEWRITE -DENABLE_CHECKSUM -DHSE12MHZ" bin

bp-bootloader:
	@$(MAKE) -B -C lib/stm32-dfu-bootloader bin

# The correct bootloader.bin must be generated beforehand
stm-program.jlink:
	@echo "Creating Jlink configuration file"
	echo "erase" >> $@
	echo "loadbin lib/stm32-dfu-bootloader/bootloader-dfu-fw.bin 0x08000000" >> $@
	echo "r" >> $@
	echo "q" >> $@

stm-bootloader-program: stm-program.jlink
	JLink -device STM32F103xB -if SWD -speed auto -CommanderScript bp-dfu-program.jlink

.PHONY: