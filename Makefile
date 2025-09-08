
norule:
	@echo "Available boards:"
	@echo "	l4p, Laser4 plus"
	@echo "	bp,  bluepill"
	@echo "Build options"
	@echo "	<board>-bootloader, Build DFU bootloader "
	@echo "	<board>-release, Build dfu image"

all: lp4-release

clean:
	@$(RM) -rf build

# Common variables
MAKE_TGT = $(MAKE) -C target/laser4_plus PROJECT_DIR=$(CURDIR)

# Per-board settings
L4P_FLAGS = XTAL=HSE12MHZ
BP_FLAGS  = TARGET=bp BOARD=BOARD_BLUEPILL

# Default build targets
l4p:         ; @$(MAKE_TGT) $(L4P_FLAGS)
bp:          ; @$(MAKE_TGT) $(BP_FLAGS)

# Programming
l4p-program: ; @$(MAKE_TGT) $(L4P_FLAGS) jlink-program
bp-program:  ; @$(MAKE_TGT) $(BP_FLAGS) jlink-program

# Release builds
l4p-release: ; @$(MAKE_TGT) $(L4P_FLAGS) dfu
	py lib/stm32-dfu-bootloader/bin2dfu.py build/l4p/l4p.dfu build/l4p/l4p.bin 0x8001000

bp-release:  ; @$(MAKE_TGT) $(BP_FLAGS) dfu
	py lib/stm32-dfu-bootloader/bin2dfu.py build/bp/bp.dfu build/bp/bp.bin 0x8001000

# Bootloaders
l4p-bootloader:
	@$(MAKE) -B -C lib/stm32-dfu-bootloader CONFIG="-DENABLE_CUSTOM_DFU_BOOT -DENABLE_SAFEWRITE -DENABLE_CHECKSUM -DHSE12MHZ" bin

bp-bootloader:
	@$(MAKE) -B -C lib/stm32-dfu-bootloader bin

%-release:
	@$(MAKE_TGT) $(if $(findstring l4p,$@),$(L4P_FLAGS),$(BP_FLAGS)) dfu
	py lib/stm32-dfu-bootloader/bin2dfu.py build/$*/$*.dfu build/$*/$*.bin 0x8001000

# The correct bootloader.bin must be created beforhand
stm-program.jlink:
	@echo "Creating Jlink configuration file"
	echo "erase" >> $@
	echo "loadbin lib/stm32-dfu-bootloader/bootloader-dfu-fw.bin 0x08000000" >> $@
	echo "r" >> $@
	echo "q" >> $@

stm-bootloader-program: stm-program.jlink
	JLink -device STM32F103xB -if SWD -speed auto -CommanderScript bp-dfu-program.jlink

.PHONY: