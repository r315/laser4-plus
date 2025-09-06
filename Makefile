
norule:
	@echo "specify board"
	@echo "Available boards:"
	@echo "	l4p - Laser4 plus"
	@echo "	bp  - bluepill"

all: lp4-release

clean:
	@$(RM) -rf build

l4p:
	@"$(MAKE)" -C target/laser4_plus PROJECT_DIR=$(CURDIR) XTAL=HSE12MHZ

bp:
	@"$(MAKE)" -C target/laser4_plus PROJECT_DIR=$(CURDIR) TARGET=bp BOARD=BOARD_BLUEPILL

l4p-program:
	@"$(MAKE)" -C target/laser4_plus PROJECT_DIR=$(CURDIR) XTAL=HSE12MHZ program

bp-program:
	@"$(MAKE)" -C target/laser4_plus PROJECT_DIR=$(CURDIR) TARGET=bp BOARD=BOARD_BLUEPILL program

l4p-release:
	@"$(MAKE)" -C target/laser4_plus PROJECT_DIR=$(CURDIR) XTAL=HSE12MHZ dfu

bp-release:
	@"$(MAKE)" -C target/laser4_plus PROJECT_DIR=$(CURDIR) TARGET=bp BOARD=BOARD_BLUEPILL dfu

l4p-bootloader:
	@$(MAKE) -B -C lib/stm32-dfu-bootloader CONFIG="-DENABLE_CUSTOM_DFU_BOOT -DENABLE_SAFEWRITE -DENABLE_CHECKSUM" HSE=HSE12MHZ bin

bp-bootloader:
	@$(MAKE) -B -C lib/stm32-dfu-bootloader bin

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