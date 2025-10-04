######################################
# target
######################################
TARGET ?=l4p
BOARD  ?=BOARD_LASER4PLUS
XTAL   ?=HSE8MHZ
######################################
# Features
######################################
FEATURES += USE_MY_CONFIG
FEATURES += CC2500_INSTALLED
FEATURES += TX35_MHZ_INSTALLED
FEATURES += FRSKYD_CC2500_INO
#FEATURES += ENABLE_FAILSAFE
FEATURES += ENABLE_PPM
FEATURES += ENABLE_PPM_OUTPUT
FEATURES += ENABLE_BATTERY_MONITOR
FEATURES += ENABLE_GAME_CONTROLLER
FEATURES += ENABLE_I2C
FEATURES += ENABLE_DISPLAY
FEATURES += ENABLE_VCP
FEATURES += ENABLE_AUX_ENCODER
FEATURES += ENABLE_AUX_SWITCHES
#FEATURES += ENABLE_UART
FEATURES += ENABLE_CLI
FEATURES += ENABLE_BUZZER
FEATURES += ENABLE_DEBUG
FEATURES += ENABLE_DEBUG_APP
FEATURES += ENABLE_DEBUG_NVDATA
FEATURES += ENABLE_DEBUG_MULTI
FEATURES += ENABLE_DEBUG_FRSKY

#######################################
# paths
#######################################
# source path
PROJECT_DIR :=.
BUILD_DIR :=$(PROJECT_DIR)/build/$(TARGET)
APP_PATH  :=$(PROJECT_DIR)/app
TARGET_PATH =$(CURDIR)
LIBEMB_PATH =$(PROJECT_DIR)/lib/libemb

LIB_USB_PATH :=$(PROJECT_DIR)/lib/stm32-usb-cdc_hid
LIB_DFU_PATH :=$(PROJECT_DIR)/lib/stm32-dfu-bootloader
LIB_SERIAL_PATH :=$(CURPROJECT_DIRDIR)/lib/stm32-serial
LIB_MULTIPROTOCOL_PATH :=$(PROJECT_DIR)/lib/multiprotocol

######################################
# Include
######################################

# AS includes
AS_INCLUDES =

# C includes
C_INCLUDES =  \
$(APP_PATH) \
$(TARGET_PATH)/inc \
$(LIB_USB_PATH) \
$(LIB_SERIAL_PATH) \
$(LIB_DFU_PATH) \
$(LIBEMB_PATH)/lib/inc \
$(LIBEMB_PATH)/drv/inc \
$(LIBEMB_PATH)/component/display \
$(LIBEMB_PATH)/CMSIS/Device/ST/STM32F10x/inc \
$(LIBEMB_PATH)/CMSIS/Core/Include \
$(LIB_MULTIPROTOCOL_PATH) \

######################################
# source files
######################################

TARGET_SOURCES =  \
$(LIBEMB_PATH)/CMSIS/Device/ST/STM32F10x/src/startup_stm32f103xb.c \
$(LIBEMB_PATH)/drv/uart/uart_stm32f10x.c \
$(LIBEMB_PATH)/drv/dma/dma_stm32f1xx.c \
$(LIBEMB_PATH)/drv/tone/tone_stm32f1xx.c \
$(LIBEMB_PATH)/drv/gpio/gpio_stm32f1xx.c \
$(LIBEMB_PATH)/drv/clock/clock_stm32f1xx.c \
$(LIBEMB_PATH)/drv/spi/spi_stm32f1xx.c \
$(LIBEMB_PATH)/drv/i2c/i2c_stm32f1xx.c \
$(TARGET_PATH)/src/laser4_plus_board.c \

APP_SOURCES = \
$(LIB_MULTIPROTOCOL_PATH)/cc2500_spi.c \
$(LIB_MULTIPROTOCOL_PATH)/FrSkyDVX_Common.c \
$(LIB_MULTIPROTOCOL_PATH)/FrSkyD_cc2500.c \
$(LIB_MULTIPROTOCOL_PATH)/ppm_decode.c \
$(LIBEMB_PATH)/lib/src/strfunc.c \
$(LIBEMB_PATH)/lib/src/fifo.c \
$(LIBEMB_PATH)/lib/src/nvdata.c \
$(LIBEMB_PATH)/lib/src/debug.c \
$(LIBEMB_PATH)/component/display/ssd1306.c \
$(LIBEMB_PATH)/component/display/font.c \
$(APP_PATH)/game_controller.c \

C_SOURCES = \
$(TARGET_SOURCES) \
$(APP_SOURCES) \


ifneq ($(filter ENABLE_VCP, $(FEATURES)),)
C_SOURCES += $(USB_DEVICE_SOURCES)
endif

# CPP sources
CPP_SOURCES += \
$(wildcard $(APP_PATH)/*.cpp) \
$(LIBEMB_PATH)/lib/src/console.cpp \
$(LIB_MULTIPROTOCOL_PATH)/multiprotocol.cpp \

# ASM sources
ASM_SOURCES =  \

#######################################
# binaries
#######################################
BINPATH =
PREFIX = arm-none-eabi-
CC = $(BINPATH)$(PREFIX)gcc
CPP = $(BINPATH)$(PREFIX)g++
AS = $(BINPATH)$(PREFIX)gcc -x assembler-with-cpp
CP = $(BINPATH)$(PREFIX)objcopy
AR = $(BINPATH)$(PREFIX)ar
LD = $(BINPATH)$(PREFIX)gcc
SZ = $(BINPATH)$(PREFIX)size
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
JLINK =JLink

ifeq ($(GCC_COLORS), )
export GCC_COLORS='error=01;31:warning=01;35:note=01;36:caret=01;32:locus=01:quote=01'
#unexport GCC_COLORS
endif
#######################################
# CFLAGS
#######################################
DEVICE =STM32F103xB

# Core
CORE =cortex-m3

# fpu
# NONE for Cortex-M0/M0+/M3

# float-abi
#FLOAT-ABI = #-u_printf_float

# mcu
MCU = -mcpu=$(CORE) -mthumb

ifeq ($(RELEASE),1)
GIT_TAG :=$(shell git describe --abbrev=0 --tags 2>/dev/null || true)
VERSION :=VERSION_STRING=\"$(GIT_TAG)\"
OPT = -Os -Wall -Wextra
else
OPT = -O0 -g -gdwarf-2 -Wall -Wextra
endif

# macros for gcc
# AS defines
AS_DEFS =

# C defines
C_DEFS = \
$(DEVICE) \
$(BOARD) \
$(FEATURES) \
UART_RX_MODE=UART_MODE_FIFO \
UART_TX_MODE=UART_MODE_FIFO \
FIFO_SIZE=128 \
FIFO_INTERRUPT_SAFE \
CONSOLE_WIDTH=128 \
USE_PIXELDUST_FONT \
$(XTAL) \
$(VERSION) \

ifeq ($(ENABLE_DFU),1)
C_DEFS +=ENABLE_DFU
endif

# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -fdata-sections -ffunction-sections
CFLAGS = $(MCU) $(addprefix -D, $(C_DEFS)) $(addprefix -I, $(C_INCLUDES)) $(OPT) -fdata-sections -ffunction-sections -fno-builtin -std=gnu11
CPPFLAGS = $(MCU) $(addprefix -D, $(C_DEFS)) $(addprefix -I, $(C_INCLUDES)) $(OPT) -fdata-sections -ffunction-sections -fno-unwind-tables -fno-exceptions -fno-rtti -fno-builtin

# Generate dependency information
#CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst))

#######################################
# Linker flags and libraries
#######################################
# link script
LDSCRIPT :=startup/STM32F103C8Tx_FLASH.ld

SPECS =--specs=nano.specs #--specs=nosys.specs

# libraries
#-nostartfiles -nostdlib -lnosys -lm
  #-lrdimon
LIBS =-lstdc++ -lcdchid
LIBDIR =-L$(LIB_USB_PATH)
LDFLAGS = $(MCU) $(SPECS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,-gc-sections,-cref

#######################################
# Rules
#######################################
# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).bin size

dfu:
	"$(MAKE)" LDSCRIPT=startup/f103c8tx_dfu.ld TARGET=$(TARGET) BOARD=$(BOARD) XTAL=$(XTAL) RELEASE=1 ENABLE_DFU=1
#confirm if checksum.py only necessary with dfu-util
#py "$(LIB_DFU_PATH)/checksum.py" $(BUILD_DIR)/$(TARGET).bin


$(BUILD_DIR)/$(TARGET).cfg:
	@echo "Creating opencod configuration file"
	echo "interface jlink" >> $@
	echo "transport select swd" >> $@
	echo "source [find target/stm32f1x.cfg]" >> $@
	echo "adapter_khz 4000" >> $@

#use winusb driver
openocd-program: $(BUILD_DIR)/$(TARGET).elf $(TARGET).cfg
	openocd -f $(TARGET).cfg -c "program $(BUILD_DIR)/$(TARGET).elf verify reset exit"
#openocd -f $(TARGET).cfg -c "program $(BUILD_DIR)/$(TARGET).bin 0x08001000 verify reset exit"

$(BUILD_DIR)/$(TARGET).jlink:
	@echo "Creating Jlink configuration file"
	echo "erase" >> $@
	echo "loadbin  $(BUILD_DIR)/$(TARGET).bin , 0x08000000" >> $@
	echo "r" >> $@
	echo "q" >> $@

jlink-program: $(BUILD_DIR)/$(TARGET).bin $(BUILD_DIR)/$(TARGET).jlink
	$(JLINK) -device $(DEVICE) -if SWD -speed auto -CommanderScript $(BUILD_DIR)/$(TARGET).jlink

dfu-upload: $(BUILD_DIR)/$(TARGET).bin
	sudo dfu-util -a 0 -s 0x08001000 -D $< -R

#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(CPP_SOURCES:.cpp=.obj)))
vpath %.cpp $(sort $(dir $(CPP_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
#vpath %.s $(sort $(dir $(ASM_SOURCES)))

#VPATH +=$(SOURCES_PATH)

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	@echo "CC  " $<
	@$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.obj: %.cpp Makefile | $(BUILD_DIR)
	@echo "CP  " $<
	@$(CPP) -c $(CPPFLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	@echo "AS " $<
	@$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) #Makefile
	@echo "--- Linking ---"
	@$(LD) $(LIBDIR) -T$(LDSCRIPT) $(LDFLAGS) $(OBJECTS) $(LIBS) -o $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@

$(BUILD_DIR):
	mkdir -p $@

size: $(BUILD_DIR)/$(TARGET).elf
	@echo "--- Size ---"
	$(SZ) -A -x $<
	$(SZ) -B $<

#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)

#######################################
# dependencies
#######################################
#-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)

# *** EOF ***
