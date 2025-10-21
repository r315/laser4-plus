######################################
# target
######################################
TARGET ?=dk415-multiprotocol
BOARD  ?=BOARD_DK415
#######################################
# Features
#######################################
FEATURES += USE_MY_CONFIG
FEATURES += CC2500_INSTALLED
FEATURES += TX35_MHZ_INSTALLED
FEATURES += FRSKYD_CC2500_INO
FEATURES += ENABLE_PPM
FEATURES += ENABLE_PPM_OUTPUT
FEATURES += ENABLE_AUX_ENCODER
FEATURES += ENABLE_AUX_SWITCHES
FEATURES += ENABLE_BUZZER
FEATURES += ENABLE_BATTERY_MONITOR
#FEATURES += ENABLE_GAME_CONTROLLER
FEATURES += ENABLE_I2C
FEATURES += ENABLE_DISPLAY
#FEATURES += ENABLE_VCP
FEATURES += ENABLE_UART
FEATURES += ENABLE_CLI
FEATURES += ENABLE_DEBUG
FEATURES += ENABLE_DEBUG_BOARD
FEATURES += ENABLE_DEBUG_NVDATA
FEATURES += ENABLE_DEBUG_APP
FEATURES += ENABLE_DEBUG_MULTI
FEATURES += ENABLE_DEBUG_FRSKY


#######################################
# paths
#######################################

PROJECT_DIR :=.
BUILD_DIR   :=$(PROJECT_DIR)/build/$(TARGET)
APP_PATH    :=$(PROJECT_DIR)/app
TARGET_PATH :=$(CURDIR)
LIBEMB_PATH :=$(PROJECT_DIR)/lib/libemb
DRIVER_PATH :=$(LIBEMB_PATH)/drv
DRIVER_SOC_PATH :=$(LIBEMB_PATH)/drv/SOC/at32f4xx
DRIVER_COMPONENT_PATH :=$(LIBEMB_PATH)/component

LIB_USB_CDC_PATH :=$(PROJECT_DIR)/lib/stm32-usb-cdc
LIB_USB_HID_PATH :=$(PROJECT_DIR)/lib/stm32-usb-cdc_hid
LIB_DFU_PATH :=$(PROJECT_DIR)/lib/stm32-dfu-bootloader
LIB_SERIAL_PATH :=$(CURPROJECT_DIRDIR)/lib/stm32-serial
LIB_MULTIPROTOCOL_PATH :=$(PROJECT_DIR)/lib/multiprotocol

#######################################
# Includes
#######################################
C_INCLUDES = \
$(LIBEMB_PATH)/lib/inc \
$(LIBEMB_PATH)/drv/inc \
$(LIBEMB_PATH)/CMSIS/Core/Include \
$(LIBEMB_PATH)/CMSIS/Device/ARTERY/AT32F4xx/inc \
$(LIBEMB_PATH)/CMSIS/Device/ARTERY/AT32F4xx/inc \
$(LIBEMB_PATH)/component/display \
$(TARGET_PATH)/inc \
$(DRIVER_SOC_PATH)/inc \
$(LIB_MULTIPROTOCOL_PATH) \
$(APP_PATH) \

AS_INCLUDES =\

######################################
# Sources
######################################
TARGET_SOURCES = \
$(LIBEMB_PATH)/CMSIS/Device/ARTERY/AT32F4xx/src/startup_at32f415cbt7.c \
$(LIBEMB_PATH)/CMSIS/Device/ARTERY/AT32F4xx/src/system_at32f4xx.c \
$(TARGET_PATH)/src/415dk_board.c \
$(DRIVER_SOC_PATH)/src/at32f4xx_rcc.c \
$(DRIVER_PATH)/uart/uart_at32f4xx.c \
$(DRIVER_PATH)/dma/dma_at32f4xx.c \
$(DRIVER_PATH)/flash/fls_at32f4xx.c \
$(DRIVER_PATH)/gpio/gpio_at32f4xx.c \
$(DRIVER_PATH)/i2c/i2c_at32f4xx.c \
$(DRIVER_PATH)/spi/spi_at32f4xx.c \
$(DRIVER_PATH)/tone/tone_at32f4xx.c \
$(DRIVER_PATH)/wdt/wdt_at32f4xx.c \

LIB_SOURCES = \
$(LIBEMB_PATH)/lib/src/fifo.c \
$(LIBEMB_PATH)/lib/src/strfunc.c \
$(LIBEMB_PATH)/lib/src/fifo.c \
$(LIBEMB_PATH)/lib/src/nvdata.c \
$(LIBEMB_PATH)/lib/src/debug.c \
$(LIBEMB_PATH)/lib/src/stimer.c \
$(LIBEMB_PATH)/lib/src/lib2d.c \
$(LIBEMB_PATH)/lib/src/font.c \
$(LIBEMB_PATH)/component/display/ssd1306.c \

C_SOURCES = \
$(TARGET_SOURCES) \
$(LIB_SOURCES) \
$(LIB_MULTIPROTOCOL_PATH)/cc2500_spi.c \
$(LIB_MULTIPROTOCOL_PATH)/FrSkyDVX_Common.c \
$(LIB_MULTIPROTOCOL_PATH)/FrSkyD_cc2500.c \
$(LIB_MULTIPROTOCOL_PATH)/ppm_decode.c \

CPP_SOURCES = \
$(LIB_MULTIPROTOCOL_PATH)/multiprotocol.cpp \
$(APP_PATH)/laser4_plus.cpp \
$(APP_PATH)/commands.cpp \
$(APP_PATH)/mpanel.cpp \
$(LIBEMB_PATH)/lib/src/console.cpp \

LDSCRIPT :=AT32F415xB_FLASH.ld
#######################################
# Misc
#######################################

C_DEFS +=\
$(BOARD) \
$(DEVICE) \
$(FEATURES) \
USE_STDPERIPH_DRIVER \
UART_RX_MODE=UART_MODE_FIFO \
UART_TX_MODE=UART_MODE_FIFO \
FIFO_SIZE=128 \
FIFO_INTERRUPT_SAFE \
CONSOLE_WIDTH=128 \
USE_PIXELDUST_FONT \
$(VERSION) \

#######################################
# CFLAGS
#######################################

DEVICE =AT32F415CBT7
CORE =cortex-m4
MCU =-mcpu=$(CORE) -mthumb

ifdef RELEASE
GIT_TAG :=$(shell git describe --abbrev=0 --tags 2>/dev/null || true)
VERSION :=VERSION_STRING=\"$(GIT_TAG)\"
OPT   =-O2
else
OPT   =-O0 -g -gdwarf-2 -Wall -Wextra
endif

ifndef V
VERBOSE =@
endif

ASFLAGS  =$(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -fdata-sections -ffunction-sections
CFLAGS   =$(MCU) $(OPT) $(addprefix -D, $(C_DEFS)) $(addprefix -I, $(C_INCLUDES)) -fdata-sections -ffunction-sections -fno-builtin -std=gnu11
CPPFLAGS =$(MCU) $(OPT) $(addprefix -D, $(C_DEFS)) $(addprefix -I, $(C_INCLUDES)) -fdata-sections -ffunction-sections -fno-unwind-tables -fno-exceptions -fno-rtti -fno-builtin
LDFLAGS  =$(MCU) $(SPECS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,-gc-sections,-cref

#CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst))
#######################################
# Libraries
#######################################

LIBDIR =
ifdef SEMIHOSTING
SPECS =--specs=rdimon.specs
LDLIBS =-nostartfiles -lc -lrdimon #-nostdlib
else
SPECS=--specs=nano.specs #--specs=nosys.specs
LIBS =-lstdc++
endif

#######################################
# Objects
#######################################

OBJECTS = $(addprefix $(BUILD_DIR)/, $(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

OBJECTS += $(addprefix $(BUILD_DIR)/, $(notdir $(CPP_SOURCES:.cpp=.obj)))
vpath %.cpp $(sort $(dir $(CPP_SOURCES)))

#OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
#vpath %.s $(sort $(dir $(ASM_SOURCES)))

#######################################
# Tool binaries
#######################################
PREFIX = arm-none-eabi-
CC  =$(PREFIX)gcc
CPP =$(PREFIX)g++
AS  =$(PREFIX)gcc -x assembler-with-cpp
LD  =$(PREFIX)gcc
AR  =$(PREFIX)ar
SZ  =$(PREFIX)size
OBJCOPY =$(PREFIX)objcopy
OBJDUMP =$(PREFIX)objdump
HEX = $(OBJCOPY) -O ihex
BIN = $(OBJCOPY) -O binary -S

ifeq ($(shell uname -s), Linux)
JLK =JLinkExe
else
JLK =JLink
endif

#######################################
# Rules
#######################################
all: $(BUILD_DIR)/$(TARGET).elf size
#@echo $(CMSIS_PATH)

bin: $(BUILD_DIR)/$(TARGET).bin

jlink-program: bin $(BUILD_DIR)/$(TARGET).jlink
	$(VERBOSE)$(JLK) -device $(DEVICE) -if SWD -speed auto -CommanderScript $(BUILD_DIR)/$(TARGET).jlink

stlink-program: bin
	$(VERBOSE)openocd -f $(TARGET_PATH)/at32f415.cfg -c "program $(BUILD_DIR)/$(TARGET).elf verify reset exit"
#ERASE_CMD =openocd -f $(PRG_CFG) -c "init" -c "halt" -c "stm32f1x unlock 0" -c "stm32f1x mass_erase 0" -c "exit"

$(BUILD_DIR)/$(TARGET).jlink: $(BUILD_DIR)/$(TARGET).bin
	@echo "Creating Jlink configuration file"
	@echo "loadfile $< 0x08000000" > $@
	@echo "r" >> $@
	@echo "q" >> $@
#######################################
# build rules
#######################################
$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	@echo "[CC]  $<"
	$(VERBOSE)$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.obj: %.cpp Makefile | $(BUILD_DIR)
	@echo "[CP]  $<"
	$(VERBOSE)$(CPP) -c $(CPPFLAGS)  $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	@echo "[AS]  $<"
	$(VERBOSE)$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) | Makefile
	@echo "[LD]  $@"
	$(VERBOSE)$(LD) $(LIBDIR) -T$(LDSCRIPT) $(LDFLAGS) $^ $(LIBS) -o $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(VERBOSE)$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(VERBOSE)$(BIN) $< $@

$(BUILD_DIR):
	mkdir -p $@

size: $(BUILD_DIR)/$(TARGET).elf
	@echo "--- Size ---"
	$(VERBOSE)$(SZ) -A -x $<
	$(VERBOSE)$(SZ) -B $<

#######################################
# clean up
#######################################
clean:
	$(VERBOSE)-rm -fR $(BUILD_DIR)

#######################################
# dependencies
#######################################
#-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)
ifeq ($(GCC_COLORS), )
export GCC_COLORS='error=01;31:warning=01;35:note=01;36:caret=01;32:locus=01:quote=01'
#unexport GCC_COLORS
endif

# *** EOF ***
