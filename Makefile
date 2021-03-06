##########################################################################################################################
# File automatically-generated by tool: [projectgenerator] version: [2.27.0] date: [Sat Nov 17 23:27:28 WET 2018] 
##########################################################################################################################

# ------------------------------------------------
# Generic Makefile (based on gcc)
#
# ChangeLog :
#	2017-02-10 - Several enhancements + project update mode
#   2015-07-22 - first version
# ------------------------------------------------

######################################
# target
######################################
TARGET =laser4+

######################################
# building variables
######################################
ifeq ($(RELEASE), 1)
DEBUG = 1
OPT = -Os
else
DEBUG = 1
OPT = -Og
endif

#######################################
# paths
#######################################
# source path

APP_SRC_PATH :=$(CURDIR)/app
#LIB_USB_CDC_PATH :=$(CURDIR)/lib/stm32-usb-cdc
LIB_USB_HID_PATH :=$(CURDIR)/lib/stm32-usb-cdc_hid
LIB_DFU_PATH :=$(CURDIR)/lib/stm32-dfu-bootloader
LIB_SERIAL_PATH :=$(CURDIR)/lib/stm32-serial
LIB_MULTIPROTOCOL_PATH :=$(CURDIR)/lib/multiprotocol
#LIB_MULTIPROTOCOL_PATH :=$(CURDIR)/lib/cc2500
STARTUP_PATH :=$(CURDIR)/startup
LIBEMB_PATH =./lib/libemb

FW :=STM32Cube_FW_F1_V1.8.0/

ifeq ($(shell uname -s), Linux)
REPOSITORY :=$(HOME)/STM32Cube/Repository/$(FW)
else
REPOSITORY :=C:/Users/hmr/STM32Cube/Repository/$(FW)
endif

FREERTOS_DIR :=$(REPOSITORY)Middlewares/Third_Party/FreeRTOS/Source

SOURCES_PATH =  \
$(STARTUP_PATH) \
$(APP_SRC_PATH) \
$(REPOSITORY)Drivers/CMSIS \
$(REPOSITORY)Drivers/STM32F1xx_HAL_Driver/Src \
$(REPOSITORY)Middlewares/ST/STM32_USB_Device_Library/Core/Src/ \
$(LIBEMB_PATH)/button \
$(LIBEMB_PATH)/misc \
$(LIBEMB_PATH)/console \
$(LIBEMB_PATH)/display \
$(LIBEMB_PATH)/drv/tft \
$(FREERTOS_DIR) \
$(FREERTOS_DIR)/portable/GCC/ARM_CM3 \
$(FREERTOS_DIR)/portable/MemMang \
$(FREERTOS_DIR)/CMSIS_RTOS \
$(LIB_MULTIPROTOCOL_PATH) \
$(LIB_USB_HID_PATH) \
$(LIB_USB_CDC_PATH) \
$(LIB_SERIAL_PATH) \

# firmware library path
PERIFLIB_PATH = 

# Build path
BUILD_DIR :=build

######################################
# source
######################################

# C sources
C_SOURCES =  \
$(wildcard $(APP_SRC_PATH)/*.c) \
$(STARTUP_PATH)/startup_stm32f103.c \
$(LIB_MULTIPROTOCOL_PATH)/cc2500_spi.c \
$(LIB_MULTIPROTOCOL_PATH)/FrSkyDVX_Common.c \
$(LIB_MULTIPROTOCOL_PATH)/FrSkyD_cc2500.c \
$(LIB_MULTIPROTOCOL_PATH)/ppm_decode.c \
$(LIB_SERIAL_PATH)/usart.c \
$(LIBEMB_PATH)/misc/nvdata.c \
$(LIBEMB_PATH)/misc/strfunc.c \
$(LIBEMB_PATH)/misc/fifo.c \
$(LIBEMB_PATH)/misc/debug.c \
$(LIBEMB_PATH)/drv/tft/ssd1306.c \
$(LIBEMB_PATH)/drv/display/font.c \

#C_SOURCES +=  \
$(FREERTOS_DIR)/portable/GCC/ARM_CM3/port.c \
$(FREERTOS_DIR)/portable/MemMang/heap_4.c \
$(FREERTOS_DIR)/croutine.c \
$(FREERTOS_DIR)/list.c \
$(FREERTOS_DIR)/queue.c \
$(FREERTOS_DIR)/tasks.c \
$(FREERTOS_DIR)/timers.c \
$(FREERTOS_DIR)/CMSIS_RTOS/cmsis_os.c \

ifneq ($(USB_DEVICE),NO_USB_DEVICE)

C_SOURCES += \
$(REPOSITORY)Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c \
$(REPOSITORY)Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c \
$(REPOSITORY)Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c \
$(REPOSITORY)Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c \
$(REPOSITORY)Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_usb.c \
$(REPOSITORY)Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pcd.c \
$(REPOSITORY)Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pcd_ex.c \
$(wildcard $(LIB_USB_HID_PATH)/*.c) \

SOURCES_PATH += \
$(REPOSITORY)Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/ \

endif

C_SOURCES += \
$(REPOSITORY)Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_spi.c \
$(REPOSITORY)Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_i2c.c \
$(REPOSITORY)Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c \
$(REPOSITORY)Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c \
$(REPOSITORY)Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c \
$(REPOSITORY)Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c \

# CPP sources
CPP_SOURCES += \
$(wildcard $(APP_SRC_PATH)/*.cpp) \
$(LIBEMB_PATH)/console/console.cpp \
$(LIB_MULTIPROTOCOL_PATH)/multiprotocol.cpp \

# ASM sources
ASM_SOURCES =  \
#$(STARTUP_PATH)/startup_stm32f103xb.s

######################################
# Include
######################################

# AS includes
AS_INCLUDES = 

# C includes
C_INCLUDES =  \
$(APP_SRC_PATH) \
$(LIB_USB_CDC_PATH) \
$(LIB_USB_HID_PATH) \
$(LIB_SERIAL_PATH) \
$(LIB_DFU_PATH) \
$(LIBEMB_PATH)/include \
$(REPOSITORY)Drivers/CMSIS/Include \
$(REPOSITORY)Drivers/STM32F1xx_HAL_Driver/Inc \
$(REPOSITORY)Drivers/STM32F1xx_HAL_Driver/Inc/Legacy \
$(REPOSITORY)Drivers/CMSIS/Device/ST/STM32F1xx/Include \
$(REPOSITORY)Middlewares/ST/STM32_USB_Device_Library/Core/Inc \
$(REPOSITORY)Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc \
$(FREERTOS_DIR)/include \
$(FREERTOS_DIR)/CMSIS_RTOS \
$(FREERTOS_DIR)/portable/GCC/ARM_CM3 \
$(LIB_MULTIPROTOCOL_PATH) \
#$(REPOSITORY)Middlewares/ST/STM32_USB_Device_Library/Class/HID/Inc \

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
SZ = $(BINPATH)$(PREFIX)size
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

ifeq ($(GCC_COLORS), )
export GCC_COLORS='error=01;31:warning=01;35:note=01;36:caret=01;32:locus=01:quote=01'
#unexport GCC_COLORS
endif
#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m3

# fpu
# NONE for Cortex-M0/M0+/M3

# float-abi
FLOAT-ABI = #-u_printf_float

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS += \
-DUSE_HAL_DRIVER \
-DSTM32F103xB \
-DSTM32_BOARD \
-DCC2500_INSTALLED \
-DFRSKYD_CC2500_INO \
-DAETR \
-DENABLE_PPM \
-DUSE_MY_CONFIG \
-DENABLE_SERIAL_FIFOS \
-DENABLE_USART \
-DENABLE_VCOM \
-DENABLE_DEBUG \
-DENABLE_CLI \
-DENABLE_GAME_CONTROLLER \
-DENABLE_DISPLAY \
-DUSE_MY_CONFIG \
-DFIFO_SIZE=1024 \
-DCONSOLE_PRINT_MAX_LEN=256 \

ifeq ($(XTAL), 12MHZ)
C_DEFS +=-DXTAL12MHZ
endif

ifeq ($(ENABLE_DFU), 1)
C_DEFS +=-DENABLE_DFU
endif

# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(addprefix -I, $(C_INCLUDES)) $(OPT) -Wall -fdata-sections -ffunction-sections -std=gnu11
CPPFLAGS = $(MCU) $(C_DEFS) $(addprefix -I, $(C_INCLUDES)) $(OPT) -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
DEBUGFLAGS =-g -gdwarf-2
endif

CFLAGS +=$(DEBUGFLAGS)
CPPFLAGS +=$(DEBUGFLAGS)

# Generate dependency information
#CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst))


#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT := startup/STM32F103C8Tx_FLASH.ld
#LDSCRIPT =startup/f103c8tx_dfu.ld

SPECS =-specs=nano.specs
# libraries
#LIBS =-nostartfiles -nostdlib
LIBS =-lstdc++#-lnosys -lm
LIBDIR = 
LDFLAGS = $(MCU) $(SPECS) -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections


#######################################
# Rules
#######################################
# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).bin size

ifeq ($(shell uname -s), Linux)
$(TARGET).cfg:
	@echo "Creating opencod configuration file"
	echo "interface jlink" >> $@
	echo "transport select swd" >> $@
	echo "source [find target/stm32f1x.cfg]" >> $@
	echo "adapter_khz 4000" >> $@

#use winusb driver
program: $(BUILD_DIR)/$(TARGET).elf $(TARGET).cfg
	openocd -f $(TARGET).cfg -c "program $(BUILD_DIR)/$(TARGET).elf verify reset exit"
#openocd -f $(TARGET).cfg -c "program $(BUILD_DIR)/$(TARGET).bin 0x08001000 verify reset exit"
else
JLINK =c:/tools/jlink_v500/JLink
DEVICE =STM32F103T8
$(TARGET).jlink:
	@echo "Creating Jlink configuration file"
	echo "erase" >> $@
	echo "loadbin  $(BUILD_DIR)/$(TARGET).bin , 0x08000000" >> $@
	echo "r" >> $@
	echo "q" >> $@

program: $(BUILD_DIR)/$(TARGET).bin $(TARGET).jlink
	$(JLINK) -device $(DEVICE) -if SWD -speed auto -CommanderScript $(TARGET).jlink

endif
# DFU rules
bootloader:
#$(MAKE) -C $(LIB_DFU_PATH) CONFIG="-DENABLE_GPIO_DFU_BOOT -DGPIO_DFU_BOOT_PORT=GPIOB -DGPIO_DFU_BOOT_PIN=4 -DENABLE_SAFEWRITE -DENABLE_CHECKSUM -DHSE12MHZ" CROSS_COMPILE=arm-none-eabi-
	$(MAKE) -C $(LIB_DFU_PATH) CONFIG="-DENABLE_CUSTOM_DFU_BOOT -DENABLE_SAFEWRITE -DENABLE_CHECKSUM -DHSE12MHZ" CROSS_COMPILE=arm-none-eabi-
	openocd -f $(TARGET).cfg -c "program $(LIB_DFU_PATH)/bootloader-dfu-fw.elf verify reset exit"

dfu:
	"$(MAKE)" LDSCRIPT=startup/f103c8tx_dfu.ld XTAL=12MHZ RELEASE=1 ENABLE_DFU=1
	python "$(LIB_DFU_PATH)/checksum.py" $(BUILD_DIR)/$(TARGET).bin

upload: $(BUILD_DIR)/$(TARGET).bin
	sudo dfu-util -a 0 -s 0x08001000 -D $< -R

test:
#@echo $(CURDIR)
	@echo ""; $(foreach d, $(C_SOURCES), echo $(d);)
#@echo $(C_SOURCES)

#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
#vpath %.c $(sort $(dir $(C_SOURCES)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(CPP_SOURCES:.cpp=.obj)))
#vpath %.cpp $(sort $(dir $(CPP_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
#vpath %.s $(sort $(dir $(ASM_SOURCES)))

VPATH +=$(SOURCES_PATH)

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	@echo "CC  " $<
	@$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.obj: %.cpp Makefile | $(BUILD_DIR)
	@echo "CP  " $<
	@$(CPP) -c $(CPPFLAGS)  -fno-exceptions -fno-unwind-tables -fno-rtti $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	@echo "AS " $<
	@$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) #Makefile
	@echo "--- Linking ---"
	@$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	

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
