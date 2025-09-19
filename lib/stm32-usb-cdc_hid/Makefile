TARGET :=libcdchid
LIB_USB_HID_PATH :=$(CURDIR)
BUILD_DIR :=build
FW :=STM32Cube_FW_F1_V1.8.4/
ifeq ($(shell uname -s), Linux)
REPOSITORY :=$(HOME)/STM32Cube/Repository/$(FW)
else
REPOSITORY :=C:/Users/hmr/STM32Cube/Repository/$(FW)
endif

C_INCLUDES =  \
$(LIB_USB_HID_PATH) \
$(REPOSITORY)/Drivers/CMSIS/Core/Include \
$(REPOSITORY)/Drivers/CMSIS/Device/ST/STM32F1xx/Include \
$(REPOSITORY)Drivers/STM32F1xx_HAL_Driver/Inc \
$(REPOSITORY)Drivers/STM32F1xx_HAL_Driver/Inc/Legacy \
$(REPOSITORY)Middlewares/ST/STM32_USB_Device_Library/Core/Inc \
$(REPOSITORY)Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc \
$(LIBEMB_PATH)/lib/inc

C_SOURCES =\
$(REPOSITORY)Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c \
$(REPOSITORY)Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c \
$(REPOSITORY)Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c \
$(REPOSITORY)Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c \
$(REPOSITORY)Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_usb.c \
$(REPOSITORY)Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pcd.c \
$(REPOSITORY)Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pcd_ex.c \
$(LIB_USB_HID_PATH)/usb_device.c \
$(LIB_USB_HID_PATH)/usbd_cdc.c \
$(LIB_USB_HID_PATH)/usbd_composite.c \
$(LIB_USB_HID_PATH)/usbd_conf.c \
$(LIB_USB_HID_PATH)/usbd_desc.c \
$(LIB_USB_HID_PATH)/usbd_cdc_if.c \
$(LIB_USB_HID_PATH)/usbd_joystick_hid.c

#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
CC = $(BINPATH)$(PREFIX)gcc
AR = $(BINPATH)$(PREFIX)ar

#######################################
# CFLAGS
#######################################
DEVICE =STM32F103xB

# Core
CORE =cortex-m3

MCU = -mcpu=$(CORE) -mthumb

OPT = -Os -Wall -Wextra

C_DEFS = \
$(DEVICE) \
USE_HAL_DRIVER \

# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -fdata-sections -ffunction-sections
CFLAGS = $(MCU) $(addprefix -D, $(C_DEFS)) $(addprefix -I, $(C_INCLUDES)) $(OPT) -fdata-sections -ffunction-sections -fno-builtin -std=gnu11
CPPFLAGS = $(MCU) $(addprefix -D, $(C_DEFS)) $(addprefix -I, $(C_INCLUDES)) $(OPT) -fdata-sections -ffunction-sections -fno-unwind-tables -fno-exceptions -fno-rtti -fno-builtin

all: $(TARGET).a

clean:
	rm -fR $(BUILD_DIR) $(TARGET).a

#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

$(BUILD_DIR)/%.o: %.c  $(BUILD_DIR)
	@echo "CC  " $<
	@$(CC) -c $(CFLAGS) $< -o $@

$(TARGET).a: $(OBJECTS)
	$(AR) rcs $@ $^

$(BUILD_DIR):
	mkdir -p $@

.PHONY: clean