###############################################################################
#
# Makefile for building the baseflight firmware.
#
# Invoke this with 'make help' to see the list of supported targets.
# 

# The target to build
#TARGET		?= BOOTLOADER
TARGET		?= 10dM3UOP88

# Compile-time options
OPTIONS		?=

###############################################################################
# Things that need to be maintained as the source changes
#

VALID_TARGETS	 = 10dM3UOP88 BOOTLOADER

# Working directories
# ROOT		 	= $(dir $(lastword $(MAKEFILE_LIST)))
ROOT			= .
DIR_BOOTLOADER  = $(ROOT)/bootloader
DIR_APP		 	= $(ROOT)/app
DIR_DRV			= $(ROOT)/drv
DIR_USB			= $(ROOT)/usb
DIR_OSD			= $(ROOT)/osd
DIR_LIB_PRINTF	= $(ROOT)/lib/printf
DIR_LIB_FIFO	= $(ROOT)/lib/FifoBuffer
DIR_LIB_MAVLINK	= $(ROOT)/lib/mavlink
DIR_CMSIS	 	= $(ROOT)/lib/CMSIS
DIR_LIB_USB		= $(ROOT)/lib/STM32_USB-FS-Device_Driver
DIR_STDPERIPH	= $(ROOT)/lib/STM32F10x_StdPeriph_Driver
DIR_FREERTOS    = $(ROOT)/lib/FreeRTOS
OBJECT_DIR	 	= $(ROOT)/../obj
BIN_DIR		 	= $(ROOT)/../obj

# Source files for the ABSINTH target
SRC_10dM3UOP88 = $(DIR_APP)/src/main.c \
	$(DIR_APP)/src/cli.c \
	$(DIR_APP)/src/config.c \
	$(DIR_APP)/src/control.c \
	$(DIR_APP)/src/FreeRTOSHooks.c \
	$(DIR_APP)/src/frsky.c \
	$(DIR_APP)/src/gps.c \
	$(DIR_APP)/src/imu.c \
	$(DIR_APP)/src/MadgwickAHRS.c \
	$(DIR_APP)/src/mavlink.c \
	$(DIR_APP)/src/mission.c \
	$(DIR_APP)/src/mixer.c \
	$(DIR_APP)/src/msp.c \
	$(DIR_APP)/src/navigate.c \
	$(DIR_APP)/src/sbus.c \
	$(DIR_APP)/src/sensors.c \
	$(DIR_APP)/src/signal.c \
	$(DIR_APP)/src/simulator.c \
	$(DIR_APP)/src/spektrum.c \
	$(DIR_APP)/src/syscalls.c \
	$(DIR_APP)/src/waypoints.c \
	$(DIR_DRV)/src/adc.c \
	$(DIR_DRV)/src/hcsr04.c \
	$(DIR_DRV)/src/i2c.c \
	$(DIR_DRV)/src/l3gd20.c \
	$(DIR_DRV)/src/ledring.c \
	$(DIR_DRV)/src/lps331ap.c \
	$(DIR_DRV)/src/lsm303dlhc.c \
	$(DIR_DRV)/src/pwm.c \
	$(DIR_DRV)/src/rtc.c \
	$(DIR_DRV)/src/system.c \
	$(DIR_DRV)/src/uart.c \
	$(DIR_DRV)/src/wdg.c \
	$(DIR_USB)/src/hw_config.c \
	$(DIR_USB)/src/usb_desc.c \
	$(DIR_USB)/src/usb_endp.c \
	$(DIR_USB)/src/usb_istr.c \
	$(DIR_USB)/src/usb_prop.c \
	$(DIR_USB)/src/usb_pwr.c \
	$(DIR_OSD)/src/fonts.c \
	$(DIR_OSD)/src/font_outlined8x8.c \
	$(DIR_OSD)/src/font_outlined8x14.c \
	$(DIR_OSD)/src/osd_core.c \
	$(DIR_OSD)/src/osd_gen.c \
	$(DIR_LIB_FIFO)/src/fifo_buffer.c \
	$(DIR_LIB_PRINTF)/src/printf.c \
	$(SRC_CMSIS) \
	$(SRC_STDPERIPH) \
	$(SRC_USBLIB) \
	$(SRC_FREERTOS) \
	$(DIR_CMSIS)/CM3/DeviceSupport/ST/STM32F10x/startup/startup_stm32f10x_hd.s


# Source files for the BOOTLOADER target
SRC_BOOTLOADER = $(DIR_BOOTLOADER)/src/main.c \
	$(DIR_BOOTLOADER)/src/dfu_mal.c \
	$(DIR_BOOTLOADER)/src/flash_if.c \
	$(DIR_BOOTLOADER)/src/hw_config.c \
	$(DIR_BOOTLOADER)/src/usb_desc.c \
	$(DIR_BOOTLOADER)/src/usb_istr.c \
	$(DIR_BOOTLOADER)/src/usb_prop.c \
	$(DIR_BOOTLOADER)/src/usb_pwr.c \
	$(SRC_CMSIS) \
	$(SRC_STDPERIPH) \
	$(SRC_USBLIB) \
	$(DIR_CMSIS)/CM3/DeviceSupport/ST/STM32F10x/startup/startup_stm32f10x_hd.s

# Search path for sources
# make uses VPATH as a search list for both prerequisites and targets of rules.
# VPATH		:= $(DIR_APP)/src:$(DIR_CMSIS)/CM3/DeviceSupport/ST/STM32F10x/startup

# Search path and source files for the CMSIS sources
VPATH		:= $(VPATH):$(DIR_CMSIS)/CM3/CoreSupport:$(DIR_CMSIS)/CM3/DeviceSupport/ST/STM32F10x
SRC_CMSIS	 = $(notdir $(wildcard $(DIR_CMSIS)/CM3/CoreSupport/*.c \
	$(DIR_CMSIS)/CM3/DeviceSupport/ST/STM32F10x/*.c))

# Search path and source files for the ST stdperiph library
VPATH		:= $(VPATH):$(DIR_STDPERIPH)/src
SRC_STDPERIPH	 = $(notdir $(wildcard $(DIR_STDPERIPH)/src/*.c))

# Search path and source files for the ST USB library
VPATH		:= $(VPATH):$(DIR_LIB_USB)/src
SRC_USBLIB	 = $(notdir $(wildcard $(DIR_LIB_USB)/src/*.c))

# Search path and source files for the FREERTOS library
VPATH		:= $(VPATH):$(DIR_FREERTOS)/src:$(DIR_FREERTOS)/portable/MemMang:$(DIR_FREERTOS)/portable/GCC/ARM_CM3
SRC_FREERTOS = $(notdir $(wildcard $(DIR_FREERTOS)/src/*.c \
	$(DIR_FREERTOS)/portable/MemMang/*.c \
	$(DIR_FREERTOS)/portable/GCC/ARM_CM3/*.c))

# Search path and source files for the FREERTOS library
# VPATH		:= $(VPATH):$(DIR_BOOTLOADER)/src
# SRC_BOOTLOADER = $(notdir $(wildcard $(DIR_BOOTLOADER)/src/*.c))

###############################################################################
# Things that might need changing to use different tools
#

# Tool names
CC			= arm-none-eabi-gcc
OBJCOPY		= arm-none-eabi-objcopy
SIZE		= arm-none-eabi-size

#
# Tool options.
INCLUDE_DIRS_10dM3UOP88 = $(DIR_APP)/inc \
	$(DIR_DRV)/inc \
	$(DIR_USB)/inc \
	$(DIR_OSD)/inc \
	$(DIR_LIB_USB)/inc \
	$(DIR_STDPERIPH)/inc \
	$(DIR_FREERTOS)/inc \
	$(DIR_FREERTOS)/portable/GCC/ARM_CM3/ \
	$(DIR_LIB_PRINTF)/inc \
	$(DIR_LIB_FIFO)/inc \
	$(DIR_LIB_MAVLINK)/inc \
	$(DIR_CMSIS)/CM3/CoreSupport \
	$(DIR_CMSIS)/CM3/DeviceSupport/ST/STM32F10x \

INCLUDE_DIRS_BOOTLOADER = $(DIR_BOOTLOADER)/inc \
	$(DIR_LIB_USB)/inc \
	$(DIR_STDPERIPH)/inc \
	$(DIR_CMSIS)/CM3/CoreSupport \
	$(DIR_CMSIS)/CM3/DeviceSupport/ST/STM32F10x \

ARCH_FLAGS = -mthumb -mcpu=cortex-m3 -g3 -ggdb

# Optimization flag
OPT_10dM3UOP88 = -O0
OPT_BOOTLOADER = -Os

CFLAGS = -DUSE_STDPERIPH_DRIVER \
	-DHSE_VALUE=8000000 \
	-DSTM32F10X_HD \
	-DSTM32F103RE \
	$(addprefix -D,$(OPTIONS)) \
	$(addprefix -I,$(INCLUDE_DIRS_$(TARGET))) \
	-std=gnu99 \
	$(OPT_$(TARGET)) \
	-Wall \
	-MD \
	-Wa,-adhlns="$@.lst" \
	-funsigned-bitfields \
	-c \
	-fmessage-length=0 \
	$(ARCH_FLAGS)

ASFLAGS = $(ARCH_FLAGS) \
	-x assembler-with-cpp \
	$(addprefix -I,$(INCLUDE_DIRS_$(TARGET)))

# XXX Map/crossref output?
LD_SCRIPT_10dM3UOP88 = $(ROOT)/stm32_flash_fw.ld
LD_SCRIPT_BOOTLOADER = $(ROOT)/stm32_flash_bl.ld

LDFLAGS = -lm \
	-static \
	-Xlinker \
	--gc-sections \
	-Wl,-Map,$(TARGET_MAP) \
	-T$(LD_SCRIPT_$(TARGET)) \
	$(ARCH_FLAGS)

###############################################################################
# No user-serviceable parts below
###############################################################################

#
# Things we will build
#
ifeq ($(filter $(TARGET),$(VALID_TARGETS)),)
$(error Target '$(TARGET)' is not valid, must be one of $(VALID_TARGETS))
endif

TARGET_MAP	 = $(BIN_DIR)/$(TARGET).map
TARGET_HEX	 = $(BIN_DIR)/$(TARGET).hex
TARGET_ELF	 = $(BIN_DIR)/$(TARGET).elf
TARGET_OBJS	 = $(addsuffix .o,$(addprefix $(OBJECT_DIR)/$(TARGET)/,$(basename $(SRC_$(TARGET)))))

# List of buildable ELF files and their object dependencies.
# It would be nice to compute these lists, but that seems to be just beyond make.

# Generate HEX and print Size
$(TARGET_HEX): $(TARGET_ELF)
	@echo HEX $(notdir $<)
	@$(OBJCOPY) -O ihex $< $@
	@echo '---- Size --------------------------------------'
	@$(SIZE) --format=berkeley $(TARGET_ELF)

# Link
$(TARGET_ELF):  $(TARGET_OBJS)
	@mkdir -p $(dir $@)
	@echo LD $(notdir $<)
	@$(CC) -o $@ $^ $(LDFLAGS)

# Compile
$(OBJECT_DIR)/$(TARGET)/%.o: %.c
	@mkdir -p $(dir $@)
	@echo CC $(notdir $<)
	@$(CC) -c -o $@ $(CFLAGS) $<

# Assemble
$(OBJECT_DIR)/$(TARGET)/%.o: %.s
	@mkdir -p $(dir $@)
	@echo AS $(notdir $<)
	@$(CC) -c -o $@ $(ASFLAGS) $< 
$(OBJECT_DIR)/$(TARGET)/%.o: %.S
	@mkdir -p $(dir $@)
	@echo AS $(notdir $<)
	@$(CC) -c -o $@ $(ASFLAGS) $< 

# Print size
size:
	$(SIZE) --format=berkeley $(TARGET_ELF)
	@echo 'Finished building: $@'
	@echo ' '

clean:
	@cs-rm -f $(TARGET_HEX) $(TARGET_ELF) $(TARGET_OBJS) $(TARGET_MAP)

help:
	@echo ' '
	@echo 'Makefile for the 10dM3UOP88 firmware'
	@echo ' '
	@echo 'Usage:'
	@echo '    make [TARGET=<target>] [OPTIONS=<options>]'
	@echo ' '
	@echo 'Valid TARGET values are: $(VALID_TARGETS)'

-include $(TARGET_OBJS:.o=.d)
