# Path to GNU ARM Toolchain, leave empty if already in system PATH
TOOLCHAIN_ROOT =
# Path to the STM32Cube software package
VENDOR_ROOT = C:/STM32/STM32Cube/

###############################################################################
# Project specific
TARGET = main.elf
SRC_DIR = src/
INC_DIR = inc/
# Toolchain
CC = $(TOOLCHAIN_ROOT)arm-none-eabi-gcc
DB = $(TOOLCHAIN_ROOT)arm-none-eabi-gdb
# Project sources
SRC_FILES = $(wildcard $(SRC_DIR)*.c) $(wildcard $(SRC_DIR)*/*.c)
ASM_FILES = $(wildcard $(SRC_DIR)*.s) $(wildcard $(SRC_DIR)*/*.s)
# Update script name and path if necessary
LD_SCRIPT = STM32F429ZITx_FLASH.ld
# Project includes
INCLUDES = -I$(INC_DIR)
# Vendor sources: Note that files in "Templates" are normally copied into project for customization,
# but that is not necessary for this simple project.
ASM_FILES += $(VENDOR_ROOT)Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc/startup_stm32f429xx.s
SRC_FILES += $(VENDOR_ROOT)Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/system_stm32f4xx.c
SRC_FILES += $(VENDOR_ROOT)Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_spi.c
# STM32Cube HAL Drivers
SRC_FILES += $(VENDOR_ROOT)Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
SRC_FILES += $(VENDOR_ROOT)Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
SRC_FILES += $(VENDOR_ROOT)Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
SRC_FILES += $(VENDOR_ROOT)Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
SRC_FILES += $(VENDOR_ROOT)Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
SRC_FILES += $(VENDOR_ROOT)Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma2d.c
SRC_FILES += $(VENDOR_ROOT)Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
SRC_FILES += $(VENDOR_ROOT)Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_ltdc.c
SRC_FILES += $(VENDOR_ROOT)Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
SRC_FILES += $(VENDOR_ROOT)Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_sdram.c
SRC_FILES += $(VENDOR_ROOT)Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c
SRC_FILES += $(VENDOR_ROOT)Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_fmc.c
SRC_FILES += $(VENDOR_ROOT)Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c
SRC_FILES += $(VENDOR_ROOT)Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_tim.c
SRC_FILES += $(VENDOR_ROOT)Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_rcc.c
# STM32Cube Board Support package for LCD
SRC_FILES += $(VENDOR_ROOT)Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery.c
SRC_FILES += $(VENDOR_ROOT)Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery_lcd.c
SRC_FILES += $(VENDOR_ROOT)Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery_sdram.c
SRC_FILES += $(VENDOR_ROOT)Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery_gyroscope.c
SRC_FILES += $(VENDOR_ROOT)Drivers/BSP/Components/ili9341/ili9341.c
SRC_FILES += $(VENDOR_ROOT)Drivers/BSP/Components/l3gd20/l3gd20.c
# Vendor includes
INCLUDES += -I$(VENDOR_ROOT)Drivers/CMSIS/DSP/Include
INCLUDES += -I$(VENDOR_ROOT)Drivers/CMSIS/Core/Include
INCLUDES += -I$(VENDOR_ROOT)Drivers/CMSIS/Device/ST/STM32F4xx/Include
INCLUDES += -I$(VENDOR_ROOT)Drivers/STM32F4xx_HAL_Driver/Inc
INCLUDES += -I$(VENDOR_ROOT)Drivers/BSP/STM32F429I-Discovery
# Compiler Flags
CFLAGS = -g -O0 -Wall -Wextra -Warray-bounds -Wno-unused-parameter
#CFLAGS += -mcpu=cortex-m4 -mthumb -mlittle-endian -mthumb-interwork <- for STM32F401
#CFLAGS += -mthumb -march=armv7-m <- for STM32F429
#CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16 <- for STM32F401
CFLAGS += -mcpu=cortex-m4 -mthumb -mlittle-endian -mthumb-interwork
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -DSTM32F429xx
CFLAGS += -DARM_MATH_CM4
CFLAGS += -DUSE_FULL_LL_DRIVER
CFLAGS += $(INCLUDES)
# Linker Flags
LFLAGS = -Wl,--gc-sections -Wl,-T$(LD_SCRIPT) --specs=rdimon.specs
# Static libraries
LIBS = -L$(VENDOR_ROOT)Drivers/CMSIS/Lib/GCC -larm_cortexM4lf_math -lm

###############################################################################
# This does an in-source build. An out-of-source build that places all object
# files into a build directory would be a better solution, but the goal was to
# keep this file very simple.
CXX_OBJS = $(SRC_FILES:.c=.o)
ASM_OBJS = $(ASM_FILES:.s=.o)
ALL_OBJS = $(ASM_OBJS) $(CXX_OBJS)
.PHONY: clean gdb-server_stlink gdb-server_openocd gdb-client
all: $(TARGET)
# Compile
$(CXX_OBJS): %.o: %.c
$(ASM_OBJS): %.o: %.s
$(ALL_OBJS):
	@echo "[CC] $@"
	@$(CC) $(CFLAGS) -c $< -o $@
# Link
%.elf: $(ALL_OBJS)
	@echo "[LD] $@"
	@$(CC) $(CFLAGS) $(LFLAGS) $(ALL_OBJS) $(LIBS) -o $@
# Clean
clean:
	@rm -f $(ALL_OBJS) $(TARGET)
# Debug
gdb-server_stlink:
	st-util
gdb-server_openocd:
	openocd -f ./openocd.cfg
gdb-client: $(TARGET)
	$(DB) $(TARGET)