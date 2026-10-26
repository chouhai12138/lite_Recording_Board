# =============================================================================
#  STM32F407VET6 工程构建脚本（GCC + Make）
#
#  常用命令：
#    make                 编译，产出 build/firmware.elf/.hex/.bin + 占用统计
#    make size            只看 Flash/RAM 占用
#    make flash           编译并用 OpenOCD 烧写（自动识别 DAPLink / ST-Link）
#    make erase           全片擦除
#    make reset           复位芯片
#    make debug           启动 OpenOCD GDB Server（另一个终端再 make gdb）
#    make gdb             连接 GDB
#    make monitor         串口终端（自动挑 /dev/ttyACM0 或 /dev/ttyUSB0，可 SERIAL_PORT= 覆盖）
#    make probe           只检测调试器是否可用
#    make test            主机侧单测（协议 + 重力，gcc 编译，不需要板子）
#    make clean           清理
#
#  调试器选择：
#    make flash PROBE=dap      只用 CMSIS-DAP / DAPLink
#    make flash PROBE=wch      只用 WCH-Link（1a86:8012，实验性，见 README 3.5）
#    make flash PROBE=stlink   只用 ST-Link
#    make flash PROBE=auto     自动（默认）
# =============================================================================

TARGET    := firmware
BUILD_DIR := build

# ---------------------------------------------------------------------------
# 工具链
# ---------------------------------------------------------------------------
PREFIX  := arm-none-eabi-
CC      := $(PREFIX)gcc
AS      := $(CC) -x assembler-with-cpp
CP      := $(PREFIX)objcopy
SZ      := $(PREFIX)size
OD      := $(PREFIX)objdump
GDB     := gdb-multiarch

# ---------------------------------------------------------------------------
# 芯片参数：STM32F407VET6 = Cortex-M4F，512KB Flash / 128KB RAM / 64KB CCM
# ---------------------------------------------------------------------------
CPU     := -mcpu=cortex-m4
FPU     := -mfpu=fpv4-sp-d16 -mfloat-abi=hard
MCU     := $(CPU) -mthumb $(FPU)

# 外部晶振频率（和 board_config.h / hal_conf 保持一致）
# 板载晶振 25MHz（实测：HSE 起振、PLL 锁定 168MHz）
HSE     := 25000000
# 时钟源：hse（默认，用外部晶振）| hsi（无晶振/晶振坏时的兜底）
CLOCK   ?= hse
# 串口波特率（WCH-Link 串口桥在 115200 下偶发丢字节，可改 57600 更稳）
UART_BAUD ?= 115200
DEFS    := -DUSE_HAL_DRIVER -DSTM32F407xx -DHSE_VALUE=$(HSE)U \
           -DUART_BAUD=$(UART_BAUD)U $(EXTRA_DEFS)
ifeq ($(CLOCK),hse)
  DEFS  += -DUSE_HSE_CLOCK
endif

# ---------------------------------------------------------------------------
# 目录
# ---------------------------------------------------------------------------
C_INCLUDES := \
  -ICore/Inc \
  -IBSP/Inc \
  -IApp/motor \
  -ISim \
  -IBSP/lcd154 \
  -IApp/ui \
  -IApp/control \
  -IApp/cfg \
  -IApp/motion \
  -IDrivers/CMSIS/Include \
  -IDrivers/CMSIS/Device/ST/STM32F4xx/Include \
  -IDrivers/STM32F4xx_HAL_Driver/Inc \
  -IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy

LDSCRIPT := Core/STM32F407VETx_FLASH.ld

# ---------------------------------------------------------------------------
# 源文件
# ---------------------------------------------------------------------------
APP_SOURCES := \
  Core/Src/main.c \
  Core/Src/stm32f4xx_it.c \
  Core/Src/syscalls.c \
  BSP/Src/bsp_can.c \
  BSP/Src/bsp_w25q.c \
  BSP/Src/bsp_lcd.c \
  BSP/lcd154/lcd_spi_154.c \
  BSP/lcd154/lcd_fonts.c \
  BSP/lcd154/lcd_image.c \
  BSP/Src/bsp_led.c \
  BSP/Src/bsp_keys.c \
  BSP/Src/bsp_uart.c \
  App/motor/rs_protocol.c \
  App/motor/can_bus.c \
  App/motor/motor_mgr.c \
  Sim/sim_can.c \
  App/ui/ui_core.c \
  App/ui/ui_text.c \
  App/ui/ui_lcd.c \
  App/ui/zh_font.c \
  App/control/mit_ctrl.c \
  App/control/gravity.c \
  App/motion/traj.c

# 需要哪个 HAL 模块就在这里加对应的 .c（用不到的不编译，省时间）
HAL_DIR := Drivers/STM32F4xx_HAL_Driver/Src
HAL_SOURCES := \
  $(HAL_DIR)/stm32f4xx_hal.c \
  $(HAL_DIR)/stm32f4xx_hal_cortex.c \
  $(HAL_DIR)/stm32f4xx_hal_rcc.c \
  $(HAL_DIR)/stm32f4xx_hal_rcc_ex.c \
  $(HAL_DIR)/stm32f4xx_hal_gpio.c \
  $(HAL_DIR)/stm32f4xx_hal_dma.c \
  $(HAL_DIR)/stm32f4xx_hal_dma_ex.c \
  $(HAL_DIR)/stm32f4xx_hal_pwr.c \
  $(HAL_DIR)/stm32f4xx_hal_pwr_ex.c \
  $(HAL_DIR)/stm32f4xx_hal_flash.c \
  $(HAL_DIR)/stm32f4xx_hal_flash_ex.c \
  $(HAL_DIR)/stm32f4xx_hal_flash_ramfunc.c \
  $(HAL_DIR)/stm32f4xx_hal_exti.c \
  $(HAL_DIR)/stm32f4xx_hal_uart.c \
  $(HAL_DIR)/stm32f4xx_hal_usart.c \
  $(HAL_DIR)/stm32f4xx_hal_tim.c \
  $(HAL_DIR)/stm32f4xx_hal_tim_ex.c \
  $(HAL_DIR)/stm32f4xx_hal_spi.c \
  $(HAL_DIR)/stm32f4xx_hal_i2s.c \
  $(HAL_DIR)/stm32f4xx_hal_sai.c \
  $(HAL_DIR)/stm32f4xx_hal_can.c \
  $(HAL_DIR)/stm32f4xx_hal_adc.c \
  $(HAL_DIR)/stm32f4xx_hal_sd.c \
  $(HAL_DIR)/stm32f4xx_hal_pcd.c \
  $(HAL_DIR)/stm32f4xx_hal_rtc.c

# 想一次性编译全部 HAL 模块？取消下面这行注释（慢，但省事）
# HAL_SOURCES := $(wildcard $(HAL_DIR)/*.c)

SYSTEM_SOURCES := \
  Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/system_stm32f4xx.c

ASM_SOURCES := \
  Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc/startup_stm32f407xx.s

C_SOURCES := $(APP_SOURCES) $(HAL_SOURCES) $(SYSTEM_SOURCES)

# ---------------------------------------------------------------------------
# 编译选项
# ---------------------------------------------------------------------------
OPT     := -Og
DEBUG   := -g3 -gdwarf-2

CFLAGS := $(MCU) $(DEFS) $(C_INCLUDES) $(OPT) $(DEBUG) \
  -Wall -Wextra -Wno-unused-parameter \
  -fdata-sections -ffunction-sections \
  -fno-common -std=gnu11

ASFLAGS := $(MCU) $(OPT) $(DEBUG) -Wall -fdata-sections -ffunction-sections

# 注意：newlib-nano 默认不支持 printf 浮点（%.1f 会输出空）。
# 需要浮点打印时再加 -u _printf_float（Flash 会多十几 KB）。
LDFLAGS := $(MCU) \
  -specs=nano.specs -specs=nosys.specs \
  -Wl,-u,_printf_float -Wl,-u,_scanf_float \
  -T$(LDSCRIPT) \
  -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref \
  -Wl,--gc-sections \
  -Wl,--print-memory-usage \
  -static

LIBS := -lc -lm

# ---------------------------------------------------------------------------
# 构建规则
# ---------------------------------------------------------------------------
OBJECTS  := $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin
	@$(MAKE) --no-print-directory size

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	@echo "  CC      $<"
	@$(CC) -c $(CFLAGS) -MMD -MP -MF"$(@:%.o=%.d)" $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	@echo "  AS      $<"
	@$(AS) -c $(ASFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) $(LDSCRIPT) Makefile
	@echo "  LD      $@"
	@$(CC) $(OBJECTS) $(LDFLAGS) $(LIBS) -o $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	@echo "  HEX     $@"
	@$(CP) -O ihex $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	@echo "  BIN     $@"
	@$(CP) -O binary -S $< $@

$(BUILD_DIR):
	@mkdir -p $@

size: $(BUILD_DIR)/$(TARGET).elf
	@$(SZ) $<

# ---------------------------------------------------------------------------
# 烧写 / 调试（真正的探测逻辑在 tools/flash.sh）
# ---------------------------------------------------------------------------
PROBE       ?= auto
# 串口：不指定时自动挑（WCH-Link 自带 CDC = ttyACM0，USB-TTL = ttyUSB0）
SERIAL_PORT ?= $(shell ls /dev/ttyACM0 /dev/ttyUSB0 2>/dev/null | head -1)
TOOLS       := tools

flash: $(BUILD_DIR)/$(TARGET).elf
	@$(TOOLS)/flash.sh $(PROBE) $(BUILD_DIR)/$(TARGET).elf

flash-bin: $(BUILD_DIR)/$(TARGET).bin
	@$(TOOLS)/flash.sh $(PROBE) $(BUILD_DIR)/$(TARGET).bin

erase:
	@$(TOOLS)/flash.sh $(PROBE) --erase

reset:
	@$(TOOLS)/flash.sh $(PROBE) --reset

probe:
	@$(TOOLS)/flash.sh $(PROBE) --probe-only

chipid:
	@$(TOOLS)/flash.sh $(PROBE) --chip-id

debug: $(BUILD_DIR)/$(TARGET).elf
	@$(TOOLS)/flash.sh $(PROBE) --gdb-server

gdb: $(BUILD_DIR)/$(TARGET).elf
	$(GDB) -q $(BUILD_DIR)/$(TARGET).elf -x $(TOOLS)/gdbinit

BAUD ?= 115200

# ---------------------------------------------------------------------------
# 主机侧单测（不需要板子）
# ---------------------------------------------------------------------------
HOSTCC ?= cc

test:
	@echo "  TEST    Tests/test_protocol.c (host gcc)"
	@$(HOSTCC) -std=c11 -Wall -Wextra -O2 -IApp/motor \
	  Tests/test_protocol.c App/motor/rs_protocol.c -lm -o $(BUILD_DIR)/test_protocol
	@$(BUILD_DIR)/test_protocol
	@echo "  TEST    Tests/test_gravity.c (host gcc)"
	@$(HOSTCC) -std=c11 -Wall -Wextra -O2 -IApp/control -IApp/cfg \
	  Tests/test_gravity.c App/control/gravity.c -lm -o $(BUILD_DIR)/test_gravity
	@$(BUILD_DIR)/test_gravity

monitor:
	@port="$(SERIAL_PORT)"; \
	if [ -z "$$port" ]; then \
	  echo "找不到串口设备，请插好 USB 后重试： ls /dev/ttyACM* /dev/ttyUSB*"; \
	  exit 1; \
	fi; \
	echo "打开 $$port @ $(BAUD)，退出按 Ctrl+A 再 Ctrl+X"; \
	picocom -b $(BAUD) "$$port"

# 烧写后直接打开串口看输出
run: flash
	@sleep 1
	@$(MAKE) --no-print-directory monitor

disasm: $(BUILD_DIR)/$(TARGET).elf
	@$(OD) -d -S $< | less

clean:
	@rm -rf $(BUILD_DIR)
	@echo "cleaned."

.PHONY: all size flash flash-bin erase reset probe chipid debug gdb monitor run disasm test clean

-include $(wildcard $(BUILD_DIR)/*.d)
