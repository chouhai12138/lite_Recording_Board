#!/usr/bin/env bash
# =============================================================================
#  环境自检：编译链 / 调试器 / 串口
#  用法：tools/check-env.sh
# =============================================================================
set -uo pipefail
cd "$(dirname "$0")/.." || exit 1

ok()   { printf "  \033[32m✔\033[0m %s\n" "$*"; }
bad()  { printf "  \033[31m✘\033[0m %s\n" "$*"; }
warn() { printf "  \033[33m!\033[0m %s\n" "$*"; }

echo "================ 主机 ================"
echo "  $(uname -srm)"
echo "  $(lsb_release -ds 2>/dev/null || cat /etc/os-release 2>/dev/null | grep PRETTY_NAME | cut -d= -f2)"

echo "================ 工具链 ================"
for t in arm-none-eabi-gcc arm-none-eabi-objcopy arm-none-eabi-size make openocd gdb-multiarch lsusb; do
    if p=$(command -v "$t" 2>/dev/null); then
        ok "$t -> $p"
    else
        bad "$t 未安装"
    fi
done
if command -v arm-none-eabi-gcc >/dev/null 2>&1; then
    echo "     $(arm-none-eabi-gcc --version | head -1)"
fi
if command -v openocd >/dev/null 2>&1; then
    echo "     $(openocd --version 2>&1 | head -1)"
    ocd_adapters="$(openocd -c "adapter list" 2>&1 || true)"
    if printf '%s\n' "$ocd_adapters" | grep -q "cmsis-dap"; then
        ok "OpenOCD 支持 CMSIS-DAP（DAPLink 可用）"
    else
        bad "OpenOCD 未编译 CMSIS-DAP 支持"
    fi
fi

echo "================ USB 调试器 ================"
if command -v lsusb >/dev/null 2>&1; then
    if lsusb | grep -qiE "(c251|0d28):[0-9a-f]{4}|1209:0000|2e8a:0004"; then
        ok "发现 DAPLink / CMSIS-DAP"
    elif lsusb | grep -qiE "0483:[0-9a-f]{4}"; then
        ok "发现 ST-Link"
    elif lsusb | grep -qiE "1366:[0-9a-f]{4}"; then
        ok "发现 J-Link"
    else
        warn "没有发现标准调试器"
    fi
    if lsusb | grep -qi "1a86:8012"; then
        ok "发现 WCH-Link（1a86:8012）——CMSIS-DAP v2 可用（tools/wchlink.cfg）"
        warn "  前提：tools/install-udev.sh + NRST 接线；偶发 USB 卡死时拔插一次"
    fi
    echo "  当前 USB 设备："
    lsusb | sed 's/^/    /'
fi

echo "================ 工程 ================"
[ -f Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h ] && ok "CMSIS 器件头文件" || bad "缺少 CMSIS 器件头文件"
[ -f Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc/startup_stm32f407xx.s ] && ok "GCC 启动文件" || bad "缺少启动文件"
[ -f Core/STM32F407VETx_FLASH.ld ] && ok "链接脚本 (512KB Flash / 128KB RAM)" || bad "缺少链接脚本"
[ -x tools/flash.sh ] && ok "烧写脚本" || warn "tools/flash.sh 没有可执行权限：chmod +x tools/flash.sh"

echo "================ 串口 ================"
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | sed 's/^/  /' || warn "没有可见的串口设备"

echo
echo "编译：make          烧写：make flash"
