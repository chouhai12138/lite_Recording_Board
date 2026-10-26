#!/usr/bin/env bash
# =============================================================================
#  安装 USB 调试器 / 串口权限规则（Linux 上的“驱动”，只需执行一次）
#
#  用法：tools/install-udev.sh
#  需要 sudo 密码。
# =============================================================================
set -euo pipefail
cd "$(dirname "$0")/.."

echo "==> 复制 99-openocd.rules 到 /etc/udev/rules.d/"
sudo cp tools/99-openocd.rules /etc/udev/rules.d/99-openocd.rules

echo "==> 重新加载 udev 规则"
sudo udevadm control --reload-rules
sudo udevadm trigger

# 串口组（可选，失败也不影响 OpenOCD）
if [ -n "${USER:-}" ] && ! id -nG "$USER" | grep -qw dialout; then
    echo "==> 把 $USER 加入 dialout 组（串口 /dev/ttyACM* 用，需重新登录生效）"
    sudo usermod -aG dialout "$USER" || true
fi

echo
echo "完成。请【拔插一次】调试器，然后执行："
echo "    make probe            # 自动识别"
echo "    make probe PROBE=wch   # 只用 WCH-Link 试"
echo
echo "设备当前权限："
ls -l /dev/bus/usb/*/* 2>/dev/null | tail -5 || true
