#!/usr/bin/env bash
# =============================================================================
#  STM32F407 烧写 / 调试辅助脚本（OpenOCD 前端）
#
#  用法：
#    tools/flash.sh [auto|dap|wch|stlink|jlink] <firmware.elf|.hex|.bin>
#    tools/flash.sh [auto|dap|wch|stlink|jlink] --erase
#    tools/flash.sh [auto|dap|wch|stlink|jlink] --reset
#    tools/flash.sh [auto|dap|wch|stlink|jlink] --probe-only
#    tools/flash.sh [auto|dap|wch|stlink|jlink] --chip-id
#    tools/flash.sh [auto|dap|wch|stlink|jlink] --gdb-server
#
#  调试器：
#    dap    DAPLink / CMSIS-DAP
#    wch    WCH-Link (1a86:8012)，按 CMSIS-DAP v2 over bulk 使用
#    stlink ST-Link
#    jlink  J-Link
#    auto   自动（按 lsusb 猜测 + 逐个真连测试）
# =============================================================================
set -uo pipefail

TOOLS_DIR="$(cd "$(dirname "$0")" && pwd)"
OCD_TARGET="target/stm32f4x.cfg"
OCD_SPEED="${OCD_SPEED:-1000}"
OPENOCD_BIN="${OPENOCD:-openocd}"
LAST_LOG=""
OcdLog=""

if [ -t 1 ]; then
    C_RED=$'\033[31m'; C_GRN=$'\033[32m'; C_YEL=$'\033[33m'
    C_CYN=$'\033[36m'; C_RST=$'\033[0m'
else
    C_RED=""; C_GRN=""; C_YEL=""; C_CYN=""; C_RST=""
fi

info() { printf "%s[flash]%s %s\n" "$C_CYN" "$C_RST" "$*"; }
warn() { printf "%s[flash]%s %s\n" "$C_YEL" "$C_RST" "$*" >&2; }
err()  { printf "%s[flash]%s %s\n" "$C_RED" "$C_RST" "$*" >&2; }

# ---------- 参数 ----------
PROBE="auto"
ACTION=""
TARGET_FILE=""

for arg in "$@"; do
    case "$arg" in
        auto|dap|wch|stlink|jlink) PROBE="$arg" ;;
        --erase|--reset|--probe-only|--chip-id|--gdb-server) ACTION="$arg" ;;
        -*) err "未知选项: $arg"; exit 1 ;;
        *)
            if [ -n "$TARGET_FILE" ]; then
                err "多余参数: $arg"
                err "（提示：命令后面不要带说明文字，例如只写 tools/flash.sh auto build/firmware.elf）"
                err "用法: $0 [auto|dap|wch|stlink|jlink] <firmware.elf|.hex|.bin|--erase|--reset|--probe-only|--chip-id|--gdb-server>"
                exit 1
            fi
            TARGET_FILE="$arg" ;;
    esac
done

if [ -z "$ACTION" ] && [ -z "$TARGET_FILE" ]; then
    err "用法: $0 [auto|dap|wch|stlink|jlink] <firmware.elf|.hex|.bin|--erase|--reset|--probe-only|--chip-id|--gdb-server>"
    exit 1
fi

probe_cfg() {
    case "$1" in
        dap)    echo "interface/cmsis-dap.cfg" ;;
        wch)    echo "$TOOLS_DIR/wchlink.cfg" ;;
        stlink) echo "interface/stlink.cfg" ;;
        jlink)  echo "interface/jlink.cfg" ;;
    esac
}

# 按 USB ID 猜测调试器
detect_by_usb() {
    local usb
    usb="$(lsusb 2>/dev/null || true)"
    [ -z "$usb" ] && return 0

    if echo "$usb" | grep -qiE "(c251|0d28):[0-9a-f]{4}|1209:0000|2e8a:0004"; then
        echo "dap"; return 0
    fi
    if echo "$usb" | grep -qiE "0483:[0-9a-f]{4}"; then
        echo "stlink"; return 0
    fi
    if echo "$usb" | grep -qiE "1366:[0-9a-f]{4}"; then
        echo "jlink"; return 0
    fi
    if echo "$usb" | grep -qiE "1a86:8012"; then
        echo "wch"; return 0
    fi
    return 0
}

is_usb_error() { grep -qE "error reading data|Input/Output Error|LIBUSB_ERROR_IO|LIBUSB_ERROR_PIPE|EPIPE" "$1"; }
is_perm_error() { grep -qiE "LIBUSB_ERROR_ACCESS|Access denied|Permission denied|could not open|unable to open" "$1"; }

# 带重试地跑一次 openocd；成功时 OcdLog 指向日志
ocd_run() {
    local cfg="$1"; shift
    local log rc attempt

    for attempt in 1 2 3; do
        log="$(mktemp /tmp/dsh-ocd.XXXXXX)"
        LAST_LOG="$log"
        timeout 180 "$OPENOCD_BIN" -f "$cfg" -f "$OCD_TARGET" \
            -c "adapter speed $OCD_SPEED" "$@" >"$log" 2>&1
        rc=$?

        if [ $rc -eq 0 ]; then
            OcdLog="$log"
            return 0
        fi

        if is_usb_error "$log" && [ $attempt -lt 3 ]; then
            warn "调试器 USB 通信异常，重试中（$attempt/3）…"
            sleep 2
            continue
        fi
        return $rc
    done
}

# 验证某个调试器配置真能连上目标
probe_works() {
    local cfg="$1" rc
    ocd_run "$cfg" -c "init" -c "halt" -c "shutdown"
    rc=$?

    if [ $rc -ne 0 ]; then
        if is_perm_error "$LAST_LOG"; then
            err "USB 权限不足，打不开调试器！装一次 udev 规则即可："
            err "  tools/install-udev.sh"
            exit 3
        fi
        return $rc
    fi
    return 0
}

choose_probe() {
    local cands=() p

    case "$PROBE" in
        dap)    cands=(dap) ;;
        wch)    cands=(wch) ;;
        stlink) cands=(stlink) ;;
        jlink)  cands=(jlink) ;;
        auto)
            local guess
            guess="$(detect_by_usb)"
            [ -n "$guess" ] && cands=("$guess")
            cands+=("dap" "wch" "stlink" "jlink")
            ;;
    esac

    local uniq=()
    for p in "${cands[@]}"; do
        [[ " ${uniq[*]-} " == *" $p "* ]] || uniq+=("$p")
    done

    if [ "${#uniq[@]}" -eq 0 ]; then
        err "没有找到调试器"
        exit 2
    fi

    for p in "${uniq[@]}"; do
        local cfg; cfg="$(probe_cfg "$p")"
        info "尝试调试器: $p ($cfg)"
        if probe_works "$cfg"; then
            PROBE_CFG="$cfg"; PROBE_NAME="$p"
            return 0
        fi
    done

    return 1
}

list_hardware_hint() {
    local usb; usb="$(lsusb 2>/dev/null || true)"

    if [ -n "${LAST_LOG:-}" ] && [ -f "$LAST_LOG" ] && is_usb_error "$LAST_LOG"; then
        err "调试器 USB 通道卡死（WCH-Link 固件的常见毛病）。处理："
        err "  1) 拔插一次 WCH-Link，让它重新枚举"
        err "  2) 再跑一次 make probe"
        err "  3) 仍不行就换 USB 线 / 换 USB 口（别接 hub）"
        err "OpenOCD 最后输出："
        tail -8 "$LAST_LOG" | sed 's/^/    /' >&2
        return 0
    fi

    if printf '%s\n' "$usb" | grep -qiE "1a86:8012"; then
        err "检测到 WCH-Link (1a86:8012)，但没连上目标板。排查："
        err "  · SWDIO / SWCLK / GND / 3V3 是否接对、目标板是否供电"
        err "  · 建议接上 NRST（tools/wchlink.cfg 用了 connect_assert_srst）"
        err "  · 目标是否被别的程序占用（另一个 openocd / Keil）"
    else
        err "没有找到可用的调试器（DAPLink / WCH-Link / ST-Link / J-Link）"
    fi

    if [ -n "${LAST_LOG:-}" ] && [ -f "$LAST_LOG" ]; then
        echo >&2
        info "OpenOCD 最后输出：" >&2
        tail -12 "$LAST_LOG" | sed 's/^/    /' >&2
    fi
}

# ---------- 执行动作 ----------
main() {
    command -v "$OPENOCD_BIN" >/dev/null 2>&1 || {
        err "找不到 openocd，请先安装：sudo apt install openocd"
        exit 1
    }

    if [ "$ACTION" = "--probe-only" ]; then
        if choose_probe; then
            echo
            info "调试器可用: ${C_GRN}$PROBE_CFG${C_RST}"
            exit 0
        fi
        list_hardware_hint
        exit 2
    fi

    if ! choose_probe; then
        err "没有可用的调试器。"
        list_hardware_hint
        exit 2
    fi
    info "使用调试器: $PROBE_NAME ($PROBE_CFG)"

    local rc=0
    case "$ACTION" in
        --erase)
            info "全片擦除..."
            ocd_run "$PROBE_CFG" -c "init" -c "reset halt" \
                -c "stm32f4x mass_erase 0" -c "shutdown" || rc=$?
            ;;
        --reset)
            info "复位芯片..."
            ocd_run "$PROBE_CFG" -c "init" -c "reset run" -c "shutdown" || rc=$?
            ;;
        --chip-id)
            info "读取芯片 ID（DBGMCU_IDCODE @ 0xE0042000）..."
            ocd_run "$PROBE_CFG" -c "init" -c "reset halt" \
                -c "mdw 0xE0042000 1" -c "flash probe 0" -c "shutdown" || rc=$?
            [ -n "$OcdLog" ] && grep -E "device id|flash size|0xe0042000" "$OcdLog" | sed 's/^/    /'
            ;;
        --gdb-server)
            echo
            info "OpenOCD GDB Server 启动中（telnet 4444 / gdb 3333），Ctrl+C 结束"
            echo
            "$OPENOCD_BIN" -f "$PROBE_CFG" -f "$OCD_TARGET" \
                -c "adapter speed $OCD_SPEED" -c "init" -c "reset halt"
            ;;
        *)
            [ -f "$TARGET_FILE" ] || { err "文件不存在: $TARGET_FILE"; exit 1; }
            local abs
            abs="$(realpath "$TARGET_FILE")"
            info "烧写: $TARGET_FILE"

            if [[ "$TARGET_FILE" == *.bin ]]; then
                ocd_run "$PROBE_CFG" -c "init" -c "reset halt" \
                    -c "program \"$abs\" 0x08000000 verify reset exit" || rc=$?
            else
                ocd_run "$PROBE_CFG" -c "init" -c "reset halt" \
                    -c "program \"$abs\" verify reset exit" || rc=$?
            fi
            if [ $rc -eq 0 ]; then
                [ -n "$OcdLog" ] && grep -E "Verified|Programming" "$OcdLog" | sed 's/^/    /'
                info "${C_GRN}烧写完成${C_RST}（已校验并复位运行）"
            fi
            ;;
    esac

    if [ $rc -ne 0 ]; then
        err "操作失败（exit=$rc）"
        [ -n "$OcdLog" ] && [ -f "$OcdLog" ] && tail -15 "$OcdLog" | sed 's/^/    /' >&2
        list_hardware_hint
        exit $rc
    fi
}

main
