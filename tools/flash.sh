#!/usr/bin/env bash
# 通过 CMSIS-DAP（含 Horco 无线 DAPLink）烧录 MC02 并复位。
#
# 用法：tools/flash.sh [elf路径]      默认 bsp/mc02/build/Debug/MC02.elf
#       tools/flash.sh --backup       只备份整片 Flash 到 backup/
#
# 关键点：必须用 connect_mode=attach。
#   pyocd 内置的 stm32h723xx 目标默认走 under-reset 连接，会断言硬件 nRST 已拉低；
#   MC02 的 SWD 口只有 VCC/GND/SWCLK/SWDIO 四针，没有 nRST，所以默认模式必然失败。
# 无线 DAPLink 是半双工链路，速率约 11 kB/s，整片读取约 40 s，烧录 112 KB 约 11 s。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYOCD="${PYOCD:-$HOME/.local/bin/pyocd}"
OPTS=(-t stm32h723xx -f 1000000 -O connect_mode=attach)
run() { env -u PYTHONPATH "$PYOCD" "$@"; }   # ROS 的 PYTHONPATH 会污染 pyocd 的 venv

if [ "${1:-}" = "--backup" ]; then
  mkdir -p "$ROOT/backup"
  out="$ROOT/backup/mc02_flash_$(date +%Y%m%d_%H%M%S).bin"
  run commander "${OPTS[@]}" -c "savemem 0x08000000 0x100000 $out"
  echo "已备份到 $out"
  exit 0
fi

ELF="${1:-$ROOT/bsp/mc02/build/Debug/MC02.elf}"
[ -f "$ELF" ] || { echo "找不到 $ELF，请先编译" >&2; exit 1; }
run list | grep -q "CMSIS-DAP" || { echo "没有检测到 CMSIS-DAP 探针（检查 udev 规则和连接）" >&2; exit 1; }

run flash "${OPTS[@]}" "$ELF"
run reset "${OPTS[@]}"
echo "烧录完成并已复位。USB CDC 终端：lsusb | grep 0483:5740"
