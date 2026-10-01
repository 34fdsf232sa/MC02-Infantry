#!/usr/bin/env bash
# 重新生成 MC02 BSP：CubeMX 代码 + LibXR app_main。
#
# 改外设配置的流程：
#   1. 用 CubeMX GUI 打开 MC02.ioc 修改并保存（提示升级固件包时选 Continue，保持 FW_H7 V1.11.1）
#   2. 运行本脚本
#   3. git diff 检查后提交
#
# 依赖：~/STM32CubeMX（6.18.1）、~/.local/share/libxr-venv（pip libxr 5.2.4）
# 注意：运行前关闭 CubeMX GUI，否则 CLI 实例拿不到 Updater 锁会卡住。
set -euo pipefail

BSP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CUBEMX="${CUBEMX:-$HOME/STM32CubeMX/STM32CubeMX}"
XR_BIN="${XR_BIN:-$HOME/.local/share/libxr-venv/bin}"

if pgrep -u "$USER" -f "jre/bin/java -jar .*STM32CubeMX" >/dev/null; then
  echo "STM32CubeMX GUI 正在运行，请先关闭" >&2
  exit 1
fi

# 1. LibXR 要求的 .ioc 调整（幂等）
python3 "$BSP_DIR/tools/ioc_libxr_patch.py" "$BSP_DIR/MC02.ioc" "$BSP_DIR/MC02.ioc"

# 2. CubeMX 生成（CubeMX 会切换工作目录，脚本路径必须是绝对路径）
script="$(mktemp --suffix=.cubemx)"
trap 'rm -f "$script"' EXIT
printf 'config load %s/MC02.ioc\nproject generate\nexit\n' "$BSP_DIR" > "$script"
log="$(mktemp --suffix=.log)"
timeout 900 "$CUBEMX" -q "$script" > "$log" 2>&1
if [ "$(grep -cxE 'OK' "$log")" -lt 2 ]; then
  echo "CubeMX 生成失败，日志：$log" >&2
  exit 1
fi

# 3. LibXR：解析 .ioc -> 生成 User/app_main.cpp（保留 User Code 段和 libxr_config.yaml）
#    LibXR.CMake 已手工指向 third_party/libxr，xr_stm32_cmake 只会规范化标准/系统设置，不改该路径
export PATH="$XR_BIN:$PATH"
cd "$BSP_DIR"
env -u PYTHONPATH xr_parse_ioc -d . -o .config.yaml
env -u PYTHONPATH xr_gen_code_stm32 -i .config.yaml -o User/app_main.cpp
env -u PYTHONPATH xr_stm32_cmake .

# 4. 检查 freertos.c 里的 app_main 钩子（位于 USER CODE 段，CubeMX 应保留）
grep -q "app_main();" Core/Src/freertos.c || { echo "freertos.c 缺少 app_main() 调用" >&2; exit 1; }
echo "重新生成完成，请 git diff 检查"
