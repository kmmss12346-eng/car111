#!/usr/bin/env bash
# (10-23 用户"一键启动是一个物理的上电开关")装开机自启动：树莓派开机进桌面以后开一个终端运行 boot_race.sh。
#   bash install_boot_race.sh       装(只要装一次)
#   bash install_boot_race.sh off   卸掉
# 装了以后每次开机都会开一个终端：配置里 boot_race 没打开(bootrace 1 / bootrace 2)程序马上退出，调试照常手动 bash run_live.sh。
# 树莓派要设成开机自动登录桌面(sudo raspi-config -> System Options -> Boot / Auto Login -> Desktop Autologin)。
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
DIR="$(pwd)"
F="$HOME/.config/autostart/car_boot_race.desktop"
if [[ "${1:-}" == "off" ]]; then
    rm -f "$F"
    echo "已卸掉开机自启动：$F"
    exit 0
fi
chmod +x boot_race.sh
mkdir -p "$HOME/.config/autostart"
if command -v lxterminal >/dev/null 2>&1; then
    T="lxterminal --title=boot_race -e"
else
    T="x-terminal-emulator -e"
fi
cat > "$F" <<DESK
[Desktop Entry]
Type=Application
Name=car boot race
Exec=$T $DIR/boot_race.sh
X-GNOME-Autostart-enabled=true
DESK
echo "已装好开机自启动：$F"
echo "以后开机进桌面会开一个终端运行 $DIR/boot_race.sh。"
echo "要比赛：开程序输入 bootrace 1 或 bootrace 2，q 退出，关电；裁判说开始时打开电闸，开机以后自己开跑。"
