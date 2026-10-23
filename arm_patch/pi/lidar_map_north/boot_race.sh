#!/usr/bin/env bash
# (10-23 用户"一键启动是一个物理的上电开关、整车的电闸")开机自启动时运行(install_boot_race.sh 装的)：
# 配置里 boot_race=true(终端 bootrace 1 / bootrace 2 打开)才按那个启停区直接开跑，否则程序马上退出。窗口留着看日志。
cd -- "$(dirname -- "${BASH_SOURCE[0]}")" || exit 1
bash run_live.sh --boot
echo
echo '程序已退出。按回车关掉这个窗口。'
read -r _
