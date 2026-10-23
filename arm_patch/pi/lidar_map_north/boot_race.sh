#!/usr/bin/env bash
# (10-23 用户"一键启动是一个物理的上电开关、整车的电闸")开机自启动时运行(install_boot_race.sh 装的)：
# 配置里 boot_race=true(终端 bootrace 1 / bootrace 2 打开)才按那个启停区直接开跑，否则程序马上退出。窗口留着看日志。
# (10-23 晚)日志同时存到 logs/boot_月日_时分秒.log(关了电也在，比赛那一轮跑得怎么样回来能看)。
cd -- "$(dirname -- "${BASH_SOURCE[0]}")" || exit 1
mkdir -p logs
LOG="logs/boot_$(date +%m%d_%H%M%S).log"
PYTHONUNBUFFERED=1 bash run_live.sh --boot 2>&1 | tee -a "$LOG"
echo
echo "程序已退出。日志存在 $(pwd)/$LOG"
echo '按回车关掉这个窗口。'
read -r _
