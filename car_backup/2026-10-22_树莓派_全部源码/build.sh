#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
if ! command -v g++ >/dev/null 2>&1; then
    echo '缺少g++。请执行: sudo apt install g++'
    exit 1
fi
g++ -std=c++11 -O2 -pthread -include pthread.h -I vendor/ldlidar_driver/include \
    scan_bridge.cpp vendor/ldlidar_driver/src/*.cpp -o scan_bridge
echo '编译完成。执行: python3 ld14p_scan.py --list'
