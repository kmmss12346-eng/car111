#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
if ! python3 -c 'import numpy, matplotlib' >/dev/null 2>&1; then
    echo '缺少依赖，请执行: sudo apt install python3-numpy python3-matplotlib g++'
    exit 1
fi
if [[ ! -x scan_bridge ]]; then
    bash build.sh
fi
exec python3 map_merge_live.py --port /dev/ttyACM0 "$@"
