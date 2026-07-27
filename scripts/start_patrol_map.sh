#!/usr/bin/env bash

set -Eeuo pipefail

WS="/home/nvidia/patrol_ws"

set +u
source /opt/ros/humble/setup.bash
source /home/nvidia/ros2_humble_main/install/setup.bash
source "$WS/install/setup.bash"
set -u

if [ -z "${AMAP_JS_KEY:-}" ]; then
    echo "[patrol-map] 缺少 AMAP_JS_KEY"
    exit 1
fi

if [ -z "${AMAP_SECURITY_CODE:-}" ]; then
    echo "[patrol-map] 缺少 AMAP_SECURITY_CODE"
    exit 1
fi

exec python3 "$WS/tools/patrol_map_server.py" \
    --host 0.0.0.0 \
    --port 8080 \
    --routes-dir "$WS/routes"
