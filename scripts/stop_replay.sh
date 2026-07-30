#!/usr/bin/env bash

set -Eeuo pipefail

WS="/home/nvidia/patrol_ws"

echo "========================================"
echo "停止当前路线巡迹"
echo "========================================"
echo "[patrol] 上层常驻节点和底层节点将继续运行。"
echo

exec "$WS/scripts/stop_mission.sh"
