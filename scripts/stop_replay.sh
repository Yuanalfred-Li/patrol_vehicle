#!/usr/bin/env bash

set +e
set -u

WS="/home/nvidia/patrol_ws"
LOGS_DIR="$WS/logs"
RUN_DIR="$LOGS_DIR/stop_replay_$(date +%Y%m%d_%H%M%S)"
PID_FILE="$LOGS_DIR/latest_replay/replay_launch.pid"

mkdir -p "$RUN_DIR"
ln -sfn "$RUN_DIR" "$LOGS_DIR/latest_stop_replay"

set +u
source /opt/ros/humble/setup.bash
source /home/nvidia/ros2_humble_main/install/setup.bash
source /home/nvidia/patrol_ws/install/setup.bash
set -u

echo "========================================"
echo "停止路线巡迹"
echo "========================================"

echo "[patrol] 先请求任务安全停车..."
"$WS/scripts/stop_mission.sh" \
    > "$RUN_DIR/stop_mission.log" 2>&1 || true

if [ -f "$PID_FILE" ]; then
    PID="$(cat "$PID_FILE" 2>/dev/null)"

    if [[ "$PID" =~ ^[0-9]+$ ]] &&
       kill -0 "$PID" 2>/dev/null; then

        COMMAND_LINE="$(
            ps -o args= -p "$PID" 2>/dev/null
        )"

        if grep -Eq \
            'replay_patrol\.launch\.py|patrol_system\.launch\.py' \
            <<< "$COMMAND_LINE"; then

            PGID="$(
                ps -o pgid= -p "$PID" |
                tr -d ' '
            )"

            if [[ "$PGID" =~ ^[0-9]+$ ]]; then
                echo "[patrol] 关闭巡迹进程组：$PGID"

                kill -INT -- "-$PGID" \
                    2>/dev/null || true

                for _ in $(seq 1 30); do
                    if ! kill -0 "$PID" 2>/dev/null; then
                        break
                    fi
                    sleep 0.2
                done

                if kill -0 "$PID" 2>/dev/null; then
                    kill -TERM -- "-$PGID" \
                        2>/dev/null || true
                fi
            fi
        fi
    fi
fi

pkill -INT -f \
    '[r]os2 launch patrol_bringup replay_patrol.launch.py' \
    2>/dev/null || true

sleep 2

echo "[patrol] 巡迹上层模块已请求关闭。"
echo "[patrol] MINS200 和底盘 CAN 保持运行。"
echo "[patrol] 日志：$RUN_DIR"
