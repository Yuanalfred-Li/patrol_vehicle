#!/usr/bin/env bash

set -Eeuo pipefail

WS="/home/nvidia/patrol_ws"
LOGS_DIR="$WS/logs"
RUN_DIR="$LOGS_DIR/manual_$(date +%Y%m%d_%H%M%S)"
LAUNCH_LOG="$RUN_DIR/manual_launch.log"
PID_FILE="$RUN_DIR/manual_launch.pid"

mkdir -p "$RUN_DIR"
ln -sfn "$RUN_DIR" "$LOGS_DIR/latest_manual"

set +u
source /opt/ros/humble/setup.bash
source /home/nvidia/ros2_humble_main/install/setup.bash
source /home/nvidia/patrol_ws/install/setup.bash
set -u

LAUNCH_PID=""

service_exists() {
    timeout 3 ros2 service list \
        2>/dev/null |
        grep -Fxq "$1"
}

set_stop_mode() {
    if service_exists "/patrol/set_control_mode"; then
        timeout 5 ros2 service call \
            /patrol/set_control_mode \
            patrol_interfaces/srv/SetControlMode \
            "{mode: 0}" \
            > "$RUN_DIR/stop_mode.log" \
            2>&1 || true
    fi
}

stop_launch() {
    if [ -z "$LAUNCH_PID" ]; then
        return
    fi

    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
        return
    fi

    local pgid
    pgid="$(
        ps -o pgid= -p "$LAUNCH_PID" |
        tr -d ' '
    )"

    kill -INT -- "-$pgid" \
        2>/dev/null || true

    for _ in $(seq 1 20); do
        if ! kill -0 "$LAUNCH_PID" \
            2>/dev/null; then
            return
        fi

        sleep 0.2
    done

    kill -TERM -- "-$pgid" \
        2>/dev/null || true
}

cleanup() {
    local result=$?

    trap - EXIT INT TERM
    set +e

    echo
    echo "[patrol] 退出键盘控制，切换 STOP..."
    set_stop_mode
    sleep 0.5
    stop_launch

    exit "$result"
}

trap cleanup EXIT
trap 'exit 130' INT TERM

echo "========================================"
echo "巡检小车键盘遥控"
echo "========================================"
echo "日志目录：$RUN_DIR"
echo

NODES="$(ros2 node list 2>/dev/null || true)"

if ! grep -Fxq \
    "/vehicle_interface_node" \
    <<< "$NODES"; then

    echo "[patrol] 底盘 CAN 节点未运行。"
    echo "[patrol] 请先执行："
    echo "  ./scripts/start_base.sh"
    exit 1
fi

if grep -Fxq \
    "/patrol_command_manager" \
    <<< "$NODES"; then

    echo "[patrol] 已有巡逻控制管理器运行。"
    echo "[patrol] 请先执行："
    echo "  ./scripts/stop_all.sh"
    echo "然后重新执行 start_base.sh。"
    exit 1
fi

ip -details link show can0 \
    > "$RUN_DIR/can0.txt" \
    2>&1 || true

if ! grep -q \
    "can state ERROR-ACTIVE" \
    "$RUN_DIR/can0.txt"; then

    echo "[patrol] can0 不是 ERROR-ACTIVE："
    cat "$RUN_DIR/can0.txt"
    exit 1
fi

echo "[patrol] 启动手动控制管理器..."

setsid ros2 launch \
    patrol_bringup \
    manual_patrol.launch.py \
    vehicle_command_topic:=/vehicle/command \
    > "$LAUNCH_LOG" 2>&1 &

LAUNCH_PID=$!
echo "$LAUNCH_PID" > "$PID_FILE"

CONTROL_READY=0

for _ in $(seq 1 80); do
    if service_exists "/patrol/set_control_mode"; then
        CONTROL_READY=1
        break
    fi

    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
        break
    fi

    sleep 0.25
done

if [ "$CONTROL_READY" -ne 1 ]; then
    echo "[patrol] 控制管理器启动失败："
    tail -100 "$LAUNCH_LOG"
    exit 1
fi

sleep 1

ros2 topic info -v /vehicle/command \
    > "$RUN_DIR/vehicle_command_info.txt" \
    2>&1 || true

PUBLISHER_COUNT="$(
    awk '
        /Publisher count:/ {
            print $3
            exit
        }
    ' "$RUN_DIR/vehicle_command_info.txt"
)"

SUBSCRIPTION_COUNT="$(
    awk '
        /Subscription count:/ {
            print $3
            exit
        }
    ' "$RUN_DIR/vehicle_command_info.txt"
)"

if [ "${PUBLISHER_COUNT:-0}" -ne 1 ]; then
    echo "[patrol] /vehicle/command 发布者数量异常："
    cat "$RUN_DIR/vehicle_command_info.txt"
    exit 1
fi

if [ "${SUBSCRIPTION_COUNT:-0}" -lt 1 ]; then
    echo "[patrol] /vehicle/command 没有底盘订阅者："
    cat "$RUN_DIR/vehicle_command_info.txt"
    exit 1
fi

if ! grep -q \
    "patrol_command_manager" \
    "$RUN_DIR/vehicle_command_info.txt"; then

    echo "[patrol] 最终命令发布者不正确："
    cat "$RUN_DIR/vehicle_command_info.txt"
    exit 1
fi

set_stop_mode

echo "[patrol] 切换到 MANUAL 模式..."

MODE_RESULT="$(
    ros2 service call \
        /patrol/set_control_mode \
        patrol_interfaces/srv/SetControlMode \
        "{mode: 1}" \
        2>&1
)"

echo "$MODE_RESULT" |
    tee "$RUN_DIR/manual_mode_result.txt"

if ! grep -Eqi \
    "success[=:][[:space:]]*(true|True)" \
    <<< "$MODE_RESULT"; then

    echo "[patrol] 无法进入 MANUAL 模式。"
    exit 1
fi

echo
echo "========================================"
echo "键盘控制已启动"
echo "========================================"
echo "Q/W/E：左前 / 前进 / 右前"
echo "A/S/D：左转 / 停止 / 右转"
echo "Z/X/C：左后 / 后退 / 右后"
echo "空格：停止"
echo "英文句号：退出"
echo
echo "请先确认车辆周围安全。"
echo

set +e
ros2 run patrol_teleop teleop_node
TELEOP_RESULT=$?
set -e

echo
echo "[patrol] 键盘节点退出码：$TELEOP_RESULT"

trap - EXIT INT TERM

set_stop_mode
sleep 0.5
stop_launch

echo "[patrol] 手动控制已结束，底层节点保持运行。"
echo "[patrol] 需要关闭底层时执行："
echo "  ./scripts/stop_all.sh"
