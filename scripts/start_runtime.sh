#!/usr/bin/env bash

set -Eeuo pipefail

WS="/home/nvidia/patrol_ws"
LOGS_DIR="$WS/logs"
CONFIG_FILE="$WS/config/patrol_system.yaml"

INITIAL_ROUTE="${1:-$WS/routes/test_route.yaml}"

if [[ "$INITIAL_ROUTE" != /* ]]; then
    INITIAL_ROUTE="$WS/$INITIAL_ROUTE"
fi

INITIAL_ROUTE="$(
    realpath -m "$INITIAL_ROUTE"
)"

RUN_DIR="$LOGS_DIR/runtime_$(date +%Y%m%d_%H%M%S)"
LAUNCH_LOG="$RUN_DIR/runtime_launch.log"
PID_FILE="$RUN_DIR/runtime_launch.pid"

UPPER_NODES=(
    /patrol_localization
    /patrol_route_recorder
    /patrol_entry_planner
    /patrol_entry_executor
    /patrol_route_follower
    /patrol_auto_command_mux
    /patrol_command_manager
    /patrol_mission_manager
)

REQUIRED_SERVICES=(
    /patrol/set_control_mode
    /patrol/mission/start
    /patrol/mission/stop
    /patrol/mission/load_route
    /patrol/localization/load_route
    /patrol/entry_planner/load_route
    /patrol/entry_planner/plan
    /patrol/entry_executor/enable
    /patrol/route_follower/load_route
    /patrol/route_follower/enable
)

set +u
source /opt/ros/humble/setup.bash
source /home/nvidia/ros2_humble_main/install/setup.bash
source "$WS/install/setup.bash"
set -u

if [ ! -f "$CONFIG_FILE" ]; then
    echo "[patrol] 统一配置文件不存在：$CONFIG_FILE"
    exit 1
fi

if [ ! -f "$INITIAL_ROUTE" ]; then
    echo "[patrol] 初始路线不存在：$INITIAL_ROUTE"
    exit 1
fi

python3 - "$CONFIG_FILE" <<'PY'
import sys
from patrol_bringup.config_loader import load_patrol_config

load_patrol_config(sys.argv[1])
print("[patrol] 统一配置校验通过")
PY

node_exists() {
    local node_name="$1"

    ros2 node list 2>/dev/null |
        grep -Fxq "$node_name"
}

service_exists() {
    local service_name="$1"

    ros2 service list 2>/dev/null |
        grep -Fxq "$service_name"
}

check_real_command_chain() {
    local output_file="$1"

    ros2 topic info -v /vehicle/command \
        > "$output_file" \
        2>&1 || true

    local publisher_count
    local subscriber_count

    publisher_count="$(
        awk '
            /Publisher count:/ {
                print $3
                exit
            }
        ' "$output_file"
    )"

    subscriber_count="$(
        awk '
            /Subscription count:/ {
                print $3
                exit
            }
        ' "$output_file"
    )"

    if [ "${publisher_count:-0}" -ne 1 ]; then
        echo "[patrol] /vehicle/command 发布者数量异常："
        cat "$output_file"
        return 1
    fi

    if [ "${subscriber_count:-0}" -lt 1 ]; then
        echo "[patrol] /vehicle/command 没有底盘订阅者："
        cat "$output_file"
        return 1
    fi

    if ! grep -q \
        "patrol_command_manager" \
        "$output_file"; then

        echo "[patrol] /vehicle/command 唯一发布者不是 patrol_command_manager："
        cat "$output_file"
        return 1
    fi

    return 0
}

echo "========================================"
echo "巡检小车上层常驻运行时"
echo "========================================"
echo "初始路线：$INITIAL_ROUTE"
echo

if ! node_exists "/smins200_tcp_demo"; then
    echo "[patrol] MINS200 未运行。"
    echo "请先执行："
    echo "  ./scripts/start_base.sh"
    exit 1
fi

if ! node_exists "/vehicle_interface_node"; then
    echo "[patrol] 底盘 CAN 节点未运行。"
    echo "请先执行："
    echo "  ./scripts/start_base.sh"
    exit 1
fi

INITIAL_NODES="$(
    ros2 node list 2>/dev/null || true
)"

RUNNING_COUNT=0

for node_name in "${UPPER_NODES[@]}"; do
    if grep -Fxq "$node_name" <<< "$INITIAL_NODES"; then
        RUNNING_COUNT=$((RUNNING_COUNT + 1))
    fi
done

if [ "$RUNNING_COUNT" -gt 0 ]; then
    echo "[patrol] 检测到已有上层运行时，执行健康检查..."

    RUNTIME_HEALTHY=0
    LAST_NODES=""
    LAST_SERVICES=""

    # ROS 2图发现可能短暂漏报，最多等待3秒。
    for _ in $(seq 1 12); do
        LAST_NODES="$(
            ros2 node list 2>/dev/null || true
        )"

        LAST_SERVICES="$(
            ros2 service list 2>/dev/null || true
        )"

        MISSING_SERVICE=0

        for service_name in "${REQUIRED_SERVICES[@]}"; do
            if ! grep -Fxq \
                "$service_name" \
                <<< "$LAST_SERVICES"; then

                MISSING_SERVICE=1
                break
            fi
        done

        if [ "$MISSING_SERVICE" -eq 0 ]; then
            RUNTIME_HEALTHY=1
            break
        fi

        sleep 0.25
    done

    if [ "$RUNTIME_HEALTHY" -eq 1 ]; then
        TEMP_INFO="/tmp/patrol_runtime_vehicle_command_info.txt"

        if ! check_real_command_chain "$TEMP_INFO"; then
            echo "[patrol] 当前常驻运行时命令链异常。"
            echo "[patrol] 请执行 ./scripts/stop_all.sh 后重新启动。"
            exit 1
        fi

        MISSING_NODES=()

        for node_name in "${UPPER_NODES[@]}"; do
            if ! grep -Fxq \
                "$node_name" \
                <<< "$LAST_NODES"; then

                MISSING_NODES+=("$node_name")
            fi
        done

        if [ "${#MISSING_NODES[@]}" -gt 0 ]; then
            echo "[patrol] 警告：节点图暂未显示以下节点，"
            echo "[patrol] 但其关键服务和命令链均正常："

            for node_name in "${MISSING_NODES[@]}"; do
                echo "  $node_name"
            done
        fi

        echo "[patrol] 常驻运行时健康，可直接复用。"
        exit 0
    fi

    echo "[patrol] 已有上层运行时不完整。"
    echo
    echo "节点状态："

    for node_name in "${UPPER_NODES[@]}"; do
        if grep -Fxq "$node_name" <<< "$LAST_NODES"; then
            echo "  已发现：$node_name"
        else
            echo "  未发现：$node_name"
        fi
    done

    echo
    echo "缺失服务："

    for service_name in "${REQUIRED_SERVICES[@]}"; do
        if ! grep -Fxq \
            "$service_name" \
            <<< "$LAST_SERVICES"; then

            echo "  $service_name"
        fi
    done

    echo
    echo "[patrol] 为避免重复发布和状态错乱，请执行："
    echo "  ./scripts/stop_all.sh"
    exit 1
fi

mkdir -p "$RUN_DIR"
ln -sfn "$RUN_DIR" "$LOGS_DIR/latest_runtime"

echo "[patrol] 启动上层常驻节点..."
echo "[patrol] 日志：$LAUNCH_LOG"

setsid ros2 launch \
    patrol_bringup \
    replay_patrol.launch.py \
    route_file:="$INITIAL_ROUTE" \
    config_file:="$CONFIG_FILE" \
    vehicle_command_topic:=/vehicle/command \
    > "$LAUNCH_LOG" 2>&1 &

LAUNCH_PID=$!
echo "$LAUNCH_PID" > "$PID_FILE"

cleanup_failed_runtime() {
    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
        return
    fi

    local pgid
    pgid="$(
        ps -o pgid= -p "$LAUNCH_PID" 2>/dev/null |
        tr -d ' '
    )"

    if [[ "$pgid" =~ ^[0-9]+$ ]]; then
        kill -INT -- "-$pgid" 2>/dev/null || true
        sleep 2

        if kill -0 "$LAUNCH_PID" 2>/dev/null; then
            kill -TERM -- "-$pgid" 2>/dev/null || true
        fi
    fi
}

READY=0

for _ in $(seq 1 160); do
    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
        echo "[patrol] 上层 launch 已异常退出："
        tail -120 "$LAUNCH_LOG"
        exit 1
    fi

    ALL_READY=1

    for node_name in "${UPPER_NODES[@]}"; do
        if ! node_exists "$node_name"; then
            ALL_READY=0
            break
        fi
    done

    if [ "$ALL_READY" -eq 1 ]; then
        for service_name in "${REQUIRED_SERVICES[@]}"; do
            if ! service_exists "$service_name"; then
                ALL_READY=0
                break
            fi
        done
    fi

    if [ "$ALL_READY" -eq 1 ]; then
        READY=1
        break
    fi

    sleep 0.25
done

if [ "$READY" -ne 1 ]; then
    echo "[patrol] 等待上层常驻节点超时。"
    tail -120 "$LAUNCH_LOG"
    cleanup_failed_runtime
    exit 1
fi

if ! check_real_command_chain \
    "$RUN_DIR/vehicle_command_info.txt"; then

    cleanup_failed_runtime
    exit 1
fi

# 常驻运行时启动后必须保持停车状态。
timeout 5 ros2 service call \
    /patrol/mission/stop \
    std_srvs/srv/Trigger \
    "{}" \
    > "$RUN_DIR/initial_mission_stop.txt" \
    2>&1 || true

timeout 5 ros2 service call \
    /patrol/set_control_mode \
    patrol_interfaces/srv/SetControlMode \
    "{mode: 0}" \
    > "$RUN_DIR/initial_control_stop.txt" \
    2>&1 || true

echo
echo "========================================"
echo "上层常驻运行时启动完成"
echo "========================================"
echo "PID：$LAUNCH_PID"
echo "日志：$LAUNCH_LOG"
echo
echo "节点将保持运行。"
echo "停止单次任务："
echo "  ./scripts/stop_mission.sh"
echo
echo "彻底关闭系统："
echo "  ./scripts/stop_all.sh"
