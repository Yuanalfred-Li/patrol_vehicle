#!/usr/bin/env bash

set +e
set -u

WS="/home/nvidia/patrol_ws"
LOGS_DIR="$WS/logs"
RUN_DIR="$LOGS_DIR/status_$(date +%Y%m%d_%H%M%S)"

mkdir -p "$RUN_DIR"
ln -sfn "$RUN_DIR" "$LOGS_DIR/latest_status"

set +u
source /opt/ros/humble/setup.bash
source /home/nvidia/ros2_humble_main/install/setup.bash
source /home/nvidia/patrol_ws/install/setup.bash
set -u

echo "========================================"
echo "巡检小车状态"
echo "========================================"
echo "时间：$(date '+%Y-%m-%d %H:%M:%S')"
echo

ros2 node list \
    > "$RUN_DIR/node_list.txt" \
    2>&1 || true

echo "===== 底层节点 ====="

for node_name in \
    /smins200_tcp_demo \
    /vehicle_interface_node
do
    if grep -Fxq \
        "$node_name" \
        "$RUN_DIR/node_list.txt"; then
        echo "正常：$node_name"
    else
        echo "未运行：$node_name"
    fi
done

echo
echo "===== 上层节点 ====="

for node_name in \
    /patrol_localization \
    /patrol_route_recorder \
    /patrol_entry_planner \
    /patrol_entry_executor \
    /patrol_route_follower \
    /patrol_auto_command_mux \
    /patrol_command_manager \
    /patrol_mission_manager
do
    if grep -Fxq \
        "$node_name" \
        "$RUN_DIR/node_list.txt"; then
        echo "正常：$node_name"
    fi
done

echo
echo "===== CAN ====="

if ip -details link show can0 \
    > "$RUN_DIR/can0.txt" \
    2>&1; then

    grep -E \
        "^[0-9]+: can0|state |can state|bitrate" \
        "$RUN_DIR/can0.txt" || true
else
    echo "未找到 can0"
fi

echo
echo "===== GPS话题 ====="

timeout 3 ros2 topic info /gps/data \
    2>&1 |
    tee "$RUN_DIR/gps_info.txt" || true

echo
echo "===== IMU话题 ====="

timeout 3 ros2 topic info /imu/status \
    2>&1 |
    tee "$RUN_DIR/imu_info.txt" || true

echo
echo "===== 定位状态 ====="

timeout 3 ros2 topic echo \
    /patrol/localization_status \
    --once \
    2>&1 |
    tee "$RUN_DIR/localization_status.txt" || true

echo
echo "===== 控制模式 ====="

timeout 3 ros2 topic echo \
    /patrol/control_mode \
    --once \
    2>&1 |
    tee "$RUN_DIR/control_mode.txt" || true

echo
echo "===== 自动任务 ====="

timeout 3 ros2 topic echo \
    /patrol/mission/status \
    --once \
    2>&1 |
    tee "$RUN_DIR/mission_status.txt" || true

echo
echo "===== 最终底盘命令链路 ====="

timeout 3 ros2 topic info -v \
    /vehicle/command \
    2>&1 |
    tee "$RUN_DIR/vehicle_command_info.txt" || true

echo
echo "===== 相关进程 ====="

ps -eo pid,ppid,pgid,args |
    grep -E \
    "patrol_|smins200|vehicle_interface|record_patrol|replay_patrol" |
    grep -v grep |
    tee "$RUN_DIR/processes.txt" || true

echo
echo "========================================"
echo "状态日志：$RUN_DIR"
