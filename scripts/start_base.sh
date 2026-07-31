#!/usr/bin/env bash

set -Eeuo pipefail

WS="/home/nvidia/patrol_ws"
LOGS_DIR="$WS/logs"
RUN_DIR="$LOGS_DIR/base_$(date +%Y%m%d_%H%M%S)"
LAUNCH_LOG="$RUN_DIR/hardware_launch.log"
PID_FILE="$RUN_DIR/hardware_launch.pid"

mkdir -p "$RUN_DIR"
ln -sfn "$RUN_DIR" "$LOGS_DIR/latest_base"

set +u
source /opt/ros/humble/setup.bash
source /home/nvidia/ros2_humble_main/install/setup.bash
source /home/nvidia/patrol_ws/install/setup.bash
set -u

echo "========================================"
echo "巡检小车底层启动"
echo "========================================"
echo "日志目录：$RUN_DIR"
echo

NODES="$(ros2 node list 2>/dev/null || true)"

MINS_RUNNING=0
CAN_RUNNING=0

if grep -Fxq "/smins200_tcp_demo" <<< "$NODES"; then
    MINS_RUNNING=1
fi

if grep -Fxq "/vehicle_interface_node" <<< "$NODES"; then
    CAN_RUNNING=1
fi

if [ "$MINS_RUNNING" -eq 1 ] &&
   [ "$CAN_RUNNING" -eq 1 ]; then
    echo "[patrol] MINS200 和底盘 CAN 已经运行。"
    echo "[patrol] 无需重复启动。"
    exit 0
fi

if [ "$MINS_RUNNING" -eq 1 ] ||
   [ "$CAN_RUNNING" -eq 1 ]; then
    echo "[patrol] 检测到部分底层节点已经运行："
    ros2 node list | grep -E \
        "smins200_tcp_demo|vehicle_interface_node" \
        || true
    echo
    echo "[patrol] 请先执行 stop_all.sh 清理后再启动。"
    exit 1
fi

echo "[patrol] 检查 MINS200 网络..."

if ping -c 1 -W 1 192.168.1.33 \
    > "$RUN_DIR/ping_mins_before.txt" 2>&1; then
    echo "[patrol] MINS200 网络已连通。"
else
    echo "[patrol] 当前无法连接 192.168.1.33。"
    echo "[patrol] 尝试运行现有网络配置脚本。"

    sudo -v

    timeout 30 \
        /home/nvidia/ros2_humble_main/configure_network.sh \
        1 5 0 \
        > "$RUN_DIR/configure_network.log" \
        2>&1 || true

    if ! ping -c 2 -W 1 192.168.1.33 \
        > "$RUN_DIR/ping_mins_after.txt" 2>&1; then
        echo "[patrol] MINS200 网络仍然不通。"
        echo "[patrol] 请检查："
        echo "  1. MINS200 是否上电"
        echo "  2. 网线和网口是否正确"
        echo "  3. 192.168.1.33 是否能 ping 通"
        echo
        echo "[patrol] 网络配置日志："
        tail -60 "$RUN_DIR/configure_network.log" \
            2>/dev/null || true
        exit 1
    fi

    echo "[patrol] MINS200 网络配置完成。"
fi

echo "[patrol] 配置 can0：500000 bit/s..."

sudo -v
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 type can \
    bitrate 500000 \
    sample-point 0.750 \
    listen-only off \
    berr-reporting on \
    restart-ms 0
sudo ip link set can0 up

sleep 0.5

echo "[patrol] 启动 MINS200 和底盘 CAN 接口..."

setsid ros2 launch \
    patrol_bringup \
    hardware.launch.py \
    start_mins200:=true \
    start_vehicle_can:=true \
    can_transmit_enabled:=true \
    > "$LAUNCH_LOG" 2>&1 &

LAUNCH_PID=$!
echo "$LAUNCH_PID" > "$PID_FILE"

cleanup_failed_launch() {
    if kill -0 "$LAUNCH_PID" 2>/dev/null; then
        kill -INT -- "-$LAUNCH_PID" \
            2>/dev/null || true
        sleep 2
        kill -TERM -- "-$LAUNCH_PID" \
            2>/dev/null || true
    fi
}

READY=0

for _ in $(seq 1 80); do
    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
        echo "[patrol] 底层 launch 已异常退出："
        tail -100 "$LAUNCH_LOG"
        exit 1
    fi

    NODES="$(ros2 node list 2>/dev/null || true)"

    if grep -Fxq \
        "/smins200_tcp_demo" \
        <<< "$NODES" &&
       grep -Fxq \
        "/vehicle_interface_node" \
        <<< "$NODES"; then
        READY=1
        break
    fi

    sleep 0.25
done

if [ "$READY" -ne 1 ]; then
    echo "[patrol] 等待底层节点超时。"
    tail -100 "$LAUNCH_LOG"
    cleanup_failed_launch
    exit 1
fi

echo "[patrol] 检查 CAN 状态..."

CAN_OK=0

for _ in $(seq 1 20); do
    ip -details link show can0 \
        > "$RUN_DIR/can0.txt" \
        2>&1 || true

    if grep -Eq \
        'can( <[^>]+>)? state ERROR-ACTIVE' \
        "$RUN_DIR/can0.txt"; then
        CAN_OK=1
        break
    fi

    sleep 0.25
done

if [ "$CAN_OK" -ne 1 ]; then
    echo "[patrol] can0 未进入 ERROR-ACTIVE："
    cat "$RUN_DIR/can0.txt"
    cleanup_failed_launch
    exit 1
fi

echo "[patrol] 检查传感器话题..."

GPS_OK=0
IMU_OK=0

if timeout 8 ros2 topic echo \
    /gps/data \
    --once \
    > "$RUN_DIR/gps_once.txt" \
    2>&1; then
    GPS_OK=1
fi

if timeout 8 ros2 topic echo \
    /imu/status \
    --once \
    > "$RUN_DIR/imu_status_once.txt" \
    2>&1; then
    IMU_OK=1
fi

ros2 node list \
    > "$RUN_DIR/node_list.txt" \
    2>&1 || true

ros2 topic list \
    > "$RUN_DIR/topic_list.txt" \
    2>&1 || true

ros2 topic info /gps/data \
    > "$RUN_DIR/gps_info.txt" \
    2>&1 || true

ros2 topic info /imu/status \
    > "$RUN_DIR/imu_status_info.txt" \
    2>&1 || true

ros2 topic info /vehicle/command \
    > "$RUN_DIR/vehicle_command_info.txt" \
    2>&1 || true

echo
echo "========================================"
echo "底层启动完成"
echo "========================================"
echo "MINS200 节点：正常"
echo "底盘 CAN 节点：正常"
echo "can0：ERROR-ACTIVE"

if [ "$GPS_OK" -eq 1 ]; then
    echo "GPS 数据：已收到"
else
    echo "GPS 数据：暂未收到"
fi

if [ "$IMU_OK" -eq 1 ]; then
    echo "IMU 状态：已收到"
else
    echo "IMU 状态：暂未收到"
fi

echo
echo "PID：$LAUNCH_PID"
echo "日志：$LAUNCH_LOG"
echo
echo "接下来可以执行："
echo "  ./scripts/record_route.sh 路线名称"
