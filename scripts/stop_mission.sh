#!/usr/bin/env bash

set +e
set -u

WS="/home/nvidia/patrol_ws"
LOGS_DIR="$WS/logs"
RUN_DIR="$LOGS_DIR/stop_mission_$(date +%Y%m%d_%H%M%S)"

mkdir -p "$RUN_DIR"
ln -sfn "$RUN_DIR" "$LOGS_DIR/latest_stop_mission"

set +u
source /opt/ros/humble/setup.bash
source /home/nvidia/ros2_humble_main/install/setup.bash
source /home/nvidia/patrol_ws/install/setup.bash
set -u

service_exists() {
    timeout 3 ros2 service list \
        2>/dev/null |
        grep -Fxq "$1"
}

echo "========================================"
echo "停止当前巡逻任务"
echo "========================================"

if service_exists "/patrol/mission/stop"; then
    echo "[patrol] 停止任务管理器..."

    timeout 5 ros2 service call \
        /patrol/mission/stop \
        std_srvs/srv/Trigger \
        "{}" \
        2>&1 |
        tee "$RUN_DIR/mission_stop.txt" || true
fi

if service_exists "/patrol/entry_executor/enable"; then
    echo "[patrol] 停止入轨执行器..."

    timeout 5 ros2 service call \
        /patrol/entry_executor/enable \
        std_srvs/srv/SetBool \
        "{data: false}" \
        > "$RUN_DIR/entry_disable.txt" \
        2>&1 || true
fi

if service_exists "/patrol/route_follower/enable"; then
    echo "[patrol] 停止路线跟踪器..."

    timeout 5 ros2 service call \
        /patrol/route_follower/enable \
        std_srvs/srv/SetBool \
        "{data: false}" \
        > "$RUN_DIR/route_disable.txt" \
        2>&1 || true
fi

if service_exists "/patrol/set_control_mode"; then
    echo "[patrol] 切换到 STOP 模式..."

    timeout 5 ros2 service call \
        /patrol/set_control_mode \
        patrol_interfaces/srv/SetControlMode \
        "{mode: 0}" \
        2>&1 |
        tee "$RUN_DIR/set_stop_mode.txt" || true
else
    if timeout 3 ros2 node list \
        2>/dev/null |
        grep -Fxq "/vehicle_interface_node"; then

        echo "[patrol] 控制管理器不可用，直接补发停车命令..."

        timeout 1 ros2 topic pub -r 20 \
            /vehicle/command \
            vehicle_can_msg/msg/VehicleCommand \
            "{
                target_speed_rpm: 0.0,
                target_steering_angle_deg: 0.0,
                brake_pedal: 0,
                parking_brake: 1,
                control_mode: 1
            }" \
            > "$RUN_DIR/direct_stop.txt" \
            2>&1 || true
    fi
fi

sleep 1

echo
echo "===== 当前控制模式 ====="

timeout 3 ros2 topic echo \
    /patrol/control_mode \
    --once \
    2>&1 |
    tee "$RUN_DIR/control_mode_after_stop.txt" || true

echo
echo "===== 当前任务状态 ====="

timeout 3 ros2 topic echo \
    /patrol/mission/status \
    --once \
    2>&1 |
    tee "$RUN_DIR/mission_status_after_stop.txt" || true

echo
echo "========================================"
echo "车辆已请求停车，上层和底层节点保持运行"
echo "需要全部关闭时执行："
echo "  ./scripts/stop_all.sh"
echo "日志目录：$RUN_DIR"
