#!/usr/bin/env bash

set +e
set -u

WS="/home/nvidia/patrol_ws"
LOGS_DIR="$WS/logs"
RUN_DIR="$LOGS_DIR/stop_$(date +%Y%m%d_%H%M%S)"

mkdir -p "$RUN_DIR"
ln -sfn "$RUN_DIR" "$LOGS_DIR/latest_stop"

set +u
source /opt/ros/humble/setup.bash
source /home/nvidia/ros2_humble_main/install/setup.bash
source /home/nvidia/patrol_ws/install/setup.bash
set -u

echo "========================================"
echo "巡检小车安全停止"
echo "========================================"
echo "日志目录：$RUN_DIR"
echo

service_exists() {
    timeout 3 ros2 service list 2>/dev/null |
        grep -Fxq "$1"
}

send_stop_mode() {
    if service_exists "/patrol/set_control_mode"; then
        echo "[patrol] 切换到 STOP 模式..."

        timeout 5 ros2 service call \
            /patrol/set_control_mode \
            patrol_interfaces/srv/SetControlMode \
            "{mode: 0}" \
            > "$RUN_DIR/set_stop_mode.log" \
            2>&1 || true
    fi
}

send_direct_stop() {
    if timeout 3 ros2 node list 2>/dev/null |
        grep -Fxq "/vehicle_interface_node"; then

        echo "[patrol] 向底盘补发停车命令..."

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
            > "$RUN_DIR/direct_stop.log" \
            2>&1 || true
    fi
}

stop_process_group() {
    local pid_file="$1"
    local expected_pattern="$2"
    local label="$3"

    if [ ! -f "$pid_file" ]; then
        return
    fi

    local pid
    pid="$(cat "$pid_file" 2>/dev/null)"

    if ! [[ "$pid" =~ ^[0-9]+$ ]]; then
        return
    fi

    if ! kill -0 "$pid" 2>/dev/null; then
        return
    fi

    local command_line
    command_line="$(ps -o args= -p "$pid" 2>/dev/null)"

    if ! grep -Eq "$expected_pattern" <<< "$command_line"; then
        echo "[patrol] 忽略可能已失效的 PID：$pid_file"
        return
    fi

    local pgid
    pgid="$(ps -o pgid= -p "$pid" 2>/dev/null |
        tr -d ' ')"

    if ! [[ "$pgid" =~ ^[0-9]+$ ]]; then
        return
    fi

    echo "[patrol] 停止 $label，PID=$pid，PGID=$pgid"

    kill -INT -- "-$pgid" 2>/dev/null || true

    for _ in $(seq 1 25); do
        if ! kill -0 "$pid" 2>/dev/null; then
            return
        fi

        sleep 0.2
    done

    kill -TERM -- "-$pgid" 2>/dev/null || true

    for _ in $(seq 1 15); do
        if ! kill -0 "$pid" 2>/dev/null; then
            return
        fi

        sleep 0.2
    done

    kill -KILL -- "-$pgid" 2>/dev/null || true
}

echo "[patrol] 停止自动任务..."

if service_exists "/patrol/mission/stop"; then
    timeout 5 ros2 service call \
        /patrol/mission/stop \
        std_srvs/srv/Trigger \
        "{}" \
        > "$RUN_DIR/mission_stop.log" \
        2>&1 || true
fi

send_stop_mode

echo "[patrol] 保存正在录制的路线..."

if service_exists "/patrol/route_recorder/stop"; then
    timeout 10 ros2 service call \
        /patrol/route_recorder/stop \
        std_srvs/srv/Trigger \
        "{}" \
        > "$RUN_DIR/route_recorder_stop.log" \
        2>&1 || true
fi

echo "[patrol] 关闭键盘控制..."

pkill -INT -f \
    '[r]os2 run patrol_teleop teleop_node' \
    2>/dev/null || true

pkill -INT -f \
    '/patrol_teleop/teleop_node' \
    2>/dev/null || true

sleep 1

send_stop_mode
send_direct_stop

echo "[patrol] 关闭上层巡逻模块..."

stop_process_group \
    "$LOGS_DIR/latest_record/record_launch.pid" \
    'record_patrol\.launch\.py' \
    "路线录制模块"

stop_process_group \
    "$LOGS_DIR/latest_runtime/runtime_launch.pid" \
    'replay_patrol\.launch\.py|patrol_system\.launch\.py' \
    "上层常驻运行时"

stop_process_group \
    "$LOGS_DIR/latest_replay/replay_launch.pid" \
    'replay_patrol\.launch\.py|full_patrol\.launch\.py' \
    "路线复现模块"

pkill -INT -f \
    '[r]os2 launch patrol_bringup record_patrol.launch.py' \
    2>/dev/null || true

pkill -INT -f \
    '[r]os2 launch patrol_bringup replay_patrol.launch.py' \
    2>/dev/null || true

pkill -INT -f \
    '[r]os2 launch patrol_bringup patrol_system.launch.py' \
    2>/dev/null || true

pkill -INT -f \
    '[r]os2 launch patrol_bringup full_patrol.launch.py' \
    2>/dev/null || true

pkill -INT -f \
    '/patrol_localization/localization_node' \
    2>/dev/null || true

pkill -INT -f \
    '/patrol_route_recorder/route_recorder_node' \
    2>/dev/null || true

pkill -INT -f \
    '/patrol_entry_planner/entry_planner_node' \
    2>/dev/null || true

pkill -INT -f \
    '/patrol_entry_executor/entry_executor_node' \
    2>/dev/null || true

pkill -INT -f \
    '/patrol_route_follower/route_follower_node' \
    2>/dev/null || true

pkill -INT -f \
    '/patrol_auto_command_mux/auto_command_mux_node' \
    2>/dev/null || true

pkill -INT -f \
    '/patrol_command_manager/command_manager_node' \
    2>/dev/null || true

pkill -INT -f \
    '/patrol_mission_manager/mission_manager_node' \
    2>/dev/null || true

sleep 2

send_direct_stop

echo "[patrol] 关闭底层硬件节点..."

stop_process_group \
    "$LOGS_DIR/latest_base/hardware_launch.pid" \
    'hardware\.launch\.py' \
    "底层硬件模块"

pkill -INT -f \
    '[r]os2 launch patrol_bringup hardware.launch.py' \
    2>/dev/null || true

pkill -INT -f \
    '/vehicle_can_interface/vehicle_interface_node' \
    2>/dev/null || true

pkill -INT -f \
    '[s]mins200_tcp_demo_node' \
    2>/dev/null || true

sleep 3

pkill -TERM -f \
    '/vehicle_can_interface/vehicle_interface_node' \
    2>/dev/null || true

pkill -TERM -f \
    '[s]mins200_tcp_demo_node' \
    2>/dev/null || true

sleep 1

ros2 daemon stop \
    > "$RUN_DIR/daemon_stop.log" \
    2>&1 || true

sleep 1

ros2 daemon start \
    > "$RUN_DIR/daemon_start.log" \
    2>&1 || true

sleep 2

ros2 node list \
    > "$RUN_DIR/nodes_after_stop.txt" \
    2>&1 || true

ps -eo pid,ppid,pgid,args |
    grep -E \
    "patrol_|smins200|vehicle_interface|record_patrol|replay_patrol" |
    grep -v grep \
    > "$RUN_DIR/processes_after_stop.txt" \
    2>&1 || true

ip -details link show can0 \
    > "$RUN_DIR/can0_after_stop.txt" \
    2>&1 || true

REMAINING_NODES="$(
    grep -E \
        "patrol_|smins200|vehicle_interface" \
        "$RUN_DIR/nodes_after_stop.txt" \
        2>/dev/null || true
)"

REMAINING_PROCESSES="$(
    cat "$RUN_DIR/processes_after_stop.txt" \
        2>/dev/null || true
)"

echo
echo "========================================"

if [ -z "$REMAINING_NODES" ] &&
   [ -z "$REMAINING_PROCESSES" ]; then
    echo "全部巡检模块已停止"
else
    echo "警告：仍检测到相关节点或进程"

    if [ -n "$REMAINING_NODES" ]; then
        echo
        echo "剩余节点："
        echo "$REMAINING_NODES"
    fi

    if [ -n "$REMAINING_PROCESSES" ]; then
        echo
        echo "剩余进程："
        echo "$REMAINING_PROCESSES"
    fi
fi

echo "========================================"
echo "日志目录：$RUN_DIR"
