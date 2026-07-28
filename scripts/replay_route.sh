#!/usr/bin/env bash

set -Eeuo pipefail

WS="/home/nvidia/patrol_ws"
ROUTES_DIR="$WS/routes"
LOGS_DIR="$WS/logs"

AUTO_CONFIRM=0
ROUTE_INPUT=""

for argument in "$@"; do
    case "$argument" in
        --yes)
            AUTO_CONFIRM=1
            ;;
        -h|--help)
            ROUTE_INPUT=""
            break
            ;;
        --*)
            echo "[patrol] 未知参数：$argument"
            exit 1
            ;;
        *)
            if [ -n "$ROUTE_INPUT" ]; then
                echo "[patrol] 只能指定一条路线。"
                exit 1
            fi

            ROUTE_INPUT="$argument"
            ;;
    esac
done

ROUTE_NAME="${ROUTE_INPUT%.yaml}"

if [ -z "$ROUTE_NAME" ]; then
    echo "用法："
    echo "  $0 路线名称 [--yes]"
    echo
    echo "示例："
    echo "  $0 东门路线"
    echo "  $0 \"教学楼 夜间巡逻\""
    echo "  $0 东门路线 --yes"
    echo
    echo "--yes：通过全部安全检查后无需手动输入 START"
    exit 1
fi

case "$ROUTE_NAME" in
    *"/"*|*".."*)
        echo "[patrol] 路线名称不能包含 / 或 .."
        exit 1
        ;;
esac

ROUTE_FILE="$ROUTES_DIR/${ROUTE_NAME}.yaml"
RUN_DIR="$LOGS_DIR/replay_$(date +%Y%m%d_%H%M%S)_${ROUTE_NAME}"
LAUNCH_LOG="$RUN_DIR/replay_launch.log"
PID_FILE="$RUN_DIR/replay_launch.pid"

mkdir -p "$RUN_DIR"
ln -sfn "$RUN_DIR" "$LOGS_DIR/latest_replay"

set +u
source /opt/ros/humble/setup.bash
source /home/nvidia/ros2_humble_main/install/setup.bash
source /home/nvidia/patrol_ws/install/setup.bash
set -u

if [ ! -f "$ROUTE_FILE" ]; then
    echo "[patrol] 路线不存在：$ROUTE_FILE"
    echo
    echo "已有路线："

    find "$ROUTES_DIR" \
        -maxdepth 1 \
        -type f \
        -name "*.yaml" \
        -printf "  %f\n" \
        2>/dev/null |
        sort || true

    exit 1
fi

python3 - "$ROUTE_FILE" <<'PY' | tee "$RUN_DIR/route_check.txt"
import math
import sys
from pathlib import Path

import yaml

path = Path(sys.argv[1])

data = yaml.safe_load(
    path.read_text(encoding="utf-8")
)

if not isinstance(data, dict):
    raise SystemExit("路线 YAML 根节点无效")

format_version = int(
    data.get("format_version", 0)
)

if format_version < 2:
    raise SystemExit(
        "路线格式版本低于2，禁止实车复现"
    )

origin = data.get("origin")

if not isinstance(origin, dict):
    raise SystemExit("路线缺少 origin")

values = [
    float(origin["latitude"]),
    float(origin["longitude"]),
    float(origin["altitude"]),
]

if not all(math.isfinite(value) for value in values):
    raise SystemExit("origin 包含无效数值")

if not -90.0 <= values[0] <= 90.0:
    raise SystemExit("origin latitude 超出范围")

if not -180.0 <= values[1] <= 180.0:
    raise SystemExit("origin longitude 超出范围")

if (
    abs(values[0]) < 1.0e-8
    and abs(values[1]) < 1.0e-8
):
    raise SystemExit(
        "origin 为测试值 0,0，禁止实车复现"
    )

waypoints = data.get("waypoints")

if not isinstance(waypoints, list) or len(waypoints) < 2:
    raise SystemExit("路线轨迹点少于2个")

for index, point in enumerate(waypoints):
    if not isinstance(point, dict):
        raise SystemExit(f"轨迹点 {index} 格式无效")

    for key in (
        "latitude",
        "longitude",
        "altitude",
    ):
        if key not in point:
            raise SystemExit(
                f"轨迹点 {index} 缺少 {key}"
            )

summary = data.get("summary", {})

print("路线文件：", path)
print("格式版本：", data.get("format_version"))
print("轨迹点数：", len(waypoints))
print(
    "轨迹长度：",
    round(
        float(summary.get("total_length", 0.0)),
        3,
    ),
    "m",
)
print(
    "origin：",
    f"{values[0]:.10f},",
    f"{values[1]:.10f},",
    f"{values[2]:.3f}",
)
print(
    "origin yaw：",
    origin.get("yaw_deg", "未保存"),
)
PY

echo
echo "[patrol] 检查底层节点..."

NODES="$(ros2 node list 2>/dev/null || true)"

if ! grep -Fxq \
    "/smins200_tcp_demo" \
    <<< "$NODES"; then

    echo "[patrol] MINS200 未运行。"
    echo "[patrol] 请先执行："
    echo "  ./scripts/start_base.sh"
    exit 1
fi

if ! grep -Fxq \
    "/vehicle_interface_node" \
    <<< "$NODES"; then

    echo "[patrol] 底盘 CAN 节点未运行。"
    echo "[patrol] 请先执行："
    echo "  ./scripts/start_base.sh"
    exit 1
fi

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
    if grep -Fxq "$node_name" <<< "$NODES"; then
        echo "[patrol] 检测到旧上层节点：$node_name"
        echo "[patrol] 请先执行："
        echo "  ./scripts/stop_all.sh"
        echo "然后重新启动底层。"
        exit 1
    fi
done

ip -details link show can0 \
    > "$RUN_DIR/can0.txt" \
    2>&1 || true

CAN_STATE="$(
    awk '/can state / {
        print $3
        exit
    }' "$RUN_DIR/can0.txt"
)"

case "$CAN_STATE" in
    ERROR-ACTIVE)
        echo "[patrol] can0：ERROR-ACTIVE"
        ;;
    ERROR-WARNING)
        echo "[patrol] 警告：can0 当前为 ERROR-WARNING，临时允许低速复现。"
        cat "$RUN_DIR/can0.txt"
        ;;
    *)
        echo "[patrol] can0 状态不允许复现：${CAN_STATE:-UNKNOWN}"
        cat "$RUN_DIR/can0.txt"
        exit 1
        ;;
esac

LAUNCH_PID=""
KEEP_RUNNING=0

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

    kill -INT -- "-$pgid" 2>/dev/null || true
    sleep 2

    if kill -0 "$LAUNCH_PID" 2>/dev/null; then
        kill -TERM -- "-$pgid" \
            2>/dev/null || true
    fi
}

cleanup() {
    local exit_code=$?

    trap - EXIT INT TERM
    set +e

    if [ "$KEEP_RUNNING" -ne 1 ]; then
        if ros2 service list 2>/dev/null |
            grep -Fxq "/patrol/set_control_mode"; then

            timeout 5 ros2 service call \
                /patrol/set_control_mode \
                patrol_interfaces/srv/SetControlMode \
                "{mode: 0}" \
                >/dev/null 2>&1 || true
        fi

        stop_launch
    fi

    exit "$exit_code"
}

trap cleanup EXIT
trap 'exit 130' INT TERM

echo
echo "[patrol] 启动路线复现模块..."

setsid ros2 launch \
    patrol_bringup \
    replay_patrol.launch.py \
    route_file:="$ROUTE_FILE" \
    vehicle_command_topic:=/vehicle/command \
    > "$LAUNCH_LOG" 2>&1 &

LAUNCH_PID=$!
echo "$LAUNCH_PID" > "$PID_FILE"

wait_service() {
    local service_name="$1"

    for _ in $(seq 1 100); do
        if ros2 service list 2>/dev/null |
            grep -Fxq "$service_name"; then
            return 0
        fi

        if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
            return 1
        fi

        sleep 0.2
    done

    return 1
}

if ! wait_service "/patrol/set_control_mode"; then
    echo "[patrol] 控制服务启动失败："
    tail -100 "$LAUNCH_LOG"
    exit 1
fi

if ! wait_service "/patrol/mission/start"; then
    echo "[patrol] 任务服务启动失败："
    tail -100 "$LAUNCH_LOG"
    exit 1
fi

echo "[patrol] 等待真实定位有效..."

LOCALIZATION_OK=0

for _ in $(seq 1 120); do
    if timeout 2 ros2 topic echo \
        /patrol/localization_status \
        --once \
        > "$RUN_DIR/localization_status.txt" \
        2>&1; then

        if grep -Eq \
            "^[[:space:]]*valid:[[:space:]]*true" \
            "$RUN_DIR/localization_status.txt"; then

            LOCALIZATION_OK=1
            break
        fi
    fi

    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
        break
    fi

    sleep 0.25
done

if [ "$LOCALIZATION_OK" -ne 1 ]; then
    echo "[patrol] 定位未达到有效状态："
    cat "$RUN_DIR/localization_status.txt" \
        2>/dev/null || true
    echo
    echo "[patrol] 任务不会启动。"
    exit 1
fi

echo
echo "===== 当前定位 ====="
cat "$RUN_DIR/localization_status.txt"

read -r CURRENT_EAST CURRENT_NORTH < <(
    awk '
        /^[[:space:]]*east:/ {
            east = $2
        }

        /^[[:space:]]*north:/ {
            north = $2
        }

        END {
            print east, north
        }
    ' "$RUN_DIR/localization_status.txt"
)

if [ -z "${CURRENT_EAST:-}" ] ||
   [ -z "${CURRENT_NORTH:-}" ]; then

    echo "[patrol] 无法读取当前 ENU 坐标，禁止开始巡迹。"
    exit 1
fi

MAX_ORIGIN_DISTANCE_M="${
    PATROL_MAX_ORIGIN_DISTANCE_M:-4.0
}"

ORIGIN_DISTANCE_M="$(
    python3 - \
        "$CURRENT_EAST" \
        "$CURRENT_NORTH" <<'PY_DISTANCE'
import math
import sys

east = float(sys.argv[1])
north = float(sys.argv[2])

print(f"{math.hypot(east, north):.3f}")
PY_DISTANCE
)"

echo
echo "===== 路线原点距离检查 ====="
echo "当前 east ：$CURRENT_EAST m"
echo "当前 north：$CURRENT_NORTH m"
echo "距原点    ：$ORIGIN_DISTANCE_M m"
echo "允许距离  ：≤ $MAX_ORIGIN_DISTANCE_M m"

if ! python3 - \
    "$ORIGIN_DISTANCE_M" \
    "$MAX_ORIGIN_DISTANCE_M" <<'PY_CHECK'
import math
import sys

distance = float(sys.argv[1])
maximum = float(sys.argv[2])

valid = (
    math.isfinite(distance)
    and math.isfinite(maximum)
    and maximum > 0.0
    and distance <= maximum
)

raise SystemExit(0 if valid else 1)
PY_CHECK
then
    echo
    echo "[patrol] 当前车辆距离路线原点过远。"
    echo "[patrol] 请先将车辆移动至路线原点附近。"
    echo "[patrol] 任务不会启动。"
    exit 1
fi

echo "[patrol] 原点距离检查通过。"

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

    echo "[patrol] 唯一发布者不是 patrol_command_manager："
    cat "$RUN_DIR/vehicle_command_info.txt"
    exit 1
fi

ros2 service call \
    /patrol/set_control_mode \
    patrol_interfaces/srv/SetControlMode \
    "{mode: 0}" \
    > "$RUN_DIR/initial_stop.log" \
    2>&1 || true


set_low_speed_parameter() {
    local node_name="$1"
    local parameter_name="$2"
    local parameter_value="$3"
    local result

    result="$(
        timeout 5 ros2 param set             "$node_name"             "$parameter_name"             "$parameter_value"             2>&1
    )"

    echo "$node_name $parameter_name=$parameter_value"
    echo "$result"
    echo "$node_name $parameter_name=$parameter_value: $result"         >> "$RUN_DIR/speed_parameters.log"

    if ! grep -q         "Set parameter successful"         <<< "$result"; then

        echo "[patrol] 低速参数设置失败，禁止开始复现。"
        exit 1
    fi
}

echo "[patrol] 设置低速复现参数..."

set_low_speed_parameter     /patrol_entry_executor     forward_speed_rpm     15.0

set_low_speed_parameter     /patrol_entry_executor     reverse_speed_rpm     10.0

set_low_speed_parameter \
    /patrol_route_follower \
    minimum_speed_rpm \
    10.0

set_low_speed_parameter \
    /patrol_route_follower \
    max_speed_rpm \
    15.0

set_low_speed_parameter \
    /patrol_route_follower \
    lookahead_distance \
    1.0

echo "[patrol] 低速参数设置完成："
echo "  入轨前进：15 RPM"
echo "  入轨倒车：10 RPM"
echo "  路线速度：10～15 RPM"

echo
echo "========================================"
echo "路线复现准备完成"
echo "========================================"
echo "路线：$ROUTE_NAME"
echo "文件：$ROUTE_FILE"
echo
echo "当前系统没有障碍物地图。"
echo "Hybrid A* 可能规划倒车。"
echo "必须确认车辆前后区域完全无障碍物。"
echo
if [ "$AUTO_CONFIRM" -eq 1 ]; then
    echo "[patrol] --yes 已启用，安全检查通过后自动确认 START。"
    answer="START"
else
    read -r -p \
        "确认安全后输入 START 开始复现：" \
        answer
fi

if [ "$answer" != "START" ]; then
    echo "[patrol] 已取消，车辆保持 STOP。"
    exit 0
fi

echo "[patrol] 切换 AUTO 模式..."

MODE_RESULT="$(
    ros2 service call \
        /patrol/set_control_mode \
        patrol_interfaces/srv/SetControlMode \
        "{mode: 2}" \
        2>&1
)"

echo "$MODE_RESULT" |
    tee "$RUN_DIR/auto_mode_result.txt"

if ! grep -Eqi \
    "success[=:][[:space:]]*(true|True)" \
    <<< "$MODE_RESULT"; then

    echo "[patrol] AUTO 模式设置失败。"
    exit 1
fi

sleep 0.5

echo "[patrol] 启动自动任务..."

MISSION_RESULT="$(
    ros2 service call \
        /patrol/mission/start \
        std_srvs/srv/Trigger \
        "{}" \
        2>&1
)"

echo "$MISSION_RESULT" |
    tee "$RUN_DIR/mission_start_result.txt"

if ! grep -Eqi \
    "success[=:][[:space:]]*(true|True)" \
    <<< "$MISSION_RESULT"; then

    echo "[patrol] 自动任务启动失败。"

    ros2 service call \
        /patrol/set_control_mode \
        patrol_interfaces/srv/SetControlMode \
        "{mode: 0}" \
        >/dev/null 2>&1 || true

    exit 1
fi

KEEP_RUNNING=1

echo
echo "========================================"
echo "指定路线复现已启动"
echo "========================================"
echo "路线：$ROUTE_NAME"
echo "日志：$LAUNCH_LOG"
echo
echo "查看状态："
echo "  ./scripts/status.sh"
echo
echo "停止任务："
echo "  ./scripts/stop_mission.sh"
echo
echo "关闭全部："
echo "  ./scripts/stop_all.sh"

trap - EXIT INT TERM
exit 0
