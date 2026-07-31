#!/usr/bin/env bash

set -Eeuo pipefail

WS="/home/nvidia/patrol_ws"
ROUTES_DIR="$WS/routes"
LOGS_DIR="$WS/logs"
CONFIG_FILE="$WS/config/patrol_system.yaml"

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

mkdir -p "$RUN_DIR"
ln -sfn "$RUN_DIR" "$LOGS_DIR/latest_replay"

set +u
source /opt/ros/humble/setup.bash
source /home/nvidia/ros2_humble_main/install/setup.bash
source "$WS/install/setup.bash"
set -u

if [ ! -f "$CONFIG_FILE" ]; then
    echo "[patrol] 统一配置文件不存在：$CONFIG_FILE"
    exit 1
fi

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

python3 - "$ROUTE_FILE" <<'PY' \
    | tee "$RUN_DIR/route_check.txt"
import math
import sys
from pathlib import Path

import yaml

path = Path(sys.argv[1])
data = yaml.safe_load(path.read_text(encoding="utf-8"))

if not isinstance(data, dict):
    raise SystemExit("路线 YAML 根节点无效")

format_version = int(data.get("format_version", 0))

if format_version < 2:
    raise SystemExit("路线格式版本低于2，禁止实车复现")

origin = data.get("origin")

if not isinstance(origin, dict):
    raise SystemExit("路线缺少 origin")

try:
    origin_values = [
        float(origin["latitude"]),
        float(origin["longitude"]),
        float(origin["altitude"]),
    ]
except (KeyError, TypeError, ValueError) as exc:
    raise SystemExit(f"路线 origin 无效：{exc}") from exc

if not all(math.isfinite(value) for value in origin_values):
    raise SystemExit("origin 包含无效数值")

if not -90.0 <= origin_values[0] <= 90.0:
    raise SystemExit("origin latitude 超出范围")

if not -180.0 <= origin_values[1] <= 180.0:
    raise SystemExit("origin longitude 超出范围")

if (
    abs(origin_values[0]) < 1.0e-8
    and abs(origin_values[1]) < 1.0e-8
):
    raise SystemExit("origin 为测试值 0,0，禁止实车复现")

waypoints = data.get("waypoints")

if not isinstance(waypoints, list) or len(waypoints) < 2:
    raise SystemExit("路线轨迹点少于2个")

for index, point in enumerate(waypoints):
    if not isinstance(point, dict):
        raise SystemExit(f"轨迹点 {index} 格式无效")

    for key in ("latitude", "longitude", "altitude"):
        if key not in point:
            raise SystemExit(f"轨迹点 {index} 缺少 {key}")

summary = data.get("summary", {})

print("路线文件：", path)
print("格式版本：", format_version)
print("轨迹点数：", len(waypoints))
print(
    "轨迹长度：",
    round(float(summary.get("total_length", 0.0)), 3),
    "m",
)
print("origin yaw：", origin.get("yaw_deg", "未保存"))
PY

service_exists() {
    local service_name="$1"
    local services

    services="$(
        timeout 3 ros2 service list 2>/dev/null || true
    )"

    grep -Fxq "$service_name" <<< "$services"
}

safe_stop() {
    set +e

    if service_exists "/patrol/mission/stop"; then
        timeout 5 ros2 service call \
            /patrol/mission/stop \
            std_srvs/srv/Trigger \
            "{}" \
            >/dev/null 2>&1 || true
    fi

    if service_exists "/patrol/set_control_mode"; then
        timeout 5 ros2 service call \
            /patrol/set_control_mode \
            patrol_interfaces/srv/SetControlMode \
            "{mode: 0}" \
            >/dev/null 2>&1 || true
    fi

    set -e
}

START_SUCCEEDED=0

cleanup() {
    local exit_code=$?

    trap - EXIT INT TERM

    if [ "$START_SUCCEEDED" -ne 1 ]; then
        safe_stop
    fi

    exit "$exit_code"
}

trap cleanup EXIT
trap 'exit 130' INT TERM

echo
echo "[patrol] 检查底层节点..."

NODES="$(ros2 node list 2>/dev/null || true)"

if ! grep -Fxq "/smins200_tcp_demo" <<< "$NODES"; then
    echo "[patrol] MINS200 未运行。"
    echo "请先执行："
    echo "  ./scripts/start_base.sh"
    exit 1
fi

if ! grep -Fxq "/vehicle_interface_node" <<< "$NODES"; then
    echo "[patrol] 底盘 CAN 节点未运行。"
    echo "请先执行："
    echo "  ./scripts/start_base.sh"
    exit 1
fi

ip -details link show can0 \
    > "$RUN_DIR/can0.txt" \
    2>&1 || true

CAN_STATE="$(
    awk '
        /can .*state / {
            for (i = 1; i <= NF; i++) {
                if ($i == "state" && i < NF) {
                    print $(i + 1)
                    exit
                }
            }
        }
    ' "$RUN_DIR/can0.txt"
)"

case "$CAN_STATE" in
    ERROR-ACTIVE)
        echo "[patrol] can0：ERROR-ACTIVE"
        ;;
    ERROR-WARNING)
        echo "[patrol] 警告：can0 为 ERROR-WARNING，临时允许低速复现。"
        ;;
    *)
        echo "[patrol] can0 状态不允许复现：${CAN_STATE:-UNKNOWN}"
        cat "$RUN_DIR/can0.txt"
        exit 1
        ;;
esac

echo
echo "[patrol] 启动或复用上层常驻运行时..."

"$WS/scripts/start_runtime.sh" "$ROUTE_FILE" |
    tee "$RUN_DIR/runtime_start.txt"

for service_name in \
    /patrol/set_control_mode \
    /patrol/mission/start \
    /patrol/mission/stop \
    /patrol/mission/load_route
do
    if ! service_exists "$service_name"; then
        echo "[patrol] 必要服务不可用：$service_name"
        exit 1
    fi
done

echo
echo "[patrol] 停止上一任务并保持 STOP 模式..."
safe_stop

echo
echo "[patrol] 动态加载路线..."

MISSION_STATUS_LOG="$RUN_DIR/mission_load_status.txt"
: > "$MISSION_STATUS_LOG"

# 在发送加载请求前建立持续订阅，避免错过短暂的完成状态。
stdbuf -oL -eL timeout 20 ros2 topic echo \
    /patrol/mission/status \
    > "$MISSION_STATUS_LOG" \
    2>&1 &

MISSION_STATUS_PID=$!
sleep 1.0

LOAD_RESULT="$(
    timeout 8 ros2 service call \
        /patrol/mission/load_route \
        patrol_interfaces/srv/LoadRoute \
        "{route_file: '$ROUTE_FILE'}" \
        2>&1
)"

echo "$LOAD_RESULT" |
    tee "$RUN_DIR/load_route_result.txt"

if ! grep -Eqi \
    "success[=:][[:space:]]*(true|True)" \
    <<< "$LOAD_RESULT"; then

    echo "[patrol] 路线加载请求被拒绝。"
    exit 1
fi

ROUTE_BASENAME="$(basename "$ROUTE_FILE")"
ROUTE_LOAD_DONE=0

echo "[patrol] 等待三个组件完成路线切换..."

for _ in $(seq 1 75); do
    if grep -Fq \
        "route load failed:" \
        "$MISSION_STATUS_LOG"; then

        echo "[patrol] 路线协调加载失败："
        cat "$MISSION_STATUS_LOG"

        kill "$MISSION_STATUS_PID" 2>/dev/null || true
        wait "$MISSION_STATUS_PID" 2>/dev/null || true
        exit 1
    fi

    if grep -Fq \
        "route loaded and ready: $ROUTE_BASENAME" \
        "$MISSION_STATUS_LOG"; then

        ROUTE_LOAD_DONE=1
        break
    fi

    sleep 0.2
done

kill "$MISSION_STATUS_PID" 2>/dev/null || true
wait "$MISSION_STATUS_PID" 2>/dev/null || true

if [ "$ROUTE_LOAD_DONE" -ne 1 ]; then
    echo "[patrol] 等待路线协调加载完成超时。"
    cat "$RUN_DIR/mission_load_status.txt" \
        2>/dev/null || true
    exit 1
fi

for node_name in \
    /patrol_mission_manager \
    /patrol_entry_planner \
    /patrol_route_follower
do
    parameter_name="route_file"

    if [ "$node_name" = "/patrol_mission_manager" ]; then
        parameter_name="current_route_file"
    fi

    VALUE="$(
        ros2 param get \
            "$node_name" \
            "$parameter_name" \
            2>/dev/null || true
    )"

    echo "$node_name：$VALUE" |
        tee -a "$RUN_DIR/route_parameters.txt"

    if ! grep -Fq "$ROUTE_FILE" <<< "$VALUE"; then
        echo "[patrol] 节点路线参数不一致：$node_name"
        exit 1
    fi
done

sleep 0.5

echo
echo "[patrol] 等待真实定位有效..."

LOCALIZATION_OK=0

for _ in $(seq 1 120); do
    timeout 2 ros2 topic echo \
        /patrol/localization_status \
        --once \
        > "$RUN_DIR/localization_status.txt" \
        2>&1 || true

    if grep -Eq \
        "^[[:space:]]*valid:[[:space:]]*true" \
        "$RUN_DIR/localization_status.txt"; then

        LOCALIZATION_OK=1
        break
    fi

    sleep 0.25
done

if [ "$LOCALIZATION_OK" -ne 1 ]; then
    echo "[patrol] 定位未达到有效状态："
    cat "$RUN_DIR/localization_status.txt" \
        2>/dev/null || true
    exit 1
fi

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

    echo "[patrol] 无法读取当前 ENU 坐标。"
    exit 1
fi

MAX_ORIGIN_DISTANCE_M="$(
    python3 - "$CONFIG_FILE" <<'PY'
import sys
from pathlib import Path

import yaml

config = yaml.safe_load(
    Path(sys.argv[1]).read_text(encoding="utf-8")
)

startup = config.get("startup", {})
print(float(startup.get("maximum_origin_distance_m", 4.0)))
PY
)"

ORIGIN_DISTANCE_M="$(
    python3 - \
        "$CURRENT_EAST" \
        "$CURRENT_NORTH" <<'PY'
import math
import sys

east = float(sys.argv[1])
north = float(sys.argv[2])

print(f"{math.hypot(east, north):.3f}")
PY
)"

echo
echo "===== 路线原点距离检查 ====="
echo "当前 east ：$CURRENT_EAST m"
echo "当前 north：$CURRENT_NORTH m"
echo "距原点    ：$ORIGIN_DISTANCE_M m"
echo "允许距离  ：≤ $MAX_ORIGIN_DISTANCE_M m"

if ! python3 - \
    "$ORIGIN_DISTANCE_M" \
    "$MAX_ORIGIN_DISTANCE_M" <<'PY'
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
PY
then
    echo "[patrol] 当前车辆距离路线原点过远。"
    echo "[patrol] 任务不会启动。"
    exit 1
fi

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

safe_stop

echo
echo "========================================"
echo "路线复现准备完成"
echo "========================================"
echo "路线：$ROUTE_NAME"
echo
echo "当前系统没有障碍物地图。"
echo "Hybrid A* 可能规划倒车。"
echo "必须确认车辆前后区域完全无障碍物。"
echo

if [ "$AUTO_CONFIRM" -eq 1 ]; then
    echo "[patrol] --yes 已启用，安全检查通过后自动确认。"
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
    timeout 8 ros2 service call \
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
    timeout 8 ros2 service call \
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
    exit 1
fi

START_SUCCEEDED=1

echo
echo "========================================"
echo "指定路线复现已启动"
echo "========================================"
echo "路线：$ROUTE_NAME"
echo "本次日志：$RUN_DIR"
echo
echo "上层运行时保持常驻。"
echo
echo "停止当前任务："
echo "  ./scripts/stop_mission.sh"
echo
echo "再次启动其他路线："
echo "  ./scripts/replay_route.sh 路线名称"
echo
echo "彻底关闭系统："
echo "  ./scripts/stop_all.sh"

trap - EXIT INT TERM
exit 0
