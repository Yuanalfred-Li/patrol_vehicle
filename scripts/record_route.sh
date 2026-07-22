#!/usr/bin/env bash

set -Eeuo pipefail

WS="/home/nvidia/patrol_ws"
ROUTES_DIR="$WS/routes"
LOGS_DIR="$WS/logs"
TOOLS_DIR="$WS/tools"

ROUTE_INPUT="${1:-}"
ROUTE_NAME="${ROUTE_INPUT%.yaml}"

if [ -z "$ROUTE_NAME" ]; then
    echo "用法："
    echo "  $0 路线名称"
    echo
    echo "示例："
    echo "  $0 东门路线"
    echo "  $0 \"教学楼 夜间巡逻\""
    exit 1
fi

case "$ROUTE_NAME" in
    *"/"*|*".."*)
        echo "[patrol] 路线名称不能包含 / 或 .."
        exit 1
        ;;
esac

ROUTE_FILE="$ROUTES_DIR/${ROUTE_NAME}.yaml"
RUN_DIR="$LOGS_DIR/record_$(date +%Y%m%d_%H%M%S)_${ROUTE_NAME}"
ORIGIN_FILE="$RUN_DIR/stable_origin.yaml"
LAUNCH_LOG="$RUN_DIR/record_launch.log"

mkdir -p "$ROUTES_DIR" "$RUN_DIR"
ln -sfn "$RUN_DIR" "$LOGS_DIR/latest_record"

set +u
source /opt/ros/humble/setup.bash
source /home/nvidia/ros2_humble_main/install/setup.bash
source /home/nvidia/patrol_ws/install/setup.bash
set -u

LAUNCH_PID=""
RECORDING_STARTED=0
ORIGIN_READY=0
FINALIZED=0

stop_mode() {
    if ros2 service list 2>/dev/null |
        grep -Fxq "/patrol/set_control_mode"; then
        timeout 5 ros2 service call \
            /patrol/set_control_mode \
            patrol_interfaces/srv/SetControlMode \
            "{mode: 0}" \
            >/dev/null 2>&1 || true
    fi
}

stop_recording() {
    if [ "$RECORDING_STARTED" -eq 1 ]; then
        echo "[patrol] 停止并保存路线..."

        timeout 10 ros2 service call \
            /patrol/route_recorder/stop \
            std_srvs/srv/Trigger \
            "{}" \
            2>&1 |
            tee "$RUN_DIR/stop_recording.log" || true

        RECORDING_STARTED=0
    fi
}

stop_launch() {
    if [ -n "$LAUNCH_PID" ] &&
        kill -0 "$LAUNCH_PID" 2>/dev/null; then

        echo "[patrol] 关闭录制节点..."

        kill -INT -- "-$LAUNCH_PID" \
            2>/dev/null || true

        for _ in $(seq 1 30); do
            if ! kill -0 "$LAUNCH_PID" \
                2>/dev/null; then
                break
            fi

            sleep 0.2
        done

        if kill -0 "$LAUNCH_PID" 2>/dev/null; then
            kill -TERM -- "-$LAUNCH_PID" \
                2>/dev/null || true
        fi

        wait "$LAUNCH_PID" 2>/dev/null || true
    fi

    LAUNCH_PID=""
}

finalize_route() {
    if [ "$ORIGIN_READY" -ne 1 ]; then
        return
    fi

    if [ "$FINALIZED" -eq 1 ]; then
        return
    fi

    if [ ! -s "$ROUTE_FILE" ]; then
        return
    fi

    echo "[patrol] 使用稳定 origin 重新计算整条路线..."

    if python3 "$TOOLS_DIR/finalize_recorded_route.py" \
        --route "$ROUTE_FILE" \
        --origin "$ORIGIN_FILE" |
        tee "$RUN_DIR/finalize_route.log"; then

        FINALIZED=1
    else
        echo "[patrol] 警告：路线最终整理失败。"
        echo "[patrol] 原始文件仍保留在：$ROUTE_FILE"
    fi
}

cleanup() {
    local exit_code=$?

    trap - EXIT INT TERM
    set +e

    stop_mode
    stop_recording
    stop_launch
    finalize_route

    exit "$exit_code"
}

trap cleanup EXIT
trap 'exit 130' INT TERM

wait_service() {
    local service_name="$1"
    local timeout_seconds="${2:-20}"
    local loops=$((timeout_seconds * 4))

    for _ in $(seq 1 "$loops"); do
        if ros2 service list 2>/dev/null |
            grep -Fxq "$service_name"; then
            return 0
        fi

        if [ -n "$LAUNCH_PID" ] &&
            ! kill -0 "$LAUNCH_PID" \
            2>/dev/null; then
            return 1
        fi

        sleep 0.25
    done

    return 1
}

echo "========================================"
echo "巡检路线录制"
echo "========================================"
echo "路线名称：$ROUTE_NAME"
echo "输出文件：$ROUTE_FILE"
echo "日志目录：$RUN_DIR"
echo

if [ -e "$ROUTE_FILE" ]; then
    read -r -p \
        "路线已存在，是否覆盖？[y/N] " \
        answer

    case "$answer" in
        y|Y|yes|YES)
            rm -f "$ROUTE_FILE"
            ;;
        *)
            echo "[patrol] 已取消。"
            exit 0
            ;;
    esac
fi

echo "[patrol] 检查底层节点..."

NODES="$(ros2 node list 2>/dev/null || true)"

if ! grep -Fxq \
    "/smins200_tcp_demo" \
    <<< "$NODES"; then

    echo "[patrol] MINS200 未运行。"
    echo "[patrol] 请先启动底层系统。"
    exit 1
fi

if ! grep -Fxq \
    "/vehicle_interface_node" \
    <<< "$NODES"; then

    echo "[patrol] 底盘 CAN 节点未运行。"
    echo "[patrol] 请先启动底层系统。"
    exit 1
fi

for node_name in \
    /patrol_localization \
    /patrol_route_recorder \
    /patrol_command_manager
do
    if grep -Fxq "$node_name" <<< "$NODES"; then
        echo "[patrol] 检测到旧节点：$node_name"
        echo "[patrol] 请先关闭上一次录制或巡逻任务。"
        exit 1
    fi
done

if ! ip -details link show can0 \
    > "$RUN_DIR/can0_before_record.txt" \
    2>&1; then

    echo "[patrol] 找不到 can0。"
    exit 1
fi

CAN_STATE="$(
    awk '/can state / {
        print $3
        exit
    }' "$RUN_DIR/can0_before_record.txt"
)"

case "$CAN_STATE" in
    ERROR-ACTIVE)
        echo "[patrol] can0：ERROR-ACTIVE"
        ;;
    ERROR-WARNING)
        echo "[patrol] 警告：can0 当前为 ERROR-WARNING，临时允许继续录制。"
        cat "$RUN_DIR/can0_before_record.txt"
        ;;
    *)
        echo "[patrol] can0 状态不允许继续：${CAN_STATE:-UNKNOWN}"
        cat "$RUN_DIR/can0_before_record.txt"
        exit 1
        ;;
esac

echo "[patrol] 请让车辆保持完全静止。"
echo "[patrol] 等待有效 GNSS 后连续采样 5 秒..."
echo

python3 "$TOOLS_DIR/estimate_route_origin.py" \
    --output "$ORIGIN_FILE" \
    --duration 5.0 \
    --maximum-wait 60.0 \
    --minimum-samples 20 \
    --minimum-nsv1 10 \
    --minimum-nsv2 10 \
    --maximum-horizontal-rms 0.80 \
    --maximum-horizontal-error 2.00 \
    --maximum-yaw-std 5.00 |
    tee "$RUN_DIR/origin_estimation.log"

ORIGIN_READY=1

read -r \
    ORIGIN_LATITUDE \
    ORIGIN_LONGITUDE \
    ORIGIN_ALTITUDE \
    ORIGIN_YAW_DEG \
    < <(
        python3 - "$ORIGIN_FILE" <<'PY'
import sys
import yaml

with open(sys.argv[1], "r", encoding="utf-8") as file:
    data = yaml.safe_load(file)

origin = data["origin"]

print(
    origin["latitude"],
    origin["longitude"],
    origin["altitude"],
    origin["yaw_deg"],
)
PY
    )

echo
echo "[patrol] 稳定 origin 已确定："
echo "  latitude : $ORIGIN_LATITUDE"
echo "  longitude: $ORIGIN_LONGITUDE"
echo "  altitude : $ORIGIN_ALTITUDE"
echo "  yaw      : $ORIGIN_YAW_DEG deg"
echo

echo "[patrol] 启动定位、记录器和控制管理器..."

setsid ros2 launch \
    patrol_bringup \
    record_patrol.launch.py \
    route_file:="$ROUTE_FILE" \
    origin_latitude:="$ORIGIN_LATITUDE" \
    origin_longitude:="$ORIGIN_LONGITUDE" \
    origin_altitude:="$ORIGIN_ALTITUDE" \
    vehicle_command_topic:=/vehicle/command \
    > "$LAUNCH_LOG" 2>&1 &

LAUNCH_PID=$!
echo "$LAUNCH_PID" > "$RUN_DIR/record_launch.pid"

if ! wait_service \
    /patrol/route_recorder/start \
    20; then

    echo "[patrol] 录制服务启动失败："
    tail -100 "$LAUNCH_LOG"
    exit 1
fi

if ! wait_service \
    /patrol/set_control_mode \
    20; then

    echo "[patrol] 控制模式服务启动失败："
    tail -100 "$LAUNCH_LOG"
    exit 1
fi

echo "[patrol] 检查定位状态..."

LOCALIZATION_OK=0

for _ in $(seq 1 30); do
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

    sleep 0.3
done

if [ "$LOCALIZATION_OK" -ne 1 ]; then
    echo "[patrol] 定位未变为有效状态："
    cat "$RUN_DIR/localization_status.txt" \
        2>/dev/null || true
    exit 1
fi

sleep 1

ros2 topic info /vehicle/command \
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

stop_mode

echo "[patrol] 开始记录轨迹..."

START_RESULT="$(
    ros2 service call \
        /patrol/route_recorder/start \
        std_srvs/srv/Trigger \
        "{}" \
        2>&1
)"

echo "$START_RESULT" |
    tee "$RUN_DIR/start_recording.log"

if ! grep -Eqi \
    "success[=:][[:space:]]*(true|True)" \
    <<< "$START_RESULT"; then

    echo "[patrol] 轨迹记录启动失败。"
    exit 1
fi

RECORDING_STARTED=1

MODE_RESULT="$(
    ros2 service call \
        /patrol/set_control_mode \
        patrol_interfaces/srv/SetControlMode \
        "{mode: 1}" \
        2>&1
)"

echo "$MODE_RESULT" |
    tee "$RUN_DIR/manual_mode.log"

if ! grep -Eqi \
    "success[=:][[:space:]]*(true|True)" \
    <<< "$MODE_RESULT"; then

    echo "[patrol] 无法进入手动模式。"
    exit 1
fi

echo
echo "========================================"
echo "稳定 origin 已保存，轨迹录制已开始"
echo "========================================"
echo "Q/W/E：左前 / 前进 / 右前"
echo "A/S/D：左转 / 停止 / 右转"
echo "Z/X/C：左后 / 后退 / 右后"
echo "空格：停止"
echo "英文句号：结束录制"
echo
echo "请先确认周围安全，再开始按键。"
echo

set +e
ros2 run patrol_teleop teleop_node
TELEOP_RESULT=$?
set -e

echo
echo "[patrol] 键盘控制已退出，立即停车..."

stop_mode
sleep 0.5

stop_recording
stop_launch
finalize_route

echo
echo "========================================"
echo "路线录制完成"
echo "========================================"
echo "路线文件：$ROUTE_FILE"
echo "日志目录：$RUN_DIR"
echo "键盘退出码：$TELEOP_RESULT"

python3 - "$ROUTE_FILE" <<'PY'
import sys
import yaml

with open(sys.argv[1], "r", encoding="utf-8") as file:
    data = yaml.safe_load(file)

origin = data["origin"]
summary = data["summary"]

print("轨迹点数：", summary["point_count"])
print("轨迹长度：", round(summary["total_length"], 3), "m")
print(
    "origin：",
    f'{origin["latitude"]:.10f},',
    f'{origin["longitude"]:.10f},',
    f'{origin["altitude"]:.3f}',
)
print("origin yaw：", round(origin["yaw_deg"], 3), "deg")
PY

trap - EXIT INT TERM
exit 0
