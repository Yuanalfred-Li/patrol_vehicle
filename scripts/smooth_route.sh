#!/usr/bin/env bash

set -Eeuo pipefail

WS="/home/nvidia/patrol_ws"
ROUTES_DIR="$WS/routes"
TOOLS_DIR="$WS/tools"
CONFIG_FILE="$WS/config/route_smoothing.yaml"

INPUT_NAME="${1:-}"
OUTPUT_NAME="${2:-}"

INPUT_NAME="${INPUT_NAME%.yaml}"
OUTPUT_NAME="${OUTPUT_NAME%.yaml}"

if [ -z "$INPUT_NAME" ]; then
    echo "用法："
    echo "  $0 输入路线 [输出路线]"
    echo
    echo "示例："
    echo "  $0 测试路线_raw"
    echo "  $0 测试路线 测试路线平滑版"
    exit 1
fi

case "$INPUT_NAME" in
    *"/"*|*".."*)
        echo "[patrol] 输入路线名称不能包含 / 或 .."
        exit 1
        ;;
esac

if [ -z "$OUTPUT_NAME" ]; then
    case "$INPUT_NAME" in
        *_raw)
            OUTPUT_NAME="${INPUT_NAME%_raw}"
            ;;
        *)
            OUTPUT_NAME="${INPUT_NAME}_smooth"
            ;;
    esac
fi

case "$OUTPUT_NAME" in
    *"/"*|*".."*)
        echo "[patrol] 输出路线名称不能包含 / 或 .."
        exit 1
        ;;
esac

INPUT_FILE="$ROUTES_DIR/${INPUT_NAME}.yaml"
OUTPUT_FILE="$ROUTES_DIR/${OUTPUT_NAME}.yaml"
TEMP_FILE="$ROUTES_DIR/.${OUTPUT_NAME}.yaml.tmp.$$"

if [ ! -f "$INPUT_FILE" ]; then
    echo "[patrol] 输入路线不存在：$INPUT_FILE"
    exit 1
fi

if [ ! -f "$CONFIG_FILE" ]; then
    echo "[patrol] 平滑参数文件不存在：$CONFIG_FILE"
    exit 1
fi

mapfile -t PARAMETERS < <(
    python3 - "$CONFIG_FILE" <<'PY'
import sys
import yaml

with open(sys.argv[1], "r", encoding="utf-8") as file:
    config = yaml.safe_load(file)

keys = [
    "spacing_m",
    "dense_spacing_m",
    "sigma_m",
    "strength",
    "maximum_shift_m",
    "endpoint_hold_m",
    "heading_window_distance_m",
    "maximum_anchor_shift_m",
    "spike_threshold_m",
    "spike_angle_deg",
]

for key in keys:
    if key not in config:
        raise SystemExit(f"平滑配置缺少参数：{key}")

    value = float(config[key])

    if value < 0.0:
        raise SystemExit(f"平滑参数不能为负数：{key}")

    print(value)
PY
)

SPACING="${PARAMETERS[0]}"
DENSE_SPACING="${PARAMETERS[1]}"
SIGMA="${PARAMETERS[2]}"
STRENGTH="${PARAMETERS[3]}"
MAXIMUM_SHIFT="${PARAMETERS[4]}"
ENDPOINT_HOLD="${PARAMETERS[5]}"
HEADING_WINDOW="${PARAMETERS[6]}"
MAXIMUM_ANCHOR_SHIFT="${PARAMETERS[7]}"
SPIKE_THRESHOLD="${PARAMETERS[8]}"
SPIKE_ANGLE="${PARAMETERS[9]}"

if ! python3 - "$STRENGTH" <<'PY'
import sys

value = float(sys.argv[1])

if not 0.0 <= value <= 1.0:
    raise SystemExit("strength 必须处于 0 到 1")
PY
then
    exit 1
fi

cleanup() {
    rm -f "$TEMP_FILE"
}

trap cleanup EXIT INT TERM

echo "========================================"
echo "巡检路线平滑"
echo "========================================"
echo "原始路线：$INPUT_FILE"
echo "正式路线：$OUTPUT_FILE"
echo
echo "轨迹点间距：$SPACING m"
echo "位置平滑尺度：$SIGMA m"
echo "平滑强度：$STRENGTH"
echo "最大偏移：$MAXIMUM_SHIFT m"
echo "航向窗口：前后各 $HEADING_WINDOW m"
echo

python3 "$TOOLS_DIR/smooth_route.py" \
    "$INPUT_FILE" \
    "$TEMP_FILE" \
    --spacing "$SPACING" \
    --dense-spacing "$DENSE_SPACING" \
    --sigma "$SIGMA" \
    --strength "$STRENGTH" \
    --maximum-shift "$MAXIMUM_SHIFT" \
    --endpoint-hold "$ENDPOINT_HOLD" \
    --heading-window-distance "$HEADING_WINDOW" \
    --maximum-anchor-shift "$MAXIMUM_ANCHOR_SHIFT" \
    --spike-threshold "$SPIKE_THRESHOLD" \
    --spike-angle "$SPIKE_ANGLE" \
    --force

mv -f "$TEMP_FILE" "$OUTPUT_FILE"

trap - EXIT INT TERM

echo
echo "[patrol] 正式平滑路线已生成："
echo "  $OUTPUT_FILE"
