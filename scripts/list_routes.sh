#!/usr/bin/env bash

set -euo pipefail

ROUTES_DIR="/home/nvidia/patrol_ws/routes"

mkdir -p "$ROUTES_DIR"

python3 - "$ROUTES_DIR" <<'PY'
import sys
from pathlib import Path

import yaml

routes_dir = Path(sys.argv[1])
route_files = sorted(
    routes_dir.glob("*.yaml"),
    key=lambda path: path.stat().st_mtime,
    reverse=True,
)

print("========================================")
print("已保存巡逻路线")
print("========================================")

if not route_files:
    print("暂无路线。")
    print()
    print("录制方式：")
    print("  ./scripts/record_route.sh 路线名称")
    raise SystemExit(0)

for index, path in enumerate(route_files, start=1):
    try:
        data = yaml.safe_load(
            path.read_text(encoding="utf-8")
        )

        if not isinstance(data, dict):
            raise ValueError("YAML 根节点不是 mapping")

        origin = data.get("origin", {})
        waypoints = data.get("waypoints", [])
        summary = data.get("summary", {})

        if not isinstance(origin, dict):
            origin = {}

        if not isinstance(waypoints, list):
            waypoints = []

        if not isinstance(summary, dict):
            summary = {}

        point_count = int(
            summary.get(
                "point_count",
                len(waypoints),
            )
        )

        total_length = float(
            summary.get(
                "total_length",
                0.0,
            )
        )

        latitude = origin.get("latitude")
        longitude = origin.get("longitude")
        altitude = origin.get("altitude")
        yaw_deg = origin.get("yaw_deg")
        format_version = int(
            data.get("format_version", 0)
        )

        valid_route = True
        invalid_reasons = []

        if format_version < 2:
            valid_route = False
            invalid_reasons.append(
                "格式版本低于2"
            )

        if (
            latitude is None
            or longitude is None
            or altitude is None
        ):
            valid_route = False
            invalid_reasons.append(
                "缺少绝对 origin"
            )
        else:
            latitude_value = float(latitude)
            longitude_value = float(longitude)

            if (
                abs(latitude_value) < 1.0e-8
                and abs(longitude_value) < 1.0e-8
            ):
                valid_route = False
                invalid_reasons.append(
                    "origin 为测试值 0,0"
                )

        if len(waypoints) < 2:
            valid_route = False
            invalid_reasons.append(
                "轨迹点少于2个"
            )

        status_text = (
            "可复现"
            if valid_route
            else "不可复现"
        )

        print(
            f"{index}. {path.stem} [{status_text}]"
        )
        print(f"   文件：{path}")

        print(
            f"   轨迹点：{point_count}，"
            f"长度：{total_length:.3f} m"
        )

        if (
            latitude is not None
            and longitude is not None
            and altitude is not None
        ):
            print(
                "   origin："
                f"{float(latitude):.10f}, "
                f"{float(longitude):.10f}, "
                f"{float(altitude):.3f}"
            )
        else:
            print("   origin：缺失")

        if yaw_deg is not None:
            print(
                f"   初始航向："
                f"{float(yaw_deg):.3f}°"
            )

        print(
            f"   格式版本：{format_version}"
        )

        if invalid_reasons:
            print(
                "   原因："
                + "；".join(invalid_reasons)
            )

    except Exception as exc:
        print(f"{index}. {path.stem}")
        print(f"   文件：{path}")
        print(f"   状态：无效路线文件")
        print(f"   原因：{exc}")

    print()

print("复现方式：")
print("  ./scripts/replay_route.sh 路线名称")
PY
