#!/usr/bin/env python3
"""
对比巡检小车原始路线 YAML 与平滑后路线 YAML。

示例：
    python3 plot_route_compare.py \
        routes/auto_smooth_test_01_raw.yaml \
        routes/auto_smooth_test_01.yaml

保存图片但不弹出窗口：
    python3 plot_route_compare.py \
        routes/auto_smooth_test_01_raw.yaml \
        routes/auto_smooth_test_01.yaml \
        --output /tmp/auto_smooth_test_01_compare.png \
        --no-show
"""

from __future__ import annotations

import argparse
import math
import os
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import yaml

WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


@dataclass
class RouteData:
    path: Path
    label: str
    points: list[tuple[float, float]]
    source: str
    yaml_data: dict[str, Any]


def geodetic_to_ecef(
    latitude_deg: float,
    longitude_deg: float,
    altitude: float,
) -> tuple[float, float, float]:
    latitude = math.radians(latitude_deg)
    longitude = math.radians(longitude_deg)

    sin_latitude = math.sin(latitude)
    cos_latitude = math.cos(latitude)
    sin_longitude = math.sin(longitude)
    cos_longitude = math.cos(longitude)

    radius = WGS84_A / math.sqrt(
        1.0 - WGS84_E2 * sin_latitude * sin_latitude
    )

    x = (radius + altitude) * cos_latitude * cos_longitude
    y = (radius + altitude) * cos_latitude * sin_longitude
    z = (radius * (1.0 - WGS84_E2) + altitude) * sin_latitude

    return x, y, z


def geodetic_to_enu(
    latitude_deg: float,
    longitude_deg: float,
    altitude: float,
    origin_latitude_deg: float,
    origin_longitude_deg: float,
    origin_altitude: float,
) -> tuple[float, float, float]:
    x, y, z = geodetic_to_ecef(
        latitude_deg,
        longitude_deg,
        altitude,
    )

    origin_x, origin_y, origin_z = geodetic_to_ecef(
        origin_latitude_deg,
        origin_longitude_deg,
        origin_altitude,
    )

    dx = x - origin_x
    dy = y - origin_y
    dz = z - origin_z

    origin_latitude = math.radians(origin_latitude_deg)
    origin_longitude = math.radians(origin_longitude_deg)

    sin_latitude = math.sin(origin_latitude)
    cos_latitude = math.cos(origin_latitude)
    sin_longitude = math.sin(origin_longitude)
    cos_longitude = math.cos(origin_longitude)

    east = -sin_longitude * dx + cos_longitude * dy

    north = (
        -sin_latitude * cos_longitude * dx
        - sin_latitude * sin_longitude * dy
        + cos_latitude * dz
    )

    up = (
        cos_latitude * cos_longitude * dx
        + cos_latitude * sin_longitude * dy
        + sin_latitude * dz
    )

    return east, north, up


def load_yaml(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(f"路线文件不存在：{path}")

    data = yaml.safe_load(path.read_text(encoding="utf-8"))

    if not isinstance(data, dict):
        raise ValueError(f"YAML 根节点必须是映射：{path}")

    return data


def route_points_from_yaml(
    data: dict[str, Any],
) -> tuple[list[tuple[float, float]], str]:
    waypoints = data.get("waypoints")

    if not isinstance(waypoints, list) or len(waypoints) < 2:
        raise ValueError("路线至少需要两个 waypoints")

    origin = data.get("origin")

    can_use_geodetic = (
        isinstance(origin, dict)
        and all(
            key in origin
            for key in ("latitude", "longitude", "altitude")
        )
        and all(
            isinstance(point, dict)
            and all(
                key in point
                for key in ("latitude", "longitude", "altitude")
            )
            for point in waypoints
        )
    )

    points: list[tuple[float, float]] = []

    if can_use_geodetic:
        origin_latitude = float(origin["latitude"])
        origin_longitude = float(origin["longitude"])
        origin_altitude = float(origin["altitude"])

        for index, point in enumerate(waypoints):
            try:
                east, north, _ = geodetic_to_enu(
                    float(point["latitude"]),
                    float(point["longitude"]),
                    float(point["altitude"]),
                    origin_latitude,
                    origin_longitude,
                    origin_altitude,
                )
            except (TypeError, ValueError, KeyError) as exc:
                raise ValueError(
                    f"轨迹点 {index} 经纬度无效：{exc}"
                ) from exc

            points.append((east, north))

        return points, "WGS84 → ENU"

    for index, point in enumerate(waypoints):
        if not isinstance(point, dict):
            raise ValueError(f"轨迹点 {index} 格式无效")

        try:
            x = float(point["x"])
            y = float(point["y"])
        except (TypeError, ValueError, KeyError) as exc:
            raise ValueError(
                f"轨迹点 {index} 缺少有效 x/y：{exc}"
            ) from exc

        points.append((x, y))

    return points, "YAML x/y"


def load_route(path: Path, label: str) -> RouteData:
    data = load_yaml(path)
    points, source = route_points_from_yaml(data)

    return RouteData(
        path=path,
        label=label,
        points=points,
        source=source,
        yaml_data=data,
    )


def route_length(points: list[tuple[float, float]]) -> float:
    return sum(
        math.hypot(
            points[index + 1][0] - points[index][0],
            points[index + 1][1] - points[index][1],
        )
        for index in range(len(points) - 1)
    )


def segment_lengths(
    points: list[tuple[float, float]],
) -> list[float]:
    return [
        math.hypot(
            points[index + 1][0] - points[index][0],
            points[index + 1][1] - points[index][1],
        )
        for index in range(len(points) - 1)
    ]


def normalize_angle_deg(angle: float) -> float:
    while angle > 180.0:
        angle -= 360.0

    while angle <= -180.0:
        angle += 360.0

    return angle


def heading_changes(
    points: list[tuple[float, float]],
) -> list[float]:
    headings: list[float] = []

    for index in range(len(points) - 1):
        dx = points[index + 1][0] - points[index][0]
        dy = points[index + 1][1] - points[index][1]

        if math.hypot(dx, dy) <= 1.0e-9:
            continue

        headings.append(math.degrees(math.atan2(dy, dx)))

    return [
        abs(
            normalize_angle_deg(
                headings[index + 1] - headings[index]
            )
        )
        for index in range(len(headings) - 1)
    ]


def percentile(values: Iterable[float], ratio: float) -> float:
    ordered = sorted(float(value) for value in values)

    if not ordered:
        return 0.0

    if len(ordered) == 1:
        return ordered[0]

    position = (len(ordered) - 1) * ratio
    low = int(math.floor(position))
    high = int(math.ceil(position))

    if low == high:
        return ordered[low]

    weight = position - low
    return ordered[low] * (1.0 - weight) + ordered[high] * weight


def point_to_segment_distance(
    point: tuple[float, float],
    start: tuple[float, float],
    end: tuple[float, float],
) -> float:
    px, py = point
    ax, ay = start
    bx, by = end

    vx = bx - ax
    vy = by - ay
    length_squared = vx * vx + vy * vy

    if length_squared <= 1.0e-12:
        return math.hypot(px - ax, py - ay)

    projection = ((px - ax) * vx + (py - ay) * vy) / length_squared
    projection = max(0.0, min(1.0, projection))

    closest_x = ax + projection * vx
    closest_y = ay + projection * vy

    return math.hypot(px - closest_x, py - closest_y)


def distances_to_route(
    source_points: list[tuple[float, float]],
    target_points: list[tuple[float, float]],
) -> list[float]:
    distances: list[float] = []

    for point in source_points:
        minimum = math.inf

        for index in range(len(target_points) - 1):
            distance = point_to_segment_distance(
                point,
                target_points[index],
                target_points[index + 1],
            )

            if distance < minimum:
                minimum = distance

        distances.append(minimum)

    return distances


def print_route_summary(route: RouteData) -> None:
    lengths = segment_lengths(route.points)
    changes = heading_changes(route.points)

    print(f"\n===== {route.label} =====")
    print("文件：", route.path)
    print("坐标来源：", route.source)
    print("轨迹点数：", len(route.points))
    print("路线长度：", f"{route_length(route.points):.3f} m")
    print("最大相邻点距离：", f"{max(lengths):.3f} m")
    print("相邻点距离95%：", f"{percentile(lengths, 0.95):.3f} m")
    print("航向变化95%：", f"{percentile(changes, 0.95):.2f}°")
    print("最大航向变化：", f"{max(changes) if changes else 0.0:.2f}°")


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="绘制原始路线和平滑路线，并输出简单统计。"
    )

    parser.add_argument(
        "raw_route",
        type=Path,
        help="原始路线 YAML，例如 routes/test_raw.yaml",
    )

    parser.add_argument(
        "smooth_route",
        type=Path,
        nargs="?",
        default=None,
        help="平滑后路线 YAML，例如 routes/test.yaml",
    )

    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="保存 PNG 的路径",
    )

    parser.add_argument(
        "--no-show",
        action="store_true",
        help="只保存图片，不弹出窗口",
    )

    parser.add_argument(
        "--show-points",
        action="store_true",
        help="显示采样点",
    )

    parser.add_argument(
        "--point-step",
        type=int,
        default=5,
        help="显示采样点时每隔多少个点绘制一次，默认5",
    )

    parser.add_argument(
        "--annotate-every",
        type=int,
        default=0,
        help="每隔多少个点标注索引；0表示不标注",
    )

    parser.add_argument(
        "--title",
        default="巡检路线处理前后对比",
        help="图标题",
    )

    return parser.parse_args()


def main() -> int:
    args = parse_arguments()

    if args.no_show or not os.environ.get("DISPLAY"):
        import matplotlib
        matplotlib.use("Agg")

    import matplotlib.pyplot as plt

    raw = load_route(args.raw_route, "原始路线")
    smooth = (
        load_route(args.smooth_route, "平滑路线")
        if args.smooth_route is not None
        else None
    )

    print_route_summary(raw)

    if smooth is not None:
        print_route_summary(smooth)

        raw_to_smooth = distances_to_route(
            raw.points,
            smooth.points,
        )

        print("\n===== 原始路线到平滑路线偏差 =====")
        print("中位数：", f"{statistics.median(raw_to_smooth):.3f} m")
        print("95%：", f"{percentile(raw_to_smooth, 0.95):.3f} m")
        print("最大值：", f"{max(raw_to_smooth):.3f} m")

    figure, axis = plt.subplots(figsize=(10, 8))

    raw_x = [point[0] for point in raw.points]
    raw_y = [point[1] for point in raw.points]

    axis.plot(
        raw_x,
        raw_y,
        linewidth=1.4,
        alpha=0.65,
        label=f"原始路线（{len(raw.points)}点）",
    )

    if args.show_points:
        step = max(1, args.point_step)
        axis.scatter(raw_x[::step], raw_y[::step], s=14, alpha=0.65)

    if smooth is not None:
        smooth_x = [point[0] for point in smooth.points]
        smooth_y = [point[1] for point in smooth.points]

        axis.plot(
            smooth_x,
            smooth_y,
            linewidth=2.2,
            label=f"平滑路线（{len(smooth.points)}点）",
        )

        if args.show_points:
            step = max(1, args.point_step)
            axis.scatter(smooth_x[::step], smooth_y[::step], s=18)

    axis.scatter(
        [raw.points[0][0]],
        [raw.points[0][1]],
        s=80,
        marker="o",
        label="起点",
        zorder=5,
    )

    axis.scatter(
        [raw.points[-1][0]],
        [raw.points[-1][1]],
        s=90,
        marker="X",
        label="原始终点",
        zorder=5,
    )

    if smooth is not None:
        axis.scatter(
            [smooth.points[-1][0]],
            [smooth.points[-1][1]],
            s=90,
            marker="*",
            label="平滑终点",
            zorder=6,
        )

    annotate_every = max(0, args.annotate_every)

    if annotate_every > 0:
        for index in range(0, len(raw.points), annotate_every):
            axis.annotate(str(index), raw.points[index], fontsize=8)

    axis.set_title(args.title)
    axis.set_xlabel("East / X (m)")
    axis.set_ylabel("North / Y (m)")
    axis.axis("equal")
    axis.grid(True, linestyle="--", alpha=0.4)
    axis.legend()
    figure.tight_layout()

    output = args.output

    if output is None:
        output = args.raw_route.with_name(
            f"{args.raw_route.stem}_compare.png"
        )

    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=180)

    print("\n图片已保存：", output)

    if not args.no_show and os.environ.get("DISPLAY"):
        plt.show()
    else:
        plt.close(figure)

    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"错误：{exc}", file=sys.stderr)
        raise SystemExit(1)
