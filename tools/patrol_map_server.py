#!/usr/bin/env python3
"""
巡检小车地图前端（V1.1 直线+圆弧）

功能：
1. 实时显示蓝色小车图标、定位状态和航向；
2. 显示车辆实际行驶轨迹（蓝色线）；
3. 在高德地图上绘制/编辑开放或闭环路线，预览为绿色线；
4. 将设计路线保存为当前巡迹系统可读取的 YAML。

本版本不实现点对点导航，也不直接向 /vehicle/command 发布命令。
保存的路线继续通过现有 replay_route.sh 执行，因此寻找路线起点时
仍使用现有 Hybrid A* 入轨流程。

运行前：

    source /opt/ros/humble/setup.bash
    source /home/nvidia/ros2_humble_main/install/setup.bash
    source /home/nvidia/patrol_ws/install/setup.bash

    export AMAP_JS_KEY='你的高德 JS API Key'
    export AMAP_SECURITY_CODE='你的高德安全密钥'

运行：

    python3 tools/patrol_map_server.py

浏览器：

    http://JETSON_IP:8080
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import signal
import threading
import time
from dataclasses import dataclass, field
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Iterable, Optional
from urllib.parse import parse_qs, unquote, urlparse

import rclpy
import yaml
from patrol_interfaces.msg import LocalizationStatus, TaskStatus
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3
EARTH_RADIUS = 6378137.0


def finite_float(value: Any, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} 不是有效数字：{value!r}") from exc

    if not math.isfinite(result):
        raise ValueError(f"{name} 不是有限数字：{value!r}")

    return result


def normalize_angle_deg(angle: float) -> float:
    return float(angle % 360.0)


def sanitize_route_name(value: str) -> str:
    name = str(value).strip()
    name = re.sub(r"[^\w\u4e00-\u9fff.\-]+", "_", name)
    name = name.strip("._-")

    if not name:
        raise ValueError("路线名称为空")

    if len(name) > 80:
        raise ValueError("路线名称过长")

    return name


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
    z = (
        radius * (1.0 - WGS84_E2) + altitude
    ) * sin_latitude

    return x, y, z


def ecef_to_geodetic(
    x: float,
    y: float,
    z: float,
) -> tuple[float, float, float]:
    longitude = math.atan2(y, x)
    horizontal = math.hypot(x, y)

    latitude = math.atan2(
        z,
        horizontal * (1.0 - WGS84_E2),
    )
    altitude = 0.0

    for _ in range(20):
        sin_latitude = math.sin(latitude)
        radius = WGS84_A / math.sqrt(
            1.0 - WGS84_E2 * sin_latitude * sin_latitude
        )

        cos_latitude = math.cos(latitude)

        if abs(cos_latitude) > 1.0e-12:
            altitude = horizontal / cos_latitude - radius
        else:
            altitude = (
                z / max(abs(sin_latitude), 1.0e-12)
                - radius * (1.0 - WGS84_E2)
            )

        denominator = horizontal * (
            1.0
            - WGS84_E2
            * radius
            / max(radius + altitude, 1.0)
        )

        updated = math.atan2(z, denominator)

        if abs(updated - latitude) < 1.0e-13:
            latitude = updated
            break

        latitude = updated

    return (
        math.degrees(latitude),
        math.degrees(longitude),
        altitude,
    )


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

    latitude0 = math.radians(origin_latitude_deg)
    longitude0 = math.radians(origin_longitude_deg)

    sin_latitude = math.sin(latitude0)
    cos_latitude = math.cos(latitude0)
    sin_longitude = math.sin(longitude0)
    cos_longitude = math.cos(longitude0)

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


def enu_to_geodetic(
    east: float,
    north: float,
    up: float,
    origin_latitude_deg: float,
    origin_longitude_deg: float,
    origin_altitude: float,
) -> tuple[float, float, float]:
    origin_x, origin_y, origin_z = geodetic_to_ecef(
        origin_latitude_deg,
        origin_longitude_deg,
        origin_altitude,
    )

    latitude0 = math.radians(origin_latitude_deg)
    longitude0 = math.radians(origin_longitude_deg)

    sin_latitude = math.sin(latitude0)
    cos_latitude = math.cos(latitude0)
    sin_longitude = math.sin(longitude0)
    cos_longitude = math.cos(longitude0)

    dx = (
        -sin_longitude * east
        - sin_latitude * cos_longitude * north
        + cos_latitude * cos_longitude * up
    )
    dy = (
        cos_longitude * east
        - sin_latitude * sin_longitude * north
        + cos_latitude * sin_longitude * up
    )
    dz = cos_latitude * north + sin_latitude * up

    return ecef_to_geodetic(
        origin_x + dx,
        origin_y + dy,
        origin_z + dz,
    )


def horizontal_distance_wgs84(
    first: tuple[float, float],
    second: tuple[float, float],
) -> float:
    longitude0 = math.radians(0.5 * (first[0] + second[0]))
    del longitude0  # 仅用于保持参数含义清晰

    latitude0 = math.radians(0.5 * (first[1] + second[1]))
    dx = (
        EARTH_RADIUS
        * math.cos(latitude0)
        * math.radians(second[0] - first[0])
    )
    dy = EARTH_RADIUS * math.radians(second[1] - first[1])

    return math.hypot(dx, dy)


# GCJ-02 转换。地图仅用于国内项目的本地可视化与路线设计。
GCJ_A = 6378245.0
GCJ_EE = 0.00669342162296594323


def outside_china(longitude: float, latitude: float) -> bool:
    return not (
        72.004 <= longitude <= 137.8347
        and 0.8293 <= latitude <= 55.8271
    )


def transform_latitude(x: float, y: float) -> float:
    value = (
        -100.0
        + 2.0 * x
        + 3.0 * y
        + 0.2 * y * y
        + 0.1 * x * y
        + 0.2 * math.sqrt(abs(x))
    )
    value += (
        20.0 * math.sin(6.0 * x * math.pi)
        + 20.0 * math.sin(2.0 * x * math.pi)
    ) * 2.0 / 3.0
    value += (
        20.0 * math.sin(y * math.pi)
        + 40.0 * math.sin(y / 3.0 * math.pi)
    ) * 2.0 / 3.0
    value += (
        160.0 * math.sin(y / 12.0 * math.pi)
        + 320.0 * math.sin(y * math.pi / 30.0)
    ) * 2.0 / 3.0
    return value


def transform_longitude(x: float, y: float) -> float:
    value = (
        300.0
        + x
        + 2.0 * y
        + 0.1 * x * x
        + 0.1 * x * y
        + 0.1 * math.sqrt(abs(x))
    )
    value += (
        20.0 * math.sin(6.0 * x * math.pi)
        + 20.0 * math.sin(2.0 * x * math.pi)
    ) * 2.0 / 3.0
    value += (
        20.0 * math.sin(x * math.pi)
        + 40.0 * math.sin(x / 3.0 * math.pi)
    ) * 2.0 / 3.0
    value += (
        150.0 * math.sin(x / 12.0 * math.pi)
        + 300.0 * math.sin(x / 30.0 * math.pi)
    ) * 2.0 / 3.0
    return value


def wgs84_to_gcj02(
    longitude: float,
    latitude: float,
) -> tuple[float, float]:
    if outside_china(longitude, latitude):
        return longitude, latitude

    delta_latitude = transform_latitude(
        longitude - 105.0,
        latitude - 35.0,
    )
    delta_longitude = transform_longitude(
        longitude - 105.0,
        latitude - 35.0,
    )

    rad_latitude = math.radians(latitude)
    magic = math.sin(rad_latitude)
    magic = 1.0 - GCJ_EE * magic * magic
    sqrt_magic = math.sqrt(magic)

    delta_latitude = (
        delta_latitude
        * 180.0
        / (
            (GCJ_A * (1.0 - GCJ_EE))
            / (magic * sqrt_magic)
            * math.pi
        )
    )
    delta_longitude = (
        delta_longitude
        * 180.0
        / (
            GCJ_A
            / sqrt_magic
            * math.cos(rad_latitude)
            * math.pi
        )
    )

    return (
        longitude + delta_longitude,
        latitude + delta_latitude,
    )


def gcj02_to_wgs84_exact(
    longitude: float,
    latitude: float,
) -> tuple[float, float]:
    if outside_china(longitude, latitude):
        return longitude, latitude

    minimum_longitude = longitude - 0.02
    maximum_longitude = longitude + 0.02
    minimum_latitude = latitude - 0.02
    maximum_latitude = latitude + 0.02

    result_longitude = longitude
    result_latitude = latitude

    for _ in range(40):
        result_longitude = (
            minimum_longitude + maximum_longitude
        ) * 0.5
        result_latitude = (
            minimum_latitude + maximum_latitude
        ) * 0.5

        converted_longitude, converted_latitude = (
            wgs84_to_gcj02(
                result_longitude,
                result_latitude,
            )
        )

        longitude_error = (
            converted_longitude - longitude
        )
        latitude_error = converted_latitude - latitude

        if (
            abs(longitude_error) < 1.0e-8
            and abs(latitude_error) < 1.0e-8
        ):
            break

        if longitude_error > 0.0:
            maximum_longitude = result_longitude
        else:
            minimum_longitude = result_longitude

        if latitude_error > 0.0:
            maximum_latitude = result_latitude
        else:
            minimum_latitude = result_latitude

    return result_longitude, result_latitude


def remove_duplicate_points(
    points: list[tuple[float, float]],
    minimum_distance: float,
    closed: bool,
) -> list[tuple[float, float]]:
    if not points:
        return []

    result = [points[0]]

    for point in points[1:]:
        if math.hypot(
            point[0] - result[-1][0],
            point[1] - result[-1][1],
        ) >= minimum_distance:
            result.append(point)

    if (
        closed
        and len(result) >= 2
        and math.hypot(
            result[0][0] - result[-1][0],
            result[0][1] - result[-1][1],
        ) < minimum_distance
    ):
        result.pop()

    return result


def build_fillet_curve(
    keypoints: list[tuple[float, float]],
    closed: bool,
    radius: float,
    dense_spacing: float,
) -> list[tuple[float, float]]:
    """Build straight segments joined by tangent circular arcs.

    The path remains exactly straight away from corners.  Every valid corner
    is replaced by a circular fillet with the requested radius.  If adjacent
    fillets cannot fit on a segment, the route is rejected instead of silently
    shrinking below the requested vehicle turning radius.
    """
    count = len(keypoints)

    if closed and count < 3:
        raise ValueError("闭环路线至少需要 3 个关键点")

    if not closed and count < 2:
        raise ValueError("开放路线至少需要 2 个关键点")

    radius = max(0.01, float(radius))
    dense_spacing = max(0.03, float(dense_spacing))

    corner_info: list[Optional[dict[str, Any]]] = [
        None for _ in range(count)
    ]

    corner_indices = (
        range(count)
        if closed
        else range(1, count - 1)
    )

    for index in corner_indices:
        previous = keypoints[(index - 1) % count]
        current = keypoints[index]
        following = keypoints[(index + 1) % count]

        incoming_x = current[0] - previous[0]
        incoming_y = current[1] - previous[1]
        outgoing_x = following[0] - current[0]
        outgoing_y = following[1] - current[1]

        incoming_length = math.hypot(
            incoming_x,
            incoming_y,
        )
        outgoing_length = math.hypot(
            outgoing_x,
            outgoing_y,
        )

        if incoming_length < 1.0e-6 or outgoing_length < 1.0e-6:
            raise ValueError(
                f"关键点 {index} 附近存在零长度线段"
            )

        incoming_unit = (
            incoming_x / incoming_length,
            incoming_y / incoming_length,
        )
        outgoing_unit = (
            outgoing_x / outgoing_length,
            outgoing_y / outgoing_length,
        )

        dot_value = max(
            -1.0,
            min(
                1.0,
                incoming_unit[0] * outgoing_unit[0]
                + incoming_unit[1] * outgoing_unit[1],
            ),
        )
        turn_angle = math.acos(dot_value)
        turn_cross = (
            incoming_unit[0] * outgoing_unit[1]
            - incoming_unit[1] * outgoing_unit[0]
        )

        # Collinear continuation: keep the point on a straight line.
        if turn_angle < math.radians(1.0):
            corner_info[index] = {
                "tangent_in": current,
                "tangent_out": current,
                "tangent_distance": 0.0,
                "arc": [current],
            }
            continue

        if turn_angle > math.radians(175.0):
            raise ValueError(
                f"关键点 {index} 接近掉头，无法用单个圆弧连接"
            )

        tangent_distance = radius * math.tan(
            0.5 * turn_angle
        )

        tangent_in = (
            current[0]
            - incoming_unit[0] * tangent_distance,
            current[1]
            - incoming_unit[1] * tangent_distance,
        )
        tangent_out = (
            current[0]
            + outgoing_unit[0] * tangent_distance,
            current[1]
            + outgoing_unit[1] * tangent_distance,
        )

        turn_sign = 1.0 if turn_cross > 0.0 else -1.0
        left_normal = (
            -incoming_unit[1],
            incoming_unit[0],
        )
        center = (
            tangent_in[0]
            + turn_sign * radius * left_normal[0],
            tangent_in[1]
            + turn_sign * radius * left_normal[1],
        )

        start_angle = math.atan2(
            tangent_in[1] - center[1],
            tangent_in[0] - center[0],
        )
        end_angle = math.atan2(
            tangent_out[1] - center[1],
            tangent_out[0] - center[0],
        )
        delta_angle = math.atan2(
            math.sin(end_angle - start_angle),
            math.cos(end_angle - start_angle),
        )

        if turn_sign > 0.0 and delta_angle < 0.0:
            delta_angle += 2.0 * math.pi
        elif turn_sign < 0.0 and delta_angle > 0.0:
            delta_angle -= 2.0 * math.pi

        arc_length = abs(delta_angle) * radius
        arc_samples = max(
            4,
            int(math.ceil(
                arc_length / dense_spacing
            )),
        )
        arc = []

        for sample in range(arc_samples + 1):
            angle = (
                start_angle
                + delta_angle * sample / arc_samples
            )
            arc.append((
                center[0] + radius * math.cos(angle),
                center[1] + radius * math.sin(angle),
            ))

        corner_info[index] = {
            "tangent_in": tangent_in,
            "tangent_out": tangent_out,
            "tangent_distance": tangent_distance,
            "arc": arc,
        }

    # Make sure the requested radii fit between adjacent keypoints.
    segment_count = count if closed else count - 1

    for index in range(segment_count):
        next_index = (index + 1) % count
        segment_length = math.hypot(
            keypoints[next_index][0] - keypoints[index][0],
            keypoints[next_index][1] - keypoints[index][1],
        )
        used_at_start = (
            float(corner_info[index]["tangent_distance"])
            if corner_info[index] is not None
            else 0.0
        )
        used_at_end = (
            float(corner_info[next_index]["tangent_distance"])
            if corner_info[next_index] is not None
            else 0.0
        )
        required = used_at_start + used_at_end

        if required > segment_length - 0.05:
            raise ValueError(
                f"关键点 {index}->{next_index} 的线段只有 "
                f"{segment_length:.2f} m，无法容纳半径 "
                f"{radius:.2f} m 的相邻转弯；请拉开关键点或减小半径"
            )

    result: list[tuple[float, float]] = []

    def append_point(point: tuple[float, float]) -> None:
        if not result or math.hypot(
            point[0] - result[-1][0],
            point[1] - result[-1][1],
        ) > 1.0e-9:
            result.append(point)

    if not closed:
        append_point(keypoints[0])

        for index in range(1, count - 1):
            info = corner_info[index]
            assert info is not None
            append_point(info["tangent_in"])

            for point in info["arc"][1:]:
                append_point(point)

        append_point(keypoints[-1])
        return result

    first_info = corner_info[0]
    assert first_info is not None
    append_point(first_info["tangent_out"])

    for index in list(range(1, count)) + [0]:
        info = corner_info[index]
        assert info is not None
        append_point(info["tangent_in"])

        for point in info["arc"][1:]:
            append_point(point)

    return result


def resample_curve(
    points: list[tuple[float, float]],
    closed: bool,
    spacing: float,
) -> list[tuple[float, float]]:
    if closed:
        source = points + [points[0]]
    else:
        source = list(points)

    filtered = [source[0]]

    for point in source[1:]:
        if math.hypot(
            point[0] - filtered[-1][0],
            point[1] - filtered[-1][1],
        ) > 1.0e-8:
            filtered.append(point)

    segment_lengths = [
        math.hypot(
            filtered[index + 1][0] - filtered[index][0],
            filtered[index + 1][1] - filtered[index][1],
        )
        for index in range(len(filtered) - 1)
    ]

    cumulative = [0.0]

    for length in segment_lengths:
        cumulative.append(cumulative[-1] + length)

    total_length = cumulative[-1]

    if total_length <= spacing:
        raise ValueError(
            f"路线总长度 {total_length:.2f} m 过短"
        )

    if closed:
        sample_count = max(
            4,
            int(math.ceil(total_length / spacing)),
        )
        targets = [
            total_length * index / sample_count
            for index in range(sample_count)
        ]
    else:
        sample_count = max(
            2,
            int(math.ceil(total_length / spacing)) + 1,
        )
        targets = [
            total_length * index / (sample_count - 1)
            for index in range(sample_count)
        ]

    result = []
    segment_index = 0

    for target in targets:
        while (
            segment_index + 1 < len(cumulative)
            and cumulative[segment_index + 1]
            < target
        ):
            segment_index += 1

        segment_index = min(
            segment_index,
            len(segment_lengths) - 1,
        )

        segment_start = cumulative[segment_index]
        segment_length = segment_lengths[segment_index]

        ratio = (
            0.0
            if segment_length <= 1.0e-12
            else (
                target - segment_start
            ) / segment_length
        )

        start = filtered[segment_index]
        end = filtered[segment_index + 1]

        result.append((
            start[0] + ratio * (end[0] - start[0]),
            start[1] + ratio * (end[1] - start[1]),
        ))

    if closed:
        result.append(result[0])

    return result


def polyline_length(
    points: list[tuple[float, float]],
) -> float:
    return sum(
        math.hypot(
            points[index + 1][0] - points[index][0],
            points[index + 1][1] - points[index][1],
        )
        for index in range(len(points) - 1)
    )


def cross(
    first: tuple[float, float],
    second: tuple[float, float],
    third: tuple[float, float],
) -> float:
    return (
        (second[0] - first[0])
        * (third[1] - first[1])
        - (second[1] - first[1])
        * (third[0] - first[0])
    )


def segments_intersect(
    a: tuple[float, float],
    b: tuple[float, float],
    c: tuple[float, float],
    d: tuple[float, float],
) -> bool:
    epsilon = 1.0e-9
    first = cross(a, b, c)
    second = cross(a, b, d)
    third = cross(c, d, a)
    fourth = cross(c, d, b)

    return (
        first * second < -epsilon
        and third * fourth < -epsilon
    )


def find_self_intersections(
    points: list[tuple[float, float]],
) -> list[tuple[int, int]]:
    segment_count = len(points) - 1
    result = []

    for first_index in range(segment_count):
        for second_index in range(
            first_index + 1,
            segment_count,
        ):
            if abs(first_index - second_index) <= 1:
                continue

            if (
                first_index == 0
                and second_index == segment_count - 1
                and points[0] == points[-1]
            ):
                continue

            if segments_intersect(
                points[first_index],
                points[first_index + 1],
                points[second_index],
                points[second_index + 1],
            ):
                result.append(
                    (first_index, second_index)
                )

    return result


def circumradius(
    first: tuple[float, float],
    middle: tuple[float, float],
    last: tuple[float, float],
) -> float:
    a = math.hypot(
        middle[0] - first[0],
        middle[1] - first[1],
    )
    b = math.hypot(
        last[0] - middle[0],
        last[1] - middle[1],
    )
    c = math.hypot(
        last[0] - first[0],
        last[1] - first[1],
    )

    twice_area = abs(cross(first, middle, last))

    if twice_area <= 1.0e-8:
        return math.inf

    return a * b * c / (2.0 * twice_area)


def measured_turning_radii(
    points: list[tuple[float, float]],
    closed: bool,
    spacing: float,
) -> list[float]:
    if len(points) < 5:
        return []

    unique = points[:-1] if closed else points
    count = len(unique)
    window = max(1, int(round(1.0 / spacing)))
    radii = []

    for index in range(count):
        if not closed and (
            index < window
            or index + window >= count
        ):
            continue

        first = unique[
            (index - window) % count
        ]
        middle = unique[index]
        last = unique[
            (index + window) % count
        ]

        radius = circumradius(
            first,
            middle,
            last,
        )

        if math.isfinite(radius):
            radii.append(radius)

    return radii


def percentile(
    values: Iterable[float],
    ratio: float,
) -> float:
    ordered = sorted(values)

    if not ordered:
        return math.inf

    position = (len(ordered) - 1) * ratio
    low = int(math.floor(position))
    high = int(math.ceil(position))

    if low == high:
        return ordered[low]

    weight = position - low
    return (
        ordered[low] * (1.0 - weight)
        + ordered[high] * weight
    )


@dataclass
class SharedState:
    trace_spacing_m: float
    trace_max_points: int
    lock: threading.Lock = field(
        default_factory=threading.Lock
    )
    localization: Optional[dict[str, Any]] = None
    mission: Optional[dict[str, Any]] = None
    trace: list[list[float]] = field(
        default_factory=list
    )
    trace_epoch: int = 0

    def update_localization(
        self,
        message: LocalizationStatus,
    ) -> None:
        payload = {
            "valid": bool(message.valid),
            "gps_status": int(message.gps_status),
            "nsv1": int(message.nsv1),
            "nsv2": int(message.nsv2),
            "latitude": float(message.latitude),
            "longitude": float(message.longitude),
            "altitude": float(message.altitude),
            "east": float(message.east),
            "north": float(message.north),
            "heading_deg": float(message.yaw_deg),
            "reason": str(message.reason),
            "received_at": time.time(),
        }

        with self.lock:
            self.localization = payload

            if not payload["valid"]:
                return

            point = [
                payload["longitude"],
                payload["latitude"],
            ]

            if not all(
                math.isfinite(value)
                for value in point
            ):
                return

            should_append = not self.trace

            if self.trace:
                should_append = (
                    horizontal_distance_wgs84(
                        (
                            self.trace[-1][0],
                            self.trace[-1][1],
                        ),
                        (point[0], point[1]),
                    )
                    >= self.trace_spacing_m
                )

            if should_append:
                self.trace.append(point)

                if len(self.trace) > self.trace_max_points:
                    remove_count = (
                        len(self.trace)
                        - self.trace_max_points
                    )
                    del self.trace[:remove_count]
                    self.trace_epoch += 1

    def update_mission(
        self,
        message: TaskStatus,
    ) -> None:
        with self.lock:
            self.mission = {
                "state": int(message.state),
                "task": str(message.task),
                "message": str(message.message),
                "progress": float(message.progress),
                "received_at": time.time(),
            }

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            return {
                "localization": (
                    dict(self.localization)
                    if self.localization is not None
                    else None
                ),
                "mission": (
                    dict(self.mission)
                    if self.mission is not None
                    else None
                ),
                "trace_count": len(self.trace),
                "trace_epoch": self.trace_epoch,
                "server_time": time.time(),
            }

    def trace_after(
        self,
        index: int,
        epoch: int,
    ) -> dict[str, Any]:
        with self.lock:
            if epoch != self.trace_epoch:
                return {
                    "epoch": self.trace_epoch,
                    "start_index": 0,
                    "points": list(self.trace),
                }

            start = max(
                0,
                min(index, len(self.trace)),
            )

            return {
                "epoch": self.trace_epoch,
                "start_index": start,
                "points": list(self.trace[start:]),
            }

    def clear_trace(self) -> None:
        with self.lock:
            self.trace.clear()
            self.trace_epoch += 1

    def latest_altitude(self) -> Optional[float]:
        with self.lock:
            if self.localization is None:
                return None

            altitude = self.localization.get("altitude")

            if (
                altitude is None
                or not math.isfinite(float(altitude))
            ):
                return None

            return float(altitude)


class PatrolMapBridge(Node):
    def __init__(
        self,
        shared_state: SharedState,
    ) -> None:
        super().__init__("patrol_map_bridge")

        self.shared_state = shared_state

        self.create_subscription(
            LocalizationStatus,
            "/patrol/localization_status",
            self.localization_callback,
            20,
        )

        self.create_subscription(
            TaskStatus,
            "/patrol/mission/status",
            self.mission_callback,
            10,
        )

        self.get_logger().info(
            "patrol map bridge ready; "
            "read-only visualization and route saving"
        )

    def localization_callback(
        self,
        message: LocalizationStatus,
    ) -> None:
        self.shared_state.update_localization(
            message
        )

    def mission_callback(
        self,
        message: TaskStatus,
    ) -> None:
        self.shared_state.update_mission(message)


class RouteStore:
    def __init__(
        self,
        routes_directory: Path,
        shared_state: SharedState,
    ) -> None:
        self.routes_directory = routes_directory
        self.shared_state = shared_state
        self.lock = threading.Lock()
        self.routes_directory.mkdir(
            parents=True,
            exist_ok=True,
        )

    def list_routes(self) -> list[dict[str, Any]]:
        result = []

        for path in sorted(
            self.routes_directory.glob("*.yaml")
        ):
            if path.name.endswith("_raw.yaml"):
                continue

            try:
                data = yaml.safe_load(
                    path.read_text(encoding="utf-8")
                )

                summary = (
                    data.get("summary", {})
                    if isinstance(data, dict)
                    else {}
                )

                result.append({
                    "name": path.stem,
                    "filename": path.name,
                    "point_count": int(
                        summary.get("point_count", 0)
                    ),
                    "total_length": float(
                        summary.get("total_length", 0.0)
                    ),
                    "closed_loop": bool(
                        summary.get("closed_loop", False)
                    ),
                    "modified_at": path.stat().st_mtime,
                })
            except Exception:
                result.append({
                    "name": path.stem,
                    "filename": path.name,
                    "invalid": True,
                    "modified_at": path.stat().st_mtime,
                })

        return result

    def load_route(
        self,
        name: str,
    ) -> dict[str, Any]:
        safe_name = sanitize_route_name(name)
        path = self.routes_directory / (
            safe_name + ".yaml"
        )

        if not path.is_file():
            raise FileNotFoundError(
                f"路线不存在：{safe_name}"
            )

        data = yaml.safe_load(
            path.read_text(encoding="utf-8")
        )

        if not isinstance(data, dict):
            raise ValueError("路线 YAML 根节点无效")

        waypoints = data.get("waypoints")

        if not isinstance(waypoints, list):
            raise ValueError("路线没有 waypoints")

        points = []

        for index, point in enumerate(waypoints):
            if not isinstance(point, dict):
                raise ValueError(
                    f"waypoint {index} 无效"
                )

            points.append([
                finite_float(
                    point.get("longitude"),
                    f"waypoint {index}.longitude",
                ),
                finite_float(
                    point.get("latitude"),
                    f"waypoint {index}.latitude",
                ),
            ])

        return {
            "name": safe_name,
            "closed_loop": bool(
                data.get(
                    "summary",
                    {},
                ).get("closed_loop", False)
            ),
            "summary": data.get("summary", {}),
            "points_wgs84": points,
        }

    def save_route(
        self,
        request: dict[str, Any],
    ) -> dict[str, Any]:
        route_name = sanitize_route_name(
            request.get("name", "")
        )
        closed = bool(
            request.get("closed", True)
        )

        raw_keypoints = request.get(
            "keypoints_gcj02"
        )

        if not isinstance(raw_keypoints, list):
            raise ValueError(
                "缺少 keypoints_gcj02"
            )

        required_count = 3 if closed else 2

        if len(raw_keypoints) < required_count:
            raise ValueError(
                f"当前路线至少需要 {required_count} 个关键点"
            )

        keypoints_wgs84 = []

        for index, point in enumerate(raw_keypoints):
            if (
                not isinstance(point, list)
                or len(point) < 2
            ):
                raise ValueError(
                    f"关键点 {index} 格式无效"
                )

            longitude_gcj = finite_float(
                point[0],
                f"关键点 {index} 经度",
            )
            latitude_gcj = finite_float(
                point[1],
                f"关键点 {index} 纬度",
            )

            keypoints_wgs84.append(
                gcj02_to_wgs84_exact(
                    longitude_gcj,
                    latitude_gcj,
                )
            )

        altitude = request.get("altitude")

        if altitude is None:
            altitude = self.shared_state.latest_altitude()

        if altitude is None:
            altitude = 0.0

        altitude = finite_float(
            altitude,
            "路线高度",
        )

        spacing = max(
            0.10,
            min(
                2.0,
                finite_float(
                    request.get("spacing_m", 0.40),
                    "spacing_m",
                ),
            ),
        )
        minimum_radius = max(
            0.50,
            finite_float(
                request.get(
                    "minimum_turning_radius_m",
                    1.50,
                ),
                "minimum_turning_radius_m",
            ),
        )

        origin_longitude = keypoints_wgs84[0][0]
        origin_latitude = keypoints_wgs84[0][1]

        keypoints_enu = []

        for longitude, latitude in keypoints_wgs84:
            east, north, _ = geodetic_to_enu(
                latitude,
                longitude,
                altitude,
                origin_latitude,
                origin_longitude,
                altitude,
            )
            keypoints_enu.append((east, north))

        keypoints_enu = remove_duplicate_points(
            keypoints_enu,
            minimum_distance=0.20,
            closed=closed,
        )

        if len(keypoints_enu) < required_count:
            raise ValueError(
                "去除重复点后关键点数量不足"
            )

        curve = build_fillet_curve(
            keypoints_enu,
            closed=closed,
            radius=minimum_radius,
            dense_spacing=0.08,
        )
        route_points = resample_curve(
            curve,
            closed=closed,
            spacing=spacing,
        )

        intersections = find_self_intersections(
            route_points
        )
        radii = measured_turning_radii(
            route_points,
            closed=closed,
            spacing=spacing,
        )

        measured_minimum_radius = (
            min(radii)
            if radii
            else math.inf
        )
        radius_p05 = percentile(radii, 0.05)

        warnings = []

        if intersections:
            warnings.append(
                "路线存在自交："
                + ", ".join(
                    f"{first}-{second}"
                    for first, second in intersections[:8]
                )
            )

        if (
            math.isfinite(measured_minimum_radius)
            and measured_minimum_radius
            < 0.80 * minimum_radius
        ):
            warnings.append(
                "最小转弯半径不足："
                f"{measured_minimum_radius:.2f} m "
                f"< {minimum_radius:.2f} m"
            )

        if warnings and not bool(
            request.get("allow_warnings", False)
        ):
            return {
                "success": False,
                "requires_confirmation": True,
                "warnings": warnings,
                "preview_wgs84": self.points_to_wgs84(
                    route_points,
                    origin_latitude,
                    origin_longitude,
                    altitude,
                ),
                "statistics": {
                    "point_count": len(route_points),
                    "total_length_m": polyline_length(
                        route_points
                    ),
                    "minimum_turning_radius_m": (
                        None
                        if not math.isfinite(
                            measured_minimum_radius
                        )
                        else measured_minimum_radius
                    ),
                    "turning_radius_p05_m": (
                        None
                        if not math.isfinite(radius_p05)
                        else radius_p05
                    ),
                    "self_intersection_count": len(
                        intersections
                    ),
                },
            }

        waypoints = self.build_waypoints(
            route_points,
            origin_latitude,
            origin_longitude,
            altitude,
            closed,
        )

        total_length = polyline_length(
            route_points
        )
        first_heading = waypoints[0][
            "heading_deg"
        ]

        output_data = {
            "format_version": 2,
            "frame_id": "patrol_map",
            "coordinate_system": {
                "geodetic": "WGS84",
                "local": "ENU",
                "origin_source": (
                    "map_designed_first_waypoint"
                ),
                "map_display": "AMap_GCJ-02",
                "closed_loop": closed,
            },
            "origin": {
                "latitude": origin_latitude,
                "longitude": origin_longitude,
                "altitude": altitude,
                "yaw_deg": first_heading,
            },
            "route_design": {
                "source": "patrol_map_frontend_v1_1",
                "keypoint_count": len(keypoints_enu),
                "spacing_m": spacing,
                "minimum_turning_radius_required_m": (
                    minimum_radius
                ),
                "minimum_turning_radius_measured_m": (
                    None
                    if not math.isfinite(
                        measured_minimum_radius
                    )
                    else measured_minimum_radius
                ),
                "turning_radius_p05_m": (
                    None
                    if not math.isfinite(radius_p05)
                    else radius_p05
                ),
                "geometry_mode": "straight_segments_with_circular_fillets",
                "corner_radius_m": minimum_radius,
                "self_intersection_count": len(
                    intersections
                ),
                "created_at_unix": time.time(),
            },
            "summary": {
                "point_count": len(waypoints),
                "total_length": total_length,
                "closed_loop": closed,
            },
            "end_pose": {
                "latitude": waypoints[-1][
                    "latitude"
                ],
                "longitude": waypoints[-1][
                    "longitude"
                ],
                "altitude": waypoints[-1][
                    "altitude"
                ],
                "yaw_deg": waypoints[-1][
                    "heading_deg"
                ],
                "x": waypoints[-1]["x"],
                "y": waypoints[-1]["y"],
                "z": waypoints[-1]["z"],
            },
            "end_pose_estimation": {
                "source": (
                    "map_designed_closed_loop"
                    if closed
                    else "map_designed_endpoint"
                ),
            },
            "waypoints": waypoints,
        }

        path = self.routes_directory / (
            route_name + ".yaml"
        )
        temporary_path = path.with_suffix(
            ".yaml.tmp"
        )

        with self.lock:
            temporary_path.write_text(
                yaml.safe_dump(
                    output_data,
                    allow_unicode=True,
                    sort_keys=False,
                ),
                encoding="utf-8",
            )
            temporary_path.replace(path)

        return {
            "success": True,
            "name": route_name,
            "path": str(path),
            "warnings": warnings,
            "preview_wgs84": [
                [
                    point["longitude"],
                    point["latitude"],
                ]
                for point in waypoints
            ],
            "statistics": {
                "point_count": len(waypoints),
                "total_length_m": total_length,
                "minimum_turning_radius_m": (
                    None
                    if not math.isfinite(
                        measured_minimum_radius
                    )
                    else measured_minimum_radius
                ),
                "turning_radius_p05_m": (
                    None
                    if not math.isfinite(radius_p05)
                    else radius_p05
                ),
                "self_intersection_count": len(
                    intersections
                ),
            },
            "replay_command": (
                "./scripts/replay_route.sh "
                + route_name
            ),
        }

    @staticmethod
    def points_to_wgs84(
        route_points: list[tuple[float, float]],
        origin_latitude: float,
        origin_longitude: float,
        altitude: float,
    ) -> list[list[float]]:
        result = []

        for east, north in route_points:
            latitude, longitude, _ = enu_to_geodetic(
                east,
                north,
                0.0,
                origin_latitude,
                origin_longitude,
                altitude,
            )
            result.append([longitude, latitude])

        return result

    @staticmethod
    def build_waypoints(
        route_points: list[tuple[float, float]],
        origin_latitude: float,
        origin_longitude: float,
        altitude: float,
        closed: bool,
    ) -> list[dict[str, Any]]:
        waypoints = []
        unique_count = (
            len(route_points) - 1
            if closed
            else len(route_points)
        )

        for index, point in enumerate(route_points):
            logical_index = (
                0
                if closed and index == unique_count
                else index
            )

            if closed:
                previous = route_points[
                    (logical_index - 1) % unique_count
                ]
                following = route_points[
                    (logical_index + 1) % unique_count
                ]
            else:
                previous = route_points[
                    max(0, logical_index - 1)
                ]
                following = route_points[
                    min(
                        unique_count - 1,
                        logical_index + 1,
                    )
                ]

            dx = following[0] - previous[0]
            dy = following[1] - previous[1]

            ros_yaw_deg = normalize_angle_deg(
                math.degrees(math.atan2(dy, dx))
            )
            heading_deg = normalize_angle_deg(
                90.0 - ros_yaw_deg
            )

            latitude, longitude, point_altitude = (
                enu_to_geodetic(
                    point[0],
                    point[1],
                    0.0,
                    origin_latitude,
                    origin_longitude,
                    altitude,
                )
            )

            waypoints.append({
                "index": index,
                "x": float(point[0]),
                "y": float(point[1]),
                "z": 0.0,
                "latitude": latitude,
                "longitude": longitude,
                "altitude": point_altitude,
                "heading_deg": heading_deg,
                "ros_yaw_deg": ros_yaw_deg,
            })

        return waypoints


def build_html(
    amap_key: str,
    security_code: str,
    default_longitude: float,
    default_latitude: float,
) -> str:
    return f"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta
  name="viewport"
  content="width=device-width,initial-scale=1.0"
>
<title>巡检小车地图前端</title>

<style>
html, body {{
  height: 100%;
  margin: 0;
  font-family:
    system-ui, -apple-system, BlinkMacSystemFont,
    "Segoe UI", "Microsoft YaHei", sans-serif;
  background: #f5f7fa;
}}
#layout {{
  display: grid;
  grid-template-columns: minmax(0, 1fr) 410px;
  height: 100%;
}}
#map {{
  height: 100%;
  min-width: 0;
}}
#panel {{
  box-sizing: border-box;
  overflow: auto;
  padding: 14px;
  background: rgba(255, 255, 255, 0.98);
  border-left: 1px solid #d9d9d9;
}}
h1 {{
  margin: 0 0 10px;
  font-size: 21px;
}}
h2 {{
  margin: 18px 0 8px;
  font-size: 16px;
}}
button {{
  margin: 3px 2px;
  padding: 7px 10px;
  cursor: pointer;
}}
input, select {{
  box-sizing: border-box;
  width: 100%;
  margin: 3px 0 8px;
  padding: 7px;
}}
input[type="checkbox"] {{
  width: auto;
}}
.grid2 {{
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 8px;
}}
.card {{
  margin-top: 8px;
  padding: 10px;
  border: 1px solid #d9d9d9;
  border-radius: 6px;
  background: #ffffff;
  line-height: 1.55;
}}
.ok {{
  border-color: #95de64;
  background: #f6ffed;
}}
.warn {{
  border-color: #ffe58f;
  background: #fffbe6;
}}
.error {{
  display: none;
  border-color: #ffa39e;
  background: #fff1f0;
  white-space: pre-wrap;
}}
.legend {{
  display: grid;
  gap: 6px;
  font-size: 13px;
}}
.legend-row {{
  display: flex;
  align-items: center;
  gap: 8px;
}}
.legend-line {{
  width: 34px;
  height: 4px;
  border-radius: 2px;
}}
.vehicle-icon {{
  width: 28px;
  height: 36px;
  transform-origin: 50% 50%;
}}
.vehicle-body {{
  position: relative;
  width: 24px;
  height: 28px;
  margin: 6px 2px 2px;
  border: 2px solid #ffffff;
  border-radius: 7px;
  background: #1677ff;
  box-shadow: 0 1px 6px rgba(0,0,0,.45);
}}
.vehicle-body::before {{
  content: "";
  position: absolute;
  left: 6px;
  top: -9px;
  width: 0;
  height: 0;
  border-left: 6px solid transparent;
  border-right: 6px solid transparent;
  border-bottom: 10px solid #1677ff;
}}
.small {{
  color: #666666;
  font-size: 12px;
  line-height: 1.5;
}}
code {{
  background: #f0f0f0;
  padding: 1px 4px;
}}
@media (max-width: 900px) {{
  #layout {{
    grid-template-columns: 1fr;
    grid-template-rows: 65vh auto;
    height: auto;
  }}
  #map {{
    height: 65vh;
  }}
}}
</style>

<script>
window._AMapSecurityConfig = {{
  securityJsCode: {json.dumps(security_code)}
}};
</script>

<script
  src="https://webapi.amap.com/maps?v=2.0&key={amap_key}&plugin=AMap.PolylineEditor"
></script>
</head>

<body>
<div id="layout">
  <div id="map"></div>

  <aside id="panel">
    <h1>巡检小车地图前端</h1>

    <div>
      <button onclick="showStandard()">标准地图</button>
      <button onclick="showSatellite()">卫星 + 路网</button>
      <button onclick="focusVehicle()">跟随小车</button>
      <button onclick="fitAll()">显示全部</button>
    </div>

    <div class="legend card">
      <div class="legend-row">
        <span class="legend-line" style="background:#00a870"></span>
        <span>规划/设计路线</span>
      </div>
      <div class="legend-row">
        <span class="legend-line" style="background:#1677ff"></span>
        <span>车辆实际轨迹</span>
      </div>
      <div class="legend-row">
        <span class="legend-line" style="background:#fa8c16"></span>
        <span>人工关键点折线</span>
      </div>
    </div>

    <h2>车辆状态</h2>
    <div id="vehicle-status" class="card warn">
      等待 /patrol/localization_status
    </div>

    <div id="mission-status" class="card">
      任务状态：暂无
    </div>

    <div>
      <button onclick="clearActualTrace()">清除实际轨迹</button>
    </div>

    <h2>路线绘制</h2>

    <label>路线名称</label>
    <input id="route-name" value="campus_loop_01">

    <div class="grid2">
      <div>
        <label>点间距 / m</label>
        <input
          id="spacing"
          type="number"
          value="0.40"
          min="0.10"
          max="2.00"
          step="0.05"
        >
      </div>

      <div>
        <label>转弯圆弧半径 / m</label>
        <input
          id="minimum-radius"
          type="number"
          value="1.50"
          min="0.50"
          step="0.10"
        >
      </div>
    </div>

    <div class="card small">
      路径采用“直线段 + 相切圆弧”生成：道路直行部分严格保持直线，
      只在关键点转角附近按上面的转弯半径生成圆弧。
    </div>

    <label>
      <input id="closed" type="checkbox" checked>
      闭环路线
    </label>

    <div>
      <button onclick="startDrawing()">开始新路线</button>
      <button onclick="finishDrawing()">结束绘制</button>
      <button onclick="startEditing()">编辑关键点</button>
      <button onclick="finishEditing()">结束编辑</button>
    </div>

    <div>
      <button onclick="undoPoint()">撤销最后一点</button>
      <button onclick="clearDesign()">清空设计</button>
      <button onclick="saveRoute(false)">检查并保存</button>
    </div>

    <div id="design-status" class="card ok">
      点击“开始新路线”，然后沿道路中心线设置关键点。
    </div>

    <div id="error" class="card error"></div>

    <h2>已保存路线</h2>

    <select id="route-select">
      <option value="">选择路线</option>
    </select>

    <div>
      <button onclick="refreshRoutes()">刷新列表</button>
      <button onclick="loadSelectedRoute()">显示路线</button>
    </div>

    <div id="saved-route-info" class="card">
      保存后仍使用现有命令复现：<br>
      <code>./scripts/replay_route.sh 路线名</code><br><br>
      现有任务流程会先调用 Hybrid A* 寻找路线起点，
      然后切换到 Pure Pursuit 巡迹。
    </div>

    <h2>安全提示</h2>
    <div class="card warn small">
      当前前端不进行障碍物检测，也不直接控制车辆。
      卫星图只用于粗略设计。实车复现前仍需确认车辆前后、
      路线转弯区域和整条道路无障碍，并保持急停可用。
    </div>
  </aside>
</div>

<script>
const map = new AMap.Map("map", {{
  viewMode: "2D",
  zoom: 18,
  center: [{default_longitude}, {default_latitude}]
}});

const standardLayer = AMap.createDefaultLayer({{
  visible: true,
  zIndex: 0
}});
const satelliteLayer = new AMap.TileLayer.Satellite({{
  visible: false,
  zIndex: 0
}});
const roadLayer = new AMap.TileLayer.RoadNet({{
  visible: false,
  zIndex: 1
}});

map.setLayers([
  standardLayer,
  satelliteLayer,
  roadLayer
]);

let vehicleMarker = null;
let vehicleGcj = null;
let followVehicleEnabled = true;
let firstVehicleFix = true;

let traceEpoch = -1;
let traceIndex = 0;
let actualTraceGcj = [];

const actualTracePolyline = new AMap.Polyline({{
  path: [],
  strokeColor: "#1677ff",
  strokeWeight: 5,
  strokeOpacity: 0.90,
  lineJoin: "round",
  lineCap: "round",
  zIndex: 45
}});

const keypointPolyline = new AMap.Polyline({{
  path: [],
  strokeColor: "#fa8c16",
  strokeWeight: 4,
  strokeStyle: "dashed",
  strokeOpacity: 0.95,
  lineJoin: "round",
  lineCap: "round",
  zIndex: 70
}});

const closurePolyline = new AMap.Polyline({{
  path: [],
  strokeColor: "#fa8c16",
  strokeWeight: 3,
  strokeStyle: "dashed",
  strokeOpacity: 0.75,
  zIndex: 69
}});

const plannedPolyline = new AMap.Polyline({{
  path: [],
  strokeColor: "#00a870",
  strokeWeight: 6,
  strokeOpacity: 0.95,
  lineJoin: "round",
  lineCap: "round",
  zIndex: 60
}});

map.add([
  actualTracePolyline,
  plannedPolyline,
  closurePolyline,
  keypointPolyline
]);

let drawing = false;
let keypointsGcj = [];
let editor = null;

function showStandard() {{
  standardLayer.show();
  satelliteLayer.hide();
  roadLayer.hide();
}}

function showSatellite() {{
  standardLayer.hide();
  satelliteLayer.show();
  roadLayer.show();
}}

function showError(message) {{
  const element = document.getElementById("error");

  if (!message) {{
    element.style.display = "none";
    element.textContent = "";
    return;
  }}

  element.style.display = "block";
  element.textContent = String(message);
}}

function convertBatch(points) {{
  return new Promise((resolve, reject) => {{
    if (!points.length) {{
      resolve([]);
      return;
    }}

    AMap.convertFrom(
      points,
      "gps",
      function(status, result) {{
        if (
          status === "complete"
          && result
          && Array.isArray(result.locations)
        ) {{
          resolve(
            result.locations.map(
              point => [point.lng, point.lat]
            )
          );
          return;
        }}

        reject(
          new Error(
            "WGS84→高德坐标转换失败："
            + status
          )
        );
      }}
    );
  }});
}}

async function convertAll(points) {{
  const converted = [];

  for (let start = 0; start < points.length; start += 40) {{
    const batch = points.slice(start, start + 40);
    const result = await convertBatch(batch);
    converted.push(...result);
  }}

  return converted;
}}

function vehicleContent(heading) {{
  return (
    '<div class="vehicle-icon" style="transform:rotate('
    + Number(heading || 0)
    + 'deg)">'
    + '<div class="vehicle-body"></div>'
    + '</div>'
  );
}}

function setVehicleMarker(position, heading) {{
  vehicleGcj = position;

  if (!vehicleMarker) {{
    vehicleMarker = new AMap.Marker({{
      position: position,
      content: vehicleContent(heading),
      offset: new AMap.Pixel(-14, -18),
      zIndex: 100,
      title: "巡检小车"
    }});

    map.add(vehicleMarker);
  }} else {{
    vehicleMarker.setPosition(position);
    vehicleMarker.setContent(
      vehicleContent(heading)
    );
  }}

  if (firstVehicleFix || followVehicleEnabled) {{
    map.setCenter(position);
  }}

  firstVehicleFix = false;
}}

async function updateState() {{
  try {{
    const response = await fetch(
      "/api/state",
      {{cache: "no-store"}}
    );
    const data = await response.json();

    const localization = data.localization;
    const vehicleStatus = document.getElementById(
      "vehicle-status"
    );

    if (localization) {{
      const stateText = localization.valid
        ? "有效"
        : "无效";

      vehicleStatus.className = localization.valid
        ? "card ok"
        : "card warn";

      vehicleStatus.textContent =
        "定位：" + stateText
        + "\\n经度：" + localization.longitude.toFixed(9)
        + "\\n纬度：" + localization.latitude.toFixed(9)
        + "\\n高度：" + localization.altitude.toFixed(2) + " m"
        + "\\n航向：" + localization.heading_deg.toFixed(1) + "°"
        + "\\n卫星：" + localization.nsv1
        + " / " + localization.nsv2
        + "\\n原因：" + localization.reason;

      if (
        Number.isFinite(localization.longitude)
        && Number.isFinite(localization.latitude)
      ) {{
        const converted = await convertBatch([[
          localization.longitude,
          localization.latitude
        ]]);

        setVehicleMarker(
          converted[0],
          localization.heading_deg
        );
      }}
    }}

    const mission = data.mission;
    const missionStatus = document.getElementById(
      "mission-status"
    );

    if (mission) {{
      missionStatus.textContent =
        "任务状态：" + mission.state
        + "\\n任务：" + mission.task
        + "\\n进度："
        + (100 * mission.progress).toFixed(1)
        + "%"
        + "\\n信息：" + mission.message;
    }}

    await updateTrace(
      data.trace_count,
      data.trace_epoch
    );
  }} catch (error) {{
    showError("状态更新失败：" + String(error));
  }}
}}

async function updateTrace(count, epoch) {{
  if (traceEpoch !== epoch) {{
    traceEpoch = epoch;
    traceIndex = 0;
    actualTraceGcj = [];
    actualTracePolyline.setPath([]);
  }}

  if (traceIndex >= count) {{
    return;
  }}

  const response = await fetch(
    "/api/trace?after="
    + traceIndex
    + "&epoch="
    + traceEpoch,
    {{cache: "no-store"}}
  );
  const data = await response.json();

  if (data.epoch !== traceEpoch) {{
    traceEpoch = data.epoch;
    traceIndex = 0;
    actualTraceGcj = [];
  }}

  const converted = await convertAll(
    data.points
  );

  actualTraceGcj.push(...converted);
  traceIndex = data.start_index + data.points.length;

  actualTracePolyline.setPath(
    actualTraceGcj
  );
}}

function focusVehicle() {{
  followVehicleEnabled = true;

  if (vehicleGcj) {{
    map.setZoomAndCenter(19, vehicleGcj);
  }}
}}

function fitAll() {{
  followVehicleEnabled = false;

  const overlays = [
    actualTracePolyline,
    plannedPolyline,
    keypointPolyline
  ];

  if (vehicleMarker) {{
    overlays.push(vehicleMarker);
  }}

  map.setFitView(
    overlays,
    false,
    [60, 60, 60, 60]
  );
}}

function syncKeypoints() {{
  keypointsGcj = keypointPolyline
    .getPath()
    .map(point => [point.lng, point.lat]);

  updateDesignPreview();
}}

function localFrame(points) {{
  const longitude0 = points[0][0];
  const latitude0 = points[0][1];
  const cosine = Math.cos(latitude0 * Math.PI / 180.0);

  return {{
    longitude0: longitude0,
    latitude0: latitude0,
    cosine: cosine,
    points: points.map(point => [
      EARTH_RADIUS_JS * cosine
        * (point[0] - longitude0)
        * Math.PI / 180.0,
      EARTH_RADIUS_JS
        * (point[1] - latitude0)
        * Math.PI / 180.0
    ])
  }};
}}

const EARTH_RADIUS_JS = 6378137.0;

function localToLngLat(point, frame) {{
  return [
    frame.longitude0
      + point[0]
        / (EARTH_RADIUS_JS * frame.cosine)
        * 180.0 / Math.PI,
    frame.latitude0
      + point[1]
        / EARTH_RADIUS_JS
        * 180.0 / Math.PI
  ];
}}

function filletPreview(points, closed, radius) {{
  if (points.length < 2) {{
    return {{path: points.slice(), error: ""}};
  }}

  if (closed && points.length < 3) {{
    return {{
      path: points.concat([points[0]]),
      error: "闭环路线至少需要 3 个关键点"
    }};
  }}

  const frame = localFrame(points);
  const local = frame.points;
  const count = local.length;
  const info = new Array(count).fill(null);
  const cornerIndices = [];

  if (closed) {{
    for (let index = 0; index < count; index++) {{
      cornerIndices.push(index);
    }}
  }} else {{
    for (let index = 1; index < count - 1; index++) {{
      cornerIndices.push(index);
    }}
  }}

  for (const index of cornerIndices) {{
    const previous = local[(index - 1 + count) % count];
    const current = local[index];
    const following = local[(index + 1) % count];

    const incoming = [
      current[0] - previous[0],
      current[1] - previous[1]
    ];
    const outgoing = [
      following[0] - current[0],
      following[1] - current[1]
    ];
    const incomingLength = Math.hypot(
      incoming[0], incoming[1]
    );
    const outgoingLength = Math.hypot(
      outgoing[0], outgoing[1]
    );

    if (incomingLength < 0.001 || outgoingLength < 0.001) {{
      return {{path: points.slice(), error: "关键点过近"}};
    }}

    const uIn = [
      incoming[0] / incomingLength,
      incoming[1] / incomingLength
    ];
    const uOut = [
      outgoing[0] / outgoingLength,
      outgoing[1] / outgoingLength
    ];
    const dot = Math.max(
      -1,
      Math.min(1, uIn[0]*uOut[0] + uIn[1]*uOut[1])
    );
    const angle = Math.acos(dot);
    const turnCross = uIn[0]*uOut[1] - uIn[1]*uOut[0];

    if (angle < Math.PI / 180.0) {{
      info[index] = {{
        tangentIn: current,
        tangentOut: current,
        tangentDistance: 0,
        arc: [current]
      }};
      continue;
    }}

    if (angle > 175.0 * Math.PI / 180.0) {{
      return {{
        path: points.slice(),
        error: "关键点 " + index + " 接近掉头，无法生成圆弧"
      }};
    }}

    const tangentDistance = radius * Math.tan(angle / 2.0);
    const tangentIn = [
      current[0] - uIn[0] * tangentDistance,
      current[1] - uIn[1] * tangentDistance
    ];
    const tangentOut = [
      current[0] + uOut[0] * tangentDistance,
      current[1] + uOut[1] * tangentDistance
    ];
    const sign = turnCross > 0 ? 1 : -1;
    const leftNormal = [-uIn[1], uIn[0]];
    const center = [
      tangentIn[0] + sign * radius * leftNormal[0],
      tangentIn[1] + sign * radius * leftNormal[1]
    ];
    const startAngle = Math.atan2(
      tangentIn[1] - center[1],
      tangentIn[0] - center[0]
    );
    const endAngle = Math.atan2(
      tangentOut[1] - center[1],
      tangentOut[0] - center[0]
    );
    let delta = Math.atan2(
      Math.sin(endAngle - startAngle),
      Math.cos(endAngle - startAngle)
    );

    if (sign > 0 && delta < 0) {{
      delta += 2*Math.PI;
    }} else if (sign < 0 && delta > 0) {{
      delta -= 2*Math.PI;
    }}

    const samples = Math.max(
      8,
      Math.ceil(Math.abs(delta) * radius / 0.20)
    );
    const arc = [];

    for (let sample = 0; sample <= samples; sample++) {{
      const a = startAngle + delta * sample / samples;
      arc.push([
        center[0] + radius * Math.cos(a),
        center[1] + radius * Math.sin(a)
      ]);
    }}

    info[index] = {{
      tangentIn: tangentIn,
      tangentOut: tangentOut,
      tangentDistance: tangentDistance,
      arc: arc
    }};
  }}

  const segmentCount = closed ? count : count - 1;

  for (let index = 0; index < segmentCount; index++) {{
    const next = (index + 1) % count;
    const segmentLength = Math.hypot(
      local[next][0] - local[index][0],
      local[next][1] - local[index][1]
    );
    const usedStart = info[index]
      ? info[index].tangentDistance : 0;
    const usedEnd = info[next]
      ? info[next].tangentDistance : 0;

    if (usedStart + usedEnd > segmentLength - 0.05) {{
      return {{
        path: points.slice(),
        error:
          "关键点 " + index + "→" + next
          + " 的线段太短，无法容纳半径 "
          + radius.toFixed(2) + " m 的转弯"
      }};
    }}
  }}

  const result = [];

  function append(point) {{
    const last = result[result.length - 1];

    if (
      !last
      || Math.hypot(point[0]-last[0], point[1]-last[1]) > 1e-9
    ) {{
      result.push(point);
    }}
  }}

  if (!closed) {{
    append(local[0]);

    for (let index = 1; index < count - 1; index++) {{
      append(info[index].tangentIn);

      for (const point of info[index].arc.slice(1)) {{
        append(point);
      }}
    }}

    append(local[count - 1]);
  }} else {{
    append(info[0].tangentOut);

    for (const index of [
      ...Array.from({{length: count - 1}}, (_, offset) => offset + 1),
      0
    ]) {{
      append(info[index].tangentIn);

      for (const point of info[index].arc.slice(1)) {{
        append(point);
      }}
    }}
  }}

  return {{
    path: result.map(point => localToLngLat(point, frame)),
    error: ""
  }};
}}

function updateDesignPreview() {{
  const closed = document.getElementById(
    "closed"
  ).checked;
  const radius = Math.max(
    0.50,
    Number(
      document.getElementById("minimum-radius").value
    ) || 1.50
  );

  keypointPolyline.setPath(keypointsGcj);

  if (closed && keypointsGcj.length >= 2) {{
    closurePolyline.setPath([
      keypointsGcj[keypointsGcj.length - 1],
      keypointsGcj[0]
    ]);
  }} else {{
    closurePolyline.setPath([]);
  }}

  const preview = filletPreview(
    keypointsGcj,
    closed,
    radius
  );
  plannedPolyline.setPath(preview.path);

  document.getElementById(
    "design-status"
  ).textContent =
    "关键点数：" + keypointsGcj.length
    + "\\n路线类型："
    + (closed ? "闭环" : "开放")
    + "\\n几何模式：直线段 + 相切圆弧"
    + "\\n当前模式："
    + (drawing ? "地图点击绘制" : "等待操作")
    + (preview.error ? "\\n提示：" + preview.error : "");
}}

function startDrawing() {{
  finishEditing();
  keypointsGcj = [];
  drawing = true;
  showError("");
  updateDesignPreview();
}}

function finishDrawing() {{
  drawing = false;
  updateDesignPreview();
}}

function startEditing() {{
  drawing = false;
  finishEditing();

  if (keypointsGcj.length < 2) {{
    showError("至少需要两个关键点才能编辑。");
    return;
  }}

  editor = new AMap.PolylineEditor(
    map,
    keypointPolyline
  );

  for (const eventName of [
    "addnode",
    "adjust",
    "removenode",
    "end"
  ]) {{
    editor.on(eventName, syncKeypoints);
  }}

  editor.open();
}}

function finishEditing() {{
  if (!editor) {{
    return;
  }}

  try {{
    editor.close();
  }} catch (error) {{
    console.warn(error);
  }}

  syncKeypoints();
  editor = null;
}}

function undoPoint() {{
  finishEditing();

  if (keypointsGcj.length) {{
    keypointsGcj.pop();
    updateDesignPreview();
  }}
}}

function clearDesign() {{
  finishEditing();
  drawing = false;
  keypointsGcj = [];
  keypointPolyline.setPath([]);
  closurePolyline.setPath([]);
  plannedPolyline.setPath([]);
  updateDesignPreview();
  showError("");
}}

async function saveRoute(allowWarnings) {{
  finishEditing();
  showError("");

  const payload = {{
    name: document.getElementById(
      "route-name"
    ).value,
    closed: document.getElementById(
      "closed"
    ).checked,
    spacing_m: Number(
      document.getElementById("spacing").value
    ),
    minimum_turning_radius_m: Number(
      document.getElementById(
        "minimum-radius"
      ).value
    ),
    keypoints_gcj02: keypointsGcj,
    allow_warnings: allowWarnings
  }};

  try {{
    const response = await fetch(
      "/api/routes/save",
      {{
        method: "POST",
        headers: {{
          "Content-Type": "application/json"
        }},
        body: JSON.stringify(payload)
      }}
    );

    const result = await response.json();

    if (!response.ok) {{
      throw new Error(
        result.error || "保存失败"
      );
    }}

    if (result.preview_wgs84) {{
      try {{
        const step = Math.max(
          1,
          Math.ceil(result.preview_wgs84.length / 240)
        );

        const previewPoints = result.preview_wgs84.filter(
          (_, index) => index % step === 0
        );

        const lastPoint =
          result.preview_wgs84[
            result.preview_wgs84.length - 1
          ];

        if (previewPoints.length && lastPoint) {{
          previewPoints.push(lastPoint);
        }}

        plannedPolyline.setPath(
          await convertAll(previewPoints)
        );
      }} catch (previewError) {{
        console.warn(
          "路线已保存，但高德预览转换失败：",
          previewError
        );
      }}
    }}

    if (
      !result.success
      && result.requires_confirmation
    ) {{
      const warningText = result.warnings.join("\\n");

      const confirmed = window.confirm(
        warningText
        + "\\n\\n仍然保存这条路线吗？"
      );

      if (confirmed) {{
        await saveRoute(true);
      }}

      return;
    }}

    document.getElementById(
      "design-status"
    ).textContent =
      "路线已保存：" + result.name
      + "\\n点数：" + result.statistics.point_count
      + "\\n长度："
      + result.statistics.total_length_m.toFixed(2)
      + " m"
      + "\\n最小转弯半径："
      + (
        result.statistics.minimum_turning_radius_m === null
          ? "未检出"
          : result.statistics.minimum_turning_radius_m.toFixed(2)
            + " m"
      )
      + "\\n复现命令："
      + result.replay_command;

    await refreshRoutes();
  }} catch (error) {{
    showError("路线保存失败：" + String(error));
  }}
}}

async function refreshRoutes() {{
  try {{
    const response = await fetch(
      "/api/routes",
      {{cache: "no-store"}}
    );
    const routes = await response.json();
    const select = document.getElementById(
      "route-select"
    );

    const selected = select.value;
    select.innerHTML =
      '<option value="">选择路线</option>';

    for (const route of routes) {{
      const option = document.createElement("option");
      option.value = route.name;
      option.textContent =
        route.name
        + (
          route.invalid
            ? "（无效）"
            : "（"
              + route.total_length.toFixed(1)
              + "m）"
        );
      select.appendChild(option);
    }}

    select.value = selected;
  }} catch (error) {{
    showError("路线列表读取失败：" + String(error));
  }}
}}

async function loadSelectedRoute() {{
  const name = document.getElementById(
    "route-select"
  ).value;

  if (!name) {{
    return;
  }}

  try {{
    const response = await fetch(
      "/api/routes/"
      + encodeURIComponent(name),
      {{cache: "no-store"}}
    );
    const route = await response.json();

    if (!response.ok) {{
      throw new Error(
        route.error || "路线读取失败"
      );
    }}

    const step = Math.max(
      1,
      Math.ceil(route.points_wgs84.length / 240)
    );

    const displayPoints = route.points_wgs84.filter(
      (_, index) => index % step === 0
    );

    const lastPoint =
      route.points_wgs84[
        route.points_wgs84.length - 1
      ];

    if (displayPoints.length && lastPoint) {{
      displayPoints.push(lastPoint);
    }}

    const converted = await convertAll(
      displayPoints
    );

    plannedPolyline.setPath(converted);

    document.getElementById(
      "saved-route-info"
    ).textContent =
      "路线：" + route.name
      + "\\n点数：" + route.summary.point_count
      + "\\n长度："
      + Number(route.summary.total_length).toFixed(2)
      + " m"
      + "\\n闭环："
      + (route.closed_loop ? "是" : "否")
      + "\\n复现命令：./scripts/replay_route.sh "
      + route.name;

    fitAll();
  }} catch (error) {{
    showError("路线显示失败：" + String(error));
  }}
}}

async function clearActualTrace() {{
  try {{
    await fetch(
      "/api/trace/clear",
      {{method: "POST"}}
    );

    traceEpoch = -1;
    traceIndex = 0;
    actualTraceGcj = [];
    actualTracePolyline.setPath([]);
  }} catch (error) {{
    showError("实际轨迹清除失败：" + String(error));
  }}
}}

map.on("click", function(event) {{
  if (!drawing) {{
    followVehicleEnabled = false;
    return;
  }}

  keypointsGcj.push([
    event.lnglat.lng,
    event.lnglat.lat
  ]);

  updateDesignPreview();
}});

map.on("dragstart", function() {{
  followVehicleEnabled = false;
}});

document.getElementById(
  "closed"
).addEventListener(
  "change",
  updateDesignPreview
);

document.getElementById(
  "minimum-radius"
).addEventListener(
  "input",
  updateDesignPreview
);

refreshRoutes();
updateDesignPreview();
setInterval(updateState, 500);
updateState();
</script>
</body>
</html>
"""


class ApiContext:
    def __init__(
        self,
        shared_state: SharedState,
        route_store: RouteStore,
        html_page: str,
    ) -> None:
        self.shared_state = shared_state
        self.route_store = route_store
        self.html_page = html_page


def make_handler(
    context: ApiContext,
):
    class Handler(BaseHTTPRequestHandler):
        server_version = "PatrolMapHTTP/1.0"

        def log_message(
            self,
            format_string: str,
            *arguments: Any,
        ) -> None:
            print(
                "[patrol-map] "
                + format_string % arguments
            )

        def send_json(
            self,
            payload: Any,
            status: int = HTTPStatus.OK,
        ) -> None:
            body = json.dumps(
                payload,
                ensure_ascii=False,
            ).encode("utf-8")

            self.send_response(status)
            self.send_header(
                "Content-Type",
                "application/json; charset=utf-8",
            )
            self.send_header(
                "Cache-Control",
                "no-store",
            )
            self.send_header(
                "Content-Length",
                str(len(body)),
            )
            self.end_headers()
            self.wfile.write(body)

        def send_html(
            self,
            page: str,
        ) -> None:
            body = page.encode("utf-8")
            self.send_response(HTTPStatus.OK)
            self.send_header(
                "Content-Type",
                "text/html; charset=utf-8",
            )
            self.send_header(
                "Cache-Control",
                "no-store",
            )
            self.send_header(
                "Content-Length",
                str(len(body)),
            )
            self.end_headers()
            self.wfile.write(body)

        def read_json(self) -> dict[str, Any]:
            content_length = int(
                self.headers.get(
                    "Content-Length",
                    "0",
                )
            )

            if content_length <= 0:
                return {}

            if content_length > 2_000_000:
                raise ValueError("请求体过大")

            raw = self.rfile.read(content_length)
            payload = json.loads(
                raw.decode("utf-8")
            )

            if not isinstance(payload, dict):
                raise ValueError(
                    "JSON 根节点必须是对象"
                )

            return payload

        def do_GET(self) -> None:
            parsed = urlparse(self.path)

            try:
                if parsed.path in ("/", "/index.html"):
                    self.send_html(context.html_page)
                    return

                if parsed.path == "/api/state":
                    self.send_json(
                        context.shared_state.snapshot()
                    )
                    return

                if parsed.path == "/api/trace":
                    query = parse_qs(parsed.query)
                    after = int(
                        query.get("after", ["0"])[0]
                    )
                    epoch = int(
                        query.get("epoch", ["-1"])[0]
                    )

                    self.send_json(
                        context.shared_state.trace_after(
                            after,
                            epoch,
                        )
                    )
                    return

                if parsed.path == "/api/routes":
                    self.send_json(
                        context.route_store.list_routes()
                    )
                    return

                route_prefix = "/api/routes/"

                if parsed.path.startswith(route_prefix):
                    name = unquote(
                        parsed.path[len(route_prefix):]
                    )
                    self.send_json(
                        context.route_store.load_route(
                            name
                        )
                    )
                    return

                if parsed.path == "/favicon.ico":
                    self.send_response(
                        HTTPStatus.NO_CONTENT
                    )
                    self.end_headers()
                    return

                self.send_json(
                    {"error": "not found"},
                    HTTPStatus.NOT_FOUND,
                )
            except FileNotFoundError as exc:
                self.send_json(
                    {"error": str(exc)},
                    HTTPStatus.NOT_FOUND,
                )
            except Exception as exc:
                self.send_json(
                    {"error": str(exc)},
                    HTTPStatus.BAD_REQUEST,
                )

        def do_POST(self) -> None:
            parsed = urlparse(self.path)

            try:
                if parsed.path == "/api/trace/clear":
                    context.shared_state.clear_trace()
                    self.send_json({"success": True})
                    return

                if parsed.path == "/api/routes/save":
                    request = self.read_json()
                    result = (
                        context.route_store.save_route(
                            request
                        )
                    )
                    self.send_json(result)
                    return

                self.send_json(
                    {"error": "not found"},
                    HTTPStatus.NOT_FOUND,
                )
            except Exception as exc:
                self.send_json(
                    {"error": str(exc)},
                    HTTPStatus.BAD_REQUEST,
                )

    return Handler


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "巡检小车实时地图与路线绘制前端"
        )
    )

    parser.add_argument(
        "--host",
        default="0.0.0.0",
        help="监听地址，默认 0.0.0.0",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=8080,
        help="监听端口，默认 8080",
    )
    parser.add_argument(
        "--routes-dir",
        type=Path,
        default=Path(
            "/home/nvidia/patrol_ws/routes"
        ),
        help="路线保存目录",
    )
    parser.add_argument(
        "--amap-key",
        default=os.environ.get(
            "AMAP_JS_KEY",
            "",
        ),
        help="高德 JS API Key",
    )
    parser.add_argument(
        "--security-code",
        default=os.environ.get(
            "AMAP_SECURITY_CODE",
            "",
        ),
        help="高德安全密钥",
    )
    parser.add_argument(
        "--default-longitude",
        type=float,
        default=113.0,
        help="无定位时地图默认经度",
    )
    parser.add_argument(
        "--default-latitude",
        type=float,
        default=34.0,
        help="无定位时地图默认纬度",
    )
    parser.add_argument(
        "--trace-spacing",
        type=float,
        default=0.25,
        help="实际轨迹采样最小间距，单位 m",
    )
    parser.add_argument(
        "--trace-max-points",
        type=int,
        default=5000,
        help="实际轨迹最多保留点数",
    )

    return parser.parse_args()


def main() -> int:
    args = parse_arguments()

    if not args.amap_key:
        raise ValueError(
            "缺少 AMAP_JS_KEY"
        )

    if not args.security_code:
        raise ValueError(
            "缺少 AMAP_SECURITY_CODE"
        )

    shared_state = SharedState(
        trace_spacing_m=max(
            0.05,
            args.trace_spacing,
        ),
        trace_max_points=max(
            100,
            args.trace_max_points,
        ),
    )

    route_store = RouteStore(
        args.routes_dir,
        shared_state,
    )

    html_page = build_html(
        args.amap_key,
        args.security_code,
        args.default_longitude,
        args.default_latitude,
    )

    rclpy.init()
    node = PatrolMapBridge(shared_state)
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    ros_thread = threading.Thread(
        target=executor.spin,
        name="ros2-spin",
        daemon=True,
    )
    ros_thread.start()

    context = ApiContext(
        shared_state,
        route_store,
        html_page,
    )

    server = ThreadingHTTPServer(
        (args.host, args.port),
        make_handler(context),
    )

    stopping = threading.Event()

    def stop_handler(
        signum: int,
        frame: Any,
    ) -> None:
        del signum
        del frame

        if stopping.is_set():
            return

        stopping.set()

        threading.Thread(
            target=server.shutdown,
            daemon=True,
        ).start()

    signal.signal(
        signal.SIGINT,
        stop_handler,
    )
    signal.signal(
        signal.SIGTERM,
        stop_handler,
    )

    print(
        f"[patrol-map] 服务已启动：http://"
        f"{args.host}:{args.port}"
    )
    print(
        "[patrol-map] 当前版本只显示状态、记录实际轨迹、"
        "绘制并保存路线；不会直接控制车辆。"
    )

    try:
        server.serve_forever(
            poll_interval=0.2
        )
    finally:
        server.server_close()
        executor.shutdown()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        ros_thread.join(timeout=2.0)

    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
    except Exception as exc:
        print(f"错误：{exc}")
        raise SystemExit(1)
