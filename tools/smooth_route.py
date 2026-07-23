#!/usr/bin/env python3

import argparse
import math
from copy import deepcopy
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import yaml


WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


def cumulative_distance(x, y):
    distances = np.hypot(np.diff(x), np.diff(y))
    return np.concatenate(([0.0], np.cumsum(distances)))


def normalize_angle_deg(angle):
    return (angle + 180.0) % 360.0 - 180.0


def percentile(values, q):
    if not values:
        return 0.0
    return float(np.percentile(np.asarray(values), q))


def route_metrics(x, y):
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)

    if len(x) < 3:
        return {
            "length": 0.0,
            "local_mean": 0.0,
            "local_p95": 0.0,
            "local_max": 0.0,
            "heading_mean": 0.0,
            "heading_p95": 0.0,
            "heading_max": 0.0,
            "turn_switches": 0,
        }

    s = cumulative_distance(x, y)

    headings = np.arctan2(
        np.diff(y),
        np.diff(x),
    )

    heading_changes = []
    signed_changes = []

    for first, second in zip(headings, headings[1:]):
        difference = (
            second - first + math.pi
        ) % (2.0 * math.pi) - math.pi

        signed_changes.append(
            math.degrees(difference)
        )
        heading_changes.append(
            abs(math.degrees(difference))
        )

    local_deviations = []

    for index in range(1, len(x) - 1):
        ax = x[index - 1]
        ay = y[index - 1]
        px = x[index]
        py = y[index]
        bx = x[index + 1]
        by = y[index + 1]

        vx = bx - ax
        vy = by - ay
        length = math.hypot(vx, vy)

        if length < 1e-9:
            continue

        deviation = abs(
            vx * (ay - py) -
            (ax - px) * vy
        ) / length

        local_deviations.append(deviation)

    turn_switches = 0
    previous_sign = 0

    for value in signed_changes:
        if abs(value) < 3.0:
            continue

        sign = 1 if value > 0 else -1

        if previous_sign and sign != previous_sign:
            turn_switches += 1

        previous_sign = sign

    return {
        "length": float(s[-1]),
        "local_mean": (
            float(np.mean(local_deviations))
            if local_deviations else 0.0
        ),
        "local_p95": percentile(
            local_deviations,
            95,
        ),
        "local_max": (
            float(max(local_deviations))
            if local_deviations else 0.0
        ),
        "heading_mean": (
            float(np.mean(heading_changes))
            if heading_changes else 0.0
        ),
        "heading_p95": percentile(
            heading_changes,
            95,
        ),
        "heading_max": (
            float(max(heading_changes))
            if heading_changes else 0.0
        ),
        "turn_switches": turn_switches,
    }


def remove_duplicate_points(x, y, z, minimum_distance):
    kept_x = [float(x[0])]
    kept_y = [float(y[0])]
    kept_z = [float(z[0])]

    for index in range(1, len(x) - 1):
        distance = math.hypot(
            float(x[index]) - kept_x[-1],
            float(y[index]) - kept_y[-1],
        )

        if distance >= minimum_distance:
            kept_x.append(float(x[index]))
            kept_y.append(float(y[index]))
            kept_z.append(float(z[index]))

    kept_x.append(float(x[-1]))
    kept_y.append(float(y[-1]))
    kept_z.append(float(z[-1]))

    return (
        np.asarray(kept_x),
        np.asarray(kept_y),
        np.asarray(kept_z),
    )


def point_to_segment_distance(
    px,
    py,
    ax,
    ay,
    bx,
    by,
):
    vx = bx - ax
    vy = by - ay

    length_squared = vx * vx + vy * vy

    if length_squared < 1e-12:
        return math.hypot(px - ax, py - ay)

    ratio = (
        (px - ax) * vx +
        (py - ay) * vy
    ) / length_squared

    ratio = max(0.0, min(1.0, ratio))

    nearest_x = ax + ratio * vx
    nearest_y = ay + ratio * vy

    return math.hypot(
        px - nearest_x,
        py - nearest_y,
    )


def suppress_spikes(
    x,
    y,
    z,
    deviation_threshold,
    angle_threshold_deg,
    passes,
):
    x = x.copy()
    y = y.copy()
    z = z.copy()

    removed = 0

    for _ in range(passes):
        new_x = x.copy()
        new_y = y.copy()
        new_z = z.copy()

        for index in range(1, len(x) - 1):
            first_heading = math.atan2(
                y[index] - y[index - 1],
                x[index] - x[index - 1],
            )

            second_heading = math.atan2(
                y[index + 1] - y[index],
                x[index + 1] - x[index],
            )

            heading_change = abs(
                math.degrees(
                    (
                        second_heading -
                        first_heading +
                        math.pi
                    ) % (2.0 * math.pi) -
                    math.pi
                )
            )

            deviation = point_to_segment_distance(
                x[index],
                y[index],
                x[index - 1],
                y[index - 1],
                x[index + 1],
                y[index + 1],
            )

            if (
                heading_change >= angle_threshold_deg
                and deviation >= deviation_threshold
            ):
                new_x[index] = (
                    x[index - 1] +
                    x[index + 1]
                ) * 0.5

                new_y[index] = (
                    y[index - 1] +
                    y[index + 1]
                ) * 0.5

                new_z[index] = (
                    z[index - 1] +
                    z[index + 1]
                ) * 0.5

                removed += 1

        x = new_x
        y = new_y
        z = new_z

    return x, y, z, removed


def gaussian_smooth(values, sigma_samples):
    if sigma_samples <= 0.0:
        return values.copy()

    radius = max(
        1,
        int(math.ceil(3.0 * sigma_samples)),
    )

    positions = np.arange(
        -radius,
        radius + 1,
        dtype=float,
    )

    kernel = np.exp(
        -0.5 * (
            positions / sigma_samples
        ) ** 2
    )

    kernel /= np.sum(kernel)

    padded = np.pad(
        values,
        (radius, radius),
        mode="edge",
    )

    return np.convolve(
        padded,
        kernel,
        mode="valid",
    )


def geodetic_to_ecef(latitude, longitude, altitude):
    latitude_rad = math.radians(latitude)
    longitude_rad = math.radians(longitude)

    sin_latitude = math.sin(latitude_rad)
    cos_latitude = math.cos(latitude_rad)
    sin_longitude = math.sin(longitude_rad)
    cos_longitude = math.cos(longitude_rad)

    normal = WGS84_A / math.sqrt(
        1.0 -
        WGS84_E2 * sin_latitude * sin_latitude
    )

    x = (
        normal + altitude
    ) * cos_latitude * cos_longitude

    y = (
        normal + altitude
    ) * cos_latitude * sin_longitude

    z = (
        normal * (1.0 - WGS84_E2) +
        altitude
    ) * sin_latitude

    return x, y, z


def geodetic_to_enu(
    latitude,
    longitude,
    altitude,
    origin_latitude,
    origin_longitude,
    origin_altitude,
):
    x, y, z = geodetic_to_ecef(
        latitude,
        longitude,
        altitude,
    )

    origin_x, origin_y, origin_z = geodetic_to_ecef(
        origin_latitude,
        origin_longitude,
        origin_altitude,
    )

    dx = x - origin_x
    dy = y - origin_y
    dz = z - origin_z

    latitude_rad = math.radians(origin_latitude)
    longitude_rad = math.radians(origin_longitude)

    sin_latitude = math.sin(latitude_rad)
    cos_latitude = math.cos(latitude_rad)
    sin_longitude = math.sin(longitude_rad)
    cos_longitude = math.cos(longitude_rad)

    east = (
        -sin_longitude * dx
        + cos_longitude * dy
    )

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


def ecef_to_geodetic(x, y, z):
    longitude = math.atan2(y, x)
    horizontal = math.hypot(x, y)

    latitude = math.atan2(
        z,
        horizontal * (1.0 - WGS84_E2),
    )

    altitude = 0.0

    for _ in range(10):
        sin_latitude = math.sin(latitude)

        normal = WGS84_A / math.sqrt(
            1.0 -
            WGS84_E2 *
            sin_latitude *
            sin_latitude
        )

        cos_latitude = math.cos(latitude)

        if abs(cos_latitude) < 1e-12:
            altitude = (
                abs(z) -
                normal * (1.0 - WGS84_E2)
            )
        else:
            altitude = (
                horizontal / cos_latitude -
                normal
            )

        denominator = horizontal * (
            1.0 -
            WGS84_E2 *
            normal /
            (normal + altitude)
        )

        new_latitude = math.atan2(
            z,
            denominator,
        )

        if abs(new_latitude - latitude) < 1e-13:
            latitude = new_latitude
            break

        latitude = new_latitude

    return (
        math.degrees(latitude),
        math.degrees(longitude),
        altitude,
    )


def enu_to_geodetic(
    east,
    north,
    up,
    origin_latitude,
    origin_longitude,
    origin_altitude,
):
    origin_x, origin_y, origin_z = geodetic_to_ecef(
        origin_latitude,
        origin_longitude,
        origin_altitude,
    )

    latitude_rad = math.radians(origin_latitude)
    longitude_rad = math.radians(origin_longitude)

    sin_latitude = math.sin(latitude_rad)
    cos_latitude = math.cos(latitude_rad)
    sin_longitude = math.sin(longitude_rad)
    cos_longitude = math.cos(longitude_rad)

    delta_x = (
        -sin_longitude * east
        - sin_latitude * cos_longitude * north
        + cos_latitude * cos_longitude * up
    )

    delta_y = (
        cos_longitude * east
        - sin_latitude * sin_longitude * north
        + cos_latitude * sin_longitude * up
    )

    delta_z = (
        cos_latitude * north
        + sin_latitude * up
    )

    return ecef_to_geodetic(
        origin_x + delta_x,
        origin_y + delta_y,
        origin_z + delta_z,
    )


def make_uniform_samples(total_length, spacing):
    count = max(
        2,
        int(math.ceil(total_length / spacing)) + 1,
    )

    return np.linspace(
        0.0,
        total_length,
        count,
    )


def main():
    parser = argparse.ArgumentParser(
        description="平滑巡检路线并重新采样"
    )

    parser.add_argument("input")
    parser.add_argument("output")

    parser.add_argument(
        "--spacing",
        type=float,
        default=0.5,
        help="最终轨迹点间距，默认0.5m",
    )

    parser.add_argument(
        "--dense-spacing",
        type=float,
        default=0.1,
        help="内部平滑采样间距，默认0.1m",
    )

    parser.add_argument(
        "--sigma",
        type=float,
        default=0.8,
        help="高斯平滑尺度，单位m，默认0.8m",
    )

    parser.add_argument(
        "--strength",
        type=float,
        default=0.85,
        help="平滑强度0到1，默认0.85",
    )

    parser.add_argument(
        "--maximum-shift",
        type=float,
        default=0.45,
        help="平滑点相对原轨迹最大移动量，默认0.45m",
    )

    parser.add_argument(
        "--endpoint-hold",
        type=float,
        default=2.0,
        help="起终点平滑收敛范围，默认2.0m",
    )

    parser.add_argument(
        "--heading-window-distance",
        type=float,
        default=1.5,
        help=(
            "航向计算时向前和向后各取的距离，"
            "默认1.5m"
        ),
    )

    parser.add_argument(
        "--maximum-anchor-shift",
        type=float,
        default=1.5,
        help=(
            "稳定起终点与原始首尾点允许的最大偏差，"
            "默认1.5m"
        ),
    )

    parser.add_argument(
        "--spike-threshold",
        type=float,
        default=0.20,
        help="异常回摆点横向阈值，默认0.20m",
    )

    parser.add_argument(
        "--spike-angle",
        type=float,
        default=100.0,
        help="异常回摆角度阈值，默认100度",
    )

    parser.add_argument(
        "--spike-passes",
        type=int,
        default=2,
        help="异常点过滤次数，默认2",
    )

    parser.add_argument(
        "--force",
        action="store_true",
        help="允许覆盖已有输出文件",
    )

    args = parser.parse_args()

    input_path = Path(args.input)
    output_path = Path(args.output)

    if not input_path.exists():
        raise SystemExit(
            f"输入文件不存在：{input_path}"
        )

    if output_path.exists() and not args.force:
        raise SystemExit(
            f"输出文件已经存在：{output_path}\n"
            "需要覆盖时添加 --force"
        )

    data = yaml.safe_load(
        input_path.read_text(encoding="utf-8")
    )

    origin = data.get("origin") or {}
    end_pose = data.get("end_pose")
    waypoints = data.get("waypoints") or []

    if len(waypoints) < 4:
        raise SystemExit("轨迹点少于4个，无法平滑")

    for key in (
        "latitude",
        "longitude",
        "altitude",
    ):
        if key not in origin:
            raise SystemExit(
                f"origin 缺少字段：{key}"
            )

    x = np.asarray(
        [float(point["x"]) for point in waypoints]
    )

    y = np.asarray(
        [float(point["y"]) for point in waypoints]
    )

    z = np.asarray(
        [
            float(point.get("z", 0.0))
            for point in waypoints
        ]
    )

    origin_latitude = float(
        origin["latitude"]
    )
    origin_longitude = float(
        origin["longitude"]
    )
    origin_altitude = float(
        origin["altitude"]
    )
    origin_yaw_deg = float(
        origin.get("yaw_deg", 0.0)
    )

    start_anchor_x = 0.0
    start_anchor_y = 0.0
    start_anchor_z = 0.0

    endpoint_anchor_source = "last_waypoint_fallback"
    endpoint_yaw_deg = None

    if isinstance(end_pose, dict):
        required_end_keys = (
            "latitude",
            "longitude",
            "altitude",
        )

        if all(
            key in end_pose
            for key in required_end_keys
        ):
            (
                end_anchor_x,
                end_anchor_y,
                end_anchor_z,
            ) = geodetic_to_enu(
                float(end_pose["latitude"]),
                float(end_pose["longitude"]),
                float(end_pose["altitude"]),
                origin_latitude,
                origin_longitude,
                origin_altitude,
            )

            if "yaw_deg" in end_pose:
                endpoint_yaw_deg = float(
                    end_pose["yaw_deg"]
                )

            endpoint_anchor_source = (
                "stable_post_record_average"
            )
        else:
            end_anchor_x = float(x[-1])
            end_anchor_y = float(y[-1])
            end_anchor_z = float(z[-1])
    else:
        end_anchor_x = float(x[-1])
        end_anchor_y = float(y[-1])
        end_anchor_z = float(z[-1])

    start_anchor_shift = math.hypot(
        float(x[0]) - start_anchor_x,
        float(y[0]) - start_anchor_y,
    )

    end_anchor_shift = math.hypot(
        float(x[-1]) - end_anchor_x,
        float(y[-1]) - end_anchor_y,
    )

    maximum_anchor_shift = max(
        0.0,
        float(args.maximum_anchor_shift),
    )

    if start_anchor_shift > maximum_anchor_shift:
        raise SystemExit(
            "稳定起点与原始首点相差过大："
            f"{start_anchor_shift:.3f}m > "
            f"{maximum_anchor_shift:.3f}m"
        )

    if end_anchor_shift > maximum_anchor_shift:
        raise SystemExit(
            "稳定终点与原始末点相差过大："
            f"{end_anchor_shift:.3f}m > "
            f"{maximum_anchor_shift:.3f}m"
        )

    before_metrics = route_metrics(x, y)

    x, y, z = remove_duplicate_points(
        x,
        y,
        z,
        minimum_distance=0.05,
    )

    (
        x,
        y,
        z,
        suppressed_spikes,
    ) = suppress_spikes(
        x,
        y,
        z,
        deviation_threshold=args.spike_threshold,
        angle_threshold_deg=args.spike_angle,
        passes=args.spike_passes,
    )

    source_s = cumulative_distance(x, y)
    source_total = float(source_s[-1])

    dense_s = make_uniform_samples(
        source_total,
        args.dense_spacing,
    )

    dense_x = np.interp(dense_s, source_s, x)
    dense_y = np.interp(dense_s, source_s, y)
    dense_z = np.interp(dense_s, source_s, z)

    total_length = max(
        float(dense_s[-1]),
        1e-6,
    )

    endpoint_hold = max(
        float(args.endpoint_hold),
        args.dense_spacing,
    )

    start_anchor_weight = np.clip(
        1.0 - dense_s / endpoint_hold,
        0.0,
        1.0,
    )

    end_anchor_weight = np.clip(
        1.0 - (
            total_length - dense_s
        ) / endpoint_hold,
        0.0,
        1.0,
    )

    combined_anchor_weight = (
        start_anchor_weight
        + end_anchor_weight
    )

    overlap = combined_anchor_weight > 1.0

    start_anchor_weight[overlap] /= (
        combined_anchor_weight[overlap]
    )

    end_anchor_weight[overlap] /= (
        combined_anchor_weight[overlap]
    )

    anchored_dense_x = (
        dense_x
        + start_anchor_weight
        * (start_anchor_x - dense_x[0])
        + end_anchor_weight
        * (end_anchor_x - dense_x[-1])
    )

    anchored_dense_y = (
        dense_y
        + start_anchor_weight
        * (start_anchor_y - dense_y[0])
        + end_anchor_weight
        * (end_anchor_y - dense_y[-1])
    )

    anchored_dense_z = (
        dense_z
        + start_anchor_weight
        * (start_anchor_z - dense_z[0])
        + end_anchor_weight
        * (end_anchor_z - dense_z[-1])
    )

    sigma_samples = (
        args.sigma / args.dense_spacing
    )

    filtered_x = gaussian_smooth(
        anchored_dense_x,
        sigma_samples,
    )

    filtered_y = gaussian_smooth(
        anchored_dense_y,
        sigma_samples,
    )

    filtered_z = gaussian_smooth(
        anchored_dense_z,
        sigma_samples,
    )

    endpoint_taper = np.minimum(
        1.0,
        np.minimum(
            dense_s /
            max(args.endpoint_hold, 1e-6),
            (
                total_length - dense_s
            ) /
            max(args.endpoint_hold, 1e-6),
        ),
    )

    endpoint_taper = np.clip(
        endpoint_taper,
        0.0,
        1.0,
    )

    delta_x = (
        filtered_x - anchored_dense_x
    ) * args.strength * endpoint_taper

    delta_y = (
        filtered_y - anchored_dense_y
    ) * args.strength * endpoint_taper

    delta_z = (
        filtered_z - anchored_dense_z
    ) * args.strength * endpoint_taper

    maximum_displacement = float(
        np.max(
            np.hypot(delta_x, delta_y)
        )
    )

    displacement_scale = 1.0

    if (
        maximum_displacement >
        args.maximum_shift
    ):
        displacement_scale = (
            args.maximum_shift /
            maximum_displacement
        )

    smooth_dense_x = (
        anchored_dense_x
        + delta_x * displacement_scale
    )

    smooth_dense_y = (
        anchored_dense_y
        + delta_y * displacement_scale
    )

    smooth_dense_z = (
        anchored_dense_z
        + delta_z * displacement_scale
    )

    smooth_dense_x[0] = start_anchor_x
    smooth_dense_y[0] = start_anchor_y
    smooth_dense_z[0] = start_anchor_z

    smooth_dense_x[-1] = end_anchor_x
    smooth_dense_y[-1] = end_anchor_y
    smooth_dense_z[-1] = end_anchor_z

    smooth_s = cumulative_distance(
        smooth_dense_x,
        smooth_dense_y,
    )

    final_s = make_uniform_samples(
        float(smooth_s[-1]),
        args.spacing,
    )

    final_x = np.interp(
        final_s,
        smooth_s,
        smooth_dense_x,
    )

    final_y = np.interp(
        final_s,
        smooth_s,
        smooth_dense_y,
    )

    final_z = np.interp(
        final_s,
        smooth_s,
        smooth_dense_z,
    )

    heading_window_points = max(
        1,
        int(round(
            args.heading_window_distance
            / args.spacing
        )),
    )

    ros_yaw = []

    for index in range(len(final_x)):
        first_index = max(
            0,
            index - heading_window_points,
        )

        second_index = min(
            len(final_x) - 1,
            index + heading_window_points,
        )

        dx = (
            final_x[second_index] -
            final_x[first_index]
        )

        dy = (
            final_y[second_index] -
            final_y[first_index]
        )

        yaw = math.degrees(
            math.atan2(dy, dx)
        )

        ros_yaw.append(
            normalize_angle_deg(yaw)
        )

    ros_yaw[0] = normalize_angle_deg(
        90.0 - origin_yaw_deg
    )

    if endpoint_yaw_deg is not None:
        ros_yaw[-1] = normalize_angle_deg(
            90.0 - endpoint_yaw_deg
        )

    new_waypoints = []

    for index in range(len(final_x)):
        source_index = int(
            np.searchsorted(
                source_s,
                min(final_s[index], source_s[-1]),
                side="left",
            )
        )

        source_index = min(
            max(source_index, 0),
            len(waypoints) - 1,
        )

        point = deepcopy(
            waypoints[source_index]
        )

        latitude, longitude, altitude = (
            enu_to_geodetic(
                float(final_x[index]),
                float(final_y[index]),
                float(final_z[index]),
                origin_latitude,
                origin_longitude,
                origin_altitude,
            )
        )

        heading = (
            90.0 - ros_yaw[index]
        ) % 360.0

        point["x"] = float(final_x[index])
        point["y"] = float(final_y[index])
        point["z"] = float(final_z[index])

        point["latitude"] = float(latitude)
        point["longitude"] = float(longitude)
        point["altitude"] = float(altitude)

        point["ros_yaw_deg"] = float(
            ros_yaw[index]
        )

        point["heading_deg"] = float(
            heading
        )

        if "index" in point:
            point["index"] = index

        for key in (
            "distance",
            "distance_m",
            "cumulative_distance",
            "cumulative_distance_m",
        ):
            if key in point:
                point[key] = float(final_s[index])

        new_waypoints.append(point)

    after_metrics = route_metrics(
        final_x,
        final_y,
    )

    result = deepcopy(data)
    result["format_version"] = 2
    result["waypoints"] = new_waypoints

    if "route_name" in result:
        result["route_name"] = output_path.stem

    summary = result.get("summary")

    if not isinstance(summary, dict):
        summary = {}

    summary["point_count"] = len(new_waypoints)
    summary["total_length"] = float(
        after_metrics["length"]
    )
    summary["total_length_m"] = round(
        after_metrics["length"],
        6,
    )

    result["summary"] = summary

    result["smoothing"] = {
        "created_at": datetime.now(
            timezone.utc
        ).isoformat(),
        "source_file": input_path.name,
        "method": (
            "spike suppression, Gaussian smoothing, "
            "arc-length resampling and tangent heading"
        ),
        "spacing_m": args.spacing,
        "dense_spacing_m": args.dense_spacing,
        "sigma_m": args.sigma,
        "strength": args.strength,
        "maximum_shift_m": args.maximum_shift,
        "endpoint_hold_m": args.endpoint_hold,
        "heading_window_distance_m":
            args.heading_window_distance,
        "maximum_anchor_shift_m":
            args.maximum_anchor_shift,
        "start_anchor_shift_m":
            start_anchor_shift,
        "end_anchor_shift_m":
            end_anchor_shift,
        "endpoint_anchor_source":
            endpoint_anchor_source,
        "suppressed_spikes": suppressed_spikes,
        "original_point_count": len(waypoints),
        "smoothed_point_count": len(new_waypoints),
        "original_length_m": round(
            before_metrics["length"],
            6,
        ),
        "smoothed_length_m": round(
            after_metrics["length"],
            6,
        ),
    }

    output_path.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    output_path.write_text(
        yaml.safe_dump(
            result,
            allow_unicode=True,
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    print("===== 平滑完成 =====")
    print("输入文件:", input_path)
    print("输出文件:", output_path)
    print("异常回摆点修正:", suppressed_spikes)
    print(
        "稳定起点偏差:",
        round(start_anchor_shift, 3),
        "m",
    )
    print(
        "稳定终点偏差:",
        round(end_anchor_shift, 3),
        "m",
    )
    print(
        "终点锚点来源:",
        endpoint_anchor_source,
    )
    print(
        "航向窗口:",
        round(args.heading_window_distance, 3),
        "m（前后各取）",
    )
    print(
        "最大平滑位移:",
        round(
            maximum_displacement *
            displacement_scale,
            3,
        ),
        "m",
    )

    print()
    print("===== 平滑前 =====")
    print(
        "轨迹点数:",
        len(waypoints),
    )
    print(
        "轨迹长度:",
        round(before_metrics["length"], 3),
        "m",
    )
    print(
        "局部偏移95%:",
        round(before_metrics["local_p95"], 3),
        "m",
    )
    print(
        "航向变化95%:",
        round(before_metrics["heading_p95"], 2),
        "deg",
    )
    print(
        "航向变化最大:",
        round(before_metrics["heading_max"], 2),
        "deg",
    )
    print(
        "左右转向切换:",
        before_metrics["turn_switches"],
    )

    print()
    print("===== 平滑后 =====")
    print(
        "轨迹点数:",
        len(new_waypoints),
    )
    print(
        "轨迹长度:",
        round(after_metrics["length"], 3),
        "m",
    )
    print(
        "局部偏移95%:",
        round(after_metrics["local_p95"], 3),
        "m",
    )
    print(
        "航向变化95%:",
        round(after_metrics["heading_p95"], 2),
        "deg",
    )
    print(
        "航向变化最大:",
        round(after_metrics["heading_max"], 2),
        "deg",
    )
    print(
        "左右转向切换:",
        after_metrics["turn_switches"],
    )


if __name__ == "__main__":
    main()
