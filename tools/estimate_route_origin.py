#!/usr/bin/env python3

import argparse
import math
import statistics
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Tuple

import rclpy
import yaml
from msg_out.msg import ImuStatus
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix


WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


def normalize_angle_deg(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


def circular_mean_deg(values: List[float]) -> float:
    sin_sum = sum(
        math.sin(math.radians(value))
        for value in values
    )
    cos_sum = sum(
        math.cos(math.radians(value))
        for value in values
    )

    if abs(sin_sum) < 1.0e-12 and abs(cos_sum) < 1.0e-12:
        raise ValueError('yaw samples have no stable mean')

    return math.degrees(math.atan2(sin_sum, cos_sum)) % 360.0


def circular_std_deg(values: List[float]) -> float:
    sin_mean = sum(
        math.sin(math.radians(value))
        for value in values
    ) / len(values)

    cos_mean = sum(
        math.cos(math.radians(value))
        for value in values
    ) / len(values)

    resultant = math.hypot(sin_mean, cos_mean)
    resultant = min(1.0, max(1.0e-12, resultant))

    return math.degrees(
        math.sqrt(-2.0 * math.log(resultant))
    )


def geodetic_to_ecef(
    latitude_deg: float,
    longitude_deg: float,
    altitude: float,
) -> Tuple[float, float, float]:
    latitude = math.radians(latitude_deg)
    longitude = math.radians(longitude_deg)

    sin_latitude = math.sin(latitude)
    cos_latitude = math.cos(latitude)
    sin_longitude = math.sin(longitude)
    cos_longitude = math.cos(longitude)

    radius = WGS84_A / math.sqrt(
        1.0 - WGS84_E2 * sin_latitude * sin_latitude
    )

    x = (
        radius + altitude
    ) * cos_latitude * cos_longitude

    y = (
        radius + altitude
    ) * cos_latitude * sin_longitude

    z = (
        radius * (1.0 - WGS84_E2) + altitude
    ) * sin_latitude

    return x, y, z


def ecef_to_geodetic(
    x: float,
    y: float,
    z: float,
) -> Tuple[float, float, float]:
    longitude = math.atan2(y, x)
    horizontal = math.hypot(x, y)

    latitude = math.atan2(
        z,
        horizontal * (1.0 - WGS84_E2),
    )

    altitude = 0.0

    for _ in range(15):
        sin_latitude = math.sin(latitude)

        radius = WGS84_A / math.sqrt(
            1.0 - WGS84_E2
            * sin_latitude
            * sin_latitude
        )

        cos_latitude = math.cos(latitude)

        if abs(cos_latitude) < 1.0e-12:
            altitude = (
                abs(z)
                - radius * (1.0 - WGS84_E2)
            )
        else:
            altitude = horizontal / cos_latitude - radius

        denominator = horizontal * (
            1.0
            - WGS84_E2
            * radius
            / (radius + altitude)
        )

        new_latitude = math.atan2(z, denominator)

        if abs(new_latitude - latitude) < 1.0e-13:
            latitude = new_latitude
            break

        latitude = new_latitude

    return (
        math.degrees(latitude),
        math.degrees(longitude),
        altitude,
    )


def geodetic_to_local(
    latitude: float,
    longitude: float,
    altitude: float,
    origin_latitude: float,
    origin_longitude: float,
    origin_altitude: float,
) -> Tuple[float, float, float]:
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

    latitude0 = math.radians(origin_latitude)
    longitude0 = math.radians(origin_longitude)

    sin_latitude = math.sin(latitude0)
    cos_latitude = math.cos(latitude0)
    sin_longitude = math.sin(longitude0)
    cos_longitude = math.cos(longitude0)

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


class OriginSampler(Node):

    def __init__(
        self,
        minimum_nsv1: int,
        minimum_nsv2: int,
        maximum_imu_age_sec: float,
    ) -> None:
        super().__init__('patrol_origin_sampler')

        self.minimum_nsv1 = minimum_nsv1
        self.minimum_nsv2 = minimum_nsv2
        self.maximum_imu_age_sec = maximum_imu_age_sec

        self.latest_imu = None
        self.latest_imu_received = 0.0

        self.samples: List[Dict[str, float]] = []
        self.first_valid_time = None

        self.create_subscription(
            ImuStatus,
            '/imu/status',
            self.imu_callback,
            50,
        )

        self.create_subscription(
            NavSatFix,
            '/gps/data',
            self.gps_callback,
            20,
        )

    def imu_callback(self, message: ImuStatus) -> None:
        self.latest_imu = message
        self.latest_imu_received = time.monotonic()

    def gps_callback(self, message: NavSatFix) -> None:
        now = time.monotonic()

        if self.latest_imu is None:
            return

        if (
            now - self.latest_imu_received
            > self.maximum_imu_age_sec
        ):
            return

        latitude = float(message.latitude)
        longitude = float(message.longitude)
        altitude = float(message.altitude)

        yaw = float(self.latest_imu.yaw)
        nsv1 = int(self.latest_imu.nsv1)
        nsv2 = int(self.latest_imu.nsv2)

        values = (
            latitude,
            longitude,
            altitude,
            yaw,
        )

        if not all(math.isfinite(value) for value in values):
            return

        if not -90.0 <= latitude <= 90.0:
            return

        if not -180.0 <= longitude <= 180.0:
            return

        if (
            abs(latitude) < 1.0e-9
            and abs(longitude) < 1.0e-9
        ):
            return

        if nsv1 < self.minimum_nsv1:
            return

        if nsv2 < self.minimum_nsv2:
            return

        if self.first_valid_time is None:
            self.first_valid_time = now

        self.samples.append({
            'latitude': latitude,
            'longitude': longitude,
            'altitude': altitude,
            'yaw_deg': yaw % 360.0,
            'gps_status': int(message.status.status),
            'nsv1': nsv1,
            'nsv2': nsv2,
        })


def estimate_origin(
    samples: List[Dict[str, float]],
    minimum_inlier_count: int,
    maximum_horizontal_rms_m: float,
    maximum_horizontal_error_m: float,
    maximum_yaw_std_deg: float,
) -> Dict:
    if len(samples) < minimum_inlier_count:
        raise RuntimeError(
            f'not enough valid samples: '
            f'{len(samples)} < {minimum_inlier_count}'
        )

    reference_latitude = statistics.median(
        sample['latitude']
        for sample in samples
    )

    reference_longitude = statistics.median(
        sample['longitude']
        for sample in samples
    )

    reference_altitude = statistics.median(
        sample['altitude']
        for sample in samples
    )

    local_samples = []

    for sample in samples:
        east, north, up = geodetic_to_local(
            sample['latitude'],
            sample['longitude'],
            sample['altitude'],
            reference_latitude,
            reference_longitude,
            reference_altitude,
        )

        local_samples.append({
            **sample,
            'east': east,
            'north': north,
            'up': up,
        })

    median_east = statistics.median(
        sample['east']
        for sample in local_samples
    )

    median_north = statistics.median(
        sample['north']
        for sample in local_samples
    )

    residuals = [
        math.hypot(
            sample['east'] - median_east,
            sample['north'] - median_north,
        )
        for sample in local_samples
    ]

    residual_median = statistics.median(residuals)

    residual_mad = statistics.median(
        abs(value - residual_median)
        for value in residuals
    )

    position_threshold = max(
        0.20,
        residual_median
        + 3.0 * 1.4826 * residual_mad,
    )

    position_threshold = min(
        maximum_horizontal_error_m,
        position_threshold,
    )

    position_inliers = [
        sample
        for sample, residual
        in zip(local_samples, residuals)
        if residual <= position_threshold
    ]

    required_inliers = max(
        minimum_inlier_count,
        math.ceil(len(samples) * 0.60),
    )

    if len(position_inliers) < required_inliers:
        raise RuntimeError(
            f'too many position outliers: '
            f'{len(position_inliers)}/{len(samples)}'
        )

    ecef_points = [
        geodetic_to_ecef(
            sample['latitude'],
            sample['longitude'],
            sample['altitude'],
        )
        for sample in position_inliers
    ]

    mean_x = statistics.fmean(
        point[0]
        for point in ecef_points
    )

    mean_y = statistics.fmean(
        point[1]
        for point in ecef_points
    )

    mean_z = statistics.fmean(
        point[2]
        for point in ecef_points
    )

    origin_latitude, origin_longitude, origin_altitude = (
        ecef_to_geodetic(
            mean_x,
            mean_y,
            mean_z,
        )
    )

    final_position_errors = []

    for sample in position_inliers:
        east, north, _ = geodetic_to_local(
            sample['latitude'],
            sample['longitude'],
            sample['altitude'],
            origin_latitude,
            origin_longitude,
            origin_altitude,
        )

        final_position_errors.append(
            math.hypot(east, north)
        )

    horizontal_rms = math.sqrt(
        statistics.fmean(
            value * value
            for value in final_position_errors
        )
    )

    horizontal_max = max(final_position_errors)

    if horizontal_rms > maximum_horizontal_rms_m:
        raise RuntimeError(
            f'position is not stable: '
            f'RMS={horizontal_rms:.3f}m > '
            f'{maximum_horizontal_rms_m:.3f}m'
        )

    initial_yaw = circular_mean_deg([
        sample['yaw_deg']
        for sample in position_inliers
    ])

    yaw_errors = [
        abs(normalize_angle_deg(
            sample['yaw_deg'] - initial_yaw
        ))
        for sample in position_inliers
    ]

    yaw_error_median = statistics.median(yaw_errors)

    yaw_error_mad = statistics.median(
        abs(value - yaw_error_median)
        for value in yaw_errors
    )

    yaw_threshold = max(
        2.0,
        yaw_error_median
        + 3.0 * 1.4826 * yaw_error_mad,
    )

    yaw_threshold = min(20.0, yaw_threshold)

    yaw_inliers = [
        sample
        for sample, error
        in zip(position_inliers, yaw_errors)
        if error <= yaw_threshold
    ]

    if len(yaw_inliers) < required_inliers:
        raise RuntimeError(
            f'too many yaw outliers: '
            f'{len(yaw_inliers)}/{len(samples)}'
        )

    origin_yaw = circular_mean_deg([
        sample['yaw_deg']
        for sample in yaw_inliers
    ])

    yaw_std = circular_std_deg([
        sample['yaw_deg']
        for sample in yaw_inliers
    ])

    yaw_max_error = max(
        abs(normalize_angle_deg(
            sample['yaw_deg'] - origin_yaw
        ))
        for sample in yaw_inliers
    )

    if yaw_std > maximum_yaw_std_deg:
        raise RuntimeError(
            f'yaw is not stable: '
            f'std={yaw_std:.3f}deg > '
            f'{maximum_yaw_std_deg:.3f}deg'
        )

    return {
        'origin': {
            'latitude': origin_latitude,
            'longitude': origin_longitude,
            'altitude': origin_altitude,
            'yaw_deg': origin_yaw,
        },
        'estimation': {
            'method': (
                '5-second robust position average '
                'and circular yaw average'
            ),
            'created_at': datetime.now(
                timezone.utc
            ).isoformat(),
            'total_samples': len(samples),
            'position_inliers': len(position_inliers),
            'yaw_inliers': len(yaw_inliers),
            'position_outlier_threshold_m':
                position_threshold,
            'horizontal_rms_m': horizontal_rms,
            'horizontal_max_error_m': horizontal_max,
            'yaw_outlier_threshold_deg': yaw_threshold,
            'yaw_std_deg': yaw_std,
            'yaw_max_error_deg': yaw_max_error,
            'minimum_nsv1': min(
                sample['nsv1']
                for sample in yaw_inliers
            ),
            'minimum_nsv2': min(
                sample['nsv2']
                for sample in yaw_inliers
            ),
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser()

    parser.add_argument(
        '--output',
        required=True,
    )

    parser.add_argument(
        '--duration',
        type=float,
        default=5.0,
    )

    parser.add_argument(
        '--maximum-wait',
        type=float,
        default=60.0,
    )

    parser.add_argument(
        '--minimum-samples',
        type=int,
        default=20,
    )

    parser.add_argument(
        '--minimum-nsv1',
        type=int,
        default=10,
    )

    parser.add_argument(
        '--minimum-nsv2',
        type=int,
        default=10,
    )

    parser.add_argument(
        '--maximum-horizontal-rms',
        type=float,
        default=0.80,
    )

    parser.add_argument(
        '--maximum-horizontal-error',
        type=float,
        default=2.00,
    )

    parser.add_argument(
        '--maximum-yaw-std',
        type=float,
        default=5.0,
    )

    arguments = parser.parse_args()

    rclpy.init()

    node = OriginSampler(
        minimum_nsv1=arguments.minimum_nsv1,
        minimum_nsv2=arguments.minimum_nsv2,
        maximum_imu_age_sec=0.50,
    )

    wait_started = time.monotonic()

    try:
        while rclpy.ok():
            rclpy.spin_once(
                node,
                timeout_sec=0.10,
            )

            now = time.monotonic()

            if (
                node.first_valid_time is not None
                and now - node.first_valid_time
                >= arguments.duration
                and len(node.samples)
                >= arguments.minimum_samples
            ):
                break

            if (
                now - wait_started
                > arguments.maximum_wait
            ):
                raise RuntimeError(
                    'timed out waiting for stable '
                    'GNSS and yaw data'
                )

        result = estimate_origin(
            node.samples,
            minimum_inlier_count=arguments.minimum_samples,
            maximum_horizontal_rms_m=(
                arguments.maximum_horizontal_rms
            ),
            maximum_horizontal_error_m=(
                arguments.maximum_horizontal_error
            ),
            maximum_yaw_std_deg=(
                arguments.maximum_yaw_std
            ),
        )

        output_path = Path(arguments.output)
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
            encoding='utf-8',
        )

        origin = result['origin']
        estimation = result['estimation']

        print(
            f"ORIGIN_LATITUDE={origin['latitude']:.10f}"
        )
        print(
            f"ORIGIN_LONGITUDE={origin['longitude']:.10f}"
        )
        print(
            f"ORIGIN_ALTITUDE={origin['altitude']:.4f}"
        )
        print(
            f"ORIGIN_YAW_DEG={origin['yaw_deg']:.4f}"
        )
        print(
            f"HORIZONTAL_RMS_M="
            f"{estimation['horizontal_rms_m']:.4f}"
        )
        print(
            f"YAW_STD_DEG="
            f"{estimation['yaw_std_deg']:.4f}"
        )
        print(f"OUTPUT_FILE={output_path}")

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
