#!/usr/bin/env python3

import argparse
import math
from pathlib import Path
from typing import Dict, Tuple

import yaml


WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


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

    return (
        (radius + altitude)
        * cos_latitude
        * cos_longitude,
        (radius + altitude)
        * cos_latitude
        * sin_longitude,
        (
            radius * (1.0 - WGS84_E2)
            + altitude
        ) * sin_latitude,
    )


def geodetic_to_enu(
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

    x0, y0, z0 = geodetic_to_ecef(
        origin_latitude,
        origin_longitude,
        origin_altitude,
    )

    dx = x - x0
    dy = y - y0
    dz = z - z0

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


def get_coordinate(
    point: Dict,
    long_name: str,
    short_name: str,
) -> float:
    if long_name in point:
        return float(point[long_name])

    if short_name in point:
        return float(point[short_name])

    raise KeyError(
        f'waypoint has no {long_name}'
    )


def main() -> None:
    parser = argparse.ArgumentParser()

    parser.add_argument(
        '--route',
        required=True,
    )

    parser.add_argument(
        '--origin',
        required=True,
    )

    parser.add_argument(
        '--endpoint',
        required=False,
        default=None,
    )

    arguments = parser.parse_args()

    route_path = Path(arguments.route)
    origin_path = Path(arguments.origin)

    endpoint_path = (
        Path(arguments.endpoint)
        if arguments.endpoint
        else None
    )

    route_data = yaml.safe_load(
        route_path.read_text(encoding='utf-8')
    )

    origin_data = yaml.safe_load(
        origin_path.read_text(encoding='utf-8')
    )

    endpoint_data = None

    if endpoint_path is not None:
        endpoint_data = yaml.safe_load(
            endpoint_path.read_text(encoding='utf-8')
        )

    if not isinstance(route_data, dict):
        raise RuntimeError(
            'route YAML root must be a mapping'
        )

    if not isinstance(origin_data, dict):
        raise RuntimeError(
            'origin YAML root must be a mapping'
        )

    if (
        endpoint_data is not None
        and not isinstance(endpoint_data, dict)
    ):
        raise RuntimeError(
            'endpoint YAML root must be a mapping'
        )

    origin = origin_data['origin']

    origin_latitude = float(origin['latitude'])
    origin_longitude = float(origin['longitude'])
    origin_altitude = float(origin['altitude'])
    origin_yaw_deg = float(origin['yaw_deg'])

    waypoints = route_data.get('waypoints')

    if not isinstance(waypoints, list):
        raise RuntimeError(
            'route has no waypoint list'
        )

    if len(waypoints) < 2:
        raise RuntimeError(
            'route requires at least two waypoints'
        )

    total_length = 0.0
    previous = None

    for index, point in enumerate(waypoints):
        latitude = get_coordinate(
            point,
            'latitude',
            'lat',
        )

        longitude = get_coordinate(
            point,
            'longitude',
            'lon',
        )

        altitude = get_coordinate(
            point,
            'altitude',
            'alt',
        )

        east, north, up = geodetic_to_enu(
            latitude,
            longitude,
            altitude,
            origin_latitude,
            origin_longitude,
            origin_altitude,
        )

        point['index'] = index
        point['latitude'] = latitude
        point['longitude'] = longitude
        point['altitude'] = altitude
        point['x'] = east
        point['y'] = north
        point['z'] = up

        if previous is not None:
            total_length += math.hypot(
                east - previous[0],
                north - previous[1],
            )

        previous = (east, north)

    route_data['format_version'] = 2
    route_data['frame_id'] = 'patrol_map'

    route_data['coordinate_system'] = {
        'geodetic': 'WGS84',
        'local': 'ENU',
        'origin_source':
            'stable_pre_record_average',
    }

    route_data['origin'] = {
        'latitude': origin_latitude,
        'longitude': origin_longitude,
        'altitude': origin_altitude,
        'yaw_deg': origin_yaw_deg,
    }

    route_data['origin_estimation'] = (
        origin_data.get('estimation', {})
    )

    if endpoint_data is not None:
        endpoint = endpoint_data['origin']

        endpoint_latitude = float(
            endpoint['latitude']
        )
        endpoint_longitude = float(
            endpoint['longitude']
        )
        endpoint_altitude = float(
            endpoint['altitude']
        )
        endpoint_yaw_deg = float(
            endpoint['yaw_deg']
        )

        endpoint_east, endpoint_north, endpoint_up = (
            geodetic_to_enu(
                endpoint_latitude,
                endpoint_longitude,
                endpoint_altitude,
                origin_latitude,
                origin_longitude,
                origin_altitude,
            )
        )

        route_data['end_pose'] = {
            'latitude': endpoint_latitude,
            'longitude': endpoint_longitude,
            'altitude': endpoint_altitude,
            'yaw_deg': endpoint_yaw_deg,
            'x': endpoint_east,
            'y': endpoint_north,
            'z': endpoint_up,
        }

        route_data['end_pose_estimation'] = (
            endpoint_data.get('estimation', {})
        )

        route_data['coordinate_system'][
            'endpoint_source'
        ] = 'stable_post_record_average'

    summary = route_data.setdefault(
        'summary',
        {},
    )

    summary['point_count'] = len(waypoints)
    summary['total_length'] = total_length

    temporary_path = Path(
        str(route_path) + '.tmp'
    )

    temporary_path.write_text(
        yaml.safe_dump(
            route_data,
            allow_unicode=True,
            sort_keys=False,
        ),
        encoding='utf-8',
    )

    temporary_path.replace(route_path)

    print(f'ROUTE_FILE={route_path}')
    print(f'POINT_COUNT={len(waypoints)}')
    print(f'TOTAL_LENGTH={total_length:.3f}')
    print(
        f'ORIGIN={origin_latitude:.10f},'
        f'{origin_longitude:.10f},'
        f'{origin_altitude:.4f}'
    )
    print(f'ORIGIN_YAW_DEG={origin_yaw_deg:.4f}')

    if endpoint_data is not None:
        print(
            f'ENDPOINT={endpoint_latitude:.10f},'
            f'{endpoint_longitude:.10f},'
            f'{endpoint_altitude:.4f}'
        )
        print(
            f'ENDPOINT_YAW_DEG='
            f'{endpoint_yaw_deg:.4f}'
        )
        print(
            f'ENDPOINT_ENU='
            f'{endpoint_east:.3f},'
            f'{endpoint_north:.3f},'
            f'{endpoint_up:.3f}'
        )


if __name__ == '__main__':
    main()
