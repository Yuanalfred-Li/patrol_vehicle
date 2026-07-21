#!/usr/bin/env python3

import math
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as NavPath
from patrol_interfaces.msg import LocalizationStatus, TaskStatus
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_srvs.srv import SetBool
from vehicle_can_msg.msg import VehicleCommand


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


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


def geodetic_to_enu(
    latitude_deg: float,
    longitude_deg: float,
    altitude: float,
    origin_latitude_deg: float,
    origin_longitude_deg: float,
    origin_altitude: float,
) -> Tuple[float, float, float]:
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

    origin_latitude = math.radians(
        origin_latitude_deg
    )
    origin_longitude = math.radians(
        origin_longitude_deg
    )

    sin_latitude = math.sin(origin_latitude)
    cos_latitude = math.cos(origin_latitude)
    sin_longitude = math.sin(origin_longitude)
    cos_longitude = math.cos(origin_longitude)

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


def parse_route_origin(
    data: Dict,
) -> Optional[Tuple[float, float, float]]:
    origin = data.get('origin')

    if origin is None:
        return None

    if not isinstance(origin, dict):
        raise ValueError(
            'route origin must be a mapping'
        )

    try:
        latitude = float(origin['latitude'])
        longitude = float(origin['longitude'])
        altitude = float(origin['altitude'])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(
            f'invalid route origin: {exc}'
        ) from exc

    if not all(math.isfinite(value) for value in (
        latitude,
        longitude,
        altitude,
    )):
        raise ValueError(
            'route origin contains non-finite value'
        )

    if not -90.0 <= latitude <= 90.0:
        raise ValueError(
            'route origin latitude is out of range'
        )

    if not -180.0 <= longitude <= 180.0:
        raise ValueError(
            'route origin longitude is out of range'
        )

    return latitude, longitude, altitude


def route_waypoint_to_local(
    raw: Dict,
    index: int,
    origin: Optional[Tuple[float, float, float]],
) -> Tuple[float, float, float]:
    if not isinstance(raw, dict):
        raise ValueError(
            f'waypoint {index} must be a mapping'
        )

    absolute_keys = (
        'latitude',
        'longitude',
        'altitude',
    )
    absolute_count = sum(
        key in raw for key in absolute_keys
    )

    if absolute_count:
        if absolute_count != len(absolute_keys):
            raise ValueError(
                f'waypoint {index} has incomplete '
                'absolute coordinates'
            )

        if origin is None:
            raise ValueError(
                f'waypoint {index} has absolute '
                'coordinates but route origin is missing'
            )

        try:
            latitude = float(raw['latitude'])
            longitude = float(raw['longitude'])
            altitude = float(raw['altitude'])
        except (TypeError, ValueError) as exc:
            raise ValueError(
                f'invalid absolute waypoint {index}: {exc}'
            ) from exc

        if not all(math.isfinite(value) for value in (
            latitude,
            longitude,
            altitude,
        )):
            raise ValueError(
                f'waypoint {index} contains '
                'non-finite absolute coordinates'
            )

        if not -90.0 <= latitude <= 90.0:
            raise ValueError(
                f'waypoint {index} latitude '
                'is out of range'
            )

        if not -180.0 <= longitude <= 180.0:
            raise ValueError(
                f'waypoint {index} longitude '
                'is out of range'
            )

        return geodetic_to_enu(
            latitude,
            longitude,
            altitude,
            origin[0],
            origin[1],
            origin[2],
        )

    try:
        x = float(raw['x'])
        y = float(raw['y'])
        z = float(raw.get('z', 0.0))
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(
            f'invalid local waypoint {index}: {exc}'
        ) from exc

    if not all(math.isfinite(value) for value in (
        x,
        y,
        z,
    )):
        raise ValueError(
            f'waypoint {index} contains '
            'non-finite local coordinates'
        )

    return x, y, z


class PatrolRouteFollower(Node):

    def __init__(self) -> None:
        super().__init__('patrol_route_follower')

        self.declare_parameter(
            'route_file',
            '/home/nvidia/patrol_ws/routes/route.yaml',
        )
        self.declare_parameter('pose_topic', '/patrol/pose')
        self.declare_parameter(
            'localization_status_topic',
            '/patrol/localization_status',
        )
        self.declare_parameter(
            'command_topic',
            '/patrol/route_command',
        )
        self.declare_parameter(
            'route_path_topic',
            '/patrol/target_path',
        )
        self.declare_parameter(
            'lookahead_target_topic',
            '/patrol/lookahead_target',
        )
        self.declare_parameter(
            'status_topic',
            '/patrol/route_follower/status',
        )

        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('lookahead_distance', 1.0)

        self.declare_parameter('wheelbase', 0.67)
        self.declare_parameter(
            'max_mechanical_steering_deg',
            30.0,
        )
        self.declare_parameter(
            'max_steering_request',
            400.0,
        )

        self.declare_parameter('max_speed_rpm', 40.0)
        self.declare_parameter('minimum_speed_rpm', 20.0)
        self.declare_parameter('slowdown_distance', 2.0)

        self.declare_parameter(
            'maximum_entry_path_error',
            1.0,
        )
        self.declare_parameter(
            'maximum_entry_heading_error_deg',
            180.0,
        )
        self.declare_parameter(
            'maximum_tracking_error',
            1.5,
        )
        self.declare_parameter('final_tolerance', 0.30)
        self.declare_parameter(
            'maximum_localization_age_sec',
            0.50,
        )

        self.route_file = Path(
            str(self.get_parameter('route_file').value)
        )
        self.pose_topic = str(
            self.get_parameter('pose_topic').value
        )
        self.status_topic = str(
            self.get_parameter(
                'localization_status_topic'
            ).value
        )
        self.command_topic = str(
            self.get_parameter('command_topic').value
        )

        self.latest_pose: Optional[PoseStamped] = None
        self.latest_status: Optional[LocalizationStatus] = None
        self.pose_receive_time = 0.0
        self.status_receive_time = 0.0

        self.enabled = False
        self.progress_initialized = False
        self.progress_s = 0.0
        self.last_log_time = 0.0

        self.points: List[Dict[str, float]] = []
        self.cumulative_s: List[float] = []
        self.total_length = 0.0
        self.frame_id = 'patrol_map'

        self.load_route()

        self.command_pub = self.create_publisher(
            VehicleCommand,
            self.command_topic,
            20,
        )

        route_qos = QoSProfile(depth=1)
        route_qos.reliability = ReliabilityPolicy.RELIABLE
        route_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.route_path_pub = self.create_publisher(
            NavPath,
            str(
                self.get_parameter(
                    'route_path_topic'
                ).value
            ),
            route_qos,
        )

        self.lookahead_pub = self.create_publisher(
            PoseStamped,
            str(
                self.get_parameter(
                    'lookahead_target_topic'
                ).value
            ),
            10,
        )

        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.status_pub = self.create_publisher(
            TaskStatus,
            str(
                self.get_parameter(
                    'status_topic'
                ).value
            ),
            status_qos,
        )

        self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self.pose_callback,
            20,
        )
        self.create_subscription(
            LocalizationStatus,
            self.status_topic,
            self.status_callback,
            20,
        )

        self.create_service(
            SetBool,
            '/patrol/route_follower/enable',
            self.enable_callback,
        )

        rate = float(
            self.get_parameter('control_rate_hz').value
        )
        if rate <= 0.0:
            rate = 20.0

        self.create_timer(
            1.0 / rate,
            self.control_timer,
        )

        self.publish_route_path()
        self.publish_status(
            TaskStatus.IDLE,
            'route follower ready',
            0.0,
        )

        self.get_logger().info(
            f'route follower ready: '
            f'points={len(self.points)}, '
            f'length={self.total_length:.2f}m, '
            f'file={self.route_file}'
        )

    def load_route(self) -> None:
        if not self.route_file.is_file():
            raise FileNotFoundError(
                f'route file not found: '
                f'{self.route_file}'
            )

        with self.route_file.open(
            'r',
            encoding='utf-8',
        ) as file:
            data = yaml.safe_load(file)

        if not isinstance(data, dict):
            raise ValueError(
                'route YAML root must be a mapping'
            )

        raw_points = data.get('waypoints')

        if not isinstance(raw_points, list):
            raise ValueError(
                'route has no waypoints list'
            )

        if len(raw_points) < 2:
            raise ValueError(
                'route must contain at least '
                'two waypoints'
            )

        self.frame_id = str(
            data.get('frame_id', 'patrol_map')
        )

        origin = parse_route_origin(data)
        points = []

        for index, raw in enumerate(raw_points):
            x, y, z = route_waypoint_to_local(
                raw,
                index,
                origin,
            )

            points.append({
                'x': x,
                'y': y,
                'z': z,
            })

        cumulative = [0.0]

        for index in range(len(points) - 1):
            length = math.hypot(
                points[index + 1]['x']
                - points[index]['x'],
                points[index + 1]['y']
                - points[index]['y'],
            )

            if length < 1.0e-6:
                raise ValueError(
                    f'route segment {index + 1} '
                    'has zero length'
                )

            cumulative.append(
                cumulative[-1] + length
            )

        self.points = points
        self.cumulative_s = cumulative
        self.total_length = cumulative[-1]

    def pose_callback(self, msg: PoseStamped) -> None:
        self.latest_pose = msg
        self.pose_receive_time = time.monotonic()

    def status_callback(
        self,
        msg: LocalizationStatus,
    ) -> None:
        self.latest_status = msg
        self.status_receive_time = time.monotonic()

    def enable_callback(
        self,
        request: SetBool.Request,
        response: SetBool.Response,
    ) -> SetBool.Response:
        if not request.data:
            self.enabled = False
            self.progress_initialized = False
            self.publish_stop()

            response.success = True
            response.message = 'route follower disabled'

            self.publish_status(
                TaskStatus.IDLE,
                response.message,
                0.0,
            )

            self.get_logger().warning(response.message)
            return response

        reason = self.localization_problem()
        if reason is not None:
            response.success = False
            response.message = (
                f'cannot enable route follower: {reason}'
            )
            return response

        assert self.latest_pose is not None

        x = float(self.latest_pose.pose.position.x)
        y = float(self.latest_pose.pose.position.y)

        projection = self.project_to_route(x, y)
        path_error = projection['distance']

        maximum_entry_error = float(
            self.get_parameter(
                'maximum_entry_path_error'
            ).value
        )

        if path_error > maximum_entry_error:
            response.success = False
            response.message = (
                f'entry path error too large: '
                f'{path_error:.2f}m > '
                f'{maximum_entry_error:.2f}m'
            )
            return response

        segment_index = int(projection['segment'])
        segment_start = self.points[segment_index]
        segment_end = self.points[segment_index + 1]

        route_heading = math.atan2(
            segment_end['y'] - segment_start['y'],
            segment_end['x'] - segment_start['x'],
        )

        vehicle_heading = self.quaternion_to_yaw(
            self.latest_pose.pose.orientation
        )

        heading_error_deg = abs(math.degrees(
            normalize_angle(vehicle_heading - route_heading)
        ))

        maximum_heading_error = abs(float(
            self.get_parameter(
                'maximum_entry_heading_error_deg'
            ).value
        ))

        if heading_error_deg > maximum_heading_error:
            response.success = False
            response.message = (
                f'entry heading error too large: '
                f'{heading_error_deg:.1f}deg > '
                f'{maximum_heading_error:.1f}deg'
            )
            return response

        self.progress_s = projection['s']
        self.progress_initialized = True
        self.enabled = True

        response.success = True
        response.message = (
            f'route follower enabled: '
            f'progress={self.progress_s:.2f}/'
            f'{self.total_length:.2f}m, '
            f'path_error={path_error:.2f}m, '
            f'heading_error={heading_error_deg:.1f}deg'
        )

        self.publish_status(
            TaskStatus.RUNNING,
            response.message,
            (
                self.progress_s / self.total_length
                if self.total_length > 1.0e-6
                else 0.0
            ),
        )

        self.get_logger().warning(response.message)
        return response

    def control_timer(self) -> None:
        if not self.enabled:
            self.publish_stop()
            return

        reason = self.localization_problem()
        if reason is not None:
            self.safety_stop(reason)
            return

        assert self.latest_pose is not None

        pose = self.latest_pose.pose
        x = float(pose.position.x)
        y = float(pose.position.y)
        yaw = self.quaternion_to_yaw(pose.orientation)

        projection = self.project_to_route(x, y)

        maximum_error = float(
            self.get_parameter(
                'maximum_tracking_error'
            ).value
        )

        if projection['distance'] > maximum_error:
            self.safety_stop(
                f'path error too large: '
                f'{projection["distance"]:.2f}m'
            )
            return

        if not self.progress_initialized:
            self.progress_s = projection['s']
            self.progress_initialized = True
        else:
            self.progress_s = max(
                self.progress_s,
                projection['s'],
            )

        remaining = max(
            0.0,
            self.total_length - self.progress_s,
        )

        final_point = self.points[-1]
        final_distance = math.hypot(
            x - final_point['x'],
            y - final_point['y'],
        )
        final_tolerance = float(
            self.get_parameter('final_tolerance').value
        )

        if (
            remaining <= final_tolerance
            and final_distance <= final_tolerance
        ):
            self.enabled = False
            self.publish_stop()

            message = 'route completed; vehicle stopped'

            self.publish_status(
                TaskStatus.SUCCEEDED,
                message,
                1.0,
            )

            self.get_logger().warning(message)
            return

        lookahead = max(
            0.10,
            float(
                self.get_parameter(
                    'lookahead_distance'
                ).value
            ),
        )

        target_s = min(
            self.total_length,
            self.progress_s + lookahead,
        )
        target = self.point_at_s(target_s)

        self.publish_lookahead_target(target)

        dx = target['x'] - x
        dy = target['y'] - y

        target_distance = max(
            0.10,
            math.hypot(dx, dy),
        )
        target_heading = math.atan2(dy, dx)
        heading_error = normalize_angle(
            target_heading - yaw
        )

        wheelbase = float(
            self.get_parameter('wheelbase').value
        )

        steering_angle = math.atan2(
            2.0
            * wheelbase
            * math.sin(heading_error),
            target_distance,
        )

        maximum_steering_angle = math.radians(
            abs(float(
                self.get_parameter(
                    'max_mechanical_steering_deg'
                ).value
            ))
        )

        steering_angle = max(
            -maximum_steering_angle,
            min(
                maximum_steering_angle,
                steering_angle,
            ),
        )

        maximum_request = abs(float(
            self.get_parameter(
                'max_steering_request'
            ).value
        ))

        if maximum_steering_angle > 1.0e-6:
            steering_request = (
                steering_angle
                / maximum_steering_angle
                * maximum_request
            )
        else:
            steering_request = 0.0

        speed_rpm = self.calculate_speed(remaining)

        command = VehicleCommand()
        command.header.stamp = (
            self.get_clock().now().to_msg()
        )
        command.header.frame_id = self.frame_id

        command.target_speed_rpm = float(speed_rpm)
        command.target_steering_angle_deg = float(
            steering_request
        )

        command.brake_pedal = 0
        command.parking_brake = 1
        command.control_mode = 1
        command.brake_light = False
        command.emergency_stop = False

        self.command_pub.publish(command)

        now = time.monotonic()
        if now - self.last_log_time >= 1.0:
            progress = (
                self.progress_s / self.total_length
                if self.total_length > 1.0e-6
                else 0.0
            )

            self.publish_status(
                TaskStatus.RUNNING,
                (
                    f'progress={self.progress_s:.2f}/'
                    f'{self.total_length:.2f}m'
                ),
                progress,
            )
            self.last_log_time = now

            self.get_logger().info(
                f'progress={self.progress_s:.2f}/'
                f'{self.total_length:.2f}m, '
                f'error={projection["distance"]:.2f}m, '
                f'remaining={remaining:.2f}m, '
                f'speed={speed_rpm:.1f}rpm, '
                f'steering={steering_request:.1f}'
            )

    def localization_problem(self) -> Optional[str]:
        if self.latest_pose is None:
            return 'no pose'

        if self.latest_status is None:
            return 'no localization status'

        maximum_age = float(
            self.get_parameter(
                'maximum_localization_age_sec'
            ).value
        )

        pose_age = (
            time.monotonic() - self.pose_receive_time
        )
        status_age = (
            time.monotonic() - self.status_receive_time
        )

        if pose_age > maximum_age:
            return f'pose timeout: {pose_age:.2f}s'

        if status_age > maximum_age:
            return (
                f'localization status timeout: '
                f'{status_age:.2f}s'
            )

        if not self.latest_status.valid:
            return (
                f'localization invalid: '
                f'{self.latest_status.reason}'
            )

        return None

    def project_to_route(
        self,
        x: float,
        y: float,
    ) -> Dict[str, float]:
        best = None

        for index in range(len(self.points) - 1):
            first = self.points[index]
            second = self.points[index + 1]

            vx = second['x'] - first['x']
            vy = second['y'] - first['y']
            length_squared = vx * vx + vy * vy

            t = (
                (x - first['x']) * vx
                + (y - first['y']) * vy
            ) / length_squared

            t = max(0.0, min(1.0, t))

            projected_x = first['x'] + t * vx
            projected_y = first['y'] + t * vy

            distance = math.hypot(
                x - projected_x,
                y - projected_y,
            )

            segment_length = math.sqrt(length_squared)
            route_s = (
                self.cumulative_s[index]
                + t * segment_length
            )

            candidate = {
                'segment': float(index),
                't': t,
                'x': projected_x,
                'y': projected_y,
                's': route_s,
                'distance': distance,
            }

            if (
                best is None
                or distance < best['distance']
            ):
                best = candidate

        assert best is not None
        return best

    def point_at_s(
        self,
        route_s: float,
    ) -> Dict[str, float]:
        route_s = max(
            0.0,
            min(self.total_length, route_s),
        )

        for index in range(len(self.points) - 1):
            segment_start = self.cumulative_s[index]
            segment_end = self.cumulative_s[index + 1]

            if route_s <= segment_end:
                length = segment_end - segment_start
                ratio = (
                    (route_s - segment_start) / length
                    if length > 1.0e-9
                    else 0.0
                )

                first = self.points[index]
                second = self.points[index + 1]

                return {
                    'x': first['x']
                    + ratio * (second['x'] - first['x']),
                    'y': first['y']
                    + ratio * (second['y'] - first['y']),
                    'z': first['z']
                    + ratio * (second['z'] - first['z']),
                }

        return dict(self.points[-1])

    def calculate_speed(self, remaining: float) -> float:
        maximum_speed = abs(float(
            self.get_parameter('max_speed_rpm').value
        ))
        minimum_speed = abs(float(
            self.get_parameter(
                'minimum_speed_rpm'
            ).value
        ))
        minimum_speed = min(minimum_speed, maximum_speed)

        slowdown_distance = max(
            0.01,
            float(
                self.get_parameter(
                    'slowdown_distance'
                ).value
            ),
        )

        if remaining >= slowdown_distance:
            return maximum_speed

        ratio = max(
            0.0,
            min(1.0, remaining / slowdown_distance),
        )

        return (
            minimum_speed
            + ratio * (maximum_speed - minimum_speed)
        )

    def publish_stop(self) -> None:
        command = VehicleCommand()
        command.header.stamp = (
            self.get_clock().now().to_msg()
        )
        command.header.frame_id = self.frame_id

        command.target_speed_rpm = 0.0
        command.target_steering_angle_deg = 0.0
        command.brake_pedal = 0
        command.parking_brake = 1
        command.control_mode = 1
        command.brake_light = True

        self.command_pub.publish(command)

    def safety_stop(self, reason: str) -> None:
        self.enabled = False
        self.publish_stop()

        message = f'SAFETY_STOP: {reason}'

        self.publish_status(
            TaskStatus.FAILED,
            message,
            0.0,
        )

        self.get_logger().error(message)

    def publish_status(
        self,
        state: int,
        message: str,
        progress: float,
    ) -> None:
        status = TaskStatus()
        status.header.stamp = (
            self.get_clock().now().to_msg()
        )
        status.header.frame_id = self.frame_id
        status.state = int(state)
        status.task = 'route_following'
        status.message = str(message)
        status.progress = float(max(
            0.0,
            min(1.0, progress),
        ))

        self.status_pub.publish(status)

    def publish_route_path(self) -> None:
        message = NavPath()
        message.header.stamp = (
            self.get_clock().now().to_msg()
        )
        message.header.frame_id = self.frame_id

        for point in self.points:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = point['x']
            pose.pose.position.y = point['y']
            pose.pose.position.z = point['z']
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)

        self.route_path_pub.publish(message)

    def publish_lookahead_target(
        self,
        point: Dict[str, float],
    ) -> None:
        message = PoseStamped()
        message.header.stamp = (
            self.get_clock().now().to_msg()
        )
        message.header.frame_id = self.frame_id
        message.pose.position.x = point['x']
        message.pose.position.y = point['y']
        message.pose.position.z = point['z']
        message.pose.orientation.w = 1.0

        self.lookahead_pub.publish(message)

    @staticmethod
    def quaternion_to_yaw(quaternion) -> float:
        return math.atan2(
            2.0 * (
                quaternion.w * quaternion.z
                + quaternion.x * quaternion.y
            ),
            1.0 - 2.0 * (
                quaternion.y * quaternion.y
                + quaternion.z * quaternion.z
            ),
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PatrolRouteFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            for _ in range(5):
                node.publish_stop()
                time.sleep(0.03)

        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
