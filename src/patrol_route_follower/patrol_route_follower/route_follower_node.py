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
from patrol_interfaces.srv import LoadRoute
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_srvs.srv import SetBool
from vehicle_can_msg.msg import VehicleCommand

from patrol_route_follower.gnss_hold_controller import (
    GnssHoldController,
    HOLD,
    HoldConfig,
    NORMAL,
    RESUME,
    STOP,
)


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
        self.declare_parameter(
            'load_route_service',
            '/patrol/route_follower/load_route',
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
            'handoff_recovery_distance',
            3.0,
        )
        self.declare_parameter(
            'handoff_recovery_speed_rpm',
            8.0,
        )
        self.declare_parameter(
            'steering_slowdown_start_request',
            180.0,
        )
        self.declare_parameter(
            'steering_slowdown_full_request',
            320.0,
        )
        self.declare_parameter(
            'steering_slowdown_speed_rpm',
            8.0,
        )

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

        self.declare_parameter(
            'gnss_degraded_enabled',
            False,
        )
        self.declare_parameter(
            'gnss_weak_debounce_sec',
            0.3,
        )
        self.declare_parameter(
            'gnss_maximum_hold_sec',
            3.0,
        )
        self.declare_parameter(
            'gnss_hold_speed_rpm',
            8.0,
        )
        self.declare_parameter(
            'gnss_maximum_heading_error_deg',
            8.0,
        )
        self.declare_parameter(
            'gnss_heading_control_kp',
            8.0,
        )
        self.declare_parameter(
            'gnss_maximum_steering_request',
            80.0,
        )
        self.declare_parameter(
            'gnss_maximum_imu_age_sec',
            0.5,
        )
        self.declare_parameter(
            'gnss_maximum_entry_steering_request',
            80.0,
        )
        self.declare_parameter(
            'gnss_minimum_straight_segment_length_m',
            4.0,
        )
        self.declare_parameter(
            'gnss_maximum_straight_heading_change_deg',
            5.0,
        )
        self.declare_parameter(
            'gnss_recovery_stable_sec',
            1.0,
        )
        self.declare_parameter(
            'gnss_recovery_max_path_error_m',
            1.0,
        )
        self.declare_parameter(
            'gnss_recovery_max_heading_error_deg',
            15.0,
        )
        self.declare_parameter(
            'gnss_recovery_max_progress_jump_m',
            2.0,
        )
        self.declare_parameter(
            'gnss_auto_resume_after_stop',
            False,
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
        self.route_start_progress_s = 0.0
        self.last_log_time = 0.0
        self.last_hold_status_time = 0.0
        self.last_hold_status_state = ''

        self.gnss_hold = GnssHoldController(
            HoldConfig(
                enabled=bool(
                    self.get_parameter(
                        'gnss_degraded_enabled'
                    ).value
                ),
                weak_debounce_sec=float(
                    self.get_parameter(
                        'gnss_weak_debounce_sec'
                    ).value
                ),
                maximum_hold_sec=float(
                    self.get_parameter(
                        'gnss_maximum_hold_sec'
                    ).value
                ),
                hold_speed_rpm=float(
                    self.get_parameter(
                        'gnss_hold_speed_rpm'
                    ).value
                ),
                maximum_heading_error_deg=float(
                    self.get_parameter(
                        'gnss_maximum_heading_error_deg'
                    ).value
                ),
                heading_control_kp=float(
                    self.get_parameter(
                        'gnss_heading_control_kp'
                    ).value
                ),
                maximum_steering_request=float(
                    self.get_parameter(
                        'gnss_maximum_steering_request'
                    ).value
                ),
                maximum_entry_steering_request=float(
                    self.get_parameter(
                        'gnss_maximum_entry_steering_request'
                    ).value
                ),
                minimum_straight_segment_length_m=float(
                    self.get_parameter(
                        'gnss_minimum_straight_segment_length_m'
                    ).value
                ),
                recovery_stable_sec=float(
                    self.get_parameter(
                        'gnss_recovery_stable_sec'
                    ).value
                ),
                recovery_max_path_error_m=float(
                    self.get_parameter(
                        'gnss_recovery_max_path_error_m'
                    ).value
                ),
                recovery_max_heading_error_deg=float(
                    self.get_parameter(
                        'gnss_recovery_max_heading_error_deg'
                    ).value
                ),
                recovery_max_progress_jump_m=float(
                    self.get_parameter(
                        'gnss_recovery_max_progress_jump_m'
                    ).value
                ),
                auto_resume_after_stop=bool(
                    self.get_parameter(
                        'gnss_auto_resume_after_stop'
                    ).value
                ),
            )
        )

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

        # 与 /patrol/pose 发布端保持一致，
        # 晚启动订阅时立即收到最近位姿，避免启动竞态。
        state_qos = QoSProfile(depth=20)
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self.pose_callback,
            state_qos,
        )
        self.create_subscription(
            LocalizationStatus,
            self.status_topic,
            self.status_callback,
            state_qos,
        )

        self.create_service(
            SetBool,
            '/patrol/route_follower/enable',
            self.enable_callback,
        )

        self.create_service(
            LoadRoute,
            str(
                self.get_parameter(
                    'load_route_service'
                ).value
            ),
            self.load_route_callback,
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

    def parse_route_file(
        self,
        route_file: Path,
    ) -> Tuple[
        str,
        List[Dict[str, float]],
        List[float],
        float,
    ]:
        if not route_file.is_file():
            raise FileNotFoundError(
                f'route file not found: {route_file}'
            )

        with route_file.open(
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

        frame_id = str(
            data.get('frame_id', 'patrol_map')
        )

        origin = parse_route_origin(data)
        points: List[Dict[str, float]] = []

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

        total_length = cumulative[-1]

        return (
            frame_id,
            points,
            cumulative,
            total_length,
        )

    def load_route(
        self,
        route_file: Optional[Path] = None,
    ) -> None:
        candidate = (
            self.route_file
            if route_file is None
            else route_file
        )

        (
            frame_id,
            points,
            cumulative,
            total_length,
        ) = self.parse_route_file(candidate)

        self.route_file = candidate
        self.frame_id = frame_id
        self.points = points
        self.cumulative_s = cumulative
        self.total_length = total_length

    def load_route_callback(
        self,
        request: LoadRoute.Request,
        response: LoadRoute.Response,
    ) -> LoadRoute.Response:
        if self.enabled:
            response.success = False
            response.message = (
                'cannot load route while route follower '
                'is enabled'
            )
            return response

        raw_path = str(request.route_file).strip()

        if not raw_path:
            response.success = False
            response.message = 'route_file is empty'
            return response

        route_path = Path(raw_path).expanduser()

        if not route_path.is_absolute():
            response.success = False
            response.message = (
                'route_file must be an absolute path'
            )
            return response

        route_path = route_path.resolve()

        try:
            (
                frame_id,
                points,
                cumulative,
                total_length,
            ) = self.parse_route_file(route_path)
        except Exception as exc:
            response.success = False
            response.message = (
                f'route load failed: {exc}'
            )
            return response

        parameter_result = self.set_parameters_atomically([
            Parameter(
                'route_file',
                Parameter.Type.STRING,
                str(route_path),
            ),
        ])

        if not parameter_result.successful:
            response.success = False
            response.message = (
                'failed to update route_file parameter: '
                f'{parameter_result.reason}'
            )
            return response

        # 所有校验完成后再整体替换，避免失败时破坏旧路线。
        self.route_file = route_path
        self.frame_id = frame_id
        self.points = points
        self.cumulative_s = cumulative
        self.total_length = total_length

        self.progress_initialized = False
        self.progress_s = 0.0
        self.route_start_progress_s = 0.0
        self.last_log_time = 0.0
        self.last_hold_status_time = 0.0
        self.last_hold_status_state = ''
        self.gnss_hold.reset()

        self.publish_stop()
        self.publish_route_path()
        self.publish_status(
            TaskStatus.IDLE,
            (
                f'route loaded: {route_path.name}; '
                f'points={len(points)}, '
                f'length={total_length:.2f}m'
            ),
            0.0,
        )

        response.success = True
        response.message = (
            f'route follower loaded {route_path.name}; '
            f'points={len(points)}, '
            f'length={total_length:.2f}m'
        )

        self.get_logger().warning(response.message)
        return response

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
            self.route_start_progress_s = 0.0
            self.gnss_hold.reset()
            self.last_hold_status_state = ''
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
        self.route_start_progress_s = self.progress_s
        self.progress_initialized = True
        self.gnss_hold.reset()
        self.last_hold_status_time = 0.0
        self.last_hold_status_state = ''
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

        status_reason = (
            self.localization_status_problem()
        )

        if status_reason is not None:
            self.safety_stop(status_reason)
            return

        assert self.latest_status is not None
        status = self.latest_status
        now = time.monotonic()

        #######################################################################
        # GNSS恢复阶段
        #######################################################################

        if status.valid:
            if self.gnss_hold.state != NORMAL:
                pose_reason = self.pose_problem()

                if pose_reason is not None:
                    decision = self.gnss_hold.update_bad(
                        now=now,
                        imu_heading_deg=float(
                            status.yaw_deg
                        ),
                    )
                    self.apply_hold_decision(
                        decision
                    )
                    return

                recovery = (
                    self.current_route_context()
                )

                decision = self.gnss_hold.update_good(
                    now=now,
                    imu_heading_deg=float(
                        status.yaw_deg
                    ),
                    recovery_path_error_m=float(
                        recovery['path_error']
                    ),
                    recovery_heading_error_deg=float(
                        recovery['heading_error_deg']
                    ),
                    recovered_progress_s=float(
                        recovery['projection']['s']
                    ),
                )

                if decision.action != RESUME:
                    self.apply_hold_decision(
                        decision
                    )
                    return

                if (
                    decision.recovered_progress_s
                    is not None
                ):
                    self.progress_s = max(
                        self.progress_s,
                        float(
                            decision.recovered_progress_s
                        ),
                    )

                self.last_hold_status_state = ''
                self.publish_status(
                    TaskStatus.RUNNING,
                    (
                        'GNSS recovery validated; '
                        'normal route following resumed'
                    ),
                    self.route_progress(),
                    task=(
                        'route_following/'
                        'GNSS_NORMAL'
                    ),
                )

            pose_reason = self.pose_problem()

            if pose_reason is not None:
                self.safety_stop(pose_reason)
                return

            self.run_normal_control()
            return

        #######################################################################
        # 定位无效：判断能否进入短时IMU航向保持
        #######################################################################

        if not self.gnss_hold.config.enabled:
            self.safety_stop(
                f'localization invalid: '
                f'{status.reason}'
            )
            return

        if not bool(
            status.gnss_degraded_candidate
        ):
            self.safety_stop(
                f'localization invalid and not '
                f'degradable: {status.reason}'
            )
            return

        maximum_imu_age = float(
            self.get_parameter(
                'gnss_maximum_imu_age_sec'
            ).value
        )

        if (
            not math.isfinite(
                float(status.imu_age_sec)
            )
            or float(status.imu_age_sec)
            > maximum_imu_age
        ):
            self.safety_stop(
                'IMU is too old for GNSS heading '
                f'hold: {status.imu_age_sec:.2f}s'
            )
            return

        imu_heading = float(status.yaw_deg)

        if self.gnss_hold.state == NORMAL:
            pose_reason = self.pose_problem()

            if pose_reason is not None:
                self.safety_stop(
                    'cannot begin GNSS heading hold: '
                    + pose_reason
                )
                return

            context = self.current_route_context()

            decision = self.gnss_hold.begin(
                now=now,
                target_heading_rad=float(
                    context['route_heading']
                ),
                pose_yaw_rad=float(
                    context['vehicle_yaw']
                ),
                imu_heading_deg=imu_heading,
                straight_remaining_m=float(
                    context['straight_remaining']
                ),
                frozen_progress_s=float(
                    self.progress_s
                ),
            )
        else:
            decision = self.gnss_hold.update_bad(
                now=now,
                imu_heading_deg=imu_heading,
            )

        self.apply_hold_decision(decision)

    def run_normal_control(self) -> None:
        assert self.latest_pose is not None

        pose = self.latest_pose.pose
        x = float(pose.position.x)
        y = float(pose.position.y)
        yaw = self.quaternion_to_yaw(
            pose.orientation
        )

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
            self.get_parameter(
                'final_tolerance'
            ).value
        )

        if (
            remaining <= final_tolerance
            and final_distance <= final_tolerance
        ):
            self.enabled = False
            self.gnss_hold.reset()
            self.publish_stop()

            message = (
                'route completed; vehicle stopped'
            )

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
            self.get_parameter(
                'wheelbase'
            ).value
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

        speed_rpm, speed_limit_reason = (
            self.calculate_speed(
                remaining,
                steering_request,
            )
        )

        command = VehicleCommand()
        command.header.stamp = (
            self.get_clock().now().to_msg()
        )
        command.header.frame_id = self.frame_id
        command.target_speed_rpm = float(
            speed_rpm
        )
        command.target_steering_angle_deg = (
            float(steering_request)
        )
        command.brake_pedal = 0
        command.parking_brake = 1
        command.control_mode = 1
        command.brake_light = False
        command.emergency_stop = False

        self.command_pub.publish(command)

        now = time.monotonic()

        if now - self.last_log_time >= 1.0:
            progress = self.route_progress()

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
                f'speed_limit={speed_limit_reason}, '
                f'steering={steering_request:.1f}'
            )

    def apply_hold_decision(
        self,
        decision,
    ) -> None:
        if decision.action == STOP:
            self.safety_stop(
                'GNSS heading hold rejected: '
                + str(decision.reason)
            )
            return

        if decision.action == RESUME:
            return

        if decision.action != HOLD:
            self.safety_stop(
                'unknown GNSS hold action: '
                + str(decision.action)
            )
            return

        self.publish_hold_command(
            decision.speed_rpm,
            decision.steering_request,
        )

        now = time.monotonic()
        state_changed = (
            decision.state
            != self.last_hold_status_state
        )

        if (
            state_changed
            or now - self.last_hold_status_time
            >= 0.20
        ):
            self.publish_status(
                TaskStatus.RUNNING,
                (
                    f'{decision.state}: '
                    f'remaining='
                    f'{decision.remaining_sec:.2f}s, '
                    f'heading_error='
                    f'{decision.heading_error_deg:.1f}deg, '
                    f'speed={decision.speed_rpm:.1f}rpm, '
                    f'steering='
                    f'{decision.steering_request:.1f}'
                ),
                self.route_progress(),
                task=(
                    'route_following/'
                    + str(decision.state)
                ),
            )

            self.last_hold_status_time = now
            self.last_hold_status_state = (
                str(decision.state)
            )

    def publish_hold_command(
        self,
        speed_rpm: float,
        steering_request: float,
    ) -> None:
        command = VehicleCommand()
        command.header.stamp = (
            self.get_clock().now().to_msg()
        )
        command.header.frame_id = self.frame_id
        command.target_speed_rpm = float(
            speed_rpm
        )
        command.target_steering_angle_deg = (
            float(steering_request)
        )
        command.brake_pedal = 0
        command.parking_brake = 1
        command.control_mode = 1
        command.brake_light = False
        command.emergency_stop = False

        self.command_pub.publish(command)

    def current_route_context(
        self,
    ) -> Dict[str, object]:
        assert self.latest_pose is not None

        pose = self.latest_pose.pose
        x = float(pose.position.x)
        y = float(pose.position.y)

        projection = self.project_to_route(x, y)
        segment_index = int(
            projection['segment']
        )
        route_heading = self.segment_heading(
            segment_index
        )
        vehicle_yaw = self.quaternion_to_yaw(
            pose.orientation
        )
        heading_error_deg = math.degrees(
            normalize_angle(
                vehicle_yaw - route_heading
            )
        )

        return {
            'projection': projection,
            'path_error': float(
                projection['distance']
            ),
            'route_heading': route_heading,
            'vehicle_yaw': vehicle_yaw,
            'heading_error_deg':
                heading_error_deg,
            'straight_remaining':
                self.straight_remaining(
                    projection,
                ),
        }

    def segment_heading(
        self,
        index: int,
    ) -> float:
        first = self.points[index]
        second = self.points[index + 1]

        return math.atan2(
            second['y'] - first['y'],
            second['x'] - first['x'],
        )

    def straight_remaining(
        self,
        projection: Dict[str, float],
    ) -> float:
        index = int(projection['segment'])
        base_heading = self.segment_heading(
            index
        )

        remaining = max(
            0.0,
            self.cumulative_s[index + 1]
            - float(projection['s']),
        )

        maximum_change = math.radians(
            abs(float(
                self.get_parameter(
                    'gnss_maximum_straight_heading_change_deg'
                ).value
            ))
        )

        for segment in range(
            index + 1,
            len(self.points) - 1,
        ):
            heading = self.segment_heading(
                segment
            )
            difference = abs(
                normalize_angle(
                    heading - base_heading
                )
            )

            if difference > maximum_change:
                break

            remaining += (
                self.cumulative_s[segment + 1]
                - self.cumulative_s[segment]
            )

        return float(remaining)

    def route_progress(self) -> float:
        if self.total_length <= 1.0e-6:
            return 0.0

        return max(
            0.0,
            min(
                1.0,
                self.progress_s
                / self.total_length,
            ),
        )

    def localization_status_problem(
        self,
    ) -> Optional[str]:
        if self.latest_status is None:
            return 'no localization status'

        maximum_age = float(
            self.get_parameter(
                'maximum_localization_age_sec'
            ).value
        )
        status_age = (
            time.monotonic()
            - self.status_receive_time
        )

        if status_age > maximum_age:
            return (
                'localization status timeout: '
                f'{status_age:.2f}s'
            )

        return None

    def pose_problem(self) -> Optional[str]:
        if self.latest_pose is None:
            return 'no pose'

        maximum_age = float(
            self.get_parameter(
                'maximum_localization_age_sec'
            ).value
        )
        pose_age = (
            time.monotonic()
            - self.pose_receive_time
        )

        if pose_age > maximum_age:
            return f'pose timeout: {pose_age:.2f}s'

        return None

    def localization_problem(self) -> Optional[str]:
        reason = self.localization_status_problem()

        if reason is not None:
            return reason

        assert self.latest_status is not None

        if not self.latest_status.valid:
            return (
                f'localization invalid: '
                f'{self.latest_status.reason}'
            )

        return self.pose_problem()

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

    def calculate_speed(
        self,
        remaining: float,
        steering_request: float,
    ) -> Tuple[float, str]:
        maximum_speed = abs(float(
            self.get_parameter('max_speed_rpm').value
        ))
        minimum_speed = abs(float(
            self.get_parameter(
                'minimum_speed_rpm'
            ).value
        ))
        minimum_speed = min(
            minimum_speed,
            maximum_speed,
        )

        slowdown_distance = max(
            0.01,
            float(
                self.get_parameter(
                    'slowdown_distance'
                ).value
            ),
        )

        if remaining >= slowdown_distance:
            base_speed = maximum_speed
            base_reason = 'normal'
        else:
            ratio = max(
                0.0,
                min(
                    1.0,
                    remaining / slowdown_distance,
                ),
            )

            base_speed = (
                minimum_speed
                + ratio
                * (maximum_speed - minimum_speed)
            )
            base_reason = 'endpoint'

        speed_limits = [
            (base_reason, base_speed),
        ]

        ###################################################################
        # 正式路线刚接管后的低速恢复阶段。
        ###################################################################

        recovery_distance = max(
            0.0,
            float(
                self.get_parameter(
                    'handoff_recovery_distance'
                ).value
            ),
        )

        recovery_speed = min(
            maximum_speed,
            abs(float(
                self.get_parameter(
                    'handoff_recovery_speed_rpm'
                ).value
            )),
        )

        travelled_since_handoff = max(
            0.0,
            self.progress_s
            - self.route_start_progress_s,
        )

        if (
            recovery_distance > 1.0e-6
            and travelled_since_handoff
            < recovery_distance
        ):
            recovery_ratio = max(
                0.0,
                min(
                    1.0,
                    travelled_since_handoff
                    / recovery_distance,
                ),
            )

            recovery_limit = (
                recovery_speed
                + recovery_ratio
                * (maximum_speed - recovery_speed)
            )

            speed_limits.append((
                'handoff_recovery',
                recovery_limit,
            ))

        ###################################################################
        # 大转向请求自动限速。
        ###################################################################

        steering_start = abs(float(
            self.get_parameter(
                'steering_slowdown_start_request'
            ).value
        ))

        steering_full = abs(float(
            self.get_parameter(
                'steering_slowdown_full_request'
            ).value
        ))

        if steering_full < steering_start:
            steering_start, steering_full = (
                steering_full,
                steering_start,
            )

        steering_speed = min(
            maximum_speed,
            abs(float(
                self.get_parameter(
                    'steering_slowdown_speed_rpm'
                ).value
            )),
        )

        absolute_steering = abs(
            float(steering_request)
        )

        if (
            steering_full
            <= steering_start + 1.0e-6
        ):
            if absolute_steering >= steering_full:
                speed_limits.append((
                    'steering_slowdown',
                    steering_speed,
                ))

        elif absolute_steering > steering_start:
            steering_ratio = max(
                0.0,
                min(
                    1.0,
                    (
                        absolute_steering
                        - steering_start
                    )
                    / (
                        steering_full
                        - steering_start
                    ),
                ),
            )

            steering_limit = (
                maximum_speed
                - steering_ratio
                * (maximum_speed - steering_speed)
            )

            speed_limits.append((
                'steering_slowdown',
                steering_limit,
            ))

        limit_reason, limited_speed = min(
            speed_limits,
            key=lambda item: item[1],
        )

        return (
            max(
                0.0,
                min(maximum_speed, limited_speed),
            ),
            limit_reason,
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
        task: str = 'route_following',
    ) -> None:
        status = TaskStatus()
        status.header.stamp = (
            self.get_clock().now().to_msg()
        )
        status.header.frame_id = self.frame_id
        status.state = int(state)
        status.task = str(task)
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
    except (KeyboardInterrupt, ExternalShutdownException):
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
