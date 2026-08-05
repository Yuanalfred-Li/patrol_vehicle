#!/usr/bin/env python3

import math
import time
from pathlib import Path
from typing import Optional, Tuple

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, TransformStamped
from msg_out.msg import ImuStatus
from nav_msgs.msg import Odometry
from patrol_interfaces.msg import LocalizationStatus
from patrol_interfaces.srv import LoadRoute
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import NavSatFix
from tf2_ros import TransformBroadcaster


WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


def geodetic_to_ecef(
    latitude_deg: float,
    longitude_deg: float,
    altitude: float,
) -> Tuple[float, float, float]:
    lat = math.radians(latitude_deg)
    lon = math.radians(longitude_deg)

    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)

    radius = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)

    x = (radius + altitude) * cos_lat * cos_lon
    y = (radius + altitude) * cos_lat * sin_lon
    z = (radius * (1.0 - WGS84_E2) + altitude) * sin_lat

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

    x0, y0, z0 = geodetic_to_ecef(
        origin_latitude_deg,
        origin_longitude_deg,
        origin_altitude,
    )

    dx = x - x0
    dy = y - y0
    dz = z - z0

    lat0 = math.radians(origin_latitude_deg)
    lon0 = math.radians(origin_longitude_deg)

    sin_lat0 = math.sin(lat0)
    cos_lat0 = math.cos(lat0)
    sin_lon0 = math.sin(lon0)
    cos_lon0 = math.cos(lon0)

    east = -sin_lon0 * dx + cos_lon0 * dy
    north = (
        -sin_lat0 * cos_lon0 * dx
        - sin_lat0 * sin_lon0 * dy
        + cos_lat0 * dz
    )
    up = (
        cos_lat0 * cos_lon0 * dx
        + cos_lat0 * sin_lon0 * dy
        + sin_lat0 * dz
    )

    return east, north, up


class PatrolLocalization(Node):

    def __init__(self) -> None:
        super().__init__('patrol_localization')

        self.declare_parameter('gps_topic', '/gps/data')
        self.declare_parameter('imu_status_topic', '/imu/status')

        self.declare_parameter('pose_topic', '/patrol/pose')
        self.declare_parameter('odom_topic', '/patrol/odom')
        self.declare_parameter(
            'status_topic',
            '/patrol/localization_status',
        )
        self.declare_parameter(
            'load_route_service',
            '/patrol/localization/load_route',
        )

        self.declare_parameter('frame_id', 'patrol_map')
        self.declare_parameter('child_frame_id', 'base_link')

        self.declare_parameter('origin_set', False)
        self.declare_parameter('origin_latitude', 0.0)
        self.declare_parameter('origin_longitude', 0.0)
        self.declare_parameter('origin_altitude', 0.0)

        self.declare_parameter('yaw_offset_deg', 0.0)
        self.declare_parameter('minimum_nsv1', 10)
        self.declare_parameter('minimum_nsv2', 10)
        self.declare_parameter('maximum_data_age_sec', 1.0)

        # 当前驱动的 /gps/data status 可能固定为 -1，
        # 第一阶段仅记录该值，不直接据此判定定位无效。
        self.declare_parameter(
            'accept_negative_navsat_status',
            True,
        )
        self.declare_parameter('publish_tf', True)

        self.gps_topic = str(
            self.get_parameter('gps_topic').value
        )
        self.imu_status_topic = str(
            self.get_parameter('imu_status_topic').value
        )

        self.pose_topic = str(
            self.get_parameter('pose_topic').value
        )
        self.odom_topic = str(
            self.get_parameter('odom_topic').value
        )
        self.status_topic = str(
            self.get_parameter('status_topic').value
        )

        self.frame_id = str(
            self.get_parameter('frame_id').value
        )
        self.child_frame_id = str(
            self.get_parameter('child_frame_id').value
        )

        self.latest_gps: Optional[NavSatFix] = None
        self.latest_imu: Optional[ImuStatus] = None

        self.gps_receive_time = 0.0
        self.imu_receive_time = 0.0

        # pose 与状态供后订阅节点立即可得最近值，
        # 避免控制节点晚启动时错过全部消息（启动竞态）。
        state_qos = QoSProfile(depth=10)
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.pose_pub = self.create_publisher(
            PoseStamped,
            self.pose_topic,
            state_qos,
        )
        self.odom_pub = self.create_publisher(
            Odometry,
            self.odom_topic,
            10,
        )
        self.status_pub = self.create_publisher(
            LocalizationStatus,
            self.status_topic,
            state_qos,
        )

        self.create_subscription(
            NavSatFix,
            self.gps_topic,
            self.gps_callback,
            20,
        )
        self.create_subscription(
            ImuStatus,
            self.imu_status_topic,
            self.imu_callback,
            20,
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

        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_timer(0.1, self.update)

        self.get_logger().info(
            'patrol_localization started; '
            f'gps={self.gps_topic}, '
            f'imu={self.imu_status_topic}'
        )

    def load_route_callback(
        self,
        request: LoadRoute.Request,
        response: LoadRoute.Response,
    ) -> LoadRoute.Response:
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

        if not route_path.is_file():
            response.success = False
            response.message = (
                f'route file not found: {route_path}'
            )
            return response

        try:
            with route_path.open(
                'r',
                encoding='utf-8',
            ) as file:
                data = yaml.safe_load(file)
        except (OSError, yaml.YAMLError) as exc:
            response.success = False
            response.message = (
                f'route YAML read failed: {exc}'
            )
            return response

        if not isinstance(data, dict):
            response.success = False
            response.message = (
                'route YAML root must be a mapping'
            )
            return response

        waypoints = data.get('waypoints')

        if (
            not isinstance(waypoints, list)
            or len(waypoints) < 2
        ):
            response.success = False
            response.message = (
                'route requires at least two waypoints'
            )
            return response

        origin = data.get('origin')

        if not isinstance(origin, dict):
            response.success = False
            response.message = (
                'route origin must be a mapping'
            )
            return response

        try:
            latitude = float(origin['latitude'])
            longitude = float(origin['longitude'])
            altitude = float(origin['altitude'])
        except (KeyError, TypeError, ValueError) as exc:
            response.success = False
            response.message = (
                f'invalid route origin: {exc}'
            )
            return response

        values = (
            latitude,
            longitude,
            altitude,
        )

        if not all(math.isfinite(value) for value in values):
            response.success = False
            response.message = (
                'route origin contains non-finite value'
            )
            return response

        if not -90.0 <= latitude <= 90.0:
            response.success = False
            response.message = (
                'route origin latitude is out of range'
            )
            return response

        if not -180.0 <= longitude <= 180.0:
            response.success = False
            response.message = (
                'route origin longitude is out of range'
            )
            return response

        result = self.set_parameters_atomically([
            Parameter(
                'origin_set',
                Parameter.Type.BOOL,
                True,
            ),
            Parameter(
                'origin_latitude',
                Parameter.Type.DOUBLE,
                latitude,
            ),
            Parameter(
                'origin_longitude',
                Parameter.Type.DOUBLE,
                longitude,
            ),
            Parameter(
                'origin_altitude',
                Parameter.Type.DOUBLE,
                altitude,
            ),
        ])

        if not result.successful:
            response.success = False
            response.message = (
                'failed to update localization origin: '
                f'{result.reason}'
            )
            return response

        response.success = True
        response.message = (
            'localization origin loaded from '
            f'{route_path.name}'
        )

        self.get_logger().warning(response.message)
        return response

    def gps_callback(self, msg: NavSatFix) -> None:
        self.latest_gps = msg
        self.gps_receive_time = time.monotonic()

    def imu_callback(self, msg: ImuStatus) -> None:
        self.latest_imu = msg
        self.imu_receive_time = time.monotonic()

    def update(self) -> None:
        now_monotonic = time.monotonic()
        reasons = []

        maximum_age = max(
            0.01,
            float(
                self.get_parameter(
                    'maximum_data_age_sec'
                ).value
            ),
        )

        status = LocalizationStatus()
        status.header.stamp = (
            self.get_clock().now().to_msg()
        )
        status.header.frame_id = self.frame_id

        status.east = math.nan
        status.north = math.nan

        gps_age = 1.0e9
        imu_age = 1.0e9

        gps_data_valid = False
        imu_data_valid = False
        origin_valid = False
        satellites_ok = False
        geometry_valid = False

        #######################################################################
        # 路线原点
        #######################################################################

        origin_set = bool(
            self.get_parameter('origin_set').value
        )

        origin_lat = float(
            self.get_parameter(
                'origin_latitude'
            ).value
        )
        origin_lon = float(
            self.get_parameter(
                'origin_longitude'
            ).value
        )
        origin_alt = float(
            self.get_parameter(
                'origin_altitude'
            ).value
        )

        if not origin_set:
            reasons.append('origin is not configured')
        elif not all(
            math.isfinite(value)
            for value in (
                origin_lat,
                origin_lon,
                origin_alt,
            )
        ):
            reasons.append(
                'origin contains non-finite value'
            )
        elif not -90.0 <= origin_lat <= 90.0:
            reasons.append(
                'origin latitude is out of range'
            )
        elif not -180.0 <= origin_lon <= 180.0:
            reasons.append(
                'origin longitude is out of range'
            )
        else:
            origin_valid = True

        #######################################################################
        # GPS数据
        #######################################################################

        if self.latest_gps is None:
            reasons.append('no GPS message')
        else:
            gps = self.latest_gps
            gps_age = max(
                0.0,
                now_monotonic - self.gps_receive_time,
            )

            status.gps_status = int(
                gps.status.status
            )
            status.latitude = float(gps.latitude)
            status.longitude = float(gps.longitude)
            status.altitude = float(gps.altitude)

            gps_reasons = []

            if gps_age > maximum_age:
                gps_reasons.append(
                    f'GPS data timeout: '
                    f'{gps_age:.2f}s'
                )

            if not all(
                math.isfinite(value)
                for value in (
                    gps.latitude,
                    gps.longitude,
                    gps.altitude,
                )
            ):
                gps_reasons.append(
                    'GPS contains non-finite value'
                )

            if not (
                -90.0
                <= float(gps.latitude)
                <= 90.0
            ):
                gps_reasons.append(
                    'GPS latitude is out of range'
                )

            if not (
                -180.0
                <= float(gps.longitude)
                <= 180.0
            ):
                gps_reasons.append(
                    'GPS longitude is out of range'
                )

            accept_negative = bool(
                self.get_parameter(
                    'accept_negative_navsat_status'
                ).value
            )

            if (
                int(gps.status.status) < 0
                and not accept_negative
            ):
                gps_reasons.append(
                    f'NavSatFix status invalid: '
                    f'{gps.status.status}'
                )

            gps_data_valid = not gps_reasons
            reasons.extend(gps_reasons)

        #######################################################################
        # IMU数据
        #######################################################################

        if self.latest_imu is None:
            reasons.append('no IMU status message')
        else:
            imu = self.latest_imu
            imu_age = max(
                0.0,
                now_monotonic - self.imu_receive_time,
            )

            status.nsv1 = int(imu.nsv1)
            status.nsv2 = int(imu.nsv2)
            status.yaw_deg = float(imu.yaw)
            status.nav_st = int(
            getattr(imu, 'nav_st', 0)
        )

            imu_reasons = []

            if imu_age > maximum_age:
                imu_reasons.append(
                    f'IMU data timeout: '
                    f'{imu_age:.2f}s'
                )

            if not math.isfinite(float(imu.yaw)):
                imu_reasons.append(
                    'IMU yaw is not finite'
                )

            imu_data_valid = not imu_reasons
            reasons.extend(imu_reasons)

            minimum_nsv1 = int(
                self.get_parameter(
                    'minimum_nsv1'
                ).value
            )
            minimum_nsv2 = int(
                self.get_parameter(
                    'minimum_nsv2'
                ).value
            )

            nsv1_ok = (
                int(imu.nsv1) >= minimum_nsv1
            )
            nsv2_ok = (
                int(imu.nsv2) >= minimum_nsv2
            )

            satellites_ok = nsv1_ok and nsv2_ok

            if not nsv1_ok:
                reasons.append(
                    f'nsv1 too low: {imu.nsv1}'
                )

            if not nsv2_ok:
                reasons.append(
                    f'nsv2 too low: {imu.nsv2}'
                )

        #######################################################################
        # GNSS质量与局部坐标
        #######################################################################

        gnss_quality_ok = (
            gps_data_valid
            and satellites_ok
        )

        east = math.nan
        north = math.nan
        up = math.nan

        if origin_valid and gps_data_valid:
            assert self.latest_gps is not None

            east, north, up = geodetic_to_enu(
                self.latest_gps.latitude,
                self.latest_gps.longitude,
                self.latest_gps.altitude,
                origin_lat,
                origin_lon,
                origin_alt,
            )

            geometry_valid = all(
                math.isfinite(value)
                for value in (
                    east,
                    north,
                    up,
                )
            )

            if not geometry_valid:
                reasons.append(
                    'ENU conversion produced '
                    'non-finite value'
                )

        # 仅当原点和IMU正常，而无效原因属于GNSS质量时，
        # 才允许上层考虑短时航向保持。
        degraded_candidate = (
            origin_valid
            and imu_data_valid
            and not gnss_quality_ok
        )

        status.gps_data_valid = bool(
            gps_data_valid
        )
        status.imu_data_valid = bool(
            imu_data_valid
        )
        status.origin_valid = bool(
            origin_valid
        )
        status.gnss_quality_ok = bool(
            gnss_quality_ok
        )
        status.gnss_degraded_candidate = bool(
            degraded_candidate
        )

        status.gps_age_sec = float(gps_age)
        status.imu_age_sec = float(imu_age)

        status.valid = bool(
            origin_valid
            and imu_data_valid
            and gnss_quality_ok
            and geometry_valid
        )

        status.reason = (
            'OK'
            if status.valid
            else '; '.join(reasons)
        )

        if geometry_valid:
            status.east = float(east)
            status.north = float(north)

        self.status_pub.publish(status)

        if not status.valid:
            return

        assert self.latest_imu is not None

        #######################################################################
        # 正常定位位姿
        #######################################################################

        heading_deg = float(
            self.latest_imu.yaw
        )
        yaw_offset_deg = float(
            self.get_parameter(
                'yaw_offset_deg'
            ).value
        )

        # MINS：北为0°，顺时针增加。
        # ROS：东为0°，逆时针增加。
        ros_yaw = math.radians(
            90.0
            - heading_deg
            + yaw_offset_deg
        )

        half_yaw = 0.5 * ros_yaw
        qz = math.sin(half_yaw)
        qw = math.cos(half_yaw)

        stamp = self.get_clock().now().to_msg()

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = east
        pose.pose.position.y = north
        pose.pose.position.z = up
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self.pose_pub.publish(pose)

        odom = Odometry()
        odom.header = pose.header
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose = pose.pose
        self.odom_pub.publish(odom)

        if bool(
            self.get_parameter(
                'publish_tf'
            ).value
        ):
            transform = TransformStamped()
            transform.header = pose.header
            transform.child_frame_id = (
                self.child_frame_id
            )
            transform.transform.translation.x = east
            transform.transform.translation.y = north
            transform.transform.translation.z = up
            transform.transform.rotation = (
                pose.pose.orientation
            )
            self.tf_broadcaster.sendTransform(
                transform
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PatrolLocalization()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
