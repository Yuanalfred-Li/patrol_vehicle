#!/usr/bin/env python3

import math
import os
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as NavPath
from patrol_interfaces.msg import LocalizationStatus
from rclpy.node import Node
from std_srvs.srv import Trigger


class PatrolRouteRecorder(Node):

    def __init__(self) -> None:
        super().__init__('patrol_route_recorder')

        self.declare_parameter('pose_topic', '/patrol/pose')
        self.declare_parameter(
            'localization_status_topic',
            '/patrol/localization_status',
        )
        self.declare_parameter(
            'recorded_path_topic',
            '/patrol/recorded_path',
        )
        self.declare_parameter(
            'route_file',
            '/home/nvidia/patrol_ws/routes/route.yaml',
        )

        self.declare_parameter('frame_id', 'patrol_map')
        self.declare_parameter('minimum_point_spacing', 0.30)
        self.declare_parameter('maximum_status_age_sec', 0.50)

        self.declare_parameter('origin_set', False)
        self.declare_parameter('origin_latitude', 0.0)
        self.declare_parameter('origin_longitude', 0.0)
        self.declare_parameter('origin_altitude', 0.0)

        self.pose_topic = str(
            self.get_parameter('pose_topic').value
        )
        self.status_topic = str(
            self.get_parameter(
                'localization_status_topic'
            ).value
        )
        self.path_topic = str(
            self.get_parameter('recorded_path_topic').value
        )
        self.route_file = Path(
            str(self.get_parameter('route_file').value)
        )
        self.frame_id = str(
            self.get_parameter('frame_id').value
        )

        self.latest_status: Optional[LocalizationStatus] = None
        self.status_receive_time = 0.0

        self.recording = False
        self.points = []
        self.total_length = 0.0
        self.started_at = ''
        self.last_warning_time = 0.0

        self.create_subscription(
            LocalizationStatus,
            self.status_topic,
            self.status_callback,
            20,
        )
        self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self.pose_callback,
            20,
        )

        self.path_pub = self.create_publisher(
            NavPath,
            self.path_topic,
            10,
        )

        self.create_service(
            Trigger,
            '/patrol/route_recorder/start',
            self.start_callback,
        )
        self.create_service(
            Trigger,
            '/patrol/route_recorder/stop',
            self.stop_callback,
        )

        self.get_logger().info(
            'patrol_route_recorder started; '
            f'route_file={self.route_file}'
        )

    def status_callback(
        self,
        msg: LocalizationStatus,
    ) -> None:
        self.latest_status = msg
        self.status_receive_time = time.monotonic()

    def pose_callback(self, msg: PoseStamped) -> None:
        if not self.recording:
            return

        status = self.latest_status
        if status is None:
            self.warn_throttled('waiting for localization status')
            return

        maximum_age = float(
            self.get_parameter(
                'maximum_status_age_sec'
            ).value
        )
        status_age = (
            time.monotonic() - self.status_receive_time
        )

        if status_age > maximum_age:
            self.warn_throttled(
                f'localization status timeout: '
                f'{status_age:.2f}s'
            )
            return

        if not status.valid:
            self.warn_throttled(
                f'localization invalid: {status.reason}'
            )
            return

        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)
        z = float(msg.pose.position.z)

        if not all(math.isfinite(v) for v in (x, y, z)):
            self.warn_throttled(
                'pose contains non-finite value'
            )
            return

        minimum_spacing = float(
            self.get_parameter(
                'minimum_point_spacing'
            ).value
        )

        distance = 0.0
        if self.points:
            previous = self.points[-1]
            distance = math.hypot(
                x - previous['x'],
                y - previous['y'],
            )

            if distance < minimum_spacing:
                return

        q = msg.pose.orientation
        ros_yaw_deg = math.degrees(
            math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (
                    q.y * q.y + q.z * q.z
                ),
            )
        )

        point = {
            'index': len(self.points),
            'stamp': {
                'sec': int(msg.header.stamp.sec),
                'nanosec': int(msg.header.stamp.nanosec),
            },
            'x': x,
            'y': y,
            'z': z,
            'latitude': float(status.latitude),
            'longitude': float(status.longitude),
            'altitude': float(status.altitude),
            'heading_deg': float(status.yaw_deg),
            'ros_yaw_deg': ros_yaw_deg,
            'gps_status': int(status.gps_status),
            'nsv1': int(status.nsv1),
            'nsv2': int(status.nsv2),
        }

        self.points.append(point)
        self.total_length += distance
        self.publish_recorded_path()

        if len(self.points) == 1 or len(self.points) % 10 == 0:
            self.get_logger().info(
                f'recorded points={len(self.points)}, '
                f'length={self.total_length:.2f}m'
            )

    def start_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request

        if self.recording:
            response.success = False
            response.message = 'route recording already active'
            return response

        if not bool(
            self.get_parameter('origin_set').value
        ):
            response.success = False
            response.message = 'origin is not configured'
            return response

        self.points = []
        self.total_length = 0.0
        self.started_at = self.iso_time_now()
        self.recording = True

        self.publish_recorded_path()

        response.success = True
        response.message = (
            f'recording started: {self.route_file}'
        )

        self.get_logger().info(response.message)
        return response

    def stop_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request

        if not self.recording:
            response.success = False
            response.message = 'route recording is not active'
            return response

        self.recording = False

        if not self.points:
            response.success = False
            response.message = (
                'recording stopped, but no valid points '
                'were recorded'
            )
            self.get_logger().warning(response.message)
            return response

        try:
            self.save_route()
        except Exception as exc:
            response.success = False
            response.message = f'failed to save route: {exc}'
            self.get_logger().error(response.message)
            return response

        response.success = True
        response.message = (
            f'saved {len(self.points)} points, '
            f'length={self.total_length:.2f}m, '
            f'file={self.route_file}'
        )

        self.get_logger().info(response.message)
        return response

    def save_route(self) -> None:
        self.route_file.parent.mkdir(
            parents=True,
            exist_ok=True,
        )

        data = {
            'format_version': 1,
            'frame_id': self.frame_id,
            'created_at': self.started_at,
            'saved_at': self.iso_time_now(),
            'origin': {
                'latitude': float(
                    self.get_parameter(
                        'origin_latitude'
                    ).value
                ),
                'longitude': float(
                    self.get_parameter(
                        'origin_longitude'
                    ).value
                ),
                'altitude': float(
                    self.get_parameter(
                        'origin_altitude'
                    ).value
                ),
            },
            'recording': {
                'minimum_point_spacing': float(
                    self.get_parameter(
                        'minimum_point_spacing'
                    ).value
                ),
            },
            'summary': {
                'point_count': len(self.points),
                'total_length': self.total_length,
            },
            'waypoints': self.points,
        }

        temporary_file = Path(
            str(self.route_file) + '.tmp'
        )

        with temporary_file.open(
            'w',
            encoding='utf-8',
        ) as file:
            yaml.safe_dump(
                data,
                file,
                allow_unicode=True,
                sort_keys=False,
            )

        os.replace(temporary_file, self.route_file)

    def publish_recorded_path(self) -> None:
        path = NavPath()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.frame_id

        for point in self.points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = point['x']
            pose.pose.position.y = point['y']
            pose.pose.position.z = point['z']

            yaw = math.radians(point['ros_yaw_deg'])
            pose.pose.orientation.z = math.sin(yaw * 0.5)
            pose.pose.orientation.w = math.cos(yaw * 0.5)

            path.poses.append(pose)

        self.path_pub.publish(path)

    def warn_throttled(self, text: str) -> None:
        now = time.monotonic()
        if now - self.last_warning_time >= 2.0:
            self.last_warning_time = now
            self.get_logger().warning(text)

    @staticmethod
    def iso_time_now() -> str:
        return datetime.now(timezone.utc).isoformat()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PatrolRouteRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
