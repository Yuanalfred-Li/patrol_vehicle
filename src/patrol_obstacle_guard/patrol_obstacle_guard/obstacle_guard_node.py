"""Forward clearance obstacle guard.

订阅前方净空距离（sensor_msgs/Range）与扫描健康心跳（JSON），
按分级策略持续发布限速/停车命令给 command manager：

- STALE：距离话题超时、健康心跳缺失/超时/不新鲜 -> 停车
- CLEAR：新鲜扫描且前方量程内无目标 -> 不限速
- SLOW ：距离在 (stop, slow] 区间 -> 限速 slow_speed_rpm
- STOP ：距离 <= stop_distance -> 停车；恢复需超过滞回并连续
          clear_hold_frames 帧确认

本节点只输出测量限速命令，不感知控制模式；
MANUAL 下是否忽略由 command manager 仲裁决定。
"""

from __future__ import annotations

import json
import math
import time
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import String
from vehicle_can_msg.msg import VehicleCommand


STALE = 'STALE'
CLEAR = 'CLEAR'
SLOW = 'SLOW'
STOP = 'STOP'


class ObstacleGuardNode(Node):

    def __init__(self) -> None:
        super().__init__('patrol_obstacle_guard')

        self.declare_parameter(
            'distance_topic',
            '/gap_follower/front_clearance_distance',
        )
        self.declare_parameter(
            'health_topic',
            '/gap_follower/front_clearance_health',
        )
        self.declare_parameter(
            'guard_command_topic',
            '/patrol/guard_command',
        )
        self.declare_parameter(
            'status_topic',
            '/patrol/obstacle_guard/status',
        )

        self.declare_parameter('control_rate_hz', 20.0)

        self.declare_parameter('stop_distance_m', 1.2)
        self.declare_parameter('slow_distance_m', 3.0)
        self.declare_parameter('slow_speed_rpm', 8.0)
        self.declare_parameter('clear_speed_rpm', 1000.0)
        self.declare_parameter('stop_hysteresis_m', 0.3)
        self.declare_parameter('clear_hold_frames', 5)

        self.declare_parameter('distance_timeout_sec', 0.5)
        self.declare_parameter('health_timeout_sec', 1.0)

        self.load_parameters()

        self.latest_distance: Optional[Range] = None
        self.distance_receive_time = 0.0

        self.health_fresh = False
        self.health_receive_time = 0.0

        self.state = STALE
        self.stop_clear_frames = 0

        self.command_pub = self.create_publisher(
            VehicleCommand,
            self.guard_command_topic,
            20,
        )
        self.status_pub = self.create_publisher(
            String,
            self.status_topic,
            10,
        )

        self.create_subscription(
            Range,
            self.distance_topic,
            self.distance_callback,
            10,
        )
        self.create_subscription(
            String,
            self.health_topic,
            self.health_callback,
            10,
        )

        rate = float(self.get_parameter('control_rate_hz').value)
        if rate <= 0.0:
            rate = 20.0
        self.create_timer(1.0 / rate, self.timer_callback)

        self.get_logger().info(
            'obstacle guard started: '
            f'distance={self.distance_topic}, '
            f'health={self.health_topic}, '
            f'stop<={self.stop_distance_m:.2f}m, '
            f'slow<={self.slow_distance_m:.2f}m'
        )

    def load_parameters(self) -> None:
        self.distance_topic = str(
            self.get_parameter('distance_topic').value
        )
        self.health_topic = str(
            self.get_parameter('health_topic').value
        )
        self.guard_command_topic = str(
            self.get_parameter('guard_command_topic').value
        )
        self.status_topic = str(
            self.get_parameter('status_topic').value
        )
        self.stop_distance_m = abs(float(
            self.get_parameter('stop_distance_m').value
        ))
        self.slow_distance_m = abs(float(
            self.get_parameter('slow_distance_m').value
        ))
        self.slow_speed_rpm = abs(float(
            self.get_parameter('slow_speed_rpm').value
        ))
        self.clear_speed_rpm = abs(float(
            self.get_parameter('clear_speed_rpm').value
        ))
        self.stop_hysteresis_m = abs(float(
            self.get_parameter('stop_hysteresis_m').value
        ))
        self.clear_hold_frames = max(
            1,
            int(self.get_parameter('clear_hold_frames').value),
        )
        self.distance_timeout_sec = abs(float(
            self.get_parameter('distance_timeout_sec').value
        ))
        self.health_timeout_sec = abs(float(
            self.get_parameter('health_timeout_sec').value
        ))

        if self.slow_distance_m <= self.stop_distance_m:
            raise ValueError(
                'slow_distance_m must be greater than stop_distance_m'
            )

    def distance_callback(self, msg: Range) -> None:
        self.latest_distance = msg
        self.distance_receive_time = time.monotonic()

    def health_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            self.health_fresh = bool(payload.get('data_fresh', False))
        except (ValueError, TypeError):
            self.health_fresh = False
        self.health_receive_time = time.monotonic()

    def timer_callback(self) -> None:
        now = time.monotonic()
        distance_fresh = (
            self.latest_distance is not None
            and now - self.distance_receive_time
            <= self.distance_timeout_sec
        )
        health_ok = (
            self.health_fresh
            and now - self.health_receive_time
            <= self.health_timeout_sec
        )

        if not distance_fresh or not health_ok:
            self.state = STALE
        else:
            self.update_state(self.latest_distance)

        command = self.make_command()
        self.command_pub.publish(command)
        self.status_pub.publish(self.make_status())

    def update_state(self, distance: Range) -> None:
        value = float(distance.range)

        if math.isinf(value):
            self.state = CLEAR
            return

        if value <= self.stop_distance_m:
            self.state = STOP
            self.stop_clear_frames = 0
            return

        if self.state == STOP:
            self.stop_clear_frames += 1
            clear_of_stop = (
                value > (
                    self.stop_distance_m
                    + self.stop_hysteresis_m
                )
                and self.stop_clear_frames
                >= self.clear_hold_frames
            )
            if not clear_of_stop:
                return

        if value < self.slow_distance_m:
            self.state = SLOW
        else:
            self.state = CLEAR

    def make_command(self) -> VehicleCommand:
        command = VehicleCommand()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = 'patrol_obstacle_guard'
        command.target_steering_angle_deg = 0.0
        command.control_mode = 1

        if self.state in (STOP, STALE):
            command.target_speed_rpm = 0.0
            command.brake_pedal = 100
            command.parking_brake = 0
            command.brake_light = True
        elif self.state == SLOW:
            command.target_speed_rpm = self.slow_speed_rpm
            command.brake_pedal = 0
            command.parking_brake = 1
        else:
            command.target_speed_rpm = self.clear_speed_rpm
            command.brake_pedal = 0
            command.parking_brake = 1

        return command

    def make_status(self) -> String:
        distance = None
        if self.latest_distance is not None:
            value = float(self.latest_distance.range)
            if math.isfinite(value):
                distance = round(value, 4)

        status = String()
        status.data = json.dumps({
            'state': self.state,
            'valid': self.state in (SLOW, STOP),
            'distance_m': distance,
            'health_fresh': self.health_fresh,
            'slow_distance_m': self.slow_distance_m,
            'stop_distance_m': self.stop_distance_m,
        }, ensure_ascii=False)
        return status


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObstacleGuardNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
