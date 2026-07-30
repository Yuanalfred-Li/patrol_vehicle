#!/usr/bin/env python3

import time
from typing import Optional

import rclpy
from patrol_interfaces.msg import TaskStatus
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from vehicle_can_msg.msg import VehicleCommand


class PatrolAutoCommandMux(Node):

    def __init__(self) -> None:
        super().__init__('patrol_auto_command_mux')

        self.declare_parameter(
            'entry_command_topic',
            '/patrol/entry_command',
        )
        self.declare_parameter(
            'route_command_topic',
            '/patrol/route_command',
        )
        self.declare_parameter(
            'output_command_topic',
            '/patrol/auto_command',
        )
        self.declare_parameter(
            'entry_status_topic',
            '/patrol/entry_executor/status',
        )
        self.declare_parameter(
            'route_status_topic',
            '/patrol/route_follower/status',
        )
        self.declare_parameter('command_timeout_sec', 0.50)
        self.declare_parameter('publish_rate_hz', 20.0)

        self.entry_command: Optional[VehicleCommand] = None
        self.route_command: Optional[VehicleCommand] = None
        self.entry_status: Optional[TaskStatus] = None
        self.route_status: Optional[TaskStatus] = None

        self.entry_command_time = 0.0
        self.route_command_time = 0.0
        self.active_source = 'STOP'

        self.last_warning = ''
        self.last_warning_time = 0.0

        self.output_pub = self.create_publisher(
            VehicleCommand,
            str(
                self.get_parameter(
                    'output_command_topic'
                ).value
            ),
            20,
        )

        self.create_subscription(
            VehicleCommand,
            str(
                self.get_parameter(
                    'entry_command_topic'
                ).value
            ),
            self.entry_command_callback,
            20,
        )

        self.create_subscription(
            VehicleCommand,
            str(
                self.get_parameter(
                    'route_command_topic'
                ).value
            ),
            self.route_command_callback,
            20,
        )

        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(
            TaskStatus,
            str(
                self.get_parameter(
                    'entry_status_topic'
                ).value
            ),
            self.entry_status_callback,
            status_qos,
        )

        self.create_subscription(
            TaskStatus,
            str(
                self.get_parameter(
                    'route_status_topic'
                ).value
            ),
            self.route_status_callback,
            status_qos,
        )

        rate = float(
            self.get_parameter('publish_rate_hz').value
        )

        if rate <= 0.0:
            rate = 20.0

        self.create_timer(1.0 / rate, self.control_timer)

        self.get_logger().info(
            'automatic command mux ready: '
            '/patrol/entry_command + '
            '/patrol/route_command -> '
            '/patrol/auto_command'
        )

    def entry_command_callback(
        self,
        msg: VehicleCommand,
    ) -> None:
        self.entry_command = msg
        self.entry_command_time = time.monotonic()

    def route_command_callback(
        self,
        msg: VehicleCommand,
    ) -> None:
        self.route_command = msg
        self.route_command_time = time.monotonic()

    def entry_status_callback(
        self,
        msg: TaskStatus,
    ) -> None:
        self.entry_status = msg

    def route_status_callback(
        self,
        msg: TaskStatus,
    ) -> None:
        self.route_status = msg

    def control_timer(self) -> None:
        entry_running = (
            self.entry_status is not None
            and self.entry_status.state
            == TaskStatus.RUNNING
        )

        route_running = (
            self.route_status is not None
            and self.route_status.state
            == TaskStatus.RUNNING
        )

        if entry_running and route_running:
            self.set_source('STOP')
            self.publish_stop()
            self.log_warning(
                'entry and route controllers are '
                'both RUNNING; output stopped'
            )
            return

        if entry_running:
            self.forward_command(
                'ENTRY',
                self.entry_command,
                self.entry_command_time,
            )
            return

        if route_running:
            self.forward_command(
                'ROUTE',
                self.route_command,
                self.route_command_time,
            )
            return

        self.set_source('STOP')
        self.publish_stop()

    def forward_command(
        self,
        source: str,
        command: Optional[VehicleCommand],
        receive_time: float,
    ) -> None:
        timeout = max(
            0.05,
            float(
                self.get_parameter(
                    'command_timeout_sec'
                ).value
            ),
        )

        age = time.monotonic() - receive_time

        if command is None or age > timeout:
            self.set_source('STOP')
            self.publish_stop()
            self.log_warning(
                f'{source.lower()} command timeout: '
                f'{age:.2f}s'
            )
            return

        self.set_source(source)
        self.output_pub.publish(command)

    def set_source(self, source: str) -> None:
        if source == self.active_source:
            return

        self.active_source = source
        self.get_logger().warning(
            f'automatic command source: {source}'
        )

    def log_warning(self, message: str) -> None:
        now = time.monotonic()

        if (
            message != self.last_warning
            or now - self.last_warning_time >= 2.0
        ):
            self.last_warning = message
            self.last_warning_time = now
            self.get_logger().error(message)

    def publish_stop(self) -> None:
        command = VehicleCommand()
        command.header.stamp = (
            self.get_clock().now().to_msg()
        )
        command.header.frame_id = 'patrol_map'
        command.target_speed_rpm = 0.0
        command.target_steering_angle_deg = 0.0
        command.brake_pedal = 0
        command.parking_brake = 1
        command.control_mode = 1
        command.brake_light = True
        command.emergency_stop = False

        self.output_pub.publish(command)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PatrolAutoCommandMux()

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
