#!/usr/bin/env python3

import math
import time
from typing import Optional

import rclpy
from patrol_interfaces.msg import ControlMode
from patrol_interfaces.srv import SetControlMode
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from vehicle_can_msg.msg import VehicleCommand


STOP = 0
MANUAL = 1
AUTO = 2


class PatrolCommandManager(Node):

    def __init__(self) -> None:
        super().__init__('patrol_command_manager')

        self.declare_parameter(
            'manual_command_topic',
            '/patrol/manual_command',
        )
        self.declare_parameter(
            'auto_command_topic',
            '/patrol/auto_command',
        )
        self.declare_parameter(
            'vehicle_command_topic',
            '/vehicle/command',
        )
        self.declare_parameter(
            'mode_topic',
            '/patrol/control_mode',
        )
        self.declare_parameter(
            'set_mode_service',
            '/patrol/set_control_mode',
        )

        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('command_timeout_sec', 0.50)

        self.declare_parameter('max_speed_rpm', 40.0)
        self.declare_parameter('max_steering_request', 400.0)

        self.declare_parameter('stop_brake_pedal', 0)
        self.declare_parameter('stop_parking_brake', 0)

        manual_topic = str(
            self.get_parameter('manual_command_topic').value
        )
        auto_topic = str(
            self.get_parameter('auto_command_topic').value
        )
        output_topic = str(
            self.get_parameter('vehicle_command_topic').value
        )
        mode_topic = str(
            self.get_parameter('mode_topic').value
        )
        service_name = str(
            self.get_parameter('set_mode_service').value
        )

        self.mode = STOP
        self.mode_source = 'startup'

        self.latest_manual: Optional[VehicleCommand] = None
        self.latest_auto: Optional[VehicleCommand] = None

        self.manual_receive_time = 0.0
        self.auto_receive_time = 0.0

        self.output_pub = self.create_publisher(
            VehicleCommand,
            output_topic,
            20,
        )
        self.mode_pub = self.create_publisher(
            ControlMode,
            mode_topic,
            10,
        )

        self.create_subscription(
            VehicleCommand,
            manual_topic,
            self.manual_callback,
            20,
        )
        self.create_subscription(
            VehicleCommand,
            auto_topic,
            self.auto_callback,
            20,
        )

        self.create_service(
            SetControlMode,
            service_name,
            self.set_mode_callback,
        )

        publish_rate = float(
            self.get_parameter('publish_rate_hz').value
        )
        if publish_rate <= 0.0:
            publish_rate = 20.0

        self.create_timer(
            1.0 / publish_rate,
            self.timer_callback,
        )

        self.publish_mode()

        self.get_logger().info(
            'patrol_command_manager started in STOP mode; '
            f'manual={manual_topic}, '
            f'auto={auto_topic}, '
            f'output={output_topic}'
        )

    def manual_callback(self, msg: VehicleCommand) -> None:
        self.latest_manual = msg
        self.manual_receive_time = time.monotonic()

    def auto_callback(self, msg: VehicleCommand) -> None:
        self.latest_auto = msg
        self.auto_receive_time = time.monotonic()

    def set_mode_callback(
        self,
        request: SetControlMode.Request,
        response: SetControlMode.Response,
    ) -> SetControlMode.Response:
        requested_mode = int(request.mode)

        if requested_mode not in (STOP, MANUAL, AUTO):
            response.success = False
            response.message = (
                f'invalid mode {requested_mode}; '
                'valid modes: STOP=0, MANUAL=1, AUTO=2'
            )
            return response

        previous_mode = self.mode
        self.mode = requested_mode
        self.mode_source = 'set_control_mode service'

        # 切换模式时清除旧速度命令。
        # STOP 模式启用驻车；MANUAL/AUTO 保持驻车释放。
        if self.mode == STOP:
            self.output_pub.publish(self.make_stop_command())
        else:
            self.output_pub.publish(
                self.make_motion_stop_command()
            )
        self.publish_mode()

        response.success = True
        response.message = (
            f'control mode changed: '
            f'{self.mode_name(previous_mode)} -> '
            f'{self.mode_name(self.mode)}'
        )

        self.get_logger().warning(response.message)
        return response

    def timer_callback(self) -> None:
        now = time.monotonic()
        timeout = float(
            self.get_parameter('command_timeout_sec').value
        )

        if self.mode == STOP:
            output = self.make_stop_command()

        elif self.mode == MANUAL:
            if (
                self.latest_manual is None
                or now - self.manual_receive_time > timeout
            ):
                output = self.make_motion_stop_command()
            else:
                output = self.sanitize_command(
                    self.latest_manual
                )

        elif self.mode == AUTO:
            if (
                self.latest_auto is None
                or now - self.auto_receive_time > timeout
            ):
                output = self.make_motion_stop_command()
            else:
                output = self.sanitize_command(
                    self.latest_auto
                )

        else:
            output = self.make_stop_command()

        self.output_pub.publish(output)
        self.publish_mode()

    def sanitize_command(
        self,
        source: VehicleCommand,
    ) -> VehicleCommand:
        command = VehicleCommand()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = source.header.frame_id

        max_speed = abs(float(
            self.get_parameter('max_speed_rpm').value
        ))
        max_steering = abs(float(
            self.get_parameter(
                'max_steering_request'
            ).value
        ))

        speed = float(source.target_speed_rpm)
        steering = float(source.target_steering_angle_deg)

        if not math.isfinite(speed):
            speed = 0.0
        if not math.isfinite(steering):
            steering = 0.0

        command.target_speed_rpm = max(
            -max_speed,
            min(max_speed, speed),
        )
        command.target_steering_angle_deg = max(
            -max_steering,
            min(max_steering, steering),
        )

        command.brake_pedal = max(
            0,
            min(100, int(source.brake_pedal)),
        )

        # 所有 ROS 手动/自动命令都通过底盘自动控制通道发送。
        command.parking_brake = 1
        command.control_mode = 1

        command.headlamp = bool(source.headlamp)
        command.left_net_catch = bool(
            source.left_net_catch
        )
        command.left_turn_light = bool(
            source.left_turn_light
        )
        command.right_net_catch = bool(
            source.right_net_catch
        )
        command.right_turn_light = bool(
            source.right_turn_light
        )
        command.flash_light = bool(source.flash_light)
        command.brake_light = bool(source.brake_light)
        command.emergency_stop = bool(
            source.emergency_stop
        )
        command.net_launch_enable = bool(
            source.net_launch_enable
        )

        if command.emergency_stop:
            command.target_speed_rpm = 0.0
            command.parking_brake = 0
            command.brake_light = True

        return command

    def make_motion_stop_command(self) -> VehicleCommand:
        """停止运动，但保持驻车制动释放。"""
        command = VehicleCommand()
        command.header.stamp = self.get_clock().now().to_msg()

        command.target_speed_rpm = 0.0
        command.target_steering_angle_deg = 0.0
        command.brake_pedal = 0

        # 底盘协议：1=释放驻车，0=启用驻车。
        command.parking_brake = 1
        command.control_mode = 1
        command.brake_light = True
        command.emergency_stop = False

        return command

    def make_stop_command(self) -> VehicleCommand:
        command = VehicleCommand()
        command.header.stamp = self.get_clock().now().to_msg()

        command.target_speed_rpm = 0.0
        command.target_steering_angle_deg = 0.0

        command.brake_pedal = max(
            0,
            min(
                100,
                int(
                    self.get_parameter(
                        'stop_brake_pedal'
                    ).value
                ),
            ),
        )
        command.parking_brake = max(
            0,
            min(
                1,
                int(
                    self.get_parameter(
                        'stop_parking_brake'
                    ).value
                ),
            ),
        )

        command.control_mode = 1
        command.brake_light = True
        command.emergency_stop = False

        return command

    def publish_mode(self) -> None:
        message = ControlMode()
        message.mode = int(self.mode)
        message.source = self.mode_source
        self.mode_pub.publish(message)

    @staticmethod
    def mode_name(mode: int) -> str:
        return {
            STOP: 'STOP',
            MANUAL: 'MANUAL',
            AUTO: 'AUTO',
        }.get(mode, f'UNKNOWN({mode})')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PatrolCommandManager()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            # 退出前连续发送停车命令。
            for _ in range(5):
                node.output_pub.publish(
                    node.make_stop_command()
                )
                time.sleep(0.05)

        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
