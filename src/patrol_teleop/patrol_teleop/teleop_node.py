#!/usr/bin/env python3

import os
import select
import sys
import termios
import time
import tty
from typing import Optional

import rclpy
from patrol_interfaces.srv import SetControlMode
from rclpy.node import Node
from vehicle_can_msg.msg import VehicleCommand


STOP = 0
MANUAL = 1


class PatrolTeleop(Node):

    def __init__(self) -> None:
        super().__init__('patrol_teleop')

        self.declare_parameter(
            'command_topic',
            '/patrol/manual_command',
        )
        self.declare_parameter(
            'set_mode_service',
            '/patrol/set_control_mode',
        )
        self.declare_parameter('speed_rpm', 40.0)
        self.declare_parameter('steering_request', 250.0)
        self.declare_parameter('publish_rate_hz', 100.0)
        self.declare_parameter('key_timeout_sec', 0.10)

        command_topic = str(
            self.get_parameter('command_topic').value
        )
        service_name = str(
            self.get_parameter('set_mode_service').value
        )

        self.command_pub = self.create_publisher(
            VehicleCommand,
            command_topic,
            20,
        )

        self.mode_client = self.create_client(
            SetControlMode,
            service_name,
        )

        self.current_key: Optional[str] = None
        self.last_key_time = 0.0
        self.exit_requested = False

        rate = float(
            self.get_parameter('publish_rate_hz').value
        )
        if rate <= 0.0:
            rate = 100.0

        self.create_timer(
            1.0 / rate,
            self.timer_callback,
        )

        self.get_logger().info(
            f'patrol_teleop started; output={command_topic}'
        )

    def request_mode(self, mode: int):
        if not self.mode_client.wait_for_service(
            timeout_sec=5.0
        ):
            raise RuntimeError(
                '/patrol/set_control_mode 不可用，'
                '请先启动 patrol_command_manager'
            )

        request = SetControlMode.Request()
        request.mode = int(mode)

        future = self.mode_client.call_async(request)

        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=2.0,
        )

        if not future.done():
            return None

        return future.result()

    def activate_manual_mode(self) -> None:
        response = self.request_mode(MANUAL)

        if response is None or not response.success:
            message = (
                '没有响应'
                if response is None
                else response.message
            )
            raise RuntimeError(
                f'进入 MANUAL 模式失败：{message}'
            )

        self.get_logger().warning(response.message)

    def read_key(self) -> Optional[str]:
        """清空SSH终端中已积压的字符，只采用最新按键。"""
        fd = sys.stdin.fileno()
        latest_key = None

        while True:
            readable, _, _ = select.select(
                [fd],
                [],
                [],
                0.0,
            )

            if not readable:
                break

            data = os.read(fd, 4096)
            if not data:
                break

            for character in data.decode(
                errors='ignore'
            ).lower():
                if character in (
                    '\x03', '.',
                    'q', 'w', 'e',
                    'a', 's', 'd',
                    'z', 'x', 'c',
                    ' ',
                ):
                    latest_key = character

        return latest_key

    def timer_callback(self) -> None:
        key = self.read_key()

        if key is not None:
            if key in ('\x03', '.'):
                self.current_key = None
                self.exit_requested = True

            elif key in ('s', ' '):
                self.current_key = None

            elif key in (
                'q', 'w', 'e',
                'a', 'd',
                'z', 'x', 'c',
            ):
                self.current_key = key
                self.last_key_time = time.monotonic()

        timeout = float(
            self.get_parameter('key_timeout_sec').value
        )

        if (
            self.current_key is not None
            and time.monotonic() - self.last_key_time > timeout
        ):
            self.current_key = None

        self.command_pub.publish(
            self.make_command(self.current_key)
        )

    def make_command(
        self,
        key: Optional[str],
    ) -> VehicleCommand:
        speed = abs(float(
            self.get_parameter('speed_rpm').value
        ))
        steering = abs(float(
            self.get_parameter(
                'steering_request'
            ).value
        ))

        target_speed = 0.0
        target_steering = 0.0

        if key == 'w':
            target_speed = speed
        elif key == 'x':
            target_speed = -speed
        elif key == 'a':
            target_steering = steering
        elif key == 'd':
            target_steering = -steering
        elif key == 'q':
            target_speed = speed
            target_steering = steering
        elif key == 'e':
            target_speed = speed
            target_steering = -steering
        elif key == 'z':
            target_speed = -speed
            target_steering = steering
        elif key == 'c':
            target_speed = -speed
            target_steering = -steering

        command = VehicleCommand()
        command.header.stamp = (
            self.get_clock().now().to_msg()
        )

        command.target_speed_rpm = target_speed
        command.target_steering_angle_deg = (
            target_steering
        )
        command.brake_pedal = 0
        command.parking_brake = 1
        command.control_mode = 1

        command.left_turn_light = (
            target_steering > 1.0
        )
        command.right_turn_light = (
            target_steering < -1.0
        )
        command.brake_light = (
            abs(target_speed) < 0.1
        )

        return command

    def stop_before_exit(self) -> None:
        stop = self.make_command(None)

        for _ in range(5):
            stop.header.stamp = (
                self.get_clock().now().to_msg()
            )
            self.command_pub.publish(stop)
            time.sleep(0.05)

        response = self.request_mode(STOP)
        if response is not None:
            self.get_logger().warning(
                response.message
            )

    @staticmethod
    def print_help() -> None:
        print()
        print('========== 巡检小车键盘控制 ==========')
        print(' Q    W    E     左前 / 前进 / 右前')
        print(' A    S    D     左转 / 停车 / 右转')
        print(' Z    X    C     左后 / 后退 / 右后')
        print()
        print('空格：停车')
        print('. 或 Ctrl+C：停车并退出')
        print('======================================')
        print()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PatrolTeleop()
    terminal_settings = None

    try:
        if not sys.stdin.isatty():
            raise RuntimeError(
                '必须在交互式 SSH 终端中运行'
            )

        node.activate_manual_mode()
        node.print_help()

        terminal_settings = termios.tcgetattr(
            sys.stdin.fileno()
        )
        tty.setcbreak(sys.stdin.fileno())

        while (
            rclpy.ok()
            and not node.exit_requested
        ):
            rclpy.spin_once(
                node,
                timeout_sec=0.005,
            )

    except KeyboardInterrupt:
        pass
    except Exception as exc:
        node.get_logger().error(str(exc))
    finally:
        if terminal_settings is not None:
            termios.tcsetattr(
                sys.stdin.fileno(),
                termios.TCSADRAIN,
                terminal_settings,
            )

        if rclpy.ok():
            node.stop_before_exit()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
