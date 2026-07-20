#!/usr/bin/env python3

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
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('key_timeout_sec', 0.30)

        command_topic = str(
            self.get_parameter('command_topic').value
        )
        service_name = str(
            self.get_parameter('set_mode_service').value
        )

        self.publisher = self.create_publisher(
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

        publish_rate = float(
            self.get_parameter('publish_rate_hz').value
        )
        if publish_rate <= 0.0:
            publish_rate = 20.0

        self.create_timer(
            1.0 / publish_rate,
            self.timer_callback,
        )

        self.get_logger().info(
            'patrol_teleop started; '
            f'output={command_topic}'
        )

        self.print_help()

    def activate_manual_mode(self) -> None:
        if not self.mode_client.wait_for_service(
            timeout_sec=5.0
        ):
            raise RuntimeError(
                '/patrol/set_control_mode is unavailable; '
                'start patrol_command_manager first'
            )

        response = self.request_mode(
            MANUAL,
            wait=True,
        )

        if response is None or not response.success:
            message = (
                'no response'
                if response is None
                else response.message
            )
            raise RuntimeError(
                f'failed to enter MANUAL mode: {message}'
            )

        self.get_logger().warning(response.message)

    def request_mode(
        self,
        mode: int,
        wait: bool = False,
    ):
        if not self.mode_client.service_is_ready():
            if not self.mode_client.wait_for_service(
                timeout_sec=1.0
            ):
                self.get_logger().error(
                    'control mode service unavailable'
                )
                return None

        request = SetControlMode.Request()
        request.mode = int(mode)

        future = self.mode_client.call_async(request)

        if not wait:
            future.add_done_callback(
                self.mode_response_callback
            )
            return None

        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=2.0,
        )

        if not future.done():
            return None

        try:
            return future.result()
        except Exception as exc:
            self.get_logger().error(
                f'control mode request failed: {exc}'
            )
            return None

    def mode_response_callback(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(
                f'control mode request failed: {exc}'
            )
            return

        if response.success:
            self.get_logger().warning(response.message)
        else:
            self.get_logger().error(response.message)

    def timer_callback(self) -> None:
        key = self.read_key()

        if key is not None:
            self.process_key(key)

        timeout = float(
            self.get_parameter('key_timeout_sec').value
        )

        if (
            self.current_key is not None
            and time.monotonic() - self.last_key_time > timeout
        ):
            self.current_key = None

        self.publisher.publish(
            self.make_command(self.current_key)
        )

    def read_key(self) -> Optional[str]:
        readable, _, _ = select.select(
            [sys.stdin],
            [],
            [],
            0.0,
        )

        if not readable:
            return None

        return sys.stdin.read(1).lower()

    def process_key(self, key: str) -> None:
        if key in ('\x03', '.'):
            self.current_key = None
            self.exit_requested = True
            return

        if key == 'm':
            self.current_key = None
            self.request_mode(MANUAL)
            return

        if key == 'p':
            self.current_key = None
            self.request_mode(STOP)
            return

        if key in ('s', ' '):
            self.current_key = None
            return

        if key in (
            'q', 'w', 'e',
            'a', 'd',
            'z', 'x', 'c',
        ):
            self.current_key = key
            self.last_key_time = time.monotonic()

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

        message = VehicleCommand()
        message.header.stamp = self.get_clock().now().to_msg()

        message.target_speed_rpm = float(target_speed)
        message.target_steering_angle_deg = float(
            target_steering
        )

        message.brake_pedal = 0
        message.parking_brake = 1
        message.control_mode = 1

        message.left_turn_light = target_steering > 1.0
        message.right_turn_light = target_steering < -1.0
        message.brake_light = abs(target_speed) < 0.1

        message.emergency_stop = False
        message.headlamp = False
        message.left_net_catch = False
        message.right_net_catch = False
        message.flash_light = False
        message.net_launch_enable = False

        return message

    def stop_before_exit(self) -> None:
        stop_command = self.make_command(None)

        for _ in range(5):
            stop_command.header.stamp = (
                self.get_clock().now().to_msg()
            )
            self.publisher.publish(stop_command)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.03)

        response = self.request_mode(
            STOP,
            wait=True,
        )

        if response is not None:
            self.get_logger().warning(response.message)

    @staticmethod
    def print_help() -> None:
        print()
        print('========== 巡检小车键盘控制 ==========')
        print(' Q    W    E     左前 / 前进 / 右前')
        print(' A    S    D     左转轮 / 停车 / 右转轮')
        print(' Z    X    C     左后 / 后退 / 右后')
        print()
        print(' M：切换到 MANUAL 模式')
        print(' P：切换到 STOP 模式')
        print(' 空格：停车')
        print(' . 或 Ctrl+C：停车并退出')
        print('======================================')
        print()


def main(args=None) -> None:
    rclpy.init(args=args)

    node = PatrolTeleop()
    terminal_settings = None

    try:
        if not sys.stdin.isatty():
            raise RuntimeError(
                'patrol_teleop must run in an interactive terminal'
            )

        node.activate_manual_mode()

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
                timeout_sec=0.05,
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
