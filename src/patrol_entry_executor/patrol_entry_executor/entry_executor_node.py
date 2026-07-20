#!/usr/bin/env python3

import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from patrol_interfaces.msg import EntryPath, LocalizationStatus, TaskStatus
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


class PatrolEntryExecutor(Node):

    def __init__(self) -> None:
        super().__init__('patrol_entry_executor')

        self.declare_parameter(
            'entry_path_topic',
            '/patrol/entry_path',
        )
        self.declare_parameter('pose_topic', '/patrol/pose')
        self.declare_parameter(
            'localization_status_topic',
            '/patrol/localization_status',
        )
        self.declare_parameter(
            'command_topic',
            '/patrol/entry_command',
        )
        self.declare_parameter(
            'status_topic',
            '/patrol/entry_executor/status',
        )

        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('forward_speed_rpm', 20.0)
        self.declare_parameter('reverse_speed_rpm', 15.0)

        self.declare_parameter('wheelbase', 0.67)
        self.declare_parameter(
            'lookahead_distance',
            0.40,
        )
        self.declare_parameter(
            'maximum_tracking_error',
            0.80,
        )

        self.declare_parameter(
            'maximum_mechanical_steering_deg',
            30.0,
        )
        self.declare_parameter(
            'maximum_steering_request',
            400.0,
        )

        self.declare_parameter(
            'waypoint_tolerance',
            0.08,
        )
        self.declare_parameter(
            'maximum_pass_lateral_error',
            0.20,
        )
        self.declare_parameter(
            'maximum_start_error',
            0.60,
        )
        self.declare_parameter(
            'gear_change_pause_sec',
            0.80,
        )
        self.declare_parameter(
            'maximum_localization_age_sec',
            0.50,
        )

        self.latest_pose: Optional[PoseStamped] = None
        self.latest_status: Optional[LocalizationStatus] = None
        self.entry_path: Optional[EntryPath] = None

        self.pose_receive_time = 0.0
        self.status_receive_time = 0.0

        self.enabled = False
        self.target_index = 1
        self.current_direction = 0
        self.pending_direction = 0
        self.gear_pause_until = 0.0
        self.last_log_time = 0.0

        path_qos = QoSProfile(depth=1)
        path_qos.reliability = ReliabilityPolicy.RELIABLE
        path_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(
            EntryPath,
            str(
                self.get_parameter(
                    'entry_path_topic'
                ).value
            ),
            self.path_callback,
            path_qos,
        )

        self.create_subscription(
            PoseStamped,
            str(self.get_parameter('pose_topic').value),
            self.pose_callback,
            20,
        )

        self.create_subscription(
            LocalizationStatus,
            str(
                self.get_parameter(
                    'localization_status_topic'
                ).value
            ),
            self.status_callback,
            20,
        )

        self.command_pub = self.create_publisher(
            VehicleCommand,
            str(
                self.get_parameter(
                    'command_topic'
                ).value
            ),
            20,
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

        self.create_service(
            SetBool,
            '/patrol/entry_executor/enable',
            self.enable_callback,
        )

        control_rate = float(
            self.get_parameter('control_rate_hz').value
        )

        if control_rate <= 0.0:
            control_rate = 20.0

        self.create_timer(
            1.0 / control_rate,
            self.control_timer,
        )

        self.publish_status(
            TaskStatus.IDLE,
            'entry executor ready',
            0.0,
        )

        self.get_logger().info(
            'entry executor ready; output topic='
            + str(
                self.get_parameter(
                    'command_topic'
                ).value
            )
        )

    def path_callback(self, msg: EntryPath) -> None:
        if not self.path_is_valid(msg):
            self.get_logger().error(
                'received invalid entry path'
            )
            return

        if (
            self.entry_path is not None
            and self.paths_are_equivalent(
                self.entry_path,
                msg,
            )
        ):
            self.get_logger().debug(
                'duplicate entry path ignored'
            )
            return

        if self.enabled:
            self.safety_stop(
                'entry path changed while executing'
            )
            return

        self.entry_path = msg

        self.get_logger().info(
            f'entry path received: '
            f'points={len(msg.poses)}, '
            f'length={msg.total_length:.2f}m'
        )

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
            self.disable_execution()
            response.success = True
            response.message = 'entry executor disabled'
            self.get_logger().warning(response.message)
            return response

        problem = self.localization_problem()

        if problem is not None:
            response.success = False
            response.message = (
                f'cannot enable entry executor: {problem}'
            )
            return response

        if (
            self.entry_path is None
            or not self.path_is_valid(self.entry_path)
        ):
            response.success = False
            response.message = 'no valid entry path'
            return response

        assert self.latest_pose is not None

        start_pose = self.entry_path.poses[0]

        start_error = math.hypot(
            self.latest_pose.pose.position.x
            - start_pose.position.x,
            self.latest_pose.pose.position.y
            - start_pose.position.y,
        )

        maximum_start_error = float(
            self.get_parameter(
                'maximum_start_error'
            ).value
        )

        if start_error > maximum_start_error:
            response.success = False
            response.message = (
                f'start error too large: '
                f'{start_error:.2f}m > '
                f'{maximum_start_error:.2f}m'
            )
            return response

        self.target_index = 1
        self.current_direction = 0
        self.pending_direction = 0
        self.gear_pause_until = 0.0
        self.enabled = True

        response.success = True
        response.message = (
            f'entry executor enabled: '
            f'points={len(self.entry_path.poses)}, '
            f'start_error={start_error:.2f}m'
        )

        self.publish_status(
            TaskStatus.RUNNING,
            response.message,
            0.0,
        )

        self.get_logger().warning(response.message)
        return response

    def control_timer(self) -> None:
        if not self.enabled:
            self.publish_stop()
            return

        problem = self.localization_problem()

        if problem is not None:
            self.safety_stop(problem)
            return

        assert self.entry_path is not None
        assert self.latest_pose is not None

        tolerance = max(
            0.02,
            float(
                self.get_parameter(
                    'waypoint_tolerance'
                ).value
            ),
        )

        maximum_pass_lateral_error = max(
            0.05,
            float(
                self.get_parameter(
                    'maximum_pass_lateral_error'
                ).value
            ),
        )

        while self.target_index < len(
            self.entry_path.poses
        ):
            previous = self.entry_path.poses[
                self.target_index - 1
            ]
            target = self.entry_path.poses[
                self.target_index
            ]

            current_x = float(
                self.latest_pose.pose.position.x
            )
            current_y = float(
                self.latest_pose.pose.position.y
            )

            distance = math.hypot(
                target.position.x - current_x,
                target.position.y - current_y,
            )

            reached = distance <= tolerance

            segment_x = (
                target.position.x
                - previous.position.x
            )
            segment_y = (
                target.position.y
                - previous.position.y
            )
            segment_length_squared = (
                segment_x * segment_x
                + segment_y * segment_y
            )

            if (
                not reached
                and segment_length_squared > 1.0e-9
            ):
                relative_x = (
                    current_x - previous.position.x
                )
                relative_y = (
                    current_y - previous.position.y
                )

                progress = (
                    relative_x * segment_x
                    + relative_y * segment_y
                ) / segment_length_squared

                lateral_error = abs(
                    relative_x * segment_y
                    - relative_y * segment_x
                ) / math.sqrt(segment_length_squared)

                reached = (
                    progress >= 1.0
                    and lateral_error
                    <= maximum_pass_lateral_error
                )

            if not reached:
                break

            self.target_index += 1

        if self.target_index >= len(
            self.entry_path.poses
        ):
            self.complete_execution()
            return

        direction = int(
            self.entry_path.directions[
                self.target_index
            ]
        )

        if direction not in (-1, 1):
            self.safety_stop(
                f'invalid direction at point '
                f'{self.target_index}: {direction}'
            )
            return

        now = time.monotonic()

        if self.current_direction == 0:
            self.current_direction = direction

        elif direction != self.current_direction:
            if self.pending_direction != direction:
                self.pending_direction = direction
                self.gear_pause_until = (
                    now
                    + max(
                        0.0,
                        float(
                            self.get_parameter(
                                'gear_change_pause_sec'
                            ).value
                        ),
                    )
                )

                self.publish_stop()

                self.get_logger().warning(
                    f'gear change: '
                    f'{self.current_direction:+d} '
                    f'-> {direction:+d}; stopping'
                )
                return

            if now < self.gear_pause_until:
                self.publish_stop()
                return

            self.current_direction = direction
            self.pending_direction = 0

            self.get_logger().warning(
                f'gear change complete: '
                f'direction={direction:+d}'
            )

        current_x = float(
            self.latest_pose.pose.position.x
        )
        current_y = float(
            self.latest_pose.pose.position.y
        )

        current_target = self.entry_path.poses[
            self.target_index
        ]

        current_target_distance = math.hypot(
            current_target.position.x - current_x,
            current_target.position.y - current_y,
        )

        maximum_tracking_error = max(
            0.20,
            float(
                self.get_parameter(
                    'maximum_tracking_error'
                ).value
            ),
        )

        if current_target_distance > maximum_tracking_error:
            self.safety_stop(
                f'entry tracking error too large: '
                f'{current_target_distance:.2f}m > '
                f'{maximum_tracking_error:.2f}m'
            )
            return

        lookahead_distance = max(
            0.15,
            float(
                self.get_parameter(
                    'lookahead_distance'
                ).value
            ),
        )

        lookahead_index = self.target_index
        lookahead_target = current_target

        while (
            lookahead_index + 1
            < len(self.entry_path.poses)
        ):
            next_index = lookahead_index + 1
            next_direction = int(
                self.entry_path.directions[next_index]
            )

            if next_direction != direction:
                break

            candidate = self.entry_path.poses[
                next_index
            ]

            candidate_distance = math.hypot(
                candidate.position.x - current_x,
                candidate.position.y - current_y,
            )

            lookahead_index = next_index
            lookahead_target = candidate

            if candidate_distance >= lookahead_distance:
                break

        dx = lookahead_target.position.x - current_x
        dy = lookahead_target.position.y - current_y

        target_distance = max(
            0.05,
            math.hypot(dx, dy),
        )

        target_heading = math.atan2(dy, dx)

        vehicle_yaw = self.quaternion_to_yaw(
            self.latest_pose.pose.orientation
        )

        if direction > 0:
            travel_heading = vehicle_yaw
        else:
            travel_heading = normalize_angle(
                vehicle_yaw + math.pi
            )

        heading_error = normalize_angle(
            target_heading - travel_heading
        )

        wheelbase = max(
            0.01,
            float(
                self.get_parameter('wheelbase').value
            ),
        )

        steering_angle = (
            direction
            * math.atan2(
                2.0
                * wheelbase
                * math.sin(heading_error),
                target_distance,
            )
        )

        maximum_mechanical = math.radians(abs(float(
            self.get_parameter(
                'maximum_mechanical_steering_deg'
            ).value
        )))

        steering_angle = max(
            -maximum_mechanical,
            min(maximum_mechanical, steering_angle),
        )

        maximum_request = abs(float(
            self.get_parameter(
                'maximum_steering_request'
            ).value
        ))

        if maximum_mechanical > 1.0e-6:
            steering_request = (
                steering_angle
                / maximum_mechanical
                * maximum_request
            )
        else:
            steering_request = 0.0

        if direction > 0:
            speed_rpm = abs(float(
                self.get_parameter(
                    'forward_speed_rpm'
                ).value
            ))
        else:
            speed_rpm = -abs(float(
                self.get_parameter(
                    'reverse_speed_rpm'
                ).value
            ))

        self.publish_motion(
            speed_rpm,
            steering_request,
        )

        if now - self.last_log_time >= 1.0:
            self.last_log_time = now

            target = self.entry_path.poses[
                self.target_index
            ]

            distance = math.hypot(
                target.position.x
                - self.latest_pose.pose.position.x,
                target.position.y
                - self.latest_pose.pose.position.y,
            )

            progress = (
                self.target_index
                / max(1, len(self.entry_path.poses) - 1)
            )

            self.publish_status(
                TaskStatus.RUNNING,
                (
                    f'target={self.target_index}/'
                    f'{len(self.entry_path.poses) - 1}'
                ),
                progress,
            )

            self.get_logger().info(
                f'target={self.target_index}/'
                f'{len(self.entry_path.poses) - 1}, '
                f'distance={distance:.2f}m, '
                f'direction={direction:+d}, '
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
            time.monotonic()
            - self.pose_receive_time
        )
        status_age = (
            time.monotonic()
            - self.status_receive_time
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

    @staticmethod
    def path_is_valid(path: EntryPath) -> bool:
        count = len(path.poses)

        return (
            count >= 2
            and len(path.directions) == count
            and len(path.steering_angles_deg) == count
        )

    @staticmethod
    def paths_are_equivalent(
        first: EntryPath,
        second: EntryPath,
    ) -> bool:
        if len(first.poses) != len(second.poses):
            return False

        if list(first.directions) != list(second.directions):
            return False

        for first_pose, second_pose in zip(
            first.poses,
            second.poses,
        ):
            if (
                abs(
                    first_pose.position.x
                    - second_pose.position.x
                ) > 1.0e-5
                or abs(
                    first_pose.position.y
                    - second_pose.position.y
                ) > 1.0e-5
                or abs(
                    first_pose.orientation.z
                    - second_pose.orientation.z
                ) > 1.0e-5
                or abs(
                    first_pose.orientation.w
                    - second_pose.orientation.w
                ) > 1.0e-5
            ):
                return False

        for first_steering, second_steering in zip(
            first.steering_angles_deg,
            second.steering_angles_deg,
        ):
            if abs(
                float(first_steering)
                - float(second_steering)
            ) > 1.0e-4:
                return False

        return True

    def publish_motion(
        self,
        speed_rpm: float,
        steering_request: float,
    ) -> None:
        command = VehicleCommand()
        command.header.stamp = (
            self.get_clock().now().to_msg()
        )
        command.header.frame_id = (
            self.entry_path.header.frame_id
            if self.entry_path is not None
            else 'patrol_map'
        )

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

        self.command_pub.publish(command)

    def disable_execution(self) -> None:
        self.enabled = False
        self.current_direction = 0
        self.pending_direction = 0
        self.publish_stop()
        self.publish_status(
            TaskStatus.IDLE,
            'entry executor disabled',
            0.0,
        )

    def complete_execution(self) -> None:
        final_pose = self.entry_path.poses[-1]

        position_error = math.hypot(
            final_pose.position.x
            - self.latest_pose.pose.position.x,
            final_pose.position.y
            - self.latest_pose.pose.position.y,
        )

        final_yaw = self.quaternion_to_yaw(
            final_pose.orientation
        )
        current_yaw = self.quaternion_to_yaw(
            self.latest_pose.pose.orientation
        )

        heading_error = abs(math.degrees(
            normalize_angle(final_yaw - current_yaw)
        ))

        self.enabled = False
        self.publish_stop()

        message = (
            f'entry execution completed: '
            f'position_error={position_error:.2f}m, '
            f'heading_error={heading_error:.1f}deg'
        )

        self.publish_status(
            TaskStatus.SUCCEEDED,
            message,
            1.0,
        )

        self.get_logger().warning(message)

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
        status.header.frame_id = 'patrol_map'
        status.state = int(state)
        status.task = 'entry_execution'
        status.message = str(message)
        status.progress = float(max(
            0.0,
            min(1.0, progress),
        ))

        self.status_pub.publish(status)

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
    node = PatrolEntryExecutor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(5):
            node.publish_stop()
            time.sleep(0.03)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
