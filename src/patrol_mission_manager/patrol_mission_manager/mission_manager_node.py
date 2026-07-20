#!/usr/bin/env python3

from typing import Optional

import rclpy
from patrol_interfaces.msg import TaskStatus
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_srvs.srv import SetBool, Trigger


class PatrolMissionManager(Node):

    def __init__(self) -> None:
        super().__init__('patrol_mission_manager')

        self.declare_parameter(
            'status_topic',
            '/patrol/mission/status',
        )
        self.declare_parameter(
            'entry_status_topic',
            '/patrol/entry_executor/status',
        )
        self.declare_parameter(
            'route_status_topic',
            '/patrol/route_follower/status',
        )

        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.status_pub = self.create_publisher(
            TaskStatus,
            str(self.get_parameter('status_topic').value),
            status_qos,
        )

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

        self.plan_client = self.create_client(
            Trigger,
            '/patrol/entry_planner/plan',
        )
        self.entry_enable_client = self.create_client(
            SetBool,
            '/patrol/entry_executor/enable',
        )
        self.route_enable_client = self.create_client(
            SetBool,
            '/patrol/route_follower/enable',
        )

        self.create_service(
            Trigger,
            '/patrol/mission/start',
            self.start_callback,
        )
        self.create_service(
            Trigger,
            '/patrol/mission/stop',
            self.stop_callback,
        )

        self.phase = 'IDLE'

        self.plan_future = None
        self.entry_enable_future = None
        self.route_enable_future = None

        self.entry_status: Optional[TaskStatus] = None
        self.route_status: Optional[TaskStatus] = None

        self.create_timer(0.05, self.state_timer)

        self.publish_status(
            TaskStatus.IDLE,
            'mission manager ready',
            0.0,
        )

        self.get_logger().info('mission manager ready')

    def start_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request

        if self.phase not in (
            'IDLE',
            'SUCCEEDED',
            'FAILED',
        ):
            response.success = False
            response.message = (
                f'mission already active: {self.phase}'
            )
            return response

        if not self.plan_client.service_is_ready():
            response.success = False
            response.message = (
                'entry planner service unavailable'
            )
            return response

        self.phase = 'PLANNING'
        self.entry_status = None
        self.route_status = None

        self.plan_future = self.plan_client.call_async(
            Trigger.Request()
        )

        self.publish_status(
            TaskStatus.RUNNING,
            'planning entry path',
            0.02,
        )

        response.success = True
        response.message = 'mission start accepted'

        self.get_logger().warning(response.message)
        return response

    def stop_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request

        self.disable_components()

        self.phase = 'IDLE'
        self.plan_future = None
        self.entry_enable_future = None
        self.route_enable_future = None
        self.entry_status = None
        self.route_status = None

        self.publish_status(
            TaskStatus.IDLE,
            'mission stopped',
            0.0,
        )

        response.success = True
        response.message = 'mission stopped'

        self.get_logger().warning(response.message)
        return response

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

    def state_timer(self) -> None:
        if self.phase == 'PLANNING':
            self.process_planning()

        elif self.phase == 'STARTING_ENTRY':
            self.process_entry_start()

        elif self.phase == 'ENTRY':
            self.process_entry_execution()

        elif self.phase == 'STARTING_ROUTE':
            self.process_route_start()

        elif self.phase == 'ROUTE':
            self.process_route_following()

    def process_planning(self) -> None:
        if (
            self.plan_future is None
            or not self.plan_future.done()
        ):
            return

        try:
            result = self.plan_future.result()
        except Exception as exc:
            self.fail_mission(
                f'entry planning exception: {exc}'
            )
            return

        self.plan_future = None

        if result is None or not result.success:
            message = (
                result.message
                if result is not None
                else 'no planner response'
            )
            self.fail_mission(
                f'entry planning failed: {message}'
            )
            return

        if not self.entry_enable_client.service_is_ready():
            self.fail_mission(
                'entry executor service unavailable'
            )
            return

        request = SetBool.Request()
        request.data = True

        self.entry_status = None
        self.entry_enable_future = (
            self.entry_enable_client.call_async(request)
        )
        self.phase = 'STARTING_ENTRY'

        self.publish_status(
            TaskStatus.RUNNING,
            'entry path planned; starting entry execution',
            0.08,
        )

    def process_entry_start(self) -> None:
        if (
            self.entry_enable_future is None
            or not self.entry_enable_future.done()
        ):
            return

        try:
            result = self.entry_enable_future.result()
        except Exception as exc:
            self.fail_mission(
                f'entry executor exception: {exc}'
            )
            return

        self.entry_enable_future = None

        if result is None or not result.success:
            message = (
                result.message
                if result is not None
                else 'no entry executor response'
            )
            self.fail_mission(
                f'entry executor start failed: {message}'
            )
            return

        self.phase = 'ENTRY'

        self.publish_status(
            TaskStatus.RUNNING,
            'entry execution running',
            0.10,
        )

        self.get_logger().warning(
            'mission phase: ENTRY'
        )

    def process_entry_execution(self) -> None:
        if self.entry_status is None:
            return

        if self.entry_status.state == TaskStatus.FAILED:
            self.fail_mission(
                self.entry_status.message
            )
            return

        if self.entry_status.state == TaskStatus.SUCCEEDED:
            if not self.route_enable_client.service_is_ready():
                self.fail_mission(
                    'route follower service unavailable'
                )
                return

            request = SetBool.Request()
            request.data = True

            self.route_status = None
            self.route_enable_future = (
                self.route_enable_client.call_async(
                    request
                )
            )
            self.phase = 'STARTING_ROUTE'

            self.publish_status(
                TaskStatus.RUNNING,
                'entry completed; starting route following',
                0.50,
            )
            return

        progress = 0.10 + 0.40 * float(
            self.entry_status.progress
        )

        self.publish_status(
            TaskStatus.RUNNING,
            self.entry_status.message,
            progress,
        )

    def process_route_start(self) -> None:
        if (
            self.route_enable_future is None
            or not self.route_enable_future.done()
        ):
            return

        try:
            result = self.route_enable_future.result()
        except Exception as exc:
            self.fail_mission(
                f'route follower exception: {exc}'
            )
            return

        self.route_enable_future = None

        if result is None or not result.success:
            message = (
                result.message
                if result is not None
                else 'no route follower response'
            )
            self.fail_mission(
                f'route follower start failed: {message}'
            )
            return

        self.phase = 'ROUTE'

        self.publish_status(
            TaskStatus.RUNNING,
            'route following running',
            0.50,
        )

        self.get_logger().warning(
            'mission phase: ROUTE'
        )

    def process_route_following(self) -> None:
        if self.route_status is None:
            return

        if self.route_status.state == TaskStatus.FAILED:
            self.fail_mission(
                self.route_status.message
            )
            return

        if self.route_status.state == TaskStatus.SUCCEEDED:
            self.phase = 'SUCCEEDED'

            self.publish_status(
                TaskStatus.SUCCEEDED,
                'patrol mission completed',
                1.0,
            )

            self.get_logger().warning(
                'patrol mission completed'
            )
            return

        progress = 0.50 + 0.50 * float(
            self.route_status.progress
        )

        self.publish_status(
            TaskStatus.RUNNING,
            self.route_status.message,
            progress,
        )

    def fail_mission(self, reason: str) -> None:
        self.disable_components()
        self.phase = 'FAILED'

        message = f'mission failed: {reason}'

        self.publish_status(
            TaskStatus.FAILED,
            message,
            0.0,
        )

        self.get_logger().error(message)

    def disable_components(self) -> None:
        request = SetBool.Request()
        request.data = False

        if self.entry_enable_client.service_is_ready():
            self.entry_enable_client.call_async(request)

        request = SetBool.Request()
        request.data = False

        if self.route_enable_client.service_is_ready():
            self.route_enable_client.call_async(request)

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
        status.task = 'patrol_mission'
        status.message = str(message)
        status.progress = float(max(
            0.0,
            min(1.0, progress),
        ))

        self.status_pub.publish(status)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PatrolMissionManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.disable_components()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
