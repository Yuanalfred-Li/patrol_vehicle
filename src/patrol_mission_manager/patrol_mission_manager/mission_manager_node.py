#!/usr/bin/env python3

from pathlib import Path
from typing import Optional

import rclpy
from patrol_interfaces.msg import TaskStatus
from patrol_interfaces.srv import LoadRoute
from rclpy.node import Node
from rclpy.parameter import Parameter
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

        self.declare_parameter(
            'start_service',
            '/patrol/mission/start',
        )
        self.declare_parameter(
            'stop_service',
            '/patrol/mission/stop',
        )
        self.declare_parameter(
            'load_route_service',
            '/patrol/mission/load_route',
        )

        self.declare_parameter(
            'localization_load_route_service',
            '/patrol/localization/load_route',
        )
        self.declare_parameter(
            'entry_planner_load_route_service',
            '/patrol/entry_planner/load_route',
        )
        self.declare_parameter(
            'route_follower_load_route_service',
            '/patrol/route_follower/load_route',
        )
        self.declare_parameter(
            'current_route_file',
            '',
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

        self.localization_load_client = self.create_client(
            LoadRoute,
            str(
                self.get_parameter(
                    'localization_load_route_service'
                ).value
            ),
        )
        self.entry_planner_load_client = self.create_client(
            LoadRoute,
            str(
                self.get_parameter(
                    'entry_planner_load_route_service'
                ).value
            ),
        )
        self.route_follower_load_client = self.create_client(
            LoadRoute,
            str(
                self.get_parameter(
                    'route_follower_load_route_service'
                ).value
            ),
        )

        self.create_service(
            Trigger,
            str(
                self.get_parameter(
                    'start_service'
                ).value
            ),
            self.start_callback,
        )
        self.create_service(
            Trigger,
            str(
                self.get_parameter(
                    'stop_service'
                ).value
            ),
            self.stop_callback,
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

        self.phase = 'IDLE'

        self.plan_future = None
        self.entry_enable_future = None
        self.route_enable_future = None

        self.load_route_future = None
        self.rollback_futures = []
        self.pending_route_file = ''
        self.previous_route_file = ''
        self.route_load_error = ''
        self.current_route_file = str(
            self.get_parameter(
                'current_route_file'
            ).value
        )

        self.entry_status: Optional[TaskStatus] = None
        self.route_status: Optional[TaskStatus] = None

        self.create_timer(0.05, self.state_timer)

        self.publish_status(
            TaskStatus.IDLE,
            'mission manager ready',
            0.0,
        )

        self.get_logger().info('mission manager ready')

    def route_load_clients(self):
        return [
            (
                'route follower',
                self.route_follower_load_client,
            ),
            (
                'entry planner',
                self.entry_planner_load_client,
            ),
            (
                'localization',
                self.localization_load_client,
            ),
        ]

    @staticmethod
    def make_load_route_request(
        route_file: str,
    ) -> LoadRoute.Request:
        request = LoadRoute.Request()
        request.route_file = str(route_file)
        return request

    def load_route_callback(
        self,
        request: LoadRoute.Request,
        response: LoadRoute.Response,
    ) -> LoadRoute.Response:
        if self.phase not in (
            'IDLE',
            'SUCCEEDED',
            'FAILED',
        ):
            response.success = False
            response.message = (
                f'cannot load route while mission state '
                f'is {self.phase}'
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

        if not route_path.is_file():
            response.success = False
            response.message = (
                f'route file not found: {route_path}'
            )
            return response

        unavailable = [
            name
            for name, client in self.route_load_clients()
            if not client.service_is_ready()
        ]

        if unavailable:
            response.success = False
            response.message = (
                'route load services unavailable: '
                + ', '.join(unavailable)
            )
            return response

        self.disable_components()

        self.pending_route_file = str(route_path)
        self.previous_route_file = (
            self.current_route_file
        )
        self.route_load_error = ''
        self.rollback_futures = []

        self.load_route_future = (
            self.route_follower_load_client.call_async(
                self.make_load_route_request(
                    self.pending_route_file
                )
            )
        )

        self.phase = 'LOADING_ROUTE_FOLLOWER'

        self.publish_status(
            TaskStatus.RUNNING,
            (
                f'loading route follower: '
                f'{route_path.name}'
            ),
            0.10,
        )

        response.success = True
        response.message = (
            f'route load accepted: {route_path}'
        )

        self.get_logger().warning(response.message)
        return response

    def process_route_load(self) -> None:
        if (
            self.load_route_future is None
            or not self.load_route_future.done()
        ):
            return

        current_phase = self.phase

        try:
            result = self.load_route_future.result()
        except Exception as exc:
            self.begin_route_rollback(
                f'{current_phase} exception: {exc}'
            )
            return

        self.load_route_future = None

        if result is None or not result.success:
            message = (
                result.message
                if result is not None
                else 'no route load response'
            )

            self.begin_route_rollback(
                f'{current_phase} failed: {message}'
            )
            return

        if current_phase == 'LOADING_ROUTE_FOLLOWER':
            self.load_route_future = (
                self.entry_planner_load_client.call_async(
                    self.make_load_route_request(
                        self.pending_route_file
                    )
                )
            )
            self.phase = 'LOADING_ENTRY_PLANNER'

            self.publish_status(
                TaskStatus.RUNNING,
                'route follower loaded; loading entry planner',
                0.40,
            )
            return

        if current_phase == 'LOADING_ENTRY_PLANNER':
            self.load_route_future = (
                self.localization_load_client.call_async(
                    self.make_load_route_request(
                        self.pending_route_file
                    )
                )
            )
            self.phase = 'LOADING_LOCALIZATION'

            self.publish_status(
                TaskStatus.RUNNING,
                'entry planner loaded; loading localization',
                0.70,
            )
            return

        parameter_result = self.set_parameters_atomically([
            Parameter(
                'current_route_file',
                Parameter.Type.STRING,
                self.pending_route_file,
            ),
        ])

        if not parameter_result.successful:
            self.begin_route_rollback(
                'failed to update current_route_file: '
                f'{parameter_result.reason}'
            )
            return

        self.current_route_file = self.pending_route_file
        loaded_name = Path(
            self.current_route_file
        ).name

        self.pending_route_file = ''
        self.previous_route_file = ''
        self.route_load_error = ''
        self.phase = 'IDLE'

        self.publish_status(
            TaskStatus.IDLE,
            f'route loaded and ready: {loaded_name}',
            0.0,
        )

        self.get_logger().warning(
            f'route load completed: '
            f'{self.current_route_file}'
        )

    def begin_route_rollback(
        self,
        reason: str,
    ) -> None:
        self.disable_components()
        self.load_route_future = None
        self.route_load_error = str(reason)
        self.rollback_futures = []

        if (
            self.previous_route_file
            and self.previous_route_file
            != self.pending_route_file
        ):
            for name, client in self.route_load_clients():
                if not client.service_is_ready():
                    continue

                future = client.call_async(
                    self.make_load_route_request(
                        self.previous_route_file
                    )
                )
                self.rollback_futures.append(
                    (name, future)
                )

        if not self.rollback_futures:
            self.finish_route_load_failure(
                'rollback not available'
            )
            return

        self.phase = 'ROLLING_BACK_ROUTE'

        self.publish_status(
            TaskStatus.RUNNING,
            (
                f'route load failed; restoring previous '
                f'route: {self.route_load_error}'
            ),
            0.0,
        )

    def process_route_rollback(self) -> None:
        if any(
            not future.done()
            for _, future in self.rollback_futures
        ):
            return

        failures = []

        for name, future in self.rollback_futures:
            try:
                result = future.result()
            except Exception as exc:
                failures.append(
                    f'{name}: {exc}'
                )
                continue

            if result is None or not result.success:
                message = (
                    result.message
                    if result is not None
                    else 'no response'
                )
                failures.append(
                    f'{name}: {message}'
                )

        detail = (
            'rollback completed'
            if not failures
            else 'rollback issues: ' + '; '.join(failures)
        )

        self.finish_route_load_failure(detail)

    def finish_route_load_failure(
        self,
        rollback_detail: str,
    ) -> None:
        message = (
            f'route load failed: {self.route_load_error}; '
            f'{rollback_detail}'
        )

        self.phase = 'FAILED'
        self.load_route_future = None
        self.rollback_futures = []
        self.pending_route_file = ''
        self.previous_route_file = ''

        self.publish_status(
            TaskStatus.FAILED,
            message,
            0.0,
        )

        self.get_logger().error(message)

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

        if self.phase in (
            'LOADING_ROUTE_FOLLOWER',
            'LOADING_ENTRY_PLANNER',
            'LOADING_LOCALIZATION',
            'ROLLING_BACK_ROUTE',
        ):
            response.success = True
            response.message = (
                'motion stopped; route operation still '
                'in progress'
            )
            return response

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

        elif self.phase in (
            'LOADING_ROUTE_FOLLOWER',
            'LOADING_ENTRY_PLANNER',
            'LOADING_LOCALIZATION',
        ):
            self.process_route_load()

        elif self.phase == 'ROLLING_BACK_ROUTE':
            self.process_route_rollback()

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
        if rclpy.ok():
            node.disable_components()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
