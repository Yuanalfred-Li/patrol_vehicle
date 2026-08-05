#!/usr/bin/env python3

import heapq
import itertools
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
import yaml
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path as NavPath
from patrol_interfaces.msg import EntryPath, LocalizationStatus
from patrol_interfaces.srv import LoadRoute
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_srvs.srv import Trigger


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


def geodetic_to_ecef(
    latitude_deg: float,
    longitude_deg: float,
    altitude: float,
) -> Tuple[float, float, float]:
    latitude = math.radians(latitude_deg)
    longitude = math.radians(longitude_deg)

    sin_latitude = math.sin(latitude)
    cos_latitude = math.cos(latitude)
    sin_longitude = math.sin(longitude)
    cos_longitude = math.cos(longitude)

    radius = WGS84_A / math.sqrt(
        1.0 - WGS84_E2 * sin_latitude * sin_latitude
    )

    x = (
        radius + altitude
    ) * cos_latitude * cos_longitude

    y = (
        radius + altitude
    ) * cos_latitude * sin_longitude

    z = (
        radius * (1.0 - WGS84_E2) + altitude
    ) * sin_latitude

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

    origin_x, origin_y, origin_z = geodetic_to_ecef(
        origin_latitude_deg,
        origin_longitude_deg,
        origin_altitude,
    )

    dx = x - origin_x
    dy = y - origin_y
    dz = z - origin_z

    origin_latitude = math.radians(
        origin_latitude_deg
    )
    origin_longitude = math.radians(
        origin_longitude_deg
    )

    sin_latitude = math.sin(origin_latitude)
    cos_latitude = math.cos(origin_latitude)
    sin_longitude = math.sin(origin_longitude)
    cos_longitude = math.cos(origin_longitude)

    east = (
        -sin_longitude * dx
        + cos_longitude * dy
    )

    north = (
        -sin_latitude * cos_longitude * dx
        - sin_latitude * sin_longitude * dy
        + cos_latitude * dz
    )

    up = (
        cos_latitude * cos_longitude * dx
        + cos_latitude * sin_longitude * dy
        + sin_latitude * dz
    )

    return east, north, up


def parse_route_origin(
    data: Dict,
) -> Optional[Tuple[float, float, float]]:
    origin = data.get('origin')

    if origin is None:
        return None

    if not isinstance(origin, dict):
        raise ValueError(
            'route origin must be a mapping'
        )

    try:
        latitude = float(origin['latitude'])
        longitude = float(origin['longitude'])
        altitude = float(origin['altitude'])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(
            f'invalid route origin: {exc}'
        ) from exc

    if not all(math.isfinite(value) for value in (
        latitude,
        longitude,
        altitude,
    )):
        raise ValueError(
            'route origin contains non-finite value'
        )

    if not -90.0 <= latitude <= 90.0:
        raise ValueError(
            'route origin latitude is out of range'
        )

    if not -180.0 <= longitude <= 180.0:
        raise ValueError(
            'route origin longitude is out of range'
        )

    return latitude, longitude, altitude


def route_waypoint_to_local(
    raw: Dict,
    index: int,
    origin: Optional[Tuple[float, float, float]],
) -> Tuple[float, float, float]:
    if not isinstance(raw, dict):
        raise ValueError(
            f'waypoint {index} must be a mapping'
        )

    absolute_keys = (
        'latitude',
        'longitude',
        'altitude',
    )
    absolute_count = sum(
        key in raw for key in absolute_keys
    )

    if absolute_count:
        if absolute_count != len(absolute_keys):
            raise ValueError(
                f'waypoint {index} has incomplete '
                'absolute coordinates'
            )

        if origin is None:
            raise ValueError(
                f'waypoint {index} has absolute '
                'coordinates but route origin is missing'
            )

        try:
            latitude = float(raw['latitude'])
            longitude = float(raw['longitude'])
            altitude = float(raw['altitude'])
        except (TypeError, ValueError) as exc:
            raise ValueError(
                f'invalid absolute waypoint {index}: {exc}'
            ) from exc

        if not all(math.isfinite(value) for value in (
            latitude,
            longitude,
            altitude,
        )):
            raise ValueError(
                f'waypoint {index} contains '
                'non-finite absolute coordinates'
            )

        if not -90.0 <= latitude <= 90.0:
            raise ValueError(
                f'waypoint {index} latitude '
                'is out of range'
            )

        if not -180.0 <= longitude <= 180.0:
            raise ValueError(
                f'waypoint {index} longitude '
                'is out of range'
            )

        return geodetic_to_enu(
            latitude,
            longitude,
            altitude,
            origin[0],
            origin[1],
            origin[2],
        )

    try:
        x = float(raw['x'])
        y = float(raw['y'])
        z = float(raw.get('z', 0.0))
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(
            f'invalid local waypoint {index}: {exc}'
        ) from exc

    if not all(math.isfinite(value) for value in (
        x,
        y,
        z,
    )):
        raise ValueError(
            f'waypoint {index} contains '
            'non-finite local coordinates'
        )

    return x, y, z


@dataclass
class SearchNode:
    x: float
    y: float
    yaw: float
    direction: int
    steering: float
    cost: float
    parent: Optional[Tuple[int, int, int, int]]


class PatrolEntryPlanner(Node):

    def __init__(self) -> None:
        super().__init__('patrol_entry_planner')

        self.declare_parameter(
            'route_file',
            '/home/nvidia/patrol_ws/routes/route.yaml',
        )
        self.declare_parameter(
            'load_route_service',
            '/patrol/entry_planner/load_route',
        )
        self.declare_parameter('pose_topic', '/patrol/pose')
        self.declare_parameter(
            'localization_status_topic',
            '/patrol/localization_status',
        )
        self.declare_parameter(
            'entry_path_topic',
            '/patrol/entry_path',
        )
        self.declare_parameter(
            'entry_nav_path_topic',
            '/patrol/entry_path_nav',
        )

        self.declare_parameter('wheelbase', 0.67)
        self.declare_parameter(
            'maximum_steering_angle_deg',
            30.0,
        )
        self.declare_parameter('motion_step', 0.25)
        self.declare_parameter('motion_substeps', 5)

        self.declare_parameter('xy_resolution', 0.15)
        self.declare_parameter(
            'yaw_resolution_deg',
            10.0,
        )
        self.declare_parameter('planning_margin', 4.0)
        self.declare_parameter('maximum_expansions', 50000)

        self.declare_parameter(
            'goal_position_tolerance',
            0.25,
        )
        self.declare_parameter(
            'goal_heading_tolerance_deg',
            15.0,
        )

        self.declare_parameter('reverse_cost_ratio', 1.15)
        self.declare_parameter('gear_switch_cost', 0.60)
        self.declare_parameter('steering_cost_ratio', 0.05)
        self.declare_parameter(
            'maximum_localization_age_sec',
            0.50,
        )

        self.latest_pose: Optional[PoseStamped] = None
        self.latest_status: Optional[LocalizationStatus] = None
        self.pose_receive_time = 0.0
        self.status_receive_time = 0.0

        path_qos = QoSProfile(depth=1)
        path_qos.reliability = ReliabilityPolicy.RELIABLE
        path_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.entry_path_pub = self.create_publisher(
            EntryPath,
            str(
                self.get_parameter(
                    'entry_path_topic'
                ).value
            ),
            path_qos,
        )

        self.nav_path_pub = self.create_publisher(
            NavPath,
            str(
                self.get_parameter(
                    'entry_nav_path_topic'
                ).value
            ),
            path_qos,
        )

        # 与 /patrol/pose 发布端保持一致，
        # 晚启动订阅时立即收到最近位姿，避免启动竞态。
        state_qos = QoSProfile(depth=20)
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(
            PoseStamped,
            str(self.get_parameter('pose_topic').value),
            self.pose_callback,
            state_qos,
        )

        self.create_subscription(
            LocalizationStatus,
            str(
                self.get_parameter(
                    'localization_status_topic'
                ).value
            ),
            self.status_callback,
            state_qos,
        )

        self.create_service(
            Trigger,
            '/patrol/entry_planner/plan',
            self.plan_callback,
        )

        self.create_service(
            Trigger,
            '/patrol/entry_planner/clear',
            self.clear_callback,
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

        self.get_logger().info(
            'entry planner ready; planning only, '
            'no vehicle command output'
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

    def plan_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request

        problem = self.localization_problem()

        if problem is not None:
            response.success = False
            response.message = problem
            return response

        try:
            frame_id, goal_x, goal_y, goal_yaw = (
                self.load_route_start()
            )
        except Exception as exc:
            response.success = False
            response.message = f'route load failed: {exc}'
            return response

        assert self.latest_pose is not None

        start_x = float(
            self.latest_pose.pose.position.x
        )
        start_y = float(
            self.latest_pose.pose.position.y
        )
        start_yaw = self.quaternion_to_yaw(
            self.latest_pose.pose.orientation
        )

        self.get_logger().info(
            f'planning entry path: '
            f'start=({start_x:.2f}, {start_y:.2f}, '
            f'{math.degrees(start_yaw):.1f}deg), '
            f'goal=({goal_x:.2f}, {goal_y:.2f}, '
            f'{math.degrees(goal_yaw):.1f}deg)'
        )

        start_time = time.monotonic()

        path, expansions = self.hybrid_a_star(
            start_x,
            start_y,
            start_yaw,
            goal_x,
            goal_y,
            goal_yaw,
        )

        planning_time = time.monotonic() - start_time

        if path is None:
            response.success = False
            response.message = (
                f'planning failed after {expansions} '
                f'expansions in {planning_time:.2f}s'
            )
            self.get_logger().error(response.message)
            return response

        total_length = self.path_length(path)

        final = path[-1]

        final_position_error = math.hypot(
            final.x - goal_x,
            final.y - goal_y,
        )

        final_heading_error = abs(math.degrees(
            normalize_angle(final.yaw - goal_yaw)
        ))

        self.publish_path(
            frame_id,
            path,
            total_length,
            final_position_error,
            final_heading_error,
        )

        gear_switches = 0
        previous_direction = 0

        for item in path:
            if (
                previous_direction != 0
                and item.direction != 0
                and item.direction != previous_direction
            ):
                gear_switches += 1

            if item.direction != 0:
                previous_direction = item.direction

        response.success = True
        response.message = (
            f'entry path planned: '
            f'points={len(path)}, '
            f'length={total_length:.2f}m, '
            f'gear_switches={gear_switches}, '
            f'position_error={final_position_error:.2f}m, '
            f'heading_error={final_heading_error:.1f}deg, '
            f'expansions={expansions}, '
            f'time={planning_time:.2f}s'
        )

        self.get_logger().info(response.message)
        return response

    def clear_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request

        entry = EntryPath()
        entry.header.stamp = (
            self.get_clock().now().to_msg()
        )
        entry.header.frame_id = 'patrol_map'
        self.entry_path_pub.publish(entry)

        nav_path = NavPath()
        nav_path.header = entry.header
        self.nav_path_pub.publish(nav_path)

        response.success = True
        response.message = 'entry path cleared'
        return response

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
            time.monotonic() - self.pose_receive_time
        )
        status_age = (
            time.monotonic() - self.status_receive_time
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

    def inspect_route_file(
        self,
        route_file: Path,
    ) -> Tuple[
        str,
        float,
        float,
        float,
        int,
        float,
    ]:
        if not route_file.is_file():
            raise FileNotFoundError(route_file)

        with route_file.open(
            'r',
            encoding='utf-8',
        ) as file:
            data = yaml.safe_load(file)

        if not isinstance(data, dict):
            raise ValueError(
                'route YAML root must be a mapping'
            )

        waypoints = data.get('waypoints')

        if (
            not isinstance(waypoints, list)
            or len(waypoints) < 2
        ):
            raise ValueError(
                'route requires at least two waypoints'
            )

        origin = parse_route_origin(data)
        points = []

        for index, waypoint in enumerate(waypoints):
            x, y, _ = route_waypoint_to_local(
                waypoint,
                index,
                origin,
            )
            points.append((x, y))

        total_length = 0.0

        for index in range(len(points) - 1):
            segment_length = math.hypot(
                points[index + 1][0]
                - points[index][0],
                points[index + 1][1]
                - points[index][1],
            )

            if segment_length < 1.0e-6:
                raise ValueError(
                    f'route segment {index + 1} '
                    'has zero length'
                )

            total_length += segment_length

        first_x, first_y = points[0]
        second_x, second_y = points[1]

        heading = math.atan2(
            second_y - first_y,
            second_x - first_x,
        )

        return (
            str(data.get('frame_id', 'patrol_map')),
            first_x,
            first_y,
            heading,
            len(points),
            total_length,
        )

    def load_route_start(
        self,
    ) -> Tuple[str, float, float, float]:
        route_file = Path(str(
            self.get_parameter('route_file').value
        ))

        (
            frame_id,
            first_x,
            first_y,
            heading,
            _,
            _,
        ) = self.inspect_route_file(route_file)

        return (
            frame_id,
            first_x,
            first_y,
            heading,
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

        try:
            (
                _,
                _,
                _,
                _,
                point_count,
                total_length,
            ) = self.inspect_route_file(route_path)
        except Exception as exc:
            response.success = False
            response.message = (
                f'route load failed: {exc}'
            )
            return response

        result = self.set_parameters_atomically([
            Parameter(
                'route_file',
                Parameter.Type.STRING,
                str(route_path),
            ),
        ])

        if not result.successful:
            response.success = False
            response.message = (
                'failed to update route_file parameter: '
                f'{result.reason}'
            )
            return response

        # 清除此前规划并发布的入轨路径，避免误用旧路径。
        entry = EntryPath()
        entry.header.stamp = (
            self.get_clock().now().to_msg()
        )
        entry.header.frame_id = 'patrol_map'
        self.entry_path_pub.publish(entry)

        nav_path = NavPath()
        nav_path.header = entry.header
        self.nav_path_pub.publish(nav_path)

        response.success = True
        response.message = (
            f'entry planner loaded {route_path.name}; '
            f'points={point_count}, '
            f'length={total_length:.2f}m'
        )

        self.get_logger().warning(response.message)
        return response

    def hybrid_a_star(
        self,
        start_x: float,
        start_y: float,
        start_yaw: float,
        goal_x: float,
        goal_y: float,
        goal_yaw: float,
    ) -> Tuple[
        Optional[List[SearchNode]],
        int,
    ]:
        wheelbase = float(
            self.get_parameter('wheelbase').value
        )

        maximum_steering = math.radians(abs(float(
            self.get_parameter(
                'maximum_steering_angle_deg'
            ).value
        )))

        motion_step = max(
            0.05,
            float(
                self.get_parameter(
                    'motion_step'
                ).value
            ),
        )

        motion_substeps = max(
            1,
            int(
                self.get_parameter(
                    'motion_substeps'
                ).value
            ),
        )

        xy_resolution = max(
            0.05,
            float(
                self.get_parameter(
                    'xy_resolution'
                ).value
            ),
        )

        yaw_resolution = math.radians(max(
            1.0,
            float(
                self.get_parameter(
                    'yaw_resolution_deg'
                ).value
            ),
        ))

        planning_margin = max(
            1.0,
            float(
                self.get_parameter(
                    'planning_margin'
                ).value
            ),
        )

        maximum_expansions = max(
            100,
            int(
                self.get_parameter(
                    'maximum_expansions'
                ).value
            ),
        )

        position_tolerance = max(
            0.05,
            float(
                self.get_parameter(
                    'goal_position_tolerance'
                ).value
            ),
        )

        heading_tolerance = math.radians(max(
            1.0,
            float(
                self.get_parameter(
                    'goal_heading_tolerance_deg'
                ).value
            ),
        ))

        reverse_cost_ratio = max(
            1.0,
            float(
                self.get_parameter(
                    'reverse_cost_ratio'
                ).value
            ),
        )

        gear_switch_cost = max(
            0.0,
            float(
                self.get_parameter(
                    'gear_switch_cost'
                ).value
            ),
        )

        steering_cost_ratio = max(
            0.0,
            float(
                self.get_parameter(
                    'steering_cost_ratio'
                ).value
            ),
        )

        minimum_x = (
            min(start_x, goal_x) - planning_margin
        )
        maximum_x = (
            max(start_x, goal_x) + planning_margin
        )
        minimum_y = (
            min(start_y, goal_y) - planning_margin
        )
        maximum_y = (
            max(start_y, goal_y) + planning_margin
        )

        yaw_bins = max(
            8,
            int(round(
                2.0 * math.pi / yaw_resolution
            )),
        )

        def state_key(
            x: float,
            y: float,
            yaw: float,
            direction: int,
        ) -> Tuple[int, int, int, int]:
            x_index = int(round(
                (x - minimum_x) / xy_resolution
            ))
            y_index = int(round(
                (y - minimum_y) / xy_resolution
            ))

            yaw_positive = yaw % (2.0 * math.pi)

            yaw_index = int(round(
                yaw_positive
                / (2.0 * math.pi)
                * yaw_bins
            )) % yaw_bins

            return (
                x_index,
                y_index,
                yaw_index,
                direction,
            )

        minimum_turning_radius = (
            wheelbase / math.tan(maximum_steering)
            if maximum_steering > 1.0e-6
            else wheelbase
        )

        def heuristic(
            x: float,
            y: float,
            yaw: float,
        ) -> float:
            distance = math.hypot(
                goal_x - x,
                goal_y - y,
            )

            heading_error = abs(
                normalize_angle(goal_yaw - yaw)
            )

            return (
                distance
                + 0.20
                * minimum_turning_radius
                * heading_error
            )

        start_key = state_key(
            start_x,
            start_y,
            start_yaw,
            0,
        )

        records: Dict[
            Tuple[int, int, int, int],
            SearchNode,
        ] = {
            start_key: SearchNode(
                x=start_x,
                y=start_y,
                yaw=start_yaw,
                direction=0,
                steering=0.0,
                cost=0.0,
                parent=None,
            )
        }

        best_cost = {start_key: 0.0}

        queue = []
        counter = itertools.count()

        heapq.heappush(
            queue,
            (
                heuristic(
                    start_x,
                    start_y,
                    start_yaw,
                ),
                0.0,
                next(counter),
                start_key,
            ),
        )

        steering_values = [
            -maximum_steering,
            -0.5 * maximum_steering,
            0.0,
            0.5 * maximum_steering,
            maximum_steering,
        ]

        expansions = 0
        goal_key = None

        while (
            queue
            and expansions < maximum_expansions
        ):
            (
                estimated_cost,
                queued_cost,
                unused_counter,
                current_key,
            ) = heapq.heappop(queue)

            del estimated_cost
            del unused_counter

            if (
                queued_cost
                > best_cost.get(
                    current_key,
                    math.inf,
                )
                + 1.0e-9
            ):
                continue

            current = records[current_key]
            expansions += 1

            position_error = math.hypot(
                current.x - goal_x,
                current.y - goal_y,
            )

            heading_error = abs(
                normalize_angle(
                    current.yaw - goal_yaw
                )
            )

            if (
                position_error <= position_tolerance
                and heading_error <= heading_tolerance
            ):
                goal_key = current_key
                break

            for direction in (1, -1):
                for steering in steering_values:
                    next_x = current.x
                    next_y = current.y
                    next_yaw = current.yaw

                    signed_substep = (
                        direction
                        * motion_step
                        / motion_substeps
                    )

                    for _ in range(motion_substeps):
                        next_x += (
                            signed_substep
                            * math.cos(next_yaw)
                        )

                        next_y += (
                            signed_substep
                            * math.sin(next_yaw)
                        )

                        next_yaw = normalize_angle(
                            next_yaw
                            + signed_substep
                            / wheelbase
                            * math.tan(steering)
                        )

                    if not (
                        minimum_x <= next_x <= maximum_x
                        and minimum_y <= next_y <= maximum_y
                    ):
                        continue

                    next_key = state_key(
                        next_x,
                        next_y,
                        next_yaw,
                        direction,
                    )

                    movement_cost = motion_step

                    if direction < 0:
                        movement_cost *= reverse_cost_ratio

                    if (
                        current.direction != 0
                        and direction != current.direction
                    ):
                        movement_cost += gear_switch_cost

                    if maximum_steering > 1.0e-6:
                        movement_cost += (
                            steering_cost_ratio
                            * motion_step
                            * abs(
                                steering
                                / maximum_steering
                            )
                        )

                    next_cost = (
                        current.cost + movement_cost
                    )

                    if (
                        next_cost
                        >= best_cost.get(
                            next_key,
                            math.inf,
                        )
                    ):
                        continue

                    records[next_key] = SearchNode(
                        x=next_x,
                        y=next_y,
                        yaw=next_yaw,
                        direction=direction,
                        steering=steering,
                        cost=next_cost,
                        parent=current_key,
                    )

                    best_cost[next_key] = next_cost

                    estimated_total = (
                        next_cost
                        + heuristic(
                            next_x,
                            next_y,
                            next_yaw,
                        )
                    )

                    heapq.heappush(
                        queue,
                        (
                            estimated_total,
                            next_cost,
                            next(counter),
                            next_key,
                        ),
                    )

        if goal_key is None:
            return None, expansions

        path = []
        current_key = goal_key

        while current_key is not None:
            current = records[current_key]
            path.append(current)
            current_key = current.parent

        path.reverse()
        return path, expansions

    def publish_path(
        self,
        frame_id: str,
        path: List[SearchNode],
        total_length: float,
        final_position_error: float,
        final_heading_error: float,
    ) -> None:
        stamp = self.get_clock().now().to_msg()

        entry_message = EntryPath()
        entry_message.header.stamp = stamp
        entry_message.header.frame_id = frame_id
        entry_message.total_length = float(total_length)
        entry_message.final_position_error = float(
            final_position_error
        )
        entry_message.final_heading_error_deg = float(
            final_heading_error
        )

        nav_message = NavPath()
        nav_message.header = entry_message.header

        for item in path:
            pose = Pose()
            pose.position.x = float(item.x)
            pose.position.y = float(item.y)
            pose.orientation.z = math.sin(
                item.yaw / 2.0
            )
            pose.orientation.w = math.cos(
                item.yaw / 2.0
            )

            entry_message.poses.append(pose)
            entry_message.directions.append(
                int(item.direction)
            )
            entry_message.steering_angles_deg.append(
                float(math.degrees(item.steering))
            )

            stamped = PoseStamped()
            stamped.header = entry_message.header
            stamped.pose = pose
            nav_message.poses.append(stamped)

        self.entry_path_pub.publish(entry_message)
        self.nav_path_pub.publish(nav_message)

    @staticmethod
    def path_length(
        path: List[SearchNode],
    ) -> float:
        total = 0.0

        for index in range(len(path) - 1):
            total += math.hypot(
                path[index + 1].x - path[index].x,
                path[index + 1].y - path[index].y,
            )

        return total

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
    node = PatrolEntryPlanner()

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
