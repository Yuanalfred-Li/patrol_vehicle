#!/usr/bin/env python3

import math
import os
import signal
import subprocess
import time
from pathlib import Path

import rclpy
import yaml
from msg_out.msg import ImuStatus
from patrol_interfaces.msg import LocalizationStatus, TaskStatus
from patrol_interfaces.srv import SetControlMode
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import Trigger
from vehicle_can_msg.msg import VehicleCommand


ORIGIN_LATITUDE = 34.0
ORIGIN_LONGITUDE = 113.0
ORIGIN_ALTITUDE = 100.0

ROUTE_FILE = Path("/tmp/test_absolute_full_route.yaml")
RECORD_FILE = Path("/tmp/test_absolute_recorded_route.yaml")
LOG_FILE = Path("/tmp/test_absolute_full_mission.log")

WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3

WHEELBASE = 0.67
MAX_STEERING_DEG = 30.0
MAX_STEERING_REQUEST = 400.0
RPM_TO_MPS = 0.01
DT = 0.05
MAX_TEST_TIME = 80.0


def enu_to_geodetic(east, north, up):
    latitude0 = math.radians(ORIGIN_LATITUDE)

    sin_latitude = math.sin(latitude0)

    denominator = math.sqrt(
        1.0 - WGS84_E2 * sin_latitude * sin_latitude
    )

    radius_n = WGS84_A / denominator

    radius_m = (
        WGS84_A
        * (1.0 - WGS84_E2)
        / denominator ** 3
    )

    latitude = latitude0 + north / (
        radius_m + ORIGIN_ALTITUDE
    )

    longitude = math.radians(
        ORIGIN_LONGITUDE
    ) + east / (
        (radius_n + ORIGIN_ALTITUDE)
        * math.cos(latitude0)
    )

    altitude = ORIGIN_ALTITUDE + up

    return (
        math.degrees(latitude),
        math.degrees(longitude),
        altitude,
    )


def make_route():
    local_points = [
        (0.0, 0.0),
        (1.5, 0.0),
        (3.0, 0.0),
    ]

    waypoints = []

    for index, (east, north) in enumerate(local_points):
        latitude, longitude, altitude = enu_to_geodetic(
            east,
            north,
            0.0,
        )

        waypoints.append({
            "index": index,
            "x": 10000.0 + index,
            "y": -20000.0 - index,
            "z": 0.0,
            "latitude": latitude,
            "longitude": longitude,
            "altitude": altitude,
            "heading_deg": 90.0,
            "ros_yaw_deg": 0.0,
            "gps_status": 2,
            "nsv1": 20,
            "nsv2": 20,
        })

    data = {
        "format_version": 2,
        "frame_id": "patrol_map",
        "coordinate_system": {
            "geodetic": "WGS84",
            "local": "ENU",
            "origin_source": "first_waypoint",
        },
        "origin": {
            "latitude": ORIGIN_LATITUDE,
            "longitude": ORIGIN_LONGITUDE,
            "altitude": ORIGIN_ALTITUDE,
        },
        "summary": {
            "point_count": len(waypoints),
            "total_length": 3.0,
        },
        "waypoints": waypoints,
    }

    ROUTE_FILE.write_text(
        yaml.safe_dump(
            data,
            allow_unicode=True,
            sort_keys=False,
        ),
        encoding="utf-8",
    )


make_route()

log_handle = LOG_FILE.open(
    "w",
    encoding="utf-8",
)

launch_process = subprocess.Popen(
    [
        "ros2",
        "launch",
        "patrol_bringup",
        "full_patrol.launch.py",
        f"route_file:={ROUTE_FILE}",
        f"record_route_file:={RECORD_FILE}",
        "start_mins200:=false",
        "start_vehicle_can:=false",
        "start_localization:=true",
        "vehicle_command_topic:=/patrol/test_vehicle_command",
    ],
    stdout=log_handle,
    stderr=subprocess.STDOUT,
    start_new_session=True,
)


rclpy.init()
node = Node("absolute_full_mission_test")

gps_pub = node.create_publisher(
    NavSatFix,
    "/gps/data",
    20,
)

imu_pub = node.create_publisher(
    ImuStatus,
    "/imu/status",
    20,
)

status_qos = QoSProfile(depth=1)
status_qos.reliability = ReliabilityPolicy.RELIABLE
status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

vehicle_command = None
localization_status = None
entry_status = None
route_status = None
mission_status = None


def vehicle_callback(msg):
    global vehicle_command
    vehicle_command = msg


def localization_callback(msg):
    global localization_status
    localization_status = msg


def entry_callback(msg):
    global entry_status
    entry_status = msg


def route_callback(msg):
    global route_status
    route_status = msg


def mission_callback(msg):
    global mission_status
    mission_status = msg


node.create_subscription(
    VehicleCommand,
    "/patrol/test_vehicle_command",
    vehicle_callback,
    20,
)

node.create_subscription(
    LocalizationStatus,
    "/patrol/localization_status",
    localization_callback,
    20,
)

node.create_subscription(
    TaskStatus,
    "/patrol/entry_executor/status",
    entry_callback,
    status_qos,
)

node.create_subscription(
    TaskStatus,
    "/patrol/route_follower/status",
    route_callback,
    status_qos,
)

node.create_subscription(
    TaskStatus,
    "/patrol/mission/status",
    mission_callback,
    status_qos,
)

mode_client = node.create_client(
    SetControlMode,
    "/patrol/set_control_mode",
)

mission_client = node.create_client(
    Trigger,
    "/patrol/mission/start",
)


east = 1.50
north = 0.80
up = 0.0
ros_yaw = math.radians(170.0)


def publish_sensor_state():
    latitude, longitude, altitude = enu_to_geodetic(
        east,
        north,
        up,
    )

    stamp = node.get_clock().now().to_msg()

    gps = NavSatFix()
    gps.header.stamp = stamp
    gps.header.frame_id = "gps_link"
    gps.status.status = 2
    gps.latitude = latitude
    gps.longitude = longitude
    gps.altitude = altitude
    gps_pub.publish(gps)

    imu = ImuStatus()

    if hasattr(imu, "header"):
        imu.header.stamp = stamp
        imu.header.frame_id = "imu_link"

    imu.nsv1 = 20
    imu.nsv2 = 20
    imu.yaw = (
        90.0 - math.degrees(ros_yaw)
    ) % 360.0

    imu_pub.publish(imu)


def spin_for(duration):
    end_time = time.monotonic() + duration

    while time.monotonic() < end_time:
        publish_sensor_state()
        rclpy.spin_once(node, timeout_sec=0.02)
        time.sleep(0.03)


def wait_future(future, timeout):
    deadline = time.monotonic() + timeout

    while (
        not future.done()
        and time.monotonic() < deadline
    ):
        publish_sensor_state()
        rclpy.spin_once(node, timeout_sec=0.05)

    return future.done()


try:
    service_deadline = time.monotonic() + 20.0

    while time.monotonic() < service_deadline:
        publish_sensor_state()
        rclpy.spin_once(node, timeout_sec=0.05)

        if (
            mode_client.service_is_ready()
            and mission_client.service_is_ready()
        ):
            break

        time.sleep(0.05)

    if not mode_client.service_is_ready():
        raise RuntimeError("控制模式服务不可用")

    if not mission_client.service_is_ready():
        raise RuntimeError("任务启动服务不可用")

    localization_deadline = time.monotonic() + 10.0

    while time.monotonic() < localization_deadline:
        publish_sensor_state()
        rclpy.spin_once(node, timeout_sec=0.05)

        if (
            localization_status is not None
            and localization_status.valid
        ):
            break

    if (
        localization_status is None
        or not localization_status.valid
    ):
        reason = (
            localization_status.reason
            if localization_status is not None
            else "no localization status"
        )
        raise RuntimeError(
            f"定位没有生效: {reason}"
        )

    print(
        "INITIAL_LOCALIZATION_EAST:",
        round(localization_status.east, 3),
    )
    print(
        "INITIAL_LOCALIZATION_NORTH:",
        round(localization_status.north, 3),
    )
    print(
        "INITIAL_LOCALIZATION_VALID:",
        localization_status.valid,
    )

    mode_request = SetControlMode.Request()
    mode_request.mode = 2

    mode_future = mode_client.call_async(
        mode_request
    )

    if not wait_future(mode_future, 5.0):
        raise RuntimeError("AUTO模式设置超时")

    mode_response = mode_future.result()

    print("AUTO_MODE_SUCCESS:", mode_response.success)

    if not mode_response.success:
        raise RuntimeError(
            f"AUTO模式设置失败: "
            f"{mode_response.message}"
        )

    mission_future = mission_client.call_async(
        Trigger.Request()
    )

    if not wait_future(mission_future, 5.0):
        raise RuntimeError("任务启动服务超时")

    mission_response = mission_future.result()

    print(
        "MISSION_START_SUCCESS:",
        mission_response.success,
    )
    print(
        "MISSION_START_MESSAGE:",
        mission_response.message,
    )

    if not mission_response.success:
        raise RuntimeError(
            "任务启动失败"
        )

    entry_forward_steps = 0
    entry_reverse_steps = 0
    entry_switches = 0
    route_forward_steps = 0

    previous_entry_direction = 0
    succeeded = False
    failed = False

    start_time = time.monotonic()
    last_print_time = start_time

    while (
        time.monotonic() - start_time
        < MAX_TEST_TIME
    ):
        loop_start = time.monotonic()

        publish_sensor_state()
        rclpy.spin_once(node, timeout_sec=0.01)

        active_source = "STOP"

        if (
            entry_status is not None
            and entry_status.state
            == TaskStatus.RUNNING
        ):
            active_source = "ENTRY"

        elif (
            route_status is not None
            and route_status.state
            == TaskStatus.RUNNING
        ):
            active_source = "ROUTE"

        if vehicle_command is not None:
            rpm = float(
                vehicle_command.target_speed_rpm
            )

            steering_request = float(
                vehicle_command
                .target_steering_angle_deg
            )

            if active_source == "ENTRY":
                direction = 0

                if rpm > 0.1:
                    direction = 1
                    entry_forward_steps += 1
                elif rpm < -0.1:
                    direction = -1
                    entry_reverse_steps += 1

                if direction != 0:
                    if (
                        previous_entry_direction != 0
                        and direction
                        != previous_entry_direction
                    ):
                        entry_switches += 1

                    previous_entry_direction = direction

            elif (
                active_source == "ROUTE"
                and rpm > 0.1
            ):
                route_forward_steps += 1

            if abs(rpm) > 0.1:
                steering_request = max(
                    -MAX_STEERING_REQUEST,
                    min(
                        MAX_STEERING_REQUEST,
                        steering_request,
                    ),
                )

                steering_angle = math.radians(
                    steering_request
                    / MAX_STEERING_REQUEST
                    * MAX_STEERING_DEG
                )

                velocity = rpm * RPM_TO_MPS

                east += (
                    velocity
                    * math.cos(ros_yaw)
                    * DT
                )

                north += (
                    velocity
                    * math.sin(ros_yaw)
                    * DT
                )

                ros_yaw += (
                    velocity
                    / WHEELBASE
                    * math.tan(steering_angle)
                    * DT
                )

                ros_yaw = math.atan2(
                    math.sin(ros_yaw),
                    math.cos(ros_yaw),
                )

        now = time.monotonic()

        if now - last_print_time >= 2.0:
            last_print_time = now

            print(
                f"t={now - start_time:5.1f}s "
                f"source={active_source:5s} "
                f"east={east:+6.3f} "
                f"north={north:+6.3f} "
                f"yaw={math.degrees(ros_yaw):+7.2f}"
            )

        if mission_status is not None:
            if (
                mission_status.state
                == TaskStatus.SUCCEEDED
            ):
                succeeded = True
                break

            if (
                mission_status.state
                == TaskStatus.FAILED
            ):
                failed = True
                break

        elapsed = time.monotonic() - loop_start

        if elapsed < DT:
            time.sleep(DT - elapsed)

    final_latitude, final_longitude, _ = (
        enu_to_geodetic(
            east,
            north,
            up,
        )
    )

    route_end_error = math.hypot(
        east - 3.0,
        north,
    )

    route_heading_error = abs(
        math.degrees(
            math.atan2(
                math.sin(ros_yaw),
                math.cos(ros_yaw),
            )
        )
    )

    passed = (
        succeeded
        and not failed
        and entry_forward_steps > 0
        and entry_reverse_steps > 0
        and entry_switches >= 1
        and route_forward_steps > 0
        and route_end_error <= 0.35
        and route_heading_error <= 10.0
    )

    print()
    print("===== RESULT =====")
    print("MISSION_SUCCEEDED:", succeeded)
    print("MISSION_FAILED:", failed)
    print(
        "ENTRY_FORWARD_STEPS:",
        entry_forward_steps,
    )
    print(
        "ENTRY_REVERSE_STEPS:",
        entry_reverse_steps,
    )
    print(
        "ENTRY_DIRECTION_SWITCHES:",
        entry_switches,
    )
    print(
        "ROUTE_FORWARD_STEPS:",
        route_forward_steps,
    )
    print("FINAL_EAST:", round(east, 3))
    print("FINAL_NORTH:", round(north, 3))
    print(
        "FINAL_YAW_DEG:",
        round(math.degrees(ros_yaw), 3),
    )
    print(
        "FINAL_LATITUDE:",
        f"{final_latitude:.10f}",
    )
    print(
        "FINAL_LONGITUDE:",
        f"{final_longitude:.10f}",
    )
    print(
        "ROUTE_END_ERROR:",
        round(route_end_error, 3),
    )
    print(
        "ROUTE_HEADING_ERROR_DEG:",
        round(route_heading_error, 3),
    )
    print(
        "TEST_RESULT:",
        "PASS" if passed else "FAIL",
    )

finally:
    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()

    try:
        os.killpg(
            launch_process.pid,
            signal.SIGINT,
        )
        launch_process.wait(timeout=8.0)
    except (
        ProcessLookupError,
        subprocess.TimeoutExpired,
    ):
        try:
            os.killpg(
                launch_process.pid,
                signal.SIGKILL,
            )
        except ProcessLookupError:
            pass

    log_handle.close()

    print()
    print("===== LAUNCH LOG TAIL =====")

    if LOG_FILE.exists():
        lines = LOG_FILE.read_text(
            encoding="utf-8",
            errors="replace",
        ).splitlines()

        for line in lines[-25:]:
            print(line)
