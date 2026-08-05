#!/usr/bin/env python3
"""障碍守护完整任务仿真测试（复用 test_absolute_full_mission 模式）。

在真实任务链路上运行完整任务（入轨 -> 巡迹），并在 ROUTE 阶段
按时间窗注入前方净空场景，验证分级仲裁：

- 净空(+inf)              -> 直通 15 RPM
- 障碍 2.5m               -> 限速 8 RPM
- 障碍 0.8m               -> 停车（速度 0 + 制动）
- 障碍 1.8m（超滞回）      -> 恢复 8 RPM
- 净空                     -> 恢复 15 RPM
- health 断流              -> 失效停车
- health 恢复              -> 恢复直通
最后任务应 MISSION_SUCCEEDED。
"""

import math
import os
import signal
import subprocess
import time
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from msg_out.msg import ImuStatus
from patrol_interfaces.msg import LocalizationStatus, TaskStatus
from patrol_interfaces.srv import SetControlMode
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix, Range
from std_msgs.msg import String
from std_srvs.srv import Trigger
from vehicle_can_msg.msg import VehicleCommand


ORIGIN_LATITUDE = 34.0
ORIGIN_LONGITUDE = 113.0
ORIGIN_ALTITUDE = 100.0

ROUTE_FILE = Path("/tmp/test_obstacle_guard_route.yaml")
RECORD_FILE = Path("/tmp/test_obstacle_guard_recorded.yaml")
LOG_FILE = Path("/tmp/test_obstacle_guard.log")

WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3

WHEELBASE = 0.67
MAX_STEERING_DEG = 30.0
MAX_STEERING_REQUEST = 400.0
RPM_TO_MPS = 0.01

MAX_TEST_TIME = 150.0
DT = 0.05

AUTO_SPEED_RPM = 15.0
SLOW_SPEED_RPM = 8.0

# ROUTE 阶段相对时间窗（秒）：起始点 + 各场景持续时长。
# 场景 1 处于接管恢复区（前 3m 速度 8→15 RPM 爬坡），
# 只验证守卫 CLEAR 状态，速度接受恢复区范围。
SCENARIOS = [
    (0.0, math.inf, True, "clear_warmup", 6.0, 16.0),
    (4.0, 2.5, True, "slow", SLOW_SPEED_RPM - 0.5, SLOW_SPEED_RPM + 0.5),
    (8.0, 0.8, True, "stop", 0.0, 0.0),
    (12.0, 1.8, True, "recover_slow", SLOW_SPEED_RPM - 0.5, SLOW_SPEED_RPM + 0.5),
    (16.0, math.inf, True, "clear_again", 12.0, 16.0),
    (20.0, math.inf, False, "stale", 0.0, 0.0),
    (24.0, math.inf, True, "recover_clear", 12.0, 16.0),
]


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
    # 8 m 直线，给障碍场景留出时间窗余量。
    route_length = 8.0
    local_points = [
        (float(index) * 0.5, 0.0)
        for index in range(int(route_length / 0.5) + 1)
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
            "total_length": route_length,
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

log_handle = LOG_FILE.open("w", encoding="utf-8")

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
        "start_obstacle_guard:=true",
        "vehicle_command_topic:=/patrol/test_vehicle_command",
    ],
    stdout=log_handle,
    stderr=subprocess.STDOUT,
    start_new_session=True,
)


rclpy.init()
node = Node("obstacle_guard_mission_test")

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

range_pub = node.create_publisher(
    Range,
    "/gap_follower/front_clearance_distance",
    20,
)

health_pub = node.create_publisher(
    String,
    "/gap_follower/front_clearance_health",
    20,
)

status_qos = QoSProfile(depth=1)
status_qos.reliability = ReliabilityPolicy.RELIABLE
status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

vehicle_command = None
localization_status = None
pose_count = 0
route_status = None
mission_status = None
guard_status = None


def vehicle_callback(msg):
    global vehicle_command
    vehicle_command = msg


def localization_callback(msg):
    global localization_status
    localization_status = msg


def pose_callback(msg):
    global pose_count
    del msg
    pose_count += 1


def route_callback(msg):
    global route_status
    route_status = msg


def mission_callback(msg):
    global mission_status
    mission_status = msg


def guard_status_callback(msg):
    global guard_status
    try:
        import json
        guard_status = json.loads(msg.data)
    except (ValueError, TypeError):
        guard_status = None


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
    PoseStamped,
    "/patrol/pose",
    pose_callback,
    20,
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

node.create_subscription(
    String,
    "/patrol/obstacle_guard/status",
    guard_status_callback,
    10,
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

current_range = math.inf
current_health_active = True


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

    distance = Range()
    distance.header.stamp = stamp
    distance.header.frame_id = "livox_frame"
    distance.radiation_type = Range.INFRARED
    distance.field_of_view = math.radians(40.0)
    distance.min_range = 0.25
    distance.max_range = 3.0
    distance.range = current_range
    range_pub.publish(distance)

    health = String()
    health.data = (
        '{"data_fresh": true, "reason": "scan_fresh"}'
        if current_health_active
        else '{"data_fresh": false, "reason": "scan_timeout"}'
    )
    health_pub.publish(health)


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


def update_motion():
    """按最近车辆命令推进仿真位置（与主测试同一运动模型）。"""
    global east, north, ros_yaw

    if vehicle_command is None:
        return

    rpm = float(vehicle_command.target_speed_rpm)

    if abs(rpm) <= 0.1:
        return

    steering_request = float(
        vehicle_command.target_steering_angle_deg
    )
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

    east += velocity * math.cos(ros_yaw) * DT
    north += velocity * math.sin(ros_yaw) * DT

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


def run_scenario_window(
    duration,
    expect_speed_min,
    expect_speed_max,
    expect_state,
    window_name,
):
    samples = []
    end_time = time.monotonic() + duration

    while time.monotonic() < end_time:
        publish_sensor_state()
        update_motion()
        rclpy.spin_once(node, timeout_sec=0.02)
        time.sleep(0.02)

        if vehicle_command is not None:
            samples.append((
                float(vehicle_command.target_speed_rpm),
                int(vehicle_command.brake_pedal),
            ))

    check_samples = samples[len(samples) // 2:]

    speeds = [item[0] for item in check_samples]
    median_speed = (
        sorted(speeds)[len(speeds) // 2]
        if speeds
        else 0.0
    )
    max_brake = (
        max(item[1] for item in check_samples)
        if check_samples
        else 0
    )

    state_ok = (
        guard_status is not None
        and guard_status.get("state") == expect_state
    )
    speed_ok = (
        expect_speed_min
        <= median_speed
        <= expect_speed_max
    )
    brake_ok = (
        max_brake >= 50
        if expect_speed_max < 0.5
        else True
    )

    passed = state_ok and speed_ok and brake_ok

    print(
        f"{window_name}: "
        f"state={guard_status.get('state') if guard_status else None}"
        f"(want {expect_state}) "
        f"median_speed={median_speed:.1f} "
        f"(want {expect_speed_min}~{expect_speed_max}) "
        f"brake={max_brake} "
        f"-> {'PASS' if passed else 'FAIL'}"
    )
    return passed


def main():
    global east, north, ros_yaw
    global current_range, current_health_active

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
            and pose_count >= 3
        ):
            break

    if (
        localization_status is None
        or not localization_status.valid
    ):
        raise RuntimeError("定位没有生效")

    if pose_count < 3:
        raise RuntimeError(
            f"/patrol/pose 未稳定，收到 {pose_count} 帧"
        )

    # 继续发布一小段时间，确保所有控制节点都收到位姿。
    spin_for(2.0)

    print("INITIAL_POSE_SAMPLES:", pose_count)

    mode_request = SetControlMode.Request()
    mode_request.mode = 2

    mode_future = mode_client.call_async(mode_request)

    if not wait_future(mode_future, 5.0):
        raise RuntimeError("AUTO模式设置超时")

    mode_response = mode_future.result()

    print("AUTO_MODE_SUCCESS:", mode_response.success)

    if not mode_response.success:
        raise RuntimeError("AUTO模式设置失败")

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

    if not mission_response.success:
        raise RuntimeError("任务启动失败")

    succeeded = False
    failed = False
    route_start_time = None

    results = []
    scenario_index = 0

    start_time = time.monotonic()
    last_print_time = start_time

    while (
        time.monotonic() - start_time
        < MAX_TEST_TIME
    ):
        publish_sensor_state()
        rclpy.spin_once(node, timeout_sec=0.01)

        now = time.monotonic()

        if (
            route_status is not None
            and route_status.state == TaskStatus.RUNNING
        ):
            if route_start_time is None:
                route_start_time = now
                print("ROUTE_PHASE_STARTED")

            # 按时间窗切换前方净空场景。
            elapsed = now - route_start_time

            while (
                scenario_index < len(SCENARIOS)
                and elapsed
                >= SCENARIOS[scenario_index][0]
            ):
                _, range_value, health_active, name, min_speed, max_speed = (
                    SCENARIOS[scenario_index]
                )
                current_range = range_value
                current_health_active = health_active

                print(
                    f"SCENARIO: {name} "
                    f"range={range_value} "
                    f"health={health_active}"
                )

                results.append(run_scenario_window(
                    4.0,
                    min_speed,
                    max_speed,
                    {
                        "clear_warmup": "CLEAR",
                        "slow": "SLOW",
                        "stop": "STOP",
                        "recover_slow": "SLOW",
                        "clear_again": "CLEAR",
                        "stale": "STALE",
                        "recover_clear": "CLEAR",
                    }[name],
                    f"scenario_{scenario_index + 1}_{name}",
                ))
                scenario_index += 1

                now = time.monotonic()
                elapsed = now - route_start_time

        if (
            mission_status is not None
            and mission_status.state == TaskStatus.SUCCEEDED
        ):
            succeeded = True
            break

        if (
            mission_status is not None
            and mission_status.state == TaskStatus.FAILED
        ):
            failed = True
            break

        update_motion()

        if now - last_print_time >= 2.0:
            last_print_time = now
            print(
                f"t={now - start_time:5.1f}s "
                f"east={east:+6.3f} north={north:+6.3f}"
            )

    if route_start_time is None:
        print("FAIL: route phase never started")
        return False

    if scenario_index < len(SCENARIOS):
        print(
            "FAIL: mission ended before all scenarios ran "
            f"({scenario_index}/{len(SCENARIOS)})"
        )
        return False

    print("MISSION_SUCCEEDED:", succeeded)
    print("MISSION_FAILED:", failed)

    passed = (
        succeeded
        and not failed
        and all(results)
    )

    print("TEST_RESULT:", "PASS" if passed else "FAIL")
    return passed


success = False

try:
    success = main()
except BaseException as exc:
    import traceback
    print("MAIN_EXCEPTION:", type(exc).__name__, exc)
    traceback.print_exc()
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

        for line in lines[-15:]:
            print(line)

    if not success:
        raise SystemExit(1)
