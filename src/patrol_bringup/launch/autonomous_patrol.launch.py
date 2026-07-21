#!/usr/bin/env python3

from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    route_file = LaunchConfiguration(
        'route_file'
    ).perform(context)

    route_path = Path(route_file)

    if not route_path.is_file():
        raise RuntimeError(
            f'Route file does not exist: {route_path}'
        )

    with route_path.open(
        'r',
        encoding='utf-8',
    ) as file:
        route_data = yaml.safe_load(file)

    if not isinstance(route_data, dict):
        raise RuntimeError(
            'Route YAML root must be a mapping'
        )

    origin = route_data.get('origin')

    if not isinstance(origin, dict):
        raise RuntimeError(
            'Route file does not contain an origin mapping'
        )

    try:
        origin_latitude = float(origin['latitude'])
        origin_longitude = float(origin['longitude'])
        origin_altitude = float(origin['altitude'])
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError(
            f'Invalid route origin: {exc}'
        ) from exc

    start_localization = LaunchConfiguration(
        'start_localization'
    )

    vehicle_command_topic = LaunchConfiguration(
        'vehicle_command_topic'
    )

    common_arguments = ['--ros-args']

    nodes = [
        LogInfo(
            msg=[
                'Patrol route file: ',
                route_file,
            ]
        ),
        LogInfo(
            msg=[
                'Vehicle command output: ',
                vehicle_command_topic,
            ]
        ),

        Node(
            package='patrol_localization',
            executable='localization_node',
            name='patrol_localization',
            output='screen',
            emulate_tty=True,
            condition=IfCondition(start_localization),
            parameters=[{
                'origin_set': True,
                'origin_latitude': origin_latitude,
                'origin_longitude': origin_longitude,
                'origin_altitude': origin_altitude,
            }],
        ),

        Node(
            package='patrol_entry_planner',
            executable='entry_planner_node',
            name='patrol_entry_planner',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'route_file': route_file,
            }],
        ),

        Node(
            package='patrol_entry_executor',
            executable='entry_executor_node',
            name='patrol_entry_executor',
            output='screen',
            emulate_tty=True,
        ),

        Node(
            package='patrol_route_follower',
            executable='route_follower_node',
            name='patrol_route_follower',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'route_file': route_file,
            }],
        ),

        Node(
            package='patrol_auto_command_mux',
            executable='auto_command_mux_node',
            name='patrol_auto_command_mux',
            output='screen',
            emulate_tty=True,
        ),

        Node(
            package='patrol_command_manager',
            executable='command_manager_node',
            name='patrol_command_manager',
            output='screen',
            emulate_tty=True,
            remappings=[
                (
                    '/vehicle/command',
                    vehicle_command_topic,
                ),
            ],
        ),

        Node(
            package='patrol_mission_manager',
            executable='mission_manager_node',
            name='patrol_mission_manager',
            output='screen',
            emulate_tty=True,
        ),
    ]

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'route_file',
            default_value=(
                '/home/nvidia/patrol_ws/'
                'routes/test_route.yaml'
            ),
            description='Recorded patrol route YAML file',
        ),

        DeclareLaunchArgument(
            'start_localization',
            default_value='true',
            description='Start patrol localization node',
        ),

        DeclareLaunchArgument(
            'vehicle_command_topic',
            default_value='/patrol/test_vehicle_command',
            description=(
                'Final command output topic. '
                'Use /vehicle/command only for real vehicle tests.'
            ),
        ),

        OpaqueFunction(function=launch_setup),
    ])
