#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    route_file = LaunchConfiguration('route_file')
    vehicle_command_topic = LaunchConfiguration(
        'vehicle_command_topic'
    )

    origin_latitude = ParameterValue(
        LaunchConfiguration('origin_latitude'),
        value_type=float,
    )

    origin_longitude = ParameterValue(
        LaunchConfiguration('origin_longitude'),
        value_type=float,
    )

    origin_altitude = ParameterValue(
        LaunchConfiguration('origin_altitude'),
        value_type=float,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'route_file',
            default_value=(
                '/home/nvidia/patrol_ws/'
                'routes/recorded_route.yaml'
            ),
        ),

        DeclareLaunchArgument(
            'origin_latitude',
        ),

        DeclareLaunchArgument(
            'origin_longitude',
        ),

        DeclareLaunchArgument(
            'origin_altitude',
        ),

        DeclareLaunchArgument(
            'vehicle_command_topic',
            default_value='/patrol/test_vehicle_command',
        ),

        LogInfo(
            msg=[
                'Recording route to: ',
                route_file,
            ],
        ),

        Node(
            package='patrol_localization',
            executable='localization_node',
            name='patrol_localization',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'origin_set': True,
                'origin_latitude': origin_latitude,
                'origin_longitude': origin_longitude,
                'origin_altitude': origin_altitude,
            }],
        ),

        Node(
            package='patrol_route_recorder',
            executable='route_recorder_node',
            name='patrol_route_recorder',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'route_file': route_file,
                'origin_set': True,
                'origin_latitude': origin_latitude,
                'origin_longitude': origin_longitude,
                'origin_altitude': origin_altitude,
            }],
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
    ])
