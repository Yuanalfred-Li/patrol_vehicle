#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare('patrol_bringup')

    hardware_launch = PathJoinSubstitution([
        bringup_share,
        'launch',
        'hardware.launch.py',
    ])

    patrol_system_launch = PathJoinSubstitution([
        bringup_share,
        'launch',
        'patrol_system.launch.py',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'route_file',
            default_value=(
                '/home/nvidia/patrol_ws/'
                'routes/test_route.yaml'
            ),
        ),

        DeclareLaunchArgument(
            'record_route_file',
            default_value=(
                '/home/nvidia/patrol_ws/'
                'routes/recorded_route.yaml'
            ),
        ),

        DeclareLaunchArgument(
            'start_mins200',
            default_value='true',
        ),

        DeclareLaunchArgument(
            'start_vehicle_can',
            default_value='false',
        ),

        DeclareLaunchArgument(
            'start_localization',
            default_value='false',
        ),

        DeclareLaunchArgument(
            'vehicle_command_topic',
            default_value='/patrol/test_vehicle_command',
        ),

        LogInfo(
            msg='Starting complete patrol system',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                hardware_launch
            ),
            launch_arguments={
                'start_mins200':
                    LaunchConfiguration('start_mins200'),
                'start_vehicle_can':
                    LaunchConfiguration('start_vehicle_can'),
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                patrol_system_launch
            ),
            launch_arguments={
                'route_file':
                    LaunchConfiguration('route_file'),
                'record_route_file':
                    LaunchConfiguration('record_route_file'),
                'start_localization':
                    LaunchConfiguration('start_localization'),
                'vehicle_command_topic':
                    LaunchConfiguration('vehicle_command_topic'),
            }.items(),
        ),
    ])
