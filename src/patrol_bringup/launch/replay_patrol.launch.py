#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    patrol_system_launch = PathJoinSubstitution([
        FindPackageShare('patrol_bringup'),
        'launch',
        'patrol_system.launch.py',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'route_file',
        ),

        DeclareLaunchArgument(
            'config_file',
            default_value=(
                '/home/nvidia/patrol_ws/'
                'config/patrol_system.yaml'
            ),
        ),

        DeclareLaunchArgument(
            'vehicle_command_topic',
            default_value='/vehicle/command',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                patrol_system_launch
            ),
            launch_arguments={
                'route_file':
                    LaunchConfiguration('route_file'),
                'config_file':
                    LaunchConfiguration('config_file'),
                'record_route_file':
                    '/tmp/patrol_replay_unused_record.yaml',
                'start_localization':
                    'true',
                'vehicle_command_topic':
                    LaunchConfiguration(
                        'vehicle_command_topic'
                    ),
            }.items(),
        ),
    ])
