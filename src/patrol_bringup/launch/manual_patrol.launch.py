#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    vehicle_command_topic = LaunchConfiguration(
        'vehicle_command_topic'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'vehicle_command_topic',
            default_value='/vehicle/command',
            description='Final vehicle command output topic',
        ),

        LogInfo(
            msg=[
                'Manual control output: ',
                vehicle_command_topic,
            ],
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
