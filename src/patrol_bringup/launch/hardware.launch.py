#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_mins200 = LaunchConfiguration('start_mins200')
    start_vehicle_can = LaunchConfiguration('start_vehicle_can')

    mins200_launch = PathJoinSubstitution([
        FindPackageShare('mins200_tcp_demo'),
        'launch',
        'mins200_tcp_demo.launch',
    ])

    vehicle_launch = PathJoinSubstitution([
        FindPackageShare('vehicle_can_interface'),
        'launch',
        'vehicle_interface.launch.py',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_mins200',
            default_value='true',
        ),

        DeclareLaunchArgument(
            'start_vehicle_can',
            default_value='false',
        ),

        LogInfo(
            msg='Starting patrol hardware layer',
        ),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                mins200_launch
            ),
            condition=IfCondition(start_mins200),
        ),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                vehicle_launch
            ),
            condition=IfCondition(start_vehicle_can),
        ),
    ])
