#!/usr/bin/env python3

from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from patrol_bringup.config_loader import (
    load_patrol_config,
    section_parameters,
)


def launch_setup(context):
    route_file = LaunchConfiguration(
        'route_file'
    ).perform(context)

    config_file = LaunchConfiguration(
        'config_file'
    ).perform(context)

    vehicle_command_topic = LaunchConfiguration(
        'vehicle_command_topic'
    ).perform(context)

    record_route_file = LaunchConfiguration(
        'record_route_file'
    ).perform(context)

    start_localization = LaunchConfiguration(
        'start_localization'
    )

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

    config = load_patrol_config(config_file)

    localization_parameters = section_parameters(
        config,
        'localization',
        exclude={'origin_from_route'},
    )
    localization_parameters.update({
        'origin_set': True,
        'origin_latitude': origin_latitude,
        'origin_longitude': origin_longitude,
        'origin_altitude': origin_altitude,
    })

    entry_planner_parameters = section_parameters(
        config,
        'entry_planner',
        rename={
            'wheelbase_m': 'wheelbase',
            'motion_step_m': 'motion_step',
            'xy_resolution_m': 'xy_resolution',
            'planning_margin_m': 'planning_margin',
            'goal_position_tolerance_m':
                'goal_position_tolerance',
        },
    )
    entry_planner_parameters['route_file'] = route_file

    entry_executor_parameters = section_parameters(
        config,
        'entry_executor',
        rename={
            'wheelbase_m': 'wheelbase',
            'lookahead_distance_m':
                'lookahead_distance',
            'maximum_tracking_error_m':
                'maximum_tracking_error',
            'waypoint_tolerance_m':
                'waypoint_tolerance',
            'maximum_pass_lateral_error_m':
                'maximum_pass_lateral_error',
            'maximum_start_error_m':
                'maximum_start_error',
        },
    )

    route_follower_parameters = section_parameters(
        config,
        'route_follower',
        rename={
            'lookahead_distance_m':
                'lookahead_distance',
            'wheelbase_m': 'wheelbase',
            'maximum_mechanical_steering_deg':
                'max_mechanical_steering_deg',
            'maximum_steering_request':
                'max_steering_request',
            'maximum_speed_rpm': 'max_speed_rpm',
            'slowdown_distance_m':
                'slowdown_distance',
            'maximum_entry_path_error_m':
                'maximum_entry_path_error',
            'maximum_tracking_error_m':
                'maximum_tracking_error',
            'final_tolerance_m':
                'final_tolerance',
        },
    )
    route_follower_parameters['route_file'] = route_file

    auto_mux_parameters = section_parameters(
        config,
        'auto_command_mux',
    )

    command_manager_parameters = section_parameters(
        config,
        'command_manager',
        rename={
            'maximum_speed_rpm': 'max_speed_rpm',
            'maximum_steering_request':
                'max_steering_request',
        },
    )
    command_manager_parameters[
        'vehicle_command_topic'
    ] = vehicle_command_topic

    mission_manager_parameters = section_parameters(
        config,
        'mission_manager',
        exclude={
            'start_service',
            'stop_service',
            'load_route_service',
        },
    )

    nodes = [
        LogInfo(
            msg=[
                'Patrol configuration: ',
                config_file,
            ],
        ),
        LogInfo(
            msg=[
                'Patrol route file: ',
                route_file,
            ],
        ),
        LogInfo(
            msg=[
                'Vehicle command output: ',
                vehicle_command_topic,
            ],
        ),

        Node(
            package='patrol_localization',
            executable='localization_node',
            name='patrol_localization',
            output='screen',
            emulate_tty=True,
            condition=IfCondition(start_localization),
            parameters=[localization_parameters],
        ),

        Node(
            package='patrol_route_recorder',
            executable='route_recorder_node',
            name='patrol_route_recorder',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'route_file': record_route_file,
            }],
        ),

        Node(
            package='patrol_entry_planner',
            executable='entry_planner_node',
            name='patrol_entry_planner',
            output='screen',
            emulate_tty=True,
            parameters=[entry_planner_parameters],
        ),

        Node(
            package='patrol_entry_executor',
            executable='entry_executor_node',
            name='patrol_entry_executor',
            output='screen',
            emulate_tty=True,
            parameters=[entry_executor_parameters],
        ),

        Node(
            package='patrol_route_follower',
            executable='route_follower_node',
            name='patrol_route_follower',
            output='screen',
            emulate_tty=True,
            parameters=[route_follower_parameters],
        ),

        Node(
            package='patrol_auto_command_mux',
            executable='auto_command_mux_node',
            name='patrol_auto_command_mux',
            output='screen',
            emulate_tty=True,
            parameters=[auto_mux_parameters],
        ),

        Node(
            package='patrol_command_manager',
            executable='command_manager_node',
            name='patrol_command_manager',
            output='screen',
            emulate_tty=True,
            parameters=[command_manager_parameters],
        ),

        Node(
            package='patrol_mission_manager',
            executable='mission_manager_node',
            name='patrol_mission_manager',
            output='screen',
            emulate_tty=True,
            parameters=[mission_manager_parameters],
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
            'config_file',
            default_value=(
                '/home/nvidia/patrol_ws/'
                'config/patrol_system.yaml'
            ),
            description='Unified patrol configuration YAML',
        ),

        DeclareLaunchArgument(
            'record_route_file',
            default_value=(
                '/home/nvidia/patrol_ws/'
                'routes/recorded_route.yaml'
            ),
            description='Output YAML file for route recording',
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
