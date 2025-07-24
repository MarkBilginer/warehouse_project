import os
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch import logging as launch_logging

def launch_setup(context, *args, **kwargs):
    logger = launch_logging.get_logger('pathplanner.launch.py')

    # 🧠 Detect sim vs real by reading the map filename
    try:
        with open('/tmp/active_map_file.txt', 'r') as f:
            map_filename = f.read().strip()
            is_sim = 'sim' in map_filename.lower()
            logger.info(f"📄 Detected map: {map_filename}")
    except FileNotFoundError:
        is_sim = False
        logger.warn("⚠️ Could not detect map file, assuming real robot environment.")

    logger.info(f"{'🟢' if is_sim else '🟠'} {'Simulation' if is_sim else 'Real'} environment selected.")

    pkg_share = FindPackageShare('path_planner_server')
    config_dir = PathJoinSubstitution([pkg_share, 'config'])

    nav2_params_file = PathJoinSubstitution([
        config_dir,
        'nav2_params_sim.yaml' if is_sim else 'nav2_params_real.yaml'
    ])

    rviz_config = PathJoinSubstitution([
        config_dir,
        'pathplanning_minimal_sim.rviz' if is_sim else 'pathplanning_minimal_real.rviz'
    ])

    cmd_vel_remap = [
        ('/cmd_vel', '/diffbot_base_controller/cmd_vel_unstamped')
    ] if is_sim else []

    return [
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params_file],
            remappings=cmd_vel_remap
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params_file]
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='recoveries_server',
            output='screen',
            parameters=[nav2_params_file]
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params_file]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanner',
            output='screen',
            parameters=[{
                'use_sim_time': is_sim,
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'recoveries_server',
                    'bt_navigator'
                ]
            }]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': is_sim}]
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
