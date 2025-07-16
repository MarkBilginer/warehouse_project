import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('path_planner_server')

    controller_yaml = os.path.join(pkg_path, 'config', 'controller.yaml')
    planner_yaml = os.path.join(pkg_path, 'config', 'planner_server.yaml')
    recovery_yaml = os.path.join(pkg_path, 'config', 'recovery.yaml')
    bt_yaml = os.path.join(pkg_path, 'config', 'bt.yaml')
    local_costmap_yaml = os.path.join(pkg_path, 'config', 'local_costmap.yaml')
    global_costmap_yaml = os.path.join(pkg_path, 'config', 'global_costmap.yaml')
    rviz_config = os.path.join(pkg_path, 'config', 'pathplanning.rviz')

    return LaunchDescription([
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml, local_costmap_yaml]
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml, global_costmap_yaml]
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='recoveries_server',
            output='screen',
            parameters=[recovery_yaml]
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_yaml]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanner',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'recoveries_server',
                    'bt_navigator',
                ]
            }]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        )
    ])
