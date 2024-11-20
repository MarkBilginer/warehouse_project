from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    package_dir = get_package_share_directory('path_planner_server')

    # Define paths to configuration files
    controller_yaml = os.path.join(package_dir, 'config', 'controller_server.yaml')
    planner_yaml = os.path.join(package_dir, 'config', 'planner_server.yaml')
    local_costmap_yaml = os.path.join(package_dir, 'config', 'local_costmap.yaml')
    global_costmap_yaml = os.path.join(package_dir, 'config', 'global_costmap.yaml')
    recoveries_yaml = os.path.join(package_dir, 'config', 'recoveries_server.yaml')
    bt_yaml = os.path.join(package_dir, 'config', 'bt_navigator.yaml')
    bt_file = os.path.join(package_dir, 'config', 'navigate_w_replanning_and_recovery.xml')
    rviz_config = os.path.join(package_dir, 'rviz', 'pathplanning.rviz')

    return LaunchDescription([
            Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanner',
            output='screen',
            parameters=[
                {'use_sim_time': True},  # Sync with simulation
                {'autostart': True},
                {'node_names': [
                    'controller_server',
                    'planner_server',
                    'recoveries_server',
                    'bt_navigator',
                ]}
            ]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[
                {'use_sim_time': True}  # Sync with simulation
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_map_publisher',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[
                controller_yaml,
                {'global_frame': 'map'},  # Override
                {'robot_base_frame': 'robot_base_footprint'},  # Override
                {'use_sim_time': True}  # Sync with simulation
            ],
            remappings=[
                ("/cmd_vel", "/diffbot_base_controller/cmd_vel_unstamped"),
            ],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[
                planner_yaml,
                {'global_frame': 'map'},  # Override
                {'robot_base_frame': 'robot_base_footprint'},  # Override
                {'use_sim_time': True}  # Sync with simulation
            ]
        ),
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='global_costmap',
            output='screen',
            parameters=[
                global_costmap_yaml,
                {'global_frame': 'map'},  # Override
                {'robot_base_frame': 'robot_base_footprint'},  # Override
                {'use_sim_time': True}  # Sync with simulation
            ]
        ),
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='local_costmap',
            output='screen',
            parameters=[
                local_costmap_yaml,
                 {'global_frame': 'odom'},  # Override
                {'robot_base_frame': 'robot_base_footprint'},  # Override
                {'use_sim_time': True}  # Sync with simulation
            ]
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='recoveries_server',
            parameters=[
                recoveries_yaml,
                {'global_frame': 'map'},  # Override
                {'local_frame': 'odom'},  # Override
                {'robot_base_frame': 'robot_base_footprint'},  # Override
                {'use_sim_time': True}  # Sync with simulation
            ],
            output='screen'
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[
                bt_yaml,
                {'default_nav_to_pose_bt_xml': bt_file},  # Include BT XML
                {'global_frame': 'map'},  # Override
                {'robot_base_frame': 'robot_base_footprint'},  # Override
                {'odom_topic': 'odom'},  # Override
                {'use_sim_time': True}  # Sync with simulation
            ]
        ),

        Node(
            package='path_planner_server',
            executable='nav_to_pose_action_client',
            name='nav_to_pose_action_client',
            output='screen',
            parameters=[
                {'use_sim_time': True}  # Sync with simulation
            ]
        ),
    ])

