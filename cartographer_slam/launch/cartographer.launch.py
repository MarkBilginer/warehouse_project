import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

    cartographer_config_dir = os.path.join(
        get_package_share_directory('cartographer_slam'), 'config'
    )

    # Declare launch arguments
    configuration_basename = LaunchConfiguration('configuration_basename')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_configuration_basename_cmd = DeclareLaunchArgument(
        'configuration_basename',
        default_value='cartographer_sim.lua',  # Default to sim, can override in real robot launch
        description='Name of the Cartographer LUA config file'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time (true) or real time (false)'
    )

    rviz_config_file = os.path.join(
        get_package_share_directory('cartographer_slam'),
        'rviz',
        'cartographer_sim.rviz'
    )

    return LaunchDescription([
        declare_configuration_basename_cmd,
        declare_use_sim_time_cmd,

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', configuration_basename
            ]
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-resolution', '0.03', '-publish_period_sec', '1.0']
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        ),
    ])
