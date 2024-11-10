import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable


def generate_launch_description():

    package_name = "localization_server"

    rviz_config_file = os.path.join(get_package_share_directory(package_name), 'rviz', 'localization.rviz')
    print(f"Rviz config file: {rviz_config_file}")
    
    amcl_config_file = os.path.join(get_package_share_directory(package_name), 'config', 'amcl_config.yaml') # if run with RB1 in simulation
    print(f"Amcl config file: {amcl_config_file}")

    map_file_launch_arg = DeclareLaunchArgument(
        'map_file',
        default_value = 'warehouse_map_sim.yaml'
    )

    # Use PathJoinSubstitution to construct the full path for the map file
    map_file_path = PathJoinSubstitution(
        [get_package_share_directory(package_name), 'config', LaunchConfiguration('map_file')]
    )

    return LaunchDescription([
        # Set log buffer to prevent losing logs during rapid execution
        SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),
        map_file_launch_arg,

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server', 'amcl']}]
        ),
        
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True}, 
                        {'yaml_filename':map_file_path}]
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
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_config_file]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        )
    ])
