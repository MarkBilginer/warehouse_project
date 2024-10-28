import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable

def generate_launch_description():

    package_name = "map_server"

    map_file = os.path.join(get_package_share_directory(package_name), 'config', 'warehouse_map_sim.yaml')

    rviz_config_file = os.path.join(get_package_share_directory(package_name), 'rviz', 'mapping.rviz')
    print(f"Rviz config file: {rviz_config_file}")

    return LaunchDescription([
            # Set log buffer to prevent losing logs during rapid execution
        SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True, 
                         'yaml_filename':map_file}]
            ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': ['map_server']}]
        )
    ])
