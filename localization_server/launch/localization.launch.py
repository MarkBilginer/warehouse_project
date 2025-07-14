from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        description='YAML map file name (e.g., warehouse_map_sim.yaml)'
    )

    amcl_param_file_arg = DeclareLaunchArgument(
        'amcl_param_file',
        default_value='amcl_real.yaml',
        description='AMCL config YAML file'
    )

    # Shared paths
    pkg_share = FindPackageShare('localization_server')

    # Substitutions for paths
    map_file_path = PathJoinSubstitution([pkg_share, 'config', LaunchConfiguration('map_file')])
    amcl_param_path = PathJoinSubstitution([pkg_share, 'config', LaunchConfiguration('amcl_param_file')])
    rviz_config_path = PathJoinSubstitution([pkg_share, 'rviz', 'localization.rviz'])

    return LaunchDescription([
        map_file_arg,
        amcl_param_file_arg,

        # Map Server Node
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': False}, {'yaml_filename': map_file_path}]
        ),

        # AMCL Node
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_param_path]
        ),
        
        # Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[
                {'use_sim_time': False},
                {'autostart': True},
                {'node_names': ['map_server', 'amcl']}
            ]
        ),

        # RViz2 Node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        )
    ])
