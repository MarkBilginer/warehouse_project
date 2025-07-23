from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    map_file = LaunchConfiguration('map_file').perform(context)
    is_sim = 'sim' in map_file.lower()

    # Choose AMCL and map_server config files based on map type
    amcl_param_filename = 'amcl_sim.yaml' if is_sim else 'amcl_real.yaml'
    map_server_param_filename = 'map_server_sim.yaml' if is_sim else 'map_server_real.yaml'
    use_sim_time = is_sim

    # Shared paths
    pkg_share = FindPackageShare('localization_server')
    map_file_path = PathJoinSubstitution([pkg_share, 'config', map_file])
    amcl_param_path = PathJoinSubstitution([pkg_share, 'config', amcl_param_filename])
    map_server_param_path = PathJoinSubstitution([pkg_share, 'config', map_server_param_filename])
    rviz_config_path = PathJoinSubstitution([pkg_share, 'rviz', 'localization.rviz'])

    with open('/tmp/active_map_file.txt', 'w') as f:
        f.write(map_file + '\n')

    return [
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                map_server_param_path,
                {'yaml_filename': map_file_path}
            ]
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                amcl_param_path,
                {'use_sim_time': use_sim_time}  # override if not in file
            ]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': True},
                {'node_names': ['map_server', 'amcl']}
            ]
        ),
        Node(
             package='rviz2',
             executable='rviz2',
             name='rviz2',
             output='screen',
             arguments=['-d', rviz_config_path],
             parameters=[{'use_sim_time': use_sim_time}]
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_file',
            description='YAML map file name (e.g., warehouse_map_sim.yaml or warehouse_map_real.yaml)'
        ),
        OpaqueFunction(function=launch_setup)
    ])
