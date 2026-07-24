import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    this_package = FindPackageShare('go2_config')
    default_params_file_path = PathJoinSubstitution(
        [this_package, 'config/autonomy', 'octomap.yaml']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='params_file',
            default_value=default_params_file_path,
            description='octomap_server params file',
        ),
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
            remappings=[('cloud_in', '/lidar3d/points')],
        ),
    ])
