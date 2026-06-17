import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_path = get_package_share_directory('ego_planner')
    default_config = os.path.join(pkg_path, 'config', 'octo_path_optimizer.yaml')

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Path to octo_path_optimizer parameter YAML.'
    )

    optimizer_node = Node(
        package='ego_planner',
        executable='octo_path_optimizer_node',
        name='octo_path_optimizer_node',
        output='screen',
        parameters=[LaunchConfiguration('config')],
    )

    return LaunchDescription([
        config_arg,
        optimizer_node,
    ])
