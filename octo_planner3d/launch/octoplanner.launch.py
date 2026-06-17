import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg_path = os.path.join(get_package_share_directory('octo_planner3d'))

    rviz_config_file = os.path.join(pkg_path, 'rviz', 'octoplanner.rviz')
    planner_config_file = os.path.join(pkg_path, 'config', 'octoplanner.yaml')

    parameter_overrides = {}
    input_pcd = LaunchConfiguration('input_pcd').perform(context)
    output_bt = LaunchConfiguration('output_bt').perform(context)
    if input_pcd:
        parameter_overrides['input_pcd'] = input_pcd
    if output_bt:
        parameter_overrides['output_bt'] = output_bt

    node_octo = Node(
        package="octo_planner3d",
        executable="octo_planner_rviz_node",
        name="octo_planner_rviz_node",
        output="screen",
        parameters=[planner_config_file, parameter_overrides],
    )

    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return [node_octo, node_rviz]


def generate_launch_description():
    input_pcd_arg = DeclareLaunchArgument(
        'input_pcd',
        default_value='',
        description='Override input PCD. Empty uses config/octoplanner.yaml.'
    )

    output_bt_arg = DeclareLaunchArgument(
        'output_bt',
        default_value='',
        description='Override output OctoMap .bt file. Empty uses config/octoplanner.yaml.'
    )

    return LaunchDescription([
        input_pcd_arg,
        output_bt_arg,
        OpaqueFunction(function=launch_setup),
    ])
