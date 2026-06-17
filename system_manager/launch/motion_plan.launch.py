import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = get_package_share_directory("system_manager")
    default_config = os.path.join(pkg_share, "config", "motion_plan.yaml")

    ego = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ego_planner"), "launch", "octo_path_optimizer.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("use_octo_path_optimizer")),
    )

    config_arg = DeclareLaunchArgument(
        "config",
        default_value=default_config,
        description="YAML config for EgoPlanner OctoPlanner/FASTLIO/Gazebo bridge.",
    )
    optimizer_arg = DeclareLaunchArgument(
        "use_octo_path_optimizer",
        default_value="true",
        description="Also start ego_planner/octo_path_optimizer_node, which publishes /optimized_path.",
    )

    motion_plan = Node(
        package="system_manager",
        executable="motion_plan",
        name="motion_plan",
        output="screen",
        parameters=[LaunchConfiguration("config")],
    )

    return LaunchDescription([
        config_arg,
        optimizer_arg,
        motion_plan,
        ego,
    ])
