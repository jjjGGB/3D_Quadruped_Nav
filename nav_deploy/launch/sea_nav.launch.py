"""Launch only the SEA-Nav policy node for an existing robot stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution(
        [FindPackageShare("nav_deploy"), "config", "sea_nav_go2.yaml"]
    )
    default_model = PathJoinSubstitution(
        [FindPackageShare("nav_deploy"), "models", "sea_nav_go2.onnx"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=default_config),
            DeclareLaunchArgument("model_path", default_value=default_model),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="nav_deploy",
                executable="nav_policy_node",
                name="sea_nav_policy",
                output="screen",
                parameters=[
                    LaunchConfiguration("config_file"),
                    {
                        "model_path": LaunchConfiguration("model_path"),
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    },
                ],
            ),
        ]
    )
