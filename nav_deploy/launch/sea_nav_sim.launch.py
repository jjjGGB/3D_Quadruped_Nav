"""Start Quadruped_sim Go2, lidar conversion, and SEA-Nav together."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("nav_deploy"), "config", "sea_nav_go2.rviz"]
    )
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("rl_sar"), "launch", "simulation_robot.launch.py"]
            )
        ),
        launch_arguments={"rname": "go2"}.items(),
    )
    cloud_to_scan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        parameters=[
            LaunchConfiguration("config_file"),
            {"use_sim_time": True},
        ],
        remappings=[
            ("cloud_in", LaunchConfiguration("pointcloud_topic")),
            ("scan", LaunchConfiguration("scan_topic")),
        ],
    )
    policy = Node(
        package="nav_deploy",
        executable="nav_policy_node",
        name="sea_nav_policy",
        output="screen",
        parameters=[
            LaunchConfiguration("config_file"),
            {
                "model_path": LaunchConfiguration("model_path"),
                "scan_topic": LaunchConfiguration("scan_topic"),
                "use_sim_time": True,
            },
        ],
    )
    rl_sim = Node(
        package="rl_sar",
        executable="rl_sim",
        name="rl_sim_node",
        output="screen",
    )
    autostart = Node(
        package="nav_deploy",
        executable="rl_sar_autostart",
        output="screen",
        condition=IfCondition(LaunchConfiguration("auto_start")),
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=default_config),
            DeclareLaunchArgument("model_path", default_value=default_model),
            DeclareLaunchArgument(
                "pointcloud_topic",
                default_value="/velodyne_points",
                description="Quadruped_sim raw VLP-16 PointCloud2 topic",
            ),
            DeclareLaunchArgument(
                "scan_topic", default_value="/sea_nav/scan"
            ),
            DeclareLaunchArgument(
                "auto_start",
                default_value="true",
                description="Stand Go2 up and enable rl_sar navigation mode",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz2 with the /goal_pose tool configured",
            ),
            simulation,
            cloud_to_scan,
            policy,
            rviz,
            # Give Gazebo time to create ros2_control before rl_sim starts its
            # joint controller. The action client then waits for rl_sim itself.
            TimerAction(period=5.0, actions=[rl_sim, autostart]),
        ]
    )
