import launch
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare("fastlio2"), "rviz", "fastlio2.rviz"]
    )

    config_path = PathJoinSubstitution(
        [FindPackageShare("fastlio2"), "config", "lio.yaml"]
    )


    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_lio_odom_broadcaster',
        arguments=[
            '--x', '-5.0', 
            '--y', '7.0', 
            '--z', '0.5', 
            '--yaw', '0.0', 
            '--pitch', '0.0', 
            '--roll', '0.0', 
            '--frame-id', 'map', 
            '--child-frame-id', 'lio_odom'
        ],
        output='screen'
    )

    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="fastlio2",
                namespace="fastlio2",
                executable="lio_node",
                name="lio_node",
                output="log",
                parameters=[{"config_path": config_path.perform(launch.LaunchContext())}]
            ),
            static_tf_node,
            # launch_ros.actions.Node(
            #     package="rviz2",
            #     namespace="fastlio2",
            #     executable="rviz2",
            #     name="rviz2",
            #     output="log",
            #     arguments=["-d", rviz_cfg.perform(launch.LaunchContext())],
            # ),
        ]
    )
