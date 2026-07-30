"""ROS 2 node that connects the SEA-Nav Go2 policy to rl_sar."""

import math
import threading
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from tf2_ros import (
    Buffer,
    TransformBroadcaster,
    TransformException,
    TransformListener,
)

from nav_deploy.policy_core import NUM_RAYS, SeaNavGo2Policy, rotate_vector


class SeaNavPolicyNode(Node):
    def __init__(self):
        super().__init__("sea_nav_policy")
        self.lock = threading.Lock()
        self.last_log_times = {}

        self.declare_parameter("model_path", "")
        self.declare_parameter("state_source", "gazebo_model_states")
        self.declare_parameter("state_frame", "world")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_gazebo_tf", True)
        self.declare_parameter("model_name", "robot_model")
        self.declare_parameter("scan_topic", "/sea_nav/scan")
        self.declare_parameter("model_states_topic", "/gazebo/model_states")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("control_rate", 20.0)
        self.declare_parameter("sensor_timeout", 0.5)
        self.declare_parameter("goal_tolerance", 0.5)
        self.declare_parameter("initial_goal", [0.0, 7.0])
        self.declare_parameter("odom_twist_in_body_frame", True)
        self.declare_parameter("ray_angle_min_deg", -120.0)
        self.declare_parameter("ray_angle_max_deg", 120.0)
        self.declare_parameter("ray_min", 0.1)
        self.declare_parameter("ray_max", 5.0)
        self.declare_parameter("action_filter_alpha", 0.5)
        self.declare_parameter("action_min", [-0.5, -1.0, -1.0])
        self.declare_parameter("action_max", [2.0, 1.0, 1.0])

        model_path = self.get_parameter("model_path").value
        if not model_path:
            model_path = str(
                get_package_share_directory("nav_deploy")
                + "/models/sea_nav_go2.onnx"
            )
        self.policy = SeaNavGo2Policy(
            model_path=model_path,
            ray_angle_min=math.radians(
                self.get_parameter("ray_angle_min_deg").value
            ),
            ray_angle_max=math.radians(
                self.get_parameter("ray_angle_max_deg").value
            ),
            ray_min=self.get_parameter("ray_min").value,
            ray_max=self.get_parameter("ray_max").value,
            action_alpha=self.get_parameter("action_filter_alpha").value,
            action_min=self.get_parameter("action_min").value,
            action_max=self.get_parameter("action_max").value,
        )

        self.state_source = self.get_parameter("state_source").value
        if self.state_source not in ("gazebo_model_states", "odom"):
            raise ValueError(
                "state_source must be 'gazebo_model_states' or 'odom'"
            )
        self.state_frame = self.get_parameter("state_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.publish_gazebo_tf = self.get_parameter(
            "publish_gazebo_tf"
        ).value
        self.model_name = self.get_parameter("model_name").value
        self.sensor_timeout = self.get_parameter("sensor_timeout").value
        self.goal_tolerance = self.get_parameter("goal_tolerance").value
        self.odom_twist_in_body_frame = self.get_parameter(
            "odom_twist_in_body_frame"
        ).value

        self.position = None
        self.orientation = None
        self.linear_velocity = None
        self.angular_velocity = None
        self.twist_in_body_frame = False
        self.rays = None
        self.last_state_time = None
        self.last_scan_time = None
        self.scan_interface_logged = False
        self.goal = np.asarray(
            self.get_parameter("initial_goal").value[:2], dtype=np.float32
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.cmd_publisher = self.create_publisher(
            Twist, self.get_parameter("cmd_vel_topic").value, 1
        )
        self.raw_action_publisher = self.create_publisher(
            Float32MultiArray, "/sea_nav/raw_action", 1
        )
        self.filtered_action_publisher = self.create_publisher(
            Float32MultiArray, "/sea_nav/filtered_action", 1
        )
        self.create_subscription(
            LaserScan,
            self.get_parameter("scan_topic").value,
            self.scan_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseStamped,
            self.get_parameter("goal_topic").value,
            self.goal_callback,
            10,
        )
        if self.state_source == "gazebo_model_states":
            self.create_subscription(
                ModelStates,
                self.get_parameter("model_states_topic").value,
                self.model_states_callback,
                10,
            )
        else:
            self.create_subscription(
                Odometry,
                self.get_parameter("odom_topic").value,
                self.odom_callback,
                qos_profile_sensor_data,
            )

        control_rate = self.get_parameter("control_rate").value
        self.timer = self.create_timer(1.0 / control_rate, self.control_callback)
        self.get_logger().info(
            f"Loaded Go2 SEA-Nav ONNX policy: {model_path}; "
            f"state source: {self.state_source}"
        )

    def log_throttled(self, level, key, message, period=2.0):
        now = time.monotonic()
        if now - self.last_log_times.get(key, -period) < period:
            return
        self.last_log_times[key] = now
        # Humble's rclpy binds a logger call site to its first severity. A
        # dynamic getattr on one source line therefore crashes when a later
        # call uses another level (for example WARNING while inputs initialize,
        # followed by INFO after inference starts). Keep each severity on a
        # distinct source line so the throttling helper remains safe.
        if level == "debug":
            self.get_logger().debug(message)
        elif level == "info":
            self.get_logger().info(message)
        elif level == "warning":
            self.get_logger().warning(message)
        elif level == "error":
            self.get_logger().error(message)
        else:
            raise ValueError(f"Unsupported log level: {level}")

    def scan_callback(self, message):
        try:
            sampled = self.policy.sample_scan(
                message.ranges, message.angle_min, message.angle_increment
            )
        except ValueError as error:
            self.log_throttled("warning", "invalid_scan", str(error))
            return
        with self.lock:
            self.rays = sampled
            self.last_scan_time = time.monotonic()
        if not self.scan_interface_logged:
            self.scan_interface_logged = True
            self.get_logger().info(
                "LaserScan interface ready: frame='{}', input_bins={}, "
                "policy_rays={}".format(
                    message.header.frame_id, len(message.ranges), NUM_RAYS
                )
            )

    def model_states_callback(self, message):
        try:
            index = message.name.index(self.model_name)
        except ValueError:
            self.log_throttled(
                "warning",
                "model_missing",
                f"Model '{self.model_name}' is absent from /gazebo/model_states",
            )
            return
        pose = message.pose[index]
        self.store_state(pose, message.twist[index], False)
        if self.publish_gazebo_tf:
            self.publish_model_transform(pose)

    def publish_model_transform(self, pose):
        """Bridge Gazebo's root pose into the robot_state_publisher TF tree.

        Gazebo ModelStates is not a TF source. Without this bridge RViz can
        display links relative to base_link, but a 2D Nav Goal stamped in
        base_link cannot be transformed into the policy's world frame.
        """
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.state_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = pose.position.x
        transform.transform.translation.y = pose.position.y
        transform.transform.translation.z = pose.position.z
        transform.transform.rotation = pose.orientation
        self.tf_broadcaster.sendTransform(transform)

    def odom_callback(self, message):
        if message.header.frame_id:
            self.state_frame = message.header.frame_id
        self.store_state(
            message.pose.pose,
            message.twist.twist,
            self.odom_twist_in_body_frame,
        )

    def store_state(self, pose, twist, twist_in_body_frame):
        with self.lock:
            self.position = np.array(
                [pose.position.x, pose.position.y, pose.position.z],
                dtype=np.float32,
            )
            self.orientation = np.array(
                [
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ],
                dtype=np.float32,
            )
            self.linear_velocity = np.array(
                [twist.linear.x, twist.linear.y, twist.linear.z],
                dtype=np.float32,
            )
            self.angular_velocity = np.array(
                [twist.angular.x, twist.angular.y, twist.angular.z],
                dtype=np.float32,
            )
            self.twist_in_body_frame = twist_in_body_frame
            self.last_state_time = time.monotonic()

    def goal_callback(self, message):
        source_frame = message.header.frame_id
        point = np.array(
            [
                message.pose.position.x,
                message.pose.position.y,
                message.pose.position.z,
            ],
            dtype=np.float32,
        )
        if source_frame and source_frame != self.state_frame:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.state_frame,
                    source_frame,
                    Time(),
                    timeout=Duration(seconds=0.2),
                ).transform
            except TransformException as error:
                self.get_logger().error(
                    f"Cannot transform goal from '{source_frame}' to "
                    f"'{self.state_frame}': {error}"
                )
                return
            rotation = [
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w,
            ]
            point = rotate_vector(rotation, point) + np.array(
                [
                    transform.translation.x,
                    transform.translation.y,
                    transform.translation.z,
                ],
                dtype=np.float32,
            )

        with self.lock:
            self.goal = point[:2]
            # A new goal changes all 10 historical goal observations, so start
            # a fresh history instead of mixing two navigation tasks.
            self.policy.reset_history()
        self.get_logger().info(
            f"New goal in {self.state_frame}: x={point[0]:.3f}, y={point[1]:.3f}"
        )

    def control_callback(self):
        now = time.monotonic()
        with self.lock:
            if self.position is None or self.rays is None:
                self.log_throttled(
                    "warning",
                    "waiting_inputs",
                    "Waiting for robot state and /sea_nav/scan",
                )
                self.publish_stop()
                return
            if (
                now - self.last_state_time > self.sensor_timeout
                or now - self.last_scan_time > self.sensor_timeout
            ):
                self.log_throttled(
                    "warning",
                    "stale_inputs",
                    "Navigation input is stale; commanding stop",
                )
                self.policy.reset_history(reset_action=True)
                self.publish_stop()
                return
            position = self.position.copy()
            orientation = self.orientation.copy()
            linear_velocity = self.linear_velocity.copy()
            angular_velocity = self.angular_velocity.copy()
            rays = self.rays.copy()
            goal = self.goal.copy()
            twist_in_body_frame = self.twist_in_body_frame

        if np.linalg.norm(goal - position[:2]) <= self.goal_tolerance:
            self.policy.reset_history(reset_action=True)
            self.publish_stop()
            self.log_throttled("info", "goal_reached", "Goal reached")
            return

        observation = self.policy.build_observation(
            position,
            orientation,
            linear_velocity,
            angular_velocity,
            rays,
            goal,
            twist_in_body_frame,
        )
        try:
            raw_action, filtered_action = self.policy.infer(observation)
        except RuntimeError as error:
            self.log_throttled("error", "inference_error", str(error), 1.0)
            self.policy.reset_history(reset_action=True)
            self.publish_stop()
            return

        command = Twist()
        command.linear.x = float(filtered_action[0])
        command.linear.y = float(filtered_action[1])
        command.angular.z = float(filtered_action[2])
        self.cmd_publisher.publish(command)
        self.raw_action_publisher.publish(
            Float32MultiArray(data=raw_action.tolist())
        )
        self.filtered_action_publisher.publish(
            Float32MultiArray(data=filtered_action.tolist())
        )
        self.log_throttled(
            "info",
            "navigation_status",
            "pose=({:.2f}, {:.2f}) goal=({:.2f}, {:.2f}) "
            "cmd=({:.2f}, {:.2f}, {:.2f})".format(
                position[0],
                position[1],
                goal[0],
                goal[1],
                *filtered_action,
            ),
        )

    def publish_stop(self):
        self.cmd_publisher.publish(Twist())

    def destroy_node(self):
        # A launch SIGINT can invalidate the rclpy context before destroy_node
        # runs. Publishing after that point raises RCLError and turns a normal
        # shutdown into a process failure.
        if rclpy.ok(context=self.context):
            self.publish_stop()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SeaNavPolicyNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
