"""Activate rl_sar and enable its /cmd_vel navigation mode."""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rl_sar.action import RobotActivation, SetNavigationMode


class RlSarAutostart(Node):
    def __init__(self):
        super().__init__("rl_sar_autostart")
        self.activation_client = ActionClient(
            self, RobotActivation, "/robot_activation"
        )
        self.navigation_client = ActionClient(
            self, SetNavigationMode, "/set_navigation_mode"
        )

    def send_goal(self, client, goal, name):
        self.get_logger().info(f"Waiting for {name} action server")
        while rclpy.ok() and not client.wait_for_server(timeout_sec=1.0):
            pass
        if not rclpy.ok():
            return False

        goal_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_future)
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"{name} goal was rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        if not result.success:
            self.get_logger().error(f"{name} failed: {result.message}")
            return False
        self.get_logger().info(f"{name}: {result.message}")
        return True

    def run(self):
        activation = RobotActivation.Goal()
        activation.activate = True
        if not self.send_goal(
            self.activation_client, activation, "robot activation"
        ):
            return False

        navigation = SetNavigationMode.Goal()
        navigation.enable = True
        return self.send_goal(
            self.navigation_client, navigation, "navigation mode"
        )


def main(args=None):
    rclpy.init(args=args)
    node = RlSarAutostart()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
