import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import std_msgs.msg


class TargetSwitcher(Node):
    def __init__(self):
        super().__init__("target_switcher")

        self.create_subscription(
            std_msgs.msg.String,
            "/keyboard_input",
            self.keyboard_callback,
            10
        )

    def keyboard_callback(self, msg):
        if msg.data == 'n':
            self.get_logger().info("Manual switch requested - sent to controller")


def main():
    rclpy.init()
    try:
        node = TargetSwitcher()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.shutdown()