import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class DynamicFrameBroadcaster(Node):

    def __init__(self):
        super().__init__("dynamic_frame_tf2_broadcaster")
        self.declare_parameter("radius", 2.0)
        self.declare_parameter("direction_of_rotation", 1)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]

        self.timer = self.create_timer(0.1, self.broadcast_timer_callback)

    def broadcast_timer_callback(self):
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        elapsed_time = current_time - self.start_time
        radius = self.get_parameter("radius").get_parameter_value().double_value
        direction = (
            self.get_parameter("direction_of_rotation")
            .get_parameter_value()
            .integer_value
        )

        angle = elapsed_time * 0.3 * direction

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "turtle1"
        t.child_frame_id = "carrot1"
        t.transform.translation.x = radius * math.sin(angle)
        t.transform.translation.y = radius * math.cos(angle)
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    try:
        node = DynamicFrameBroadcaster()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.shutdown()
