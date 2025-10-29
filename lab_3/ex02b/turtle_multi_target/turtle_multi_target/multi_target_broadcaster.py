import math
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class MultiTargetBroadcaster(Node):
    def __init__(self):
        super().__init__("multi_target_broadcaster")

        self.declare_parameter("radius", 2.0)
        self.declare_parameter("direction_of_rotation", 1)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.timer = self.create_timer(0.1, self.broadcast_timer_callback)

        self.get_logger().info("Multi target broadcaster started")

    def broadcast_timer_callback(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed_time = current_time - self.start_time
        radius = self.get_parameter("radius").get_parameter_value().double_value
        direction = (
            self.get_parameter("direction_of_rotation")
            .get_parameter_value()
            .integer_value
        )

        angle1 = elapsed_time * 0.8 * direction
        angle2 = elapsed_time * 0.5 * direction

        t1 = TransformStamped()
        t1.header.stamp = self.get_clock().now().to_msg()
        t1.header.frame_id = "turtle1"
        t1.child_frame_id = "carrot1"
        t1.transform.translation.x = radius * math.sin(angle1)
        t1.transform.translation.y = radius * math.cos(angle1)
        t1.transform.translation.z = 0.0
        t1.transform.rotation.x = 0.0
        t1.transform.rotation.y = 0.0
        t1.transform.rotation.z = 0.0
        t1.transform.rotation.w = 1.0

        t2 = TransformStamped()
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = "turtle3"
        t2.child_frame_id = "carrot2"
        t2.transform.translation.x = 1.5 * math.sin(angle2)
        t2.transform.translation.y = 1.5 * math.cos(angle2)
        t2.transform.translation.z = 0.0
        t2.transform.rotation.x = 0.0
        t2.transform.rotation.y = 0.0
        t2.transform.rotation.z = 0.0
        t2.transform.rotation.w = 1.0

        t3 = TransformStamped()
        t3.header.stamp = self.get_clock().now().to_msg()
        t3.header.frame_id = "world"
        t3.child_frame_id = "static_target"
        t3.transform.translation.x = 8.0
        t3.transform.translation.y = 2.0
        t3.transform.translation.z = 0.0
        t3.transform.rotation.x = 0.0
        t3.transform.rotation.y = 0.0
        t3.transform.rotation.z = 0.0
        t3.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t1)
        self.tf_broadcaster.sendTransform(t2)
        self.tf_broadcaster.sendTransform(t3)


def main():
    rclpy.init()
    try:
        node = MultiTargetBroadcaster()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.shutdown()