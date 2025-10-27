import math
import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import std_msgs.msg


class TurtleController(Node):
    def __init__(self):
        super().__init__("turtle_controller")

        self.declare_parameter("switch_threshold", 0.8)
        self.switch_threshold = self.get_parameter("switch_threshold").value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.current_target = "carrot1"
        self.targets = ["carrot1", "carrot2", "static_target"]

        self.last_switch_time = self.get_clock().now()
        self.target_reached_count = 0

        self.publisher = self.create_publisher(Twist, "turtle2/cmd_vel", 1)

        self.create_subscription(
            std_msgs.msg.String,
            "/keyboard_input",
            self.keyboard_callback,
            10
        )

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Turtle controller started")
        self.get_logger().info(f"Initial target: {self.current_target}")
        self.get_logger().info(f"Switch threshold: {self.switch_threshold}")

    def switch_to_next_target(self):
        current_index = self.targets.index(self.current_target)
        next_index = (current_index + 1) % len(self.targets)
        self.current_target = self.targets[next_index]
        self.last_switch_time = self.get_clock().now()
        self.target_reached_count = 0
        self.get_logger().info(f"Switched to target: {self.current_target}")

    def keyboard_callback(self, msg):
        if msg.data == 'n':
            self.switch_to_next_target()

    def control_loop(self):
        try:
            t = self.tf_buffer.lookup_transform(
                "turtle2",
                self.current_target,
                rclpy.time.Time()
            )

            distance = math.sqrt(
                t.transform.translation.x ** 2 +
                t.transform.translation.y ** 2
            )

            target_angle = math.atan2(t.transform.translation.y, t.transform.translation.x)

            msg = Twist()

            angle_tolerance = 0.2  # радиан
            if abs(target_angle) > angle_tolerance:
                msg.angular.z = 3.0 * math.copysign(1.0, target_angle)
            else:
                msg.angular.z = 0.5 * target_angle

            if distance > 1.0:
                msg.linear.x = min(2.0, distance * 1.0)
            elif distance > 0.3:
                msg.linear.x = min(1.0, distance * 1.5)
            else:
                msg.linear.x = 0.3

            self.publisher.publish(msg)

            current_time = self.get_clock().now()
            time_since_switch = (current_time - self.last_switch_time).nanoseconds / 1e9

            if distance < self.switch_threshold:
                self.target_reached_count += 1
            else:
                self.target_reached_count = 0

            if (self.target_reached_count > 5 and
                    time_since_switch > 3.0):

                self.get_logger().info(
                    f"Target {self.current_target} reached, switching to next. Distance: {distance:.2f}")
                self.switch_to_next_target()

            if int(current_time.nanoseconds / 1e9) % 2 == 0:
                self.get_logger().info(
                    f"Target: {self.current_target}, Distance: {distance:.2f}, Linear: {msg.linear.x:.2f}, Angular: {msg.angular.z:.2f}")

        except TransformException as ex:
            self.get_logger().warn(f"Transform exception: {ex}")
            current_time = self.get_clock().now()
            time_since_switch = (current_time - self.last_switch_time).nanoseconds / 1e9
            if time_since_switch > 5.0:
                self.get_logger().warn(f"Cannot reach {self.current_target}, switching to next target")
                self.switch_to_next_target()


def main():
    rclpy.init()
    try:
        node = TurtleController()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.shutdown()