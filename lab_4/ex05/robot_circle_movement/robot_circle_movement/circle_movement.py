import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math


class TriangleMovementNode(Node):
    def __init__(self):
        super().__init__('triangle_movement_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)

        self.waypoints = [(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)]
        self.current_waypoint_idx = 0
        self.target_x = self.waypoints[0][0]
        self.target_y = self.waypoints[0][1]

        # Current pose
        self.current_x = None
        self.current_y = None
        self.current_theta = None
        self.pose_received = False

        # PID parameters
        self.Kp_linear = 0.5  # Reduced for smoother movement
        self.Ki_linear = 0.0
        self.Kd_linear = 0.1
        self.integral_linear = 0.0
        self.last_error_linear = 0.0

        self.Kp_angular = 1.0
        self.Ki_angular = 0.0
        self.Kd_angular = 0.5
        self.integral_angular = 0.0
        self.last_error_angular = 0.0

        # State machine
        self.current_state = 'rotation'
        self.tolerance = 0.05  # Distance tolerance to waypoint (meters)
        self.angular_tolerance = 0.05  # Angular tolerance (radians)

        # Control loop
        self.timer = self.create_timer(0.1, self.control_callback)

        self.get_logger().info('Triangle movement node started. Moving to first waypoint.')

    def odom_callback(self, msg):
        # Extract position
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        # Extract orientation (yaw) from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_theta = math.atan2(siny_cosp, cosy_cosp)
        self.pose_received = True

    def control_callback(self):
        if not self.pose_received:
            return

        if self.current_state == 'rotation':
            self.rotate_to_angle()
        elif self.current_state == 'movement':
            self.move_to_point()

    def rotate_to_angle(self):
        twist_msg = Twist()
        # Calculate desired angle to target
        angle = math.atan2(self.target_y - self.current_y, self.target_x - self.current_x)
        error = angle - self.current_theta
        # Normalize angle to [-pi, pi]
        error = math.atan2(math.sin(error), math.cos(error))

        if abs(error) <= self.angular_tolerance:
            self.current_state = 'movement'
            twist_msg.angular.z = 0.0
            self.publisher_.publish(twist_msg)
            self.integral_angular = 0.0
            self.last_error_angular = 0.0
            return

        # PID control for angular velocity
        self.integral_angular += error
        derivative = error - self.last_error_angular
        self.last_error_angular = error
        correction = (self.Kp_angular * error +
                      self.Ki_angular * self.integral_angular +
                      self.Kd_angular * derivative)
        correction = max(min(correction, 1.0), -1.0)  # Limit angular velocity
        twist_msg.angular.z = correction
        self.publisher_.publish(twist_msg)

    def move_to_point(self):
        twist_msg = Twist()
        # Calculate distance to target
        distance = math.sqrt((self.target_x - self.current_x) ** 2 +
                             (self.target_y - self.current_y) ** 2)

        if distance <= self.tolerance:
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.publisher_.publish(twist_msg)
            self.integral_linear = 0.0
            self.last_error_linear = 0.0
            self.current_waypoint_idx = (self.current_waypoint_idx + 1) % len(self.waypoints)
            self.target_x = self.waypoints[self.current_waypoint_idx][0]
            self.target_y = self.waypoints[self.current_waypoint_idx][1]
            self.current_state = 'rotation'
            self.get_logger().info(f'Reached waypoint. Moving to next: ({self.target_x}, {self.target_y})')
            return

        # PID control for linear velocity
        self.integral_linear += distance
        derivative = distance - self.last_error_linear
        self.last_error_linear = distance
        linear_speed = (self.Kp_linear * distance +
                        self.Ki_linear * self.integral_linear +
                        self.Kd_linear * derivative)
        linear_speed = max(min(linear_speed, 0.5), -0.5)  # Limit linear velocity

        angle = math.atan2(self.target_y - self.current_y, self.target_x - self.current_x)
        error = angle - self.current_theta
        error = math.atan2(math.sin(error), math.cos(error))
        angular_correction = self.Kp_angular * error
        angular_correction = max(min(angular_correction, 0.5), -0.5)

        twist_msg.linear.x = linear_speed
        twist_msg.angular.z = angular_correction
        self.publisher_.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TriangleMovementNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()