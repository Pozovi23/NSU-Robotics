import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CircleMovementNode(Node):
    def __init__(self):
        super().__init__('circle_movement_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Circle movement node started. Publishing to /cmd_vel.')

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.2  # Linear velocity for forward movement
        msg.angular.z = 0.5  # Angular velocity for turning (positive for counterclockwise circle)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CircleMovementNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()