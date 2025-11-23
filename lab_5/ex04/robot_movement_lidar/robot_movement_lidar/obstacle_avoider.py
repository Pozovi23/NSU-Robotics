import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)
        self.threshold_distance = 0.5
        self.linear_speed = 0.2

    def lidar_callback(self, msg):
        front_ranges = msg.ranges[len(msg.ranges)//2 - 30: len(msg.ranges)//2 + 30]
        min_distance = min(front_ranges) if front_ranges else float('inf')

        twist = Twist()
        if min_distance < self.threshold_distance:
            twist.linear.x = 0.0
        else:
            twist.linear.x = self.linear_speed

        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()