# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from std_msgs.msg import String


class TextToCmdVel(Node):

    def __init__(self):
        super().__init__('text_to_cmd_vel')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.subscription = self.create_subscription(
            String,
            'cmd_text',
            self.listener_callback,
            10)

        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        twist_msg = Twist()

        command = msg.data.lower().strip()

        if command == 'turn_right':
            twist_msg.angular.z = -1.5

        elif command == 'turn_left':
            twist_msg.angular.z = 1.5

        elif command == 'move_forward':
            twist_msg.linear.x = 1.0

        elif command == 'move_backward':
            twist_msg.linear.x = -1.0

        else:
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0

        self.publisher_.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)

    text_to_cmd_vel = TextToCmdVel()

    rclpy.spin(text_to_cmd_vel)

    text_to_cmd_vel.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
