import rclpy
from rclpy.node import Node
import sys
import select
import tty
import termios
from std_msgs.msg import String


class KeyboardListener(Node):
    def __init__(self):
        super().__init__('keyboard_listener')
        self.publisher = self.create_publisher(String, '/keyboard_input', 10)

    def read_key(self):
        try:
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(sys.stdin.fileno())
                [i, _, _] = select.select([sys.stdin], [], [], 0.1)
                if i:
                    key = sys.stdin.read(1)
                    return key
                return None
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        except Exception:
            return None

    def spin(self):
        try:
            while rclpy.ok():
                key = self.read_key()
                if key == 'n':
                    msg = String()
                    msg.data = 'n'
                    self.publisher.publish(msg)
                elif key == 'q':
                    break
        except Exception as e:
            self.get_logger().error(f"Error: {e}")


def main():
    rclpy.init()
    node = KeyboardListener()

    try:
        node.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()