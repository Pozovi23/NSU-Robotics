import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from action_cleaning_robot_interfaces.action import CleaningTask

class MoveToGoal(Node):

    def __init__(self, target_x, target_y, final_angle, goal_handle):
        super().__init__('move_to_goal')
        self.publisher_cmd_vel = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10)

        self.current_x = None
        self.current_y = None
        self.current_theta = None

        self.target_x = float(target_x)
        self.target_y = float(target_y)
        self.final_angle = float(final_angle)
        self.goal_handle = goal_handle

        self.pose_received = False

        self.start_x = None
        self.start_y = None
        self.total_distance = 0.0
        self.last_progress_percent = 0

        if not (0 <= self.target_x <= 11.0) or not (0 <= self.target_y <= 11.0):
            print("wrong target position")
            return

        self.Kp_angular = 1.0
        self.Ki_angular = 0.0
        self.Kd_angular = 6.0
        self._integral_angular = 0
        self._last_error_angular = 0

        self.Kp_linear = 2.0
        self.Ki_linear = 0.0
        self.Kd_linear = 0.0
        self._integral_linear = 0
        self._last_error_linear = 0

        self.control_timer = None

        self.current_state = "rotation"

        self.timer = self.create_timer(0.1, self.timer_callback)

    def pose_callback(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_theta = msg.theta

        if not self.pose_received:
            self.start_x = self.current_x
            self.start_y = self.current_y

        self.pose_received = True

    def timer_callback(self):
        if self.pose_received:
            self.timer.destroy()
            self.main_loop()

    def update_progress(self):
        if self.start_x is None or self.start_y is None:
            return

        current_distance = math.sqrt((self.current_x - self.start_x) ** 2 +
                                     (self.current_y - self.start_y) ** 2)

        total_required_distance = math.sqrt((self.target_x - self.start_x) ** 2 +
                                            (self.target_y - self.start_y) ** 2)

        if total_required_distance > 0:
            progress_percent = min(int((current_distance / total_required_distance) * 100), 100)

            if progress_percent >= self.last_progress_percent + 10 or progress_percent == 100:
                self.last_progress_percent = progress_percent

                if self.goal_handle is not None:
                    feedback_msg = CleaningTask.Feedback()
                    feedback_msg.progress_percent = progress_percent
                    feedback_msg.current_cleaned_points = 0
                    feedback_msg.current_x = self.current_x
                    feedback_msg.current_y = self.current_y
                    self.goal_handle.publish_feedback(feedback_msg)

    def control_callback(self):
        if self.current_state == "rotation":
            self.rotate_to_angle()
        elif self.current_state == "movement":
            self.update_progress()
            self.move_to_point()
        elif self.current_state == "final_rotation":
            self.rotate_to_final_angle()

    def rotate_to_angle(self):
        twist_msg = Twist()
        angle = math.atan2(self.target_y - self.current_y, self.target_x - self.current_x)
        error = angle - self.current_theta

        if abs(error) <= 0.00001:
            self.current_state = "movement"
            twist_msg.angular.z = 0.0
            self.publisher_cmd_vel.publish(twist_msg)
            self._integral_angular = 0
            self._last_error_angular = 0
            return

        self._integral_angular += error
        derivative = error - self._last_error_angular
        self._last_error_angular = error

        correction = self.Kp_angular * error + self.Ki_angular * self._integral_angular + self.Kd_angular * derivative
        correction = max(min(correction, 1.0), -1.0)
        twist_msg.angular.z = correction
        self.publisher_cmd_vel.publish(twist_msg)


    def move_to_point(self):
        twist_msg = Twist()

        distance = math.sqrt((self.target_x - self.current_x) ** 2 +
                             (self.target_y - self.current_y) ** 2)

        if distance <= 0.001:
            self.current_state = "final_rotation"
            twist_msg.linear.x = 0.0
            self.publisher_cmd_vel.publish(twist_msg)
            self._integral_linear = 0
            self._last_error_linear = 0
            return

        self._integral_linear += distance
        derivative_linear = distance - self._last_error_linear
        self._last_error_linear = distance

        linear_speed = self.Kp_linear * distance + self.Ki_linear * self._integral_linear + self.Kd_linear * derivative_linear
        linear_speed = max(min(linear_speed, 1.0), -1.0)

        twist_msg.linear.x = linear_speed
        self.publisher_cmd_vel.publish(twist_msg)

    def rotate_to_final_angle(self):
        twist_msg = Twist()
        error = self.final_angle - self.current_theta

        if abs(error) <= 0.00001:
            twist_msg.angular.z = 0.0
            self.publisher_cmd_vel.publish(twist_msg)
            self._integral_angular = 0
            self._last_error_angular = 0

            if self.control_timer:
                self.control_timer.destroy()
            self.current_state = "done"
            print("done")
            return

        self._integral_angular += error
        derivative = error - self._last_error_angular
        self._last_error_angular = error

        correction = self.Kp_angular * error + self.Ki_angular * self._integral_angular + self.Kd_angular * derivative
        correction = max(min(correction, 1.0), -1.0)
        twist_msg.angular.z = correction
        self.publisher_cmd_vel.publish(twist_msg)

    def main_loop(self):
        self.control_timer = self.create_timer(0.01, self.control_callback)

# def main():
#     rclpy.init()
#     move_to_goal = MoveToGoal()
#     rclpy.spin(move_to_goal)
#     self.destroy_node()
#     rclpy.shutdown()
#
#
# if __name__ == '__main__':
#     main()