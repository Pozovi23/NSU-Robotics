import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
import math

from action_cleaning_robot_interfaces.action import CleaningTask
from action_cleaning_robot.move_to_goal import MoveToGoal

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class CLeaningActionServer(Node):

    def __init__(self):
        super().__init__('cleaning_action_server')
        self.task_type = None
        self.area_size = None
        self.target_x = None
        self.target_y = None

        self.current_pose = None
        self.pose_received = False

        self.publisher_cmd_vel = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10)

        self.move_controller = MoveToGoal()

        self._action_server = ActionServer(
            self,
            CleaningTask,
            'CleaningTask',
            self.execute_callback)

        self.pose_check_timer = self.create_timer(1.0, self.check_pose_status)

    def pose_callback(self, msg):
        self.current_pose = msg
        if not self.pose_received:
            self.pose_received = True
            self.get_logger().info('Pose information received')

    def check_pose_status(self):
        if not self.pose_received:
            self.get_logger().warn('Waiting for pose information from /turtle1/pose...')

    def wait_for_pose(self):
        self.get_logger().info('Waiting for pose information...')
        while not self.pose_received and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def move_to_position(self, target_x, target_y, final_angle, goal_handle):
        try:
            if not self.move_controller.set_goal(target_x, target_y, final_angle, goal_handle):
                return False

            while not self.move_controller.is_done() and rclpy.ok():
                rclpy.spin_once(self.move_controller, timeout_sec=0.01)
                rclpy.spin_once(self, timeout_sec=0.01)

            return True

        except Exception as e:
            self.get_logger().error(f'Error in move_to_position: {e}')
            self.move_controller.reset()
            return False

    def return_home(self, goal_handle):
        self.get_logger().info(f'Returning home to ({self.target_x}, {self.target_y})')

        current_x = self.current_pose.x
        current_y = self.current_pose.y
        target_angle = math.atan2(self.target_y - current_y, self.target_x - current_x)

        feedback_msg = CleaningTask.Feedback()
        feedback_msg.progress_percent = 0
        feedback_msg.current_cleaned_points = 0
        feedback_msg.current_x = current_x
        feedback_msg.current_y = current_y
        goal_handle.publish_feedback(feedback_msg)

        try:
            success = self.move_to_position(self.target_x, self.target_y, target_angle, goal_handle)
        except Exception as e:
            self.get_logger().error(f'Error during movement: {e}')
            success = False

        if self.pose_received and success:
            final_x = self.current_pose.x
            final_y = self.current_pose.y
            actual_distance = math.sqrt((final_x - current_x) ** 2 + (final_y - current_y) ** 2)
        else:
            actual_distance = 0.0

        feedback_msg.progress_percent = 100
        feedback_msg.current_x = self.target_x if success else current_x
        feedback_msg.current_y = self.target_y if success else current_y
        goal_handle.publish_feedback(feedback_msg)

        return success, 0, actual_distance

    def clean_square(self, goal_handle):
        self.get_logger().info(f'Cleaning square area of size {self.area_size}')

        start_x = self.current_pose.x
        start_y = self.current_pose.y

        half_size = self.area_size / 2.0
        left = max(0.0, start_x - half_size)
        right = min(11.0, start_x + half_size)
        bottom = max(0.0, start_y - half_size)
        top = min(11.0, start_y + half_size)

        strip_width = 0.25
        num_strips = max(1, int((right - left) / strip_width))

        cleaned_points = 0
        total_distance = 0.0
        current_x = start_x
        current_y = start_y

        feedback_msg = CleaningTask.Feedback()
        feedback_msg.progress_percent = 0
        feedback_msg.current_cleaned_points = 0
        feedback_msg.current_x = current_x
        feedback_msg.current_y = current_y
        goal_handle.publish_feedback(feedback_msg)

        try:
            success = self.move_to_position(left, bottom, math.pi / 2, None)
            if not success:
                return False, 0, 0.0

            current_x = left
            current_y = bottom
            initial_cleaned_points = int(math.sqrt((left - start_x) ** 2 + (bottom - start_y) ** 2) / 0.1)
            cleaned_points += initial_cleaned_points

            for strip in range(num_strips):
                if goal_handle.is_cancel_requested:
                    return False, cleaned_points, total_distance

                target_x = left + strip * strip_width
                target_y = top
                success = self.move_to_position(target_x, target_y, math.pi / 2, None)
                if success:
                    distance = math.sqrt((target_x - current_x) ** 2 + (target_y - current_y) ** 2)
                    total_distance += distance
                    points_in_strip = max(1, int(distance / 0.1))
                    cleaned_points += points_in_strip
                    current_x, current_y = target_x, target_y

                if strip < num_strips - 1:
                    target_x = left + (strip + 1) * strip_width
                    target_y = top
                    success = self.move_to_position(target_x, target_y, 0, None)
                    if success:
                        distance = math.sqrt((target_x - current_x) ** 2 + (target_y - current_y) ** 2)
                        total_distance += distance
                        points_in_strip = max(1, int(distance / 0.1))
                        cleaned_points += points_in_strip
                        current_x, current_y = target_x, target_y

                    target_x = left + (strip + 1) * strip_width
                    target_y = bottom
                    success = self.move_to_position(target_x, target_y, -math.pi / 2, None)
                    if success:
                        distance = math.sqrt((target_x - current_x) ** 2 + (target_y - current_y) ** 2)
                        total_distance += distance
                        points_in_strip = max(1, int(distance / 0.1))
                        cleaned_points += points_in_strip
                        current_x, current_y = target_x, target_y

                progress = min(100, int((strip + 1) / num_strips * 100))
                feedback_msg.progress_percent = progress
                feedback_msg.current_cleaned_points = cleaned_points
                feedback_msg.current_x = current_x
                feedback_msg.current_y = current_y
                goal_handle.publish_feedback(feedback_msg)

                self.get_logger().info(f'Strip {strip + 1}/{num_strips} completed, cleaned {cleaned_points} points')

            self.get_logger().info(f'Square cleaning completed: {cleaned_points} points, {total_distance:.2f} meters')
            return True, cleaned_points, total_distance

        except Exception as e:
            self.get_logger().error(f'Error during square cleaning: {str(e)}')
            return False, cleaned_points, total_distance

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        self.task_type = goal_handle.request.task_type
        self.area_size = goal_handle.request.area_size
        self.target_x = goal_handle.request.target_x
        self.target_y = goal_handle.request.target_y

        if not (0 <= self.target_x <= 11.0) or not (0 <= self.target_y <= 11.0):
            self.get_logger().error(f'Invalid target coordinates')
            result = CleaningTask.Result()
            result.success = False
            result.cleaned_points = 0
            result.total_distance = 0
            return result

        while not self.pose_received and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.task_type == "return_home":
            success, cleaned_points, total_distance = self.return_home(goal_handle)
        elif self.task_type == "clean_square":
            success, cleaned_points, total_distance = self.clean_square(goal_handle)
        else:
            self.get_logger().error(f'Unknown task type: {self.task_type}')
            success, cleaned_points, total_distance = False, 0, 0

        result = CleaningTask.Result()
        result.success = success
        result.cleaned_points = cleaned_points
        result.total_distance = total_distance

        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return result

    def destroy_node(self):
        if hasattr(self, 'move_controller'):
            self.move_controller.destroy_node()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    cleaning_action_server = CLeaningActionServer()

    try:
        rclpy.spin(cleaning_action_server)
    except KeyboardInterrupt:
        pass
    finally:
        cleaning_action_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()