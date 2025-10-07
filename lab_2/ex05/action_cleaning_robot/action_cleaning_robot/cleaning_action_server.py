import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
import time
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
        self.control_timer = None

        self.publisher_cmd_vel = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10)

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

    def move_to_position(self, target_x, target_y, final_angle, goal_handle):
        try:
            move_node = MoveToGoal(target_x, target_y, final_angle, goal_handle)

            start_time = time.time()
            while not move_node.pose_received and rclpy.ok():
                if time.time() - start_time > 5.0:
                    self.get_logger().error('Timeout waiting for MoveToGoal pose')
                    move_node.destroy_node()
                    return False
                rclpy.spin_once(move_node, timeout_sec=0.1)

            start_time = time.time()
            while (move_node.current_state != "done" and
                   time.time() - start_time < 30.0 and
                   rclpy.ok()):
                rclpy.spin_once(move_node, timeout_sec=0.1)
                time.sleep(0.01)

            success = move_node.current_state == "done"
            move_node.destroy_node()
            return success

        except Exception as e:
            self.get_logger().error(f'Error in move_to_position: {e}')
            return False

    def wait_for_pose(self):
        self.get_logger().info('Waiting for pose information...')
        while not self.pose_received and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def return_home(self, goal_handle):
        self.get_logger().info(f'Returning home to ({self.target_x}, {self.target_y})')

        # Получаем текущую позицию
        current_x = self.current_pose.x
        current_y = self.current_pose.y

        # Рассчитываем расстояние до цели
        initial_distance = math.sqrt((self.target_x - current_x) ** 2 + (self.target_y - current_y) ** 2)

        # Рассчитываем угол для движения
        target_angle = math.atan2(self.target_y - current_y, self.target_x - current_x)

        # Отправляем feedback о начале движения
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
        # Получаем финальную позицию для расчета пройденного расстояния
        if self.pose_received and success:
            final_x = self.current_pose.x
            final_y = self.current_pose.y
            actual_distance = math.sqrt((final_x - current_x) ** 2 + (final_y - current_y) ** 2)
        else:
            actual_distance = 0.0

        # Отправляем финальный feedback
        feedback_msg.progress_percent = 100
        feedback_msg.current_x = self.target_x if success else current_x
        feedback_msg.current_y = self.target_y if success else current_y
        goal_handle.publish_feedback(feedback_msg)

        return success, 0, actual_distance

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        self.task_type = goal_handle.request.task_type
        self.area_size = goal_handle.request.area_size
        self.target_x = goal_handle.request.target_x
        self.target_y = goal_handle.request.target_y


        if not (0 <= self.target_x <= 11.0) or not (0 <= self.target_y <= 11.0):
            self.get_logger().error(f'Invalid target coordinates: ({self.target_x}, {self.target_y})')
            result = CleaningTask.Result()
            result.success = False
            result.cleaned_points = 0
            result.total_distance = 0
            return result

        start_time = time.time()
        while not self.pose_received and rclpy.ok():
            if time.time() - start_time > 10.0:  # Таймаут 10 секунд
                self.get_logger().error('Timeout waiting for pose information')
                result = CleaningTask.Result()
                result.success = False
                result.cleaned_points = 0
                result.total_distance = 0
                goal_handle.abort()
                return result
            rclpy.spin_once(self, timeout_sec=0.1)  # Обрабатываем события ROS

        if self.task_type == "return_home":
            success, cleaned_points, total_distance = self.return_home(goal_handle)
        elif self.task_type == "clean_square":
            success, cleaned_points, total_distance = self.clean_square(goal_handle)
        else:
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


def main(args=None):
    rclpy.init(args=args)

    cleaning_action_server = CLeaningActionServer()

    rclpy.spin(cleaning_action_server)


if __name__ == '__main__':
    main()