import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_cleaning_robot_interfaces.action import CleaningTask


class CLeaningActionClient(Node):

    def __init__(self):
        super().__init__('cleaning_action_client')
        self._action_client = ActionClient(self, CleaningTask, 'CleaningTask')
        self._current_goal_handle = None
        self._goals = []
        self._current_goal_index = 0

    def add_goal(self, task_type, area_size, target_x, target_y):
        self._goals.append({
            'task_type': task_type,
            'area_size': float(area_size),
            'target_x': float(target_x),
            'target_y': float(target_y)
        })

    def send_next_goal(self):
        if self._current_goal_index >= len(self._goals):
            self.get_logger().info('All goals completed!')
            rclpy.shutdown()
            return

        goal_data = self._goals[self._current_goal_index]
        self._current_goal_index += 1

        self.get_logger().info(f'Sending goal {self._current_goal_index}/{len(self._goals)}: {goal_data["task_type"]}')

        goal_msg = CleaningTask.Goal()
        goal_msg.task_type = goal_data['task_type']
        goal_msg.area_size = goal_data['area_size']
        goal_msg.target_x = goal_data['target_x']
        goal_msg.target_y = goal_data['target_y']

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self.send_next_goal()
            return

        self.get_logger().info('Goal accepted :)')
        self._current_goal_handle = goal_handle

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(
            f'Result: success={result.success}, cleaned_points={result.cleaned_points}, total_distance={result.total_distance:.2f}')
        self.send_next_goal()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Progress: {feedback.progress_percent}%, cleaned: {feedback.current_cleaned_points}, position: ({feedback.current_x:.2f}, {feedback.current_y:.2f})')

    def start_execution(self):
        if not self._goals:
            self.get_logger().warning('No goals to execute!')
            return
        self.send_next_goal()


def main(args=None):
    rclpy.init(args=args)

    action_client = CLeaningActionClient()

    action_client.add_goal("clean_square", 5.0, 2.0, 2.0)
    action_client.add_goal("return_home", 0.0, 2.0, 8.0)

    action_client.start_execution()

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()