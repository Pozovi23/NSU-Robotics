from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen',
            prefix='xterm -e',  # Запуск в терминале
            parameters=[{
                'scale_linear': 0.5,
                'scale_angular': 1.0
            }],
            remappings=[('/cmd_vel', '/cmd_vel')]  # Ремап на /cmd_vel
        )
    ])