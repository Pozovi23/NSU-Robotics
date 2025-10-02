from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='service_full_name',
            executable='service_name',
            name='full_name_service'
        ),
    ])