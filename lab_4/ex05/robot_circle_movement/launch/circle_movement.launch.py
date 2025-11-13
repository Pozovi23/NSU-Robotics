from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('robot_bringup'),
                'launch',
                'bringup.launch.py'
            ])
        )
    )

    circle_node = Node(
        package='robot_circle_movement',
        executable='circle_movement',
        name='circle_movement_node',
        output='screen'
    )

    return LaunchDescription([
        bringup_launch,
        circle_node
    ])