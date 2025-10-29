from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'delay',
            default_value='5.0',
            description='Time delay in seconds for time travel'
        ),

        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Whether to launch RViz'
        ),

        DeclareLaunchArgument(
            'rvizconfig',
            default_value='share/time_travel/rviz/time_travel.rviz',
            description='Path to RViz config file'
        ),

        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),

        Node(
            package='time_travel',
            executable='turtle_tf2_broadcaster',
            name='broadcaster1',
            parameters=[
                {'turtlename': 'turtle1'}
            ]
        ),

        Node(
            package='time_travel',
            executable='turtle_tf2_broadcaster',
            name='broadcaster2',
            parameters=[
                {'turtlename': 'turtle2'}
            ]
        ),

        Node(
            package='time_travel',
            executable='turtle_tf2_time_travel_listener',
            name='listener',
            parameters=[
                {'target_frame': 'turtle1'},
                {'delay': LaunchConfiguration('delay')}
            ]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('rvizconfig')],
            condition=IfCondition(LaunchConfiguration('use_rviz'))
        ),
    ])