from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "radius",
            default_value="2.0",
            description="Radius of carrot rotation around turtle1",
        ),
        DeclareLaunchArgument(
            "direction_of_rotation",
            default_value="1",
            description="Direction of rotation: 1 for clockwise, -1 for counterclockwise",
        ),
        DeclareLaunchArgument(
            "switch_threshold",
            default_value="0.8",
            description="Distance threshold for automatic target switching",
        ),

        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim',
            output='screen'
        ),

        TimerAction(
            period=2.0,
            actions=[
                # Спавним turtle2
                ExecuteProcess(
                    cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn',
                         '{x: 4.0, y: 2.0, theta: 0.0, name: "turtle2"}'],
                    output='screen'
                ),
            ]
        ),

        TimerAction(
            period=3.0,
            actions=
                ExecuteProcess(
                    cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn',
                         '{x: 1.0, y: 1.0, theta: 0.0, name: "turtle3"}'],
                    output='screen'
                ),
            ]
        ),

        Node(
            package='turtle_multi_target',
            executable='turtle_tf2_broadcaster',
            name='broadcaster1',
            parameters=[{'turtlename': 'turtle1'}]
        ),

        Node(
            package='turtle_multi_target',
            executable='turtle_tf2_broadcaster',
            name='broadcaster2',
            parameters=[{'turtlename': 'turtle2'}]
        ),

        Node(
            package='turtle_multi_target',
            executable='turtle_tf2_broadcaster',
            name='broadcaster3',
            parameters=[{'turtlename': 'turtle3'}]
        ),

        Node(
            package='turtle_multi_target',
            executable='multi_target_broadcaster',
            name='multi_target_broadcaster',
            parameters=[
                {
                    "radius": LaunchConfiguration("radius"),
                    "direction_of_rotation": LaunchConfiguration("direction_of_rotation"),
                }
            ]
        ),

        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='turtle_multi_target',
                    executable='turtle_controller',
                    name='turtle_controller',
                    parameters=[{"switch_threshold": LaunchConfiguration("switch_threshold")}]
                ),
            ]
        ),

        Node(
            package='turtle_multi_target',
            executable='target_switcher',
            name='target_switcher'
        ),

        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=[
                        '-d', PathJoinSubstitution([
                            FindPackageShare('turtle_multi_target'),
                            'rviz',
                            'carrot.rviz'
                        ])
                    ],
                    condition=launch.conditions.IfCondition("true"),
                    output='screen'
                )
            ]
        ),
    ])