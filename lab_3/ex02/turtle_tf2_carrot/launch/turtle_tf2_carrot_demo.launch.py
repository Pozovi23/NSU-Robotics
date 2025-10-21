from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
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
            Node(package="turtlesim", executable="turtlesim_node", name="sim"),
            Node(
                package="turtle_tf2_carrot",
                executable="turtle_tf2_broadcaster",
                name="broadcaster1",
                parameters=[{"turtlename": "turtle1"}],
            ),
            Node(
                package="turtle_tf2_carrot",
                executable="turtle_tf2_broadcaster",
                name="broadcaster2",
                parameters=[{"turtlename": "turtle2"}],
            ),
            Node(
                package="turtle_tf2_carrot",
                executable="dynamic_frame_tf2_broadcaster",
                name="dynamic_broadcaster",
                parameters=[
                    {
                        "radius": LaunchConfiguration("radius"),
                        "direction_of_rotation": LaunchConfiguration(
                            "direction_of_rotation"
                        ),
                    }
                ],
            ),
            Node(
                package="turtle_tf2_carrot",
                executable="turtle_tf2_listener",
                name="listener",
                parameters=[{"target_frame": "carrot1"}],
            ),
        ]
    )
