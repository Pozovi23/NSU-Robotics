from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue  # ← ДОБАВИТЬ!
from launch.conditions import IfCondition, UnlessCondition
import os

def generate_launch_description():
    # Пакеты
    pkg_robot_description = FindPackageShare(package='robot_description').find('robot_description')
    pkg_robot_gazebo = FindPackageShare(package='robot_gazebo').find('robot_gazebo')
    pkg_robot_bringup = FindPackageShare(package='robot_bringup').find('robot_bringup')

    # Пути
    default_model_path = os.path.join(pkg_robot_description, 'urdf/robot.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_robot_bringup, 'rviz/robot.rviz')

    # Аргументы
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')

    # Include Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PathJoinSubstitution([pkg_robot_gazebo, 'launch', 'robot_gazebo.launch.py']),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Robot description — КРИТИЧНО: оборачиваем в ParameterValue
    robot_description_content = Command([
        'xacro', ' ', default_model_path, ' sim:=true'
    ])
    robot_description_param = ParameterValue(robot_description_content, value_type=str)

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_param,
            'use_sim_time': use_sim_time
        }]
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock'),
        DeclareLaunchArgument('gui', default_value='true', description='Enable joint_state_publisher_gui'),
        gazebo_launch,
        robot_state_publisher_node,
        rviz_node
    ])