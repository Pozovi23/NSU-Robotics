from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
import os

def generate_launch_description():
    # Пакеты
    pkg_ros_gz_sim = FindPackageShare(package='ros_gz_sim').find('ros_gz_sim')
    pkg_robot_description = FindPackageShare(package='robot_description').find('robot_description')
    pkg_robot_gazebo = FindPackageShare(package='robot_gazebo').find('robot_gazebo')

    # Пути
    default_model_path = os.path.join(pkg_robot_description, 'urdf/robot.urdf.xacro')
    bridge_config = os.path.join(pkg_robot_gazebo, 'config/bridge.yaml')

    # Аргументы
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Gazebo симуляция
    gz_sim = IncludeLaunchDescription(
        PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py']),
        launch_arguments={'gz_args': '-r gpu_lidar_sensor.sdf'}.items()  # <-- ИЗМЕНЕНО
    )

    # Robot description
    robot_description_content = Command([
        'xacro', ' ', default_model_path, ' sim:=true'
    ])

    # Spawn entity
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'my_robot', '-topic', 'robot_description', '-z', '0.02'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[
            {'config_file': bridge_config},
            {'use_sim_time': use_sim_time}
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        gz_sim,
        spawn_entity,
        bridge
    ])