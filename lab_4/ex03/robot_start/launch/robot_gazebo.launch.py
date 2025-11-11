from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
import os

def generate_launch_description():
    # Пакеты
    pkg_share = FindPackageShare(package='robot_start').find('robot_start')
    pkg_ros_gz_sim = FindPackageShare(package='ros_gz_sim').find('ros_gz_sim')

    # Пути
    default_model_path = os.path.join(pkg_share, 'robot.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/robot.rviz')

    # Аргументы
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')

    # Gazebo симуляция
    gz_sim = IncludeLaunchDescription(
        PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py']),
        launch_arguments={'gz_args': '-r -v 4 empty.sdf'}.items()
    )

    # Robot description
    robot_description_content = Command([
        'xacro', ' ', default_model_path, ' sim:=true'
    ])

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )

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
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Joint state publisher (только если не GUI)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(gui)
    )

    # Joint state publisher GUI (если gui=true)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(gui)
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
        # Аргументы
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Flag to enable joint_state_publisher_gui'
        ),
        DeclareLaunchArgument(
            'model',
            default_value=default_model_path,
            description='Absolute path to robot urdf file'
        ),
        DeclareLaunchArgument(
            'rvizconfig',
            default_value=default_rviz_config_path,
            description='Absolute path to rviz config file'
        ),
        # Ноды
        gz_sim,
        robot_state_publisher_node,
        spawn_entity,
        bridge,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])