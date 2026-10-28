"""启动 Gazebo Harmonic 小车、ROS 话题桥接及仿真时钟。

面向 ROS 2 Jazzy + Gazebo Harmonic。场景中的小车使用 /cmd_vel 控制，
并将 /odom、/scan、/camera/image_raw 和 /tf 桥接回 ROS 2。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """启动 3D 场景并桥接小车的控制、里程计、传感器和时钟。"""
    default_world = PathJoinSubstitution(
        [FindPackageShare('car_bringup'), 'worlds', 'car_sensor_world.sdf'])
    gazebo_launch = PathJoinSubstitution(
        [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=default_world),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={'gz_args': [LaunchConfiguration('world'), ' -r']}.items(),
        ),
        Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name='gazebo_ros_bridge', output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            ],
        ),
        Node(
            package='ros_gz_image', executable='image_bridge',
            name='gazebo_camera_bridge', output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['/camera/image_raw'],
        ),
        # DiffDrive provides odom -> base_link. The SDF gives fixed mounts for
        # the sensors, so publish their ROS-side static transforms explicitly.
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='base_to_laser', output='screen',
            arguments=['0.16', '0', '0.12', '0', '0', '0', 'base_link', 'laser'],
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='base_to_camera', output='screen',
            arguments=['0.22', '0', '0.10', '0', '0', '0', 'base_link', 'camera_link'],
        ),
    ])
