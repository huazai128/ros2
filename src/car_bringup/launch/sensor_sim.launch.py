"""启动无硬件合成传感器仿真，用于接口与 RViz 可视化验收。

它不依赖 Gazebo，也不构成完整的物理仿真或可用于 SLAM/导航的移动底盘。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """声明传感器接口参数并启动合成发布器。"""
    arguments = [
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('laser_frame', default_value='laser'),
        DeclareLaunchArgument('camera_frame', default_value='camera_link'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('camera_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('rate_hz', default_value='10.0'),
        DeclareLaunchArgument('image_width', default_value='320'),
        DeclareLaunchArgument('image_height', default_value='240'),
    ]
    return LaunchDescription(arguments + [
        Node(
            package='car_bringup', executable='synthetic_sensors', name='synthetic_sensors',
            output='screen', parameters=[{
                'base_frame': LaunchConfiguration('base_frame'),
                'laser_frame': LaunchConfiguration('laser_frame'),
                'camera_frame': LaunchConfiguration('camera_frame'),
                'scan_topic': LaunchConfiguration('scan_topic'),
                'camera_topic': LaunchConfiguration('camera_topic'),
                'rate_hz': ParameterValue(LaunchConfiguration('rate_hz'), value_type=float),
                'image_width': ParameterValue(LaunchConfiguration('image_width'), value_type=int),
                'image_height': ParameterValue(LaunchConfiguration('image_height'), value_type=int),
            }],
        ),
    ])
