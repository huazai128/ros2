"""启动 SLAM Toolbox；仅在底盘、里程计、TF 与雷达验收通过后使用。

该 launch 只负责引入 SLAM 节点和时间源开关，不假设具体底盘或雷达驱动，
使硬件相关配置仍可集中在驱动包及 ``robot.yaml`` 中维护。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """声明可调仿真时间参数，并返回最小化的 SLAM 节点描述。"""
    return LaunchDescription([
        # 真实小车默认使用系统时间；回放 rosbag 或仿真时可显式改为 true。
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        # sync 节点适合与按时间戳发布的传感器数据协同工作。
        Node(
            package='slam_toolbox', executable='sync_slam_toolbox_node', name='slam_toolbox',
            output='screen', parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
    ])
