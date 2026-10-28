"""使用已保存地图启动 Nav2；不得在没有有效地图时启动。

本文件复用 ROS 发行版提供的完整 Nav2 bringup，避免用精简参数意外覆盖
必要默认值；平台调参时可通过 ``params_file`` 显式传入完整参数文件。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    """声明地图、参数与时间源参数，并转交给 Nav2 官方 bringup。"""
    # 在运行时解析包目录，避免把不同机器上的 ROS 安装路径写死到代码里。
    nav2_launch = PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py'])
    return LaunchDescription([
        # 不提供默认地图，强制操作者明确选择已保存且已验收的地图文件。
        DeclareLaunchArgument('map', description='Absolute path to a saved map YAML file'),
        # Use the distribution's complete Nav2 configuration by default. Pass a
        # platform-specific complete parameter file when tuning the vehicle.
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution(
            [FindPackageShare('nav2_bringup'), 'params', 'nav2_params.yaml'])),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        # 透传参数，以保持与当前 ROS 发行版 Nav2 启动入口的兼容性。
        IncludeLaunchDescription(PythonLaunchDescriptionSource(nav2_launch), launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()),
    ])
