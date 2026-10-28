"""以只读方式检查 ROS 图谱，作为小车分阶段 bringup 的验收门槛。

本模块只读取当前 ROS 2 图谱中的话题与消息类型；不会发布速度、修改
参数或连接任何硬件，因此可在每个阶段结束后安全执行。
"""
import argparse
import sys

import rclpy
from rclpy.node import Node


# 每个阶段的最小话题契约。键是验收阶段，值是该阶段必须出现的话题和
# 对应消息类型；空字典表示由下方的专用逻辑处理。
STAGES = {
    'base': {'/cmd_vel': 'geometry_msgs/msg/Twist'},
    'odometry': {'/odom': 'nav_msgs/msg/Odometry'},
    'imu': {'/imu/data': 'sensor_msgs/msg/Imu'},
    # A sensor check accepts either a 2D lidar or a camera.
    'sensors': {},
    'tf': {'/tf': 'tf2_msgs/msg/TFMessage', '/tf_static': 'tf2_msgs/msg/TFMessage'},
    'slam': {'/map': 'nav_msgs/msg/OccupancyGrid'},
    'nav': {'/map': 'nav_msgs/msg/OccupancyGrid', '/cmd_vel': 'geometry_msgs/msg/Twist'},
}


class HealthCheck(Node):
    """封装单次阶段检查，避免检查过程改变机器人运行状态。"""

    def __init__(self, stage):
        # 节点名固定，便于在 ROS 图谱中识别这是临时诊断节点。
        super().__init__('car_bringup_health_check')
        self.stage = stage

    def check(self):
        """比较实际话题图谱与当前阶段的最小契约。

        ``sensors`` 是兼容性检查：二维雷达和相机任一存在即可通过，
        便于不同传感器配置复用同一 bringup 流程。其余阶段要求每项
        话题和消息类型都匹配，防止同名话题发布了错误类型。
        """
        graph = dict(self.get_topic_names_and_types())
        expected = STAGES[self.stage]
        failed = []
        if self.stage == 'sensors':
            has_lidar = 'sensor_msgs/msg/LaserScan' in graph.get('/scan', [])
            has_camera = 'sensor_msgs/msg/Image' in graph.get('/camera/image_raw', [])
            if not (has_lidar or has_camera):
                failed.append('/scan (LaserScan) or /camera/image_raw (Image)')
        for topic, message_type in expected.items():
            types = graph.get(topic, [])
            if message_type not in types:
                failed.append(f'{topic} (expected {message_type}; found {types or "absent"})')
        if failed:
            self.get_logger().error('FAILED: ' + '; '.join(failed))
            return False
        self.get_logger().info(f'PASS: {self.stage}; required topics are present.')
        return True


def main():
    """解析阶段参数，执行一次只读检查，并用退出码表达验收结果。"""
    parser = argparse.ArgumentParser(description='Validate required ROS 2 topics for a bringup stage.')
    parser.add_argument('--stage', required=True, choices=STAGES)
    args = parser.parse_args()
    # 生命周期无论检查成功或失败都会结束，避免遗留诊断节点和 ROS 上下文。
    rclpy.init()
    node = HealthCheck(args.stage)
    try:
        ok = node.check()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)
