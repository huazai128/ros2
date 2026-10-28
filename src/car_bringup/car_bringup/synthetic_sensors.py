"""无硬件时用于接口验收的确定性激光雷达与相机数据发布器。

这是合成数据源，不模拟碰撞、光照、底盘运动或 Gazebo 物理；它的用途是
验证本项目的传感器话题、消息类型、时间戳和 TF 连接是否正确。
"""
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class SyntheticSensors(Node):
    """持续发布合成的 2D LaserScan 和 RGB Image，并提供传感器静态 TF。"""

    def __init__(self):
        super().__init__('synthetic_sensors')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('laser_frame', 'laser')
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('image_width', 320)
        self.declare_parameter('image_height', 240)

        self.base_frame = self.get_parameter('base_frame').value
        self.laser_frame = self.get_parameter('laser_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.width = self.get_parameter('image_width').value
        self.height = self.get_parameter('image_height').value
        rate_hz = self.get_parameter('rate_hz').value
        if rate_hz <= 0.0 or self.width <= 0 or self.height <= 0:
            raise ValueError('rate_hz, image_width and image_height must be positive')

        self.scan_publisher = self.create_publisher(
            LaserScan, self.get_parameter('scan_topic').value, 10)
        self.image_publisher = self.create_publisher(
            Image, self.get_parameter('camera_topic').value, 10)
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_transforms()
        self.phase = 0
        self.create_timer(1.0 / rate_hz, self._publish)
        self.get_logger().info(
            'Publishing synthetic sensors only: LaserScan and RGB Image; no physics or hardware.')

    def _publish_static_transforms(self):
        transforms = []
        for child_frame, x, z in (
            (self.laser_frame, 0.12, 0.15),
            (self.camera_frame, 0.18, 0.20),
        ):
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = self.base_frame
            transform.child_frame_id = child_frame
            transform.transform.translation.x = x
            transform.transform.translation.z = z
            transform.transform.rotation.w = 1.0
            transforms.append(transform)
        self.tf_broadcaster.sendTransform(transforms)

    def _publish(self):
        stamp = self.get_clock().now().to_msg()
        self.scan_publisher.publish(self._make_scan(stamp))
        self.image_publisher.publish(self._make_image(stamp))
        self.phase = (self.phase + 1) % 256

    def _make_scan(self, stamp):
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = self.laser_frame
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.tau / 360.0
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.12
        scan.range_max = 8.0
        # A room-like range profile with a moving circular feature, useful in RViz.
        scan.ranges = [
            3.0 + 0.6 * math.sin(4.0 * angle) + 0.25 * math.sin(angle + self.phase / 30.0)
            for angle in (scan.angle_min + index * scan.angle_increment for index in range(360))
        ]
        return scan

    def _make_image(self, stamp):
        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = self.camera_frame
        image.height = self.height
        image.width = self.width
        image.encoding = 'rgb8'
        image.is_bigendian = False
        image.step = self.width * 3
        pixels = bytearray(image.step * self.height)
        for y in range(self.height):
            for x in range(self.width):
                offset = y * image.step + x * 3
                pixels[offset] = (x + self.phase) % 256
                pixels[offset + 1] = y * 255 // max(1, self.height - 1)
                pixels[offset + 2] = 96 if ((x // 24 + y // 24) % 2) else 192
        image.data = bytes(pixels)
        return image


def main():
    rclpy.init()
    node = SyntheticSensors()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
