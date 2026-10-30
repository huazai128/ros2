"""USB / 合成相机；阻塞的 VideoCapture 放在可终止子进程中。"""

import multiprocessing as mp
import queue
import time
import cv2
import numpy as np
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from .common import parameters, run
from .pipeline import latest_image_qos, put_latest


def capture_worker(out, source, width, height, fps, video_file):
    """读取 USB 最新帧；每次重新打开失败后等待一秒重试。"""
    while True:
        cap = (
            cv2.VideoCapture(video_file)
            if video_file
            else cv2.VideoCapture(source, cv2.CAP_V4L2)
        )
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        try:
            while cap.isOpened():
                ok, frame = cap.read()
                if not ok:
                    break
                # 满队列优先替换旧帧，避免一直处理上一个周期的图像。
                put_latest(out, (time.time_ns(), frame))
                if video_file:
                    time.sleep(1 / fps)
        finally:
            cap.release()
        time.sleep(1)


class CameraNode(Node):
    """发布 bgr8 图像；无图像时控制器自行超时停车。"""

    def __init__(self):
        """读取参数并启动可选的 USB 捕获工作进程。"""
        super().__init__("camera_node")
        self.p = parameters(
            self,
            dict(
                simulate=True,
                device="/dev/video0",
                width=640,
                height=480,
                fps=10.0,
                video_file="",
                image_topic="/camera/image_raw",
                frame_id="camera_link",
                capture_timeout=2.0,
            ),
        )
        if (
            min(
                self.p["width"],
                self.p["height"],
                self.p["fps"],
                self.p["capture_timeout"],
            )
            <= 0
        ):
            raise ValueError("相机尺寸、帧率和超时必须大于零")
        self.bridge = CvBridge()
        self.pub = self.create_publisher(
            Image, self.p["image_topic"], latest_image_qos()
        )
        self.ctx = mp.get_context("spawn")
        self.out = self.ctx.Queue(1)
        self.worker = None
        self.last_frame = time.monotonic()
        self.start = time.monotonic()
        if not self.p["simulate"]:
            self.start_worker()
        period = (
            1 / self.p["fps"] if self.p["simulate"] else min(0.01, 1 / self.p["fps"])
        )
        self.create_timer(period, self.tick)

    def start_worker(self):
        """启动 USB 采集进程，允许主节点持续执行超时检测。"""
        self.worker = self.ctx.Process(
            target=capture_worker,
            args=(
                self.out,
                self.p["device"],
                self.p["width"],
                self.p["height"],
                self.p["fps"],
                self.p["video_file"],
            ),
            daemon=True,
        )
        self.worker.start()
        self.last_frame = time.monotonic()

    def tick(self):
        """生成测试矩形或读取 USB 帧；重建超时的采集进程。"""
        if self.p["simulate"]:
            w, h = self.p["width"], self.p["height"]
            frame = np.zeros((h, w, 3), dtype=np.uint8)
            phase = (time.monotonic() - self.start) % 18
            if 3 <= phase < 14:
                # 从右侧进入、居中、变大、消失，覆盖全部状态。
                cx = int(w * (0.8 if phase < 7 else 0.5))
                size = int(min(w, h) * (0.7 if phase > 11 else 0.22))
                cv2.rectangle(
                    frame,
                    (cx - size // 2, h // 2 - size // 2),
                    (cx + size // 2, h // 2 + size // 2),
                    (0, 255, 0),
                    -1,
                )
            stamp = self.get_clock().now().to_msg()
        else:
            try:
                wall_ns, frame = self.out.get_nowait()
            except queue.Empty:
                if time.monotonic() - self.last_frame > self.p["capture_timeout"]:
                    self.get_logger().error("摄像头断开或读取超时；停止出图并重新连接")
                    self.stop_worker()
                    self.out = self.ctx.Queue(1)
                    self.start_worker()
                return
            self.last_frame = time.monotonic()
            # 默认系统时钟；真实 USB 模式不支持 use_sim_time。
            stamp = self.get_clock().now().to_msg()
            stamp.sec, stamp.nanosec = divmod(wall_ns, 1_000_000_000)
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp, msg.header.frame_id = stamp, self.p["frame_id"]
        self.pub.publish(msg)

    def stop_worker(self):
        """终止可能卡在驱动 read() 的工作进程。"""
        if self.worker is not None:
            self.worker.terminate()
            self.worker.join(timeout=1)
            if self.worker.is_alive():
                self.worker.kill()
                self.worker.join(timeout=1)
            self.worker = None
        self.out.close()
        self.out.cancel_join_thread()

    def destroy_node(self):
        """关闭相机采集。"""
        self.stop_worker()
        return super().destroy_node()


def main():
    """camera_node 入口。"""
    run(CameraNode)
