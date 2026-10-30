"""事件驱动的 YOLO 推理；隔离超时进程，限频发布预览。"""

import json
import math
import multiprocessing as mp
import queue
import threading
import time
from pathlib import Path

import cv2
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .common import parameters, run
from .pipeline import latest_image_qos, stamp_seconds


def detect_worker(requests, results, p):
    """模型仅加载一次；管道只传检测框与耗时。"""
    try:
        model = None
        if not p["simulate"]:
            from ultralytics import YOLO

            if not Path(p["model"]).is_file():
                raise FileNotFoundError("请先下载权重并设置 model 绝对路径")
            model = YOLO(p["model"])
        results.send(("ready", None))
        while True:
            frame = requests.get()
            started = time.monotonic()
            boxes = []
            if p["simulate"]:
                time.sleep(p["simulate_inference_delay"])
                mask = cv2.inRange(frame, (0, 160, 0), (100, 255, 100))
                contours, _ = cv2.findContours(
                    mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
                )
                for contour in contours:
                    x, y, width, height = cv2.boundingRect(contour)
                    if width * height > 100:
                        boxes.append(
                            {
                                "class_id": 0,
                                "label": "sim_target",
                                "confidence": 1.0,
                                "xyxy": [x, y, x + width, y + height],
                            }
                        )
            else:
                result = model.predict(
                    frame,
                    conf=p["confidence"],
                    imgsz=p["imgsz"],
                    device=p["device"],
                    verbose=False,
                )[0]
                # 批量搬回全部框，避免逐字段触发 GPU/CPU 同步。
                for (
                    x1,
                    y1,
                    x2,
                    y2,
                    confidence,
                    class_id,
                ) in result.boxes.data.cpu().tolist():
                    label = result.names[int(class_id)]
                    if p["target_class"] and label != p["target_class"]:
                        continue
                    boxes.append(
                        {
                            "class_id": int(class_id),
                            "label": label,
                            "confidence": float(confidence),
                            "xyxy": [x1, y1, x2, y2],
                        }
                    )
            results.send(
                (
                    "result",
                    {
                        "boxes": boxes,
                        "inference_ms": (time.monotonic() - started) * 1000,
                    },
                )
            )
    except Exception as exc:
        results.send(("error", str(exc)))
    finally:
        results.close()


def receive_results(connection, inbox, stopped, guard):
    """线程只接收结果；ROS 状态由执行器内的 guard 回调修改。"""
    try:
        while not stopped.is_set():
            if not connection.poll(0.1):
                continue
            event = connection.recv()
            if stopped.is_set():
                return
            inbox.put(event)
            guard.trigger()
    except (EOFError, OSError):
        if not stopped.is_set():
            inbox.put(("error", "YOLO 结果管道中断"))
            guard.trigger()


class YoloDetectNode(Node):
    """保留最新帧，结果到达立即通知 ROS 执行器。"""

    def __init__(self):
        """配置话题、事件接收与独立看门狗。"""
        super().__init__("yolo_detect_node")
        self.p = parameters(
            self,
            {
                "simulate": True,
                "model": "",
                "device": "cpu",
                "imgsz": 320,
                "confidence": 0.5,
                "target_class": "person",
                "inference_timeout": 2.0,
                "simulate_inference_delay": 0.0,
                "startup_timeout": 60.0,
                "image_timeout": 1.0,
                "annotated_hz": 2.0,
                "image_topic": "/camera/image_raw",
                "bbox_topic": "/vision/bbox",
                "annotated_topic": "/vision/annotated",
            },
        )
        positive = ("inference_timeout", "startup_timeout", "image_timeout")
        nonnegative = ("annotated_hz", "simulate_inference_delay")
        if (
            not 0 < self.p["confidence"] <= 1
            or self.p["imgsz"] <= 0
            or any(not math.isfinite(self.p[k]) or self.p[k] <= 0 for k in positive)
            or any(not math.isfinite(self.p[k]) or self.p[k] < 0 for k in nonnegative)
        ):
            raise ValueError("置信度、推理尺寸、预览频率或超时配置无效")

        self.bridge = CvBridge()
        self.pub = self.create_publisher(String, self.p["bbox_topic"], 1)
        self.annotated = self.create_publisher(
            Image, self.p["annotated_topic"], latest_image_qos()
        )
        self.create_subscription(
            Image, self.p["image_topic"], self.image, latest_image_qos()
        )
        self.ctx = mp.get_context("spawn")
        self.latest = None
        self.job = None
        self.preview = None
        self.last_input = time.monotonic()
        self.last_fault = 0.0
        self.worker = None
        self.guard = self.create_guard_condition(self.poll)
        self.launch_worker()
        # 周期回调仅承担看门狗；结果和新任务不再等 20 ms 轮询。
        self.create_timer(0.02, self.poll)
        if self.p["annotated_hz"] > 0:
            self.create_timer(1 / self.p["annotated_hz"], self.publish_preview)

    def launch_worker(self):
        """每代工作进程使用独立管道、收件箱和停止事件。"""
        self.requests = self.ctx.Queue(1)
        self.connection, sender = self.ctx.Pipe(duplex=False)
        self.inbox = queue.Queue()
        self.stopped = threading.Event()
        self.worker = self.ctx.Process(
            target=detect_worker, args=(self.requests, sender, self.p), daemon=True
        )
        self.worker.start()
        sender.close()
        self.receiver = threading.Thread(
            target=receive_results,
            args=(self.connection, self.inbox, self.stopped, self.guard),
            daemon=True,
        )
        self.ready = False
        self.started = time.monotonic()
        self.receiver.start()

    def image(self, msg):
        """覆盖未处理图像，并尝试直接调度空闲模型。"""
        self.latest = msg
        self.last_input = time.monotonic()
        self.schedule()

    def fault(self, reason):
        """故障立即发布，清除预览并限制错误日志频率。"""
        self.preview = None
        packet = {"ok": False, "error": reason, "boxes": []}
        self.pub.publish(String(data=json.dumps(packet)))
        if time.monotonic() - self.last_fault > 2:
            self.get_logger().error(reason)
            self.last_fault = time.monotonic()

    def poll(self):
        """事件处理和超时检测均在 ROS 执行器中串行运行。"""
        while True:
            try:
                kind, payload = self.inbox.get_nowait()
            except queue.Empty:
                break
            if kind == "ready":
                self.ready = True
            elif kind == "error":
                self.fault("YOLO 工作进程错误: " + payload)
                self.restart()
                return
            elif kind == "result" and self.job is not None:
                self.publish_result(payload)

        now = time.monotonic()
        startup_expired = (
            not self.ready and now - self.started > self.p["startup_timeout"]
        )
        inference_expired = (
            self.job is not None and now - self.job[2] > self.p["inference_timeout"]
        )
        if startup_expired or inference_expired:
            self.fault("YOLO 加载/推理超时：终止子进程，丢弃旧结果")
            self.restart()
            return
        if not self.worker.is_alive():
            self.fault("YOLO 子进程意外退出")
            self.restart()
            return
        if now - self.last_input > self.p["image_timeout"]:
            self.fault("相机图像中断")
        self.schedule()

    def publish_result(self, payload):
        """再次验证时限，先发布检测包，再缓存可选预览。"""
        msg, frame, submitted = self.job
        self.job = None
        age = self.get_clock().now().nanoseconds / 1e9 - stamp_seconds(msg.header.stamp)
        elapsed = time.monotonic() - submitted
        if elapsed > self.p["inference_timeout"]:
            self.fault("推理结果超过推理时限")
            return
        if not 0 <= age <= self.p["image_timeout"]:
            self.fault("推理结果图像已过期或时间戳在未来")
            return
        packet = {
            "ok": True,
            "stamp_sec": msg.header.stamp.sec,
            "stamp_nanosec": msg.header.stamp.nanosec,
            "width": msg.width,
            "height": msg.height,
            "boxes": payload["boxes"],
            "backend": "mock" if self.p["simulate"] else "yolov8",
            "inference_ms": payload["inference_ms"],
            "result_age_ms": age * 1000,
            "job_ms": elapsed * 1000,
        }
        self.pub.publish(String(data=json.dumps(packet, allow_nan=False)))
        if self.p["annotated_hz"] > 0:
            self.preview = (msg, frame, payload["boxes"])

    def schedule(self):
        """空闲时立即取最新帧；过期图像不消耗推理资源。"""
        if not self.ready or self.job is not None or self.latest is None:
            return
        msg, self.latest = self.latest, None
        age = self.get_clock().now().nanoseconds / 1e9 - stamp_seconds(msg.header.stamp)
        if not 0 <= age <= self.p["image_timeout"]:
            self.fault("待推理图像已过期或时间戳在未来")
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.requests.put_nowait(frame)
            self.job = (msg, frame, time.monotonic())
        except Exception as exc:
            self.fault("图像转换/提交异常: " + str(exc))

    def publish_preview(self):
        """预览限频，无人订阅时跳过绘框；此回调不执行推理。"""
        preview, self.preview = self.preview, None
        if preview is None or self.annotated.get_subscription_count() == 0:
            return
        msg, source, boxes = preview
        age = self.get_clock().now().nanoseconds / 1e9 - stamp_seconds(msg.header.stamp)
        if not 0 <= age <= self.p["image_timeout"]:
            return
        frame = source.copy()
        for box in boxes:
            x1, y1, x2, y2 = map(int, box["xyxy"])
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)
            cv2.putText(
                frame,
                f"{box['label']} {box['confidence']:.2f}",
                (x1, max(18, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2,
            )
        output = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        output.header = msg.header
        self.annotated.publish(output)

    def restart(self):
        """先回收旧进程，再丢弃旧任务并启动新一代。"""
        self.stop_worker()
        self.job = None
        self.latest = None
        self.preview = None
        self.launch_worker()

    def stop_worker(self):
        """有界回收进程和接收线程，避免退出时排空图像队列。"""
        if self.worker is None:
            return
        self.stopped.set()
        self.worker.terminate()
        self.worker.join(timeout=0.3)
        if self.worker.is_alive():
            self.worker.kill()
            self.worker.join(timeout=0.3)
        self.receiver.join(timeout=0.3)
        self.connection.close()
        self.requests.cancel_join_thread()
        self.requests.close()
        self.worker = None

    def destroy_node(self):
        """先停止后台接收，再由 ROS 释放 guard 和话题资源。"""
        self.stop_worker()
        return super().destroy_node()


def main():
    """运行检测节点及其清理流程。"""
    run(YoloDetectNode)
