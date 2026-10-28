"""YOLOv8 推理与带框图像输出；独立进程隔离推理超时。"""
import json
import multiprocessing as mp
import queue
import time
from pathlib import Path
import cv2
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from .common import parameters, run


def detect_worker(requests, results, p):
    """加载一次 YOLOv8；模拟模式只检测绿色矩形，不冒充 YOLO。"""
    try:
        model = None
        if not p['simulate']:
            from ultralytics import YOLO
            if not Path(p['model']).is_file():
                raise FileNotFoundError('请先下载 YOLOv8 权重并设置 model 绝对路径')
            model = YOLO(p['model'])
        results.put(('ready', None))
        while True:
            frame = requests.get()
            if p['simulate'] and p['simulate_inference_delay'] > 0:
                time.sleep(p['simulate_inference_delay'])
            boxes = []
            if p['simulate']:
                mask = cv2.inRange(frame, (0, 160, 0), (100, 255, 100))
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for contour in contours:
                    x, y, w, h = cv2.boundingRect(contour)
                    if w*h > 100:
                        boxes.append(dict(class_id=0, label='sim_target', confidence=1.0,
                                          xyxy=[x, y, x+w, y+h]))
            else:
                result = model.predict(frame, conf=p['confidence'], imgsz=p['imgsz'],
                                       device=p['device'], verbose=False)[0]
                for box in result.boxes:
                    cls = int(box.cls.item())
                    label = result.names[cls]
                    if p['target_class'] and label != p['target_class']:
                        continue
                    boxes.append(dict(class_id=cls, label=label,
                        confidence=float(box.conf.item()), xyxy=box.xyxy[0].cpu().tolist()))
            results.put(('result', boxes))
    except Exception as exc:
        results.put(('error', str(exc)))


class YoloDetectNode(Node):
    """保留最新一帧，定时轮询工作进程，避免阻塞 ROS 回调。"""
    def __init__(self):
        """声明推理参数、图像订阅及结果输出。"""
        super().__init__('yolo_detect_node')
        self.p = parameters(self, dict(simulate=True, model='', device='cpu', imgsz=320,
            confidence=0.5, target_class='person', inference_timeout=2.0,
            simulate_inference_delay=0.0,
            startup_timeout=60.0, image_timeout=1.0, image_topic='/camera/image_raw',
            bbox_topic='/vision/bbox', annotated_topic='/vision/annotated'))
        if (not 0 < self.p['confidence'] <= 1 or self.p['imgsz'] <= 0 or
                min(self.p['inference_timeout'], self.p['startup_timeout'], self.p['image_timeout']) <= 0):
            raise ValueError('置信度、推理尺寸或超时配置无效')
        self.bridge = CvBridge()
        self.pub = self.create_publisher(String, self.p['bbox_topic'], 10)
        self.annotated = self.create_publisher(Image, self.p['annotated_topic'], qos_profile_sensor_data)
        self.create_subscription(Image, self.p['image_topic'], self.image, qos_profile_sensor_data)
        self.ctx = mp.get_context('spawn')
        self.latest = None
        self.job = None
        self.last_input = time.monotonic()
        self.ready = False
        self.worker = None
        self.launch_worker()
        self.create_timer(0.02, self.poll)
        self.last_fault = 0.0

    def launch_worker(self):
        """创建隔离推理进程和新的通信队列。"""
        self.requests, self.results = self.ctx.Queue(1), self.ctx.Queue(2)
        self.worker = self.ctx.Process(target=detect_worker,
            args=(self.requests, self.results, self.p), daemon=True)
        self.worker.start()
        self.ready = False
        self.started = time.monotonic()

    def image(self, msg):
        """仅缓存最新图像；限制待处理帧数，防止积压导致旧帧驱车。"""
        self.latest = msg
        self.last_input = time.monotonic()

    def fault(self, reason):
        """发送显式故障包，控制器收到后立即归零。"""
        self.pub.publish(String(data=json.dumps(dict(ok=False, error=reason, boxes=[]))))
        if time.monotonic()-self.last_fault > 2:
            self.get_logger().error(reason)
            self.last_fault = time.monotonic()

    def poll(self):
        """收取推理结果，检查图像/模型超时并调度最新帧。"""
        now = time.monotonic()
        try:
            kind, payload = self.results.get_nowait()
        except queue.Empty:
            kind, payload = None, None
        if kind == 'ready':
            self.ready = True
        elif kind == 'error':
            self.fault('YOLO 工作进程错误: '+payload)
            self.restart()
            return
        elif kind == 'result' and self.job is not None:
            msg, frame, _ = self.job
            self.job = None
            age = self.get_clock().now().nanoseconds/1e9 - (
                msg.header.stamp.sec + msg.header.stamp.nanosec/1e9)
            if age > self.p['image_timeout'] or age < -0.1:
                self.fault('推理结果图像已过期')
            else:
                packet = dict(ok=True, stamp_sec=msg.header.stamp.sec,
                    stamp_nanosec=msg.header.stamp.nanosec, width=msg.width,
                    height=msg.height, boxes=payload, backend='mock' if self.p['simulate'] else 'yolov8')
                self.pub.publish(String(data=json.dumps(packet, allow_nan=False)))
                for b in payload:
                    x1, y1, x2, y2 = map(int, b['xyxy'])
                    cv2.rectangle(frame, (x1,y1), (x2,y2), (0,255,255), 2)
                    cv2.putText(frame, f"{b['label']} {b['confidence']:.2f}",
                        (x1,max(18,y1-8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
                    self.get_logger().info(f"{b['label']} conf={b['confidence']:.2f} bbox={b['xyxy']}")
                output = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
                output.header = msg.header
                self.annotated.publish(output)
        if (not self.ready and now-self.started > self.p['startup_timeout']) or (
                self.job is not None and now-self.job[2] > self.p['inference_timeout']):
            self.fault('YOLO 加载/推理超时：终止子进程，丢弃旧结果')
            self.restart()
            return
        if not self.worker.is_alive() and kind is None:
            self.fault('YOLO 子进程意外退出')
            self.restart()
            return
        if now-self.last_input > self.p['image_timeout']:
            self.fault('相机图像中断')
        if self.ready and self.job is None and self.latest is not None:
            msg, self.latest = self.latest, None
            try:
                frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
                self.requests.put_nowait(frame)
                self.job = (msg, frame, now)
            except Exception as exc:
                self.fault('图像转换异常: '+str(exc))

    def restart(self):
        """回收卡死进程；旧任务绝不在恢复后继续驱动车辆。"""
        self.stop_worker()
        self.job = self.latest = None
        self.launch_worker()

    def stop_worker(self):
        """有界等待并强制终止推理进程。"""
        if self.worker is not None:
            self.worker.terminate()
            self.worker.join(timeout=0.3)
            if self.worker.is_alive():
                self.worker.kill()
                self.worker.join(timeout=0.3)
            self.requests.close()
            self.results.close()

    def destroy_node(self):
        """停止推理进程。"""
        self.stop_worker()
        return super().destroy_node()


def main():
    """yolo_detect_node 入口。"""
    run(YoloDetectNode)
