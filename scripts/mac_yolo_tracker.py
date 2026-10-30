#!/usr/bin/env python3
"""Mac 摄像头：YOLO + ByteTrack + 轨迹 + 带真实时间间隔的卡尔曼预测。"""

import argparse
from collections import deque
import csv
import json
import math
from pathlib import Path
import threading
import time


WINDOW = "YOLO + ByteTrack + Kalman | Q: quit"


class Camera:
    """后台持续采集，只保留最新帧及读取完成时间，避免推理造成队列积压。"""

    def __init__(self, cv2, index, width, height):
        self.cap = cv2.VideoCapture(index, cv2.CAP_AVFOUNDATION)
        if not self.cap.isOpened():
            self.cap.release()
            raise RuntimeError("摄像头打开失败：检查 macOS 摄像头权限或 --camera 索引")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.condition = threading.Condition()
        self.latest = None
        self.sequence = 0
        self.stopped = False
        self.failed = False
        self.thread = threading.Thread(target=self._read, daemon=True)
        self.thread.start()

    def _read(self):
        while not self.stopped:
            ok, frame = self.cap.read()
            with self.condition:
                if not ok:
                    self.failed = True
                    self.condition.notify_all()
                    return
                self.sequence += 1
                self.latest = (self.sequence, time.monotonic(), frame)
                self.condition.notify_all()

    def read(self, previous):
        with self.condition:
            ready = self.condition.wait_for(
                lambda: self.sequence > previous or self.failed, timeout=3.0
            )
            if not ready or self.failed:
                raise RuntimeError("摄像头断开或超过 3 秒没有新画面")
            return self.latest

    def close(self):
        self.stopped = True
        self.thread.join(timeout=1.0)
        self.cap.release()


class MotionTrack:
    """ByteTrack ID 负责关联；本滤波器独立估计中心速度和短时未来位置。"""

    def __init__(self, cv2, np, center, stamp, trail_length):
        self.np = np
        self.filter = cv2.KalmanFilter(4, 2, 0, cv2.CV_64F)
        self.filter.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=float)
        self.filter.measurementNoiseCov = np.eye(2) * 9.0
        self.filter.errorCovPost = np.diag([9.0, 9.0, 10000.0, 10000.0])
        self.filter.statePost = np.array([[center[0]], [center[1]], [0.0], [0.0]])
        self.last_seen = stamp
        self.first_seen = stamp
        self.count = 1
        self.missing = False
        self.history = deque([center], maxlen=trail_length)

    def update(self, center, stamp):
        dt = stamp - self.last_seen
        if dt <= 0:
            raise ValueError("观测时间必须递增")
        np = self.np
        self.filter.transitionMatrix = np.array(
            [[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=float
        )
        # 连续白噪声加速度模型 Q，随实际 dt 改变。
        self.filter.processNoiseCov = 400.0 * np.array(
            [[dt**3/3, 0, dt**2/2, 0], [0, dt**3/3, 0, dt**2/2],
             [dt**2/2, 0, dt, 0], [0, dt**2/2, 0, dt]], dtype=float
        )
        self.filter.predict()
        self.filter.correct(np.array(center, dtype=float).reshape(2, 1))
        self.last_seen = stamp
        self.count += 1
        self.history.append(center)

    def state(self, stamp, horizon):
        """外推不调用 predict()，绘制未来轨迹不会改变滤波器状态。"""
        x, y, vx, vy = self.filter.statePost.ravel()
        age = max(0.0, stamp - self.last_seen)
        return (float(x + vx * age), float(y + vy * age), float(vx), float(vy),
                float(x + vx * (age + horizon)), float(y + vy * (age + horizon)))

    def ready(self):
        return self.count >= 4 and self.last_seen - self.first_seen >= 0.2


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", default="assets/yolov8n.pt", help="已下载的本地权重路径")
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--video", type=Path, help="使用本地视频代替摄像头验收")
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=540)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--device", default="cpu", help="cpu 或 Apple Silicon 的 mps")
    parser.add_argument("--classes", nargs="+", help="类别英文名，如 bottle cup；默认所有类别")
    parser.add_argument("--conf", type=float, default=0.1, help="保留低分检测供 ByteTrack 二次关联")
    parser.add_argument("--horizon", type=float, default=0.5, help="预测未来秒数 (0, 2]")
    parser.add_argument("--lost-timeout", type=float, default=0.5, help="丢失目标最多外推秒数")
    parser.add_argument("--motion-threshold", type=float, default=20.0, help="水平静止阈值 px/s")
    parser.add_argument("--trail-length", type=int, default=90)
    parser.add_argument("--headless", action="store_true", help="不打开窗口，用于视频自动验收")
    parser.add_argument("--max-frames", type=int, default=0, help="0 表示不限帧数")
    parser.add_argument("--output", type=Path, help="创建新目录，保存 CSV、预览图及运行报告")
    args = parser.parse_args()
    finite = (args.conf, args.horizon, args.lost_timeout, args.motion_threshold)
    if (not all(math.isfinite(value) for value in finite)
            or not 0 < args.conf <= 1 or not 0 < args.horizon <= 2
            or args.lost_timeout <= 0 or args.motion_threshold < 0
            or min(args.width, args.height, args.imgsz, args.trail_length) <= 0
            or args.max_frames < 0):
        parser.error("尺寸、时间和轨迹长度须为正数；请检查阈值和帧数范围")
    if not Path(args.model).is_file():
        parser.error(f"权重不存在：{args.model}；请先按文档下载模型")
    if args.video and not args.video.is_file():
        parser.error(f"视频不存在：{args.video}")
    return args


def draw_track(cv2, np, image, track_id, track, stamp, horizon, threshold, label, observed,
               label_boxes=None):
    x, y, vx, vy, fx, fy = track.state(stamp, horizon)
    height, width = image.shape[:2]

    def pixel(px, py):
        return (int(np.clip(px, 0, width - 1)), int(np.clip(py, 0, height - 1)))

    history = np.array(track.history, dtype=np.int32).reshape(-1, 1, 2)
    if len(history) > 1:
        cv2.polylines(image, [history], False, (255, 180, 0), 2)
    cv2.circle(image, pixel(x, y), 4, (0, 0, 255), -1)
    ready = track.ready()
    direction = "RIGHT" if vx > threshold else "LEFT" if vx < -threshold else "STILL"
    event = ""
    if ready:
        for step in range(0, 12, 2):
            a, b = step / 12, (step + 1) / 12
            cv2.line(image, pixel(x + (fx-x)*a, y + (fy-y)*a),
                     pixel(x + (fx-x)*b, y + (fy-y)*b), (0, 255, 255), 2)
        cv2.drawMarker(image, pixel(fx, fy), (0, 255, 255), cv2.MARKER_CROSS, 14, 2)
        if (x < width/2 <= fx) or (fx < width/2 <= x):
            event = " | CROSS CENTER"
        if not (0 <= fx < width and 0 <= fy < height):
            event = " | EXIT FRAME"
    status = (direction if ready else "WARMUP") if observed else "LOST / PREDICTED"
    text = f"#{track_id} {label} {status} vx={vx:+.0f}px/s{event}"
    (text_width, text_height), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.48, 1)
    tx = max(0, min(int(x), width - text_width - 4))
    ty = max(75, min(int(y - 15), height - baseline - 1))
    # 文字相交时逐行向上下避让，保持目标周围的多个标签可读。
    if label_boxes is not None:
        spacing = text_height + baseline + 8
        candidates = [ty]
        for offset in range(spacing, height, spacing):
            candidates.extend((ty - offset, ty + offset))
        for candidate in candidates:
            if not 75 <= candidate < height - baseline:
                continue
            rectangle = (tx, candidate - text_height - 3, tx + text_width + 3, candidate + baseline)
            if all(rectangle[2] < other[0] or rectangle[0] > other[2]
                   or rectangle[3] < other[1] or rectangle[1] > other[3] for other in label_boxes):
                ty = candidate
                label_boxes.append(rectangle)
                break
    cv2.rectangle(image, (tx, ty - text_height - 3),
                  (min(width - 1, tx + text_width + 3), ty + baseline), (25, 25, 25), -1)
    cv2.putText(image, text, (tx, ty),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 255), 1, cv2.LINE_AA)
    return (x, y, vx, vy, fx if ready else "", fy if ready else "", status, event.strip(" |"))


def main():
    args = parse_args()
    try:
        import cv2
        import numpy as np
        from ultralytics import YOLO
    except ImportError as error:
        raise RuntimeError("请在独立环境安装 scripts/requirements-yolo-mac.txt") from error

    model = YOLO(args.model)
    classes = None
    if args.classes:
        names = {name: index for index, name in model.names.items()}
        unknown = set(args.classes) - names.keys()
        if unknown:
            raise ValueError(f"模型不支持类别：{sorted(unknown)}；可选：{sorted(names)}")
        classes = [names[name] for name in args.classes]
    # 预热并初始化 ByteTrack（依赖已显式安装，不在循环中联网安装）。
    model.track(np.zeros((args.height, args.width, 3), dtype=np.uint8),
                persist=True, tracker="bytetrack.yaml", imgsz=args.imgsz,
                conf=args.conf, device=args.device, classes=classes, verbose=False)
    source = None
    csv_file = None
    tracks, labels = {}, {}
    frames = observations = predictions = 0
    seen_ids = set()
    sequence = 0
    failure = None
    started = time.monotonic()
    csv_writer = None
    try:
        if args.output:
            args.output.mkdir(parents=True, exist_ok=False)
            csv_file = (args.output / "tracks.csv").open("w", newline="", encoding="utf-8")
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow(["frame", "time_s", "id", "observed", "x", "y", "vx_px_s",
                                 "vy_px_s", "future_x", "future_y", "status", "event"])
        if args.video:
            source = cv2.VideoCapture(str(args.video))
            if not source.isOpened():
                raise RuntimeError("无法读取视频")
            fps = source.get(cv2.CAP_PROP_FPS)
            if not math.isfinite(fps) or fps <= 0:
                raise RuntimeError("视频没有有效帧率，无法构建时间轴")
        else:
            source = Camera(cv2, args.camera, args.width, args.height)
        print("已启动：蓝线=历史轨迹，黄虚线/十字=预测，灰线=中心线；Q/Esc 退出。")
        epoch = None
        while not args.max_frames or frames < args.max_frames:
            if args.video:
                ok, frame = source.read()
                if not ok:
                    break
                stamp = frames / fps
            else:
                sequence, stamp, frame = source.read(sequence)
            if epoch is None:
                epoch = stamp
            stamp -= epoch
            result = model.track(frame, persist=True, tracker="bytetrack.yaml",
                                 conf=args.conf, imgsz=args.imgsz, device=args.device,
                                 classes=classes, verbose=False)[0]
            frames += 1
            image = frame.copy()
            observed_ids = set()
            boxes = result.boxes
            if boxes is not None and boxes.is_track and boxes.id is not None:
                for box, track_id, cls, conf in zip(boxes.xyxy.cpu().tolist(),
                        boxes.id.int().cpu().tolist(), boxes.cls.int().cpu().tolist(),
                        boxes.conf.cpu().tolist()):
                    x1, y1, x2, y2 = box
                    center = ((x1+x2)/2, (y1+y2)/2)
                    if (track_id not in tracks or (tracks[track_id].missing
                            and stamp - tracks[track_id].last_seen > args.lost_timeout)):
                        tracks[track_id] = MotionTrack(cv2, np, center, stamp, args.trail_length)
                    else:
                        tracks[track_id].update(center, stamp)
                    tracks[track_id].missing = False
                    labels[track_id] = f"{result.names[cls]} {conf:.2f}"
                    observed_ids.add(track_id)
                    seen_ids.add(track_id)
                    observations += 1
                    cv2.rectangle(image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            # 仅对没有当前观测的目标计丢失；低帧率但持续匹配时不反复重置滤波器。
            for track_id in list(tracks):
                if track_id not in observed_ids:
                    tracks[track_id].missing = True
                    if stamp - tracks[track_id].last_seen > args.lost_timeout:
                        del tracks[track_id]
                        labels.pop(track_id, None)
            cv2.line(image, (image.shape[1]//2, 0),
                     (image.shape[1]//2, image.shape[0]-1), (140, 140, 140), 1)
            label_boxes = []
            for track_id, track in tracks.items():
                observed = track_id in observed_ids
                values = draw_track(cv2, np, image, track_id, track, stamp, args.horizon,
                                    args.motion_threshold, labels[track_id], observed, label_boxes)
                predictions += int(track.ready())
                if csv_writer:
                    csv_writer.writerow([frames, stamp, track_id, int(observed), *values])
            fps_display = frames / max(time.monotonic() - started, 1e-6)
            cv2.putText(image, f"YOLO + ByteTrack | {fps_display:.1f} FPS | forecast {args.horizon:.2f}s",
                        (12, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 2)
            cv2.putText(image, "BLUE: history | YELLOW: forecast | Q/Esc: quit",
                        (12, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            if args.output and (frames == 1 or frames % 10 == 0):
                if not cv2.imwrite(str(args.output / "preview.jpg"), image):
                    raise RuntimeError("无法保存预览图片")
            if not args.headless:
                cv2.imshow(WINDOW, image)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27) or cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                    break
    except BaseException as error:
        failure = str(error) or type(error).__name__
        raise
    finally:
        if isinstance(source, Camera):
            source.close()
        elif source is not None:
            source.release()
        if csv_file:
            csv_file.close()
            report = dict(frames=frames, observations=observations, unique_ids=sorted(seen_ids),
                          predictions=predictions, source=str(args.video) if args.video else f"camera:{args.camera}",
                          model=args.model, device=args.device, classes=args.classes,
                          opencv=cv2.__version__, horizon=args.horizon,
                          elapsed_s=time.monotonic() - started, error=failure)
            (args.output / "report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
            print(json.dumps(report, ensure_ascii=False))
        if not args.headless:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    except (RuntimeError, ValueError, OSError) as error:
        raise SystemExit(f"错误：{error}") from error
