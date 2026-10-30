#!/usr/bin/env python3
"""在 macOS 摄像头画面中手动框选一个物品，并用 OpenCV 跟踪它。

快捷键：n 重新框选，q 或 Esc 退出。首次启动或跟踪丢失后会提示重新框选。
"""

import argparse
from collections import deque
import sys
import time

WINDOW_NAME = "OpenCV Mac Object Tracker"
cv2 = None


class HorizontalMotion:
    """用最近一段时间的中心点估算水平速度；右正、左负，单位为像素。"""

    def __init__(self, center, now, threshold=20.0):
        self.origin = center
        self.threshold = threshold
        self.samples = deque([(now, center)])
        self.trail = deque([center], maxlen=80)

    def update(self, center, now):
        self.samples.append((now, center))
        while len(self.samples) > 2 and self.samples[1][0] <= now - 0.3:
            self.samples.popleft()
        self.trail.append(center)
        elapsed = now - self.samples[0][0]
        speed = (center[0] - self.samples[0][1][0]) / elapsed if elapsed > 0 else 0.0
        direction = "RIGHT" if speed > self.threshold else "LEFT" if speed < -self.threshold else "STILL"
        return center[0] - self.origin[0], speed, direction


def create_csrt_tracker():
    """兼容 OpenCV 4 的普通与 legacy CSRT API。"""
    if hasattr(cv2, "TrackerCSRT_create"):
        return cv2.TrackerCSRT_create()
    if hasattr(cv2, "legacy") and hasattr(cv2.legacy, "TrackerCSRT_create"):
        return cv2.legacy.TrackerCSRT_create()
    raise RuntimeError(
        "当前 OpenCV 不含 CSRT 跟踪器；请安装 opencv-contrib-python，"
        "例如：python3 -m pip install --upgrade opencv-contrib-python"
    )


def open_camera(index: int, backend: str):
    """优先使用 macOS AVFoundation；失败时允许 OpenCV 自动选择后端。"""
    if backend == "any":
        return cv2.VideoCapture(index)

    capture = cv2.VideoCapture(index, cv2.CAP_AVFOUNDATION)
    if capture.isOpened():
        return capture
    capture.release()
    return cv2.VideoCapture(index)


def select_target(frame):
    """在当前画面上选择物品，取消选择时返回 None。"""
    roi = cv2.selectROI(WINDOW_NAME, frame, showCrosshair=True, fromCenter=False)
    x, y, width, height = (int(value) for value in roi)
    if width <= 0 or height <= 0:
        return None
    return x, y, width, height


def main():
    global cv2
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", type=int, default=0, help="摄像头索引，默认 0")
    parser.add_argument("--width", type=int, default=1280, help="请求的画面宽度")
    parser.add_argument("--height", type=int, default=720, help="请求的画面高度")
    parser.add_argument("--motion-threshold", type=float, default=20.0,
                        help="左右运动判定阈值，单位 px/s，默认 20")
    parser.add_argument(
        "--backend",
        choices=("avfoundation", "any"),
        default="avfoundation",
        help="macOS 默认使用 AVFoundation；any 交给 OpenCV 选择",
    )
    args = parser.parse_args()

    try:
        import cv2 as opencv
    except ModuleNotFoundError as error:
        raise RuntimeError(
            "未安装 OpenCV；请先执行：python3 -m pip install opencv-contrib-python"
        ) from error
    cv2 = opencv

    if args.width <= 0 or args.height <= 0 or not 0 <= args.motion_threshold < float("inf"):
        parser.error("尺寸必须大于零，运动阈值必须为非负有限数")

    create_csrt_tracker()  # 打开相机前检查依赖是否包含 CSRT。

    camera = open_camera(args.camera, args.backend)
    if not camera.isOpened():
        raise RuntimeError(
            f"无法打开摄像头索引 {args.camera}。请检查 macOS 的摄像头权限，"
            "或尝试 --camera 1。"
        )

    camera.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    tracker = None
    motion = None
    status = "Press N to select target | Q / Esc: quit"
    print("按 n 框选物品，空格/回车确认；左右移动物品查看轨迹与速度，q 退出。")

    try:
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        while True:
            ok, frame = camera.read()
            if not ok:
                raise RuntimeError("无法读取摄像头画面")

            # 始终在原始图像上跟踪和初始化；叠加文字、框与轨迹只绘制到副本。
            preview = frame.copy()
            if tracker is not None:
                found, box = tracker.update(frame)
                if found:
                    x, y, width, height = (int(value) for value in box)
                    center = (x + width // 2, y + height // 2)
                    dx, speed, direction = motion.update(center, time.monotonic())
                    cv2.rectangle(preview, (x, y), (x + width, y + height), (0, 255, 0), 2)
                    cv2.circle(preview, center, 5, (0, 0, 255), -1)
                    for start, end in zip(motion.trail, list(motion.trail)[1:]):
                        cv2.line(preview, start, end, (255, 200, 0), 2)
                    baseline = (center[0], motion.origin[1])
                    cv2.line(preview, motion.origin, baseline, (0, 255, 255), 2)
                    cv2.line(preview, (motion.origin[0], 0),
                             (motion.origin[0], frame.shape[0] - 1), (120, 120, 120), 1)
                    status = f"{direction} | dx: {dx:+d} px | vx: {speed:+.1f} px/s"
                    cv2.putText(preview, f"Target ({center[0]}, {center[1]})",
                                (max(0, x), max(60, y - 8)), cv2.FONT_HERSHEY_SIMPLEX,
                                0.55, (0, 255, 0), 2, cv2.LINE_AA)
                else:
                    tracker = None
                    motion = None
                    status = "LOST | Press N to select again"
                    print("跟踪器报告目标丢失；按 n 重新框选。")

            cv2.putText(
                preview,
                status,
                (16, 32),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(preview, "N: select | Q/Esc: quit | screen right = +X",
                        (16, frame.shape[0] - 16), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.imshow(WINDOW_NAME, preview)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27) or cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                break
            if key == ord("n"):
                box = select_target(frame)
                if box is None:
                    continue
                selected_tracker = create_csrt_tracker()
                if selected_tracker.init(frame, box) is False:
                    raise RuntimeError("跟踪器初始化失败，请重新运行并框选有效区域")
                tracker = selected_tracker
                x, y, width, height = box
                motion = HorizontalMotion((x + width // 2, y + height // 2),
                                          time.monotonic(), args.motion_threshold)
                status = "TRACKING | N: select again | Q: quit"
                print(f"已选择目标：{box}；水平位移起点已重置。")
    finally:
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        main()
    except RuntimeError as error:
        print(f"错误：{error}", file=sys.stderr)
        raise SystemExit(1) from error
