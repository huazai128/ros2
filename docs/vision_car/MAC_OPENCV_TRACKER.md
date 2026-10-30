# macOS OpenCV 物品跟踪 Demo

这是一个独立的本机摄像头演示，不启动 ROS 2 节点、不发布话题，也不会控制小车或电机。
它使用 OpenCV 的 CSRT 跟踪器：打开画面后按 `n`，拖拽框选一个物品，按空格或回车确认；按 `n` 可重新选择，按 `q` 或 `Esc` 退出。

## 运行

在 **Mac 本机** 的项目根目录执行：

```bash
python3 -m venv .venv-opencv
source .venv-opencv/bin/activate
python -m pip install --upgrade pip opencv-contrib-python
python scripts/mac_camera_object_tracker.py
```

首次运行时，请在 macOS 的“系统设置 → 隐私与安全性 → 摄像头”中允许所用终端或 IDE 访问摄像头。若内置摄像头不是索引 0，可尝试：

```bash
python scripts/mac_camera_object_tracker.py --camera 1
```

默认后端是 macOS 的 `AVFoundation`。画面无法打开时可让 OpenCV 自动选择后端：

```bash
python scripts/mac_camera_object_tracker.py --backend any
```

## 验收

通过条件：框选物品后显示绿色框、红色中心点、蓝色轨迹和灰色初始 X 位置参考线。黄色线标记从初始位置到当前位置的水平位移。水平向右移动时显示 `RIGHT`，向左显示 `LEFT`，低于阈值显示 `STILL`。`dx` 是相对本次框选位置的水平位移（px），`vx` 是最近约 0.3 秒估算的水平速度（px/s），画面向右为正。

依次让物品向右、停住、向左运动，观察方向、位移正负和轨迹。保持摄像头固定，否则相机运动也会计入结果。默认方向阈值为 20 px/s，轻微抖动可提高阈值：

```bash
python scripts/mac_camera_object_tracker.py --motion-threshold 35
```

重新按 `n` 框选会重置轨迹和位移起点，取消框选保留原目标。窗口提示使用英文，因为 OpenCV 内置字体无法直接绘制中文。跟踪器报告失败时显示 `LOST`，需按 `n` 重新框选；CSRT 可能在遮挡时漂移到背景，不能保证每次遮挡都能检测为丢失。关闭窗口、按 `q` 或 `Esc` 均会释放摄像头。

这只是单目、手动初始化的视觉跟踪，不能提供距离、避障或可靠身份识别，不能作为实体小车运动或安全验收依据。
