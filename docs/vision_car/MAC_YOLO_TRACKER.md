# Mac 摄像头 YOLO + ByteTrack + 卡尔曼预测 Demo

在 Mac 本机运行，默认打开索引 0 的摄像头。YOLO 自动检测物品，ByteTrack 关联目标 ID；
每个 ID 独立保存中心点轨迹，并用 OpenCV 卡尔曼滤波器估计速度和未来位置。无需 ROS 2。

## 安装和模型下载

以下命令在 **Mac 终端**运行：

```bash
cd /Users/JOYY/Downloads/AI/ros2
/usr/bin/python3 -m venv .venv-yolo-mac
source .venv-yolo-mac/bin/activate
python -m pip install -r scripts/requirements-yolo-mac.txt
mkdir -p assets
curl -fL https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8n.pt -o assets/yolov8n.pt
python scripts/mac_yolo_tracker.py
```

使用独立环境，避免与旧 CSRT Demo 的 `opencv-contrib-python` 混装。
固定依赖适用于本项目 Mac Python 3.9 环境，使用 `/usr/bin/python3 --version` 确认。
当前 Homebrew 的 `python3` 为 3.15，不兼容本文件固定的 PyTorch 版本，因此命令明确指定系统解释器。
其他 Mac 如没有系统 Python 3.9，可使用已安装的 Python 3.10～3.12 创建该环境。
权重必须事先下载，程序不会在摄像头循环中下载权重。

首次运行，在“系统设置 → 隐私与安全性 → 摄像头”中允许实际启动程序的终端或 IDE，
必要时重启该应用。看到 `not authorized to capture video` 表示权限未放行。
`--camera 1` 可尝试其他设备；在自己已授权的 Terminal 中执行通常更方便。

## 跟踪物品

默认跟踪模型认识的所有类别。演示杯子、瓶子可筛选：

```bash
source .venv-yolo-mac/bin/activate
python scripts/mac_yolo_tracker.py --classes bottle cup --horizon 0.5
```

`--classes` 使用模型类别英文名，`cell phone` 等带空格类别须写成 `--classes "cell phone"`。
任意自定义物品不一定属于预训练类别，需要相应训练的权重，通过 `--model` 指定。
Apple Silicon 可以尝试 `--device mps`；默认 CPU，实际帧率以窗口显示为准。

| 显示 | 含义 |
| --- | --- |
| 绿色框、`#ID`、类别和分数 | 本帧观测到的目标 |
| 红点 | 卡尔曼滤波后的中心点 |
| 蓝线 | 同 ID 历史观测中心轨迹，默认最多 90 点 |
| 黄色虚线和十字 | 预测未来 0.5 秒的位置，越界时标记裁剪在画面边缘 |
| `WARMUP` | 尚未积累至少 4 次观测和 0.2 秒数据，暂不显示未来位置 |
| `LEFT / RIGHT / STILL` | 画面水平方向，右为正，默认静止阈值 20 px/s |
| `CROSS CENTER` | 预测将在指定时间窗内越过灰色中心线 |
| `EXIT FRAME` | 预测位置超出画面范围，优先于越线提示 |
| `LOST / PREDICTED` | 当前没有检测匹配，仅短时外推；默认最多 0.5 秒后清除 |

按 `q`、`Esc` 或关闭窗口退出，Ctrl+C 也会清理设备。
需要稳定相机，速度单位是像素/秒，不是实际米/秒。突然转向、遮挡、框抖动及推理延迟均影响预测。
ByteTrack 内部滤波用于关联，本 Demo 额外的滤波器使用实际采集时间差估算速度；
摄像头线程持续采集，只传递最新帧，减少积压。时间戳是软件读取完成时间，并非传感器曝光时间。
预测相对于当前被处理帧的采集时间，不额外补偿显示延迟。
同 ID 丢失超过超时后清除历史，即使再出现相同数字也会重新预热；遮挡/交叉可能换 ID。
低帧率但同 ID 在每次处理帧中持续匹配时，保留滤波状态；不把慢推理本身当作目标丢失。

## 保存轨迹和自动验收

保存到一个**尚不存在**的新目录，避免覆盖先前结果：

```bash
python scripts/mac_yolo_tracker.py --classes bottle cup --output demo-runs/camera-01
```

输出包括逐帧 `tracks.csv`（ID、时间、是否观测、中心、速度、未来位置、事件），
每 10 帧更新的 `preview.jpg` 和退出时的 `report.json`。
CSV 中 `observed=0` 表示丢失期间的估计，`future_x/y` 为空表示预热；真实位置与预测不得混算。
CSV 持续写入，运行时间长时会增长；退出报告的 `error` 不为空表示运行失败。
模型、Mac 虚拟环境和 `demo-runs/` 不提交到 Git，也不同步到 Ubuntu。

不访问摄像头的本地视频验收：

```bash
.venv-yolo-mac/bin/python scripts/test_mac_yolo_tracker.py
.venv-yolo-mac/bin/python scripts/mac_yolo_tracker.py \
  --video docs/vision_car/results/yolo/replay.avi \
  --headless --max-frames 40 --output demo-runs/replay-01
```

视频以文件帧率构建时间轴，不用推理耗时作为运动时间。
通过条件：单元测试全部通过；回放报告 `error: null`、`observations > 0`、`predictions > 0`，
CSV 中同一目标连续多帧使用相同 ID，预览有框和轨迹。静态回放只能验证链路，
实际运动验收应在摄像头前将同一物品向右、停住、向左移动，检查方向和预测十字随运动更新。
本演示只做图像坐标预测，不连接小车运动控制。

## 本机验收记录（2026-10-30）

- 环境：Mac arm64、Python 3.9.6、OpenCV 4.10.0、Ultralytics 8.3.40、PyTorch 2.5.1，CPU。
- 5 项卡尔曼/标记测试通过；`pip check` 无依赖冲突；Python 语法、Shell 语法和 diff 格式检查通过。
- 对既有真实图片回放做左右平移，生成 60 帧、10 FPS 的软件测试视频；实际 YOLO + ByteTrack
  得到 282 次观测、272 次预测，ID #1 连续观测 60/60 帧。右移/左移阶段估计速度均值
  分别约 +30.1/-31.1 px/s，与构造视频的 +30/-30 px/s 方向一致。
- Mac 摄像头索引 0 无窗口采集验收：20 帧、94 次观测、77 次预测，`error: null`，设备正常释放。
  这是摄像头链路验收；实际手持物品的未来位置误差尚未定量测量。
- 本地输出：`demo-runs/final-validation/` 和 `demo-runs/camera-validation-20/`，其中摄像头画面仅保存在本机。
- 系统 Python 的 LibreSSL 可能使 urllib3 输出兼容性警告；本次本地权重推理与摄像头验收均成功。

API 依据：[Ultralytics 官方跟踪文档](https://docs.ultralytics.com/modes/track/)。
