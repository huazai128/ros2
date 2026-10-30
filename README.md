# ROS 2 小车分阶段搭建模板

这个工作区提供一个**先验证、后集成**的 ROS 2 bringup 包。它不绑定某一种底盘、雷达或相机：实际驱动由各厂商包提供，统一在 `src/car_bringup/config/robot.yaml` 填入话题名与坐标系名称。

没有设备时，可先启动合成传感器仿真。它发布 `/scan`、`/camera/image_raw` 与
`base_link → laser/camera_link` 静态 TF，便于验证话题、消息和 RViz 显示；它不是
Gazebo 物理仿真，不能据此验证 SLAM 或导航效果。

```bash
ros2 launch car_bringup sensor_sim.launch.py
```

在 Ubuntu 24.04 的 ROS 2 Jazzy 环境，安装 `ros-jazzy-ros-gz` 后可启动真正的
Gazebo Harmonic 3D 仿真（小车、墙体、差速运动、雷达与相机）：

```bash
ros2 launch car_bringup gazebo_sim.launch.py
```

虚拟机若遇到 Ogre2 渲染崩溃，使用 `ros2 launch car_bringup gazebo_sim.launch.py render_engine:=ogre`；
已执行的传感器和运动验证见 [Gazebo 模拟记录](docs/gazebo/README.md)。

该入口使用仿真时间，并桥接 `/cmd_vel`、`/odom`、`/scan`、`/camera/image_raw`
和 `/tf`。仿真不应被当作实体硬件的验收证据。

完整的上车顺序、Mac SSH 操作、每一步的验收标准与常用排错命令见：[运行手册](docs/RUNBOOK.md)。

## 快速开始

在小车的 Ubuntu 上：

```bash
cd ~/ros2
colcon build --symlink-install
source install/setup.bash
ros2 run car_bringup health_check --stage base
```

接着严格按 `docs/RUNBOOK.md` 的第 1 到 13 步执行。每通过一个阶段，运行对应健康检查，例如：

```bash
ros2 run car_bringup health_check --stage odometry
ros2 run car_bringup health_check --stage sensors
ros2 run car_bringup health_check --stage tf
```

> 安全提示：首次通电请将车轮架空；遥控/急停必须可用；遥控速度配置为低速后才允许落地测试。

## 视觉扫描与目标跟踪（三节点）

完整 Python 工程、YOLOv8、云台扫描、ESP32/L298N 固件、Linux PWM 后端、接线与依赖见
[视觉小车手册](docs/vision_car/README.md)。无设备可以直接运行合成目标与模拟驱动：

```bash
colcon build --symlink-install --packages-select car_bringup vision_car
source install/setup.bash
ros2 launch vision_car vision_car.launch.py
```

该模拟可验证控制状态与故障停车，不是物理仿真或真实 YOLO 检测准确率验收。实体驱动默认锁止。

如只想在 Mac 本机调用摄像头、手动框选并跟踪一个物品，可使用独立的
[macOS OpenCV 物品跟踪 Demo](docs/vision_car/MAC_OPENCV_TRACKER.md)；它不依赖 ROS 2，
也不会向小车发送任何控制命令。

自动检测、多目标 ID、轨迹及未来位置预测见
[Mac YOLO + ByteTrack + 卡尔曼 Demo](docs/vision_car/MAC_YOLO_TRACKER.md)。
