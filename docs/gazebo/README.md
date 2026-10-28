# Gazebo 模拟运行记录

2026-10-28 已在 Ubuntu 24.04 arm64 / ROS 2 Jazzy / Gazebo Harmonic 虚拟机中实际启动 GUI、传感器和桥接，随后打开 rqt_image_view。依赖此前已安装，本次未新增系统安装。仿真 ROS_DOMAIN_ID=42、ROS_LOCALHOST_ONLY=1、GZ_PARTITION=ros2_car_demo；没有连接或驱动真实硬件。

## 已修复

- Ogre2 在当前虚拟机 Mesa/EGL 初始化时段错误；新增 launch 参数 `render_engine`，本机实测用 `ogre` 和 `QT_QPA_PLATFORM=xcb LIBGL_ALWAYS_SOFTWARE=1` 成功启动。默认仍为 ogre2。
- 相机、雷达加固定关节，防止与底盘脱离；底盘增加低摩擦前后支撑球，避免双轮轴导致俯仰失稳。
- 雷达改为 Gazebo 渲染支持的 `gpu_lidar`，用 `gz_frame_id` 扩展设置传感器 frame。
- 速度桥接仅 ROS→Gazebo，其余仅 Gazebo→ROS，消除双向 `/clock` 桥接冲突；环境照明和背景变亮。

## 已验证

[实际验收 JSON](report.json)：观测到相机 36 帧、雷达 25 帧、里程计 124 帧、时钟 2332 帧，雷达最后一帧 184 个有限测距点。只在隔离仿真域内发出 0.10 m/s、持续两秒墙上时间的前进指令，随后连续发送零速；里程计位移约 0.064m。软件渲染使仿真慢于真实时间，因此不是两秒模拟时间，也不是硬件速度/制动距离标定。

[实际相机截图](camera.jpg)。静态语法检查和 SDF XML 解析通过，Ubuntu `colcon build --symlink-install --packages-select car_bringup` 通过。`gz_frame_id` 是 Gazebo 的扩展字段，SDF 解析器可能输出“保留未知字段”警告；本次传感器仍正常发布。

当前场景只有固定相机、雷达、墙体和差速车；旋转云台与 YOLO 自动跟踪尚未接入该 Gazebo 场景。已有 `vision_car` 的合成图像/视频回放演示是另一套入口，不能直接并行发布同名相机话题。

## 再次启动（Ubuntu 桌面终端）

```bash
cd ~/ros2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select car_bringup
source install/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1 GZ_PARTITION=ros2_car_demo
export QT_QPA_PLATFORM=xcb LIBGL_ALWAYS_SOFTWARE=1
ros2 launch car_bringup gazebo_sim.launch.py render_engine:=ogre
```

另开终端查看图像和遥控（这两条运行在不同终端，均保持相同环境）：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2/install/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2/install/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.10 -p turn:=0.35
```

焦点放在遥控终端：i 前进，j/l 左右转，逗号后退，k 停止。真实驱动保持关闭；本次仅限仿真，实体车仍需急停、架空、低速和现场监护验收。

本次由后台 user service 保持窗口运行。再次启动前先停止本次服务，避免重复实例：

```bash
systemctl --user stop ros2-gazebo-camera-view ros2-gazebo-demo
```

之后自己在终端启动的仿真可用 Ctrl+C 退出。
