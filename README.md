# ROS 2 小车分阶段搭建模板

这个工作区提供一个**先验证、后集成**的 ROS 2 bringup 包。它不绑定某一种底盘、雷达或相机：实际驱动由各厂商包提供，统一在 `src/car_bringup/config/robot.yaml` 填入话题名与坐标系名称。

没有设备时，可先启动合成传感器仿真。它发布 `/scan`、`/camera/image_raw` 与
`base_link → laser/camera_link` 静态 TF，便于验证话题、消息和 RViz 显示；它不是
Gazebo 物理仿真，不能据此验证 SLAM 或导航效果。

```bash
ros2 launch car_bringup sensor_sim.launch.py
```

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
