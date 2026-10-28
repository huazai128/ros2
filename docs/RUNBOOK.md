# 小车到手后的运行手册

以下命令在**小车 Ubuntu**终端运行，除非特别写明“Mac”。每开一个新终端先执行：

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/ros2/install/setup.bash
export ROS_DOMAIN_ID=0  # Mac 与小车必须一致；可改成你的专用编号
```

## 0. 一次性安装与编译

将本仓库放在小车的 `~/ros2`，安装对应硬件的底盘/雷达/相机驱动，以及 `slam_toolbox`、`nav2_bringup`、`teleop_twist_keyboard`、`rviz2`。然后：

```bash
cd ~/ros2 && colcon build --symlink-install
source install/setup.bash
```

修改 `src/car_bringup/config/robot.yaml` 中的话题和 frame，使其与实际驱动一致，然后重新编译。下文 `底盘驱动启动命令`、`雷达驱动启动命令` 是必须替换为设备厂商提供的真实命令的位置。需要调 Nav2 参数时，以发行版自带的完整 `nav2_params.yaml` 为基础复制一份，再覆盖 `base_link`、`odom`、`map` 与 `/odom` 设置，并通过 `params_file:=<路径>` 传入。

## 1–2. 供电、电池与 Ubuntu

1. 断电检查线束、急停、保险丝；首次让驱动轮离地。
2. 电池满电后接入，测量电压在底盘要求范围内；开启主控板，登录 Ubuntu。
3. 确认系统和 ROS：`hostnamectl`、`lsb_release -a`、`printenv ROS_DISTRO`。

低电压、线束发热、急停无效时立即断电，不进入下一步。

## 3. 从 Mac SSH 连接

在 Mac 上查到小车 IP 后运行：

```bash
ssh <车载用户名>@<小车IP>
```

建议给小车保留 DHCP 地址或配置静态地址。需要在 Mac 用 RViz 时，Mac 与小车使用同一 ROS 发行版、同一网络、同一 `ROS_DOMAIN_ID`；多网卡环境设置 `ROS_LOCALHOST_ONLY=0`。

## 4. 串口 / USB 设备

插拔一次设备并比对：

```bash
ls -l /dev/serial/by-id/
lsusb
dmesg -w
```

优先在驱动配置中使用 `/dev/serial/by-id/...`，不要使用会变化的 `/dev/ttyUSB0`。若权限不足：`sudo usermod -aG dialout $USER`，重新登录。

## 5. 启动底盘驱动

仅启动厂商底盘驱动（示例占位）：

```bash
# ros2 launch <底盘驱动包> <底盘启动文件>.launch.py port:=/dev/serial/by-id/<设备>
ros2 topic list
```

它至少应订阅 `/cmd_vel`、发布 `/odom` 与 `/tf`。话题不同就更新 `robot.yaml`，并使用 remap 或驱动参数统一名称。

## 6. 低速键盘遥控

确认急停可按、车轮仍架空；新终端执行：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.15 -p turn:=0.5
```

按 `i` 前进、`,` 后退、`j` 左转、`l` 右转、`k` 停止。先确认轮向正确再落地测试；方向相反必须在底盘配置中修正，不能靠导航补偿。

## 7. 编码器与里程计

```bash
ros2 topic echo /odom --once
ros2 topic hz /odom
ros2 run car_bringup health_check --stage odometry
```

缓慢推车或低速行驶，位置与朝向应连续变化；静止时不应明显漂移。异常先查编码器线、轮径/轮距、左右轮符号。

## 8. IMU

```bash
ros2 topic echo /imu/data --once
ros2 run car_bringup health_check --stage imu
```

平放时线性加速度约为 9.8 m/s²（方向取决于安装）；旋转车体时角速度应变化。安装方向错误时先修正传感器 frame/姿态配置。

## 9. 雷达或摄像头

### Gazebo 3D 仿真（无实体设备）

ROS 2 Jazzy 环境需先安装 `ros-jazzy-ros-gz`，然后启动：

```bash
ros2 launch car_bringup gazebo_sim.launch.py
```

它会打开 Gazebo 窗口，加载带墙体的场景和差速小车，并桥接 `/scan`、
`/camera/image_raw`、`/odom`、`/tf` 与 `/cmd_vel`。这是仿真验收，不是实体车验收。
在另一终端用下列命令验证传感器；看到稳定频率才算本阶段仿真接口通过：

```bash
ros2 topic hz /scan
ros2 topic hz /camera/image_raw
ros2 run car_bringup health_check --stage sensors
```

没有实体设备、只需先验收 ROS 接口时，可在 Ubuntu 上启动合成传感器：

```bash
ros2 launch car_bringup sensor_sim.launch.py
```

它同时发布 `/scan`、`/camera/image_raw` 和 `base_link → laser/camera_link` 静态 TF；
这是确定性的接口/RViz 演示数据，不是 Gazebo 物理仿真，不能作为 SLAM 或导航验收证据。

启动相应厂商驱动后检查：

```bash
ros2 topic echo /scan --once        # 雷达
ros2 topic hz /camera/image_raw     # 摄像头
ros2 run car_bringup health_check --stage sensors
```

`/scan` 的 `frame_id` 必须存在于 TF；相机则检查图像频率、分辨率和曝光。

## 10. RViz 与 TF

```bash
rviz2
ros2 run tf2_tools view_frames
```

RViz 固定坐标系先设 `odom`。添加 TF、LaserScan、Odometry（相机则添加 Image）。必须看到连通链：`odom → base_link → laser/camera`。建图后固定坐标系改为 `map`，应多出 `map → odom`。

## 11. SLAM 建图

在一个终端持续运行底盘和雷达驱动后，另开终端：

```bash
ros2 launch car_bringup slam.launch.py
ros2 run car_bringup health_check --stage slam
```

在 RViz 固定坐标系设为 `map`，低速沿墙走闭环；不要高速、打滑或反复原地旋转。地图稳定后停下。

## 12. 保存地图

```bash
mkdir -p ~/ros2/maps
ros2 run nav2_map_server map_saver_cli -f ~/ros2/maps/home
```

会生成 `home.yaml` 与 `home.pgm`。检查 YAML 内 `image` 路径有效，并保留这两个文件作为一组。

## 13. Nav2 自动导航

停止 SLAM（或改用定位），保持底盘和雷达驱动运行：

```bash
ros2 launch car_bringup navigation.launch.py map:=$HOME/ros2/maps/home.yaml
ros2 run car_bringup health_check --stage nav
```

在 RViz 设 `map` 为固定坐标系，使用 **2D Pose Estimate** 初始化位姿，再用 **Nav2 Goal** 发送近距离目标。首次导航选择开阔区域、低速、有人可随时按急停。

## 验收记录

每步记录命令、终端输出、设备 ID、topic 频率与问题。下一步的前提是上一步通过；不要同时修改多个驱动参数，否则无法定位故障。
