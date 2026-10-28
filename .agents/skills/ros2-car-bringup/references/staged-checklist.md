# 分阶段清单：说明、用法与验收

每一项均是下一项的前置条件。命令中的 `<...>` 表示需替换的真实值。

| 阶段 | 备注说明 | 用法 | 通过标准 |
| --- | --- | --- | --- |
| 1. 供电与电池 | 排除低压、反接、急停失效等高风险问题。 | 断电检查线束/保险，测电池电压，确认急停可切断驱动。 | 电压在底盘规格内，无发热和松动，急停有效。 |
| 2. 主控 Ubuntu | 先确认主控操作系统和 ROS 环境，不把启动故障误判为硬件问题。 | `hostnamectl`、`lsb_release -a`、`printenv ROS_DISTRO`。 | 能正常登录，ROS 发行版符合驱动支持范围。 |
| 3. Mac SSH | 保证远程操作通道可复现。 | Mac 执行 `ssh <用户>@<小车IP>`；统一网络和 `ROS_DOMAIN_ID`。 | 连接稳定且可打开远端 ROS 终端。 |
| 4. 串口/USB | 查清设备枚举和权限，避免不稳定的 tty 编号。 | `ls -l /dev/serial/by-id/`、`lsusb`、`dmesg -w`。 | 能找到稳定设备 ID；当前用户有串口权限。 |
| 5. 底盘驱动 | 让底盘节点独立启动，限定问题范围。 | 使用厂商命令：`ros2 launch <驱动包> <启动文件> port:=/dev/serial/by-id/<设备>`。 | 无持续报错，能发现 `/cmd_vel`、`/odom`、`/tf` 或有明确 remap。 |
| 6. 低速遥控 | 在导航介入前验证电机、方向和急停。 | 车轮架空时运行 `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.15 -p turn:=0.5`。 | 前后左右方向正确，`k` 停止有效；再进行低速落地测试。 |
| 7. 编码器/里程计 | 里程计是 SLAM/导航的基础。 | `ros2 topic echo /odom --once`、`ros2 topic hz /odom`、`ros2 run car_bringup health_check --stage odometry`。 | 缓慢移动时位姿连续，静止时无明显漂移。 |
| 8. IMU | 验证安装方向和数据质量。 | `ros2 topic echo /imu/data --once`、`ros2 run car_bringup health_check --stage imu`。 | 平放时重力加速度合理，旋转时角速度变化正确。 |
| 9. 雷达或摄像头 | 确认环境感知输入可用。 | 雷达：`ros2 topic echo /scan --once`；相机：`ros2 topic hz /camera/image_raw`；再运行 `--stage sensors`。 | 至少一个输入稳定发布，且其 `frame_id` 可被 TF 解析。 |
| 10. RViz 与 TF | 可视化验证完整坐标链，避免建图时坐标断裂。 | `rviz2`、`ros2 run tf2_tools view_frames`；先固定 `odom`。 | 有 `odom → base_link → sensor_frame` 连通链。 |
| 11. SLAM 建图 | 将已验证的输入接入建图。 | `ros2 launch car_bringup slam.launch.py`、`ros2 run car_bringup health_check --stage slam`。 | RViz 出现并持续更新 `/map`，低速闭环时地图稳定。 |
| 12. 保存地图 | 将验证过的地图持久化供定位使用。 | `ros2 run nav2_map_server map_saver_cli -f ~/ros2/maps/<地图名>`。 | 同时生成并保留 `.yaml` 和 `.pgm`，YAML 中 image 路径有效。 |
| 13. Nav2 导航 | 在已知地图中完成定位与目标导航。 | `ros2 launch car_bringup navigation.launch.py map:=<绝对路径>.yaml`；RViz 使用 2D Pose Estimate 再发送 Nav2 Goal。 | 近距离开阔区域内路径与速度正常，随时可急停。 |

## 常用失败分流

- 找不到设备：回到阶段 4，不要重装 SLAM/Nav2。
- 遥控不动或方向反：回到阶段 5–6，检查驱动端口、使能、左右轮符号和速度限制。
- 里程计/IMU 不合理：回到阶段 7–8，先修轮径轮距、编码器与安装方向。
- RViz 无传感器或固定坐标报错：回到阶段 9–10，检查 frame 与静态/动态 TF。
- 地图撕裂或导航失败：只有前面均通过后，再检查 SLAM/Nav2 参数、环境和打滑。
