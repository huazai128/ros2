# 软件验收记录（2026-10-28）

## 2026-10-30 调度优化复验

本次在 Mac 本地运行：新增队列/事件/时效回归测试，以及既有状态机和串口替身测试，
共 27 项通过；Python 静态编译和 `git diff --check` 通过。
新调度测试替代 ROS、OpenCV 依赖，仅验证真实方法的单元行为，不代表 ROS 集成测试。
格式化范围为相机、检测、控制、链路工具、launch 和新增测试；模型、限速和控制频率未调整。

本地无 rclpy；远端 `zhouweihua@192.168.64.2` 的 SSH 认证失败
（Permission denied (publickey,password)），故未完成本次 Ubuntu 编译、三节点回放、故障注入和实车延迟测量。
下列 2026-10-28 记录是历史版本的证据，不能视为此次事件通知改动的集成验收。

## 2026-10-28 历史验收

本次所有执行器均为 mock，没有连接、刷写或驱动实体硬件。测试环境为 aarch64 Ubuntu 24.04 / ROS 2 Jazzy / Python 3.12；Humble/Ubuntu 22.04 及真实香橙派尚未实测。

| 验收项 | 结果 | 证据 |
|---|---|---|
| 本机 colcon 编译 car_bringup + vision_car | 通过 | 两个包完成 |
| 同步后 Ubuntu 虚拟机 colcon 编译 | 通过 | 两个包完成 |
| 状态机、畸形 JSON、串口握手/锁止单元测试 | 19 项通过 | 本机与 Ubuntu 均通过 |
| 本机三节点模拟与故障注入 | 8 项通过 | [ros_report.json](results/ros_report.json)，670 条状态样本 |
| Ubuntu 三节点模拟与故障注入 | 8 项通过 | [ubuntu_ros_report.json](results/ubuntu_ros_report.json)，668 条状态样本 |
| 真实 YOLOv8n CPU 图片检测 | 通过 | [yolo_report.json](results/yolo/yolo_report.json)，3 person + 1 bus，首次推理约 1.06 秒 |
| 视频回放 + 真 YOLO + 三 ROS 节点 | 4 项通过 | [report.json](results/yolo_ros/report.json)，18 个有效结果包，mock 执行器 |
| ESP32 目标固件编译 | 通过 | [firmware_build.json](results/firmware_build.json)，PlatformIO espressif32@6.9.0，未上传 |
| Python 编译/函数说明/补丁格式 | 通过 | compileall、AST 函数 docstring 检查、git diff --check |

故障注入包括终止相机进程、终止检测进程，以及模拟推理耗时 0.8 秒超过 0.2 秒测试限值；控制器在三种情况下均转入 FAULT，线/角速度为零。状态机测试覆盖扫描左右极限、扫描时底盘静止、捕获冻结、左右转向符号、摄像头偏转时禁止前进、目标过近保持、丢失重扫、陈旧数据及非法参数。串口测试使用 FakePort，没有打开真实设备。

[模拟状态截图](results/simulation.jpg) 和 [真实 YOLO 带框图](results/yolo/annotated.jpg) 均由实际软件运行生成。合成目标是预设图像序列，不是运动学闭环或物理仿真；真实图片/回放识别通过不等于真实相机环境准确率验收。

完整临时验证环境的 pip 包版本见 [validation-environment-py312-arm64.txt](results/validation-environment-py312-arm64.txt)，包括开发用 PlatformIO。它仅记录 Python 3.12 arm64 本次验证环境，不应替代面向 Humble/Python 3.10 的 `requirements-yolo.txt`。当前开发机的 python3-venv/ensurepip/pip 缺失，通过仅位于 /tmp 的 venv 和官方 pip 引导脚本完成模型验证；未安装系统级依赖。临时环境继承的 PyNaCl 缺少 cffi，已在临时 venv 补齐后复查依赖。

尚需现场验收：实际香橙派板型与 Ubuntu/Humble 镜像、USB 相机稳定路径/帧率、舵机供电和极限、PWM 复用、GPIO line offset、串口 ID、轮距和开环轮速标定、硬件急停/看门狗及架空低速方向测试。GPIO/PWM 或 ESP32 输出的电气波形、故障停车的实际制动距离尚未测量。
