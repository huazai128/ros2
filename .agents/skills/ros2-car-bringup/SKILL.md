---
name: ros2-car-bringup
description: Build, configure, diagnose, or document this ROS 2 mobile-robot project, including staged hardware bringup, SLAM, Nav2, and operator-facing run instructions.
metadata:
  short-description: ROS 2 小车分阶段搭建与输出规范
---

# ROS 2 小车项目规范

使用本 Skill 处理本仓库的代码、配置、故障诊断、运行说明或硬件操作请求。目标是在实体车上以可定位、可回退的顺序完成 bringup，而不是假定硬件接口存在。

## 先读什么

1. 阅读 `README.md` 了解包结构。
2. 阅读 `config/robot.yaml`，它是话题、坐标系、速度限值的单一配置入口。
3. 对需要执行或解释的阶段，阅读 [分阶段清单](references/staged-checklist.md)。
4. 在回复用户前，遵守 [输出契约](references/output-contract.md)。

## 强制约束

- 按阶段 1 到 13 顺序工作；只有上阶段有明确验收证据才能建议执行下一阶段。
- 在不知道厂商包、串口 ID、ROS 发行版或 topic/frame 名称时，使用占位符并明确指出需要替换；绝不编造可运行的硬件命令。
- 首次电机测试必须要求急停可用、驱动轮架空、低速值不超过 `robot.yaml` 的限制。不得自动发送实际运动指令。
- 话题兼容性优先通过驱动参数/remap 解决。变更 frame、速度、端口或 Nav2 参数时，同时更新相关文档和 `robot.yaml`。
- 修改 launch 或 Python 后，至少执行可用的静态检查；若环境没有 ROS 2，说明未执行的真实集成验证及原因。
- 每次修改项目文件后，在最终回复前运行 `./scripts/sync_to_ubuntu.sh`，将源文件增量同步到 `zhouweihua@192.168.64.2:~/ros2`。同步失败须如实报告；除非用户明确同意，不得传入 `--delete`。

## 项目接口

| 内容 | 位置 | 用法 |
| --- | --- | --- |
| 硬件名称/限速 | `src/car_bringup/config/robot.yaml` | 根据实际驱动更新 topic 与 frame；改后重新构建。 |
| 阶段健康检查 | `ros2 run car_bringup health_check --stage <stage>` | 只读检查 ROS 图谱；`stage` 为 base、odometry、imu、sensors、tf、slam、nav。 |
| 建图 | `ros2 launch car_bringup slam.launch.py` | 仅在底盘、里程计、TF、雷达均通过后启动。 |
| 导航 | `ros2 launch car_bringup navigation.launch.py map:=<绝对地图YAML路径>` | 仅使用已保存地图；必须先在 RViz 做初始位姿。 |
| 操作手册 | `docs/RUNBOOK.md` | 改动运行行为、依赖或接口时同步维护。 |
| Ubuntu 同步 | `scripts/sync_to_ubuntu.sh` | 修改项目后运行；可用 `ROS2_REMOTE_*` 环境变量覆盖默认目标。 |

## 配置与文档改动规则

- 增加硬件：记录稳定的 `/dev/serial/by-id/...` 标识、驱动包、发布/订阅话题、frame 和启动命令。
- 新增诊断：定义通过/失败条件与不改变机器人状态的检查命令。
- 新增 launch：声明所有可调整参数；不要把机器相关路径、IP 或端口硬编码。
- 更改 Nav2：基于发行版完整参数文件，不用精简文件覆盖全部参数。

需要逐项说明或执行硬件 bringup 时，读取 [分阶段清单](references/staged-checklist.md)。
