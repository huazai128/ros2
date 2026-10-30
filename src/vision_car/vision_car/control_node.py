"""检测订阅、安全状态机、云台和底盘输出集中在一个控制节点。"""

import json
import math
import time
from collections import deque
from dataclasses import asdict
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String
from .common import parameters, run
from .core import Controller, Settings
from .drivers import MockDriver, SerialDriver, LinuxDriver


class ControlNode(Node):
    """默认仅模拟，真实设备需要显式使能及完成阶段验收。"""

    def __init__(self):
        """读取统一配置并创建硬件后端。"""
        super().__init__("control_node")
        defaults = dict(
            asdict(Settings()),
            bbox_topic="/vision/bbox",
            command_topic="/vision/cmd_vel",
            state_topic="/vision/state",
            driver="mock",
            hardware_enabled=False,
            control_hz=20.0,
            serial_port="",
            baudrate=115200,
            ack_timeout=0.4,
            gpio_chip="/dev/gpiochip0",
            direction_lines=[-1, -1, -1, -1],
            left_pwm="",
            right_pwm="",
            servo_pwm="",
            external_watchdog_confirmed=False,
            wheel_track=0.18,
            wheel_max_speed=0.30,
            left_sign=1,
            right_sign=1,
            servo_center_us=1500.0,
            servo_us_per_degree=10.0,
            servo_min_us=600.0,
            servo_max_us=2400.0,
            servo_sign=1,
        )
        self.p = parameters(self, defaults)
        self.controller = Controller(
            Settings(**{k: self.p[k] for k in asdict(Settings())})
        )
        if not (
            1 <= self.p["control_hz"] <= 100
            and self.p["wheel_track"] > 0
            and self.p["wheel_max_speed"] > 0
            and self.p["ack_timeout"] > 0
            and self.p["left_sign"] in (-1, 1)
            and self.p["right_sign"] in (-1, 1)
            and self.p["servo_sign"] in (-1, 1)
            and 0
            < self.p["servo_min_us"]
            < self.p["servo_center_us"]
            < self.p["servo_max_us"]
            < 20000
            and self.p["servo_us_per_degree"] > 0
        ):
            raise ValueError("硬件/控制频率参数无效")
        if self.p["driver"] not in ("mock", "serial", "linux"):
            raise ValueError("未知硬件后端")
        if self.p["driver"] != "mock" and not self.p["hardware_enabled"]:
            raise ValueError("硬件默认锁止；需先完成供电、急停、架空和低速验收")
        self.driver = {
            "mock": MockDriver,
            "serial": SerialDriver,
            "linux": LinuxDriver,
        }[self.p["driver"]](**({} if self.p["driver"] == "mock" else {"p": self.p}))
        self.packet = None
        self.failed = False
        self.last_stamp = -1.0
        self.history = deque(maxlen=400)
        self.last_tick = time.monotonic()
        self.last_state = None
        self.create_subscription(String, self.p["bbox_topic"], self.detection, 1)
        self.pub = self.create_publisher(Twist, self.p["command_topic"], 1)
        self.state_pub = self.create_publisher(String, self.p["state_topic"], 10)
        self.create_timer(1 / self.p["control_hz"], self.tick)

    def detection(self, msg):
        """校验 JSON 框、时间戳和尺寸；无效消息直接进入故障停车。"""
        try:
            packet = json.loads(msg.data)
            if packet.get("ok") is not True:
                self.packet = None
                return
            stamp = packet["stamp_sec"] + packet["stamp_nanosec"] / 1e9
            ros_now = self.get_clock().now().nanoseconds / 1e9
            age = ros_now - stamp
            if not math.isfinite(stamp) or not 0 <= age <= self.p["stale_timeout"]:
                raise ValueError("过期或未来时间戳")
            if stamp <= self.last_stamp:
                raise ValueError("重复或乱序结果")
            w, h = packet["width"], packet["height"]
            if not (0 < w <= 16384 and 0 < h <= 16384):
                raise ValueError("图像尺寸非法")
            for box in packet["boxes"]:
                x1, y1, x2, y2 = box["xyxy"]
                if not all(
                    math.isfinite(v) for v in [x1, y1, x2, y2, box["confidence"]]
                ):
                    raise ValueError("非法数值")
                if not (
                    0 <= x1 < x2 <= w
                    and 0 <= y1 < y2 <= h
                    and 0 <= box["confidence"] <= 1
                ):
                    raise ValueError("框超出图像范围")
            packet.update(received=time.monotonic(), age=age, stamp=stamp)
            self.packet, self.last_stamp = packet, stamp
        except (ValueError, KeyError, TypeError, OverflowError, AttributeError):
            self.packet = None

    def tick(self):
        """以固定周期输出控制，消息中断也能独立触发停车。"""
        now = time.monotonic()
        dt, self.last_tick = now - self.last_tick, now
        ros_now = self.get_clock().now().nanoseconds / 1e9
        capture_pan = None
        if self.packet is not None:
            self.packet["age"] = ros_now - self.packet["stamp"]
            if self.history:
                capture_pan = min(
                    self.history, key=lambda item: abs(item[0] - self.packet["stamp"])
                )[1]
        state, v, w, pan = self.controller.step(dt, now, self.packet, capture_pan)
        if self.failed:
            state, v, w = "DRIVER_FAULT", 0.0, 0.0
        try:
            if not self.failed and not self.driver.send(v, w, pan):
                state, v, w = "DRIVER_WAIT", 0.0, 0.0
        except Exception as exc:
            self.failed = True
            state, v, w = "DRIVER_FAULT", 0.0, 0.0
            self.get_logger().error("驱动锁止，需排障并重启: " + str(exc))
            try:
                self.driver.close()
            except Exception:
                pass
        self.history.append((ros_now, pan))
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = float(v), float(w)
        self.pub.publish(cmd)
        output = dict(
            state=state,
            linear=v,
            angular=w,
            pan=pan,
            driver=self.p["driver"],
            hardware_enabled=self.p["hardware_enabled"],
            detection_age_ms=(
                self.packet["age"] * 1000 if self.packet is not None else None
            ),
            tick_interval_ms=dt * 1000,
        )
        self.state_pub.publish(String(data=json.dumps(output)))
        if state != self.last_state:
            self.get_logger().info(json.dumps(output))
            self.last_state = state

    def destroy_node(self):
        """退出时先停车，再释放资源。"""
        try:
            self.driver.close()
        except Exception as exc:
            self.get_logger().error("退出停车写入失败: " + str(exc))
        if self.context.ok():
            self.pub.publish(Twist())
        return super().destroy_node()


def main():
    """control_node 入口。"""
    run(ControlNode)
