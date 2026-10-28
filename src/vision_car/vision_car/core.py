"""不依赖 ROS 的安全状态机，角度为度，正角表示左方。"""
import math
from dataclasses import dataclass


@dataclass
class Settings:
    """扫描、跟踪及失效保护参数。"""
    scan_min: float = -60.0
    scan_max: float = 60.0
    scan_speed: float = 25.0
    center_threshold: float = 0.12
    forward_speed: float = 0.10
    turn_speed: float = 0.35
    max_linear: float = 0.15
    max_angular: float = 0.50
    horizontal_fov: float = 62.0
    align_degrees: float = 5.0
    stop_area: float = 0.30
    stale_timeout: float = 0.7
    target_lost_timeout: float = 0.5
    pan_center_tolerance: float = 2.0

    def validate(self):
        """拒绝会破坏限速和状态机语义的配置。"""
        if not all(math.isfinite(v) for v in vars(self).values()):
            raise ValueError('参数必须为有限数值')
        if not (-85 <= self.scan_min < 0 < self.scan_max <= 85):
            raise ValueError('扫描范围应跨越零点且位于 -85..85 度')
        if not (0 < self.forward_speed <= self.max_linear <= 0.15):
            raise ValueError('前进速度不得超过统一限速及首次测试上限 0.15 m/s')
        if not (0 < self.turn_speed <= self.max_angular <= 0.5):
            raise ValueError('转向速度不得超过统一限速及首次测试上限 0.5 rad/s')
        if not (0 < self.center_threshold < 1 and 0 < self.stop_area < 1):
            raise ValueError('中心阈值及停止面积比例必须位于 (0,1)')
        if not (0 < self.horizontal_fov < 170):
            raise ValueError('水平视场角无效')
        if min(self.scan_speed, self.align_degrees, self.stale_timeout,
               self.target_lost_timeout, self.pan_center_tolerance) <= 0:
            raise ValueError('速度、时间及角度阈值必须大于零')


class Controller:
    """每次 tick 返回状态、前进速度、转向角速度和云台角。"""
    def __init__(self, settings):
        """初始化为静止扫描；未收到健康数据时进入 FAULT。"""
        settings.validate()
        self.s = settings
        self.pan = 0.0
        self.direction = 1.0
        self.state = 'FAULT'
        self.last_target = None

    def step(self, dt, now, packet, capture_pan=None):
        """拒绝异常/过期帧，丢目标先停再搜索，目标存在时先暂停扫描。"""
        s = self.s
        dt = max(0.0, min(dt, 0.2))
        if (packet is None or not packet.get('ok') or
                not 0 <= now - packet.get('received', -1e9) <= s.stale_timeout or
                not 0 <= packet.get('age', 1e9) <= s.stale_timeout):
            self.state = 'FAULT'
            return self.state, 0.0, 0.0, self.pan
        boxes = packet.get('boxes', [])
        if not boxes:
            if self.last_target is not None and now-self.last_target < s.target_lost_timeout:
                self.state = 'LOST'
            else:
                self.state = 'SCAN'
                self.pan += self.direction*s.scan_speed*dt
                if self.pan >= s.scan_max:
                    self.pan, self.direction = s.scan_max, -1.0
                elif self.pan <= s.scan_min:
                    self.pan, self.direction = s.scan_min, 1.0
            return self.state, 0.0, 0.0, self.pan
        self.last_target = now
        box = max(boxes, key=lambda b: b['confidence'])
        w, h = packet['width'], packet['height']
        x1, y1, x2, y2 = box['xyxy']
        offset = ((x1+x2)/2-w/2)/(w/2)
        bearing = (self.pan if capture_pan is None else capture_pan) - math.degrees(
            math.atan(offset*math.tan(math.radians(s.horizontal_fov/2))))
        # 捕获目标首个控制周期保持云台不动，车体速度也为零。
        if self.state in ('SCAN', 'LOST', 'FAULT'):
            self.state = 'ACQUIRE'
            return self.state, 0.0, 0.0, self.pan
        # 随车身对准将云台移回正前方，避免侧视目标导致错误直行。
        self.pan -= math.copysign(min(abs(self.pan), s.scan_speed*dt), self.pan)
        if (abs(bearing) > s.align_degrees or abs(offset) > s.center_threshold or
                abs(self.pan) > s.pan_center_tolerance):
            self.state = 'ALIGN'
            angular = max(-s.turn_speed, min(s.turn_speed, math.radians(bearing)*1.8))
            return self.state, 0.0, angular, self.pan
        area = (x2-x1)*(y2-y1)/(w*h)
        self.state = 'HOLD' if area >= s.stop_area else 'APPROACH'
        return self.state, (0.0 if self.state == 'HOLD' else s.forward_speed), 0.0, self.pan
