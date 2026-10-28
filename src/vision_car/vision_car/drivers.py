"""执行后端：默认模拟；实体串口包含应答监视，直连需外部硬件看门狗。"""
import math
import time
from pathlib import Path


class MockDriver:
    """不访问任何设备的模拟驱动。"""
    def send(self, linear, angular, pan):
        """模拟指令始终成功。"""
        return True

    def close(self):
        """模拟后端无需释放硬件。"""


class SerialDriver:
    """ASCII 行协议：V seq linear angular pan，ACK seq。"""
    def __init__(self, p):
        """打开稳定串口路径，初始化为锁止状态。"""
        import serial
        if not p['serial_port'].startswith('/dev/serial/by-id/'):
            raise ValueError('实体串口必须设置 /dev/serial/by-id/ 稳定路径')
        self.port = serial.Serial(p['serial_port'], p['baudrate'], timeout=0, write_timeout=0.05)
        self.seq = 0
        self.last_ack = time.monotonic()
        self.started = self.last_ack
        self.have_ack = False
        self.buffer = b''
        self.timeout = p['ack_timeout']
        self.latest_ack = -1
        self.failed = False

    def send(self, linear, angular, pan):
        """未握手时只发零速；ACK 中断后锁止到重启，不自动恢复运动。"""
        self.buffer = (self.buffer + self.port.read(4096))[-8192:]
        while b'\n' in self.buffer:
            line, self.buffer = self.buffer.split(b'\n', 1)
            parts = line.split()
            if len(parts) == 2 and parts[0] == b'ACK':
                try:
                    seq = int(parts[1])
                except ValueError:
                    continue
                if self.latest_ack < seq < self.seq and self.seq-seq <= 10:
                    self.last_ack, self.latest_ack = time.monotonic(), seq
                    self.have_ack = True
        # ESP32 打开 USB 串口后可能重启，给 3 秒启动期，期间只发停车。
        if ((self.have_ack and time.monotonic()-self.last_ack > self.timeout) or
                (not self.have_ack and time.monotonic()-self.started > 3.0)):
            self.failed = True
        healthy = self.have_ack and not self.failed
        v, w = (linear, angular) if healthy else (0.0, 0.0)
        self.port.write(f'V {self.seq} {v:.4f} {w:.4f} {pan:.2f}\n'.encode('ascii'))
        self.seq += 1
        return healthy

    def close(self):
        """尽力发送零速；断线时 ESP32 自身 300ms 看门狗负责停车。"""
        try:
            self.port.write(f'V {self.seq} 0 0 0\n'.encode('ascii'))
        finally:
            self.port.close()


class PWM:
    """Linux sysfs PWM 通道，通道需由用户按实际板卡导出。"""
    def __init__(self, path, period):
        """校验已导出的通道并在禁用状态设置周期。"""
        self.path = Path(path)
        if not path or not (self.path/'enable').is_file():
            raise ValueError('必须配置实际已导出的 PWM 通道绝对路径')
        self.period = period
        self.write('enable', 0)
        self.write('duty_cycle', 0)
        self.write('period', period)
        self.write('enable', 1)

    def write(self, name, value):
        """写 PWM sysfs 属性，权限错误直接向上报告。"""
        (self.path/name).write_text(str(int(value)))

    def duty(self, ratio):
        """设置 0..1 占空比。"""
        self.write('duty_cycle', self.period*max(0, min(1, ratio)))

    def close(self):
        """禁用 PWM 输出。"""
        self.duty(0)
        self.write('enable', 0)


class LinuxDriver:
    """gpiod v1 方向脚 + 三路硬件 PWM；型号和引脚必须现场确认。"""
    def __init__(self, p):
        """先申请低电平方向脚，再启用零占空比电机 PWM。"""
        import gpiod
        if not p['external_watchdog_confirmed']:
            raise ValueError('直连 L298N 必须有可切断 ENA/ENB 的外部看门狗')
        if len({p['left_pwm'],p['right_pwm'],p['servo_pwm']}) != 3:
            raise ValueError('左右电机和舵机必须使用三个独立 PWM 通道')
        self.p, self.pwms, self.lines, self.chip = p, [], None, None
        try:
            if len(p['direction_lines']) != 4 or min(p['direction_lines']) < 0:
                raise ValueError('direction_lines 必须是四个实测 GPIO line offset')
            self.chip = gpiod.Chip(p['gpio_chip'])
            self.lines = self.chip.get_lines(p['direction_lines'])
            self.lines.request(consumer='vision-car', type=gpiod.LINE_REQ_DIR_OUT,
                               default_vals=[0, 0, 0, 0])
            for path, period in [(p['left_pwm'], 50000), (p['right_pwm'], 50000),
                                 (p['servo_pwm'], 20000000)]:
                self.pwms.append(PWM(path, period))
        except Exception:
            self.close()
            raise

    def send(self, linear, angular, pan):
        """差速混控并等比饱和；电机换向前先撤销使能 PWM。"""
        p = self.p
        left = (linear-angular*p['wheel_track']/2)/p['wheel_max_speed']
        right = (linear+angular*p['wheel_track']/2)/p['wheel_max_speed']
        scale = max(1, abs(left), abs(right))
        left, right = left/scale*p['left_sign'], right/scale*p['right_sign']
        self.pwms[0].duty(0)
        self.pwms[1].duty(0)
        self.lines.set_values([int(left>0), int(left<0), int(right>0), int(right<0)])
        self.pwms[0].duty(abs(left))
        self.pwms[1].duty(abs(right))
        pulse = p['servo_center_us'] + p['servo_sign']*pan*p['servo_us_per_degree']
        pulse = max(p['servo_min_us'], min(p['servo_max_us'], pulse))
        self.pwms[2].duty(pulse/20000)
        return True

    def close(self):
        """逐项尽力撤销 PWM/方向输出，避免一个失败阻止其余清理。"""
        for pwm in self.pwms:
            try:
                pwm.close()
            except OSError:
                pass
        if self.lines is not None:
            try:
                self.lines.set_values([0, 0, 0, 0])
                self.lines.release()
            except OSError:
                pass
        if self.chip is not None:
            self.chip.close()
