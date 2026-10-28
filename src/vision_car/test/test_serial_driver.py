"""串口后端握手和 ACK 中断锁止，不打开实体串口。"""
from unittest.mock import patch
from vision_car.drivers import SerialDriver


class FakePort:
    """提供可控应答的串口替身。"""
    def __init__(self, *args, **kwargs):
        """记录所有输出帧。"""
        self.reply=b''; self.sent=[]; self.closed=False

    def read(self, count):
        """一次性交付测试注入的 ACK。"""
        reply,self.reply=self.reply,b''
        return reply

    def write(self, data):
        """缓存主机实际编码的协议帧。"""
        self.sent.append(data)
        return len(data)

    def close(self):
        """标记串口已关闭。"""
        self.closed=True


def test_handshake_watchdog_latches():
    """首次只发零速，ACK 允许运动，断 ACK 后即使恢复也不自动运动。"""
    now=[0.0]
    with patch('serial.Serial',FakePort), patch('vision_car.drivers.time.monotonic',lambda:now[0]):
        driver=SerialDriver(dict(serial_port='/dev/serial/by-id/test',baudrate=115200,ack_timeout=.4))
        assert not driver.send(.1,.2,10)
        assert driver.port.sent[-1]==b'V 0 0.0000 0.0000 10.00\n'
        now[0]=.05; driver.port.reply=b'ACK 0\n'
        assert driver.send(.1,.2,10)
        assert b'0.1000 0.2000' in driver.port.sent[-1]
        now[0]=1
        assert not driver.send(.1,.2,10)
        assert b'0.0000 0.0000' in driver.port.sent[-1]
        driver.port.reply=b'ACK 2\n'; now[0]=1.05
        assert not driver.send(.1,.2,10)
        driver.close()
        assert driver.port.closed and driver.port.sent[-1].endswith(b'0 0 0\n')
