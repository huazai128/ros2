"""复用真实订阅回调验证非法 bbox 不会中断控制看门狗。"""
import json
from types import SimpleNamespace
import pytest
from vision_car.control_node import ControlNode


@pytest.mark.parametrize('payload',[None,[],12,'bad',{'ok':True},
    {'ok':True,'stamp_sec':'bad','stamp_nanosec':0}])
def test_bad_packets_fail_closed(payload):
    """畸形 JSON、缺字段与类型错误都清除旧结果并保持回调存活。"""
    fake=SimpleNamespace(packet={'old':'moving'})
    ControlNode.detection(fake,SimpleNamespace(data=json.dumps(payload)))
    assert fake.packet is None
