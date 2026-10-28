"""无硬件状态机验收，覆盖限速、坐标方向和故障停车。"""
import math
import pytest
from vision_car.core import Controller, Settings


def packet(now, x=320, size=80, boxes=True):
    """构造已验证的控制输入。"""
    return dict(ok=True, received=now, age=0, width=640, height=480,
        boxes=[dict(confidence=0.9,xyxy=[x-size/2,240-size/2,x+size/2,240+size/2])]
        if boxes else [])


def test_scan_limits_and_stationary():
    """扫描可左右往复，任何扫描周期底盘均静止。"""
    c=Controller(Settings())
    angles=[]
    for i in range(1500):
        state,v,w,pan=c.step(0.05,i*.05,packet(i*.05,boxes=False))
        assert state=='SCAN' and v==w==0
        assert -60<=pan<=60
        angles.append(pan)
    assert min(angles)==-60 and max(angles)==60


def test_pause_and_turn_direction():
    """捕获瞬间冻结云台；右侧目标使车右转。"""
    c=Controller(Settings())
    c.step(.05,0,packet(0,boxes=False))
    before=c.pan
    assert c.step(.05,.05,packet(.05,x=500))==('ACQUIRE',0,0,before)
    state,v,w,_=c.step(.05,.1,packet(.1,x=500))
    assert state=='ALIGN' and v==0 and w<0


def test_off_axis_camera_must_not_advance():
    """画面居中但云台侧视，必须先转车体，不能前进。"""
    c=Controller(Settings())
    c.pan=45
    c.step(.05,0,packet(0))
    state,v,w,_=c.step(.05,.05,packet(.05),45)
    assert state=='ALIGN' and v==0 and w>0


def test_approach_hold_lost_and_fault():
    """验证居中前进、近目标保持、丢失停车和恢复扫描。"""
    c=Controller(Settings())
    c.step(.05,0,packet(0))
    assert c.step(.05,.1,packet(.1))[:3]==('APPROACH',.1,0)
    assert c.step(.05,.2,packet(.2,size=330))[:3]==('HOLD',0,0)
    assert c.step(.05,.3,packet(.3,boxes=False))[:3]==('LOST',0,0)
    assert c.step(.05,1,packet(1,boxes=False))[:3]==('SCAN',0,0)
    assert c.step(.05,2,None)[:3]==('FAULT',0,0)
    assert c.step(.05,3,packet(1))[:3]==('FAULT',0,0)
    p=packet(3); p['age']=10
    assert c.step(.05,3,p)[:3]==('FAULT',0,0)


@pytest.mark.parametrize('settings', [Settings(scan_speed=0),Settings(forward_speed=.2),
    Settings(turn_speed=.6),Settings(scan_min=10),Settings(center_threshold=1),
    Settings(stale_timeout=-1),Settings(horizontal_fov=180),Settings(scan_speed=math.nan)])
def test_invalid_settings(settings):
    """非法参数必须在启动时失败。"""
    with pytest.raises(ValueError):
        Controller(settings)
