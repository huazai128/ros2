#!/usr/bin/env python3
"""无设备 ROS 集成验收：运行三个真节点，验证状态及进程失联保护。"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import cv2
import rclpy
import yaml
from ament_index_python.packages import get_package_prefix
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


def main():
    """运行合成相机/模拟检测链路，保存报告和带框截图，不连接硬件。"""
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',default='docs/vision_car/results')
    args=parser.parse_args()
    output=Path(args.output).resolve(); output.mkdir(parents=True,exist_ok=True)
    os.environ.setdefault('ROS_DOMAIN_ID','89')
    os.environ['ROS_LOCALHOST_ONLY']='1'
    rclpy.init()
    node=Node('vision_test_observer')
    states=[]; images=[]; bridge=CvBridge(); processes={}; logs=[]
    start=time.monotonic()

    def state(msg):
        """采样所有状态及速度，保存单调时间便于验收。"""
        value=json.loads(msg.data); value['time']=round(time.monotonic()-start,3)
        states.append(value)

    def image(msg):
        """每个状态仅保留一张带框画面。"""
        current=states[-1]['state'] if states else 'START'
        if current not in [item[0] for item in images]:
            images.append((current,bridge.imgmsg_to_cv2(msg,'bgr8').copy()))

    node.create_subscription(String,'/vision/state',state,10)
    node.create_subscription(Image,'/vision/annotated',image,qos_profile_sensor_data)
    executable=Path(get_package_prefix('vision_car'))/'lib'/'vision_car'

    def launch(name, extra=None):
        """以独立进程组启动单节点，以便模拟相机/推理进程中断。"""
        cmd=[str(executable/name)]
        if extra: cmd+=['--ros-args']+extra
        log=(output/(name+'.log')).open('w'); logs.append(log)
        processes[name]=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)

    def stop(name):
        """终止节点及它的采集/推理子进程。"""
        proc=processes.pop(name,None)
        if proc is not None:
            if proc.poll() is None:
                os.killpg(proc.pid,signal.SIGTERM)
            try: proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid,signal.SIGKILL); proc.wait(timeout=2)

    def spin(seconds):
        """观察真实 ROS 消息一段时间。"""
        until=time.monotonic()+seconds
        while time.monotonic()<until:
            rclpy.spin_once(node,timeout_sec=.05)

    report={'backend':'ROS 2 '+os.environ.get('ROS_DISTRO','unknown'),
            'detector':'mock green rectangle; NOT YOLO accuracy validation','checks':{}}
    try:
        for name in ('control_node','yolo_detect_node','camera_node'): launch(name)
        spin(19)
        required={'SCAN','ACQUIRE','ALIGN','APPROACH','HOLD','LOST'}
        seen={s['state'] for s in states}
        report['checks']['all_tracking_states']=required.issubset(seen)
        report['checks']['stationary_when_not_tracking']=all(s['linear']==s['angular']==0
            for s in states if s['state'] in ('SCAN','LOST','FAULT','ACQUIRE','HOLD'))
        report['checks']['speed_limits']=all(abs(s['linear'])<=.15 and abs(s['angular'])<=.5 for s in states)
        report['checks']['three_nodes']=all((name,'/') in node.get_node_names_and_namespaces()
            for name in ('camera_node','yolo_detect_node','control_node'))
        stop('camera_node'); begin=len(states); spin(2)
        tail=states[begin:]
        report['checks']['camera_disconnect_stop']=bool(tail) and all(
            s['state']=='FAULT' and s['linear']==s['angular']==0 for s in tail[-10:])
        launch('camera_node'); spin(5)
        stop('yolo_detect_node'); begin=len(states); spin(2)
        tail=states[begin:]
        report['checks']['detector_disconnect_stop']=bool(tail) and all(
            s['state']=='FAULT' and s['linear']==s['angular']==0 for s in tail[-10:])
        # 特意让推理耗时超过超时值，验证控制节点仍持续运行并停车。
        launch('yolo_detect_node',['-p','simulate_inference_delay:=0.8','-p','inference_timeout:=0.2'])
        spin(5)
        report['checks']['inference_timeout_reported']='推理超时' in (output/'yolo_detect_node.log').read_text()
        report['checks']['inference_timeout_stop']=all(s['state']=='FAULT' and s['linear']==s['angular']==0
            for s in states[-10:])
        report['states']=sorted(seen); report['samples']=len(states)
        report['passed']=all(report['checks'].values())
    finally:
        for name in list(processes): stop(name)
        for log in logs: log.close()
        node.destroy_node(); rclpy.shutdown()
        (output/'ros_report.json').write_text(json.dumps(report,indent=2,ensure_ascii=False))
        (output/'state_trace.json').write_text(json.dumps(states,indent=2))
        tiles=[]
        for title,frame in images:
            frame=cv2.resize(frame,(320,240))
            cv2.putText(frame,title,(8,25),cv2.FONT_HERSHEY_SIMPLEX,.7,(255,255,255),2)
            tiles.append(frame)
        if tiles:
            import numpy as np
            while len(tiles)%3: tiles.append(np.zeros_like(tiles[0]))
            sheet=np.vstack([np.hstack(tiles[i:i+3]) for i in range(0,len(tiles),3)])
            cv2.imwrite(str(output/'simulation.jpg'),sheet)
    print(json.dumps(report,indent=2,ensure_ascii=False))
    if not report.get('passed'): raise SystemExit(1)


if __name__=='__main__':
    main()
