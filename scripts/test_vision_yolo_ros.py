#!/usr/bin/env python3
"""用视频回放验证真实 YOLO 三节点 ROS 链路；执行器固定 mock。"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import cv2
import rclpy
from ament_index_python.packages import get_package_prefix
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


def main():
    """在专用 ROS 域启动三个真节点，断言收到真实 YOLO 框和带框图像。"""
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model',required=True)
    parser.add_argument('--video',required=True)
    parser.add_argument('--output',default='docs/vision_car/results/yolo_ros')
    args=parser.parse_args()
    output=Path(args.output).resolve(); output.mkdir(parents=True,exist_ok=True)
    os.environ['ROS_DOMAIN_ID']='90'; os.environ['ROS_LOCALHOST_ONLY']='1'
    rclpy.init(); observer=Node('yolo_ros_test_observer')
    bridge=CvBridge(); detections=[]; states=[]; images=[]; processes=[]; logs=[]

    def bbox(msg):
        """仅记录成功且确为 YOLO 的检测包。"""
        packet=json.loads(msg.data)
        if packet.get('ok') and packet.get('backend')=='yolov8': detections.append(packet)

    def state(msg):
        """记录安全模拟后端的状态。"""
        states.append(json.loads(msg.data))

    def image(msg):
        """保留最新带框图像作为报告证据。"""
        images[:]=[bridge.imgmsg_to_cv2(msg,'bgr8').copy()]

    observer.create_subscription(String,'/vision/bbox',bbox,10)
    observer.create_subscription(String,'/vision/state',state,10)
    observer.create_subscription(Image,'/vision/annotated',image,qos_profile_sensor_data)
    executable=Path(get_package_prefix('vision_car'))/'lib'/'vision_car'
    options={'camera_node':['simulate:=false','fps:=5.0','video_file:='+str(Path(args.video).resolve())],
             'yolo_detect_node':['simulate:=false','model:='+str(Path(args.model).resolve())],
             'control_node':['driver:=mock']}
    report={'passed':False}
    try:
        for name,params in options.items():
            cmd=[sys.executable,str(executable/name),'--ros-args']
            for param in params: cmd+=['-p',param]
            log=(output/(name+'.log')).open('w'); logs.append(log)
            processes.append(subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
        deadline=time.monotonic()+30
        while time.monotonic()<deadline:
            rclpy.spin_once(observer,timeout_sec=.05)
            if len(detections)>=10 and images and any(s['state']!='FAULT' for s in states): break
        checks=dict(real_yolo_boxes=any(d['boxes'] for d in detections),
                    annotated_image=bool(images),
                    control_consumed_real_detection=any(s['state'] in ('ACQUIRE','ALIGN','APPROACH','HOLD') for s in states),
                    mock_driver_only=bool(states) and all(s['driver']=='mock' and not s['hardware_enabled'] for s in states))
        report=dict(passed=all(checks.values()),checks=checks,
            ros_distro=os.environ.get('ROS_DISTRO'),packets=len(detections),
            labels=sorted({b['label'] for d in detections for b in d['boxes']}),
            states=sorted({s['state'] for s in states}),hardware_used=False)
    finally:
        for proc in processes:
            if proc.poll() is None: os.killpg(proc.pid,signal.SIGTERM)
            try: proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid,signal.SIGKILL); proc.wait(timeout=2)
        for log in logs: log.close()
        if images: cv2.imwrite(str(output/'annotated.jpg'),images[0])
        (output/'report.json').write_text(json.dumps(report,indent=2))
        observer.destroy_node(); rclpy.shutdown()
    print(json.dumps(report,indent=2))
    if not report['passed']: raise SystemExit(1)


if __name__=='__main__':
    main()
