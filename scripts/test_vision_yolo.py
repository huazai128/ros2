#!/usr/bin/env python3
"""无摄像头真实 YOLOv8 推理，并生成可供 camera_node 循环读取的视频。"""
import argparse
import json
from pathlib import Path
import time
import cv2
from ultralytics import YOLO


def main():
    """推理指定图片并保存实际检测结果；不访问串口和 GPIO。"""
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model',required=True)
    parser.add_argument('--image',required=True)
    parser.add_argument('--output',default='docs/vision_car/results/yolo')
    args=parser.parse_args()
    output=Path(args.output); output.mkdir(parents=True,exist_ok=True)
    frame=cv2.imread(args.image)
    if frame is None: raise ValueError('无法读取图片')
    model=YOLO(args.model)
    start=time.monotonic()
    result=model.predict(frame,imgsz=320,conf=.5,device='cpu',verbose=False)[0]
    seconds=time.monotonic()-start
    boxes=[dict(label=result.names[int(b.cls.item())],confidence=float(b.conf.item()),
                xyxy=b.xyxy[0].cpu().tolist()) for b in result.boxes]
    report=dict(model=str(Path(args.model).name),seconds=seconds,boxes=boxes,
                hardware_used=False)
    (output/'yolo_report.json').write_text(json.dumps(report,indent=2))
    cv2.imwrite(str(output/'annotated.jpg'),result.plot())
    frame=cv2.resize(frame,(640,480))
    writer=cv2.VideoWriter(str(output/'replay.avi'),cv2.VideoWriter_fourcc(*'MJPG'),5,(640,480))
    if not writer.isOpened(): raise RuntimeError('无法创建 MJPG 回放视频')
    try:
        for _ in range(100): writer.write(frame)
    finally: writer.release()
    print(json.dumps(report,indent=2))
    if not boxes: raise SystemExit('样本未检出目标，请检查模型和输入')


if __name__=='__main__':
    main()
