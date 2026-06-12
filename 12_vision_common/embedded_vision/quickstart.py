# -*- coding: utf-8 -*-
"""
嵌入式视觉代码库 - 快速启动示例
==================================
演示所有模块的基本用法

运行: python quickstart.py
"""

import cv2
import numpy as np
import sys
import os

# 确保模块可导入
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def demo_all():
    """综合演示所有模块"""
    from embedded_vision.platform_utils import CameraThread, FrameCounter, optimize_opencv
    from embedded_vision.object_detector import MobileNetSSDDetector
    from embedded_vision.deepsort_tracker import SimpleDeepSORT
    from embedded_vision.semantic_segmentation import ColorSegmentor, RoadDetector
    
    optimize_opencv()
    cam = CameraThread(src=0, width=640, height=480).start()
    
    detector = MobileNetSSDDetector(conf_threshold=0.4)
    tracker = SimpleDeepSORT(iou_threshold=0.3, max_age=20)
    segmentor = ColorSegmentor()
    road = RoadDetector()
    counter = FrameCounter()
    
    mode = 0
    modes = ["检测", "跟踪", "分割", "循迹"]
    
    print("=" * 50)
    print("嵌入式视觉综合演示")
    print("按数字1-4切换模式, 按q退出")
    print("1:目标检测  2:目标跟踪  3:语义分割  4:循迹")
    print("=" * 50)
    
    while True:
        frame = cam.read()
        if frame is None:
            continue
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('1'):
            mode = 0
        elif key == ord('2'):
            mode = 1
        elif key == ord('3'):
            mode = 2
        elif key == ord('4'):
            mode = 3
        
        if mode == 0:
            # 目标检测
            results = detector.detect(frame)
            display = detector.draw_results(frame.copy(), results)
        elif mode == 1:
            # 目标跟踪
            detections = detector.detect(frame)
            tracks = tracker.update(detections)
            display = tracker.draw_tracks(frame.copy(), tracks)
        elif mode == 2:
            # 语义分割
            mask = segmentor.segment(frame)
            vis = segmentor.visualize(mask)
            display = cv2.addWeighted(frame, 0.5, vis, 0.5, 0)
        elif mode == 3:
            # 循迹
            info = road.detect(frame)
            vis = segmentor.visualize(info["mask"])
            display = cv2.addWeighted(frame, 0.5, vis, 0.5, 0)
            cv2.putText(display, f"Drive:{info['drivable_area']:.2f}", 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
        
        counter.tick()
        cv2.putText(display, f"FPS:{counter.fps:.1f} [{modes[mode]}]", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
        cv2.imshow("Embedded Vision", display)
    
    cam.stop()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    demo_all()
