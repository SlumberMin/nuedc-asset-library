# -*- coding: utf-8 -*-
"""
平台优化工具 - Orange Pi 5 / RK3588
====================================
功能:
  - OpenCV多线程/NEON优化开关
  - 帧率统计
  - 帧采集线程(解耦采集与推理)
  - 模型自动选择(RKNN / ONNX / TensorRT)

技术背景:
  1. OpenCV ARM优化: cv2.setNumThreads() + NEON编译
  2. RK3588 NPU: 6 TOPS INT8, 支持RKNN-Toolkit2
  3. YOLOv8部署: ultralytics可导出ONNX→RKNN/TensorRT
  4. 双线程流水线: camera_thread单独采集, 主线程推理
     避免采集阻塞推理, 实测可提升20-30%帧率

电赛提示:
  - Orange Pi 5 有6 TOPS NPU, 务必用RKNN推理
  - 摄像头用CSI接口(MIPI)比USB延迟低5-10ms
  - OpenCV编译时加上-D WITH_NEON=ON
"""

import cv2
import time
import threading
import numpy as np
from collections import deque


def optimize_opencv(threads=4):
    """
    开启OpenCV多线程优化(ARM平台)
    
    Args:
        threads: 线程数, Orange Pi 5建议4
    """
    cv2.setNumThreads(threads)
    # 启用OpenCL(如果可用)
    if cv2.ocl.haveOpenCL():
        cv2.ocl.setUseOpenCL(True)
        print("[优化] OpenCL已启用")
    print(f"[优化] OpenCV线程数: {cv2.getNumThreads()}")


class FrameCounter:
    """
    帧率统计器
    
    用法:
        counter = FrameCounter(window=30)
        while True:
            counter.tick()
            print(f"FPS: {counter.fps:.1f}")
    """
    def __init__(self, window=30):
        self._timestamps = deque(maxlen=window)
    
    def tick(self):
        self._timestamps.append(time.perf_counter())
    
    @property
    def fps(self):
        if len(self._timestamps) < 2:
            return 0.0
        dt = self._timestamps[-1] - self._timestamps[0]
        return (len(self._timestamps) - 1) / dt if dt > 0 else 0.0


class CameraThread:
    """
    多线程摄像头采集器
    
    原理: 单独线程不断采集帧, 主线程取最新帧推理
    优势: 采集不阻塞推理, 降低整体延迟
    
    用法:
        cam = CameraThread(src=0, width=640, height=480)
        cam.start()
        frame = cam.read()  # 非阻塞, 返回最新帧
        cam.stop()
    
    电赛应用: 所有视觉模块都应使用此采集器
    """
    def __init__(self, src=0, width=640, height=480, fps=60):
        self.cap = cv2.VideoCapture(src)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        # 降低缓冲区延迟(关键!)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        self.frame = None
        self.grabbed = False
        self.running = False
        self._lock = threading.Lock()
        self._thread = None
    
    def start(self):
        self.running = True
        self._thread = threading.Thread(target=self._update, daemon=True)
        self._thread.start()
        return self
    
    def _update(self):
        while self.running:
            grabbed, frame = self.cap.read()
            with self._lock:
                self.grabbed = grabbed
                self.frame = frame
    
    def read(self):
        with self._lock:
            return self.frame.copy() if self.frame is not None else None
    
    def stop(self):
        self.running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        self.cap.release()


def auto_select_backend():
    """
    自动选择最优推理后端
    
    优先级: RKNN > ONNX > OpenCV DNN
    
    Returns:
        str: 后端名称
    """
    try:
        from rknnlite.api import RKNNLite
        return "rknn"
    except ImportError:
        pass
    
    try:
        import onnxruntime
        providers = onnxruntime.get_available_providers()
        if 'CUDAExecutionProvider' in providers:
            return "onnx_gpu"
        return "onnx_cpu"
    except ImportError:
        pass
    
    return "opencv_dnn"


def resize_for_inference(frame, target_size=320):
    """
    缩放到推理尺寸, 保持宽高比并填充灰边
    
    Args:
        frame: 输入图像
        target_size: 目标尺寸(正方形)
    Returns:
        resized: 缩放后图像
        ratio: 缩放比例
        pad: (top, left) 填充量
    """
    h, w = frame.shape[:2]
    ratio = target_size / max(h, w)
    new_w, new_h = int(w * ratio), int(h * ratio)
    resized = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    
    # 填充到正方形
    top = (target_size - new_h) // 2
    left = (target_size - new_w) // 2
    padded = np.full((target_size, target_size, 3), 114, dtype=np.uint8)
    padded[top:top+new_h, left:left+new_w] = resized
    return padded, ratio, (top, left)


if __name__ == "__main__":
    optimize_opencv()
    print(f"推理后端: {auto_select_backend()}")
    
    # 测试帧率
    cam = CameraThread(src=0, width=640, height=480).start()
    counter = FrameCounter()
    for _ in range(100):
        frame = cam.read()
        if frame is not None:
            counter.tick()
            if counter.fps > 0:
                print(f"\rFPS: {counter.fps:.1f}", end="")
    cam.stop()
