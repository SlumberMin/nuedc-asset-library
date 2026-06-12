#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
二维码/条形码检测模块
=====================
功能：使用 OpenCV + pyzbar 实时检测二维码和条形码，支持多线程采集与解码。

依赖：
    pip install opencv-python pyzbar numpy

使用示例：
    python qr_code_detector.py                    # 使用摄像头
    python qr_code_detector.py --image test.png   # 检测单张图片

作者：电赛视觉通用代码库
"""

import cv2
import numpy as np
import time
import threading
from collections import deque
from typing import List, Dict, Tuple, Optional

try:
    from pyzbar import pyzbar
    HAS_PYZBAR = True
except ImportError:
    HAS_PYZBAR = False
    print("[警告] pyzbar 未安装，仅支持 OpenCV 自带二维码检测器")


class QRCodeDetector:
    """
    二维码/条形码检测器
    
    特性：
    1. 双引擎检测（pyzbar + OpenCV QRCodeDetector），提高识别率
    2. 多线程解码，不阻塞主循环
    3. 结果缓存与去重
    4. 支持自定义感兴趣区域（ROI）
    """
    
    def __init__(self, use_thread: bool = True, cache_time: float = 0.5):
        """
        初始化检测器
        
        Args:
            use_thread: 是否使用多线程解码
            cache_time: 检测结果缓存时间(秒)，避免重复回调
        """
        self.use_thread = use_thread
        self.cache_time = cache_time
        
        # --- OpenCV 自带检测器 ---
        self.cv_detector = cv2.QRCodeDetector()
        
        # --- 检测结果缓存 ---
        self._result_cache: Dict[str, float] = {}  # data -> last_detect_time
        self._lock = threading.Lock()
        
        # --- 线程池 ---
        self._thread_pool: List[threading.Thread] = []
        self._max_threads = 4
        
        # --- 统计 ---
        self.fps = 0.0
        self.detect_count = 0
        self._frame_count = 0
        self._fps_timer = time.time()
    
    def _decode_with_pyzbar(self, gray: np.ndarray) -> List[Dict]:
        """
        使用 pyzbar 解码（支持二维码+条形码）
        
        Args:
            gray: 灰度图像
        Returns:
            解码结果列表
        """
        if not HAS_PYZBAR:
            return []
        
        results = []
        try:
            decoded_objects = pyzbar.decode(gray, symbols=[
                pyzbar.ZBarSymbol.QRCODE,
                pyzbar.ZBarSymbol.EAN13,
                pyzbar.ZBarSymbol.EAN8,
                pyzbar.ZBarSymbol.CODE128,
                pyzbar.ZBarSymbol.CODE39,
                pyzbar.ZBarSymbol.UPCA,
                pyzbar.ZBarSymbol.UPCE,
            ])
            
            for obj in decoded_objects:
                # 提取角点
                points = obj.polygon
                if len(points) == 4:
                    pts = np.array([[p.x, p.y] for p in points], dtype=np.int32)
                else:
                    # 条形码只有两个点，扩展为矩形
                    x, y, w, h = obj.rect
                    pts = np.array([
                        [x, y], [x + w, y], [x + w, y + h], [x, y + h]
                    ], dtype=np.int32)
                
                results.append({
                    'data': obj.data.decode('utf-8', errors='replace'),
                    'type': obj.type,
                    'points': pts,
                    'rect': obj.rect,
                    'engine': 'pyzbar'
                })
        except Exception as e:
            print(f"[pyzbar 解码异常] {e}")
        
        return results
    
    def _decode_with_opencv(self, frame: np.ndarray) -> List[Dict]:
        """
        使用 OpenCV 内置检测器解码二维码
        
        Args:
            frame: BGR 彩色图像
        Returns:
            解码结果列表
        """
        results = []
        try:
            # OpenCV >= 4.5.4 支持 QRCodeDetectorAruco，速度更快
            # 普通版本使用 detectAndDecodeMulti
            retval, decoded_info, points, straight_qrcode = self.cv_detector.detectAndDecodeMulti(frame)
            
            if retval and points is not None:
                for i, (info, pts) in enumerate(zip(decoded_info, points)):
                    if info:  # 非空字符串表示成功解码
                        pts = pts.astype(np.int32).reshape(-1, 2)
                        rect = cv2.boundingRect(pts)
                        results.append({
                            'data': info,
                            'type': 'QRCODE',
                            'points': pts,
                            'rect': rect,
                            'engine': 'opencv'
                        })
        except Exception:
            # detectAndDecodeMulti 不可用时，回退到单目标检测
            try:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                retval, decoded_info = self.cv_detector.decodeMulti(gray)
                if retval and decoded_info:
                    for info in decoded_info:
                        if info:
                            results.append({
                                'data': info,
                                'type': 'QRCODE',
                                'points': np.array([[0, 0]], dtype=np.int32),
                                'rect': (0, 0, 0, 0),
                                'engine': 'opencv-fallback'
                            })
            except Exception:
                pass
        
        return results
    
    def _preprocess(self, frame: np.ndarray, roi: Optional[Tuple[int, int, int, int]] = None) -> Tuple[np.ndarray, np.ndarray]:
        """
        图像预处理：裁剪 ROI、灰度化、增强对比度
        
        Args:
            frame: BGR 输入图像
            roi: (x, y, w, h) 感兴趣区域，None 表示全图
        Returns:
            (预处理后的 BGR 图像, 灰度图像)
        """
        if roi is not None:
            x, y, w, h = roi
            frame_roi = frame[y:y+h, x:x+w]
        else:
            frame_roi = frame
            x, y = 0, 0
        
        # CLAHE 对比度增强（对光照不均匀场景效果好）
        gray = cv2.cvtColor(frame_roi, cv2.COLOR_BGR2GRAY)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        gray_enhanced = clahe.apply(gray)
        
        return frame_roi, gray_enhanced
    
    def detect(self, frame: np.ndarray, 
               roi: Optional[Tuple[int, int, int, int]] = None) -> List[Dict]:
        """
        检测图像中的二维码/条形码（同步版）
        
        Args:
            frame: BGR 输入图像
            roi: (x, y, w, h) 感兴趣区域
        Returns:
            检测结果列表，每个结果包含:
            - data: 解码内容
            - type: 码类型 (QRCODE/EAN13/CODE128 等)
            - points: 四个角点坐标 (相对于原图)
            - rect: 外接矩形 (相对于原图)
            - engine: 使用的检测引擎
        """
        roi_frame, gray = self._preprocess(frame, roi)
        offset_x, offset_y = (roi[0], roi[1]) if roi else (0, 0)
        
        all_results = []
        
        # 引擎1: pyzbar（支持条形码）
        if HAS_PYZBAR:
            pzb_results = self._decode_with_pyzbar(gray)
            all_results.extend(pzb_results)
        
        # 引擎2: OpenCV（对二维码鲁棒性好）
        cv_results = self._decode_with_opencv(roi_frame)
        all_results.extend(cv_results)
        
        # 去重：相同 data 的只保留一个
        seen = set()
        unique_results = []
        for r in all_results:
            if r['data'] not in seen:
                seen.add(r['data'])
                # 坐标偏移回原图
                r['points'][:, 0] += offset_x
                r['points'][:, 1] += offset_y
                rx, ry, rw, rh = r['rect']
                r['rect'] = (rx + offset_x, ry + offset_y, rw, rh)
                unique_results.append(r)
        
        # 更新缓存
        current_time = time.time()
        with self._lock:
            for r in unique_results:
                self._result_cache[r['data']] = current_time
        
        self.detect_count += len(unique_results)
        return unique_results
    
    def detect_async(self, frame: np.ndarray, 
                     callback=None,
                     roi: Optional[Tuple[int, int, int, int]] = None):
        """
        异步检测（多线程版本）
        
        Args:
            frame: BGR 输入图像（会被拷贝）
            callback: 检测完成后的回调函数 callback(results)
            roi: 感兴趣区域
        """
        if not self.use_thread:
            results = self.detect(frame, roi)
            if callback:
                callback(results)
            return results
        
        # 限制线程数
        self._thread_pool = [t for t in self._thread_pool if t.is_alive()]
        if len(self._thread_pool) >= self._max_threads:
            return None
        
        frame_copy = frame.copy()
        
        def _worker():
            results = self.detect(frame_copy, roi)
            if callback:
                callback(results)
        
        t = threading.Thread(target=_worker, daemon=True)
        t.start()
        self._thread_pool.append(t)
        return None
    
    def draw_results(self, frame: np.ndarray, results: List[Dict], 
                     draw_data: bool = True) -> np.ndarray:
        """
        在图像上绘制检测结果
        
        Args:
            frame: 输入图像（会被拷贝绘制）
            results: 检测结果列表
            draw_data: 是否绘制解码文本
        Returns:
            绘制后的图像
        """
        vis = frame.copy()
        
        for r in results:
            pts = r['points']
            data = r['data']
            qr_type = r['type']
            
            # 绘制多边形边框
            cv2.polylines(vis, [pts], True, (0, 255, 0), 2)
            
            # 绘制角点
            for pt in pts:
                cv2.circle(vis, tuple(pt), 4, (0, 0, 255), -1)
            
            if draw_data:
                # 计算文本位置（边框上方）
                x, y = pts[0]
                y = max(y - 10, 20)
                
                # 背景矩形
                text = f"[{qr_type}] {data}"
                (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                cv2.rectangle(vis, (x, y - th - 4), (x + tw + 4, y + 4), (0, 255, 0), -1)
                cv2.putText(vis, text, (x + 2, y), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        
        return vis
    
    def update_fps(self):
        """更新帧率统计"""
        self._frame_count += 1
        elapsed = time.time() - self._fps_timer
        if elapsed >= 1.0:
            self.fps = self._frame_count / elapsed
            self._frame_count = 0
            self._fps_timer = time.time()
    
    def is_new_detection(self, data: str) -> bool:
        """
        判断是否为新的检测结果（去重用）
        
        Args:
            data: 解码内容
        Returns:
            True 表示在缓存时间内未出现过
        """
        with self._lock:
            last_time = self._result_cache.get(data, 0)
            return (time.time() - last_time) > self.cache_time


def run_camera_demo():
    """摄像头实时检测演示"""
    print("=" * 60)
    print("  二维码/条形码实时检测")
    print("  按 'q' 退出 | 按 's' 截图")
    print("=" * 60)
    
    detector = QRCodeDetector(use_thread=False)
    
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("[错误] 无法打开摄像头")
        return
    
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # 检测
        results = detector.detect(frame)
        detector.update_fps()
        
        # 绘制
        vis = detector.draw_results(frame, results)
        
        # 显示帧率
        cv2.putText(vis, f"FPS: {detector.fps:.1f}  Detections: {len(results)}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # 打印检测到的内容
        for r in results:
            print(f"  [{r['type']}] {r['data']}")
        
        cv2.imshow("QR Code Detector", vis)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            cv2.imwrite("qr_capture.png", vis)
            print("[保存] qr_capture.png")
    
    cap.release()
    cv2.destroyAllWindows()


def run_image_demo(image_path: str):
    """单张图片检测演示"""
    frame = cv2.imread(image_path)
    if frame is None:
        print(f"[错误] 无法读取图片: {image_path}")
        return
    
    detector = QRCodeDetector()
    results = detector.detect(frame)
    
    print(f"\n检测结果 ({len(results)} 个):")
    for i, r in enumerate(results):
        print(f"  {i+1}. [{r['type']}] {r['data']}")
        print(f"     位置: {r['rect']}")
    
    vis = detector.draw_results(frame, results)
    
    # 自适应窗口大小
    h, w = vis.shape[:2]
    scale = min(800 / w, 600 / h, 1.0)
    if scale < 1.0:
        vis = cv2.resize(vis, None, fx=scale, fy=scale)
    
    cv2.imshow("QR Detection Result", vis)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description="二维码/条形码检测")
    parser.add_argument('--image', type=str, help='检测单张图片路径')
    args = parser.parse_args()
    
    if args.image:
        run_image_demo(args.image)
    else:
        run_camera_demo()
