#!/usr/bin/env python3
"""
optimized_qr_detect.py - 优化版二维码/条码识别模块
适用于 RK3588S 嵌入式平台，针对电赛场景优化

优化策略:
1. ROI预处理: 增强对比度 + 二值化 + 去噪
2. 多引擎后端: OpenCV QRCodeDetector / ZBar / Quirc
3. 金字塔加速: 先在缩小图上检测，失败再全分辨率
4. 自适应预处理: 根据图像质量选择不同预处理策略
5. 多格式支持: QR码、DataMatrix、EAN-13、Code128等

依赖:
- opencv-python (必须)
- pyzbar (可选，提供ZBar后端)
- quirc (可选，轻量级QR解码)
"""

import cv2
import numpy as np
import time
from dataclasses import dataclass
from typing import Optional, Tuple, List
from collections import deque


@dataclass
class QRResult:
    """二维码/条码检测结果"""
    data: str             # 解码内容
    format: str           # 格式 (QR, EAN13, CODE128, etc.)
    points: np.ndarray    # 定位点 (4个角点)
    center: Tuple[int, int] = (0, 0)
    confidence: float = 1.0
    decode_time_ms: float = 0.0


class OptimizedQRDetector:
    """
    优化版二维码检测器
    
    检测策略:
    1. 快速路径: 原图直接检测 (适合清晰图像)
    2. 预处理路径: 增强对比度 + 锐化 + 二值化
    3. 金字塔路径: 多尺度检测
    """

    def __init__(self,
                 use_zbar: bool = True,
                 use_pyramid: bool = True,
                 pyramid_levels: int = 3,
                 roi: Optional[Tuple[int, int, int, int]] = None,
                 enhance_contrast: bool = True,
                 sharpen: bool = True):
        """
        Args:
            use_zbar: 是否使用ZBar后端
            use_pyramid: 是否使用金字塔加速
            pyramid_levels: 金字塔层数
            roi: 感兴趣区域
            enhance_contrast: 是否增强对比度
            sharpen: 是否锐化
        """
        self.roi = roi
        self.use_pyramid = use_pyramid
        self.pyramid_levels = pyramid_levels
        self.enhance_contrast = enhance_contrast
        self.sharpen = sharpen
        
        # OpenCV内置检测器
        self._cv_detector = cv2.QRCodeDetector()
        
        # ZBar后端
        self._zbar_available = False
        if use_zbar:
            try:
                from pyzbar import pyzbar
                self._pyzbar = pyzbar
                self._zbar_available = True
            except ImportError:
                print("[WARN] pyzbar未安装，ZBar后端不可用。pip install pyzbar")
        
        # 性能统计
        self._perf = {
            'preprocess_ms': deque(maxlen=50),
            'detect_ms': deque(maxlen=50),
            'total_ms': deque(maxlen=50),
        }

    def _preprocess(self, gray: np.ndarray, 
                    strategy: str = 'default') -> np.ndarray:
        """
        图像预处理（提升识别率的关键）
        
        策略:
        - 'default': 标准预处理
        - 'aggressive': 激进增强（低质量图像）
        - 'fast': 最少处理
        """
        img = gray.copy()
        
        if strategy == 'fast':
            return img
        
        # 1. 对比度增强 (CLAHE)
        if self.enhance_contrast:
            if strategy == 'aggressive':
                clahe = cv2.createCLAHE(clipLimit=4.0, tileGridSize=(4, 4))
            else:
                clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            img = clahe.apply(img)
        
        # 2. 锐化
        if self.sharpen:
            kernel = np.array([[-1, -1, -1],
                               [-1,  9, -1],
                               [-1, -1, -1]], dtype=np.float32)
            img = cv2.filter2D(img, -1, kernel)
        
        # 3. 二值化（激进策略下使用）
        if strategy == 'aggressive':
            # 自适应二值化
            img = cv2.adaptiveThreshold(
                img, 255,
                cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                cv2.THRESH_BINARY,
                31, 10
            )
        
        return img

    def _detect_opencv(self, gray: np.ndarray) -> List[QRResult]:
        """OpenCV内置QR检测"""
        t0 = cv2.getTickCount()
        
        results = []
        
        # 单个QR码检测
        data, points, _ = cv2.QRCodeDetector().detectAndDecode(gray)
        if data and points is not None:
            cx = int(np.mean(points[0][:, 0]))
            cy = int(np.mean(points[0][:, 1]))
            results.append(QRResult(
                data=data,
                format='QR',
                points=points[0],
                center=(cx, cy),
            ))
        
        # 多QR码检测（OpenCV 4.x）
        try:
            retval, decoded_info, points, straight = \
                cv2.QRCodeDetector().detectAndDecodeMulti(gray)
            if retval:
                for i, (info, pts) in enumerate(zip(decoded_info, points)):
                    if info and (not results or 
                                 info != results[0].data):
                        cx = int(np.mean(pts[:, 0]))
                        cy = int(np.mean(pts[:, 1]))
                        results.append(QRResult(
                            data=info,
                            format='QR',
                            points=pts,
                            center=(cx, cy),
                        ))
        except (cv2.error, AttributeError):
            pass
        
        elapsed = (cv2.getTickCount() - t0) / cv2.getTickFrequency() * 1000
        self._perf['detect_ms'].append(elapsed)
        
        return results

    def _detect_zbar(self, gray: np.ndarray) -> List[QRResult]:
        """ZBar后端检测"""
        if not self._zbar_available:
            return []
        
        t0 = cv2.getTickCount()
        results = []
        
        decoded = self._pyzbar.decode(gray)
        for obj in decoded:
            # 提取角点
            if obj.polygon and len(obj.polygon) >= 4:
                points = np.array([[p.x, p.y] for p in obj.polygon], 
                                   dtype=np.float32)
            else:
                x, y, w, h = obj.rect
                points = np.array([
                    [x, y], [x+w, y], [x+w, y+h], [x, y+h]
                ], dtype=np.float32)
            
            cx = int(np.mean(points[:, 0]))
            cy = int(np.mean(points[:, 1]))
            
            # 格式映射
            format_map = {
                'QRCODE': 'QR',
                'EAN13': 'EAN13',
                'EAN8': 'EAN8',
                'CODE128': 'CODE128',
                'CODE39': 'CODE39',
                'DATAMATRIX': 'DataMatrix',
                'PDF417': 'PDF417',
            }
            fmt = format_map.get(obj.type, obj.type)
            
            results.append(QRResult(
                data=obj.data.decode('utf-8', errors='replace'),
                format=fmt,
                points=points,
                center=(cx, cy),
                confidence=obj.quality / 100.0,
            ))
        
        elapsed = (cv2.getTickCount() - t0) / cv2.getTickFrequency() * 1000
        self._perf['detect_ms'].append(elapsed)
        
        return results

    def _detect_pyramid(self, frame: np.ndarray,
                        detect_fn) -> List[QRResult]:
        """
        金字塔加速检测
        
        策略: 先在缩小图上检测，成功则用角点在原图精确定位
        """
        h, w = frame.shape[:2]
        
        # 构建金字塔
        pyramid = [frame]
        for i in range(self.pyramid_levels - 1):
            small = cv2.pyrDown(pyramid[-1])
            pyramid.append(small)
        
        # 从最小图开始检测
        for level in range(len(pyramid) - 1, -1, -1):
            level_img = pyramid[level]
            results = detect_fn(level_img)
            
            if results:
                if level > 0:
                    # 角点坐标缩放回原图
                    scale = 2 ** level
                    for r in results:
                        r.points = r.points * scale
                        r.center = (r.center[0] * scale, r.center[1] * scale)
                
                return results
        
        return []

    def _detect_with_strategy(self, gray: np.ndarray,
                               strategy: str = 'default') -> List[QRResult]:
        """使用指定策略检测"""
        # 预处理
        t0 = cv2.getTickCount()
        processed = self._preprocess(gray, strategy)
        elapsed = (cv2.getTickCount() - t0) / cv2.getTickFrequency() * 1000
        self._perf['preprocess_ms'].append(elapsed)
        
        # 检测
        results = self._detect_opencv(processed)
        
        # ZBar补充
        if not results and self._zbar_available:
            results = self._detect_zbar(processed)
        
        return results

    def detect(self, frame: np.ndarray) -> List[QRResult]:
        """
        主检测函数
        
        多策略渐进式检测:
        1. 先快速检测（最少预处理）
        2. 失败则标准预处理再检测
        3. 再失败则激进预处理
        """
        t0 = cv2.getTickCount()
        
        # ROI裁剪
        img = frame
        roi_offset = (0, 0)
        if self.roi is not None:
            x, y, w, h = self.roi
            img = img[y:y+h, x:x+w]
            roi_offset = (x, y)
        
        # 灰度化
        if len(img.shape) == 3:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        else:
            gray = img
        
        # === 策略1: 快速检测 ===
        results = self._detect_with_strategy(gray, 'fast')
        
        # === 策略2: 金字塔加速 ===
        if not results and self.use_pyramid:
            results = self._detect_pyramid(gray, 
                                           lambda g: self._detect_with_strategy(g, 'default'))
        
        # === 策略3: 标准预处理 ===
        if not results:
            results = self._detect_with_strategy(gray, 'default')
        
        # === 策略4: 激进预处理 ===
        if not results:
            results = self._detect_with_strategy(gray, 'aggressive')
        
        # ROI坐标补偿
        if self.roi is not None and results:
            for r in results:
                r.points[:, 0] += roi_offset[0]
                r.points[:, 1] += roi_offset[1]
                r.center = (r.center[0] + roi_offset[0],
                            r.center[1] + roi_offset[1])
        
        elapsed = (cv2.getTickCount() - t0) / cv2.getTickFrequency() * 1000
        self._perf['total_ms'].append(elapsed)
        
        return results

    def detect_and_draw(self, frame: np.ndarray) -> Tuple[np.ndarray, List[QRResult]]:
        """检测并绘制结果"""
        results = self.detect(frame)
        vis = self.draw_results(frame, results)
        return vis, results

    def draw_results(self, frame: np.ndarray,
                     results: List[QRResult]) -> np.ndarray:
        """绘制检测结果"""
        vis = frame.copy()
        
        for r in results:
            pts = r.points.astype(int)
            
            # 绘制定位框
            cv2.polylines(vis, [pts], True, (0, 255, 0), 2)
            
            # 中心点
            cv2.circle(vis, r.center, 5, (0, 0, 255), -1)
            
            # 文本内容（截断显示）
            text = r.data[:30] + ('...' if len(r.data) > 30 else '')
            label = f"[{r.format}] {text}"
            
            # 文本背景
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 
                                          0.5, 1)
            cv2.rectangle(vis, (r.center[0], r.center[1] - th - 10),
                          (r.center[0] + tw, r.center[1]), (0, 0, 0), -1)
            cv2.putText(vis, label, (r.center[0], r.center[1] - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        return vis

    def get_performance_stats(self) -> dict:
        """获取性能统计"""
        stats = {}
        for key, values in self._perf.items():
            if values:
                stats[key] = {
                    'avg': sum(values) / len(values),
                    'min': min(values),
                    'max': max(values),
                }
        return stats


# ============================================================
# 快捷接口
# ============================================================

def decode_qr(frame: np.ndarray,
              roi: Optional[Tuple[int, int, int, int]] = None) -> List[str]:
    """快速QR解码（只返回内容字符串）"""
    detector = OptimizedQRDetector(roi=roi)
    results = detector.detect(frame)
    return [r.data for r in results]


def decode_any_code(frame: np.ndarray) -> List[QRResult]:
    """解码任意格式条码/二维码"""
    detector = OptimizedQRDetector(use_zbar=True)
    return detector.detect(frame)


# ============================================================
# 演示
# ============================================================

def main():
    """演示 + 性能测试"""
    import sys
    
    # 尝试打开摄像头
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("无法打开摄像头")
        print("使用方法: 将摄像头对准二维码即可自动识别")
        print("\n性能测试模式...")
        
        # 创建测试QR码（需要qrcode库）
        try:
            import qrcode
            qr = qrcode.make("Hello, RTK3588S!")
            test_img = np.array(qr)
            test_img = cv2.cvtColor(test_img, cv2.COLOR_GRAY2BGR)
            test_img = cv2.resize(test_img, (400, 400))
        except ImportError:
            test_img = np.zeros((400, 400, 3), dtype=np.uint8)
            cv2.putText(test_img, "No QR test image", (50, 200),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        detector = OptimizedQRDetector()
        
        # 预热
        for _ in range(5):
            detector.detect(test_img)
        
        # 性能测试
        times = []
        for _ in range(50):
            t0 = cv2.getTickCount()
            results = detector.detect(test_img)
            elapsed = (cv2.getTickCount() - t0) / cv2.getTickFrequency() * 1000
            times.append(elapsed)
        
        print(f"  平均耗时: {np.mean(times):.2f} ms")
        print(f"  FPS: {1000/np.mean(times):.1f}")
        if results:
            print(f"  识别结果: {results[0].data}")
        
        stats = detector.get_performance_stats()
        print(f"  各阶段统计: {stats}")
        return
    
    detector = OptimizedQRDetector(use_zbar=True)
    
    print("二维码识别已启动，按 'q' 退出")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        vis, results = detector.detect_and_draw(frame)
        
        if results:
            for r in results:
                print(f"[{r.format}] {r.data}")
        
        cv2.imshow('QR Detection', vis)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
