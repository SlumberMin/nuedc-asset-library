#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
视觉循线模块
=============
功能：基于透视变换 + 车道线检测的循线算法，适用于电赛赛道循线场景。

核心思路：
    1. 鸟瞰图透视变换（消除透视畸变）
    2. 颜色/边缘阈值提取车道线
    3. 滑动窗口法检测车道线像素
    4. 多项式拟合车道线
    5. 计算横向偏差与航向偏差

依赖：
    pip install opencv-python numpy

使用示例：
    python line_following_visual.py                   # 摄像头实时循线
    python line_following_visual.py --calibrate       # 标定透视变换矩阵

作者：电赛视觉通用代码库
"""

import cv2
import numpy as np
import time
import threading
from collections import deque
from typing import Tuple, Optional, List, Dict


class PerspectiveTransformer:
    """
    透视变换器
    
    将摄像头图像变换为鸟瞰图（BEV），消除透视畸变。
    支持交互式标定或手动设置四点。
    """
    
    def __init__(self, 
                 src_points: Optional[np.ndarray] = None,
                 dst_size: Tuple[int, int] = (320, 240)):
        """
        Args:
            src_points: 源图像四点坐标 (4, 2)，顺序：左上、右上、右下、左下
            dst_size: 输出鸟瞰图尺寸 (宽, 高)
        """
        self.dst_w, self.dst_h = dst_size
        
        # 默认源点（需要根据实际摄像头视野调整）
        if src_points is not None:
            self.src_points = np.float32(src_points)
        else:
            # 默认梯形区域（适用于常见俯视安装）
            self.src_points = np.float32([
                [80, 200],    # 左上
                [560, 200],   # 右上
                [640, 400],   # 右下
                [0, 400]      # 左下
            ])
        
        # 目标矩形四点
        self.dst_points = np.float32([
            [0, 0],
            [self.dst_w, 0],
            [self.dst_w, self.dst_h],
            [0, self.dst_h]
        ])
        
        # 计算变换矩阵
        self.M = cv2.getPerspectiveTransform(self.src_points, self.dst_points)
        self.M_inv = cv2.getPerspectiveTransform(self.dst_points, self.src_points)
    
    def set_src_points(self, points: np.ndarray):
        """动态设置源点并重新计算变换矩阵"""
        self.src_points = np.float32(points)
        self.M = cv2.getPerspectiveTransform(self.src_points, self.dst_points)
        self.M_inv = cv2.getPerspectiveTransform(self.dst_points, self.src_points)
    
    def warp(self, frame: np.ndarray) -> np.ndarray:
        """执行透视变换"""
        return cv2.warpPerspective(frame, self.M, (self.dst_w, self.dst_h),
                                    flags=cv2.INTER_LINEAR)
    
    def unwarp(self, warped: np.ndarray) -> np.ndarray:
        """逆透视变换"""
        return cv2.warpPerspective(warped, self.M_inv, 
                                    (warped.shape[1], warped.shape[0]))
    
    def draw_roi(self, frame: np.ndarray) -> np.ndarray:
        """在原图上绘制 ROI 区域"""
        vis = frame.copy()
        pts = self.src_points.astype(np.int32).reshape(-1, 1, 2)
        cv2.polylines(vis, [pts], True, (0, 255, 255), 2)
        return vis
    
    def interactive_calibrate(self, frame: np.ndarray):
        """
        交互式标定
        
        在图像上依次点击四个点（左上->右上->右下->左下），按 'r' 重置，'q' 确认。
        """
        points = []
        clone = frame.copy()
        
        def mouse_callback(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN and len(points) < 4:
                points.append([x, y])
                cv2.circle(clone, (x, y), 5, (0, 0, 255), -1)
                if len(points) > 1:
                    cv2.line(clone, tuple(points[-2]), tuple(points[-1]), (0, 255, 0), 2)
                if len(points) == 4:
                    cv2.line(clone, tuple(points[-1]), tuple(points[0]), (0, 255, 0), 2)
                cv2.imshow("Calibrate", clone)
        
        cv2.namedWindow("Calibrate")
        cv2.setMouseCallback("Calibrate", mouse_callback)
        cv2.imshow("Calibrate", clone)
        
        print("点击四个点（左上->右上->右下->左下），按 'q' 确认，'r' 重置")
        
        while True:
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') and len(points) == 4:
                self.set_src_points(points)
                print(f"标定完成: {self.src_points.tolist()}")
                break
            elif key == ord('r'):
                points.clear()
                clone = frame.copy()
                cv2.imshow("Calibrate", clone)
        
        cv2.destroyWindow("Calibrate")


class LineDetector:
    """
    车道线检测器
    
    使用滑动窗口法 + 多项式拟合检测车道线。
    """
    
    def __init__(self, 
                 bev_size: Tuple[int, int] = (320, 240),
                 n_windows: int = 9,
                 window_width: int = 40,
                 min_pixels: int = 50,
                 use_color: bool = True):
        """
        Args:
            bev_size: 鸟瞰图尺寸 (宽, 高)
            n_windows: 滑动窗口数量
            window_width: 滑动窗口半宽
            min_pixels: 窗口内最少像素数（否则跳过）
            use_color: True=颜色阈值，False=边缘检测
        """
        self.bev_w, self.bev_h = bev_size
        self.n_windows = n_windows
        self.window_width = window_width
        self.min_pixels = min_pixels
        self.use_color = use_color
        
        # 车道线检测的 HSV 范围（黑色线/白色线）
        # 黑色线
        self.black_lower = np.array([0, 0, 0])
        self.black_upper = np.array([180, 255, 80])
        # 白色线
        self.white_lower = np.array([0, 0, 180])
        self.white_upper = np.array([180, 50, 255])
        
        # 上一帧的拟合结果（用于连续性约束）
        self._prev_fit_left = None
        self._prev_fit_right = None
        self._smooth_factor = 0.7  # 平滑系数
    
    def threshold(self, bev_image: np.ndarray) -> np.ndarray:
        """
        二值化阈值处理
        
        Args:
            bev_image: 鸟瞰图 (BGR)
        Returns:
            二值图（白色=车道线）
        """
        if self.use_color:
            hsv = cv2.cvtColor(bev_image, cv2.COLOR_BGR2HSV)
            
            # 黑色线掩码
            mask_black = cv2.inRange(hsv, self.black_lower, self.black_upper)
            # 白色线掩码
            mask_white = cv2.inRange(hsv, self.white_lower, self.white_upper)
            
            binary = cv2.bitwise_or(mask_black, mask_white)
        else:
            # 边缘检测方式
            gray = cv2.cvtColor(bev_image, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, (5, 5), 0)
            binary = cv2.Canny(blur, 50, 150)
        
        # 形态学处理
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=2)
        
        return binary
    
    def sliding_window_detect(self, binary: np.ndarray) -> Dict:
        """
        滑动窗口法检测车道线
        
        将图像分为上下若干区域，从底部向上逐行搜索车道线像素。
        
        Args:
            binary: 二值化鸟瞰图
        Returns:
            {
                'left_fit': 左线二次多项式系数 [a, b, c] (y = ay² + by + c),
                'right_fit': 右线二次多项式系数,
                'left_pixels': (x, y) 左线像素坐标,
                'right_pixels': (x, y) 右线像素坐标,
                'lane_center': 车道中心 x 坐标,
                'valid': 是否检测到有效车道线
            }
        """
        h, w = binary.shape
        window_h = h // self.n_windows
        
        # 底部直方图确定初始位置
        bottom_half = binary[h // 2:, :]
        histogram = np.sum(bottom_half, axis=0)
        
        # 左右半边分别找峰值
        midpoint = w // 2
        left_base = np.argmax(histogram[:midpoint])
        right_base = np.argmax(histogram[midpoint:]) + midpoint
        
        # 收集所有白色像素坐标
        nonzero = binary.nonzero()
        nonzero_y = np.array(nonzero[0])
        nonzero_x = np.array(nonzero[1])
        
        left_lane_inds = []
        right_lane_inds = []
        
        left_x_current = left_base
        right_x_current = right_base
        
        for window in range(self.n_windows):
            # 窗口边界
            y_low = h - (window + 1) * window_h
            y_high = h - window * window_h
            
            left_x_low = left_x_current - self.window_width
            left_x_high = left_x_current + self.window_width
            right_x_low = right_x_current - self.window_width
            right_x_high = right_x_current + self.window_width
            
            # 在窗口内搜索非零像素
            left_inds = ((nonzero_y >= y_low) & (nonzero_y < y_high) &
                         (nonzero_x >= left_x_low) & (nonzero_x < left_x_high)).nonzero()[0]
            right_inds = ((nonzero_y >= y_low) & (nonzero_y < y_high) &
                          (nonzero_x >= right_x_low) & (nonzero_x < right_x_high)).nonzero()[0]
            
            left_lane_inds.append(left_inds)
            right_lane_inds.append(right_inds)
            
            # 更新窗口中心
            if len(left_inds) > self.min_pixels:
                left_x_current = int(np.mean(nonzero_x[left_inds]))
            if len(right_inds) > self.min_pixels:
                right_x_current = int(np.mean(nonzero_x[right_inds]))
        
        # 合并索引
        left_lane_inds = np.concatenate(left_lane_inds) if left_lane_inds else np.array([])
        right_lane_inds = np.concatenate(right_lane_inds) if right_lane_inds else np.array([])
        
        result = {
            'left_fit': None,
            'right_fit': None,
            'left_pixels': None,
            'right_pixels': None,
            'lane_center': w // 2,
            'valid': False
        }
        
        # 提取像素坐标并拟合
        min_fit_points = 50
        
        if len(left_lane_inds) > min_fit_points:
            left_x = nonzero_x[left_lane_inds]
            left_y = nonzero_y[left_lane_inds]
            left_fit = np.polyfit(left_y, left_x, 2)
            
            # 平滑
            if self._prev_fit_left is not None:
                left_fit = self._smooth_factor * self._prev_fit_left + \
                           (1 - self._smooth_factor) * left_fit
            self._prev_fit_left = left_fit
            
            result['left_fit'] = left_fit
            result['left_pixels'] = (left_x, left_y)
        
        if len(right_lane_inds) > min_fit_points:
            right_x = nonzero_x[right_lane_inds]
            right_y = nonzero_y[right_lane_inds]
            right_fit = np.polyfit(right_y, right_x, 2)
            
            if self._prev_fit_right is not None:
                right_fit = self._smooth_factor * self._prev_fit_right + \
                            (1 - self._smooth_factor) * right_fit
            self._prev_fit_right = right_fit
            
            result['right_fit'] = right_fit
            result['right_pixels'] = (right_x, right_y)
        
        # 计算车道中心
        if result['left_fit'] is not None and result['right_fit'] is not None:
            y_eval = h  # 图像底部
            left_bottom = np.polyval(result['left_fit'], y_eval)
            right_bottom = np.polyval(result['right_fit'], y_eval)
            result['lane_center'] = (left_bottom + right_bottom) / 2.0
            result['valid'] = True
        elif result['left_fit'] is not None:
            # 只有左线，假设标准车道宽度
            y_eval = h
            left_bottom = np.polyval(result['left_fit'], y_eval)
            result['lane_center'] = left_bottom + 80  # 假设线间距约80像素
            result['valid'] = True
        elif result['right_fit'] is not None:
            y_eval = h
            right_bottom = np.polyval(result['right_fit'], y_eval)
            result['lane_center'] = right_bottom - 80
            result['valid'] = True
        
        return result
    
    def draw_detection(self, bev_image: np.ndarray, result: Dict) -> np.ndarray:
        """在鸟瞰图上绘制检测结果"""
        vis = bev_image.copy()
        h, w = vis.shape[:2]
        
        # 绘制拟合曲线
        y_range = np.linspace(0, h - 1, h).astype(np.int32)
        
        if result['left_fit'] is not None:
            left_x = np.polyval(result['left_fit'], y_range).astype(np.int32)
            pts_left = np.column_stack([left_x, y_range])
            cv2.polylines(vis, [pts_left], False, (0, 0, 255), 2)
        
        if result['right_fit'] is not None:
            right_x = np.polyval(result['right_fit'], y_range).astype(np.int32)
            pts_right = np.column_stack([right_x, y_range])
            cv2.polylines(vis, [pts_right], False, (255, 0, 0), 2)
        
        # 绘制车道中心
        center_x = int(result['lane_center'])
        cv2.line(vis, (center_x, h), (center_x, h - 40), (0, 255, 0), 2)
        cv2.circle(vis, (center_x, h - 10), 5, (0, 255, 0), -1)
        
        # 绘制图像中心参考线
        cv2.line(vis, (w // 2, h), (w // 2, h - 40), (255, 255, 255), 1)
        
        return vis


class LineFollowingController:
    """
    视觉循线控制器
    
    整合透视变换 + 车道线检测 + PID 偏差计算。
    """
    
    def __init__(self, 
                 bev_size: Tuple[int, int] = (320, 240),
                 src_points: Optional[np.ndarray] = None,
                 pid_params: Optional[Dict] = None):
        """
        Args:
            bev_size: 鸟瞰图尺寸
            src_points: 透视变换源四点
            pid_params: PID 参数
        """
        self.bev_size = bev_size
        
        # 透视变换器
        self.transformer = PerspectiveTransformer(src_points, bev_size)
        
        # 车道线检测器
        self.detector = LineDetector(bev_size)
        
        # PID 控制器
        p = pid_params or {'kp': 0.5, 'ki': 0.01, 'kd': 0.1}
        self._kp = p.get('kp', 0.5)
        self._ki = p.get('ki', 0.01)
        self._kd = p.get('kd', 0.1)
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = time.time()
        
        # 输出限幅
        self.output_limit = 100.0
        
        # 线程安全
        self._lock = threading.Lock()
        self._last_result: Optional[Dict] = None
        
        # FPS
        self.fps = 0.0
        self._frame_count = 0
        self._fps_timer = time.time()
    
    def compute_deviation(self, result: Dict) -> Dict:
        """
        计算循线偏差
        
        Args:
            result: LineDetector 的检测结果
        Returns:
            {
                'lateral_error': 横向偏差（像素，正=偏右）
                'steering': PID 转向输出（正=向右转）
                'confidence': 检测置信度 (0~1)
            }
        """
        w = self.bev_size[0]
        image_center = w / 2.0
        lane_center = result['lane_center']
        
        # 横向偏差
        lateral_error = lane_center - image_center
        
        # PID 计算
        current_time = time.time()
        dt = current_time - self._prev_time
        if dt <= 0:
            dt = 0.01
        
        self._integral += lateral_error * dt
        self._integral = np.clip(self._integral, -500, 500)
        
        derivative = (lateral_error - self._prev_error) / dt
        
        steering = (self._kp * lateral_error + 
                    self._ki * self._integral + 
                    self._kd * derivative)
        steering = np.clip(steering, -self.output_limit, self.output_limit)
        
        self._prev_error = lateral_error
        self._prev_time = current_time
        
        # 置信度
        confidence = 1.0 if result['valid'] else 0.0
        
        return {
            'lateral_error': lateral_error,
            'steering': steering,
            'confidence': confidence
        }
    
    def process_frame(self, frame: np.ndarray) -> Dict:
        """
        处理一帧图像
        
        Args:
            frame: BGR 输入图像
        Returns:
            完整处理结果
        """
        # 1. 透视变换
        bev = self.transformer.warp(frame)
        
        # 2. 二值化
        binary = self.detector.threshold(bev)
        
        # 3. 车道线检测
        line_result = self.detector.sliding_window_detect(binary)
        
        # 4. 计算偏差
        deviation = self.compute_deviation(line_result)
        
        result = {
            'bev': bev,
            'binary': binary,
            'line_result': line_result,
            'deviation': deviation,
            'lateral_error': deviation['lateral_error'],
            'steering': deviation['steering'],
            'confidence': deviation['confidence']
        }
        
        with self._lock:
            self._last_result = result
        
        return result
    
    def process_threaded(self, frame: np.ndarray, callback=None):
        """多线程处理"""
        frame_copy = frame.copy()
        
        def _worker():
            result = self.process_frame(frame_copy)
            if callback:
                callback(result)
        
        t = threading.Thread(target=_worker, daemon=True)
        t.start()
        return t
    
    def draw(self, frame: np.ndarray, result: Optional[Dict] = None) -> np.ndarray:
        """
        绘制完整可视化结果
        
        Args:
            frame: 原始输入图像
            result: 处理结果
        Returns:
            绘制后的图像
        """
        vis = frame.copy()
        
        if result is None:
            with self._lock:
                result = self._last_result
        
        if result is None:
            return vis
        
        # 原图上绘制 ROI
        vis = self.transformer.draw_roi(vis)
        
        # 鸟瞰图检测结果
        bev_vis = self.detector.draw_detection(result['bev'], result['line_result'])
        
        # 缩放鸟瞰图到右上角
        h, w = vis.shape[:2]
        bev_small = cv2.resize(bev_vis, (w // 3, h // 3))
        binary_small = cv2.resize(cv2.cvtColor(result['binary'], cv2.COLOR_GRAY2BGR), 
                                   (w // 3, h // 3))
        
        vis[0:h//3, w - w//3:w] = bev_small
        vis[h//3:h//3*2, w - w//3:w] = binary_small
        
        # 信息文本
        dev = result['deviation']
        info = [
            f"FPS: {self.fps:.1f}",
            f"Lateral: {dev['lateral_error']:.1f}px",
            f"Steering: {dev['steering']:.1f}",
            f"Confidence: {dev['confidence']:.0%}",
            "VALID" if result['line_result']['valid'] else "LOST"
        ]
        
        color = (0, 255, 0) if result['line_result']['valid'] else (0, 0, 255)
        for i, text in enumerate(info):
            cv2.putText(vis, text, (10, 25 + i * 25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        # 转向指示条
        bar_y = h - 40
        bar_center = w // 6
        bar_w = w // 6
        cv2.rectangle(vis, (bar_center - bar_w, bar_y - 5), 
                      (bar_center + bar_w, bar_y + 15), (100, 100, 100), -1)
        indicator_x = int(bar_center + np.clip(dev['steering'], -bar_w, bar_w))
        cv2.rectangle(vis, (indicator_x - 5, bar_y - 5),
                      (indicator_x + 5, bar_y + 15), (0, 0, 255), -1)
        
        return vis
    
    def update_fps(self):
        self._frame_count += 1
        elapsed = time.time() - self._fps_timer
        if elapsed >= 1.0:
            self.fps = self._frame_count / elapsed
            self._frame_count = 0
            self._fps_timer = time.time()


def run_demo():
    """摄像头实时循线演示"""
    print("=" * 60)
    print("  视觉循线演示")
    print("  按 'q' 退出 | 按 'c' 标定透视变换")
    print("=" * 60)
    
    controller = LineFollowingController()
    
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
        
        result = controller.process_frame(frame)
        controller.update_fps()
        
        vis = controller.draw(frame, result)
        cv2.imshow("Line Following", vis)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('c'):
            controller.transformer.interactive_calibrate(frame)
    
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description="视觉循线")
    parser.add_argument('--calibrate', action='store_true', help='标定透视变换')
    args = parser.parse_args()
    
    run_demo()
