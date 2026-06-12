#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
色块追踪模块
=============
功能：基于 HSV 颜色分割的色块检测与 PID 跟踪，适用于电赛小球/色块追踪场景。

依赖：
    pip install opencv-python numpy

使用示例：
    python color_blob_tracking.py                        # 摄像头实时追踪
    python color_blob_tracking.py --color red             # 指定颜色
    python color_blob_tracking.py --tune                  # 颜色标定模式

作者：电赛视觉通用代码库
"""

import cv2
import numpy as np
import time
import threading
from collections import deque
from typing import Tuple, Optional, List, Dict


# ============================================================
#  预设颜色范围（HSV 空间）—— 根据实际环境微调
# ============================================================
COLOR_PRESETS = {
    'red': {
        'lower1': np.array([0, 100, 100]),
        'upper1': np.array([10, 255, 255]),
        'lower2': np.array([160, 100, 100]),  # 红色跨越 H=0°，需要两段
        'upper2': np.array([180, 255, 255]),
        'bgr': (0, 0, 255)
    },
    'blue': {
        'lower': np.array([100, 100, 100]),
        'upper': np.array([130, 255, 255]),
        'bgr': (255, 0, 0)
    },
    'green': {
        'lower': np.array([35, 100, 100]),
        'upper': np.array([85, 255, 255]),
        'bgr': (0, 255, 0)
    },
    'yellow': {
        'lower': np.array([20, 100, 100]),
        'upper': np.array([35, 255, 255]),
        'bgr': (0, 255, 255)
    },
    'orange': {
        'lower': np.array([10, 100, 100]),
        'upper': np.array([20, 255, 255]),
        'bgr': (0, 128, 255)
    },
    'purple': {
        'lower': np.array([130, 100, 100]),
        'upper': np.array([160, 255, 255]),
        'bgr': (255, 0, 255)
    }
}


class PIDController:
    """
    PID 控制器
    
    用于根据视觉偏差计算跟踪控制量。
    """
    
    def __init__(self, kp: float = 0.5, ki: float = 0.0, kd: float = 0.1,
                 output_limit: float = 100.0, integral_limit: float = 500.0):
        """
        Args:
            kp: 比例系数
            ki: 积分系数
            kd: 微分系数
            output_limit: 输出限幅
            integral_limit: 积分限幅（防止积分饱和）
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.integral_limit = integral_limit
        
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = time.time()
    
    def compute(self, error: float) -> float:
        """
        计算 PID 输出
        
        Args:
            error: 当前误差（目标值 - 当前值）
        Returns:
            PID 控制输出
        """
        current_time = time.time()
        dt = current_time - self._prev_time
        if dt <= 0:
            dt = 0.01
        
        # 比例项
        p_out = self.kp * error
        
        # 积分项（带限幅）
        self._integral += error * dt
        self._integral = np.clip(self._integral, -self.integral_limit, self.integral_limit)
        i_out = self.ki * self._integral
        
        # 微分项
        derivative = (error - self._prev_error) / dt
        d_out = self.kd * derivative
        
        # 总输出限幅
        output = p_out + i_out + d_out
        output = np.clip(output, -self.output_limit, self.output_limit)
        
        # 更新状态
        self._prev_error = error
        self._prev_time = current_time
        
        return output
    
    def reset(self):
        """重置控制器状态"""
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = time.time()


class ColorBlobTracker:
    """
    色块追踪器
    
    工作流程：
    1. BGR -> HSV 转换
    2. 颜色阈值分割生成掩码
    3. 形态学操作去除噪点
    4. 查找轮廓 -> 挑选最大轮廓
    5. 计算质心 + 外接圆
    6. PID 计算跟踪控制量
    
    线程安全：检测与显示可分离
    """
    
    def __init__(self, color_name: str = 'red',
                 min_area: int = 500,
                 kernel_size: int = 5,
                 pid_params: Optional[Dict] = None):
        """
        Args:
            color_name: 颜色名称（red/blue/green/yellow/orange/purple）
            min_area: 最小色块面积（过滤噪点）
            kernel_size: 形态学核大小
            pid_params: PID 参数字典 {'kp', 'ki', 'kd'}
        """
        if color_name not in COLOR_PRESETS:
            raise ValueError(f"不支持的颜色: {color_name}，可选: {list(COLOR_PRESETS.keys())}")
        
        self.color_name = color_name
        self.color_preset = COLOR_PRESETS[color_name]
        self.min_area = min_area
        self.draw_color = self.color_preset['bgr']
        
        # --- 形态学核 ---
        self._kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        self._kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size * 2, kernel_size * 2))
        
        # --- PID 控制器 ---
        default_pid = {'kp': 0.3, 'ki': 0.01, 'kd': 0.05}
        if pid_params:
            default_pid.update(pid_params)
        
        self.pid_x = PIDController(**default_pid)
        self.pid_y = PIDController(**default_pid)
        
        # --- 目标位置（图像中心为零点） ---
        self.target_x = 0.0
        self.target_y = 0.0
        
        # --- 检测结果 ---
        self._lock = threading.Lock()
        self._blob_result: Optional[Dict] = None
        
        # --- 轨迹历史 ---
        self._trajectory = deque(maxlen=64)
        
        # --- FPS ---
        self.fps = 0.0
        self._frame_count = 0
        self._fps_timer = time.time()
    
    def set_color(self, color_name: str):
        """动态切换追踪颜色"""
        if color_name not in COLOR_PRESETS:
            raise ValueError(f"不支持的颜色: {color_name}")
        self.color_name = color_name
        self.color_preset = COLOR_PRESETS[color_name]
        self.draw_color = self.color_preset['bgr']
    
    def set_hsv_range(self, lower: np.ndarray, upper: np.ndarray,
                      lower2: Optional[np.ndarray] = None, upper2: Optional[np.ndarray] = None):
        """
        手动设置 HSV 范围（用于颜色标定后）
        
        Args:
            lower: HSV 下界
            upper: HSV 上界
            lower2: 第二段下界（红色等跨越 0° 的情况）
            upper2: 第二段上界
        """
        self.color_preset = {
            'lower': lower,
            'upper': upper,
            'bgr': self.draw_color
        }
        if lower2 is not None and upper2 is not None:
            self.color_preset['lower2'] = lower2
            self.color_preset['upper2'] = upper2
    
    def set_target(self, x: float, y: float):
        """设置追踪目标位置（图像坐标系，中心为 (0,0)）"""
        self.target_x = x
        self.target_y = y
    
    def _create_mask(self, hsv: np.ndarray) -> np.ndarray:
        """
        根据 HSV 范围创建颜色掩码
        
        Args:
            hsv: HSV 图像
        Returns:
            二值掩码
        """
        preset = self.color_preset
        
        # 红色有两段范围
        if 'lower2' in preset:
            mask1 = cv2.inRange(hsv, preset['lower1'], preset['upper1'])
            mask2 = cv2.inRange(hsv, preset['lower2'], preset['upper2'])
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            mask = cv2.inRange(hsv, preset['lower'], preset['upper'])
        
        # 形态学开运算：去除小噪点
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._kernel_open, iterations=2)
        # 形态学闭运算：填充孔洞
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._kernel_close, iterations=2)
        
        return mask
    
    def detect(self, frame: np.ndarray) -> Optional[Dict]:
        """
        检测色块
        
        Args:
            frame: BGR 输入图像
        Returns:
            检测结果字典，未检测到返回 None:
            - cx, cy: 质心坐标
            - radius: 外接圆半径
            - area: 轮廓面积
            - bbox: 外接矩形 (x, y, w, h)
            - control_x: PID 水平输出（正=向右）
            - control_y: PID 垂直输出（正=向下）
        """
        h_img, w_img = frame.shape[:2]
        
        # 1. 转换色彩空间
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # 2. 颜色分割
        mask = self._create_mask(hsv)
        
        # 3. 查找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            with self._lock:
                self._blob_result = None
            return None
        
        # 4. 挑选最大轮廓
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        
        if area < self.min_area:
            with self._lock:
                self._blob_result = None
            return None
        
        # 5. 计算质心与外接圆
        (cx, cy), radius = cv2.minEnclosingCircle(largest)
        cx, cy, radius = int(cx), int(cy), int(radius)
        
        # 6. 外接矩形
        bbox = cv2.boundingRect(largest)
        
        # 7. 计算相对于图像中心的偏差（像素）
        error_x = cx - w_img / 2.0
        error_y = cy - h_img / 2.0
        
        # 8. PID 计算控制量
        control_x = self.pid_x.compute(error_x - self.target_x)
        control_y = self.pid_y.compute(error_y - self.target_y)
        
        # 9. 更新轨迹
        self._trajectory.append((cx, cy))
        
        result = {
            'cx': cx,
            'cy': cy,
            'radius': radius,
            'area': area,
            'bbox': bbox,
            'error_x': error_x,
            'error_y': error_y,
            'control_x': control_x,
            'control_y': control_y,
            'mask': mask,
            'contour': largest
        }
        
        with self._lock:
            self._blob_result = result
        
        return result
    
    def detect_threaded(self, frame: np.ndarray, callback=None):
        """多线程检测"""
        frame_copy = frame.copy()
        
        def _worker():
            result = self.detect(frame_copy)
            if callback:
                callback(result)
        
        t = threading.Thread(target=_worker, daemon=True)
        t.start()
        return t
    
    def draw(self, frame: np.ndarray, result: Optional[Dict] = None,
             show_trajectory: bool = True, show_mask: bool = False) -> np.ndarray:
        """
        绘制检测结果与 PID 控制信息
        
        Args:
            frame: 输入图像
            result: 检测结果（None 则使用缓存）
            show_trajectory: 是否绘制轨迹
            show_mask: 是否在左上角显示掩码
        Returns:
            绘制后的图像
        """
        vis = frame.copy()
        h, w = vis.shape[:2]
        
        if result is None:
            with self._lock:
                result = self._blob_result
        
        # 绘制图像中心十字线
        cv2.line(vis, (w // 2 - 20, h // 2), (w // 2 + 20, h // 2), (255, 255, 255), 1)
        cv2.line(vis, (w // 2, h // 2 - 20), (w // 2, h // 2 + 20), (255, 255, 255), 1)
        
        if result is None:
            cv2.putText(vis, "No target", (10, h - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            return vis
        
        cx, cy = result['cx'], result['cy']
        radius = result['radius']
        
        # 外接圆
        cv2.circle(vis, (cx, cy), radius, self.draw_color, 2)
        # 质心
        cv2.circle(vis, (cx, cy), 5, (0, 0, 255), -1)
        
        # 连线到图像中心
        cv2.line(vis, (w // 2, h // 2), (cx, cy), (255, 255, 0), 1, cv2.LINE_AA)
        
        # 信息文本
        info_lines = [
            f"Color: {self.color_name}",
            f"Pos: ({cx}, {cy})",
            f"Error: ({result['error_x']:.0f}, {result['error_y']:.0f})",
            f"PID Out: ({result['control_x']:.1f}, {result['control_y']:.1f})",
            f"Area: {result['area']:.0f}",
            f"FPS: {self.fps:.1f}"
        ]
        
        for i, text in enumerate(info_lines):
            cv2.putText(vis, text, (10, 25 + i * 22),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        # 绘制 PID 控制箭头
        arrow_scale = 0.5
        end_x = int(w // 2 + result['control_x'] * arrow_scale)
        end_y = int(h // 2 + result['control_y'] * arrow_scale)
        cv2.arrowedLine(vis, (w // 2, h // 2), (end_x, end_y), (0, 0, 255), 2, tipLength=0.3)
        
        # 绘制轨迹
        if show_trajectory and len(self._trajectory) > 1:
            pts = list(self._trajectory)
            for i in range(1, len(pts)):
                alpha = i / len(pts)  # 渐变透明效果
                color = (0, int(255 * alpha), int(255 * (1 - alpha)))
                cv2.line(vis, pts[i - 1], pts[i], color, 2)
        
        # 左上角小窗显示掩码
        if show_mask and 'mask' in result:
            mask_rgb = cv2.cvtColor(result['mask'], cv2.COLOR_GRAY2BGR)
            small = cv2.resize(mask_rgb, (w // 4, h // 4))
            vis[0:small.shape[0], 0:small.shape[1]] = small
        
        return vis
    
    def update_fps(self):
        """更新帧率"""
        self._frame_count += 1
        elapsed = time.time() - self._fps_timer
        if elapsed >= 1.0:
            self.fps = self._frame_count / elapsed
            self._frame_count = 0
            self._fps_timer = time.time()


def color_tuner():
    """
    颜色标定工具
    
    用滑动条手动调节 HSV 阈值，适合现场标定。
    """
    print("=" * 60)
    print("  HSV 颜色标定工具")
    print("  调整滑动条 -> 按 's' 保存参数 -> 按 'q' 退出")
    print("=" * 60)
    
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("[错误] 无法打开摄像头")
        return
    
    cv2.namedWindow("Color Tuner")
    cv2.namedWindow("Trackbars")
    
    # 创建滑动条
    cv2.createTrackbar("H_min", "Trackbars", 0, 179, lambda x: None)
    cv2.createTrackbar("H_max", "Trackbars", 179, 179, lambda x: None)
    cv2.createTrackbar("S_min", "Trackbars", 100, 255, lambda x: None)
    cv2.createTrackbar("S_max", "Trackbars", 255, 255, lambda x: None)
    cv2.createTrackbar("V_min", "Trackbars", 100, 255, lambda x: None)
    cv2.createTrackbar("V_max", "Trackbars", 255, 255, lambda x: None)
    cv2.createTrackbar("Min Area", "Trackbars", 500, 5000, lambda x: None)
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        h_min = cv2.getTrackbarPos("H_min", "Trackbars")
        h_max = cv2.getTrackbarPos("H_max", "Trackbars")
        s_min = cv2.getTrackbarPos("S_min", "Trackbars")
        s_max = cv2.getTrackbarPos("S_max", "Trackbars")
        v_min = cv2.getTrackbarPos("V_min", "Trackbars")
        v_max = cv2.getTrackbarPos("V_max", "Trackbars")
        
        mask = cv2.inRange(hsv, np.array([h_min, s_min, v_min]),
                           np.array([h_max, s_max, v_max]))
        
        # 形态学处理
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        
        result = cv2.bitwise_and(frame, frame, mask=mask)
        
        # 在掩码上找轮廓并画出来
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        min_area = cv2.getTrackbarPos("Min Area", "Trackbars")
        
        for cnt in contours:
            if cv2.contourArea(cnt) > min_area:
                (cx, cy), r = cv2.minEnclosingCircle(cnt)
                cv2.circle(result, (int(cx), int(cy)), int(r), (0, 255, 0), 2)
        
        combined = np.hstack([frame, result])
        cv2.imshow("Color Tuner", combined)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            print(f"HSV 下界: [{h_min}, {s_min}, {v_min}]")
            print(f"HSV 上界: [{h_max}, {s_max}, {v_max}]")
            print(f"最小面积: {min_area}")
    
    cap.release()
    cv2.destroyAllWindows()


def run_camera_demo(color: str = 'red'):
    """摄像头实时色块追踪演示"""
    print("=" * 60)
    print(f"  色块追踪 - 颜色: {color}")
    print("  按 'q' 退出 | 按 't' 开启标定模式")
    print("=" * 60)
    
    tracker = ColorBlobTracker(color_name=color)
    
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
        
        result = tracker.detect(frame)
        tracker.update_fps()
        
        vis = tracker.draw(frame, result, show_mask=True)
        cv2.imshow("Color Blob Tracker", vis)
        
        if result:
            print(f"\r  Pos:({result['cx']},{result['cy']}) "
                  f"Ctrl:({result['control_x']:.1f},{result['control_y']:.1f})",
                  end='', flush=True)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('t'):
            cap.release()
            cv2.destroyAllWindows()
            color_tuner()
            cap = cv2.VideoCapture(0)
    
    print()
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description="色块追踪")
    parser.add_argument('--color', type=str, default='red',
                        choices=list(COLOR_PRESETS.keys()),
                        help='追踪颜色')
    parser.add_argument('--tune', action='store_true', help='启动颜色标定模式')
    args = parser.parse_args()
    
    if args.tune:
        color_tuner()
    else:
        run_camera_demo(args.color)
