#!/usr/bin/env python3
"""
视觉循线控制器 - 替代红外传感器方案
利用摄像头实时检测线路，结合PID控制实现精准循线
适用于Orange Pi 5 + OpenCV方案
"""
import cv2
import numpy as np
import time
from enum import Enum
from typing import Optional, Tuple, List
import sys
import os

# 添加项目根目录到路径
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from control.pid_controller import PIDController
from control.motor_controller import MotorController


class LineType(Enum):
    """线路类型"""
    BLACK_LINE = 0      # 黑线（白底）
    WHITE_LINE = 1      # 白线（黑底）
    COLOR_LINE = 2      # 彩色线


class LineFollower:
    """
    视觉循线控制器
    通过摄像头检测线路，计算偏移量并使用PID控制转向
    """
    
    def __init__(self, 
                 camera_id: int = 0,
                 resolution: Tuple[int, int] = (640, 480),
                 line_type: LineType = LineType.BLACK_LINE,
                 roi_y_start: float = 0.5,
                 roi_height: float = 0.3,
                 pid_params: dict = None,
                 base_speed: float = 50.0,
                 max_speed: float = 100.0):
        """
        初始化循线控制器
        
        Args:
            camera_id: 摄像头ID
            resolution: 分辨率 (宽, 高)
            line_type: 线路类型
            roi_y_start: 感兴趣区域起始Y位置 (比例0-1)
            roi_height: 感兴趣区域高度 (比例0-1)
            pid_params: PID参数字典 {'kp': 1.0, 'ki': 0.0, 'kd': 0.5}
            base_speed: 基础速度
            max_speed: 最大速度
        """
        self.camera_id = camera_id
        self.resolution = resolution
        self.line_type = line_type
        self.roi_y_start = roi_y_start
        self.roi_height = roi_height
        self.base_speed = base_speed
        self.max_speed = max_speed
        
        # 默认PID参数
        if pid_params is None:
            pid_params = {'kp': 0.8, 'ki': 0.01, 'kd': 0.3}
        
        # 初始化PID控制器
        self.pid = PIDController(
            kp=pid_params['kp'],
            ki=pid_params['ki'],
            kd=pid_params['kd'],
            output_limits=(-100, 100)
        )
        
        # 摄像头和状态
        self.cap = None
        self.is_running = False
        self.current_offset = 0.0
        self.line_detected = False
        
        # 图像处理参数
        self.threshold_value = 128
        self.min_line_area = 500
        self.gaussian_kernel = (5, 5)
        self.morph_kernel_size = 5
        
        # 性能统计
        self.frame_count = 0
        self.fps = 0.0
        self.last_fps_time = time.time()
        
    def initialize(self) -> bool:
        """
        初始化摄像头
        
        Returns:
            初始化是否成功
        """
        try:
            self.cap = cv2.VideoCapture(self.camera_id)
            if not self.cap.isOpened():
                print(f"错误: 无法打开摄像头 {self.camera_id}")
                return False
            
            # 设置分辨率
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])
            
            # 预热摄像头
            for _ in range(5):
                self.cap.read()
                
            print(f"摄像头初始化成功: {self.resolution}")
            self.is_running = True
            return True
            
        except Exception as e:
            print(f"初始化失败: {e}")
            return False
    
    def preprocess_frame(self, frame: np.ndarray) -> np.ndarray:
        """
        预处理图像帧
        
        Args:
            frame: 原始图像帧
            
        Returns:
            预处理后的二值图像
        """
        # 转换为灰度图
        if len(frame.shape) == 3:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        else:
            gray = frame.copy()
        
        # 高斯模糊去噪
        blurred = cv2.GaussianBlur(gray, self.gaussian_kernel, 0)
        
        # 自适应阈值分割
        if self.line_type == LineType.BLACK_LINE:
            binary = cv2.threshold(blurred, self.threshold_value, 255, cv2.THRESH_BINARY_INV)[1]
        else:
            binary = cv2.threshold(blurred, self.threshold_value, 255, cv2.THRESH_BINARY)[1]
        
        # 形态学操作清理噪声
        kernel = np.ones((self.morph_kernel_size, self.morph_kernel_size), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        
        return binary
    
    def extract_roi(self, binary_image: np.ndarray) -> np.ndarray:
        """
        提取感兴趣区域 (ROI)
        
        Args:
            binary_image: 二值化图像
            
        Returns:
            ROI区域图像
        """
        height, width = binary_image.shape[:2]
        y_start = int(height * self.roi_y_start)
        y_end = min(y_start + int(height * self.roi_height), height)
        
        roi = binary_image[y_start:y_end, :]
        return roi
    
    def detect_line(self, roi_image: np.ndarray) -> Tuple[float, bool]:
        """
        检测线路位置
        
        Args:
            roi_image: ROI区域图像
            
        Returns:
            (偏移量, 是否检测到线路)
        """
        height, width = roi_image.shape[:2]
        
        # 查找轮廓
        contours, _ = cv2.findContours(roi_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return 0.0, False
        
        # 找到最大轮廓（假设是线路）
        max_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(max_contour)
        
        if area < self.min_line_area:
            return 0.0, False
        
        # 计算轮廓的矩
        M = cv2.moments(max_contour)
        if M["m00"] == 0:
            return 0.0, False
        
        # 计算中心点
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        
        # 计算相对于中心的偏移量 (-1 到 1)
        center_x = width // 2
        offset = (cx - center_x) / (width // 2)
        
        return offset, True
    
    def compute_control(self, offset: float) -> Tuple[float, float]:
        """
        计算左右轮速度
        
        Args:
            offset: 线路偏移量 (-1 到 1)
            
        Returns:
            (左轮速度, 右轮速度)
        """
        # 使用PID计算转向修正量
        steering = self.pid.update(offset)
        
        # 计算左右轮速度
        left_speed = self.base_speed + steering
        right_speed = self.base_speed - steering
        
        # 限制速度范围
        left_speed = max(-self.max_speed, min(self.max_speed, left_speed))
        right_speed = max(-self.max_speed, min(self.max_speed, right_speed))
        
        return left_speed, right_speed
    
    def process_frame(self, frame: np.ndarray) -> dict:
        """
        处理单帧图像
        
        Args:
            frame: 原始图像帧
            
        Returns:
            处理结果字典
        """
        # 预处理
        binary = self.preprocess_frame(frame)
        
        # 提取ROI
        roi = self.extract_roi(binary)
        
        # 检测线路
        offset, detected = self.detect_line(roi)
        
        # 更新状态
        self.current_offset = offset
        self.line_detected = detected
        
        # 计算控制量
        if detected:
            left_speed, right_speed = self.compute_control(offset)
        else:
            # 丢失线路时停车或慢速搜索
            left_speed = 0.0
            right_speed = 0.0
        
        # 更新FPS
        self.update_fps()
        
        return {
            'offset': offset,
            'detected': detected,
            'left_speed': left_speed,
            'right_speed': right_speed,
            'binary': binary,
            'roi': roi
        }
    
    def update_fps(self):
        """更新FPS计算"""
        self.frame_count += 1
        current_time = time.time()
        elapsed = current_time - self.last_fps_time
        
        if elapsed >= 1.0:
            self.fps = self.frame_count / elapsed
            self.frame_count = 0
            self.last_fps_time = current_time
    
    def draw_debug(self, frame: np.ndarray, result: dict) -> np.ndarray:
        """
        绘制调试信息
        
        Args:
            frame: 原始图像帧
            result: 处理结果
            
        Returns:
            绘制了调试信息的图像
        """
        debug_frame = frame.copy()
        height, width = frame.shape[:2]
        
        # 绘制ROI区域
        y_start = int(height * self.roi_y_start)
        y_end = min(y_start + int(height * self.roi_height), height)
        cv2.rectangle(debug_frame, (0, y_start), (width, y_end), (0, 255, 0), 2)
        
        # 绘制中心线
        cv2.line(debug_frame, (width // 2, 0), (width // 2, height), (255, 0, 0), 1)
        
        # 绘制偏移量指示
        if result['detected']:
            offset_x = int(width // 2 + result['offset'] * (width // 2))
            cv2.circle(debug_frame, (offset_x, y_start + (y_end - y_start) // 2), 
                      10, (0, 0, 255), -1)
        
        # 绘制状态信息
        status_text = f"Offset: {result['offset']:.2f} | Detected: {result['detected']}"
        cv2.putText(debug_frame, status_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        speed_text = f"L: {result['left_speed']:.1f} R: {result['right_speed']:.1f}"
        cv2.putText(debug_frame, speed_text, (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        fps_text = f"FPS: {self.fps:.1f}"
        cv2.putText(debug_frame, fps_text, (10, 90), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        return debug_frame
    
    def run(self, show_debug: bool = True, motor_controller: Optional[MotorController] = None):
        """
        运行循线控制器主循环
        
        Args:
            show_debug: 是否显示调试窗口
            motor_controller: 电机控制器（可选）
        """
        if not self.initialize():
            return
        
        print("循线控制器已启动，按 'q' 退出")
        
        try:
            while self.is_running:
                # 读取帧
                ret, frame = self.cap.read()
                if not ret:
                    print("警告: 无法读取图像帧")
                    continue
                
                # 处理帧
                result = self.process_frame(frame)
                
                # 控制电机
                if motor_controller and result['detected']:
                    motor_controller.set_motors(result['left_speed'], result['right_speed'])
                
                # 显示调试窗口
                if show_debug:
                    debug_frame = self.draw_debug(frame, result)
                    cv2.imshow('Line Follower Debug', debug_frame)
                    
                    # 按'q'退出
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                        
        except KeyboardInterrupt:
            print("\n用户中断")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """清理资源"""
        self.is_running = False
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()
        print("循线控制器已停止")
    
    def get_status(self) -> dict:
        """
        获取当前状态
        
        Returns:
            状态字典
        """
        return {
            'is_running': self.is_running,
            'line_detected': self.line_detected,
            'current_offset': self.current_offset,
            'fps': self.fps,
            'base_speed': self.base_speed
        }
    
    def set_parameters(self, **kwargs):
        """
        动态调整参数
        
        Args:
            **kwargs: 参数字典
        """
        if 'base_speed' in kwargs:
            self.base_speed = kwargs['base_speed']
        if 'max_speed' in kwargs:
            self.max_speed = kwargs['max_speed']
        if 'roi_y_start' in kwargs:
            self.roi_y_start = kwargs['roi_y_start']
        if 'roi_height' in kwargs:
            self.roi_height = kwargs['roi_height']
        if 'threshold_value' in kwargs:
            self.threshold_value = kwargs['threshold_value']
        if 'kp' in kwargs:
            self.pid.kp = kwargs['kp']
        if 'ki' in kwargs:
            self.pid.ki = kwargs['ki']
        if 'kd' in kwargs:
            self.pid.kd = kwargs['kd']


class ColorLineFollower(LineFollower):
    """
    彩色线路循线控制器
    扩展基础循线控制器以支持彩色线路检测
    """
    
    def __init__(self, 
                 camera_id: int = 0,
                 resolution: Tuple[int, int] = (640, 480),
                 target_color: Tuple[int, int, int] = (0, 0, 255),  # BGR格式，默认红色
                 color_range: Tuple[int, int] = (20, 30),  # HSV范围
                 **kwargs):
        """
        初始化彩色循线控制器
        
        Args:
            camera_id: 摄像头ID
            resolution: 分辨率
            target_color: 目标颜色 (BGR)
            color_range: HSV范围 (色相范围, 饱和度/明度范围)
            **kwargs: 其他参数
        """
        super().__init__(camera_id, resolution, LineType.COLOR_LINE, **kwargs)
        self.target_color = target_color
        self.color_range = color_range
        self.target_hsv = self._bgr_to_hsv(target_color)
        
    def _bgr_to_hsv(self, bgr_color: Tuple[int, int, int]) -> Tuple[int, int, int]:
        """将BGR颜色转换为HSV"""
        import colorsys
        b, g, r = [x / 255.0 for x in bgr_color]
        h, s, v = colorsys.rgb_to_hsv(r, g, b)
        return (int(h * 180), int(s * 255), int(v * 255))
    
    def preprocess_frame(self, frame: np.ndarray) -> np.ndarray:
        """
        预处理图像帧（彩色版本）
        
        Args:
            frame: 原始图像帧
            
        Returns:
            二值化掩码
        """
        # 转换到HSV颜色空间
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # 计算目标颜色的HSV范围
        h, s, v = self.target_hsv
        h_range, sv_range = self.color_range
        
        # 创建掩码
        lower_bound = np.array([
            max(0, h - h_range),
            max(0, s - sv_range),
            max(0, v - sv_range)
        ])
        upper_bound = np.array([
            min(180, h + h_range),
            min(255, s + sv_range),
            min(255, v + sv_range)
        ])
        
        mask = cv2.inRange(hsv, lower_bound, upper_bound)
        
        # 形态学操作
        kernel = np.ones((self.morph_kernel_size, self.morph_kernel_size), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        return mask
    
    def set_target_color(self, bgr_color: Tuple[int, int, int]):
        """
        设置目标颜色
        
        Args:
            bgr_color: BGR格式的颜色值
        """
        self.target_color = bgr_color
        self.target_hsv = self._bgr_to_hsv(bgr_color)


# 测试代码
if __name__ == "__main__":
    # 创建循线控制器
    follower = LineFollower(
        camera_id=0,
        resolution=(640, 480),
        line_type=LineType.BLACK_LINE,
        pid_params={'kp': 0.8, 'ki': 0.01, 'kd': 0.3},
        base_speed=50.0
    )
    
    # 运行
    follower.run(show_debug=True)