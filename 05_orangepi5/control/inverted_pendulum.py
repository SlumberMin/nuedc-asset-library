#!/usr/bin/env python3
"""
倒立摆控制器 - 视觉检测+LQR控制
通过摄像头检测摆杆角度，使用LQR控制器实现倒立摆平衡
适用于Orange Pi 5 + OpenCV方案
"""
import cv2
import numpy as np
import time
from typing import Tuple, Optional, List
import sys
import os
from enum import Enum

# 添加项目根目录到路径
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from control.pid_controller import PIDController


class PendulumState(Enum):
    """倒立摆状态"""
    BALANCING = 0      # 平衡状态
    FALLING = 1        # 倒下状态
    RECOVERING = 2     # 恢复状态
    STOPPED = 3        # 停止状态


class PendulumDetector:
    """
    摆杆检测器
    使用颜色和形状检测摆杆角度
    """
    
    def __init__(self, 
                 pivot_color_lower: Tuple[int, int, int] = (0, 0, 200),
                 pivot_color_upper: Tuple[int, int, int] = (180, 30, 255),
                 rod_color_lower: Tuple[int, int, int] = (0, 100, 100),
                 rod_color_upper: Tuple[int, int, int] = (10, 255, 255),
                 min_rod_length: int = 50,
                 max_rod_length: int = 300):
        """
        初始化摆杆检测器
        
        Args:
            pivot_color_lower: 支点颜色HSV下限
            pivot_color_upper: 支点颜色HSV上限
            rod_color_lower: 摆杆颜色HSV下限
            rod_color_upper: 摆杆颜色HSV上限
            min_rod_length: 最小摆杆长度
            max_rod_length: 最大摆杆长度
        """
        self.pivot_color_lower = np.array(pivot_color_lower)
        self.pivot_color_upper = np.array(pivot_color_upper)
        self.rod_color_lower = np.array(rod_color_lower)
        self.rod_color_upper = np.array(rod_color_upper)
        self.min_rod_length = min_rod_length
        self.max_rod_length = max_rod_length
        
        # 检测参数
        self.gaussian_kernel = (5, 5)
        self.morph_kernel_size = 5
        self.angle_smoothing = 0.3
        
    def detect_pivot(self, frame: np.ndarray) -> Optional[Tuple[int, int]]:
        """
        检测支点位置
        
        Args:
            frame: 输入图像
            
        Returns:
            支点位置 (x, y) 或 None
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # 创建支点颜色掩码
        mask = cv2.inRange(hsv, self.pivot_color_lower, self.pivot_color_upper)
        
        # 形态学操作
        kernel = np.ones((self.morph_kernel_size, self.morph_kernel_size), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        # 查找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None
        
        # 找到最大轮廓
        max_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(max_contour)
        
        if area < 100:  # 最小面积阈值
            return None
        
        # 计算中心点
        M = cv2.moments(max_contour)
        if M["m00"] == 0:
            return None
        
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        
        return (cx, cy)
    
    def detect_rod(self, frame: np.ndarray, pivot_pos: Optional[Tuple[int, int]]) -> Tuple[Optional[Tuple[int, int]], Optional[float]]:
        """
        检测摆杆角度
        
        Args:
            frame: 输入图像
            pivot_pos: 支点位置
            
        Returns:
            (摆杆末端位置, 摆杆角度弧度) 或 (None, None)
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # 创建摆杆颜色掩码
        mask = cv2.inRange(hsv, self.rod_color_lower, self.rod_color_upper)
        
        # 形态学操作
        kernel = np.ones((self.morph_kernel_size, self.morph_kernel_size), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        # 查找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None, None
        
        # 找到最大轮廓
        max_contour = max(contours, key=cv2.contourArea)
        
        # 计算最小外接矩形
        rect = cv2.minAreaRect(max_contour)
        center, size, angle = rect
        
        # 计算摆杆长度
        length = max(size)
        
        if length < self.min_rod_length or length > self.max_rod_length:
            return None, None
        
        # 根据矩形方向计算摆杆角度
        if size[0] < size[1]:
            angle_rad = np.radians(angle)
        else:
            angle_rad = np.radians(angle + 90)
        
        # 如果有支点位置，计算相对于支点的角度
        if pivot_pos is not None:
            # 计算摆杆中心到支点的方向
            dx = center[0] - pivot_pos[0]
            dy = center[1] - pivot_pos[1]
            
            # 计算角度（相对于垂直向下方向）
            angle_rad = np.arctan2(dx, -dy)  # 注意坐标系转换
        
        # 计算摆杆末端位置（估计）
        if pivot_pos is not None:
            end_x = int(pivot_pos[0] + length * np.sin(angle_rad))
            end_y = int(pivot_pos[1] - length * np.cos(angle_rad))
            end_pos = (end_x, end_y)
        else:
            end_pos = (int(center[0]), int(center[1]))
        
        return end_pos, angle_rad
    
    def detect(self, frame: np.ndarray) -> Tuple[Optional[Tuple[int, int]], Optional[Tuple[int, int]], Optional[float]]:
        """
        检测倒立摆
        
        Args:
            frame: 输入图像
            
        Returns:
            (支点位置, 摆杆末端位置, 摆杆角度) 或 (None, None, None)
        """
        pivot_pos = self.detect_pivot(frame)
        rod_end, angle = self.detect_rod(frame, pivot_pos)
        
        return pivot_pos, rod_end, angle
    
    def set_pivot_color(self, lower: Tuple[int, int, int], upper: Tuple[int, int, int]):
        """
        设置支点颜色范围
        
        Args:
            lower: HSV下限
            upper: HSV上限
        """
        self.pivot_color_lower = np.array(lower)
        self.pivot_color_upper = np.array(upper)
    
    def set_rod_color(self, lower: Tuple[int, int, int], upper: Tuple[int, int, int]):
        """
        设置摆杆颜色范围
        
        Args:
            lower: HSV下限
            upper: HSV上限
        """
        self.rod_color_lower = np.array(lower)
        self.rod_color_upper = np.array(upper)


class LQRController:
    """
    LQR控制器
    用于倒立摆的最优控制
    """
    
    def __init__(self, 
                 dt: float = 0.01,
                 m: float = 0.1,      # 摆杆质量 (kg)
                 l: float = 0.5,       # 摆杆长度 (m)
                 g: float = 9.81,      # 重力加速度 (m/s^2)
                 Q: np.ndarray = None,
                 R: np.ndarray = None):
        """
        初始化LQR控制器
        
        Args:
            dt: 控制周期 (秒)
            m: 摆杆质量 (kg)
            l: 摆杆长度 (m)
            g: 重力加速度 (m/s^2)
            Q: 状态权重矩阵
            R: 控制权重矩阵
        """
        self.dt = dt
        self.m = m
        self.l = l
        self.g = g
        
        # 系统矩阵 (线性化后)
        # 状态: [角度, 角速度]
        # 控制: [角加速度]
        self.A = np.array([
            [0, 1],
            [g/l, 0]
        ])
        
        self.B = np.array([
            [0],
            [1/(m*l**2)]
        ])
        
        # 权重矩阵
        if Q is None:
            Q = np.array([
                [100, 0],   # 角度权重
                [0, 10]     # 角速度权重
            ])
        
        if R is None:
            R = np.array([[1]])  # 控制权重
        
        self.Q = Q
        self.R = R
        
        # 计算LQR增益
        self.K = self._compute_lqr_gain()
        
        # 状态估计
        self.state = np.array([[0], [0]])  # [角度, 角速度]
        self.prev_angle = 0.0
        
    def _compute_lqr_gain(self) -> np.ndarray:
        """
        计算LQR增益矩阵
        
        Returns:
            增益矩阵K
        """
        # 简化的LQR求解（离散时间近似）
        # 使用迭代法求解代数Riccati方程
        
        A = self.A
        B = self.B
        Q = self.Q
        R = self.R
        
        # 离散化
        Ad = np.eye(2) + A * self.dt
        Bd = B * self.dt
        
        # 迭代求解Riccati方程
        P = Q.copy()
        for _ in range(100):
            K = np.linalg.inv(R + Bd.T @ P @ Bd) @ Bd.T @ P @ Ad
            P = Q + Ad.T @ P @ Ad - Ad.T @ P @ Bd @ K
        
        return K
    
    def update(self, angle: float, angular_velocity: float = None) -> float:
        """
        更新控制器
        
        Args:
            angle: 当前角度 (弧度)
            angular_velocity: 当前角速度 (弧度/秒)，如果为None则自动计算
            
        Returns:
            控制输出
        """
        # 如果没有提供角速度，使用数值微分
        if angular_velocity is None:
            angular_velocity = (angle - self.prev_angle) / self.dt
            self.prev_angle = angle
        
        # 更新状态
        self.state = np.array([[angle], [angular_velocity]])
        
        # 计算控制量
        u = -self.K @ self.state
        
        return float(u[0, 0])
    
    def reset(self):
        """重置控制器状态"""
        self.state = np.array([[0], [0]])
        self.prev_angle = 0.0


class InvertedPendulum:
    """
    倒立摆控制器
    使用视觉检测和LQR控制实现倒立摆平衡
    """
    
    def __init__(self, 
                 camera_id: int = 0,
                 resolution: Tuple[int, int] = (640, 480),
                 control_frequency: float = 100.0,
                 angle_threshold: float = np.radians(30),  # 30度
                 pivot_color_lower: Tuple[int, int, int] = (0, 0, 200),
                 pivot_color_upper: Tuple[int, int, int] = (180, 30, 255),
                 rod_color_lower: Tuple[int, int, int] = (0, 100, 100),
                 rod_color_upper: Tuple[int, int, int] = (10, 255, 255),
                 lqr_params: dict = None):
        """
        初始化倒立摆控制器
        
        Args:
            camera_id: 摄像头ID
            resolution: 分辨率 (宽, 高)
            control_frequency: 控制频率 (Hz)
            angle_threshold: 角度阈值 (弧度)
            pivot_color_lower: 支点颜色HSV下限
            pivot_color_upper: 支点颜色HSV上限
            rod_color_lower: 摆杆颜色HSV下限
            rod_color_upper: 摆杆颜色HSV上限
            lqr_params: LQR参数字典
        """
        self.camera_id = camera_id
        self.resolution = resolution
        self.control_frequency = control_frequency
        self.control_period = 1.0 / control_frequency
        self.angle_threshold = angle_threshold
        
        # 初始化摆杆检测器
        self.detector = PendulumDetector(
            pivot_color_lower=pivot_color_lower,
            pivot_color_upper=pivot_color_upper,
            rod_color_lower=rod_color_lower,
            rod_color_upper=rod_color_upper
        )
        
        # 初始化LQR控制器
        if lqr_params is None:
            lqr_params = {
                'dt': self.control_period,
                'm': 0.1,
                'l': 0.5,
                'g': 9.81
            }
        
        self.lqr = LQRController(**lqr_params)
        
        # 状态变量
        self.cap = None
        self.is_running = False
        self.state = PendulumState.STOPPED
        self.current_angle = 0.0
        self.current_angular_velocity = 0.0
        self.control_output = 0.0
        
        # 位置信息
        self.pivot_position = None
        self.rod_end_position = None
        
        # 性能统计
        self.frame_count = 0
        self.fps = 0.0
        self.last_fps_time = time.time()
        self.control_count = 0
        self.control_fps = 0.0
        self.last_control_time = time.time()
        
        # 历史数据
        self.angle_history = []
        self.control_history = []
        self.max_history = 1000
        
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
            
            # 尝试设置高帧率
            self.cap.set(cv2.CAP_PROP_FPS, 120)
            
            # 预热摄像头
            for _ in range(10):
                self.cap.read()
                
            print(f"倒立摆控制器初始化成功: {self.resolution}")
            self.is_running = True
            self.state = PendulumState.BALANCING
            return True
            
        except Exception as e:
            print(f"初始化失败: {e}")
            return False
    
    def update_control(self, angle: float) -> float:
        """
        更新控制输出
        
        Args:
            angle: 当前角度 (弧度)
            
        Returns:
            控制输出
        """
        # 检查是否超过阈值
        if abs(angle) > self.angle_threshold:
            self.state = PendulumState.FALLING
            return 0.0
        
        # 使用LQR计算控制量
        control = self.lqr.update(angle)
        
        # 限制控制输出
        max_control = 10.0
        control = max(-max_control, min(max_control, control))
        
        # 更新状态
        self.current_angle = angle
        self.control_output = control
        
        # 记录历史
        self.angle_history.append(angle)
        self.control_history.append(control)
        if len(self.angle_history) > self.max_history:
            self.angle_history.pop(0)
            self.control_history.pop(0)
        
        return control
    
    def process_frame(self, frame: np.ndarray) -> dict:
        """
        处理单帧图像
        
        Args:
            frame: 原始图像帧
            
        Returns:
            处理结果字典
        """
        # 检测倒立摆
        pivot_pos, rod_end, angle = self.detector.detect(frame)
        
        # 更新位置信息
        self.pivot_position = pivot_pos
        self.rod_end_position = rod_end
        
        # 计算控制量
        if angle is not None:
            control = self.update_control(angle)
        else:
            control = 0.0
        
        # 更新FPS
        self.update_fps()
        
        return {
            'pivot_position': pivot_pos,
            'rod_end_position': rod_end,
            'angle': angle,
            'angle_degrees': np.degrees(angle) if angle is not None else None,
            'control_output': control,
            'state': self.state,
            'detected': angle is not None
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
    
    def update_control_fps(self):
        """更新控制FPS计算"""
        self.control_count += 1
        current_time = time.time()
        elapsed = current_time - self.last_control_time
        
        if elapsed >= 1.0:
            self.control_fps = self.control_count / elapsed
            self.control_count = 0
            self.last_control_time = current_time
    
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
        
        # 绘制支点位置
        if result['pivot_position'] is not None:
            cv2.circle(debug_frame, result['pivot_position'], 10, (0, 255, 0), 2)
            cv2.putText(debug_frame, "Pivot", 
                       (result['pivot_position'][0] + 15, result['pivot_position'][1]), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        # 绘制摆杆末端位置
        if result['rod_end_position'] is not None:
            cv2.circle(debug_frame, result['rod_end_position'], 8, (0, 0, 255), -1)
            cv2.putText(debug_frame, "End", 
                       (result['rod_end_position'][0] + 10, result['rod_end_position'][1]), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        
        # 绘制摆杆连线
        if result['pivot_position'] is not None and result['rod_end_position'] is not None:
            cv2.line(debug_frame, result['pivot_position'], result['rod_end_position'], 
                    (255, 0, 0), 3)
        
        # 绘制垂直参考线
        if result['pivot_position'] is not None:
            px, py = result['pivot_position']
            cv2.line(debug_frame, (px, py), (px, py - 100), (255, 255, 0), 1)
        
        # 绘制状态信息
        state_text = f"State: {self.state.name}"
        cv2.putText(debug_frame, state_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        if result['angle'] is not None:
            angle_text = f"Angle: {result['angle_degrees']:.1f}°"
            cv2.putText(debug_frame, angle_text, (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        control_text = f"Control: {result['control_output']:.2f}"
        cv2.putText(debug_frame, control_text, (10, 90), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        fps_text = f"FPS: {self.fps:.1f} | Control FPS: {self.control_fps:.1f}"
        cv2.putText(debug_frame, fps_text, (10, 120), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # 绘制角度历史图表
        if len(self.angle_history) > 1:
            self._draw_angle_chart(debug_frame, self.angle_history)
        
        return debug_frame
    
    def _draw_angle_chart(self, frame: np.ndarray, history: List[float]):
        """
        绘制角度历史图表
        
        Args:
            frame: 图像帧
            history: 角度历史
        """
        height, width = frame.shape[:2]
        chart_height = 100
        chart_width = 200
        chart_x = width - chart_width - 10
        chart_y = height - chart_height - 10
        
        # 绘制背景
        cv2.rectangle(frame, (chart_x, chart_y), 
                     (chart_x + chart_width, chart_y + chart_height), 
                     (0, 0, 0), -1)
        cv2.rectangle(frame, (chart_x, chart_y), 
                     (chart_x + chart_width, chart_y + chart_height), 
                     (255, 255, 255), 1)
        
        # 绘制零线
        zero_y = chart_y + chart_height // 2
        cv2.line(frame, (chart_x, zero_y), (chart_x + chart_width, zero_y), 
                (128, 128, 128), 1)
        
        # 绘制角度曲线
        if len(history) > 1:
            points = []
            for i, angle in enumerate(history[-chart_width:]):
                x = chart_x + i
                y = int(zero_y - (angle / np.radians(45)) * (chart_height // 2))
                y = max(chart_y, min(chart_y + chart_height, y))
                points.append((x, y))
            
            for i in range(1, len(points)):
                cv2.line(frame, points[i-1], points[i], (0, 255, 0), 1)
        
        # 添加标签
        cv2.putText(frame, "Angle History", (chart_x, chart_y - 5), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    
    def run(self, show_debug: bool = True, motor_controller=None):
        """
        运行倒立摆控制器主循环
        
        Args:
            show_debug: 是否显示调试窗口
            motor_controller: 电机控制器（可选）
        """
        if not self.initialize():
            return
        
        print("倒立摆控制器已启动，按 'q' 退出")
        print("按 'r' 重置LQR控制器")
        print("按 'c' 重新检测颜色")
        
        try:
            last_control_time = time.time()
            
            while self.is_running:
                # 读取帧
                ret, frame = self.cap.read()
                if not ret:
                    print("警告: 无法读取图像帧")
                    continue
                
                # 处理帧
                result = self.process_frame(frame)
                
                # 控制频率限制
                current_time = time.time()
                if current_time - last_control_time >= self.control_period:
                    # 控制电机
                    if motor_controller and result['detected']:
                        # 假设有set_torque或set_speed方法
                        if hasattr(motor_controller, 'set_torque'):
                            motor_controller.set_torque(result['control_output'])
                        elif hasattr(motor_controller, 'set_speed'):
                            motor_controller.set_speed(result['control_output'])
                    
                    last_control_time = current_time
                    self.update_control_fps()
                
                # 显示调试窗口
                if show_debug:
                    debug_frame = self.draw_debug(frame, result)
                    cv2.imshow('Inverted Pendulum Debug', debug_frame)
                    
                    # 按键处理
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        break
                    elif key == ord('r'):
                        # 重置LQR控制器
                        self.lqr.reset()
                        self.angle_history.clear()
                        self.control_history.clear()
                        self.state = PendulumState.BALANCING
                        print("LQR控制器已重置")
                    elif key == ord('c'):
                        # 重新检测颜色（这里只是示例，实际需要颜色校准界面）
                        print("颜色校准功能需要实现")
                        
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
        print("倒立摆控制器已停止")
    
    def get_status(self) -> dict:
        """
        获取当前状态
        
        Returns:
            状态字典
        """
        return {
            'is_running': self.is_running,
            'state': self.state.name,
            'current_angle': self.current_angle,
            'current_angle_degrees': np.degrees(self.current_angle),
            'control_output': self.control_output,
            'pivot_position': self.pivot_position,
            'rod_end_position': self.rod_end_position,
            'fps': self.fps,
            'control_fps': self.control_fps
        }
    
    def set_parameters(self, **kwargs):
        """
        动态调整参数
        
        Args:
            **kwargs: 参数字典
        """
        if 'control_frequency' in kwargs:
            self.control_frequency = kwargs['control_frequency']
            self.control_period = 1.0 / self.control_frequency
            self.lqr.dt = self.control_period
            self.lqr.K = self.lqr._compute_lqr_gain()
        
        if 'angle_threshold' in kwargs:
            self.angle_threshold = kwargs['angle_threshold']
        
        if 'pivot_color_lower' in kwargs and 'pivot_color_upper' in kwargs:
            self.detector.set_pivot_color(kwargs['pivot_color_lower'], kwargs['pivot_color_upper'])
        
        if 'rod_color_lower' in kwargs and 'rod_color_upper' in kwargs:
            self.detector.set_rod_color(kwargs['rod_color_lower'], kwargs['rod_color_upper'])
        
        if 'Q' in kwargs:
            self.lqr.Q = kwargs['Q']
            self.lqr.K = self.lqr._compute_lqr_gain()
        
        if 'R' in kwargs:
            self.lqr.R = kwargs['R']
            self.lqr.K = self.lqr._compute_lqr_gain()


class InvertedPendulumWithPID(InvertedPendulum):
    """
    使用PID控制的倒立摆控制器
    作为LQR控制器的备选方案
    """
    
    def __init__(self, 
                 pid_params: dict = None,
                 **kwargs):
        """
        初始化PID倒立摆控制器
        
        Args:
            pid_params: PID参数字典 {'kp': 1.0, 'ki': 0.0, 'kd': 0.5}
            **kwargs: 其他参数
        """
        super().__init__(**kwargs)
        
        # 默认PID参数
        if pid_params is None:
            pid_params = {'kp': 2.0, 'ki': 0.01, 'kd': 0.5}
        
        # 初始化PID控制器
        self.pid = PIDController(
            kp=pid_params['kp'],
            ki=pid_params['ki'],
            kd=pid_params['kd'],
            output_limits=(-10, 10)
        )
    
    def update_control(self, angle: float) -> float:
        """
        使用PID更新控制输出
        
        Args:
            angle: 当前角度 (弧度)
            
        Returns:
            控制输出
        """
        # 检查是否超过阈值
        if abs(angle) > self.angle_threshold:
            self.state = PendulumState.FALLING
            return 0.0
        
        # 使用PID计算控制量
        control = self.pid.update(angle)
        
        # 限制控制输出
        max_control = 10.0
        control = max(-max_control, min(max_control, control))
        
        # 更新状态
        self.current_angle = angle
        self.control_output = control
        
        # 记录历史
        self.angle_history.append(angle)
        self.control_history.append(control)
        if len(self.angle_history) > self.max_history:
            self.angle_history.pop(0)
            self.control_history.pop(0)
        
        return control


# 测试代码
if __name__ == "__main__":
    # 创建倒立摆控制器
    pendulum = InvertedPendulum(
        camera_id=0,
        resolution=(640, 480),
        control_frequency=100.0,
        angle_threshold=np.radians(30),
        pivot_color_lower=(0, 0, 200),
        pivot_color_upper=(180, 30, 255),
        rod_color_lower=(0, 100, 100),
        rod_color_upper=(10, 255, 255),
        lqr_params={
            'dt': 0.01,
            'm': 0.1,
            'l': 0.5,
            'g': 9.81
        }
    )
    
    # 运行
    pendulum.run(show_debug=True)