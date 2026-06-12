#!/usr/bin/env python3
"""
滚球平衡控制器 - 视觉检测+双轴PID控制
通过摄像头检测球的位置，使用双轴PID控制使球保持在目标位置
适用于Orange Pi 5 + OpenCV方案
"""
import cv2
import numpy as np
import time
from typing import Tuple, Optional, List
import sys
import os

# 添加项目根目录到路径
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from control.pid_controller import PIDController


class BallDetector:
    """
    球体检测器
    使用颜色和形状检测球体位置
    """
    
    def __init__(self, 
                 color_lower: Tuple[int, int, int] = (0, 100, 100),
                 color_upper: Tuple[int, int, int] = (10, 255, 255),
                 min_radius: int = 10,
                 max_radius: int = 100,
                 min_area: int = 100):
        """
        初始化球体检测器
        
        Args:
            color_lower: HSV颜色下限
            color_upper: HSV颜色上限
            min_radius: 最小半径
            max_radius: 最大半径
            min_area: 最小面积
        """
        self.color_lower = np.array(color_lower)
        self.color_upper = np.array(color_upper)
        self.min_radius = min_radius
        self.max_radius = max_radius
        self.min_area = min_area
        
        # 检测参数
        self.gaussian_kernel = (5, 5)
        self.morph_kernel_size = 5
        self.detection_method = 'color'  # 'color', 'contour', 'hough'
        
    def detect_color(self, frame: np.ndarray) -> Tuple[Optional[Tuple[int, int]], Optional[int]]:
        """
        使用颜色检测球体
        
        Args:
            frame: 输入图像
            
        Returns:
            ((cx, cy), radius) 或 (None, None)
        """
        # 转换到HSV颜色空间
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # 创建颜色掩码
        mask = cv2.inRange(hsv, self.color_lower, self.color_upper)
        
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
        area = cv2.contourArea(max_contour)
        
        if area < self.min_area:
            return None, None
        
        # 计算最小外接圆
        (cx, cy), radius = cv2.minEnclosingCircle(max_contour)
        cx, cy, radius = int(cx), int(cy), int(radius)
        
        # 检查半径范围
        if radius < self.min_radius or radius > self.max_radius:
            return None, None
        
        return (cx, cy), radius
    
    def detect_hough(self, frame: np.ndarray) -> Tuple[Optional[Tuple[int, int]], Optional[int]]:
        """
        使用霍夫圆检测球体
        
        Args:
            frame: 输入图像
            
        Returns:
            ((cx, cy), radius) 或 (None, None)
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, self.gaussian_kernel, 0)
        
        # 霍夫圆检测
        circles = cv2.HoughCircles(
            blurred, 
            cv2.HOUGH_GRADIENT, 
            dp=1, 
            minDist=50,
            param1=100, 
            param2=30,
            minRadius=self.min_radius,
            maxRadius=self.max_radius
        )
        
        if circles is None:
            return None, None
        
        # 取第一个检测到的圆
        circles = np.uint16(np.around(circles))
        cx, cy, radius = circles[0][0]
        
        return (int(cx), int(cy)), int(radius)
    
    def detect(self, frame: np.ndarray) -> Tuple[Optional[Tuple[int, int]], Optional[int]]:
        """
        检测球体位置
        
        Args:
            frame: 输入图像
            
        Returns:
            ((cx, cy), radius) 或 (None, None)
        """
        if self.detection_method == 'color':
            return self.detect_color(frame)
        elif self.detection_method == 'hough':
            return self.detect_hough(frame)
        else:
            return self.detect_color(frame)
    
    def set_color_range(self, lower: Tuple[int, int, int], upper: Tuple[int, int, int]):
        """
        设置颜色范围
        
        Args:
            lower: HSV下限
            upper: HSV上限
        """
        self.color_lower = np.array(lower)
        self.color_upper = np.array(upper)


class BallBalancer:
    """
    滚球平衡控制器
    使用双轴PID控制使球保持在平台中心
    """
    
    def __init__(self, 
                 camera_id: int = 0,
                 resolution: Tuple[int, int] = (640, 480),
                 target_position: Tuple[float, float] = (0.5, 0.5),
                 platform_center: Tuple[int, int] = None,
                 pid_x_params: dict = None,
                 pid_y_params: dict = None,
                 ball_color_lower: Tuple[int, int, int] = (0, 100, 100),
                 ball_color_upper: Tuple[int, int, int] = (10, 255, 255)):
        """
        初始化滚球平衡控制器
        
        Args:
            camera_id: 摄像头ID
            resolution: 分辨率 (宽, 高)
            target_position: 目标位置 (比例x, 比例y)
            platform_center: 平台中心坐标 (像素)
            pid_x_params: X轴PID参数 {'kp': 1.0, 'ki': 0.0, 'kd': 0.5}
            pid_y_params: Y轴PID参数 {'kp': 1.0, 'ki': 0.0, 'kd': 0.5}
            ball_color_lower: 球颜色HSV下限
            ball_color_upper: 球颜色HSV上限
        """
        self.camera_id = camera_id
        self.resolution = resolution
        self.target_position = target_position
        
        # 平台中心默认为图像中心
        if platform_center is None:
            platform_center = (resolution[0] // 2, resolution[1] // 2)
        self.platform_center = platform_center
        
        # 默认PID参数
        if pid_x_params is None:
            pid_x_params = {'kp': 0.5, 'ki': 0.01, 'kd': 0.2}
        if pid_y_params is None:
            pid_y_params = {'kp': 0.5, 'ki': 0.01, 'kd': 0.2}
        
        # 初始化PID控制器
        self.pid_x = PIDController(
            kp=pid_x_params['kp'],
            ki=pid_x_params['ki'],
            kd=pid_x_params['kd'],
            output_limits=(-30, 30)  # 角度限制
        )
        
        self.pid_y = PIDController(
            kp=pid_y_params['kp'],
            ki=pid_y_params['ki'],
            kd=pid_y_params['kd'],
            output_limits=(-30, 30)  # 角度限制
        )
        
        # 球体检测器
        self.detector = BallDetector(
            color_lower=ball_color_lower,
            color_upper=ball_color_upper
        )
        
        # 状态变量
        self.cap = None
        self.is_running = False
        self.ball_position = None
        self.ball_radius = None
        self.current_angle_x = 0.0
        self.current_angle_y = 0.0
        
        # 目标位置（像素）
        self.target_pixel = (
            int(resolution[0] * target_position[0]),
            int(resolution[1] * target_position[1])
        )
        
        # 性能统计
        self.frame_count = 0
        self.fps = 0.0
        self.last_fps_time = time.time()
        self.position_history = []
        self.max_history = 100
        
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
                
            print(f"滚球平衡控制器初始化成功: {self.resolution}")
            self.is_running = True
            return True
            
        except Exception as e:
            print(f"初始化失败: {e}")
            return False
    
    def compute_error(self, ball_pos: Tuple[int, int]) -> Tuple[float, float]:
        """
        计算位置误差
        
        Args:
            ball_pos: 球当前位置 (像素)
            
        Returns:
            (x误差, y误差) 归一化到[-1, 1]
        """
        # 计算相对于平台中心的误差
        error_x = (ball_pos[0] - self.platform_center[0]) / (self.resolution[0] // 2)
        error_y = (ball_pos[1] - self.platform_center[1]) / (self.resolution[1] // 2)
        
        return error_x, error_y
    
    def compute_control(self, error_x: float, error_y: float) -> Tuple[float, float]:
        """
        计算控制角度
        
        Args:
            error_x: X轴误差
            error_y: Y轴误差
            
        Returns:
            (x轴角度, y轴角度) 单位：度
        """
        # 使用PID计算控制量
        angle_x = self.pid_x.update(error_x)
        angle_y = self.pid_y.update(error_y)
        
        return angle_x, angle_y
    
    def process_frame(self, frame: np.ndarray) -> dict:
        """
        处理单帧图像
        
        Args:
            frame: 原始图像帧
            
        Returns:
            处理结果字典
        """
        # 检测球体
        ball_pos, radius = self.detector.detect(frame)
        
        # 更新状态
        self.ball_position = ball_pos
        self.ball_radius = radius
        
        # 计算控制量
        if ball_pos is not None:
            error_x, error_y = self.compute_error(ball_pos)
            angle_x, angle_y = self.compute_control(error_x, error_y)
            
            self.current_angle_x = angle_x
            self.current_angle_y = angle_y
            
            # 记录位置历史
            self.position_history.append(ball_pos)
            if len(self.position_history) > self.max_history:
                self.position_history.pop(0)
        else:
            error_x, error_y = 0.0, 0.0
            angle_x, angle_y = 0.0, 0.0
        
        # 更新FPS
        self.update_fps()
        
        return {
            'ball_position': ball_pos,
            'ball_radius': radius,
            'error_x': error_x,
            'error_y': error_y,
            'angle_x': angle_x,
            'angle_y': angle_y,
            'detected': ball_pos is not None
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
        
        # 绘制目标位置
        cv2.circle(debug_frame, self.target_pixel, 10, (0, 255, 0), 2)
        cv2.putText(debug_frame, "Target", 
                   (self.target_pixel[0] + 15, self.target_pixel[1]), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        # 绘制平台中心
        cv2.circle(debug_frame, self.platform_center, 5, (255, 0, 0), -1)
        cv2.putText(debug_frame, "Center", 
                   (self.platform_center[0] + 10, self.platform_center[1]), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        
        # 绘制球体位置
        if result['ball_position'] is not None:
            cx, cy = result['ball_position']
            radius = result['ball_radius']
            
            # 绘制球体
            cv2.circle(debug_frame, (cx, cy), radius, (0, 0, 255), 2)
            cv2.circle(debug_frame, (cx, cy), 3, (0, 0, 255), -1)
            
            # 绘制位置标签
            cv2.putText(debug_frame, f"Ball: ({cx}, {cy})", 
                       (cx + radius + 10, cy), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            
            # 绘制误差线
            cv2.line(debug_frame, self.platform_center, (cx, cy), (255, 255, 0), 1)
        
        # 绘制位置历史轨迹
        if len(self.position_history) > 1:
            for i in range(1, len(self.position_history)):
                if self.position_history[i-1] is not None and self.position_history[i] is not None:
                    cv2.line(debug_frame, self.position_history[i-1], self.position_history[i], 
                            (0, 255, 255), 2)
        
        # 绘制状态信息
        status_text = f"Ball: {result['detected']} | Error: ({result['error_x']:.2f}, {result['error_y']:.2f})"
        cv2.putText(debug_frame, status_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        angle_text = f"Angle: X={result['angle_x']:.1f}° Y={result['angle_y']:.1f}°"
        cv2.putText(debug_frame, angle_text, (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        fps_text = f"FPS: {self.fps:.1f}"
        cv2.putText(debug_frame, fps_text, (10, 90), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        return debug_frame
    
    def run(self, show_debug: bool = True, servo_controller=None):
        """
        运行滚球平衡控制器主循环
        
        Args:
            show_debug: 是否显示调试窗口
            servo_controller: 舵机控制器（可选）
        """
        if not self.initialize():
            return
        
        print("滚球平衡控制器已启动，按 'q' 退出")
        print("按 'c' 重新校准中心位置")
        print("按 'r' 重置PID控制器")
        
        try:
            while self.is_running:
                # 读取帧
                ret, frame = self.cap.read()
                if not ret:
                    print("警告: 无法读取图像帧")
                    continue
                
                # 处理帧
                result = self.process_frame(frame)
                
                # 控制舵机
                if servo_controller and result['detected']:
                    # 假设有set_angles方法
                    if hasattr(servo_controller, 'set_angles'):
                        servo_controller.set_angles(result['angle_x'], result['angle_y'])
                    # 或者假设有两个独立的舵机
                    elif hasattr(servo_controller, 'set_angle'):
                        servo_controller.set_angle(0, result['angle_x'])
                        servo_controller.set_angle(1, result['angle_y'])
                
                # 显示调试窗口
                if show_debug:
                    debug_frame = self.draw_debug(frame, result)
                    cv2.imshow('Ball Balancer Debug', debug_frame)
                    
                    # 按键处理
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        break
                    elif key == ord('c'):
                        # 校准中心位置
                        if result['ball_position'] is not None:
                            self.platform_center = result['ball_position']
                            print(f"中心位置已校准到: {self.platform_center}")
                    elif key == ord('r'):
                        # 重置PID控制器
                        self.pid_x.reset()
                        self.pid_y.reset()
                        self.position_history.clear()
                        print("PID控制器已重置")
                        
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
        print("滚球平衡控制器已停止")
    
    def get_status(self) -> dict:
        """
        获取当前状态
        
        Returns:
            状态字典
        """
        return {
            'is_running': self.is_running,
            'ball_detected': self.ball_position is not None,
            'ball_position': self.ball_position,
            'ball_radius': self.ball_radius,
            'current_angle_x': self.current_angle_x,
            'current_angle_y': self.current_angle_y,
            'fps': self.fps,
            'platform_center': self.platform_center
        }
    
    def set_parameters(self, **kwargs):
        """
        动态调整参数
        
        Args:
            **kwargs: 参数字典
        """
        if 'target_position' in kwargs:
            self.target_position = kwargs['target_position']
            self.target_pixel = (
                int(self.resolution[0] * self.target_position[0]),
                int(self.resolution[1] * self.target_position[1])
            )
        if 'platform_center' in kwargs:
            self.platform_center = kwargs['platform_center']
        if 'kp_x' in kwargs:
            self.pid_x.kp = kwargs['kp_x']
        if 'ki_x' in kwargs:
            self.pid_x.ki = kwargs['ki_x']
        if 'kd_x' in kwargs:
            self.pid_x.kd = kwargs['kd_x']
        if 'kp_y' in kwargs:
            self.pid_y.kp = kwargs['kp_y']
        if 'ki_y' in kwargs:
            self.pid_y.ki = kwargs['ki_y']
        if 'kd_y' in kwargs:
            self.pid_y.kd = kwargs['kd_y']
        if 'ball_color_lower' in kwargs and 'ball_color_upper' in kwargs:
            self.detector.set_color_range(kwargs['ball_color_lower'], kwargs['ball_color_upper'])


class BallBalancerWithKalman(BallBalancer):
    """
    使用卡尔曼滤波的滚球平衡控制器
    提供更平滑的位置估计
    """
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        
        # 初始化卡尔曼滤波器
        self.kalman = cv2.KalmanFilter(4, 2)  # 4状态，2测量
        self.kalman.measurementMatrix = np.array([[1, 0, 0, 0],
                                                  [0, 1, 0, 0]], np.float32)
        self.kalman.transitionMatrix = np.array([[1, 0, 1, 0],
                                                 [0, 1, 0, 1],
                                                 [0, 0, 1, 0],
                                                 [0, 0, 0, 1]], np.float32)
        self.kalman.processNoiseCov = np.array([[1, 0, 0, 0],
                                               [0, 1, 0, 0],
                                               [0, 0, 1, 0],
                                               [0, 0, 0, 1]], np.float32) * 0.03
        self.kalman_initialized = False
    
    def process_frame(self, frame: np.ndarray) -> dict:
        """
        处理单帧图像（带卡尔曼滤波）
        
        Args:
            frame: 原始图像帧
            
        Returns:
            处理结果字典
        """
        # 检测球体
        ball_pos, radius = self.detector.detect(frame)
        
        # 卡尔曼滤波预测
        prediction = self.kalman.predict()
        
        if ball_pos is not None:
            # 更新卡尔曼滤波器
            measurement = np.array([[np.float32(ball_pos[0])], 
                                   [np.float32(ball_pos[1])]])
            self.kalman.correct(measurement)
            
            # 使用滤波后的位置
            estimated_pos = (
                int(self.kalman.statePost[0]),
                int(self.kalman.statePost[1])
            )
            
            self.ball_position = estimated_pos
            self.ball_radius = radius
            
            # 计算误差
            error_x, error_y = self.compute_error(estimated_pos)
            angle_x, angle_y = self.compute_control(error_x, error_y)
            
            self.current_angle_x = angle_x
            self.current_angle_y = angle_y
            
            # 记录位置历史
            self.position_history.append(estimated_pos)
            if len(self.position_history) > self.max_history:
                self.position_history.pop(0)
            
            detected = True
        else:
            # 使用预测位置
            predicted_pos = (int(prediction[0]), int(prediction[1]))
            self.ball_position = predicted_pos
            
            error_x, error_y = self.compute_error(predicted_pos)
            angle_x, angle_y = self.compute_control(error_x, error_y)
            
            self.current_angle_x = angle_x
            self.current_angle_y = angle_y
            
            detected = False
        
        # 更新FPS
        self.update_fps()
        
        return {
            'ball_position': self.ball_position,
            'ball_radius': radius,
            'error_x': error_x,
            'error_y': error_y,
            'angle_x': angle_x,
            'angle_y': angle_y,
            'detected': detected,
            'predicted': not detected
        }


# 测试代码
if __name__ == "__main__":
    # 创建滚球平衡控制器
    balancer = BallBalancer(
        camera_id=0,
        resolution=(640, 480),
        target_position=(0.5, 0.5),
        pid_x_params={'kp': 0.5, 'ki': 0.01, 'kd': 0.2},
        pid_y_params={'kp': 0.5, 'ki': 0.01, 'kd': 0.2},
        ball_color_lower=(0, 100, 100),  # 红色球
        ball_color_upper=(10, 255, 255)
    )
    
    # 运行
    balancer.run(show_debug=True)