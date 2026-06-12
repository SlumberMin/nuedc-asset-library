#!/usr/bin/env python3
"""
避障控制器 - 超声波+视觉融合
结合超声波传感器和摄像头实现避障功能
适用于Orange Pi 5 + OpenCV方案
"""
import cv2
import numpy as np
import time
import threading
from typing import Tuple, Optional, List, Dict
from enum import Enum
import sys
import os

# 添加项目根目录到路径
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from control.pid_controller import PIDController


class ObstacleType(Enum):
    """障碍物类型"""
    UNKNOWN = 0
    WALL = 1
    OBJECT = 2
    CLIFF = 3  # 悬崖/边缘


class AvoidanceStrategy(Enum):
    """避障策略"""
    STOP = 0           # 停止
    TURN_LEFT = 1      # 左转
    TURN_RIGHT = 2     # 右转
    BACKWARD = 3       # 后退
    FOLLOW_WALL = 4    # 沿墙行驶
    SLOW_DOWN = 5      # 减速


class UltrasonicSensor:
    """
    超声波传感器接口
    支持HC-SR04等常见超声波模块
    """
    
    def __init__(self, 
                 trigger_pin: int = 23,
                 echo_pin: int = 24,
                 temperature: float = 20.0):
        """
        初始化超声波传感器
        
        Args:
            trigger_pin: 触发引脚 (BCM编号)
            echo_pin: 回响引脚 (BCM编号)
            temperature: 环境温度 (℃)
        """
        self.trigger_pin = trigger_pin
        self.echo_pin = echo_pin
        self.temperature = temperature
        
        # 声速计算 (m/s)
        self.speed_of_sound = 331.3 + 0.606 * temperature
        
        # 测量参数
        self.max_distance = 4.0  # 最大测量距离 (m)
        self.min_distance = 0.02  # 最小测量距离 (m)
        self.timeout = 0.1  # 超时时间 (秒)
        
        # 状态
        self.is_initialized = False
        self.last_distance = None
        self.last_measurement_time = 0
        
        # 模拟模式（用于测试）
        self.simulation_mode = False
        self.simulated_distance = 1.0
        
    def initialize(self) -> bool:
        """
        初始化GPIO引脚
        
        Returns:
            初始化是否成功
        """
        try:
            # 尝试导入GPIO库
            try:
                import OPi.GPIO as GPIO
                self.GPIO = GPIO
                self.GPIO.setmode(GPIO.BCM)
                self.GPIO.setup(self.trigger_pin, GPIO.OUT)
                self.GPIO.setup(self.echo_pin, GPIO.IN)
                self.GPIO.output(self.trigger_pin, False)
                self.is_initialized = True
                print(f"超声波传感器初始化成功: Trigger={self.trigger_pin}, Echo={self.echo_pin}")
                return True
            except ImportError:
                print("警告: 未找到GPIO库，使用模拟模式")
                self.simulation_mode = True
                self.is_initialized = True
                return True
                
        except Exception as e:
            print(f"超声波传感器初始化失败: {e}")
            return False
    
    def measure_distance(self) -> Optional[float]:
        """
        测量距离
        
        Returns:
            距离 (米) 或 None（测量失败）
        """
        if not self.is_initialized:
            return None
        
        if self.simulation_mode:
            # 模拟模式：返回模拟距离（添加一些噪声）
            import random
            noise = random.uniform(-0.05, 0.05)
            return self.simulated_distance + noise
        
        try:
            # 发送触发信号
            self.GPIO.output(self.trigger_pin, True)
            time.sleep(0.00001)  # 10微秒
            self.GPIO.output(self.trigger_pin, False)
            
            # 等待回响信号
            start_time = time.time()
            timeout_start = start_time
            
            # 等待回响开始
            while self.GPIO.input(self.echo_pin) == 0:
                start_time = time.time()
                if start_time - timeout_start > self.timeout:
                    return None
            
            # 等待回响结束
            stop_time = start_time
            while self.GPIO.input(self.echo_pin) == 1:
                stop_time = time.time()
                if stop_time - start_time > self.timeout:
                    return None
            
            # 计算距离
            time_elapsed = stop_time - start_time
            distance = (time_elapsed * self.speed_of_sound) / 2
            
            # 检查距离范围
            if distance < self.min_distance or distance > self.max_distance:
                return None
            
            self.last_distance = distance
            self.last_measurement_time = time.time()
            
            return distance
            
        except Exception as e:
            print(f"距离测量失败: {e}")
            return None
    
    def set_simulation_distance(self, distance: float):
        """设置模拟距离（用于测试）"""
        self.simulated_distance = distance
    
    def cleanup(self):
        """清理GPIO资源"""
        if self.is_initialized and not self.simulation_mode:
            try:
                self.GPIO.cleanup()
            except:
                pass


class VisionObstacleDetector:
    """
    视觉障碍物检测器
    使用深度学习或传统图像处理检测障碍物
    """
    
    def __init__(self, 
                 method: str = 'contour',
                 min_object_area: int = 1000,
                 max_object_area: int = 100000):
        """
        初始化视觉障碍物检测器
        
        Args:
            method: 检测方法 ('contour', 'hog', 'yolo')
            min_object_area: 最小障碍物面积
            max_object_area: 最大障碍物面积
        """
        self.method = method
        self.min_object_area = min_object_area
        self.max_object_area = max_object_area
        
        # 检测参数
        self.gaussian_kernel = (5, 5)
        self.canny_low = 50
        self.canny_high = 150
        self.morph_kernel_size = 5
        
        # HOG行人检测器
        if method == 'hog':
            self.hog = cv2.HOGDescriptor()
            self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        
        # YOLO模型（需要模型文件）
        self.yolo_net = None
        if method == 'yolo':
            self._load_yolo_model()
    
    def _load_yolo_model(self):
        """加载YOLO模型"""
        try:
            # 这里需要实际的模型文件路径
            model_path = "yolov3.weights"
            config_path = "yolov3.cfg"
            classes_path = "coco.names"
            
            if os.path.exists(model_path) and os.path.exists(config_path):
                self.yolo_net = cv2.dnn.readNetFromDarknet(config_path, model_path)
                print("YOLO模型加载成功")
            else:
                print("警告: YOLO模型文件不存在，回退到轮廓检测")
                self.method = 'contour'
        except Exception as e:
            print(f"YOLO模型加载失败: {e}")
            self.method = 'contour'
    
    def detect_contour(self, frame: np.ndarray) -> List[Dict]:
        """
        使用轮廓检测障碍物
        
        Args:
            frame: 输入图像
            
        Returns:
            障碍物列表 [{'position': (x,y), 'size': (w,h), 'area': area, 'type': type}]
        """
        # 转换为灰度图
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # 高斯模糊
        blurred = cv2.GaussianBlur(gray, self.gaussian_kernel, 0)
        
        # 边缘检测
        edges = cv2.Canny(blurred, self.canny_low, self.canny_high)
        
        # 形态学操作
        kernel = np.ones((self.morph_kernel_size, self.morph_kernel_size), np.uint8)
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
        edges = cv2.morphologyEx(edges, cv2.MORPH_OPEN, kernel)
        
        # 查找轮廓
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        obstacles = []
        for contour in contours:
            area = cv2.contourArea(contour)
            
            # 过滤面积
            if area < self.min_object_area or area > self.max_object_area:
                continue
            
            # 计算边界矩形
            x, y, w, h = cv2.boundingRect(contour)
            
            # 计算中心点
            cx = x + w // 2
            cy = y + h // 2
            
            # 判断障碍物类型
            aspect_ratio = w / h if h > 0 else 1
            if aspect_ratio > 3:
                obstacle_type = ObstacleType.WALL
            else:
                obstacle_type = ObstacleType.OBJECT
            
            obstacles.append({
                'position': (cx, cy),
                'size': (w, h),
                'area': area,
                'type': obstacle_type,
                'bbox': (x, y, w, h)
            })
        
        return obstacles
    
    def detect_hog(self, frame: np.ndarray) -> List[Dict]:
        """
        使用HOG检测行人
        
        Args:
            frame: 输入图像
            
        Returns:
            行人列表 [{'position': (x,y), 'size': (w,h), 'confidence': conf}]
        """
        # HOG行人检测
        boxes, weights = self.hog.detectMultiScale(
            frame, 
            winStride=(8, 8),
            padding=(4, 4), 
            scale=1.05
        )
        
        pedestrians = []
        for (x, y, w, h), weight in zip(boxes, weights):
            if weight > 0.5:  # 置信度阈值
                pedestrians.append({
                    'position': (x + w//2, y + h//2),
                    'size': (w, h),
                    'confidence': weight,
                    'type': ObstacleType.OBJECT,
                    'bbox': (x, y, w, h)
                })
        
        return pedestrians
    
    def detect_yolo(self, frame: np.ndarray) -> List[Dict]:
        """
        使用YOLO检测物体
        
        Args:
            frame: 输入图像
            
        Returns:
            物体列表 [{'position': (x,y), 'size': (w,h), 'class': cls, 'confidence': conf}]
        """
        if self.yolo_net is None:
            return []
        
        height, width = frame.shape[:2]
        
        # 创建blob
        blob = cv2.dnn.blobFromImage(frame, 1/255.0, (416, 416), swapRB=True, crop=False)
        self.yolo_net.setInput(blob)
        
        # 前向传播
        outputs = self.yolo_net.forward(self.yolo_net.getUnconnectedOutLayersNames())
        
        objects = []
        for output in outputs:
            for detection in output:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                
                if confidence > 0.5:
                    # 计算边界框
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)
                    
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)
                    
                    objects.append({
                        'position': (center_x, center_y),
                        'size': (w, h),
                        'class': class_id,
                        'confidence': confidence,
                        'type': ObstacleType.OBJECT,
                        'bbox': (x, y, w, h)
                    })
        
        return objects
    
    def detect(self, frame: np.ndarray) -> List[Dict]:
        """
        检测障碍物
        
        Args:
            frame: 输入图像
            
        Returns:
            障碍物列表
        """
        if self.method == 'contour':
            return self.detect_contour(frame)
        elif self.method == 'hog':
            return self.detect_hog(frame)
        elif self.method == 'yolo':
            return self.detect_yolo(frame)
        else:
            return self.detect_contour(frame)


class ObstacleAvoider:
    """
    避障控制器
    融合超声波和视觉传感器实现避障
    """
    
    def __init__(self, 
                 camera_id: int = 0,
                 resolution: Tuple[int, int] = (640, 480),
                 ultrasonic_pins: Dict[str, int] = None,
                 detection_method: str = 'contour',
                 safe_distance: float = 0.5,
                 warning_distance: float = 1.0,
                 base_speed: float = 50.0,
                 max_speed: float = 100.0,
                 pid_params: dict = None):
        """
        初始化避障控制器
        
        Args:
            camera_id: 摄像头ID
            resolution: 分辨率 (宽, 高)
            ultrasonic_pins: 超声波引脚配置 {'trigger': 23, 'echo': 24}
            detection_method: 视觉检测方法
            safe_distance: 安全距离 (米)
            warning_distance: 警告距离 (米)
            base_speed: 基础速度
            max_speed: 最大速度
            pid_params: PID参数
        """
        self.camera_id = camera_id
        self.resolution = resolution
        self.safe_distance = safe_distance
        self.warning_distance = warning_distance
        self.base_speed = base_speed
        self.max_speed = max_speed
        
        # 默认超声波引脚
        if ultrasonic_pins is None:
            ultrasonic_pins = {'trigger': 23, 'echo': 24}
        
        # 初始化传感器
        self.ultrasonic = UltrasonicSensor(
            trigger_pin=ultrasonic_pins['trigger'],
            echo_pin=ultrasonic_pins['echo']
        )
        
        self.vision_detector = VisionObstacleDetector(method=detection_method)
        
        # 默认PID参数
        if pid_params is None:
            pid_params = {'kp': 1.0, 'ki': 0.01, 'kd': 0.3}
        
        # 初始化PID控制器（用于转向控制）
        self.pid = PIDController(
            kp=pid_params['kp'],
            ki=pid_params['ki'],
            kd=pid_params['kd'],
            output_limits=(-100, 100)
        )
        
        # 状态变量
        self.cap = None
        self.is_running = False
        self.current_strategy = AvoidanceStrategy.STOP
        self.ultrasonic_distance = None
        self.vision_obstacles = []
        self.fused_obstacles = []
        
        # 性能统计
        self.frame_count = 0
        self.fps = 0.0
        self.last_fps_time = time.time()
        
        # 线程控制
        self.ultrasonic_thread = None
        self.ultrasonic_running = False
        
    def initialize(self) -> bool:
        """
        初始化所有传感器
        
        Returns:
            初始化是否成功
        """
        try:
            # 初始化摄像头
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
            
            # 初始化超声波传感器
            if not self.ultrasonic.initialize():
                print("警告: 超声波传感器初始化失败")
            
            # 启动超声波测量线程
            self.start_ultrasonic_thread()
            
            print(f"避障控制器初始化成功: {self.resolution}")
            self.is_running = True
            return True
            
        except Exception as e:
            print(f"初始化失败: {e}")
            return False
    
    def start_ultrasonic_thread(self):
        """启动超声波测量线程"""
        self.ultrasonic_running = True
        self.ultrasonic_thread = threading.Thread(target=self._ultrasonic_measurement_loop)
        self.ultrasonic_thread.daemon = True
        self.ultrasonic_thread.start()
    
    def _ultrasonic_measurement_loop(self):
        """超声波测量循环"""
        while self.ultrasonic_running:
            distance = self.ultrasonic.measure_distance()
            if distance is not None:
                self.ultrasonic_distance = distance
            time.sleep(0.05)  # 20Hz测量频率
    
    def fuse_sensor_data(self, ultrasonic_dist: Optional[float], 
                         vision_obs: List[Dict]) -> List[Dict]:
        """
        融合传感器数据
        
        Args:
            ultrasonic_dist: 超声波距离
            vision_obs: 视觉障碍物列表
            
        Returns:
            融合后的障碍物列表
        """
        fused = []
        
        # 添加超声波检测到的障碍物
        if ultrasonic_dist is not None and ultrasonic_dist < self.warning_distance:
            # 超声波检测到的是正前方的障碍物
            fused.append({
                'position': (self.resolution[0] // 2, self.resolution[1] // 2),
                'distance': ultrasonic_dist,
                'type': ObstacleType.UNKNOWN,
                'source': 'ultrasonic',
                'priority': 1  # 超声波优先级高
            })
        
        # 添加视觉检测到的障碍物
        for obs in vision_obs:
            cx, cy = obs['position']
            
            # 估算距离（基于物体大小和位置）
            # 这里使用简化的距离估算
            width, height = obs['size']
            estimated_distance = self._estimate_distance(width, height, cy)
            
            fused.append({
                'position': obs['position'],
                'distance': estimated_distance,
                'type': obs['type'],
                'source': 'vision',
                'priority': 2,
                'size': obs['size'],
                'area': obs['area']
            })
        
        # 按距离排序
        fused.sort(key=lambda x: x['distance'])
        
        return fused
    
    def _estimate_distance(self, width: int, height: int, y_position: int) -> float:
        """
        估算障碍物距离
        
        Args:
            width: 物体宽度
            height: 物体高度
            y_position: 物体Y位置
            
        Returns:
            估算距离 (米)
        """
        # 简化的距离估算方法
        # 基于物体在图像中的大小和位置
        
        # 假设物体在地面上，根据Y位置估算
        # 图像底部距离近，顶部距离远
        height_ratio = y_position / self.resolution[1]
        
        # 基于物体大小的距离估算（假设已知物体实际大小）
        # 这里使用简化的线性关系
        size_factor = max(width, height) / 100.0
        distance = 2.0 / (size_factor + 0.1)
        
        # 调整基于位置的因素
        distance *= (0.5 + height_ratio)
        
        return max(0.1, min(5.0, distance))
    
    def determine_strategy(self, fused_obstacles: List[Dict]) -> AvoidanceStrategy:
        """
        确定避障策略
        
        Args:
            fused_obstacles: 融合后的障碍物列表
            
        Returns:
            避障策略
        """
        if not fused_obstacles:
            return AvoidanceStrategy.SLOW_DOWN  # 没有障碍物，可以慢速前进
        
        # 获取最近障碍物
        nearest = fused_obstacles[0]
        distance = nearest['distance']
        position = nearest['position']
        
        # 根据距离确定策略
        if distance < self.safe_distance:
            # 紧急情况
            if position[0] < self.resolution[0] // 3:
                return AvoidanceStrategy.TURN_RIGHT
            elif position[0] > 2 * self.resolution[0] // 3:
                return AvoidanceStrategy.TURN_LEFT
            else:
                return AvoidanceStrategy.STOP
        
        elif distance < self.warning_distance:
            # 警告情况
            if position[0] < self.resolution[0] // 3:
                return AvoidanceStrategy.TURN_RIGHT
            elif position[0] > 2 * self.resolution[0] // 3:
                return AvoidanceStrategy.TURN_LEFT
            else:
                # 正前方有障碍物，需要转向
                # 检查左右两侧
                left_obstacles = [obs for obs in fused_obstacles 
                                 if obs['position'][0] < self.resolution[0] // 2]
                right_obstacles = [obs for obs in fused_obstacles 
                                  if obs['position'][0] > self.resolution[0] // 2]
                
                if len(left_obstacles) < len(right_obstacles):
                    return AvoidanceStrategy.TURN_LEFT
                else:
                    return AvoidanceStrategy.TURN_RIGHT
        else:
            # 安全距离外
            return AvoidanceStrategy.SLOW_DOWN
    
    def compute_motor_speeds(self, strategy: AvoidanceStrategy, 
                            target_x: int = None) -> Tuple[float, float]:
        """
        计算电机速度
        
        Args:
            strategy: 避障策略
            target_x: 目标X位置（用于PID控制）
            
        Returns:
            (左轮速度, 右轮速度)
        """
        if strategy == AvoidanceStrategy.STOP:
            return 0.0, 0.0
        
        elif strategy == AvoidanceStrategy.TURN_LEFT:
            return -self.base_speed * 0.5, self.base_speed * 0.5
        
        elif strategy == AvoidanceStrategy.TURN_RIGHT:
            return self.base_speed * 0.5, -self.base_speed * 0.5
        
        elif strategy == AvoidanceStrategy.BACKWARD:
            return -self.base_speed * 0.3, -self.base_speed * 0.3
        
        elif strategy == AvoidanceStrategy.SLOW_DOWN:
            # 使用PID控制转向
            if target_x is not None:
                center_x = self.resolution[0] // 2
                error = (target_x - center_x) / center_x
                steering = self.pid.update(error)
                
                left_speed = self.base_speed * 0.5 + steering
                right_speed = self.base_speed * 0.5 - steering
                
                return max(0, min(self.max_speed, left_speed)), \
                       max(0, min(self.max_speed, right_speed))
            else:
                return self.base_speed * 0.5, self.base_speed * 0.5
        
        elif strategy == AvoidanceStrategy.FOLLOW_WALL:
            # 沿墙行驶（简化实现）
            return self.base_speed * 0.3, self.base_speed * 0.5
        
        else:
            return 0.0, 0.0
    
    def process_frame(self, frame: np.ndarray) -> dict:
        """
        处理单帧图像
        
        Args:
            frame: 原始图像帧
            
        Returns:
            处理结果字典
        """
        # 视觉检测
        vision_obstacles = self.vision_detector.detect(frame)
        
        # 传感器融合
        fused_obstacles = self.fuse_sensor_data(self.ultrasonic_distance, vision_obstacles)
        
        # 确定策略
        strategy = self.determine_strategy(fused_obstacles)
        
        # 计算电机速度
        target_x = None
        if fused_obstacles:
            target_x = fused_obstacles[0]['position'][0]
        
        left_speed, right_speed = self.compute_motor_speeds(strategy, target_x)
        
        # 更新状态
        self.current_strategy = strategy
        self.vision_obstacles = vision_obstacles
        self.fused_obstacles = fused_obstacles
        
        # 更新FPS
        self.update_fps()
        
        return {
            'ultrasonic_distance': self.ultrasonic_distance,
            'vision_obstacles': vision_obstacles,
            'fused_obstacles': fused_obstacles,
            'strategy': strategy,
            'left_speed': left_speed,
            'right_speed': right_speed,
            'nearest_distance': fused_obstacles[0]['distance'] if fused_obstacles else None
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
        
        # 绘制视觉障碍物
        for obs in result['vision_obstacles']:
            x, y, w, h = obs['bbox']
            cv2.rectangle(debug_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(debug_frame, f"{obs['type'].name}", (x, y - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        # 绘制融合障碍物
        for obs in result['fused_obstacles']:
            cx, cy = obs['position']
            distance = obs['distance']
            
            # 根据距离选择颜色
            if distance < self.safe_distance:
                color = (0, 0, 255)  # 红色
            elif distance < self.warning_distance:
                color = (0, 165, 255)  # 橙色
            else:
                color = (0, 255, 0)  # 绿色
            
            # 绘制障碍物标记
            cv2.circle(debug_frame, (cx, cy), 10, color, 2)
            cv2.putText(debug_frame, f"{distance:.2f}m", (cx + 15, cy), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        
        # 绘制安全区域
        height, width = frame.shape[:2]
        safe_zone_width = int(width * 0.3)
        cv2.rectangle(debug_frame, 
                     (width//2 - safe_zone_width//2, 0),
                     (width//2 + safe_zone_width//2, height),
                     (0, 255, 0), 1)
        
        # 绘制状态信息
        strategy_text = f"Strategy: {result['strategy'].name}"
        cv2.putText(debug_frame, strategy_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        distance_text = f"Ultrasonic: {result['ultrasonic_distance']:.2f}m" \
                       if result['ultrasonic_distance'] else "Ultrasonic: N/A"
        cv2.putText(debug_frame, distance_text, (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        speed_text = f"Speed: L={result['left_speed']:.1f} R={result['right_speed']:.1f}"
        cv2.putText(debug_frame, speed_text, (10, 90), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        fps_text = f"FPS: {self.fps:.1f}"
        cv2.putText(debug_frame, fps_text, (10, 120), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # 绘制距离警告
        if result['nearest_distance'] and result['nearest_distance'] < self.safe_distance:
            warning_text = "WARNING: OBSTACLE TOO CLOSE!"
            cv2.putText(debug_frame, warning_text, (width//2 - 200, height - 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        
        return debug_frame
    
    def run(self, show_debug: bool = True, motor_controller=None):
        """
        运行避障控制器主循环
        
        Args:
            show_debug: 是否显示调试窗口
            motor_controller: 电机控制器（可选）
        """
        if not self.initialize():
            return
        
        print("避障控制器已启动，按 'q' 退出")
        print("按 's' 切换到安全模式（更保守）")
        print("按 'f' 切换到快速模式（更激进）")
        
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
                if motor_controller:
                    motor_controller.set_motors(result['left_speed'], result['right_speed'])
                
                # 显示调试窗口
                if show_debug:
                    debug_frame = self.draw_debug(frame, result)
                    cv2.imshow('Obstacle Avoider Debug', debug_frame)
                    
                    # 按键处理
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        break
                    elif key == ord('s'):
                        # 安全模式
                        self.safe_distance = 1.0
                        self.warning_distance = 2.0
                        print("切换到安全模式")
                    elif key == ord('f'):
                        # 快速模式
                        self.safe_distance = 0.3
                        self.warning_distance = 0.8
                        print("切换到快速模式")
                        
        except KeyboardInterrupt:
            print("\n用户中断")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """清理资源"""
        self.is_running = False
        self.ultrasonic_running = False
        
        if self.ultrasonic_thread:
            self.ultrasonic_thread.join(timeout=1.0)
        
        if self.cap:
            self.cap.release()
        
        self.ultrasonic.cleanup()
        cv2.destroyAllWindows()
        print("避障控制器已停止")
    
    def get_status(self) -> dict:
        """
        获取当前状态
        
        Returns:
            状态字典
        """
        return {
            'is_running': self.is_running,
            'current_strategy': self.current_strategy.name,
            'ultrasonic_distance': self.ultrasonic_distance,
            'num_vision_obstacles': len(self.vision_obstacles),
            'num_fused_obstacles': len(self.fused_obstacles),
            'nearest_distance': self.fused_obstacles[0]['distance'] if self.fused_obstacles else None,
            'fps': self.fps,
            'safe_distance': self.safe_distance,
            'warning_distance': self.warning_distance
        }
    
    def set_parameters(self, **kwargs):
        """
        动态调整参数
        
        Args:
            **kwargs: 参数字典
        """
        if 'safe_distance' in kwargs:
            self.safe_distance = kwargs['safe_distance']
        if 'warning_distance' in kwargs:
            self.warning_distance = kwargs['warning_distance']
        if 'base_speed' in kwargs:
            self.base_speed = kwargs['base_speed']
        if 'max_speed' in kwargs:
            self.max_speed = kwargs['max_speed']
        if 'detection_method' in kwargs:
            self.vision_detector = VisionObstacleDetector(method=kwargs['detection_method'])
        if 'kp' in kwargs:
            self.pid.kp = kwargs['kp']
        if 'ki' in kwargs:
            self.pid.ki = kwargs['ki']
        if 'kd' in kwargs:
            self.pid.kd = kwargs['kd']


class MultiSensorObstacleAvoider(ObstacleAvoider):
    """
    多传感器避障控制器
    支持多个超声波传感器
    """
    
    def __init__(self, 
                 ultrasonic_configs: List[Dict] = None,
                 **kwargs):
        """
        初始化多传感器避障控制器
        
        Args:
            ultrasonic_configs: 超声波传感器配置列表
            **kwargs: 其他参数
        """
        super().__init__(**kwargs)
        
        # 默认配置：左、中、右三个传感器
        if ultrasonic_configs is None:
            ultrasonic_configs = [
                {'trigger': 23, 'echo': 24, 'name': 'left'},
                {'trigger': 25, 'echo': 8, 'name': 'center'},
                {'trigger': 7, 'echo': 1, 'name': 'right'}
            ]
        
        # 初始化多个超声波传感器
        self.ultrasonic_sensors = {}
        for config in ultrasonic_configs:
            name = config.get('name', f"sensor_{len(self.ultrasonic_sensors)}")
            sensor = UltrasonicSensor(
                trigger_pin=config['trigger'],
                echo_pin=config['echo']
            )
            self.ultrasonic_sensors[name] = sensor
        
        # 传感器数据
        self.sensor_distances = {}
    
    def initialize(self) -> bool:
        """
        初始化所有传感器
        
        Returns:
            初始化是否成功
        """
        # 初始化摄像头
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
        
        # 初始化所有超声波传感器
        for name, sensor in self.ultrasonic_sensors.items():
            if sensor.initialize():
                print(f"超声波传感器 '{name}' 初始化成功")
            else:
                print(f"警告: 超声波传感器 '{name}' 初始化失败")
        
        # 启动多传感器测量线程
        self.start_multi_sensor_thread()
        
        print(f"多传感器避障控制器初始化成功: {self.resolution}")
        self.is_running = True
        return True
    
    def start_multi_sensor_thread(self):
        """启动多传感器测量线程"""
        self.ultrasonic_running = True
        self.ultrasonic_thread = threading.Thread(target=self._multi_sensor_measurement_loop)
        self.ultrasonic_thread.daemon = True
        self.ultrasonic_thread.start()
    
    def _multi_sensor_measurement_loop(self):
        """多传感器测量循环"""
        while self.ultrasonic_running:
            for name, sensor in self.ultrasonic_sensors.items():
                distance = sensor.measure_distance()
                if distance is not None:
                    self.sensor_distances[name] = distance
                time.sleep(0.02)  # 每个传感器间隔20ms
    
    def fuse_sensor_data(self, ultrasonic_dist: Optional[float], 
                         vision_obs: List[Dict]) -> List[Dict]:
        """
        融合多传感器数据
        
        Args:
            ultrasonic_dist: 中间超声波距离（保留兼容性）
            vision_obs: 视觉障碍物列表
            
        Returns:
            融合后的障碍物列表
        """
        fused = []
        
        # 添加所有超声波传感器数据
        for name, distance in self.sensor_distances.items():
            if distance < self.warning_distance:
                # 根据传感器名称确定位置
                if name == 'left':
                    x_pos = self.resolution[0] // 4
                elif name == 'right':
                    x_pos = 3 * self.resolution[0] // 4
                else:  # center
                    x_pos = self.resolution[0] // 2
                
                fused.append({
                    'position': (x_pos, self.resolution[1] // 2),
                    'distance': distance,
                    'type': ObstacleType.UNKNOWN,
                    'source': f'ultrasonic_{name}',
                    'priority': 1
                })
        
        # 添加视觉检测到的障碍物
        for obs in vision_obs:
            cx, cy = obs['position']
            width, height = obs['size']
            estimated_distance = self._estimate_distance(width, height, cy)
            
            fused.append({
                'position': obs['position'],
                'distance': estimated_distance,
                'type': obs['type'],
                'source': 'vision',
                'priority': 2,
                'size': obs['size'],
                'area': obs['area']
            })
        
        # 按距离排序
        fused.sort(key=lambda x: x['distance'])
        
        return fused
    
    def get_status(self) -> dict:
        """
        获取当前状态
        
        Returns:
            状态字典
        """
        status = super().get_status()
        status['sensor_distances'] = self.sensor_distances.copy()
        return status


# 测试代码
if __name__ == "__main__":
    # 创建避障控制器
    avoider = ObstacleAvoider(
        camera_id=0,
        resolution=(640, 480),
        ultrasonic_pins={'trigger': 23, 'echo': 24},
        detection_method='contour',
        safe_distance=0.5,
        warning_distance=1.0,
        base_speed=50.0
    )
    
    # 运行
    avoider.run(show_debug=True)