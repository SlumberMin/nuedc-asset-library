"""
障碍物检测模块
结合超声波传感器和视觉信息进行障碍物检测与融合
"""

import cv2
import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass
import time

@dataclass
class Obstacle:
    """障碍物数据结构"""
    distance: float          # 距离（厘米）
    angle: float             # 角度（度，相对于摄像头中心）
    size: Tuple[int, int]    # 在图像中的宽高
    center: Tuple[int, int]  # 在图像中的中心坐标
    confidence: float        # 置信度（0-1）
    source: str              # 来源：'ultrasonic', 'visual', 'fused'

class UltrasonicSensor:
    """超声波传感器接口"""
    def __init__(self, port: str = None, baudrate: int = 9600):
        """
        初始化超声波传感器
        Args:
            port: 串口端口（如 '/dev/ttyUSB0' 或 'COM3'）
            baudrate: 波特率
        """
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        if port:
            self._connect()
    
    def _connect(self):
        """连接到传感器"""
        try:
            import serial
            self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=1)
            time.sleep(2)  # 等待连接稳定
        except ImportError:
            print("警告：未安装pyserial库，超声波传感器功能不可用")
        except Exception as e:
            print(f"连接超声波传感器失败: {e}")
    
    def read_distance(self) -> Optional[float]:
        """
        读取超声波传感器的距离值
        Returns:
            距离值（厘米），读取失败返回None
        """
        if not self.serial_conn:
            return None
        
        try:
            # 发送测距命令（根据具体传感器协议调整）
            self.serial_conn.write(b'R')
            time.sleep(0.1)
            response = self.serial_conn.readline().decode().strip()
            
            if response:
                # 解析距离值（假设返回格式为 "D:123.4"）
                if ':' in response:
                    distance = float(response.split(':')[1])
                    return distance
                else:
                    return float(response)
        except Exception as e:
            print(f"读取超声波传感器失败: {e}")
        
        return None
    
    def close(self):
        """关闭连接"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()


class ObstacleDetector:
    """障碍物检测器"""
    def __init__(self, 
                 camera_id: int = 0,
                 ultrasonic_port: str = None,
                 min_area: int = 500,
                 max_distance: float = 200.0):
        """
        初始化障碍物检测器
        Args:
            camera_id: 摄像头ID
            ultrasonic_port: 超声波传感器端口
            min_area: 最小障碍物面积（像素）
            max_distance: 最大检测距离（厘米）
        """
        self.camera = cv2.VideoCapture(camera_id)
        self.ultrasonic = UltrasonicSensor(ultrasonic_port)
        self.min_area = min_area
        self.max_distance = max_distance
        
        # 背景减除器
        self.bg_subtractor = cv2.createBackgroundSubtractorMOG2(
            history=500, varThreshold=50, detectShadows=True
        )
        
        # 内参矩阵（需要根据摄像头标定）
        self.camera_matrix = None
        self.dist_coeffs = None
        
    def load_calibration(self, calibration_file: str):
        """加载摄像头标定参数"""
        try:
            data = np.load(calibration_file)
            self.camera_matrix = data['camera_matrix']
            self.dist_coeffs = data['dist_coeffs']
        except Exception as e:
            print(f"加载标定参数失败: {e}")
    
    def preprocess_frame(self, frame: np.ndarray) -> np.ndarray:
        """预处理图像"""
        # 转换为灰度图
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # 高斯模糊去噪
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # 直方图均衡化
        equalized = cv2.equalizeHist(blurred)
        
        return equalized
    
    def detect_visual_obstacles(self, frame: np.ndarray) -> List[Obstacle]:
        """
        视觉检测障碍物
        Args:
            frame: 输入图像
        Returns:
            检测到的障碍物列表
        """
        obstacles = []
        height, width = frame.shape[:2]
        
        # 预处理
        processed = self.preprocess_frame(frame)
        
        # 背景减除
        fg_mask = self.bg_subtractor.apply(frame)
        
        # 形态学操作去噪
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_OPEN, kernel, iterations=1)
        
        # 阈值处理
        _, thresh = cv2.threshold(fg_mask, 127, 255, cv2.THRESH_BINARY)
        
        # 查找轮廓
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_area:
                continue
            
            # 获取边界框
            x, y, w, h = cv2.boundingRect(contour)
            
            # 计算中心点
            center_x = x + w // 2
            center_y = y + h // 2
            
            # 估计距离（基于物体大小，需要已知实际大小）
            # 这里使用简化的估计方法
            estimated_distance = self._estimate_distance_from_size(w, h)
            
            # 计算角度
            angle = self._calculate_angle(center_x, width)
            
            # 计算置信度（基于面积和形状）
            confidence = min(1.0, area / (self.min_area * 10))
            
            obstacle = Obstacle(
                distance=estimated_distance,
                angle=angle,
                size=(w, h),
                center=(center_x, center_y),
                confidence=confidence,
                source='visual'
            )
            obstacles.append(obstacle)
        
        return obstacles
    
    def _estimate_distance_from_size(self, width: int, height: int) -> float:
        """基于物体大小估计距离"""
        # 简化估计：假设物体平均大小为30cm，摄像头焦距为500像素
        if self.camera_matrix is not None:
            fx = self.camera_matrix[0, 0]
            # 假设物体实际宽度为30cm
            real_width = 30.0
            if width > 0:
                distance = (real_width * fx) / width
                return distance
        
        # 默认估计
        average_size = (width + height) / 2
        if average_size > 0:
            return 5000.0 / average_size  # 经验公式
        return self.max_distance
    
    def _calculate_angle(self, x: int, image_width: int) -> float:
        """计算物体相对于图像中心的角度"""
        center_x = image_width // 2
        offset = x - center_x
        
        # 假设视场角为60度
        fov = 60.0
        angle = (offset / center_x) * (fov / 2)
        
        return angle
    
    def fuse_detections(self, 
                       visual_obstacles: List[Obstacle],
                       ultrasonic_distance: Optional[float]) -> List[Obstacle]:
        """
        融合视觉和超声波检测结果
        Args:
            visual_obstacles: 视觉检测的障碍物
            ultrasonic_distance: 超声波测量的距离
        Returns:
            融合后的障碍物列表
        """
        fused_obstacles = []
        
        if ultrasonic_distance is not None and ultrasonic_distance < self.max_distance:
            # 如果超声波检测到障碍物
            ultrasonic_obstacle = Obstacle(
                distance=ultrasonic_distance,
                angle=0.0,  # 假设超声波传感器正前方
                size=(0, 0),
                center=(0, 0),
                confidence=0.9,  # 超声波传感器通常较可靠
                source='ultrasonic'
            )
            
            # 尝试与视觉检测结果融合
            matched = False
            for vis_obs in visual_obstacles:
                # 如果角度接近且距离接近，则融合
                if abs(vis_obs.angle) < 10.0 and abs(vis_obs.distance - ultrasonic_distance) < 50.0:
                    # 融合数据
                    fused_distance = (vis_obs.distance * vis_obs.confidence + 
                                     ultrasonic_distance * 0.9) / (vis_obs.confidence + 0.9)
                    fused_confidence = min(1.0, vis_obs.confidence + 0.1)
                    
                    fused_obstacle = Obstacle(
                        distance=fused_distance,
                        angle=vis_obs.angle,
                        size=vis_obs.size,
                        center=vis_obs.center,
                        confidence=fused_confidence,
                        source='fused'
                    )
                    fused_obstacles.append(fused_obstacle)
                    matched = True
            
            if not matched:
                fused_obstacles.append(ultrasonic_obstacle)
        
        # 添加未融合的视觉障碍物
        for vis_obs in visual_obstacles:
            if not any(abs(obs.angle - vis_obs.angle) < 10.0 for obs in fused_obstacles):
                fused_obstacles.append(vis_obs)
        
        return fused_obstacles
    
    def detect(self, frame: np.ndarray = None) -> List[Obstacle]:
        """
        执行障碍物检测
        Args:
            frame: 输入图像，如果为None则从摄像头读取
        Returns:
            检测到的障碍物列表
        """
        if frame is None:
            ret, frame = self.camera.read()
            if not ret:
                return []
        
        # 视觉检测
        visual_obstacles = self.detect_visual_obstacles(frame)
        
        # 超声波检测
        ultrasonic_distance = self.ultrasonic.read_distance()
        
        # 融合结果
        fused_obstacles = self.fuse_detections(visual_obstacles, ultrasonic_distance)
        
        return fused_obstacles
    
    def visualize(self, frame: np.ndarray, obstacles: List[Obstacle]) -> np.ndarray:
        """可视化检测结果"""
        vis_frame = frame.copy()
        
        for obs in obstacles:
            if obs.source == 'ultrasonic':
                color = (0, 0, 255)  # 红色
            elif obs.source == 'visual':
                color = (0, 255, 0)  # 绿色
            else:  # fused
                color = (0, 255, 255)  # 黄色
            
            if obs.size != (0, 0):
                # 绘制边界框
                x = obs.center[0] - obs.size[0] // 2
                y = obs.center[1] - obs.size[1] // 2
                cv2.rectangle(vis_frame, (x, y), (x + obs.size[0], y + obs.size[1]), color, 2)
            
            # 绘制信息
            text = f"D:{obs.distance:.1f}cm A:{obs.angle:.1f}°"
            cv2.putText(vis_frame, text, (obs.center[0] - 50, obs.center[1] - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            # 绘制中心点
            cv2.circle(vis_frame, obs.center, 5, color, -1)
        
        return vis_frame
    
    def release(self):
        """释放资源"""
        self.camera.release()
        self.ultrasonic.close()


# 使用示例
if __name__ == "__main__":
    detector = ObstacleDetector(camera_id=0, min_area=1000)
    
    try:
        while True:
            obstacles = detector.detect()
            
            ret, frame = detector.camera.read()
            if ret:
                vis_frame = detector.visualize(frame, obstacles)
                cv2.imshow("Obstacle Detection", vis_frame)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                
                # 打印检测结果
                for obs in obstacles:
                    print(f"障碍物: 距离={obs.distance:.1f}cm, 角度={obs.angle:.1f}°, 来源={obs.source}")
    
    finally:
        detector.release()
        cv2.destroyAllWindows()