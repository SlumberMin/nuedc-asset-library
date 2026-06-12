"""
单目深度估计模块
基于目标大小、消失线、纹理梯度等单目线索进行深度估计
适用于电赛中单摄像头场景
"""

import cv2
import numpy as np
from typing import List, Tuple, Optional, Dict
from dataclasses import dataclass


@dataclass
class DepthEstimate:
    """深度估计结果"""
    distance: float        # 估计距离（厘米）
    confidence: float      # 置信度（0-1）
    method: str            # 使用的估计方法
    bbox: Tuple[int, int, int, int]  # 目标边界框


class SizeBasedEstimator:
    """
    基于目标大小的深度估计
    利用相机成像模型：Z = f * H_real / H_pixel
    """
    
    def __init__(self, focal_length: float = 500.0):
        """
        Args:
            focal_length: 焦距（像素单位），需要标定获得
        """
        self.focal_length = focal_length
        self.known_objects = {}
    
    def calibrate(self, image: np.ndarray, 
                  bbox: Tuple[int, int, int, int],
                  real_distance: float,
                  real_height: float = None,
                  real_width: float = None):
        """
        使用已知距离和物体大小标定焦距
        Args:
            image: 标定图像
            bbox: 目标边界框
            real_distance: 实际距离（厘米）
            real_height: 实际高度（厘米），可选
            real_width: 实际宽度（厘米），可选
        """
        x, y, w, h = bbox
        
        if real_height and h > 0:
            self.focal_length = h * real_distance / real_height
        elif real_width and w > 0:
            self.focal_length = w * real_distance / real_width
        
        print(f"标定焦距: {self.focal_length:.1f} 像素")
    
    def register_object(self, name: str, 
                        real_height: float = None,
                        real_width: float = None,
                        real_area: float = None):
        """
        注册已知物体的尺寸
        Args:
            name: 物体名称
            real_height: 实际高度（厘米）
            real_width: 实际宽度（厘米）
            real_area: 实际面积（平方厘米）
        """
        self.known_objects[name] = {
            'height': real_height,
            'width': real_width,
            'area': real_area
        }
    
    def estimate_by_size(self, 
                         pixel_size: float,
                         real_size: float) -> float:
        """
        基于大小估计距离
        Args:
            pixel_size: 图像中的尺寸（像素）
            real_size: 实际尺寸（厘米）
        Returns:
            估计距离（厘米）
        """
        if pixel_size <= 0:
            return float('inf')
        
        distance = self.focal_length * real_size / pixel_size
        return distance
    
    def estimate_from_bbox(self, 
                           bbox: Tuple[int, int, int, int],
                           object_name: str = None,
                           real_height: float = None,
                           real_width: float = None) -> DepthEstimate:
        """
        从边界框估计深度
        Args:
            bbox: 边界框 (x, y, w, h)
            object_name: 已注册物体名称
            real_height: 实际高度（厘米）
            real_width: 实际宽度（厘米）
        Returns:
            深度估计结果
        """
        x, y, w, h = bbox
        
        distances = []
        
        # 使用已注册物体信息
        if object_name and object_name in self.known_objects:
            obj = self.known_objects[object_name]
            if obj['height'] and h > 0:
                d = self.estimate_by_size(h, obj['height'])
                distances.append(('height', d))
            if obj['width'] and w > 0:
                d = self.estimate_by_size(w, obj['width'])
                distances.append(('width', d))
        
        # 使用直接提供的尺寸
        if real_height and h > 0:
            d = self.estimate_by_size(h, real_height)
            distances.append(('height', d))
        if real_width and w > 0:
            d = self.estimate_by_size(w, real_width)
            distances.append(('width', d))
        
        if not distances:
            return DepthEstimate(
                distance=float('inf'),
                confidence=0.0,
                method='size_based',
                bbox=bbox
            )
        
        # 取平均值
        avg_distance = np.mean([d[1] for d in distances])
        
        # 计算置信度（多个估计越一致，置信度越高）
        if len(distances) > 1:
            values = [d[1] for d in distances]
            std = np.std(values)
            confidence = max(0.0, 1.0 - std / avg_distance)
        else:
            confidence = 0.7
        
        return DepthEstimate(
            distance=avg_distance,
            confidence=confidence,
            method='size_based',
            bbox=bbox
        )


class GroundPlaneEstimator:
    """
    基于地平面假设的深度估计
    假设目标位于地面上，利用相机高度和俯仰角估计距离
    """
    
    def __init__(self, 
                 camera_height: float = 30.0,
                 camera_pitch: float = 0.0,
                 focal_length: float = 500.0,
                 image_height: int = 480):
        """
        Args:
            camera_height: 相机离地高度（厘米）
            camera_pitch: 相机俯仰角（度，向下为正）
            focal_length: 焦距（像素）
            image_height: 图像高度（像素）
        """
        self.camera_height = camera_height
        self.camera_pitch = np.radians(camera_pitch)
        self.focal_length = focal_length
        self.image_height = image_height
        self.principal_y = image_height // 2  # 主点y坐标
    
    def estimate(self, 
                 bottom_y: int,
                 object_height: float = 0.0) -> float:
        """
        基于目标底部y坐标估计距离
        Args:
            bottom_y: 目标底部在图像中的y坐标
            object_height: 物体实际高度（厘米），如果>0则补偿
        Returns:
            估计距离（厘米）
        """
        # 相对主点的偏移
        dy = bottom_y - self.principal_y
        
        if dy <= 0:
            return float('inf')
        
        # 考虑俯仰角
        angle = np.arctan2(dy, self.focal_length) + self.camera_pitch
        
        if angle <= 0:
            return float('inf')
        
        # 地面距离
        distance = self.camera_height / np.tan(angle)
        
        return distance
    
    def estimate_from_bbox(self, bbox: Tuple[int, int, int, int],
                          object_height: float = 0.0) -> DepthEstimate:
        """从边界框估计深度"""
        x, y, w, h = bbox
        bottom_y = y + h
        
        distance = self.estimate(bottom_y, object_height)
        
        # 置信度：越靠近图像底部越可信
        confidence = min(1.0, bottom_y / self.image_height)
        
        return DepthEstimate(
            distance=distance,
            confidence=confidence,
            method='ground_plane',
            bbox=bbox
        )


class TextureGradientEstimator:
    """
    基于纹理梯度的深度估计
    远处物体纹理更密集
    """
    
    def __init__(self, focal_length: float = 500.0):
        self.focal_length = focal_length
        self.reference_texture_density = None
        self.reference_distance = None
    
    def calibrate(self, image: np.ndarray, 
                  roi: Tuple[int, int, int, int],
                  real_distance: float):
        """用已知距离标定纹理密度参考"""
        x, y, w, h = roi
        patch = image[y:y+h, x:x+w]
        
        self.reference_texture_density = self._compute_texture_density(patch)
        self.reference_distance = real_distance
    
    def _compute_texture_density(self, image: np.ndarray) -> float:
        """计算纹理密度"""
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image
        
        # 使用Sobel梯度
        grad_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
        grad_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
        
        gradient_magnitude = np.sqrt(grad_x**2 + grad_y**2)
        density = np.mean(gradient_magnitude)
        
        return density
    
    def estimate(self, image: np.ndarray,
                 roi: Tuple[int, int, int, int]) -> DepthEstimate:
        """估计深度"""
        if self.reference_texture_density is None:
            return DepthEstimate(0, 0, 'texture_gradient', roi)
        
        x, y, w, h = roi
        patch = image[y:y+h, x:x+w]
        
        current_density = self._compute_texture_density(patch)
        
        if current_density <= 0:
            return DepthEstimate(float('inf'), 0.0, 'texture_gradient', roi)
        
        # 纹理密度与距离成反比
        distance = self.reference_distance * self.reference_texture_density / current_density
        
        return DepthEstimate(
            distance=distance,
            confidence=0.5,
            method='texture_gradient',
            bbox=roi
        )


class DepthEstimator:
    """
    综合深度估计器
    融合多种单目深度估计方法
    """
    
    def __init__(self, 
                 focal_length: float = 500.0,
                 camera_height: float = 30.0,
                 camera_pitch: float = 0.0,
                 image_height: int = 480):
        """
        Args:
            focal_length: 焦距（像素）
            camera_height: 相机高度（厘米）
            camera_pitch: 相机俯仰角（度）
            image_height: 图像高度
        """
        self.size_estimator = SizeBasedEstimator(focal_length)
        self.ground_estimator = GroundPlaneEstimator(
            camera_height, camera_pitch, focal_length, image_height
        )
        self.texture_estimator = TextureGradientEstimator(focal_length)
        
        self.focal_length = focal_length
    
    def calibrate_with_checkerboard(self, images: List[np.ndarray],
                                     board_size: Tuple[int, int] = (9, 6),
                                     square_size: float = 2.5):
        """
        使用棋盘格标定相机内参
        Args:
            images: 标定图像列表
            board_size: 棋盘格内角点数
            square_size: 方格大小（厘米）
        """
        obj_points = []
        img_points = []
        
        # 生成棋盘格世界坐标
        objp = np.zeros((board_size[0] * board_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:board_size[0], 0:board_size[1]].T.reshape(-1, 2)
        objp *= square_size
        
        for img in images:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if len(img.shape) == 3 else img
            
            ret, corners = cv2.findChessboardCorners(gray, board_size, None)
            
            if ret:
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                
                obj_points.append(objp)
                img_points.append(corners)
        
        if obj_points:
            ret, camera_matrix, dist_coeffs, _, _ = cv2.calibrateCamera(
                obj_points, img_points, gray.shape[::-1], None, None
            )
            
            if ret:
                self.focal_length = (camera_matrix[0, 0] + camera_matrix[1, 1]) / 2
                self.size_estimator.focal_length = self.focal_length
                self.ground_estimator.focal_length = self.focal_length
                self.texture_estimator.focal_length = self.focal_length
                
                print(f"标定完成，焦距: {self.focal_length:.1f} 像素")
                return camera_matrix, dist_coeffs
        
        return None, None
    
    def register_object(self, name: str, **kwargs):
        """注册已知物体"""
        self.size_estimator.register_object(name, **kwargs)
    
    def estimate_depth(self, 
                       image: np.ndarray,
                       bbox: Tuple[int, int, int, int],
                       object_name: str = None,
                       real_height: float = None,
                       real_width: float = None) -> DepthEstimate:
        """
        估计目标深度
        Args:
            image: 输入图像
            bbox: 目标边界框
            object_name: 已注册物体名
            real_height: 实际高度
            real_width: 实际宽度
        Returns:
            深度估计结果
        """
        estimates = []
        
        # 方法1：基于大小
        size_est = self.size_estimator.estimate_from_bbox(
            bbox, object_name, real_height, real_width
        )
        if size_est.distance < float('inf'):
            estimates.append(size_est)
        
        # 方法2：基于地平面
        ground_est = self.ground_estimator.estimate_from_bbox(bbox)
        if ground_est.distance < float('inf'):
            estimates.append(ground_est)
        
        # 方法3：基于纹理
        texture_est = self.texture_estimator.estimate(image, bbox)
        if texture_est.distance < float('inf'):
            estimates.append(texture_est)
        
        if not estimates:
            return DepthEstimate(
                distance=float('inf'),
                confidence=0.0,
                method='none',
                bbox=bbox
            )
        
        # 加权融合
        total_weight = sum(e.confidence for e in estimates)
        if total_weight == 0:
            # 等权重
            avg_distance = np.mean([e.distance for e in estimates])
            confidence = 0.5
        else:
            avg_distance = sum(e.distance * e.confidence for e in estimates) / total_weight
            confidence = min(1.0, total_weight / len(estimates))
        
        return DepthEstimate(
            distance=avg_distance,
            confidence=confidence,
            method='fused',
            bbox=bbox
        )
    
    def estimate_depth_map(self, image: np.ndarray, 
                           scale_factor: float = 0.25) -> np.ndarray:
        """
        生成简易深度图（基于图像亮度和位置）
        这是一个非常粗糙的估计，仅用于可视化
        Args:
            image: 输入图像
            scale_factor: 缩放因子
        Returns:
            深度图（float32，单位：厘米）
        """
        h, w = image.shape[:2]
        
        # 缩放以加速
        small_h, small_w = int(h * scale_factor), int(w * scale_factor)
        small = cv2.resize(image, (small_w, small_h))
        
        # 转灰度
        if len(small.shape) == 3:
            gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY).astype(np.float32)
        else:
            gray = small.astype(np.float32)
        
        # 基于y坐标的距离（越靠下越近）
        y_coords = np.linspace(0, 1, small_h).reshape(-1, 1)
        y_map = np.tile(y_coords, (1, small_w))
        
        # 简单的深度估计：假设地面，y越大距离越近
        depth = 200.0 / (y_map + 0.1)
        
        # 用边缘信息修正（边缘通常在物体边界）
        edges = cv2.Canny((gray).astype(np.uint8), 50, 150).astype(np.float32) / 255
        depth = depth * (1 + edges * 0.2)
        
        # 归一化
        depth = cv2.resize(depth, (w, h))
        
        return depth
    
    def visualize_depth(self, image: np.ndarray, 
                       estimates: List[DepthEstimate] = None) -> np.ndarray:
        """
        可视化深度信息
        Args:
            image: 输入图像
            estimates: 深度估计列表
        Returns:
            可视化图像
        """
        vis = image.copy()
        
        # 绘制深度估计
        if estimates:
            for est in estimates:
                x, y, w, h = est.bbox
                
                # 颜色根据距离
                if est.distance < 50:
                    color = (0, 0, 255)  # 红：近
                elif est.distance < 150:
                    color = (0, 255, 255)  # 黄：中
                else:
                    color = (0, 255, 0)  # 绿：远
                
                cv2.rectangle(vis, (x, y), (x + w, y + h), color, 2)
                
                text = f"{est.distance:.0f}cm ({est.confidence:.1%})"
                cv2.putText(vis, text, (x, y - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        return vis


# 使用示例
if __name__ == "__main__":
    estimator = DepthEstimator(
        focal_length=500.0,
        camera_height=30.0,
        camera_pitch=15.0
    )
    
    # 注册常见物体
    estimator.register_object("ball", real_height=6.5, real_width=6.5)
    estimator.register_object("cup", real_height=10.0, real_width=8.0)
    
    cap = cv2.VideoCapture(0)
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            # 简单目标检测（使用颜色阈值）
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, (0, 100, 100), (10, 255, 255))
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            estimates = []
            for contour in contours:
                if cv2.contourArea(contour) > 500:
                    bbox = cv2.boundingRect(contour)
                    est = estimator.estimate_depth(frame, bbox)
                    estimates.append(est)
            
            vis = estimator.visualize_depth(frame, estimates)
            cv2.imshow("Depth Estimation", vis)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    finally:
        cap.release()
        cv2.destroyAllWindows()
