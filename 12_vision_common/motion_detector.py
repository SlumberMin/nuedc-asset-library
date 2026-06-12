"""
运动检测模块
支持帧差法、背景减除法、光流法等多种运动检测方法
"""

import cv2
import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass
from enum import Enum


class MotionMethod(Enum):
    """运动检测方法"""
    FRAME_DIFF = "frame_diff"           # 帧差法
    BG_SUBTRACT_MOG2 = "mog2"           # MOG2背景减除
    BG_SUBTRACT_KNN = "knn"             # KNN背景减除
    BG_SUBTRACT_GMG = "gmg"             # GMG背景减除
    OPTICAL_FLOW = "optical_flow"       # 光流法
    ACCUM_DIFF = "accum_diff"           # 累积帧差


@dataclass
class MotionRegion:
    """运动区域"""
    bbox: Tuple[int, int, int, int]   # 边界框 (x, y, w, h)
    center: Tuple[int, int]           # 中心点
    area: int                          # 面积
    velocity: Tuple[float, float]     # 速度 (vx, vy)
    direction: float                   # 运动方向（角度）


class FrameDiffDetector:
    """帧差法运动检测"""
    
    def __init__(self, threshold: int = 30, min_area: int = 500):
        """
        Args:
            threshold: 差分阈值
            min_area: 最小运动区域面积
        """
        self.threshold = threshold
        self.min_area = min_area
        self.prev_frame = None
    
    def detect(self, frame: np.ndarray) -> Tuple[np.ndarray, List[MotionRegion]]:
        """
        检测运动
        Args:
            frame: 当前帧
        Returns:
            (运动掩码, 运动区域列表)
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if len(frame.shape) == 3 else frame
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        
        if self.prev_frame is None:
            self.prev_frame = gray
            return np.zeros_like(gray), []
        
        # 帧差
        diff = cv2.absdiff(self.prev_frame, gray)
        
        # 二值化
        _, mask = cv2.threshold(diff, self.threshold, 255, cv2.THRESH_BINARY)
        
        # 形态学操作
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.dilate(mask, kernel, iterations=1)
        
        # 更新前一帧
        self.prev_frame = gray
        
        # 提取运动区域
        regions = self._extract_regions(mask)
        
        return mask, regions
    
    def _extract_regions(self, mask: np.ndarray) -> List[MotionRegion]:
        """从掩码中提取运动区域"""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        regions = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_area:
                continue
            
            x, y, w, h = cv2.boundingRect(contour)
            center = (x + w // 2, y + h // 2)
            
            region = MotionRegion(
                bbox=(x, y, w, h),
                center=center,
                area=area,
                velocity=(0.0, 0.0),
                direction=0.0
            )
            regions.append(region)
        
        return regions


class MultiFrameDiffDetector:
    """多帧差分法"""
    
    def __init__(self, num_frames: int = 3, threshold: int = 30, min_area: int = 500):
        self.num_frames = num_frames
        self.threshold = threshold
        self.min_area = min_area
        self.prev_frames = []
    
    def detect(self, frame: np.ndarray) -> Tuple[np.ndarray, List[MotionRegion]]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if len(frame.shape) == 3 else frame
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        
        self.prev_frames.append(gray)
        if len(self.prev_frames) > self.num_frames:
            self.prev_frames.pop(0)
        
        if len(self.prev_frames) < 2:
            return np.zeros_like(gray), []
        
        # 多帧差分取交集
        masks = []
        for i in range(len(self.prev_frames) - 1):
            diff = cv2.absdiff(self.prev_frames[i], self.prev_frames[i + 1])
            _, mask = cv2.threshold(diff, self.threshold, 255, cv2.THRESH_BINARY)
            masks.append(mask)
        
        # 取交集（与操作）
        combined = masks[0]
        for m in masks[1:]:
            combined = cv2.bitwise_and(combined, m)
        
        # 形态学操作
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        combined = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, kernel, iterations=2)
        
        regions = self._extract_regions(combined)
        return combined, regions
    
    def _extract_regions(self, mask: np.ndarray) -> List[MotionRegion]:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        regions = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_area:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            center = (x + w // 2, y + h // 2)
            regions.append(MotionRegion(
                bbox=(x, y, w, h), center=center, area=area,
                velocity=(0.0, 0.0), direction=0.0
            ))
        return regions


class BgSubtractDetector:
    """背景减除法运动检测"""
    
    def __init__(self, method: str = 'mog2', 
                 min_area: int = 500,
                 history: int = 500,
                 var_threshold: int = 50):
        """
        Args:
            method: 'mog2', 'knn', 'gmg'
            min_area: 最小区域面积
            history: 背景历史帧数
            var_threshold: 方差阈值
        """
        self.min_area = min_area
        
        if method == 'mog2':
            self.subtractor = cv2.createBackgroundSubtractorMOG2(
                history=history, varThreshold=var_threshold, detectShadows=True
            )
        elif method == 'knn':
            self.subtractor = cv2.createBackgroundSubtractorKNN(
                history=history, dist2Threshold=400.0, detectShadows=True
            )
        elif method == 'gmg':
            self.subtractor = cv2.bgsegm.createBackgroundSubtractorGMG(
                initializationFrames=120, decisionThreshold=0.8
            )
        else:
            raise ValueError(f"不支持的方法: {method}")
        
        self.method = method
        self.prev_mask = None
    
    def detect(self, frame: np.ndarray) -> Tuple[np.ndarray, List[MotionRegion]]:
        # 应用背景减除
        mask = self.subtractor.apply(frame)
        
        # 去除阴影（值为127的像素）
        mask = cv2.threshold(mask, 200, 255, cv2.THRESH_BINARY)[1]
        
        # 形态学操作
        kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close, iterations=2)
        mask = cv2.dilate(mask, kernel_open, iterations=1)
        
        self.prev_mask = mask
        
        regions = self._extract_regions(mask)
        return mask, regions
    
    def _extract_regions(self, mask: np.ndarray) -> List[MotionRegion]:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        regions = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_area:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            center = (x + w // 2, y + h // 2)
            regions.append(MotionRegion(
                bbox=(x, y, w, h), center=center, area=area,
                velocity=(0.0, 0.0), direction=0.0
            ))
        return regions
    
    def learn_background(self, video_path: str, num_frames: int = 100):
        """从视频学习背景"""
        cap = cv2.VideoCapture(video_path)
        count = 0
        while cap.isOpened() and count < num_frames:
            ret, frame = cap.read()
            if not ret:
                break
            self.subtractor.apply(frame)
            count += 1
        cap.release()
        print(f"背景学习完成，使用 {count} 帧")


class OpticalFlowDetector:
    """光流法运动检测"""
    
    def __init__(self, 
                 grid_size: Tuple[int, int] = (20, 20),
                 min_magnitude: float = 2.0,
                 min_area: int = 500):
        """
        Args:
            grid_size: 稀疏光流网格大小
            min_magnitude: 最小光流幅度
            min_area: 最小运动区域面积
        """
        self.grid_size = grid_size
        self.min_magnitude = min_magnitude
        self.min_area = min_area
        self.prev_gray = None
        
        # 生成网格点
        self.points = None
        
        # Lucas-Kanade参数
        self.lk_params = dict(
            winSize=(15, 15),
            maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03)
        )
    
    def _init_points(self, shape: Tuple[int, int]):
        """初始化网格点"""
        h, w = shape
        step_x = w // self.grid_size[0]
        step_y = h // self.grid_size[1]
        
        points = []
        for y in range(step_y // 2, h, step_y):
            for x in range(step_x // 2, w, step_x):
                points.append([x, y])
        
        self.points = np.array(points, dtype=np.float32).reshape(-1, 1, 2)
    
    def detect(self, frame: np.ndarray) -> Tuple[Optional[np.ndarray], List[MotionRegion]]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if len(frame.shape) == 3 else frame
        
        if self.prev_gray is None:
            self.prev_gray = gray
            self._init_points(gray.shape)
            return None, []
        
        # 计算稀疏光流
        new_points, status, _ = cv2.calcOpticalFlowPyrLK(
            self.prev_gray, gray, self.points, None, **self.lk_params
        )
        
        # 筛选有效点
        good_old = self.points[status.flatten() == 1]
        good_new = new_points[status.flatten() == 1]
        
        # 计算光流
        flow = good_new - good_old
        magnitudes = np.sqrt(flow[:, 0]**2 + flow[:, 1]**2)
        
        # 创建运动掩码
        mask = np.zeros_like(gray)
        motion_vectors = []
        
        for i, (old, new, mag) in enumerate(zip(good_old, good_new, magnitudes)):
            if mag > self.min_magnitude:
                x, y = int(old[0][0]), int(old[0][1])
                vx, vy = new[0] - old[0]
                
                # 绘制运动区域
                cv2.circle(mask, (x, y), 10, 255, -1)
                motion_vectors.append((x, y, vx, vy, mag))
        
        # 形态学操作
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        
        # 提取运动区域
        regions = self._extract_regions(mask, motion_vectors)
        
        # 更新
        self.prev_gray = gray
        self.points = good_new.reshape(-1, 1, 2)
        
        # 重新初始化点（如果点太少）
        if len(self.points) < 50:
            self._init_points(gray.shape)
        
        return mask, regions
    
    def _extract_regions(self, mask: np.ndarray, 
                         motion_vectors: List[Tuple]) -> List[MotionRegion]:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        regions = []
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_area:
                continue
            
            x, y, w, h = cv2.boundingRect(contour)
            center = (x + w // 2, y + h // 2)
            
            # 计算区域内的平均速度
            region_vectors = [v for v in motion_vectors 
                            if x <= v[0] <= x + w and y <= v[1] <= y + h]
            
            if region_vectors:
                avg_vx = np.mean([v[2] for v in region_vectors])
                avg_vy = np.mean([v[3] for v in region_vectors])
                direction = np.degrees(np.arctan2(avg_vy, avg_vx))
            else:
                avg_vx, avg_vy = 0.0, 0.0
                direction = 0.0
            
            regions.append(MotionRegion(
                bbox=(x, y, w, h),
                center=center,
                area=area,
                velocity=(float(avg_vx), float(avg_vy)),
                direction=float(direction)
            ))
        
        return regions


class AccumulatedDiffDetector:
    """累积帧差法"""
    
    def __init__(self, threshold: int = 30, 
                 alpha: float = 0.05,
                 min_area: int = 500):
        self.threshold = threshold
        self.alpha = alpha
        self.min_area = min_area
        self.accumulated = None
    
    def detect(self, frame: np.ndarray) -> Tuple[np.ndarray, List[MotionRegion]]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if len(frame.shape) == 3 else frame
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        gray = gray.astype(np.float32)
        
        if self.accumulated is None:
            self.accumulated = gray
            return np.zeros(gray.shape, dtype=np.uint8), []
        
        # 累积帧差
        diff = cv2.absdiff(self.accumulated.astype(np.uint8), gray.astype(np.uint8))
        
        # 更新累积模型
        self.accumulated = self.alpha * gray + (1 - self.alpha) * self.accumulated
        
        # 二值化
        _, mask = cv2.threshold(diff.astype(np.uint8), self.threshold, 255, cv2.THRESH_BINARY)
        
        # 形态学操作
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        
        regions = self._extract_regions(mask)
        return mask, regions
    
    def _extract_regions(self, mask: np.ndarray) -> List[MotionRegion]:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        regions = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_area:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            center = (x + w // 2, y + h // 2)
            regions.append(MotionRegion(
                bbox=(x, y, w, h), center=center, area=area,
                velocity=(0.0, 0.0), direction=0.0
            ))
        return regions


class MotionDetector:
    """
    综合运动检测器
    支持多种检测方法，可组合使用
    """
    
    def __init__(self, 
                 method: MotionMethod = MotionMethod.BG_SUBTRACT_MOG2,
                 min_area: int = 500,
                 **kwargs):
        """
        Args:
            method: 检测方法
            min_area: 最小区域面积
            **kwargs: 传递给具体检测器的参数
        """
        self.method = method
        self.min_area = min_area
        
        if method == MotionMethod.FRAME_DIFF:
            self.detector = FrameDiffDetector(min_area=min_area, **kwargs)
        elif method == MotionMethod.BG_SUBTRACT_MOG2:
            self.detector = BgSubtractDetector(method='mog2', min_area=min_area, **kwargs)
        elif method == MotionMethod.BG_SUBTRACT_KNN:
            self.detector = BgSubtractDetector(method='knn', min_area=min_area, **kwargs)
        elif method == MotionMethod.BG_SUBTRACT_GMG:
            self.detector = BgSubtractDetector(method='gmg', min_area=min_area, **kwargs)
        elif method == MotionMethod.OPTICAL_FLOW:
            self.detector = OpticalFlowDetector(min_area=min_area, **kwargs)
        elif method == MotionMethod.ACCUM_DIFF:
            self.detector = AccumulatedDiffDetector(min_area=min_area, **kwargs)
        else:
            raise ValueError(f"不支持的方法: {method}")
    
    def detect(self, frame: np.ndarray) -> Tuple[np.ndarray, List[MotionRegion]]:
        """
        检测运动
        Args:
            frame: 当前帧
        Returns:
            (运动掩码, 运动区域列表)
        """
        return self.detector.detect(frame)
    
    def reset(self):
        """重置检测器状态"""
        if hasattr(self.detector, 'prev_frame'):
            self.detector.prev_frame = None
        if hasattr(self.detector, 'prev_gray'):
            self.detector.prev_gray = None
        if hasattr(self.detector, 'prev_frames'):
            self.detector.prev_frames = []
        if hasattr(self.detector, 'accumulated'):
            self.detector.accumulated = None
    
    def visualize(self, frame: np.ndarray, 
                  mask: np.ndarray,
                  regions: List[MotionRegion],
                  show_flow: bool = True) -> np.ndarray:
        """
        可视化运动检测结果
        Args:
            frame: 原始帧
            mask: 运动掩码
            regions: 运动区域
            show_flow: 是否显示运动向量
        Returns:
            可视化图像
        """
        vis = frame.copy()
        
        # 半透明叠加运动掩码
        if mask is not None:
            overlay = vis.copy()
            overlay[mask > 0] = [0, 0, 255]
            cv2.addWeighted(overlay, 0.3, vis, 0.7, 0, vis)
        
        # 绘制运动区域
        for region in regions:
            x, y, w, h = region.bbox
            
            # 边界框
            cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            # 面积信息
            text = f"Area: {region.area}"
            cv2.putText(vis, text, (x, y - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # 运动方向
            if show_flow and (region.velocity[0] != 0 or region.velocity[1] != 0):
                cx, cy = region.center
                vx, vy = region.velocity
                end_x = int(cx + vx * 5)
                end_y = int(cy + vy * 5)
                cv2.arrowedLine(vis, (cx, cy), (end_x, end_y), (255, 0, 0), 2)
        
        return vis
    
    def get_motion_summary(self, regions: List[MotionRegion]) -> dict:
        """
        获取运动摘要
        Args:
            regions: 运动区域列表
        Returns:
            摘要信息
        """
        if not regions:
            return {
                'count': 0,
                'total_area': 0,
                'avg_velocity': (0.0, 0.0),
                'has_motion': False
            }
        
        total_area = sum(r.area for r in regions)
        avg_vx = np.mean([r.velocity[0] for r in regions])
        avg_vy = np.mean([r.velocity[1] for r in regions])
        
        return {
            'count': len(regions),
            'total_area': total_area,
            'avg_velocity': (float(avg_vx), float(avg_vy)),
            'has_motion': True
        }


# 使用示例
if __name__ == "__main__":
    # 创建运动检测器
    detector = MotionDetector(method=MotionMethod.BG_SUBTRACT_MOG2, min_area=1000)
    
    cap = cv2.VideoCapture(0)
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            mask, regions = detector.detect(frame)
            
            summary = detector.get_motion_summary(regions)
            if summary['has_motion']:
                print(f"检测到 {summary['count']} 个运动区域")
            
            vis = detector.visualize(frame, mask, regions)
            cv2.imshow("Motion Detection", vis)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    finally:
        cap.release()
        cv2.destroyAllWindows()
