#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
视觉避障模块
=============
功能：基于深度图/双目视差的障碍物检测与简单路径规划，适用于电赛避障场景。

核心思路：
    1. 深度图获取（支持双目/StereoBM、单目伪深度、或外部深度传感器）
    2. 深度图分割 -> 前方区域障碍物检测
    3. 障碍物聚类与轮廓分析
    4. 代价地图生成 -> 路径规划（A* 简化版/VFH方向直方图）
    5. 输出避障控制指令

依赖：
    pip install opencv-python numpy scipy

使用示例：
    python obstacle_detection_visual.py                    # 单目避障
    python obstacle_detection_visual.py --stereo           # 双目避障
    python obstacle_detection_visual.py --image test.png   # 图片测试

作者：电赛视觉通用代码库
"""

import cv2
import numpy as np
import time
import threading
from collections import deque
from typing import Tuple, Optional, List, Dict


class DepthEstimator:
    """
    深度估计器
    
    支持三种模式：
    1. stereo: 双目立体匹配（需要双目摄像头）
    2. mono:   单目伪深度（基于颜色/纹理特征估计相对深度）
    3. external: 外部深度源（如 ToF、结构光）
    """
    
    def __init__(self, mode: str = 'mono', 
                 stereo_params: Optional[Dict] = None):
        """
        Args:
            mode: 'stereo' / 'mono' / 'external'
            stereo_params: 双目参数 {'num_disparities', 'block_size'}
        """
        self.mode = mode
        
        if mode == 'stereo':
            params = stereo_params or {}
            self.stereo = cv2.StereoBM_create(
                numDisparities=params.get('num_disparities', 64),
                blockSize=params.get('block_size', 15)
            )
            # StereoSGBM 效果更好但更慢
            self.stereo_sgbm = cv2.StereoSGBM_create(
                minDisparity=0,
                numDisparities=64,
                blockSize=5,
                P1=8 * 3 * 5 ** 2,
                P2=32 * 3 * 5 ** 2,
                disp12MaxDiff=1,
                uniquenessRatio=10,
                speckleWindowSize=100,
                speckleRange=32,
                preFilterCap=63,
                mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
            )
        
        # 深度图缓存
        self._lock = threading.Lock()
        self._depth_map: Optional[np.ndarray] = None
    
    def compute_depth_stereo(self, left: np.ndarray, right: np.ndarray, 
                              use_sgbm: bool = True) -> np.ndarray:
        """
        双目立体匹配计算视差图
        
        Args:
            left: 左目灰度图
            right: 右目灰度图
            use_sgbm: 是否使用 SGBM（更精确但更慢）
        Returns:
            视差图（归一化到 0-255）
        """
        if use_sgbm:
            disparity = self.stereo_sgbm.compute(left, right)
        else:
            disparity = self.stereo.compute(left, right)
        
        # 归一化到 0-255
        depth = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        
        with self._lock:
            self._depth_map = depth
        
        return depth
    
    def compute_depth_mono(self, frame: np.ndarray) -> np.ndarray:
        """
        单目伪深度估计
        
        原理：利用图像特征（颜色饱和度、纹理梯度、位置）估计相对深度。
        适用于简单场景（如室内平坦地面），不适用于精确测距。
        
        Args:
            frame: BGR 输入图像
        Returns:
            伪深度图（0=近，255=远）
        """
        h, w = frame.shape[:2]
        
        # 转换为 HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # 特征1：亮度（远处物体通常更亮/更淡）
        brightness = gray.astype(np.float32) / 255.0
        
        # 特征2：饱和度（远处物体饱和度更低）
        saturation = hsv[:, :, 1].astype(np.float32) / 255.0
        
        # 特征3：纹理复杂度（远处纹理更平滑）
        laplacian = cv2.Laplacian(gray, cv2.CV_64F)
        texture = np.abs(laplacian)
        texture = texture / (texture.max() + 1e-6)
        
        # 特征4：垂直位置（图像下方=近，上方=远，适用于俯视/前视相机）
        y_coords = np.linspace(0, 1, h).reshape(-1, 1)
        position_weight = np.tile(y_coords, (1, w))
        
        # 加权融合
        depth = (0.3 * brightness + 
                 0.2 * (1 - saturation) + 
                 0.2 * (1 - texture) + 
                 0.3 * position_weight)
        
        depth = (depth * 255).astype(np.uint8)
        
        # 高斯平滑
        depth = cv2.GaussianBlur(depth, (15, 15), 0)
        
        with self._lock:
            self._depth_map = depth
        
        return depth
    
    def compute_depth(self, frame: np.ndarray, 
                      right_frame: Optional[np.ndarray] = None) -> np.ndarray:
        """统一接口"""
        if self.mode == 'stereo' and right_frame is not None:
            left_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            right_gray = cv2.cvtColor(right_frame, cv2.COLOR_BGR2GRAY)
            return self.compute_depth_stereo(left_gray, right_gray)
        else:
            return self.compute_depth_mono(frame)
    
    def get_depth_map(self) -> Optional[np.ndarray]:
        with self._lock:
            return self._depth_map


class ObstacleDetector:
    """
    障碍物检测器
    
    从深度图中检测障碍物，输出障碍物位置、大小和危险等级。
    """
    
    def __init__(self, 
                 depth_threshold: int = 100,
                 min_obstacle_area: int = 500,
                 danger_distance: int = 60,
                 fov_h: float = 60.0):
        """
        Args:
            depth_threshold: 深度阈值（小于此值视为障碍物，0=近，255=远）
            min_obstacle_area: 最小障碍物面积
            danger_distance: 危险距离阈值（像素单位的深度值）
            fov_h: 摄像头水平视场角（度）
        """
        self.depth_threshold = depth_threshold
        self.min_obstacle_area = min_obstacle_area
        self.danger_distance = danger_distance
        self.fov_h = fov_h
        
        # 避障区域定义（图像坐标系）
        # roi_top: 前方关注区域的上边界比例 (0~1)
        self.roi_top = 0.3
        self.roi_bottom = 1.0
    
    def detect(self, depth_map: np.ndarray) -> Dict:
        """
        从深度图检测障碍物
        
        Args:
            depth_map: 深度图 (uint8, 0=近, 255=远)
        Returns:
            {
                'obstacles': List[Dict] - 障碍物列表，每个含 bbox, center, area, depth, danger
                'danger_mask': 危险区域掩码
                'safe_directions': 安全方向列表（角度, 安全距离）
                'closest_obstacle': 最近障碍物信息
            }
        """
        h, w = depth_map.shape
        
        # 提取前方关注区域
        roi_y1 = int(h * self.roi_top)
        roi_y2 = int(h * self.roi_bottom)
        roi = depth_map[roi_y1:roi_y2, :]
        
        # 二值化：障碍物区域
        _, obstacle_mask = cv2.threshold(roi, self.depth_threshold, 255, cv2.THRESH_BINARY_INV)
        
        # 形态学处理
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        obstacle_mask = cv2.morphologyEx(obstacle_mask, cv2.MORPH_OPEN, kernel, iterations=2)
        obstacle_mask = cv2.morphologyEx(obstacle_mask, cv2.MORPH_CLOSE, kernel, iterations=3)
        
        # 查找轮廓
        contours, _ = cv2.findContours(obstacle_mask, cv2.RETR_EXTERNAL, 
                                       cv2.CHAIN_APPROX_SIMPLE)
        
        obstacles = []
        closest_depth = 255
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_obstacle_area:
                continue
            
            bbox = cv2.boundingRect(cnt)
            x, y, bw, bh = bbox
            
            # 质心
            M = cv2.moments(cnt)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
            else:
                cx, cy = x + bw // 2, y + bh // 2
            
            # 平均深度（ROI 内）
            mask_roi = np.zeros_like(roi, dtype=np.uint8)
            cv2.drawContours(mask_roi, [cnt], -1, 255, -1)
            mean_depth = cv2.mean(roi, mask=mask_roi)[0]
            
            # 危险等级
            danger = 'HIGH' if mean_depth < self.danger_distance else \
                     'MEDIUM' if mean_depth < self.danger_distance * 2 else 'LOW'
            
            if mean_depth < closest_depth:
                closest_depth = mean_depth
            
            obstacles.append({
                'bbox': (x, y + roi_y1, bw, bh),  # 坐标转换到原图
                'center': (cx, cy + roi_y1),
                'area': area,
                'depth': mean_depth,
                'danger': danger,
                'contour': cnt + np.array([0, roi_y1])  # 坐标偏移
            })
        
        # 按深度排序（近的在前）
        obstacles.sort(key=lambda o: o['depth'])
        
        # 计算安全方向（VFH 简化版）
        safe_directions = self._compute_safe_directions(depth_map, obstacles)
        
        # 构建危险区域掩码（全图尺寸）
        danger_mask_full = np.zeros((h, w), dtype=np.uint8)
        for obs in obstacles:
            if obs['danger'] == 'HIGH':
                x, y, bw, bh = obs['bbox']
                cv2.rectangle(danger_mask_full, (x, y), (x + bw, y + bh), 255, -1)
        
        return {
            'obstacles': obstacles,
            'danger_mask': danger_mask_full,
            'safe_directions': safe_directions,
            'closest_obstacle': obstacles[0] if obstacles else None,
            'closest_depth': closest_depth
        }
    
    def _compute_safe_directions(self, depth_map: np.ndarray, 
                                  obstacles: List[Dict]) -> List[Dict]:
        """
        VFH 简化版：计算各个方向的安全距离
        
        将视野分为若干扇区，计算每个扇区的平均深度（安全距离）。
        
        Returns:
            [{'angle': 度, 'safe_dist': 安全距离}, ...]
        """
        h, w = depth_map.shape
        center_x = w // 2
        n_sectors = 18  # 每 10° 一个扇区（±90°）
        sector_angle = self.fov_h / n_sectors
        
        directions = []
        
        for i in range(n_sectors):
            # 扇区角度范围
            angle_start = -self.fov_h / 2 + i * sector_angle
            angle_end = angle_start + sector_angle
            angle_center = (angle_start + angle_end) / 2
            
            # 角度 -> 像素列范围
            x_start = int(center_x + (angle_start / (self.fov_h / 2)) * (w / 2))
            x_end = int(center_x + (angle_end / (self.fov_h / 2)) * (w / 2))
            x_start = max(0, min(w - 1, x_start))
            x_end = max(0, min(w, x_end))
            
            if x_end <= x_start:
                continue
            
            # 计算该扇区的平均深度
            roi = depth_map[int(h * 0.4):, x_start:x_end]
            if roi.size > 0:
                safe_dist = np.mean(roi)
            else:
                safe_dist = 255
            
            directions.append({
                'angle': angle_center,
                'safe_dist': safe_dist,
                'x_range': (x_start, x_end),
                'safe': safe_dist > self.danger_distance
            })
        
        return directions
    
    def draw(self, frame: np.ndarray, detection: Dict, 
             depth_map: Optional[np.ndarray] = None) -> np.ndarray:
        """
        绘制障碍物检测结果
        
        Args:
            frame: 原始图像
            detection: 检测结果
            depth_map: 深度图（可选，用于叠加显示）
        Returns:
            可视化图像
        """
        vis = frame.copy()
        h, w = vis.shape[:2]
        
        # 绘制障碍物
        danger_colors = {'HIGH': (0, 0, 255), 'MEDIUM': (0, 128, 255), 'LOW': (0, 255, 0)}
        
        for obs in detection['obstacles']:
            x, y, bw, bh = obs['bbox']
            color = danger_colors[obs['danger']]
            
            cv2.rectangle(vis, (x, y), (x + bw, y + bh), color, 2)
            cv2.circle(vis, obs['center'], 4, (0, 0, 255), -1)
            
            label = f"D:{obs['depth']:.0f} [{obs['danger']}]"
            cv2.putText(vis, label, (x, y - 8),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        
        # 绘制安全方向雷达图
        radar_cx, radar_cy = w // 2, h - 60
        radar_r = 50
        
        for d in detection['safe_directions']:
            angle_rad = np.radians(d['angle'] - 90)  # -90° 使 0° 朝上
            dx = int(radar_r * np.cos(angle_rad))
            dy = int(radar_r * np.sin(angle_rad))
            
            color = (0, 255, 0) if d['safe'] else (0, 0, 255)
            # 长度按安全距离缩放
            length = int(radar_r * d['safe_dist'] / 255.0)
            end_x = radar_cx + int(length * np.cos(angle_rad))
            end_y = radar_cy + int(length * np.sin(angle_rad))
            
            cv2.line(vis, (radar_cx, radar_cy), (end_x, end_y), color, 2)
        
        cv2.circle(vis, (radar_cx, radar_cy), 3, (255, 255, 255), -1)
        
        # 深度图叠加
        if depth_map is not None:
            depth_colored = cv2.applyColorMap(depth_map, cv2.COLORMAP_JET)
            small_depth = cv2.resize(depth_colored, (w // 4, h // 4))
            vis[0:h//4, 0:w//4] = small_depth
        
        # 信息
        n_high = sum(1 for o in detection['obstacles'] if o['danger'] == 'HIGH')
        n_med = sum(1 for o in detection['obstacles'] if o['danger'] == 'MEDIUM')
        info = [
            f"Obstacles: {len(detection['obstacles'])}",
            f"HIGH: {n_high}  MED: {n_med}",
            f"Closest: {detection['closest_depth']:.0f}"
        ]
        
        for i, text in enumerate(info):
            cv2.putText(vis, text, (w // 4 + 10, 25 + i * 22),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        return vis


class PathPlanner:
    """
    简单路径规划器
    
    基于 VFH（矢量场直方图）选择最优行进方向。
    """
    
    def __init__(self, target_direction: float = 0.0):
        """
        Args:
            target_direction: 目标方向（度，0=正前方）
        """
        self.target_direction = target_direction
    
    def plan(self, safe_directions: List[Dict]) -> Dict:
        """
        规划最优方向
        
        综合考虑：安全距离 + 与目标方向的偏差
        
        Args:
            safe_directions: ObstacleDetector 输出的安全方向列表
        Returns:
            {
                'best_angle': 最优方向角度
                'best_distance': 该方向安全距离
                'is_blocked': 前方是否完全被阻挡
                'steering': 转向控制量（正=右转）
            }
        """
        if not safe_directions:
            return {
                'best_angle': 0.0,
                'best_distance': 0,
                'is_blocked': True,
                'steering': 0.0
            }
        
        # 评分：安全距离高的、偏离目标方向小的优先
        best_score = -float('inf')
        best_dir = safe_directions[0]
        
        for d in safe_directions:
            if not d['safe']:
                continue
            
            angle_diff = abs(d['angle'] - self.target_direction)
            score = d['safe_dist'] - angle_diff * 2  # 偏差惩罚系数
            
            if score > best_score:
                best_score = score
                best_dir = d
        
        # 检查前方是否被阻挡
        front_dirs = [d for d in safe_directions 
                      if abs(d['angle'] - self.target_direction) < 15]
        is_blocked = all(not d['safe'] for d in front_dirs) if front_dirs else True
        
        # 转向控制量
        steering = (best_dir['angle'] - self.target_direction) / 45.0 * 100  # 归一化到 ±100
        steering = np.clip(steering, -100, 100)
        
        return {
            'best_angle': best_dir['angle'],
            'best_distance': best_dir['safe_dist'],
            'is_blocked': is_blocked,
            'steering': steering
        }


class ObstacleAvoidanceSystem:
    """
    视觉避障系统
    
    整合深度估计 + 障碍物检测 + 路径规划。
    """
    
    def __init__(self, depth_mode: str = 'mono', **kwargs):
        """
        Args:
            depth_mode: 深度估计模式
        """
        self.depth_estimator = DepthEstimator(mode=depth_mode)
        self.obstacle_detector = ObstacleDetector()
        self.path_planner = PathPlanner()
        
        self._lock = threading.Lock()
        self._last_result: Optional[Dict] = None
        
        self.fps = 0.0
        self._frame_count = 0
        self._fps_timer = time.time()
    
    def process_frame(self, frame: np.ndarray, 
                      right_frame: Optional[np.ndarray] = None) -> Dict:
        """
        处理一帧
        
        Args:
            frame: BGR 图像
            right_frame: 右目图像（双目模式）
        Returns:
            完整结果
        """
        # 1. 深度估计
        depth = self.depth_estimator.compute_depth(frame, right_frame)
        
        # 2. 障碍物检测
        detection = self.obstacle_detector.detect(depth)
        
        # 3. 路径规划
        plan = self.path_planner.plan(detection['safe_directions'])
        
        result = {
            'depth_map': depth,
            'detection': detection,
            'plan': plan,
            'n_obstacles': len(detection['obstacles']),
            'closest_depth': detection['closest_depth'],
            'is_blocked': plan['is_blocked'],
            'steering': plan['steering']
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
        """绘制完整结果"""
        vis = frame.copy()
        
        if result is None:
            with self._lock:
                result = self._last_result
        
        if result is None:
            return vis
        
        vis = self.obstacle_detector.draw(vis, result['detection'], result['depth_map'])
        
        # 路径规划结果
        plan = result['plan']
        h, w = vis.shape[:2]
        
        if plan['is_blocked']:
            cv2.putText(vis, "BLOCKED!", (w // 2 - 60, h // 2),
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
        else:
            cv2.putText(vis, f"Go: {plan['best_angle']:.0f} deg", 
                       (w // 2 - 60, h // 2),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        self._update_fps()
        cv2.putText(vis, f"FPS: {self.fps:.1f}", (w - 100, 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        return vis
    
    def _update_fps(self):
        self._frame_count += 1
        elapsed = time.time() - self._fps_timer
        if elapsed >= 1.0:
            self.fps = self._frame_count / elapsed
            self._frame_count = 0
            self._fps_timer = time.time()


def run_demo():
    """摄像头实时避障演示"""
    print("=" * 60)
    print("  视觉避障演示（单目伪深度模式）")
    print("  按 'q' 退出")
    print("=" * 60)
    
    system = ObstacleAvoidanceSystem(depth_mode='mono')
    
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
        
        result = system.process_frame(frame)
        vis = system.draw(frame, result)
        
        cv2.imshow("Obstacle Avoidance", vis)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description="视觉避障")
    parser.add_argument('--stereo', action='store_true', help='双目模式')
    parser.add_argument('--image', type=str, help='单张图片测试')
    args = parser.parse_args()
    
    if args.image:
        frame = cv2.imread(args.image)
        if frame is None:
            print(f"[错误] 无法读取: {args.image}")
        else:
            system = ObstacleAvoidanceSystem(depth_mode='mono')
            result = system.process_frame(frame)
            vis = system.draw(frame, result)
            print(f"障碍物数量: {result['n_obstacles']}")
            print(f"是否被阻挡: {result['is_blocked']}")
            print(f"转向控制: {result['steering']:.1f}")
            cv2.imshow("Result", vis)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
    else:
        run_demo()
