"""
光流法模块 - 稀疏/稠密光流 + 运动估计
适用于电赛中运动目标跟踪、运动检测、速度估计等场景

功能:
- Lucas-Kanade 稀疏光流 (跟踪特征点)
- Farneback 稠密光流 (全场运动)
- 运动矢量可视化
- 运动方向/速度估计
- 累积运动检测
"""

import cv2
import numpy as np
from typing import List, Tuple, Optional, Dict
from collections import deque


class SparseOpticalFlow:
    """稀疏光流 (Lucas-Kanade)"""

    def __init__(self, max_points: int = 200,
                 quality_level: float = 0.01,
                 min_distance: float = 10,
                 win_size: int = 21):
        """
        Args:
            max_points: 最大跟踪点数
            quality_level: 角点检测质量阈值
            min_distance: 特征点最小间距
            win_size: 搜索窗口大小
        """
        self.max_points = max_points
        self.feature_params = dict(
            maxCorners=max_points,
            qualityLevel=quality_level,
            minDistance=min_distance,
            blockSize=7
        )
        self.lk_params = dict(
            winSize=(win_size, win_size),
            maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
        )
        self.prev_gray = None
        self.prev_points = None
        self.tracks: List[List[Tuple[float, float]]] = []
        self.track_len = 30  # 轨迹最大长度

    def detect_points(self, gray: np.ndarray) -> np.ndarray:
        """检测初始特征点"""
        points = cv2.goodFeaturesToTrack(gray, **self.feature_params)
        return points

    def update(self, frame: np.ndarray) -> Dict:
        """
        处理一帧, 返回光流结果
        Args:
            frame: 当前帧 (灰度或彩色)
        Returns:
            dict: {
                'points': 当前有效点,
                'prev_points': 上一帧对应点,
                'tracks': 轨迹列表,
                'flow_vectors': 运动矢量 (dx, dy),
                'mean_motion': 平均运动矢量,
                'motion_magnitude': 平均运动幅度
            }
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if len(frame.shape) == 3 else frame
        result = {
            'points': None, 'prev_points': None, 'tracks': [],
            'flow_vectors': [], 'mean_motion': (0, 0), 'motion_magnitude': 0
        }

        if self.prev_gray is None:
            self.prev_points = self.detect_points(gray)
            self.prev_gray = gray
            return result

        if self.prev_points is None or len(self.prev_points) == 0:
            self.prev_points = self.detect_points(gray)
            self.prev_gray = gray
            return result

        # 计算光流
        next_pts, status, err = cv2.calcOpticalFlowPyrLK(
            self.prev_gray, gray, self.prev_points, None, **self.lk_params
        )

        if next_pts is None:
            self.prev_points = self.detect_points(gray)
            self.prev_gray = gray
            return result

        # 筛选好点
        good_mask = status.ravel() == 1
        good_old = self.prev_points[good_mask]
        good_new = next_pts[good_mask]

        # 更新轨迹
        flow_vectors = []
        for i, (new, old) in enumerate(zip(good_new, good_old)):
            a, b = new.ravel()
            c, d = old.ravel()
            flow_vectors.append((a - c, b - d))

            # 更新轨迹
            if i < len(self.tracks):
                self.tracks[i].append((a, b))
                if len(self.tracks[i]) > self.track_len:
                    self.tracks[i].pop(0)

        # 筛选运动量足够大的点 (排除静止点)
        if len(good_new) > 0:
            motions = np.array(flow_vectors)
            magnitudes = np.sqrt(motions[:, 0] ** 2 + motions[:, 1] ** 2)
            mean_mag = float(np.mean(magnitudes))
            mean_dx = float(np.mean(motions[:, 0]))
            mean_dy = float(np.mean(motions[:, 1]))
        else:
            mean_mag = 0
            mean_dx, mean_dy = 0, 0

        # 定期补充新特征点
        if len(good_new) < self.max_points // 2:
            new_pts = self.detect_points(gray)
            if new_pts is not None:
                if len(good_new) > 0:
                    good_new = np.vstack([good_new, new_pts.reshape(-1, 2)])
                else:
                    good_new = new_pts.reshape(-1, 2)

        result = {
            'points': good_new.reshape(-1, 1, 2) if len(good_new) > 0 else None,
            'prev_points': good_old,
            'tracks': self.tracks[-50:],  # 只返回最近50条轨迹
            'flow_vectors': flow_vectors,
            'mean_motion': (mean_dx, mean_dy),
            'motion_magnitude': mean_mag,
            'num_tracked': len(good_new)
        }

        # 更新状态
        self.prev_gray = gray
        self.prev_points = good_new.reshape(-1, 1, 2) if len(good_new) > 0 else None
        self.tracks = self.tracks[-200:]  # 限制轨迹数

        return result

    def reset(self):
        """重置跟踪状态"""
        self.prev_gray = None
        self.prev_points = None
        self.tracks = []

    @staticmethod
    def draw_flow(frame: np.ndarray, result: Dict,
                  draw_tracks: bool = True,
                  draw_arrows: bool = True) -> np.ndarray:
        """可视化稀疏光流"""
        vis = frame.copy()

        # 绘制轨迹
        if draw_tracks and result['tracks']:
            for track in result['tracks']:
                pts = np.int32(track)
                if len(pts) > 1:
                    cv2.polylines(vis, [pts], False, (0, 255, 0), 1)

        # 绘制运动箭头
        if draw_arrows and result['prev_points'] is not None and result['points'] is not None:
            for old, new in zip(result['prev_points'], result['points']):
                ox, oy = int(old[0]), int(old[1])
                nx, ny = int(new[0]), int(new[1])
                cv2.arrowedLine(vis, (ox, oy), (nx, ny), (0, 0, 255), 1, tipLength=0.3)

        # 显示平均运动
        mx, my = result['mean_motion']
        mag = result['motion_magnitude']
        cv2.putText(vis, f"Motion: ({mx:.1f}, {my:.1f}) mag={mag:.1f}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(vis, f"Points: {result['num_tracked']}",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        return vis


class DenseOpticalFlow:
    """稠密光流 (Farneback)"""

    def __init__(self, pyr_scale: float = 0.5, levels: int = 3,
                 winsize: int = 15, iterations: int = 3,
                 poly_n: int = 5, poly_sigma: float = 1.2):
        """
        Args:
            pyr_scale: 金字塔缩放因子
            levels: 金字塔层数
            winsize: 窗口大小
            iterations: 每层迭代次数
            poly_n: 多项式展开邻域大小
            poly_sigma: 高斯标准差
        """
        self.params = dict(
            pyrScale=pyr_scale, levels=levels, winSize=winsize,
            iterations=iterations, polyN=poly_n, polySigma=poly_sigma,
            flags=0
        )
        self.prev_gray = None
        self.accumulated_flow = None

    def update(self, frame: np.ndarray) -> Dict:
        """
        计算稠密光流
        Returns:
            dict: {
                'flow': 光流场 (H, W, 2),
                'magnitude': 运动幅度图,
                'angle': 运动方向图,
                'mean_magnitude': 平均运动幅度,
                'dominant_direction': 主运动方向(度),
                'motion_mask': 显著运动掩码
            }
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if len(frame.shape) == 3 else frame
        result = {
            'flow': None, 'magnitude': None, 'angle': None,
            'mean_magnitude': 0, 'dominant_direction': 0, 'motion_mask': None
        }

        if self.prev_gray is None:
            self.prev_gray = gray
            return result

        # 计算Farneback光流
        flow = cv2.calcOpticalFlowFarneback(
            self.prev_gray, gray, None, **self.params
        )

        # 分解为幅度和方向
        mag, ang = cv2.cartToPolar(flow[..., 0], flow[..., 1])

        # 统计
        mean_mag = float(np.mean(mag))
        # 计算主运动方向 (加权平均)
        valid_mask = mag > 0.5
        if np.any(valid_mask):
            ang_deg = np.degrees(ang[valid_mask])
            weights = mag[valid_mask]
            # 圆形均值
            sin_mean = np.average(np.sin(np.radians(ang_deg)), weights=weights)
            cos_mean = np.average(np.cos(np.radians(ang_deg)), weights=weights)
            dominant_dir = float(np.degrees(np.arctan2(sin_mean, cos_mean))) % 360
        else:
            dominant_dir = 0

        # 运动掩码 (阈值化)
        motion_mask = (mag > np.mean(mag) + np.std(mag)).astype(np.uint8) * 255

        # 累积运动
        if self.accumulated_flow is None:
            self.accumulated_flow = flow.copy()
        else:
            self.accumulated_flow = self.accumulated_flow * 0.9 + flow * 0.1

        result = {
            'flow': flow,
            'magnitude': mag,
            'angle': ang,
            'mean_magnitude': mean_mag,
            'dominant_direction': dominant_dir,
            'motion_mask': motion_mask,
            'accumulated_flow': self.accumulated_flow
        }

        self.prev_gray = gray
        return result

    def reset(self):
        """重置"""
        self.prev_gray = None
        self.accumulated_flow = None

    @staticmethod
    def draw_flow_hsv(flow: np.ndarray) -> np.ndarray:
        """用HSV颜色编码光流 (方向=色相, 亮度=幅度)"""
        mag, ang = cv2.cartToPolar(flow[..., 0], flow[..., 1])
        hsv = np.zeros((*flow.shape[:2], 3), dtype=np.uint8)
        hsv[..., 0] = ang * 180 / np.pi / 2  # 色相
        hsv[..., 1] = 255
        hsv[..., 2] = cv2.normalize(mag, None, 0, 255, cv2.NORM_MINMAX)
        bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        return bgr

    @staticmethod
    def draw_motion_vectors(frame: np.ndarray, flow: np.ndarray,
                            step: int = 16, scale: float = 1.0) -> np.ndarray:
        """在图上绘制运动矢量网格"""
        vis = frame.copy()
        h, w = flow.shape[:2]

        for y in range(step // 2, h, step):
            for x in range(step // 2, w, step):
                fx, fy = flow[y, x]
                dx = int(fx * scale)
                dy = int(fy * scale)
                if abs(dx) > 1 or abs(dy) > 1:
                    cv2.arrowedLine(vis, (x, y), (x + dx, y + dy),
                                    (0, 255, 0), 1, tipLength=0.3)
        return vis

    @staticmethod
    def draw_magnitude_heatmap(magnitude: np.ndarray) -> np.ndarray:
        """运动幅度热力图"""
        norm = cv2.normalize(magnitude, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        heatmap = cv2.applyColorMap(norm, cv2.COLORMAP_JET)
        return heatmap


class MotionEstimator:
    """运动估计器 - 基于光流的运动分析"""

    def __init__(self, history_len: int = 30):
        self.sparse_flow = SparseOpticalFlow()
        self.dense_flow = DenseOpticalFlow()
        self.motion_history = deque(maxlen=history_len)
        self.direction_history = deque(maxlen=history_len)

    def analyze(self, frame: np.ndarray) -> Dict:
        """
        综合运动分析
        Returns:
            dict: {
                'is_moving': 是否在运动,
                'direction': 运动方向 (度),
                'speed': 运动速度 (像素/帧),
                'motion_type': 'still'/'slow'/'fast',
                'roi_motion': 各区域运动
            }
        """
        dense_result = self.dense_flow.update(frame)
        sparse_result = self.sparse_flow.update(frame)

        mag = dense_result['mean_magnitude']
        self.motion_history.append(mag)
        self.direction_history.append(dense_result['dominant_direction'])

        avg_mag = np.mean(self.motion_history) if self.motion_history else 0

        # 判断运动类型
        if avg_mag < 0.5:
            motion_type = 'still'
        elif avg_mag < 2.0:
            motion_type = 'slow'
        else:
            motion_type = 'fast'

        return {
            'is_moving': avg_mag > 1.0,
            'direction': dense_result['dominant_direction'],
            'speed': avg_mag,
            'motion_type': motion_type,
            'dense': dense_result,
            'sparse': sparse_result
        }


# ==================== 快捷函数 ====================

def compute_flow(prev_gray: np.ndarray, curr_gray: np.ndarray) -> np.ndarray:
    """快速计算稠密光流"""
    return cv2.calcOpticalFlowFarneback(
        prev_gray, curr_gray, None, 0.5, 3, 15, 3, 5, 1.2, 0
    )


def get_motion_direction(flow: np.ndarray) -> float:
    """获取主运动方向 (度)"""
    mag, ang = cv2.cartToPolar(flow[..., 0], flow[..., 1])
    valid = mag > 0.5
    if not np.any(valid):
        return 0
    ang_deg = np.degrees(ang[valid])
    weights = mag[valid]
    sin_mean = np.average(np.sin(np.radians(ang_deg)), weights=weights)
    cos_mean = np.average(np.cos(np.radians(ang_deg)), weights=weights)
    return float(np.degrees(np.arctan2(sin_mean, cos_mean))) % 360


def get_motion_speed(flow: np.ndarray) -> float:
    """获取平均运动速度 (像素/帧)"""
    mag, _ = cv2.cartToPolar(flow[..., 0], flow[..., 1])
    return float(np.mean(mag))


# ==================== 示例与测试 ====================

if __name__ == '__main__':
    print("=== 稀疏光流测试 ===")
    lk = SparseOpticalFlow(max_points=100)

    # 模拟两帧
    frame1 = np.zeros((300, 400, 3), dtype=np.uint8)
    cv2.rectangle(frame1, (100, 100), (200, 200), (255, 255, 255), -1)

    frame2 = np.zeros((300, 400, 3), dtype=np.uint8)
    cv2.rectangle(frame2, (110, 105), (210, 205), (255, 255, 255), -1)

    r1 = lk.update(frame1)
    r2 = lk.update(frame2)
    print(f"  跟踪点数: {r2['num_tracked']}")
    print(f"  平均运动: {r2['mean_motion']}")
    print(f"  运动幅度: {r2['motion_magnitude']:.2f}")

    print("\n=== 稠密光流测试 ===")
    df = DenseOpticalFlow()
    r1 = df.update(frame1)
    r2 = df.update(frame2)
    print(f"  平均运动幅度: {r2['mean_magnitude']:.2f}")
    print(f"  主运动方向: {r2['dominant_direction']:.1f}度")

    # HSV可视化
    if r2['flow'] is not None:
        hsv_vis = DenseOpticalFlow.draw_flow_hsv(r2['flow'])
        print(f"  光流HSV图尺寸: {hsv_vis.shape}")

    print("\n=== 运动估计器测试 ===")
    me = MotionEstimator()
    me.analyze(frame1)
    analysis = me.analyze(frame2)
    print(f"  是否运动: {analysis['is_moving']}")
    print(f"  运动类型: {analysis['motion_type']}")
    print(f"  运动速度: {analysis['speed']:.2f}")

    print("\n光流法模块测试完成!")
