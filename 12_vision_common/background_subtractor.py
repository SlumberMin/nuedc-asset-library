"""
背景减除模块 - 高斯混合模型 + 自适应背景更新
适用于电赛中运动目标检测、入侵检测、客流计数等场景

功能:
- MOG2 高斯混合模型背景减除
- KNN 背景减除
- 自定义自适应背景更新
- 阴影检测与去除
- 形态学后处理
- 运动目标提取与跟踪
"""

import cv2
import numpy as np
from typing import List, Tuple, Optional, Dict


class BackgroundSubtractor:
    """背景减除器"""

    def __init__(self, method: str = 'MOG2',
                 history: int = 500,
                 var_threshold: float = 16,
                 detect_shadows: bool = True,
                 learning_rate: float = -1):
        """
        初始化背景减除器
        Args:
            method: 'MOG2' (高斯混合模型) / 'KNN'
            history: 背景模型历史帧数
            var_threshold: 方差阈值 (MOG2) 或 距离阈值 (KNN)
            detect_shadows: 是否检测阴影
            learning_rate: 学习率 (-1=自动, 0=不更新, 0.001~0.1=手动)
        """
        self.method = method.upper()
        self.learning_rate = learning_rate
        self.detect_shadows = detect_shadows

        if self.method == 'MOG2':
            self.subtractor = cv2.createBackgroundSubtractorMOG2(
                history=history,
                varThreshold=var_threshold,
                detectShadows=detect_shadows
            )
        elif self.method == 'KNN':
            self.subtractor = cv2.createBackgroundSubtractorKNN(
                history=history,
                dist2Threshold=var_threshold * var_threshold,
                detectShadows=detect_shadows
            )
        else:
            raise ValueError(f"不支持的方法: {self.method}, 可选: ['MOG2', 'KNN']")

        # 形态学核
        self._kernel_small = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        self._kernel_medium = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        self._kernel_large = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))

    def apply(self, frame: np.ndarray,
              remove_shadows: bool = True,
              morphological: bool = True,
              min_area: int = 100) -> Dict:
        """
        背景减除处理
        Args:
            frame: 输入帧 (BGR)
            remove_shadows: 是否去除阴影 (阴影像素值=127)
            morphological: 是否做形态学后处理
            min_area: 最小轮廓面积过滤
        Returns:
            dict: {
                'mask': 前景掩码 (二值),
                'fgmask_raw': 原始前景掩码,
                'contours': 轮廓列表,
                'bboxes': 边界框列表 [(x,y,w,h),...],
                'num_objects': 目标数量,
                'background': 背景模型 (如有)
            }
        """
        # 背景减除
        fgmask = self.subtractor.apply(frame, learningRate=self.learning_rate)

        # 去除阴影 (阴影=127, 设为0)
        if remove_shadows and self.detect_shadows:
            shadow_mask = (fgmask == 127)
            fgmask[shadow_mask] = 0

        # 二值化 (确保)
        _, fgmask = cv2.threshold(fgmask, 200, 255, cv2.THRESH_BINARY)

        # 形态学后处理
        if morphological:
            fgmask = self._morphological_cleanup(fgmask)

        # 提取轮廓和边界框
        contours, _ = cv2.findContours(fgmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        bboxes = []
        filtered_contours = []
        for c in contours:
            area = cv2.contourArea(c)
            if area >= min_area:
                x, y, w, h = cv2.boundingRect(c)
                bboxes.append((x, y, w, h))
                filtered_contours.append(c)

        return {
            'mask': fgmask,
            'fgmask_raw': fgmask,
            'contours': filtered_contours,
            'bboxes': bboxes,
            'num_objects': len(bboxes),
        }

    def _morphological_cleanup(self, mask: np.ndarray) -> np.ndarray:
        """形态学后处理: 开运算去噪 + 闭运算填充"""
        # 开运算: 去除小噪点
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._kernel_small, iterations=2)
        # 闭运算: 填充孔洞
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._kernel_medium, iterations=2)
        # 膨胀: 连接断裂区域
        mask = cv2.dilate(mask, self._kernel_small, iterations=1)
        return mask

    def get_background(self) -> Optional[np.ndarray]:
        """获取当前背景模型"""
        try:
            return self.subtractor.getBackgroundImage()
        except Exception:
            return None

    def reset(self):
        """重置背景模型"""
        if self.method == 'MOG2':
            self.subtractor = cv2.createBackgroundSubtractorMOG2(
                history=500, varThreshold=16, detectShadows=self.detect_shadows)
        else:
            self.subtractor = cv2.createBackgroundSubtractorKNN(
                history=500, dist2Threshold=256, detectShadows=self.detect_shadows)

    @staticmethod
    def draw_detections(frame: np.ndarray, result: Dict,
                        draw_contour: bool = True,
                        draw_bbox: bool = True,
                        draw_center: bool = True) -> np.ndarray:
        """可视化检测结果"""
        vis = frame.copy()

        for i, (bbox, contour) in enumerate(zip(result['bboxes'], result['contours'])):
            x, y, w, h = bbox

            if draw_bbox:
                cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 255, 0), 2)

            if draw_contour:
                cv2.drawContours(vis, [contour], -1, (0, 255, 255), 1)

            if draw_center:
                cx, cy = x + w // 2, y + h // 2
                cv2.circle(vis, (cx, cy), 4, (0, 0, 255), -1)

            cv2.putText(vis, f"#{i} ({w}x{h})", (x, y - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)

        cv2.putText(vis, f"Objects: {result['num_objects']}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        return vis


class AdaptiveBackgroundSubtractor:
    """自适应背景减除器 - 带动态背景建模和多模型融合"""

    def __init__(self, alpha: float = 0.01,
                 threshold: float = 30,
                 min_stability: int = 30):
        """
        Args:
            alpha: 背景更新学习率 (越小更新越慢)
            threshold: 前景判定阈值
            min_stability: 背景初始化需要的帧数
        """
        self.alpha = alpha
        self.threshold = threshold
        self.min_stability = min_stability
        self.background = None
        self.variance = None
        self.frame_count = 0
        self._kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

    def update(self, frame: np.ndarray) -> Dict:
        """
        自适应背景更新
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if len(frame.shape) == 3 else frame
        gray = gray.astype(np.float32)
        self.frame_count += 1

        if self.background is None:
            self.background = gray.copy()
            self.variance = np.full_like(gray, 400.0)  # 初始方差
            return {'mask': np.zeros_like(gray, dtype=np.uint8),
                    'contours': [], 'bboxes': [], 'num_objects': 0,
                    'initialized': False}

        if self.frame_count < self.min_stability:
            # 初始化阶段: 快速更新
            alpha = 0.1
            diff = np.abs(gray - self.background)
            self.background = self.background * (1 - alpha) + gray * alpha
            self.variance = self.variance * (1 - alpha) + diff * diff * alpha
            return {'mask': np.zeros_like(gray, dtype=np.uint8),
                    'contours': [], 'bboxes': [], 'num_objects': 0,
                    'initialized': False}

        # 自适应阈值 (基于局部方差)
        adaptive_thresh = np.maximum(self.threshold, 2.0 * np.sqrt(self.variance))

        # 前景检测
        diff = np.abs(gray - self.background)
        fgmask = (diff > adaptive_thresh).astype(np.uint8) * 255

        # 自适应更新 (前景区域不更新背景)
        alpha_map = np.full_like(gray, self.alpha)
        alpha_map[fgmask > 0] = self.alpha * 0.01  # 前景区域几乎不更新

        self.background = self.background * (1 - alpha_map) + gray * alpha_map
        self.variance = self.variance * (1 - alpha_map) + diff * diff * alpha_map

        # 形态学后处理
        fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_OPEN, self._kernel, iterations=1)
        fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_CLOSE, self._kernel, iterations=2)

        # 提取目标
        contours, _ = cv2.findContours(fgmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        bboxes = []
        filtered = []
        for c in contours:
            if cv2.contourArea(c) >= 100:
                bboxes.append(cv2.boundingRect(c))
                filtered.append(c)

        return {
            'mask': fgmask,
            'contours': filtered,
            'bboxes': bboxes,
            'num_objects': len(bboxes),
            'initialized': True,
            'background': self.background.astype(np.uint8),
            'variance': self.variance
        }

    def reset(self):
        """重置"""
        self.background = None
        self.variance = None
        self.frame_count = 0


class MultiZoneDetector:
    """多区域运动检测器"""

    def __init__(self, zones: List[Tuple[int, int, int, int]],
                 method: str = 'MOG2'):
        """
        Args:
            zones: 区域列表 [(x, y, w, h), ...]
            method: 背景减除方法
        """
        self.zones = zones
        self.subtractors = [BackgroundSubtractor(method) for _ in zones]

    def update(self, frame: np.ndarray) -> List[Dict]:
        """检测各区域的运动"""
        results = []
        for i, (x, y, w, h) in enumerate(self.zones):
            roi = frame[y:y + h, x:x + w]
            result = self.subtractors[i].apply(roi)
            # 坐标偏移到原图
            for bbox in result['bboxes']:
                bx, by, bw, bh = bbox
                bbox_global = (bx + x, by + y, bw, bh)
            results.append({
                'zone_id': i,
                'zone': (x, y, w, h),
                'result': result
            })
        return results


# ==================== 快捷函数 ====================

def detect_motion(frame: np.ndarray, subtractor: BackgroundSubtractor,
                  min_area: int = 500) -> List[Tuple[int, int, int, int]]:
    """快速运动检测, 返回边界框列表"""
    result = subtractor.apply(frame, min_area=min_area)
    return result['bboxes']


def create_motion_detector(method: str = 'MOG2',
                           history: int = 500) -> BackgroundSubtractor:
    """创建运动检测器"""
    return BackgroundSubtractor(method=method, history=history)


# ==================== 示例与测试 ====================

if __name__ == '__main__':
    # 创建测试视频
    bg = np.zeros((300, 400, 3), dtype=np.uint8)
    bg[:] = (50, 50, 50)  # 灰色背景

    print("=== MOG2 背景减除测试 ===")
    bs = BackgroundSubtractor('MOG2')

    for i in range(60):
        frame = bg.copy()
        # 移动的方块
        x = 50 + i * 3
        cv2.rectangle(frame, (x, 100), (x + 50, 150), (0, 0, 255), -1)

        result = bs.apply(frame, min_area=50)
        if i >= 10 and i % 10 == 0:
            print(f"  帧{i}: 检测到 {result['num_objects']} 个目标, "
                  f"bbox={result['bboxes']}")

    print(f"\n=== 自适应背景减除测试 ===")
    abs_sub = AdaptiveBackgroundSubtractor(alpha=0.01, threshold=30)

    for i in range(60):
        frame = bg.copy()
        x = 100 + i * 2
        cv2.circle(frame, (x, 150), 20, (0, 255, 0), -1)

        result = abs_sub.update(frame)
        if i >= 30 and i % 10 == 0:
            print(f"  帧{i}: 目标数={result['num_objects']}, 初始化={result['initialized']}")

    bg_model = abs_sub.background
    if bg_model is not None:
        print(f"  背景模型尺寸: {bg_model.shape}")

    # 可视化测试
    frame_vis = bg.copy()
    cv2.rectangle(frame_vis, (200, 100), (250, 150), (0, 0, 255), -1)
    result = bs.apply(frame_vis)
    vis = BackgroundSubtractor.draw_detections(frame_vis, result)
    print(f"  可视化图尺寸: {vis.shape}")

    print("\n背景减除模块测试完成!")
