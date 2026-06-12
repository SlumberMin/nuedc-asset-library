#!/usr/bin/env python3
"""
optimized_line_detect.py - 优化版直线检测模块
适用于 RK3588S 嵌入式平台，针对电赛场景优化

优化策略:
1. ROI裁剪，只处理感兴趣区域
2. 自适应Canny边缘检测，减少参数调优
3. HoughLinesP参数优化（rho/theta步长、阈值）
4. 直线合并算法（相似直线去重）
5. 直线过滤（角度、长度、位置）
6. 边缘检测+形态学预处理，减少噪声
"""

import cv2
import numpy as np
import math
from dataclasses import dataclass
from typing import Optional, Tuple, List
from collections import deque


@dataclass
class LineResult:
    """直线检测结果"""
    x1: int
    y1: int
    x2: int
    y2: int
    angle: float      # 角度 [0, 180)
    length: float     # 长度(像素)
    rho: float = 0.0  # 极坐标参数
    theta: float = 0.0
    confidence: float = 0.0

    @property
    def midpoint(self) -> Tuple[int, int]:
        return ((self.x1 + self.x2) // 2, (self.y1 + self.y2) // 2)

    @property
    def slope(self) -> float:
        dx = self.x2 - self.x1
        if dx == 0:
            return float('inf')
        return (self.y2 - self.y1) / dx


class OptimizedLineDetector:
    """
    优化版直线检测器
    
    流水线:
    1. ROI裁剪
    2. 灰度化 + 高斯模糊
    3. 自适应Canny边缘检测
    4. 形态学闭运算(连接断裂边缘)
    5. HoughLinesP检测
    6. 直线过滤 + 合并
    """

    def __init__(self,
                 roi: Optional[Tuple[int, int, int, int]] = None,
                 canny_low: int = 50,
                 canny_high: int = 150,
                 hough_rho: int = 1,
                 hough_theta: float = np.pi / 180,
                 hough_threshold: int = 50,
                 hough_min_line_length: int = 30,
                 hough_max_line_gap: int = 10,
                 min_length: float = 30.0,
                 max_angle_range: Optional[Tuple[float, float]] = None,
                 merge_angle_thresh: float = 5.0,
                 merge_dist_thresh: float = 20.0,
                 blur_size: int = 5,
                 detect_scale: float = 1.0,
                 use_adaptive_canny: bool = True):
        """
        Args:
            roi: (x, y, w, h) 感兴趣区域
            canny_low/high: Canny阈值
            hough_rho: Hough变换rho分辨率(像素)
            hough_theta: Hough变换theta分辨率(弧度)
            hough_threshold: 累加器阈值
            hough_min_line_length: 最短线段长度
            hough_max_line_gap: 最大线段间隙
            min_length: 结果最小长度
            max_angle_range: 角度过滤范围 (min_angle, max_angle)
            merge_angle_thresh: 合并角度阈值(度)
            merge_dist_thresh: 合并距离阈值(像素)
            blur_size: 高斯模糊核大小
            detect_scale: 检测缩放比例
            use_adaptive_canny: 是否使用自适应Canny
        """
        self.roi = roi
        self.canny_low = canny_low
        self.canny_high = canny_high
        self.hough_rho = hough_rho
        self.hough_theta = hough_theta
        self.hough_threshold = hough_threshold
        self.hough_min_line_length = hough_min_line_length
        self.hough_max_line_gap = hough_max_line_gap
        self.min_length = min_length
        self.max_angle_range = max_angle_range
        self.merge_angle_thresh = merge_angle_thresh
        self.merge_dist_thresh = merge_dist_thresh
        self.blur_size = blur_size
        self.detect_scale = detect_scale
        self.use_adaptive_canny = use_adaptive_canny
        
        # 性能统计
        self._perf = deque(maxlen=100)

    def _preprocess(self, frame: np.ndarray) -> np.ndarray:
        """
        预处理: ROI裁剪 -> 灰度 -> 模糊 -> 边缘 -> 形态学
        """
        img = frame
        
        # ROI裁剪
        if self.roi is not None:
            x, y, w, h = self.roi
            img = img[y:y+h, x:x+w]
        
        # 缩放
        if self.detect_scale < 0.99:
            h, w = img.shape[:2]
            img = cv2.resize(img, (int(w * self.detect_scale), 
                                    int(h * self.detect_scale)),
                             interpolation=cv2.INTER_AREA)
        
        # 灰度化
        if len(img.shape) == 3:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        else:
            gray = img
        
        # 高斯模糊
        blurred = cv2.GaussianBlur(gray, (self.blur_size, self.blur_size), 0)
        
        # 边缘检测
        if self.use_adaptive_canny:
            # 自适应Canny: 基于图像中值自动计算阈值
            median = np.median(blurred)
            low = int(max(0, 0.67 * median))
            high = int(min(255, 1.33 * median))
            edges = cv2.Canny(blurred, low, high)
        else:
            edges = cv2.Canny(blurred, self.canny_low, self.canny_high)
        
        # 形态学闭运算: 连接断裂边缘
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
        
        return edges

    def _line_angle(self, x1, y1, x2, y2) -> float:
        """计算线段角度 [0, 180)"""
        angle = math.degrees(math.atan2(y2 - y1, x2 - x1))
        if angle < 0:
            angle += 180
        return angle % 180

    def _line_length(self, x1, y1, x2, y2) -> float:
        """计算线段长度"""
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    def _angle_diff(self, a1, a2) -> float:
        """计算两角度最小差值"""
        diff = abs(a1 - a2)
        return min(diff, 180 - diff)

    def _merge_lines(self, lines: List[LineResult]) -> List[LineResult]:
        """
        直线合并算法
        
        将角度和位置相近的线段合并为一条
        使用加权平均（长度为权重）
        """
        if len(lines) <= 1:
            return lines
        
        # 按角度排序
        lines.sort(key=lambda l: l.angle)
        
        merged = []
        used = [False] * len(lines)
        
        for i in range(len(lines)):
            if used[i]:
                continue
            
            group = [lines[i]]
            used[i] = True
            
            for j in range(i + 1, len(lines)):
                if used[j]:
                    continue
                
                # 角度差检查
                if self._angle_diff(lines[i].angle, lines[j].angle) > self.merge_angle_thresh:
                    break  # 已排序，后续角度差更大
                
                # 距离检查（中点距离）
                mid_i = lines[i].midpoint
                mid_j = lines[j].midpoint
                dist = math.sqrt((mid_i[0] - mid_j[0])**2 + (mid_i[1] - mid_j[1])**2)
                
                if dist < self.merge_dist_thresh:
                    group.append(lines[j])
                    used[j] = True
            
            # 加权平均合并
            if len(group) == 1:
                merged.append(group[0])
            else:
                total_weight = sum(l.length for l in group)
                if total_weight == 0:
                    continue
                
                # 加权平均角度和中点
                avg_angle = sum(l.angle * l.length for l in group) / total_weight
                avg_cx = sum(l.midpoint[0] * l.length for l in group) / total_weight
                avg_cy = sum(l.midpoint[1] * l.length for l in group) / total_weight
                
                # 取最长线段的端点方向
                longest = max(group, key=lambda l: l.length)
                total_length = sum(l.length for l in group)
                
                # 沿平均角度方向延伸
                rad = math.radians(avg_angle)
                half_len = total_length / 2
                x1 = int(avg_cx - half_len * math.cos(rad))
                y1 = int(avg_cy - half_len * math.sin(rad))
                x2 = int(avg_cx + half_len * math.cos(rad))
                y2 = int(avg_cy + half_len * math.sin(rad))
                
                merged.append(LineResult(
                    x1=x1, y1=y1, x2=x2, y2=y2,
                    angle=avg_angle,
                    length=total_length,
                    confidence=max(l.confidence for l in group),
                ))
        
        return merged

    def _filter_lines(self, lines: List[LineResult]) -> List[LineResult]:
        """过滤线段"""
        filtered = []
        for line in lines:
            # 长度过滤
            if line.length < self.min_length:
                continue
            
            # 角度过滤
            if self.max_angle_range is not None:
                min_a, max_a = self.max_angle_range
                if not (min_a <= line.angle <= max_a):
                    continue
            
            # ROI坐标补偿
            if self.roi is not None:
                line.x1 += self.roi[0]
                line.y1 += self.roi[1]
                line.x2 += self.roi[0]
                line.y2 += self.roi[1]
            
            filtered.append(line)
        
        return filtered

    def detect(self, frame: np.ndarray,
               merge_lines: bool = True) -> List[LineResult]:
        """
        主检测函数
        
        Args:
            frame: BGR输入图像
            merge_lines: 是否合并相似线段
        
        Returns:
            检测到的直线列表
        """
        t0 = cv2.getTickCount()
        
        # Step 1: 预处理（ROI + 边缘检测）
        edges = self._preprocess(frame)
        
        # Step 2: HoughLinesP检测
        raw_lines = cv2.HoughLinesP(
            edges,
            rho=self.hough_rho,
            theta=self.hough_theta,
            threshold=self.hough_threshold,
            minLineLength=self.hough_min_line_length,
            maxLineGap=self.hough_max_line_gap,
        )
        
        # Step 3: 解析结果
        line_results = []
        if raw_lines is not None:
            scale_inv = 1.0 / self.detect_scale
            for line in raw_lines:
                x1, y1, x2, y2 = line[0]
                
                # 缩放补偿
                if self.detect_scale < 0.99:
                    x1 = int(x1 * scale_inv)
                    y1 = int(y1 * scale_inv)
                    x2 = int(x2 * scale_inv)
                    y2 = int(y2 * scale_inv)
                
                angle = self._line_angle(x1, y1, x2, y2)
                length = self._line_length(x1, y1, x2, y2)
                
                line_results.append(LineResult(
                    x1=x1, y1=y1, x2=x2, y2=y2,
                    angle=angle,
                    length=length,
                    confidence=min(1.0, length / 200),
                ))
        
        # Step 4: 合并
        if merge_lines:
            line_results = self._merge_lines(line_results)
        
        # Step 5: 过滤
        line_results = self._filter_lines(line_results)
        
        elapsed = (cv2.getTickCount() - t0) / cv2.getTickFrequency() * 1000
        self._perf.append(elapsed)
        
        return line_results

    def detect_vertical_horizontal(self, frame: np.ndarray,
                                    angle_tolerance: float = 15.0
                                    ) -> Tuple[List[LineResult], List[LineResult]]:
        """
        分别检测水平线和垂直线
        
        Args:
            angle_tolerance: 角度容差(度)
        
        Returns:
            (水平线列表, 垂直线列表)
        """
        all_lines = self.detect(frame)
        
        horizontal = []
        vertical = []
        
        for line in all_lines:
            # 水平线: 角度接近0或180
            if line.angle < angle_tolerance or line.angle > (180 - angle_tolerance):
                horizontal.append(line)
            # 垂直线: 角度接近90
            elif abs(line.angle - 90) < angle_tolerance:
                vertical.append(line)
        
        return horizontal, vertical

    def detect_intersection(self, frame: np.ndarray,
                            ) -> List[Tuple[int, int]]:
        """
        检测直线交点（用于网格/棋盘格检测）
        """
        h_lines, v_lines = self.detect_vertical_horizontal(frame)
        
        intersections = []
        for hl in h_lines:
            for vl in v_lines:
                # 计算交点
                x1, y1 = hl.x1, hl.y1
                x2, y2 = hl.x2, hl.y2
                x3, y3 = vl.x1, vl.y1
                x4, y4 = vl.x2, vl.y2
                
                denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
                if abs(denom) < 1e-6:
                    continue
                
                t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
                
                ix = int(x1 + t * (x2 - x1))
                iy = int(y1 + t * (y2 - y1))
                
                intersections.append((ix, iy))
        
        return intersections

    def get_fps(self) -> float:
        if not self._perf:
            return 0.0
        avg = sum(self._perf) / len(self._perf)
        return 1000.0 / avg if avg > 0 else 0.0

    def draw_results(self, frame: np.ndarray,
                     lines: List[LineResult] = None,
                     horizontal: List[LineResult] = None,
                     vertical: List[LineResult] = None) -> np.ndarray:
        """绘制检测结果"""
        vis = frame.copy()
        
        if lines:
            for line in lines:
                cv2.line(vis, (line.x1, line.y1), (line.x2, line.y2),
                         (0, 255, 0), 2)
                cv2.putText(vis, f"{line.angle:.1f}deg L:{line.length:.0f}",
                            (line.midpoint[0], line.midpoint[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        
        if horizontal:
            for line in horizontal:
                cv2.line(vis, (line.x1, line.y1), (line.x2, line.y2),
                         (0, 255, 255), 2)
        
        if vertical:
            for line in vertical:
                cv2.line(vis, (line.x1, line.y1), (line.x2, line.y2),
                         (255, 0, 255), 2)
        
        # ROI框
        if self.roi is not None:
            x, y, w, h = self.roi
            cv2.rectangle(vis, (x, y), (x + w, y + h), (255, 255, 255), 1)
        
        return vis


# ============================================================
# 快捷接口
# ============================================================

def detect_lines(frame: np.ndarray,
                 roi: Optional[Tuple[int, int, int, int]] = None,
                 min_length: float = 30.0) -> List[LineResult]:
    """快速直线检测"""
    detector = OptimizedLineDetector(roi=roi, min_length=min_length)
    return detector.detect(frame)


def detect_lanes(frame: np.ndarray,
                 roi: Optional[Tuple[int, int, int, int]] = None
                 ) -> Tuple[List[LineResult], List[LineResult]]:
    """车道线检测（水平+垂直分离）"""
    detector = OptimizedLineDetector(
        roi=roi,
        hough_threshold=30,
        hough_min_line_length=50,
        merge_angle_thresh=10.0,
    )
    return detector.detect_vertical_horizontal(frame)


# ============================================================
# 演示
# ============================================================

def main():
    """演示 + 性能测试"""
    # 创建测试图像
    test_img = np.zeros((480, 640, 3), dtype=np.uint8)
    
    # 水平线
    cv2.line(test_img, (50, 100), (590, 100), (255, 255, 255), 2)
    cv2.line(test_img, (50, 200), (590, 200), (255, 255, 255), 2)
    cv2.line(test_img, (50, 380), (590, 380), (255, 255, 255), 2)
    
    # 垂直线
    cv2.line(test_img, (100, 50), (100, 430), (255, 255, 255), 2)
    cv2.line(test_img, (320, 50), (320, 430), (255, 255, 255), 2)
    cv2.line(test_img, (540, 50), (540, 430), (255, 255, 255), 2)
    
    # 对角线
    cv2.line(test_img, (50, 50), (300, 300), (255, 255, 255), 2)
    
    # 添加噪声
    noise = np.random.randint(0, 30, test_img.shape, dtype=np.uint8)
    test_img = cv2.add(test_img, noise)
    
    detector = OptimizedLineDetector(min_length=50)
    
    # 预热
    for _ in range(10):
        detector.detect(test_img)
    
    # 性能测试
    iterations = 100
    times = []
    for _ in range(iterations):
        t0 = cv2.getTickCount()
        lines = detector.detect(test_img)
        elapsed = (cv2.getTickCount() - t0) / cv2.getTickFrequency() * 1000
        times.append(elapsed)
    
    print(f"直线检测性能测试 ({test_img.shape[1]}x{test_img.shape[0]})")
    print(f"  平均耗时: {np.mean(times):.2f} ms")
    print(f"  FPS: {1000/np.mean(times):.1f}")
    print(f"  检测到 {len(lines)} 条直线:")
    for line in lines:
        print(f"    ({line.x1},{line.y1})->({line.x2},{line.y2}) "
              f"angle={line.angle:.1f} len={line.length:.0f}")
    
    # 交点检测
    intersections = detector.detect_intersection(test_img)
    print(f"  检测到 {len(intersections)} 个交点:")
    for pt in intersections:
        print(f"    ({pt[0]}, {pt[1]})")
    
    vis = detector.draw_results(test_img, lines)
    cv2.imshow('Line Detection', vis)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
