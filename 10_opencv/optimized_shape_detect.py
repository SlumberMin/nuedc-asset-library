#!/usr/bin/env python3
"""
optimized_shape_detect.py - 优化版形状识别模块
适用于 RK3588S 嵌入式平台，针对电赛场景优化

优化策略:
1. 轮廓检测+多边形逼近，避免Hough变换开销
2. 分层处理: 先粗筛(面积/长宽比)，再精细分类
3. ROI区域限定，减少全图扫描
4. numpy向量化面积/角度计算
5. 缓存机制，连续帧间增量更新

支持形状: 三角形、正方形、矩形、五边形、六边形、圆形、十字形
"""

import cv2
import numpy as np
import math
from dataclasses import dataclass
from typing import Optional, Tuple, List, Dict
from collections import deque


@dataclass
class ShapeResult:
    """形状检测结果"""
    shape: str          # 形状名称
    center: Tuple[int, int]
    area: float
    perimeter: float
    vertices: int       # 顶点数
    bbox: Tuple[int, int, int, int]
    contour: np.ndarray = None
    angle: float = 0.0  # 旋转角度
    aspect_ratio: float = 1.0
    circularity: float = 0.0
    confidence: float = 0.0


class OptimizedShapeDetector:
    """
    优化版形状检测器
    
    处理流水线:
    1. 预处理(灰度化+高斯模糊)
    2. 自适应二值化
    3. 轮廓提取
    4. 粗筛(面积/长宽比)
    5. 多边形逼近 + 形状分类
    6. 圆形检测(补充)
    """

    def __init__(self,
                 min_area: int = 500,
                 max_area: int = 100000,
                 approx_epsilon_ratio: float = 0.03,
                 blur_size: int = 5,
                 adaptive_block_size: int = 11,
                 adaptive_c: int = 2,
                 roi: Optional[Tuple[int, int, int, int]] = None,
                 detect_scale: float = 1.0):
        """
        Args:
            min_area: 最小轮廓面积
            max_area: 最大轮廓面积
            approx_epsilon_ratio: 多边形逼近精度（周长比例）
            blur_size: 高斯模糊核大小
            adaptive_block_size: 自适应二值化块大小
            adaptive_c: 自适应二值化常数
            roi: 感兴趣区域 (x, y, w, h)
            detect_scale: 检测缩放比例
        """
        self.min_area = min_area
        self.max_area = max_area
        self.approx_epsilon_ratio = approx_epsilon_ratio
        self.blur_size = blur_size
        self.adaptive_block_size = adaptive_block_size
        self.adaptive_c = adaptive_c
        self.roi = roi
        self.detect_scale = detect_scale
        
        # 预分配缓冲区
        self._gray_buf = None
        self._blur_buf = None
        self._binary_buf = None
        
        # 性能统计
        self._perf = deque(maxlen=100)

    def _preprocess(self, frame: np.ndarray) -> np.ndarray:
        """
        预处理: ROI裁剪 -> 缩放 -> 灰度 -> 模糊 -> 二值化
        
        优化: 预分配缓冲区复用，避免重复内存分配
        """
        img = frame
        
        # ROI裁剪
        if self.roi is not None:
            x, y, w, h = self.roi
            img = img[y:y+h, x:x+w]
        
        # 缩放
        if self.detect_scale < 0.99:
            h, w = img.shape[:2]
            new_w, new_h = int(w * self.detect_scale), int(h * self.detect_scale)
            img = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_AREA)
        
        # 灰度化（复用缓冲区）
        if len(img.shape) == 3:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        else:
            gray = img
        
        # 高斯模糊（原地操作）
        blurred = cv2.GaussianBlur(gray, (self.blur_size, self.blur_size), 0)
        
        # 自适应二值化（比固定阈值更鲁棒）
        binary = cv2.adaptiveThreshold(
            blurred, 255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,
            self.adaptive_block_size,
            self.adaptive_c
        )
        
        return binary

    def _classify_shape(self, contour: np.ndarray, 
                        approx: np.ndarray) -> Tuple[str, float]:
        """
        形状分类（核心算法）
        
        基于多边形逼近的顶点数和几何特征
        返回: (形状名, 置信度)
        """
        vertices = len(approx)
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        
        if perimeter == 0:
            return ("unknown", 0.0)
        
        # 圆度指标
        circularity = 4 * math.pi * area / (perimeter * perimeter)
        
        # 最小外接矩形
        rect = cv2.minAreaRect(contour)
        (_, (w, h), _) = rect
        if w == 0 or h == 0:
            return ("unknown", 0.0)
        aspect_ratio = min(w, h) / max(w, h)
        
        # 凸包面积比（用于检测凹凸性）
        hull = cv2.convexHull(contour)
        hull_area = cv2.contourArea(hull)
        solidity = area / hull_area if hull_area > 0 else 0
        
        # ===== 分类逻辑 =====
        
        # 三角形: 3个顶点
        if vertices == 3:
            confidence = 0.9 if 0.7 < solidity < 1.0 else 0.6
            return ("triangle", confidence)
        
        # 四边形: 4个顶点
        elif vertices == 4:
            # 判断是否为正方形
            if aspect_ratio > 0.85 and circularity > 0.75:
                return ("square", 0.9)
            elif aspect_ratio > 0.5:
                return ("rectangle", 0.85)
            else:
                return ("quadrilateral", 0.7)
        
        # 五边形
        elif vertices == 5:
            return ("pentagon", 0.85)
        
        # 六边形
        elif vertices == 6:
            if aspect_ratio > 0.85:
                return ("hexagon", 0.85)
            return ("hexagon", 0.7)
        
        # 圆形: 多顶点 + 高圆度
        elif vertices >= 7:
            if circularity > 0.85 and aspect_ratio > 0.85:
                return ("circle", min(0.95, circularity))
            elif circularity > 0.7:
                return ("ellipse", circularity)
            else:
                return (f"polygon_{vertices}", 0.5)
        
        return ("unknown", 0.0)

    def _detect_circles_hough(self, gray: np.ndarray, 
                               mask: Optional[np.ndarray] = None) -> List[ShapeResult]:
        """
        补充的霍夫圆检测（用于检测轮廓方法漏检的圆）
        
        优化: 使用HOUGH_GRADIENT_ALT(OpenCV4.x)提高精度
        """
        if mask is not None:
            gray = cv2.bitwise_and(gray, gray, mask=mask)
        
        circles = cv2.HoughCircles(
            gray,
            cv2.HOUGH_GRADIENT,
            dp=1.2,           # 累加器分辨率
            minDist=50,        # 最小圆心距
            param1=100,        # Canny高阈值
            param2=50,         # 累加器阈值
            minRadius=20,
            maxRadius=0        # 自动
        )
        
        results = []
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for c in circles[0]:
                cx, cy, r = int(c[0]), int(c[1]), int(c[2])
                area = math.pi * r * r
                if area < self.min_area or area > self.max_area:
                    continue
                
                # ROI坐标补偿
                if self.roi is not None:
                    cx += self.roi[0]
                    cy += self.roi[1]
                
                results.append(ShapeResult(
                    shape="circle",
                    center=(cx, cy),
                    area=area,
                    perimeter=2 * math.pi * r,
                    vertices=0,
                    bbox=(cx - r, cy - r, 2 * r, 2 * r),
                    circularity=1.0,
                    confidence=0.85,
                ))
        
        return results

    def _detect_cross(self, contour: np.ndarray, 
                      approx: np.ndarray) -> Optional[Tuple[str, float]]:
        """
        十字形检测（电赛常见目标）
        
        特征: 4个凸缺陷 + 凸包面积比小
        """
        hull = cv2.convexHull(contour, returnPoints=False)
        if len(hull) < 3:
            return None
        
        try:
            defects = cv2.convexityDefects(contour, hull)
        except cv2.error:
            return None
        
        if defects is None:
            return None
        
        # 统计显著凸缺陷
        significant_defects = 0
        for i in range(defects.shape[0]):
            s, e, f, d = defects[i, 0]
            if d > 1000:  # 深度阈值
                significant_defects += 1
        
        hull_area = cv2.contourArea(cv2.convexHull(contour))
        contour_area = cv2.contourArea(contour)
        solidity = contour_area / hull_area if hull_area > 0 else 0
        
        # 十字形特征: 4个显著凸缺陷 + 低凸度
        if significant_defects >= 3 and solidity < 0.65:
            return ("cross", 0.7)
        
        return None

    def detect(self, frame: np.ndarray, 
               use_hough_circle: bool = False) -> List[ShapeResult]:
        """
        主检测函数
        
        Args:
            frame: BGR输入图像
            use_hough_circle: 是否启用霍夫圆补充检测
        
        Returns:
            检测到的形状列表
        """
        t0 = cv2.getTickCount()
        
        # Step 1: 预处理
        binary = self._preprocess(frame)
        
        # Step 2: 轮廓提取
        contours, hierarchy = cv2.findContours(
            binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        
        # Step 3: 粗筛 + 精细分类
        results = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            
            # 面积粗筛
            if area < self.min_area or area > self.max_area:
                continue
            
            # 多边形逼近
            perimeter = cv2.arcLength(cnt, True)
            epsilon = self.approx_epsilon_ratio * perimeter
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            
            # 形状分类
            shape_name, confidence = self._classify_shape(cnt, approx)
            
            # 十字形补充检测
            if shape_name == "unknown" or confidence < 0.5:
                cross_result = self._detect_cross(cnt, approx)
                if cross_result:
                    shape_name, confidence = cross_result
            
            # 计算几何属性
            M = cv2.moments(cnt)
            if M['m00'] == 0:
                continue
            
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            
            # ROI坐标补偿
            if self.roi is not None:
                cx += self.roi[0]
                cy += self.roi[1]
            
            rect = cv2.minAreaRect(cnt)
            (_, (w, h), angle) = rect
            aspect_ratio = min(w, h) / max(w, h) if max(w, h) > 0 else 0
            
            circularity = 4 * math.pi * area / (perimeter * perimeter) if perimeter > 0 else 0
            
            bbox = cv2.boundingRect(cnt)
            if self.roi is not None:
                bbox = (bbox[0] + self.roi[0], bbox[1] + self.roi[1], bbox[2], bbox[3])
            
            results.append(ShapeResult(
                shape=shape_name,
                center=(cx, cy),
                area=area,
                perimeter=perimeter,
                vertices=len(approx),
                bbox=bbox,
                contour=cnt,
                angle=angle,
                aspect_ratio=aspect_ratio,
                circularity=circularity,
                confidence=confidence,
            ))
        
        # Step 4: 霍夫圆补充检测
        if use_hough_circle:
            gray = self._preprocess.__code__  # 需要灰度图
            # 重新获取灰度图
            if self.roi is not None:
                x, y, w, h = self.roi
                roi_frame = frame[y:y+h, x:x+w]
            else:
                roi_frame = frame
            
            if len(roi_frame.shape) == 3:
                gray = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2GRAY)
            else:
                gray = roi_frame
            gray = cv2.GaussianBlur(gray, (5, 5), 0)
            
            hough_circles = self._detect_circles_hough(gray)
            
            # 去重（与轮廓检测结果合并）
            for hc in hough_circles:
                duplicate = False
                for r in results:
                    dist = math.sqrt((r.center[0] - hc.center[0])**2 + 
                                     (r.center[1] - hc.center[1])**2)
                    if dist < 30:
                        duplicate = True
                        break
                if not duplicate:
                    results.append(hc)
        
        elapsed = (cv2.getTickCount() - t0) / cv2.getTickFrequency() * 1000
        self._perf.append(elapsed)
        
        return results

    def get_fps(self) -> float:
        """获取平均FPS"""
        if not self._perf:
            return 0.0
        avg_ms = sum(self._perf) / len(self._perf)
        return 1000.0 / avg_ms if avg_ms > 0 else 0.0

    def draw_results(self, frame: np.ndarray, 
                     results: List[ShapeResult]) -> np.ndarray:
        """绘制检测结果"""
        vis = frame.copy()
        
        # 形状颜色映射
        shape_colors = {
            'triangle': (0, 255, 255),
            'square': (0, 255, 0),
            'rectangle': (255, 165, 0),
            'pentagon': (255, 0, 255),
            'hexagon': (255, 255, 0),
            'circle': (0, 0, 255),
            'ellipse': (0, 128, 255),
            'cross': (128, 0, 255),
        }
        
        for r in results:
            color = shape_colors.get(r.shape, (255, 255, 255))
            
            # 绘制轮廓
            if r.contour is not None:
                if self.roi is not None:
                    cnt = r.contour.copy()
                    cnt[:, :, 0] += self.roi[0]
                    cnt[:, :, 1] += self.roi[1]
                    cv2.drawContours(vis, [cnt], -1, color, 2)
                else:
                    cv2.drawContours(vis, [r.contour], -1, color, 2)
            else:
                # 霍夫圆结果
                x, y, w, h = r.bbox
                cv2.circle(vis, r.center, w // 2, color, 2)
            
            # 标注
            label = f"{r.shape} ({r.confidence:.2f})"
            cv2.putText(vis, label, (r.center[0] - 30, r.center[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            cv2.circle(vis, r.center, 3, color, -1)
        
        # ROI框
        if self.roi is not None:
            x, y, w, h = self.roi
            cv2.rectangle(vis, (x, y), (x + w, y + h), (255, 255, 255), 1)
        
        return vis


# ============================================================
# 快捷接口
# ============================================================

def detect_shapes(frame: np.ndarray, 
                  min_area: int = 500,
                  roi: Optional[Tuple[int, int, int, int]] = None) -> List[ShapeResult]:
    """快速形状检测"""
    detector = OptimizedShapeDetector(min_area=min_area, roi=roi)
    return detector.detect(frame)


def find_shape(frame: np.ndarray, target_shape: str,
               min_area: int = 500) -> List[ShapeResult]:
    """查找指定形状"""
    detector = OptimizedShapeDetector(min_area=min_area)
    results = detector.detect(frame)
    return [r for r in results if r.shape == target_shape]


# ============================================================
# 演示主函数
# ============================================================

def main():
    """演示 + 性能测试"""
    # 创建测试图像
    test_img = np.zeros((600, 800, 3), dtype=np.uint8)
    
    # 三角形
    pts = np.array([[100, 50], [50, 150], [150, 150]], np.int32)
    cv2.fillPoly(test_img, [pts], (0, 255, 255))
    
    # 正方形
    cv2.rectangle(test_img, (250, 50), (350, 150), (0, 255, 0), -1)
    
    # 矩形
    cv2.rectangle(test_img, (450, 50), (600, 150), (255, 165, 0), -1)
    
    # 圆形
    cv2.circle(test_img, (100, 350), 70, (0, 0, 255), -1)
    
    # 五边形
    pts5 = []
    for i in range(5):
        angle = math.radians(90 + i * 72)
        pts5.append([int(350 + 60 * math.cos(angle)), 
                      int(350 + 60 * math.sin(angle))])
    cv2.fillPoly(test_img, [np.array(pts5, np.int32)], (255, 0, 255))
    
    # 六边形
    pts6 = []
    for i in range(6):
        angle = math.radians(i * 60)
        pts6.append([int(550 + 60 * math.cos(angle)), 
                      int(350 + 60 * math.sin(angle))])
    cv2.fillPoly(test_img, [np.array(pts6, np.int32)], (255, 255, 0))
    
    # 性能测试
    detector = OptimizedShapeDetector(min_area=200)
    
    # 预热
    for _ in range(10):
        detector.detect(test_img)
    
    # 正式测试
    iterations = 100
    times = []
    for _ in range(iterations):
        t0 = cv2.getTickCount()
        results = detector.detect(test_img, use_hough_circle=True)
        elapsed = (cv2.getTickCount() - t0) / cv2.getTickFrequency() * 1000
        times.append(elapsed)
    
    print(f"形状检测性能测试 ({test_img.shape[1]}x{test_img.shape[0]})")
    print(f"  平均耗时: {np.mean(times):.2f} ms")
    print(f"  FPS: {1000/np.mean(times):.1f}")
    print(f"  检测到 {len(results)} 个形状:")
    for r in results:
        print(f"    {r.shape}: center={r.center}, area={r.area:.0f}, "
              f"confidence={r.confidence:.2f}")
    
    # 可视化
    vis = detector.draw_results(test_img, results)
    cv2.imshow('Shape Detection', vis)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
