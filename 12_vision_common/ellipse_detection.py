# -*- coding: utf-8 -*-
"""
椭圆检测模块 - 霍夫变换 + 最小二乘拟合
适用于电赛中椭圆目标识别（如圆环、椭圆标记等场景）
"""

import cv2
import numpy as np


class EllipseDetector:
    """椭圆检测器：基于轮廓筛选+最小二乘椭圆拟合"""

    def __init__(self, min_area=500, max_area=500000, min_contour_points=5,
                 aspect_ratio_range=(0.3, 1.0), fit_error_threshold=0.15):
        """
        参数：
            min_area: 最小轮廓面积
            max_area: 最大轮廓面积
            min_contour_points: 拟合椭圆所需最少点数
            aspect_ratio_range: 椭圆长短轴比范围 (min, max)
            fit_error_threshold: 拟合误差阈值（越小要求越严格）
        """
        self.min_area = min_area
        self.max_area = max_area
        self.min_contour_points = min_contour_points
        self.aspect_ratio_range = aspect_ratio_range
        self.fit_error_threshold = fit_error_threshold

    def preprocess(self, image):
        """图像预处理：灰度化→高斯模糊→自适应二值化→形态学操作"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image.copy()
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        # 自适应阈值
        binary = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                       cv2.THRESH_BINARY_INV, 11, 2)
        # 形态学闭操作填充小空洞
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=2)
        return binary

    def detect(self, image):
        """
        检测图像中的椭圆
        返回：椭圆列表，每个元素为 ((cx, cy), (w, h), angle)
        """
        binary = self.preprocess(image)
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        ellipses = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_area or area > self.max_area:
                continue
            if len(cnt) < self.min_contour_points:
                continue

            # 最小二乘椭圆拟合
            ellipse = cv2.fitEllipse(cnt)
            (cx, cy), (w, h), angle = ellipse

            # 长短轴比筛选
            if w == 0 or h == 0:
                continue
            ratio = min(w, h) / max(w, h)
            if ratio < self.aspect_ratio_range[0] or ratio > self.aspect_ratio_range[1]:
                continue

            # 计算拟合误差（轮廓点到椭圆的平均距离）
            if not self._check_fit_quality(cnt, ellipse):
                continue

            ellipses.append(ellipse)

        return ellipses

    def _check_fit_quality(self, contour, ellipse, num_points=36):
        """检查椭圆拟合质量：计算轮廓点到拟合椭圆的归一化误差"""
        (cx, cy), (w, h), angle = ellipse
        a, b = max(w, h) / 2, min(w, h) / 2
        if a == 0:
            return False

        # 在拟合椭圆上采样点
        theta = np.linspace(0, 2 * np.pi, num_points)
        angle_rad = np.deg2rad(angle)
        ex = cx + a * np.cos(theta) * np.cos(angle_rad) - b * np.sin(theta) * np.sin(angle_rad)
        ey = cy + a * np.cos(theta) * np.sin(angle_rad) + b * np.sin(theta) * np.cos(angle_rad)
        ellipse_pts = np.column_stack((ex, ey))

        # 对每个轮廓点找最近椭圆点的距离
        contour_pts = contour.reshape(-1, 2).astype(np.float32)
        distances = []
        for pt in contour_pts[:100]:  # 采样100个点加速
            dists = np.sqrt(np.sum((ellipse_pts - pt) ** 2, axis=1))
            distances.append(np.min(dists))

        mean_error = np.mean(distances) / a  # 归一化误差
        return mean_error < self.fit_error_threshold

    def detect_with_hough(self, image, dp=1.2, min_dist=50,
                          param1=100, param2=50, min_radius=10, max_radius=300):
        """
        霍夫圆检测辅助（适用于接近圆形的椭圆场景）
        返回：圆列表 [(x, y, r), ...]
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
        blurred = cv2.GaussianBlur(gray, (9, 9), 2)
        circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, dp, min_dist,
                                   param1=param1, param2=param2,
                                   minRadius=min_radius, maxRadius=max_radius)
        if circles is not None:
            return np.round(circles[0]).astype(int).tolist()
        return []

    def draw_results(self, image, ellipses, color=(0, 255, 0), thickness=2):
        """在图像上绘制检测到的椭圆"""
        result = image.copy()
        for ellipse in ellipses:
            cv2.ellipse(result, ellipse, color, thickness)
            (cx, cy), _, _ = ellipse
            cv2.circle(result, (int(cx), int(cy)), 3, (0, 0, 255), -1)
        return result


# ========== 使用示例 ==========
if __name__ == '__main__':
    # 创建一个含椭圆的测试图像
    img = np.zeros((500, 600, 3), dtype=np.uint8)
    cv2.ellipse(img, (200, 200), (120, 80), 30, 0, 360, (255, 255, 255), -1)
    cv2.ellipse(img, (420, 300), (90, 50), -15, 0, 360, (200, 200, 200), -1)

    detector = EllipseDetector(min_area=200, fit_error_threshold=0.2)
    ellipses = detector.detect(img)
    print(f"检测到 {len(ellipses)} 个椭圆：")
    for i, e in enumerate(ellipses):
        (cx, cy), (w, h), angle = e
        print(f"  椭圆{i+1}: 中心({cx:.0f},{cy:.0f}) 尺寸({w:.0f}x{h:.0f}) 角度{angle:.1f}°")

    result = detector.draw_results(img, ellipses)
    cv2.imwrite("ellipse_result.jpg", result)
    print("结果已保存为 ellipse_result.jpg")
