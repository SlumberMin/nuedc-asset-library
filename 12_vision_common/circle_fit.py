"""
圆拟合 - 最小二乘法 + RANSAC
适用于电赛中圆形目标定位、孔洞检测、圆环测量等场景
依赖: numpy, opencv-python
"""

import cv2
import numpy as np
from typing import Tuple, Optional, List


def fit_circle_least_squares(points: np.ndarray) -> Tuple[float, float, float]:
    """
    最小二乘法拟合圆
    points: (N, 2) 数组
    返回: (cx, cy, radius)
    """
    x = points[:, 0]
    y = points[:, 1]

    # 构建线性方程组: 2x*cx + 2y*cy + d = x^2 + y^2
    # 其中 d = cx^2 + cy^2 - r^2
    A = np.column_stack([2 * x, 2 * y, np.ones(len(x))])
    b = x**2 + y**2

    result, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
    cx, cy = result[0], result[1]
    r = np.sqrt(result[2] + cx**2 + cy**2)

    return (cx, cy, r)


def fit_circle_algebraic(points: np.ndarray) -> Tuple[float, float, float]:
    """
    代数方法拟合圆（Pratt/Taubin 方法，比简单最小二乘更稳定）
    """
    x = points[:, 0].astype(np.float64)
    y = points[:, 1].astype(np.float64)

    mx, my = x.mean(), y.mean()
    x, y = x - mx, y - my

    M = np.column_stack([x, y, np.ones(len(x))])
    b = x**2 + y**2

    result, _, _, _ = np.linalg.lstsq(M, b, rcond=None)
    cx = result[0] / 2 + mx
    cy = result[1] / 2 + my
    r = np.sqrt(result[2] + cx**2 + cy**2)

    return (cx, cy, r)


def fit_circle_ransac(points: np.ndarray,
                      n_iterations: int = 1000,
                      inlier_threshold: float = 5.0,
                      min_inlier_ratio: float = 0.5) -> Tuple[float, float, float, np.ndarray]:
    """
    RANSAC 圆拟合，抗离群点
    返回: (cx, cy, radius, inlier_mask)
    """
    n = len(points)
    if n < 3:
        raise ValueError("至少需要3个点")

    best_cx, best_cy, best_r = 0, 0, 0
    best_inlier_count = 0
    best_mask = np.zeros(n, dtype=bool)

    for _ in range(n_iterations):
        # 随机选3个点
        idx = np.random.choice(n, 3, replace=False)
        p = points[idx]

        try:
            cx, cy, r = fit_circle_least_squares(p)
        except Exception:
            continue

        if r <= 0 or r > 10000:
            continue

        # 计算内点
        dist = np.abs(np.sqrt((points[:, 0] - cx)**2 + (points[:, 1] - cy)**2) - r)
        mask = dist < inlier_threshold
        inlier_count = mask.sum()

        if inlier_count > best_inlier_count:
            best_inlier_count = inlier_count
            best_cx, best_cy, best_r = cx, cy, r
            best_mask = mask.copy()

    # 用所有内点重新拟合
    if best_inlier_count >= 3:
        inlier_pts = points[best_mask]
        best_cx, best_cy, best_r = fit_circle_least_squares(inlier_pts)

    return (best_cx, best_cy, best_r, best_mask)


def detect_circles_hough(gray: np.ndarray,
                         dp: float = 1.2,
                         min_dist: int = 50,
                         param1: int = 100,
                         param2: int = 30,
                         min_radius: int = 10,
                         max_radius: int = 200) -> List[Tuple[int, int, int]]:
    """
    霍夫圆检测
    返回: [(cx, cy, r), ...]
    """
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT,
                                dp, min_dist,
                                param1=param1, param2=param2,
                                minRadius=min_radius, maxRadius=max_radius)
    if circles is None:
        return []
    return [tuple(np.round(c).astype(int)) for c in circles[0]]


def detect_circle_contour(binary: np.ndarray,
                          min_area: int = 500,
                          circularity_threshold: float = 0.8) -> List[Tuple[float, float, float]]:
    """
    基于轮廓的圆检测（面积+圆度筛选）
    返回: [(cx, cy, radius), ...]
    """
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    circles = []

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area:
            continue

        perimeter = cv2.arcLength(cnt, True)
        if perimeter < 1e-5:
            continue

        circularity = 4 * np.pi * area / (perimeter ** 2)
        if circularity < circularity_threshold:
            continue

        (cx, cy), r = cv2.minEnclosingCircle(cnt)
        circles.append((cx, cy, r))

    return circles


def fit_ellipse_to_points(points: np.ndarray) -> Tuple:
    """
    椭圆拟合（最小二乘）
    返回 OpenCV 格式的椭圆参数 (cx, cy), (w, h), angle
    """
    pts = points.astype(np.float32).reshape(-1, 1, 2)
    if len(pts) < 5:
        raise ValueError("椭圆拟合至少需要5个点")
    ellipse = cv2.fitEllipse(pts)
    return ellipse


# ==================== 演示 ====================
if __name__ == '__main__':
    np.random.seed(42)

    # 生成带噪声的圆上的点
    true_cx, true_cy, true_r = 100, 150, 50
    angles = np.random.uniform(0, 2 * np.pi, 50)
    noise = np.random.normal(0, 1, 50)
    pts = np.column_stack([
        true_cx + (true_r + noise) * np.cos(angles),
        true_cy + (true_r + noise) * np.sin(angles)
    ])

    # 加入离群点
    outliers = np.array([[0, 0], [300, 300], [50, 200]])
    pts_with_outliers = np.vstack([pts, outliers])

    # 最小二乘法
    cx, cy, r = fit_circle_least_squares(pts)
    print(f"最小二乘: cx={cx:.1f}, cy={cy:.1f}, r={r:.1f} (真实: {true_cx}, {true_cy}, {true_r})")

    # RANSAC
    cx2, cy2, r2, mask = fit_circle_ransac(pts_with_outliers, inlier_threshold=3.0)
    print(f"RANSAC:    cx={cx2:.1f}, cy={cy2:.1f}, r={r2:.1f}, 内点{mask.sum()}/{len(pts_with_outliers)}")
