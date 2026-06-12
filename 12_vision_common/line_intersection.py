"""
直线交点计算 + 角度计算
适用于电赛中直线检测、交叉点定位、角度测量等场景
依赖: numpy, opencv-python
"""

import cv2
import numpy as np
from typing import Tuple, Optional, List


def line_intersection(line1: Tuple[float, float, float, float],
                      line2: Tuple[float, float, float, float]) -> Optional[Tuple[float, float]]:
    """
    计算两条直线的交点
    line1, line2: (x1, y1, x2, y2) 各两个点的坐标
    返回交点 (x, y)，平行时返回 None
    """
    x1, y1, x2, y2 = line1
    x3, y3, x4, y4 = line2

    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(denom) < 1e-10:
        return None  # 平行或重合

    t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom

    ix = x1 + t * (x2 - x1)
    iy = y1 + t * (y2 - y1)
    return (ix, iy)


def line_intersection_polar(rho1, theta1, rho2, theta2) -> Optional[Tuple[float, float]]:
    """
    基于霍夫变换极坐标参数 (rho, theta) 计算交点
    """
    cos1, sin1 = np.cos(theta1), np.sin(theta1)
    cos2, sin2 = np.cos(theta2), np.sin(theta2)

    denom = cos1 * sin2 - sin1 * cos2
    if abs(denom) < 1e-10:
        return None

    x = (sin2 * rho1 - sin1 * rho2) / denom
    y = (cos1 * rho2 - cos2 * rho1) / denom
    return (x, y)


def angle_between_lines(line1: Tuple[float, float, float, float],
                        line2: Tuple[float, float, float, float]) -> float:
    """
    计算两条线段所在直线的夹角（锐角），返回角度值（度）
    """
    x1, y1, x2, y2 = line1
    x3, y3, x4, y4 = line2

    dx1, dy1 = x2 - x1, y2 - y1
    dx2, dy2 = x4 - x3, y4 - y3

    dot = dx1 * dx2 + dy1 * dy2
    mag1 = np.sqrt(dx1**2 + dy1**2)
    mag2 = np.sqrt(dx2**2 + dy2**2)

    if mag1 < 1e-10 or mag2 < 1e-10:
        return 0.0

    cos_angle = np.clip(dot / (mag1 * mag2), -1.0, 1.0)
    angle_rad = np.arccos(abs(cos_angle))
    return np.degrees(angle_rad)


def angle_between_lines_polar(theta1: float, theta2: float) -> float:
    """
    基于霍夫角度计算两条线的夹角（度）
    """
    diff = abs(theta1 - theta2)
    diff = min(diff, np.pi - diff)
    return np.degrees(diff)


def point_to_line_distance(point: Tuple[float, float],
                           line: Tuple[float, float, float, float]) -> float:
    """点到直线的距离"""
    px, py = point
    x1, y1, x2, y2 = line
    dx, dy = x2 - x1, y2 - y1
    length = np.sqrt(dx**2 + dy**2)
    if length < 1e-10:
        return np.sqrt((px - x1)**2 + (py - y1)**2)
    return abs(dy * px - dx * py + x2 * y1 - y2 * x1) / length


def find_lines_hough(gray: np.ndarray,
                     threshold: int = 80,
                     min_line_length: int = 50,
                     max_line_gap: int = 10) -> List[Tuple[float, float, float, float]]:
    """
    使用概率霍夫变换检测直线
    返回线段列表 [(x1,y1,x2,y2), ...]
    """
    edges = cv2.Canny(gray, 50, 150, apertureSize=3)
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold,
                            minLineLength=min_line_length,
                            maxLineGap=max_line_gap)
    if lines is None:
        return []
    return [tuple(line[0]) for line in lines]


def find_all_intersections(lines: List[Tuple[float, float, float, float]],
                           image_shape: Optional[Tuple[int, int]] = None) -> List[Tuple[float, float]]:
    """
    计算所有线段两两之间的有效交点
    image_shape: (h, w)，若提供则只保留图像范围内的交点
    """
    intersections = []
    h, w = image_shape if image_shape else (float('inf'), float('inf'))

    for i in range(len(lines)):
        for j in range(i + 1, len(lines)):
            pt = line_intersection(lines[i], lines[j])
            if pt is not None:
                x, y = pt
                if -10 <= x <= w + 10 and -10 <= y <= h + 10:
                    intersections.append(pt)
    return intersections


def cluster_intersections(points: List[Tuple[float, float]],
                          eps: float = 20.0) -> List[Tuple[float, float]]:
    """
    对交点进行聚类，合并距离 < eps 的点
    """
    if not points:
        return []

    pts = np.array(points, dtype=np.float64)
    used = [False] * len(pts)
    clusters = []

    for i in range(len(pts)):
        if used[i]:
            continue
        group = [pts[i]]
        used[i] = True
        for j in range(i + 1, len(pts)):
            if not used[j] and np.linalg.norm(pts[i] - pts[j]) < eps:
                group.append(pts[j])
                used[j] = True
        clusters.append(tuple(np.mean(group, axis=0)))

    return clusters


# ==================== 演示 ====================
if __name__ == '__main__':
    # 1. 交点计算
    l1 = (0, 0, 10, 10)
    l2 = (0, 10, 10, 0)
    pt = line_intersection(l1, l2)
    print(f"交点: {pt}")  # (5.0, 5.0)

    # 2. 夹角计算
    angle = angle_between_lines(l1, l2)
    print(f"夹角: {angle:.1f}°")  # 90.0

    # 3. 极坐标交点
    pt2 = line_intersection_polar(5, 0, 5, np.pi/2)
    print(f"极坐标交点: {pt2}")

    # 4. 图像直线检测演示（需要实际图像）
    # img = cv2.imread('test.jpg', cv2.IMREAD_GRAYSCALE)
    # lines = find_lines_hough(img)
    # intersections = find_all_intersections(lines, img.shape)
    # clustered = cluster_intersections(intersections)
    # print(f"检测到 {len(lines)} 条线, {len(clustered)} 个交点")
