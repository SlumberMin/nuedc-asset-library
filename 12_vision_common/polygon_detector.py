"""
多边形检测 - 顶点数 + 边长 + 角度
适用于电赛中几何形状识别、场地标志检测、图形分类等场景
依赖: numpy, opencv-python
"""

import cv2
import numpy as np
from typing import Tuple, Optional, List, Dict
from dataclasses import dataclass


@dataclass
class PolygonInfo:
    """多边形信息"""
    vertices: np.ndarray          # 顶点坐标 (N, 2)
    num_vertices: int             # 顶点数
    side_lengths: np.ndarray      # 各边长度
    interior_angles: np.ndarray   # 各内角（度）
    area: float                   # 面积
    perimeter: float              # 周长
    centroid: Tuple[float, float] # 质心
    bounding_rect: Tuple[int, int, int, int]  # 外接矩形 (x, y, w, h)
    regularity: float             # 规则度 (0~1, 越接近1越规则)
    shape_name: str               # 形状名称


def compute_side_lengths(vertices: np.ndarray) -> np.ndarray:
    """计算多边形各边长度"""
    n = len(vertices)
    sides = np.zeros(n)
    for i in range(n):
        p1 = vertices[i]
        p2 = vertices[(i + 1) % n]
        sides[i] = np.linalg.norm(p2 - p1)
    return sides


def compute_interior_angles(vertices: np.ndarray) -> np.ndarray:
    """计算多边形各内角（度）"""
    n = len(vertices)
    angles = np.zeros(n)
    for i in range(n):
        p_prev = vertices[(i - 1) % n]
        p_curr = vertices[i]
        p_next = vertices[(i + 1) % n]

        v1 = p_prev - p_curr
        v2 = p_next - p_curr

        cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-10)
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        angles[i] = np.degrees(np.arccos(cos_angle))
    return angles


def compute_regularity(vertices: np.ndarray) -> float:
    """
    计算多边形的规则度 (0~1)
    基于边长和角度的均匀程度
    """
    sides = compute_side_lengths(vertices)
    angles = compute_interior_angles(vertices)

    if len(sides) < 2:
        return 0.0

    # 边长变异系数
    side_cv = np.std(sides) / (np.mean(sides) + 1e-10)
    # 角度变异系数
    angle_cv = np.std(angles) / (np.mean(angles) + 1e-10)

    # 综合规则度
    regularity = 1.0 / (1.0 + side_cv + angle_cv)
    return np.clip(regularity, 0, 1)


def identify_shape(num_vertices: int, regularity: float,
                   angles: np.ndarray, sides: np.ndarray) -> str:
    """根据几何特征识别形状"""
    if num_vertices == 3:
        # 判断三角形类型
        if np.any(np.abs(angles - 90) < 5):
            return "直角三角形"
        elif np.std(angles) < 3 and abs(np.mean(angles) - 60) < 5:
            return "等边三角形"
        elif np.std(sides) / (np.mean(sides) + 1e-10) < 0.1:
            return "等边三角形"
        elif len(np.unique(np.round(sides, 0))) < 3:
            return "等腰三角形"
        elif np.max(angles) > 90:
            return "钝角三角形"
        else:
            return "锐角三角形"

    elif num_vertices == 4:
        angle_std = np.std(angles)
        side_std = np.std(sides)
        mean_angle = np.mean(angles)

        if abs(mean_angle - 90) < 5 and angle_std < 5:
            if side_std / (np.mean(sides) + 1e-10) < 0.05:
                return "正方形"
            else:
                return "矩形"
        elif abs(mean_angle - 90) < 10:
            return "平行四边形" if side_std / (np.mean(sides) + 1e-10) > 0.1 else "菱形"
        else:
            return "四边形"

    elif num_vertices == 5:
        return "五边形" if regularity > 0.85 else "不规则五边形"

    elif num_vertices == 6:
        return "六边形" if regularity > 0.85 else "不规则六边形"

    elif num_vertices > 6:
        if regularity > 0.8:
            return f"正{num_vertices}边形"
        return f"{num_vertices}边形"

    return f"{num_vertices}边形"


def detect_polygons(gray: np.ndarray,
                    approx_epsilon: float = 0.02,
                    min_area: int = 300,
                    max_area: int = 100000,
                    min_vertices: int = 3,
                    max_vertices: int = 20) -> List[PolygonInfo]:
    """
    检测图像中的多边形
    gray: 灰度图
    approx_epsilon: 多边形逼近精度（轮廓周长的比例）
    min_area/max_area: 面积筛选范围
    """
    # 预处理
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 50, 150)

    # 膨胀以闭合边缘
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    edges = cv2.dilate(edges, kernel, iterations=1)

    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    results = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area or area > max_area:
            continue

        perimeter = cv2.arcLength(cnt, True)
        epsilon = approx_epsilon * perimeter
        approx = cv2.approxPolyDP(cnt, epsilon, True)

        n = len(approx)
        if n < min_vertices or n > max_vertices:
            continue

        vertices = approx.reshape(-1, 2).astype(np.float64)
        sides = compute_side_lengths(vertices)
        angles = compute_interior_angles(vertices)
        regularity = compute_regularity(vertices)
        centroid = tuple(np.mean(vertices, axis=0))
        rect = cv2.boundingRect(approx)

        shape = identify_shape(n, regularity, angles, sides)

        info = PolygonInfo(
            vertices=vertices,
            num_vertices=n,
            side_lengths=sides,
            interior_angles=angles,
            area=area,
            perimeter=perimeter,
            centroid=centroid,
            bounding_rect=rect,
            regularity=regularity,
            shape_name=shape
        )
        results.append(info)

    return results


def draw_polygons(image: np.ndarray, polygons: List[PolygonInfo]) -> np.ndarray:
    """在图像上绘制检测到的多边形"""
    vis = image.copy()

    for poly in polygons:
        pts = poly.vertices.astype(np.int32).reshape(-1, 1, 2)
        cv2.drawContours(vis, [pts], -1, (0, 255, 0), 2)

        # 标注顶点
        for i, v in enumerate(poly.vertices):
            cv2.circle(vis, tuple(v.astype(int)), 4, (0, 0, 255), -1)

        # 标注形状名称
        cx, cy = int(poly.centroid[0]), int(poly.centroid[1])
        cv2.putText(vis, poly.shape_name, (cx - 30, cy),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

    return vis


# ==================== 演示 ====================
if __name__ == '__main__':
    # 创建测试图像
    img = np.zeros((400, 400), dtype=np.uint8)

    # 三角形
    cv2.fillPoly(img, [np.array([[50, 50], [150, 50], [100, 150]])], 255)
    # 正方形
    cv2.rectangle(img, (200, 50), (300, 150), 255, -1)
    # 五边形
    pts5 = np.array([[100, 250], [130, 200], [170, 200], [200, 250], [150, 290]], np.int32)
    cv2.fillPoly(img, [pts5], 255)
    # 圆
    cv2.circle(img, (300, 300), 40, 255, -1)

    polygons = detect_polygons(img)

    for p in polygons:
        print(f"{p.shape_name}: {p.num_vertices}个顶点, "
              f"面积={p.area:.0f}, 规则度={p.regularity:.2f}")
        print(f"  边长: {np.round(p.side_lengths, 1)}")
        print(f"  角度: {np.round(p.interior_angles, 1)}")
