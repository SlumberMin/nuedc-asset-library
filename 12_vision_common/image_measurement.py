"""
image_measurement.py - 图像测量模块
功能: 距离测量、角度测量、面积测量、参考比例尺
用法:
    dist = measure_distance(p1, p2, scale=0.5)  # 0.5mm/pixel
    angle = measure_angle(p1, vertex, p2)
    area = measure_contour_area(contour, scale=0.25)
    scale = calibrate_scale(known_mm=50, measured_px=200)
"""

import cv2
import numpy as np
import math

# ========================= 比例尺校准 =========================

def calibrate_scale(known_mm, measured_px):
    """根据已知物体校准比例尺
    :param known_mm: 已知物体的实际尺寸(mm)
    :param measured_px: 该物体在图像中的像素尺寸
    :return: mm_per_pixel 每像素对应的实际毫米数
    """
    if measured_px <= 0:
        raise ValueError("像素尺寸必须大于0")
    return known_mm / measured_px

def calibrate_scale_from_aruco(img, dict_name='DICT_4X4_50', marker_id=0, known_mm=50):
    """通过已知大小的ArUco标记自动校准比例尺"""
    try:
        from image_fiducial import detect_fiducials
    except ImportError:
        raise ImportError("需要image_fiducial模块")

    markers = detect_fiducials(img, marker_type='aruco', dict_name=dict_name)
    marker = None
    for m in markers:
        if m['id'] == marker_id:
            marker = m
            break
    if marker is None:
        return None
    # 计算标记边长(像素)
    corners = np.array(marker['corners'])
    sides = []
    for i in range(4):
        d = np.linalg.norm(corners[i] - corners[(i + 1) % 4])
        sides.append(d)
    avg_side = np.mean(sides)
    return calibrate_scale(known_mm, avg_side)

# ========================= 距离测量 =========================

def measure_distance_px(p1, p2):
    """测量两点间像素距离"""
    p1, p2 = np.array(p1), np.array(p2)
    return np.linalg.norm(p1 - p2)

def measure_distance(p1, p2, scale=1.0):
    """测量两点间实际距离
    :param p1: (x1, y1)
    :param p2: (x2, y2)
    :param scale: mm_per_pixel
    :return: 实际距离(mm)
    """
    return measure_distance_px(p1, p2) * scale

def measure_distance_contour(contour, scale=1.0):
    """测量轮廓周长(实际长度)"""
    perimeter = cv2.arcLength(contour, True)
    return perimeter * scale

def measure_line_length(img, p1, p2, scale=1.0, draw=True):
    """测量并可视化两点间距离"""
    dist_px = measure_distance_px(p1, p2)
    dist_mm = dist_px * scale
    if draw and img is not None:
        cv2.line(img, tuple(map(int, p1)), tuple(map(int, p2)), (0, 255, 0), 2)
        mid = ((int(p1[0] + p2[0]) // 2), (int(p1[1] + p2[1]) // 2))
        cv2.putText(img, f"{dist_mm:.2f}mm", (mid[0] + 5, mid[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
    return dist_mm

# ========================= 角度测量 =========================

def measure_angle(p1, vertex, p2):
    """测量三点构成的角度(度)
    :param p1: 第一个点
    :param vertex: 顶点
    :param p2: 第二个点
    :return: 角度(度, 0~180)
    """
    p1, vertex, p2 = np.array(p1, dtype=np.float64), np.array(vertex, dtype=np.float64), np.array(p2, dtype=np.float64)
    v1 = p1 - vertex
    v2 = p2 - vertex
    cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-10)
    cos_angle = np.clip(cos_angle, -1.0, 1.0)
    return np.degrees(np.arccos(cos_angle))

def measure_angle_three_points(p1, vertex, p2, img=None):
    """测量角度并可选绘制"""
    angle = measure_angle(p1, vertex, p2)
    if img is not None:
        vp = tuple(map(int, vertex))
        cv2.line(img, vp, tuple(map(int, p1)), (0, 255, 0), 2)
        cv2.line(img, vp, tuple(map(int, p2)), (0, 255, 0), 2)
        cv2.putText(img, f"{angle:.1f}deg", (vp[0] + 10, vp[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    return angle

def measure_angle_from_lines(line1, line2):
    """测量两条线段(各用两个端点表示)的夹角
    :param line1: ((x1,y1),(x2,y2))
    :param line2: ((x3,y3),(x4,y4))
    :return: 角度(度)
    """
    v1 = np.array(line1[1]) - np.array(line1[0])
    v2 = np.array(line2[1]) - np.array(line2[0])
    cos_a = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-10)
    return np.degrees(np.arccos(np.clip(cos_a, -1, 1)))

# ========================= 面积测量 =========================

def measure_contour_area_px(contour):
    """测量轮廓像素面积"""
    return cv2.contourArea(contour)

def measure_contour_area(contour, scale=1.0):
    """测量轮廓实际面积
    :param scale: mm_per_pixel
    :return: 实际面积(mm²)
    """
    area_px = cv2.contourArea(contour)
    return area_px * (scale ** 2)

def measure_min_area_rect(contour, scale=1.0):
    """最小外接矩形的长宽和面积"""
    rect = cv2.minAreaRect(contour)
    (cx, cy), (w, h), angle = rect
    w_mm, h_mm = w * scale, h * scale
    area_mm = w_mm * h_mm
    return {
        'center': (cx, cy),
        'width_mm': min(w_mm, h_mm),
        'height_mm': max(w_mm, h_mm),
        'area_mm2': area_mm,
        'angle': angle,
        'rect': rect,
    }

def measure_bounding_box(contour, scale=1.0):
    """外接矩形测量"""
    x, y, w, h = cv2.boundingRect(contour)
    return {
        'x': x, 'y': y,
        'width_px': w, 'height_px': h,
        'width_mm': w * scale,
        'height_mm': h * scale,
        'area_mm2': (w * scale) * (h * scale),
    }

def measure_circle(contour, scale=1.0):
    """拟合圆并测量"""
    if len(contour) < 5:
        return None
    (cx, cy), radius = cv2.minEnclosingCircle(contour)
    r_mm = radius * scale
    area_mm = math.pi * r_mm * r_mm
    return {
        'center': (cx, cy),
        'radius_px': radius,
        'radius_mm': r_mm,
        'diameter_mm': 2 * r_mm,
        'area_mm2': area_mm,
        'circumference_mm': 2 * math.pi * r_mm,
    }

# ========================= 多点测量 =========================

def measure_polyline(points, closed=False, scale=1.0):
    """测量折线总长度"""
    pts = np.array(points, dtype=np.float64)
    total = 0
    for i in range(len(pts) - 1):
        total += np.linalg.norm(pts[i + 1] - pts[i])
    if closed:
        total += np.linalg.norm(pts[-1] - pts[0])
    return total * scale

def measure_polygon(points, scale=1.0):
    """测量多边形面积和周长"""
    pts = np.array(points, dtype=np.float64).reshape(-1, 1, 2)
    area = cv2.contourArea(pts)
    perimeter = cv2.arcLength(pts, True)
    return {
        'area_mm2': area * (scale ** 2),
        'perimeter_mm': perimeter * scale,
    }

# ========================= 图像测量可视化 =========================

def draw_measurement(img, p1, p2, text, color=(0, 255, 255), thickness=2):
    """绘制测量标注(带箭头线和标注文字)"""
    cv2.arrowedLine(img, tuple(map(int, p1)), tuple(map(int, p2)), color, thickness, tipLength=0.05)
    mid = ((int(p1[0] + p2[0]) // 2), (int(p1[1] + p2[1]) // 2))
    cv2.putText(img, text, (mid[0], mid[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, thickness)
    return img

def draw_angle_arc(img, p1, vertex, p2, radius=40, color=(255, 0, 0), thickness=2):
    """绘制角度弧线"""
    vp = np.array(vertex, dtype=np.float64)
    v1 = np.array(p1, dtype=np.float64) - vp
    v2 = np.array(p2, dtype=np.float64) - vp
    angle1 = np.degrees(np.arctan2(v1[1], v1[0]))
    angle2 = np.degrees(np.arctan2(v2[1], v2[0]))
    cv2.ellipse(img, tuple(map(int, vertex)), (radius, radius), 0,
                angle1, angle2, color, thickness)
    return img

def draw_scale_bar(img, scale, bar_length_mm=10, position=(10, 30), color=(255, 255, 255)):
    """绘制比例尺"""
    bar_px = int(bar_length_mm / scale)
    x, y = position
    cv2.line(img, (x, y), (x + bar_px, y), color, 3)
    cv2.line(img, (x, y - 5), (x, y + 5), color, 2)
    cv2.line(img, (x + bar_px, y - 5), (x + bar_px, y + 5), color, 2)
    cv2.putText(img, f"{bar_length_mm}mm", (x, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
    return img

# ========================= 轮廓辅助 =========================

def find_measurement_contours(img, min_area=100, max_area=100000):
    """查找可用于测量的轮廓"""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if len(img.shape) == 3 else img
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    filtered = [c for c in contours if min_area <= cv2.contourArea(c) <= max_area]
    return filtered

def measure_all_contours(img, scale=1.0, min_area=100, max_area=100000):
    """批量测量所有轮廓"""
    contours = find_measurement_contours(img, min_area, max_area)
    results = []
    for i, c in enumerate(contours):
        mbr = measure_min_area_rect(c, scale)
        circ = measure_circle(c, scale)
        results.append({
            'index': i,
            'area_mm2': measure_contour_area(c, scale),
            'perimeter_mm': measure_distance_contour(c, scale),
            'bounding_box': measure_bounding_box(c, scale),
            'min_rect': mbr,
            'circle': circ,
            'contour': c,
        })
    return results

# ========================= 测试 =========================

if __name__ == '__main__':
    print("=== 图像测量模块 ===")

    # 距离测试
    d = measure_distance((0, 0), (300, 400), scale=0.5)
    print(f"距离: {d:.2f}mm")

    # 角度测试
    a = measure_angle((0, 100), (0, 0), (100, 0))
    print(f"角度: {a:.1f}度")

    # 比例尺测试
    s = calibrate_scale(known_mm=50, measured_px=200)
    print(f"比例尺: {s:.4f} mm/pixel")

    # 多边形测试
    pts = [(0, 0), (100, 0), (100, 50), (0, 50)]
    poly = measure_polygon(pts, scale=0.25)
    print(f"多边形面积: {poly['area_mm2']:.2f}mm², 周长: {poly['perimeter_mm']:.2f}mm")
