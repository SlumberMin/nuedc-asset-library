"""
椭圆拟合 + 椭圆参数提取模块
适用场景: 电赛中圆形/椭圆目标检测、角度测量、圆度判断
"""

import cv2
import numpy as np


# ======================== 椭圆拟合 ========================

def fit_ellipse(contour):
    """
    拟合椭圆 (至少需要5个点)
    :return: ellipse = ((cx, cy), (w, h), angle)
             w, h 为椭圆两轴长度(直径), angle为旋转角度
    """
    if len(contour) < 5:
        return None
    ellipse = cv2.fitEllipse(contour)
    return ellipse


def fit_ellipse_robust(binary, min_points=5):
    """从二值图拟合最大轮廓的椭圆"""
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, None
    cnt = max(contours, key=cv2.contourArea)
    if len(cnt) < min_points:
        return None, cnt
    ellipse = cv2.fitEllipse(cnt)
    return ellipse, cnt


def fit_ellipse_all(binary, min_points=5, min_area=50):
    """拟合所有轮廓的椭圆"""
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    ellipses = []
    for cnt in contours:
        if len(cnt) < min_points:
            continue
        if cv2.contourArea(cnt) < min_area:
            continue
        ellipse = cv2.fitEllipse(cnt)
        ellipses.append({'contour': cnt, 'ellipse': ellipse})
    return ellipses


# ======================== 椭圆参数提取 ========================

def ellipse_params(ellipse):
    """
    提取椭圆详细参数
    :return: dict {center, axes, width, height, angle,
                   semi_major, semi_minor, area, eccentricity, aspect_ratio}
    """
    (cx, cy), (w, h), angle = ellipse

    semi_a = max(w, h) / 2.0  # 半长轴
    semi_b = min(w, h) / 2.0  # 半短轴

    # 长轴对应的角度修正
    if w > h:
        major_angle = angle
    else:
        major_angle = angle + 90

    # 面积
    area = np.pi * semi_a * semi_b

    # 离心率
    if semi_a > 0:
        eccentricity = np.sqrt(1 - (semi_b / semi_a) ** 2)
    else:
        eccentricity = 0

    return {
        'center': (cx, cy),
        'axes': (w, h),
        'width': w,
        'height': h,
        'angle': angle,
        'major_angle': major_angle,  # 长轴方向角度
        'semi_major': semi_a,
        'semi_minor': semi_b,
        'area': area,
        'eccentricity': eccentricity,
        'aspect_ratio': semi_b / semi_a if semi_a > 0 else 0
    }


def ellipse_roundness(contour, ellipse=None):
    """
    椭圆拟合度/圆度: 轮廓面积与拟合椭圆面积之比
    值越接近1, 轮廓越接近椭圆
    """
    if ellipse is None:
        ellipse = fit_ellipse(contour)
        if ellipse is None:
            return 0.0

    cnt_area = cv2.contourArea(contour)
    params = ellipse_params(ellipse)
    ellipse_area = params['area']

    if ellipse_area == 0:
        return 0.0
    return min(cnt_area, ellipse_area) / max(cnt_area, ellipse_area)


def is_circle(ellipse, tolerance=0.15):
    """判断拟合椭圆是否近似为圆"""
    params = ellipse_params(ellipse)
    ratio = params['aspect_ratio']
    return ratio >= (1.0 - tolerance)


def ellipse_to_circle_params(ellipse):
    """将椭圆参数转换为等效圆参数"""
    params = ellipse_params(ellipse)
    # 等效半径 (面积相等)
    equiv_radius = np.sqrt(params['area'] / np.pi)
    return {
        'center': params['center'],
        'radius': equiv_radius,
        'area': params['area'],
        'diameter': equiv_radius * 2
    }


# ======================== 椭圆点采样 ========================

def ellipse_points(ellipse, num_pts=72):
    """在拟合椭圆上均匀采样点"""
    (cx, cy), (w, h), angle = ellipse
    a = w / 2.0
    b = h / 2.0
    angle_rad = np.radians(angle)

    t = np.linspace(0, 2 * np.pi, num_pts)
    # 未旋转的椭圆点
    x = a * np.cos(t)
    y = b * np.sin(t)
    # 旋转
    x_rot = x * np.cos(angle_rad) - y * np.sin(angle_rad) + cx
    y_rot = x * np.sin(angle_rad) + y * np.cos(angle_rad) + cy

    pts = np.column_stack([x_rot, y_rot]).astype(np.float32)
    return pts


def point_in_ellipse(point, ellipse):
    """判断点是否在椭圆内部"""
    (cx, cy), (w, h), angle = ellipse
    px, py = point
    a = w / 2.0
    b = h / 2.0
    angle_rad = np.radians(-angle)  # 反旋转

    # 坐标变换到椭圆局部坐标系
    dx = px - cx
    dy = py - cy
    local_x = dx * np.cos(angle_rad) - dy * np.sin(angle_rad)
    local_y = dx * np.sin(angle_rad) + dy * np.cos(angle_rad)

    if a == 0 or b == 0:
        return False
    return (local_x / a) ** 2 + (local_y / b) ** 2 <= 1.0


# ======================== 可视化 ========================

def draw_ellipse(img, ellipse, color=(0, 255, 0), thickness=2):
    """绘制拟合椭圆"""
    result = img.copy()
    cv2.ellipse(result, ellipse, color, thickness)
    # 标注中心
    cx, cy = int(ellipse[0][0]), int(ellipse[0][1])
    cv2.circle(result, (cx, cy), 3, (0, 0, 255), -1)
    return result


def draw_ellipse_with_axes(img, ellipse, color=(0, 255, 0), thickness=2):
    """绘制椭圆及其长短轴"""
    result = img.copy()
    cv2.ellipse(result, ellipse, color, thickness)

    params = ellipse_params(ellipse)
    cx, cy = int(params['center'][0]), int(params['center'][1])

    # 绘制长轴方向
    major_angle_rad = np.radians(params['major_angle'])
    semi_a = params['semi_major']
    ex = int(cx + semi_a * np.cos(major_angle_rad))
    ey = int(cy + semi_a * np.sin(major_angle_rad))
    cv2.line(result, (cx, cy), (ex, ey), (0, 0, 255), 2)

    # 绘制短轴方向
    minor_angle_rad = major_angle_rad + np.pi / 2
    semi_b = params['semi_minor']
    ex2 = int(cx + semi_b * np.cos(minor_angle_rad))
    ey2 = int(cy + semi_b * np.sin(minor_angle_rad))
    cv2.line(result, (cx, cy), (ex2, ey2), (255, 0, 0), 2)

    return result


# ======================== 使用示例 ========================
if __name__ == '__main__':
    img = cv2.imread('test.jpg', cv2.IMREAD_GRAYSCALE)
    _, binary = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    ellipse, cnt = fit_ellipse_robust(binary)
    if ellipse:
        params = ellipse_params(ellipse)
        print(f'椭圆中心: ({params["center"][0]:.1f}, {params["center"][1]:.1f})')
        print(f'半长轴: {params["semi_major"]:.1f}, 半短轴: {params["semi_minor"]:.1f}')
        print(f'旋转角度: {params["angle"]:.1f}°')
        print(f'离心率: {params["eccentricity"]:.3f}')
        print(f'是否为圆: {is_circle(ellipse)}')

        roundness = ellipse_roundness(cnt, ellipse)
        print(f'拟合度: {roundness:.3f}')
    else:
        print('拟合失败 (轮廓点不足)')
