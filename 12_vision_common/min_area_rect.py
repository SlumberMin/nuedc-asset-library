"""
最小外接矩形 + 旋转矩形模块
适用场景: 电赛中目标朝向检测、矩形匹配、旋转角度测量
"""

import cv2
import numpy as np


# ======================== 最小外接矩形 ========================

def min_area_rect(contour):
    """
    最小面积旋转矩形
    :return: rect = ((cx, cy), (w, h), angle)
             angle: 矩形相对于水平的旋转角度 [-90, 0)
    """
    rect = cv2.minAreaRect(contour)
    return rect


def min_area_rect_box(contour):
    """返回旋转矩形的4个顶点坐标"""
    rect = cv2.minAreaRect(contour)
    box = cv2.boxPoints(rect)
    box = np.intp(box)
    return box, rect


def rect_to_params(rect):
    """
    提取旋转矩形参数
    :return: dict {center, size, angle, width, height, aspect_ratio}
    """
    (cx, cy), (w, h), angle = rect
    # 规范化: w始终为短边
    if w > h:
        w, h = h, w
        angle += 90
    return {
        'center': (cx, cy),
        'size': (w, h),
        'angle': angle,
        'width': w,
        'height': h,
        'aspect_ratio': w / h if h > 0 else 0,
        'area': w * h
    }


def normalize_angle(rect):
    """
    规范化旋转角度到 [-90, 90]
    使width为短边, height为长边
    """
    (cx, cy), (w, h), angle = rect
    if w > h:
        w, h = h, w
        angle = angle + 90
    # 将角度映射到 [-90, 90]
    if angle > 90:
        angle -= 180
    elif angle < -90:
        angle += 180
    return (cx, cy), (w, h), angle


# ======================== 外接矩形 ========================

def bounding_rect(contour):
    """正外接矩形 (不旋转)"""
    x, y, w, h = cv2.boundingRect(contour)
    return x, y, w, h


def bounding_rect_from_mask(binary):
    """从二值图获取最大轮廓的外接矩形"""
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    cnt = max(contours, key=cv2.contourArea)
    return cv2.boundingRect(cnt)


# ======================== 矩形匹配 ========================

def min_enclosing_circle(contour):
    """最小外接圆"""
    (cx, cy), radius = cv2.minEnclosingCircle(contour)
    return (int(cx), int(cy)), int(radius)


def fit_line_angle(contour):
    """拟合直线获取角度"""
    vx, vy, x0, y0 = cv2.fitLine(contour, cv2.DIST_L2, 0, 0.01, 0.01)
    angle = np.degrees(np.arctan2(vy, vx))[0]
    return angle, (x0[0], y0[0]), (vx[0], vy[0])


def rectangle_similarity(contour, thresh_ratio=0.85):
    """
    判断轮廓与矩形的相似度 (轮廓面积 / 外接矩形面积)
    :return: ratio, is_rectangle
    """
    x, y, w, h = cv2.boundingRect(contour)
    rect_area = w * h
    cnt_area = cv2.contourArea(contour)
    if rect_area == 0:
        return 0.0, False
    ratio = cnt_area / rect_area
    return ratio, ratio >= thresh_ratio


# ======================== 多目标旋转矩形提取 ========================

def find_rotated_rects(binary, min_area=100):
    """从二值图提取所有符合条件的旋转矩形"""
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    rects = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area:
            continue
        rect = cv2.minAreaRect(cnt)
        box = cv2.boxPoints(rect)
        box = np.intp(box)
        params = rect_to_params(rect)
        rects.append({
            'contour': cnt,
            'rect': rect,
            'box': box,
            'params': params
        })
    return rects


# ======================== 可视化 ========================

def draw_rotated_rect(img, rect, color=(0, 255, 0), thickness=2):
    """绘制旋转矩形"""
    box = cv2.boxPoints(rect)
    box = np.intp(box)
    result = img.copy()
    cv2.drawContours(result, [box], -1, color, thickness)
    return result


def draw_all_rects(img, rects, draw_angle=True):
    """绘制多个旋转矩形及角度标注"""
    result = img.copy()
    for r in rects:
        box = r['box']
        params = r['params']
        cv2.drawContours(result, [box], -1, (0, 255, 0), 2)
        cx, cy = int(params['center'][0]), int(params['center'][1])
        cv2.circle(result, (cx, cy), 3, (0, 0, 255), -1)
        if draw_angle:
            cv2.putText(result, f"{params['angle']:.1f}deg", (cx + 5, cy - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
    return result


# ======================== 使用示例 ========================
if __name__ == '__main__':
    img = cv2.imread('test.jpg', cv2.IMREAD_GRAYSCALE)
    _, binary = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    rects = find_rotated_rects(binary, min_area=200)
    print(f'检测到 {len(rects)} 个旋转矩形')

    for i, r in enumerate(rects):
        p = r['params']
        print(f"  矩形{i}: 中心={p['center']}, 宽高={p['width']:.0f}x{p['height']:.0f}, "
              f"角度={p['angle']:.1f}°, 宽高比={p['aspect_ratio']:.2f}")
