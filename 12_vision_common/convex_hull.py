"""
凸包计算 + 凸缺陷检测模块
适用场景: 电赛中形状分析、手势识别、凹凸性判断
"""

import cv2
import numpy as np


# ======================== 凸包计算 ========================

def compute_convex_hull(contour, clockwise=False):
    """
    计算单个轮廓的凸包
    :param contour: 轮廓点
    :param clockwise: 是否顺时针
    :return: 凸包点集
    """
    hull = cv2.convexHull(contour, clockwise=clockwise)
    return hull


def convex_hull_from_mask(binary):
    """从二值图计算最大轮廓的凸包"""
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, None
    cnt = max(contours, key=cv2.contourArea)
    hull = cv2.convexHull(cnt)
    return cnt, hull


def convex_hull_area_ratio(contour):
    """凸包面积比 = 轮廓面积 / 凸包面积 (实心度)"""
    hull = cv2.convexHull(contour)
    cnt_area = cv2.contourArea(contour)
    hull_area = cv2.contourArea(hull)
    if hull_area == 0:
        return 0.0
    return cnt_area / hull_area


def convex_hull_perimeter_ratio(contour):
    """凸包周长比 = 轮廓周长 / 凸包周长"""
    hull = cv2.convexHull(contour)
    cnt_peri = cv2.arcLength(contour, True)
    hull_peri = cv2.arcLength(hull, True)
    if hull_peri == 0:
        return 0.0
    return cnt_peri / hull_peri


def is_convex(contour):
    """判断轮廓是否凸"""
    return cv2.isContourConvex(contour)


# ======================== 凸缺陷检测 ========================

def convexity_defects(contour, return_pts=True):
    """
    凸缺陷检测
    :param contour: 轮廓
    :param return_pts: 是否返回实际坐标点
    :return: 缺陷列表, 每项 {start, end, far, depth}
    """
    hull = cv2.convexHull(contour, returnPoints=False)

    if len(hull) < 3 or len(contour) < 3:
        return []

    defects = cv2.convexityDefects(contour, hull)
    if defects is None:
        return []

    defect_list = []
    for i in range(defects.shape[0]):
        s, e, f, d = defects[i, 0]
        start = tuple(contour[s][0])
        end = tuple(contour[e][0])
        far = tuple(contour[f][0])
        depth = d / 256.0  # 深度值

        defect_list.append({
            'start_idx': s, 'end_idx': e, 'far_idx': f,
            'start': start, 'end': end, 'far': far,
            'depth': depth
        })

    return defect_list


def filter_defects_by_depth(defects, min_depth=10):
    """按深度过滤凸缺陷"""
    return [d for d in defects if d['depth'] >= min_depth]


def filter_defects_by_angle(defects, contour, min_angle_deg=20, max_angle_deg=160):
    """按角度过滤凸缺陷 (在far点处的角度)"""
    filtered = []
    for d in defects:
        s = np.array(d['start'])
        e = np.array(d['end'])
        f = np.array(d['far'])
        fs = s - f
        fe = e - f
        cos_angle = np.dot(fs, fe) / (np.linalg.norm(fs) * np.linalg.norm(fe) + 1e-6)
        angle = np.degrees(np.arccos(np.clip(cos_angle, -1, 1)))
        if min_angle_deg <= angle <= max_angle_deg:
            d['angle'] = angle
            filtered.append(d)
    return filtered


def count_fingers(contour, min_depth=20, min_angle=20, max_angle=160):
    """
    简易手指计数 (基于凸缺陷)
    :return: 手指数, 有效缺陷列表
    """
    defects = convexity_defects(contour)
    defects = filter_defects_by_depth(defects, min_depth)
    defects = filter_defects_by_angle(defects, contour, min_angle, max_angle)
    return len(defects), defects


# ======================== 可视化 ========================

def draw_hull_and_defects(img, contour, defects=None, hull_color=(0, 255, 0),
                          defect_color=(0, 0, 255)):
    """绘制凸包和凸缺陷"""
    result = img.copy()

    # 绘制轮廓
    cv2.drawContours(result, [contour], -1, (255, 255, 0), 2)

    # 绘制凸包
    hull = cv2.convexHull(contour)
    cv2.drawContours(result, [hull], -1, hull_color, 2)

    # 绘制缺陷
    if defects:
        for d in defects:
            cv2.circle(result, d['far'], 5, defect_color, -1)
            cv2.line(result, d['start'], d['far'], defect_color, 1)
            cv2.line(result, d['end'], d['far'], defect_color, 1)

    return result


# ======================== 使用示例 ========================
if __name__ == '__main__':
    img = cv2.imread('test.jpg', cv2.IMREAD_GRAYSCALE)
    _, binary = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        cnt = max(contours, key=cv2.contourArea)

        # 凸包
        hull = compute_convex_hull(cnt)
        solidity = convex_hull_area_ratio(cnt)
        print(f'实心度: {solidity:.3f}')
        print(f'是否凸: {is_convex(cnt)}')

        # 凸缺陷
        defects = convexity_defects(cnt)
        valid = filter_defects_by_depth(defects, min_depth=15)
        print(f'凸缺陷总数: {len(defects)}, 有效: {len(valid)}')
