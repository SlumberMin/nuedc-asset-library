"""
轮廓操作通用工具库
支持: 查找 / 绘制 / 近似 / 匹配 / 凸包 / 形状特征
"""
import cv2
import numpy as np


def find_contours(img, mode='external', method='simple'):
    """查找轮廓
    mode: external(仅外层), tree(全部层级), ccomp(两层), list(所有)
    method: simple, approx_tc89_l1, approx_tc89_kcos, approx_none
    """
    if len(img.shape) == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img.copy()
    _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    mode_map = {
        'external': cv2.RETR_EXTERNAL, 'tree': cv2.RETR_TREE,
        'ccomp': cv2.RETR_CCOMP, 'list': cv2.RETR_LIST,
    }
    method_map = {
        'simple': cv2.CHAIN_APPROX_SIMPLE, 'approx_none': cv2.CHAIN_APPROX_NONE,
        'approx_tc89_l1': cv2.CHAIN_APPROX_TC89_L1,
        'approx_tc89_kcos': cv2.CHAIN_APPROX_TC89_KCOS,
    }
    contours, hierarchy = cv2.findContours(binary, mode_map.get(mode, cv2.RETR_EXTERNAL),
                                            method_map.get(method, cv2.CHAIN_APPROX_SIMPLE))
    return contours, hierarchy


def draw_contours(img, contours, color=(0, 255, 0), thickness=2, contour_idx=-1):
    """绘制轮廓"""
    canvas = img.copy()
    return cv2.drawContours(canvas, contours, contour_idx, color, thickness)


def draw_contour_info(img, contours, min_area=100):
    """绘制轮廓及标注面积、中心"""
    canvas = img.copy()
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area:
            continue
        M = cv2.moments(cnt)
        if M['m00'] == 0:
            continue
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        cv2.drawContours(canvas, [cnt], -1, (0, 255, 0), 2)
        cv2.putText(canvas, f"A:{int(area)}", (cx - 30, cy - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv2.circle(canvas, (cx, cy), 4, (255, 0, 0), -1)
    return canvas


def approx_contour(contour, epsilon_ratio=0.02):
    """多边形近似轮廓"""
    peri = cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, epsilon_ratio * peri, True)
    return approx


def convex_hull(contour):
    """凸包"""
    return cv2.convexHull(contour)


def convex_hull_img(img, contours):
    """绘制所有轮廓的凸包"""
    canvas = img.copy()
    for cnt in contours:
        hull = cv2.convexHull(cnt)
        cv2.drawContours(canvas, [hull], -1, (0, 255, 0), 2)
    return canvas


def convexity_defects(contour):
    """凸包缺陷 (检测凹陷区域)"""
    hull = cv2.convexHull(contour, returnPoints=False)
    defects = cv2.convexityDefects(contour, hull)
    return defects


def match_shapes(contour1, contour2, method='i'):
    """形状匹配 (Hu矩)
    method: i(相关), c(卡方), e(欧式)
    """
    method_map = {'i': cv2.CONTOURS_MATCH_I1, 'c': cv2.CONTOURS_MATCH_I2, 'e': cv2.CONTOURS_MATCH_I3}
    return cv2.matchShapes(contour1, contour2, method_map.get(method, cv2.CONTOURS_MATCH_I1), 0)


def match_shape_template(contours, template_contour, method='i'):
    """在一组轮廓中找与模板最匹配的"""
    best_score = float('inf')
    best_idx = -1
    for i, cnt in enumerate(contours):
        score = match_shapes(cnt, template_contour, method)
        if score < best_score:
            best_score = score
            best_idx = i
    return best_idx, best_score


def contour_features(contour):
    """提取单个轮廓的几何特征"""
    area = cv2.contourArea(contour)
    perimeter = cv2.arcLength(contour, True)
    M = cv2.moments(contour)
    cx = int(M['m10'] / M['m00']) if M['m00'] != 0 else 0
    cy = int(M['m01'] / M['m00']) if M['m00'] != 0 else 0
    x, y, w, h = cv2.boundingRect(contour)
    aspect_ratio = float(w) / h if h > 0 else 0
    rect_area = w * h
    extent = float(area) / rect_area if rect_area > 0 else 0
    hull = cv2.convexHull(contour)
    hull_area = cv2.contourArea(hull)
    solidity = float(area) / hull_area if hull_area > 0 else 0
    (cx_c, cy_c), radius = cv2.minEnclosingCircle(contour)
    equi_diameter = np.sqrt(4 * area / np.pi) if area > 0 else 0
    if len(contour) >= 5:
        (cx_e, cy_e), (ma, MA), angle = cv2.fitEllipse(contour)
    else:
        (cx_e, cy_e, ma, MA, angle) = (0, 0, 0, 0, 0)
    return {
        'area': area, 'perimeter': perimeter, 'center': (cx, cy),
        'bounding_rect': (x, y, w, h), 'aspect_ratio': aspect_ratio,
        'extent': extent, 'solidity': solidity,
        'min_enclosing_radius': radius, 'equivalent_diameter': equi_diameter,
        'fit_ellipse': (cx_e, cy_e, ma, MA, angle),
    }


def filter_contours(contours, min_area=100, max_area=float('inf'),
                     min_solidity=0.0, max_solidity=1.0,
                     min_aspect=0.0, max_aspect=float('inf')):
    """按几何条件过滤轮廓"""
    result = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area or area > max_area:
            continue
        hull = cv2.convexHull(cnt)
        hull_area = cv2.contourArea(hull)
        solidity = float(area) / hull_area if hull_area > 0 else 0
        if solidity < min_solidity or solidity > max_solidity:
            continue
        x, y, w, h = cv2.boundingRect(cnt)
        ar = float(w) / h if h > 0 else 0
        if ar < min_aspect or ar > max_aspect:
            continue
        result.append(cnt)
    return result


def classify_shape(contour):
    """根据多边形近似判断形状名称"""
    approx = approx_contour(contour, 0.04)
    vertices = len(approx)
    if vertices == 3:
        return "triangle"
    elif vertices == 4:
        x, y, w, h = cv2.boundingRect(approx)
        ar = float(w) / h
        return "square" if 0.85 <= ar <= 1.15 else "rectangle"
    elif vertices == 5:
        return "pentagon"
    elif vertices == 6:
        return "hexagon"
    elif vertices > 6:
        return "circle"
    return "unknown"
