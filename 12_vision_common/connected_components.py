"""
连通域分析模块 - 标记 + 统计 + 过滤
适用场景: 电赛中目标计数、区域筛选、形状统计
"""

import cv2
import numpy as np


# ======================== 连通域标记 ========================

def label_components(binary, connectivity=8):
    """
    连通域标记
    :param binary: 二值图 (0/255)
    :param connectivity: 4或8连通
    :return: num_labels, labels, stats, centroids
    """
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
        binary, connectivity, cv2.CV_32S)
    return num_labels, labels, stats, centroids


def label_components_visual(binary, connectivity=8):
    """连通域标记 + 彩色可视化"""
    num_labels, labels, stats, centroids = label_components(binary, connectivity)

    h, w = binary.shape
    color_map = np.zeros((h, w, 3), dtype=np.uint8)
    np.random.seed(42)
    colors = np.random.randint(50, 255, size=(num_labels, 3), dtype=np.uint8)
    colors[0] = [0, 0, 0]  # 背景黑色

    for i in range(num_labels):
        color_map[labels == i] = colors[i]

    return color_map, num_labels, labels, stats, centroids


# ======================== 连通域统计 ========================

def get_component_stats(stats, centroids, num_labels):
    """提取各连通域统计信息 (排除背景label=0)"""
    components = []
    for i in range(1, num_labels):  # 跳过背景
        x, y, w, h, area = stats[i]
        cx, cy = centroids[i]
        components.append({
            'label': i,
            'bbox': (x, y, w, h),
            'area': int(area),
            'center': (float(cx), float(cy)),
            'aspect_ratio': w / h if h > 0 else 0
        })
    return components


def get_component_contours(labels, num_labels):
    """获取各连通域的轮廓"""
    contours_dict = {}
    for i in range(1, num_labels):
        mask = np.uint8(labels == i) * 255
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            contours_dict[i] = contours[0]
    return contours_dict


# ======================== 连通域过滤 ========================

def filter_by_area(stats, num_labels, min_area=100, max_area=float('inf')):
    """按面积过滤连通域, 返回合格label列表"""
    valid = []
    for i in range(1, num_labels):
        area = stats[i, cv2.CC_STAT_AREA]
        if min_area <= area <= max_area:
            valid.append(i)
    return valid


def filter_by_bbox(stats, num_labels, min_w=0, min_h=0, max_w=float('inf'), max_h=float('inf')):
    """按包围框尺寸过滤"""
    valid = []
    for i in range(1, num_labels):
        w = stats[i, cv2.CC_STAT_WIDTH]
        h = stats[i, cv2.CC_STAT_HEIGHT]
        if min_w <= w <= max_w and min_h <= h <= max_h:
            valid.append(i)
    return valid


def filter_by_aspect_ratio(stats, num_labels, min_ratio=0.0, max_ratio=float('inf')):
    """按宽高比过滤"""
    valid = []
    for i in range(1, num_labels):
        w = stats[i, cv2.CC_STAT_WIDTH]
        h = stats[i, cv2.CC_STAT_HEIGHT]
        ratio = w / h if h > 0 else 0
        if min_ratio <= ratio <= max_ratio:
            valid.append(i)
    return valid


def filter_by_solidity(labels, num_labels, min_solidity=0.5):
    """按实心度过滤 (面积/凸包面积)"""
    valid = []
    for i in range(1, num_labels):
        mask = np.uint8(labels == i) * 255
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            continue
        cnt = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(cnt)
        hull = cv2.convexHull(cnt)
        hull_area = cv2.contourArea(hull)
        if hull_area > 0:
            solidity = area / hull_area
            if solidity >= min_solidity:
                valid.append(i)
    return valid


def filter_components(binary, min_area=100, max_area=float('inf'),
                      min_w=0, min_h=0, min_ratio=0.0, max_ratio=float('inf')):
    """综合过滤: 面积 + 尺寸 + 宽高比"""
    num_labels, labels, stats, centroids = label_components(binary)
    valid = []
    for i in range(1, num_labels):
        area = stats[i, cv2.CC_STAT_AREA]
        w = stats[i, cv2.CC_STAT_WIDTH]
        h = stats[i, cv2.CC_STAT_HEIGHT]
        ratio = w / h if h > 0 else 0
        if (min_area <= area <= max_area and
                w >= min_w and h >= min_h and
                min_ratio <= ratio <= max_ratio):
            valid.append(i)

    # 生成过滤后的二值图
    mask = np.isin(labels, valid).astype(np.uint8) * 255
    return mask, valid, num_labels, stats, centroids


# ======================== 使用示例 ========================
if __name__ == '__main__':
    img = cv2.imread('test.jpg', cv2.IMREAD_GRAYSCALE)
    _, binary = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    # 连通域标记
    num_labels, labels, stats, centroids = label_components(binary)
    print(f'连通域数量 (含背景): {num_labels}')

    # 统计
    components = get_component_stats(stats, centroids, num_labels)
    for c in components:
        print(f"  标签{c['label']}: 面积={c['area']}, 中心={c['center']}, 宽高比={c['aspect_ratio']:.2f}")

    # 过滤
    filtered_mask, valid_labels, _, _, _ = filter_components(
        binary, min_area=200, min_w=10, min_h=10, min_ratio=0.3, max_ratio=3.0)
    print(f'过滤后连通域: {len(valid_labels)}')
