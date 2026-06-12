"""
图像分割模块 - 阈值分割 + 区域生长 + 分水岭
适用场景: 电赛中目标与背景分离、多目标提取
"""

import cv2
import numpy as np
from collections import deque


# ======================== 阈值分割 ========================

def threshold_binary(gray, thresh=127, max_val=255):
    """二值化分割"""
    _, mask = cv2.threshold(gray, thresh, max_val, cv2.THRESH_BINARY)
    return mask


def threshold_otsu(gray):
    """Otsu自动阈值分割"""
    thresh_val, mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    return mask, thresh_val


def threshold_adaptive(gray, block_size=11, C=2):
    """自适应阈值分割 (光照不均场景)"""
    mask = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                 cv2.THRESH_BINARY, block_size, C)
    return mask


def threshold_color_range(hsv, lower, upper):
    """HSV颜色范围分割"""
    mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
    return mask


def threshold_multi_channel(img, channel='h', thresh_range=(0, 180)):
    """单通道范围分割"""
    ch_map = {'h': 0, 's': 1, 'v': 2, 'b': 0, 'g': 1, 'r': 2}
    if len(img.shape) == 3:
        channel_idx = ch_map.get(channel, 0)
        ch = img[:, :, channel_idx]
    else:
        ch = img
    mask = cv2.inRange(ch, thresh_range[0], thresh_range[1])
    return mask


# ======================== 区域生长 ========================

def region_growing(gray, seed, thresh_diff=10):
    """
    区域生长分割
    :param gray: 灰度图
    :param seed: 种子点 (x, y)
    :param thresh_diff: 灰度差阈值
    :return: 分割mask
    """
    h, w = gray.shape
    mask = np.zeros((h, w), dtype=np.uint8)
    visited = np.zeros((h, w), dtype=bool)
    seed_val = int(gray[seed[1], seed[0]])

    queue = deque([seed])
    visited[seed[1], seed[0]] = True
    mask[seed[1], seed[0]] = 255

    neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    while queue:
        x, y = queue.popleft()
        for dx, dy in neighbors:
            nx, ny = x + dx, y + dy
            if 0 <= nx < w and 0 <= ny < h and not visited[ny, nx]:
                visited[ny, nx] = True
                if abs(int(gray[ny, nx]) - seed_val) <= thresh_diff:
                    mask[ny, nx] = 255
                    queue.append((nx, ny))
    return mask


def region_growing_multi_seed(gray, seeds, thresh_diff=10):
    """多种子点区域生长"""
    mask = np.zeros_like(gray)
    for seed in seeds:
        m = region_growing(gray, seed, thresh_diff)
        mask = cv2.bitwise_or(mask, m)
    return mask


# ======================== 分水岭 ========================

def watershed_segmentation(img, mask=None, bg_dilate_iter=1, fg_erode_iter=3):
    """
    分水岭分割
    :param img: BGR彩色图
    :param mask: 预处理后的二值mask (若无则自动Otsu)
    :return: labels, markers, result_img
    """
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if len(img.shape) == 3 else img

    if mask is None:
        _, mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    # 去噪
    denoised = cv2.medianBlur(mask, 5)

    # 形态学: 确定背景
    kernel = np.ones((3, 3), np.uint8)
    bg = cv2.dilate(denoised, kernel, iterations=bg_dilate_iter)

    # 确定前景
    dist = cv2.distanceTransform(denoised, cv2.DIST_L2, 5)
    _, fg = cv2.threshold(dist, 0.5 * dist.max(), 255, cv2.THRESH_BINARY)
    fg = fg.astype(np.uint8)
    fg = cv2.erode(fg, kernel, iterations=fg_erode_iter)

    # 未知区域
    unknown = cv2.subtract(bg, fg)

    # 标记
    num_labels, markers = cv2.connectedComponents(fg)
    markers = markers + 1
    markers[unknown == 255] = 0

    # 分水岭
    img_ws = img.copy() if len(img.shape) == 3 else cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    markers = cv2.watershed(img_ws, markers)

    # 可视化
    result = img_ws.copy()
    result[markers == -1] = [0, 0, 255]  # 边界标红

    return markers, result


# ======================== 使用示例 ========================
if __name__ == '__main__':
    img = cv2.imread('test.jpg')
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Otsu分割
    otsu_mask, thresh = threshold_otsu(gray)
    print(f'Otsu阈值: {thresh}')

    # 区域生长
    h, w = gray.shape
    rg_mask = region_growing(gray, (w // 2, h // 2), thresh_diff=15)

    # 分水岭
    markers, ws_result = watershed_segmentation(img)
    print(f'分水岭分割区域数: {markers.max()}')
