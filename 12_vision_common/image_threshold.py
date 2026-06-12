"""
阈值分割通用工具库
支持: 全局阈值 / 自适应阈值 / Otsu / 三角法 / 多级阈值
"""
import cv2
import numpy as np


def threshold_global(img, thresh=127, maxval=255, threshold_type='binary'):
    """全局阈值分割
    threshold_type: binary, binary_inv, trunc, tozero, tozero_inv
    """
    if len(img.shape) == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img.copy()
    type_map = {
        'binary': cv2.THRESH_BINARY,
        'binary_inv': cv2.THRESH_BINARY_INV,
        'trunc': cv2.THRESH_TRUNC,
        'tozero': cv2.THRESH_TOZERO,
        'tozero_inv': cv2.THRESH_TOZERO_INV,
    }
    flag = type_map.get(threshold_type, cv2.THRESH_BINARY)
    _, binary = cv2.threshold(gray, thresh, maxval, flag)
    return binary


def threshold_otsu(img, maxval=255):
    """Otsu自动阈值 (双峰分布最佳)"""
    if len(img.shape) == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img.copy()
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    thresh_val, binary = cv2.threshold(gray, 0, maxval, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    return binary, thresh_val


def threshold_triangle(img, maxval=255):
    """三角法自动阈值 (单峰分布适用)"""
    if len(img.shape) == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img.copy()
    thresh_val, binary = cv2.threshold(gray, 0, maxval, cv2.THRESH_BINARY + cv2.THRESH_TRIANGLE)
    return binary, thresh_val


def threshold_adaptive_mean(img, maxval=255, block_size=11, C=2):
    """自适应均值阈值"""
    if len(img.shape) == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img.copy()
    if block_size % 2 == 0:
        block_size += 1
    return cv2.adaptiveThreshold(gray, maxval, cv2.ADAPTIVE_THRESH_MEAN_C,
                                  cv2.THRESH_BINARY, block_size, C)


def threshold_adaptive_gaussian(img, maxval=255, block_size=11, C=2):
    """自适应高斯阈值"""
    if len(img.shape) == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img.copy()
    if block_size % 2 == 0:
        block_size += 1
    return cv2.adaptiveThreshold(gray, maxval, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                  cv2.THRESH_BINARY, block_size, C)


def threshold_multi_level(img, levels=3):
    """多级阈值分割 (K-means聚类)"""
    if len(img.shape) == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img.copy()
    pixel_values = gray.reshape((-1, 1)).astype(np.float32)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.2)
    _, labels, centers = cv2.kmeans(pixel_values, levels, None, criteria, 10,
                                     cv2.KMEANS_PP_CENTERS)
    centers = np.uint8(np.sort(centers, axis=0))
    segmented = centers[labels.flatten()].reshape(gray.shape)
    return segmented


def threshold_multi_otsu(img, num_classes=3):
    """多级Otsu阈值 (返回多个阈值和分割结果)"""
    from skimage.filters import threshold_multiotsu as _sk_multiotsu
    if len(img.shape) == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img.copy()
    thresholds = _sk_multiotsu(gray, classes=num_classes)
    result = np.digitize(gray, bins=thresholds) * (255 // num_classes)
    return result.astype(np.uint8), thresholds


def otsu_with_holes(img, maxval=255):
    """Otsu阈值 + 形态学闭合填充小孔"""
    if len(img.shape) == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img.copy()
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    _, binary = cv2.threshold(gray, 0, maxval, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    closed = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=2)
    return closed


def split_channels_otsu(img):
    """对BGR三通道分别做Otsu，返回三张二值图"""
    channels = cv2.split(img)
    results = []
    for ch in channels:
        ch = cv2.GaussianBlur(ch, (5, 5), 0)
        _, b = cv2.threshold(ch, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        results.append(b)
    return results
