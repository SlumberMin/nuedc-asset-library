"""
特征检测通用工具库
支持: 角点(Harris/Shi-Tomasi) / FAST / ORB / SIFT / BRISK
"""
import cv2
import numpy as np


def detect_harris(img, block_size=2, ksize=3, k=0.04, threshold=0.01):
    """Harris角点检测"""
    if len(img.shape) == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img.copy()
    gray = np.float32(gray)
    dst = cv2.cornerHarris(gray, block_size, ksize, k)
    dst = cv2.dilate(dst, None)
    result = np.zeros_like(dst)
    result[dst > threshold * dst.max()] = 255
    return result.astype(np.uint8), dst


def detect_harris_visual(img, block_size=2, ksize=3, k=0.04, threshold=0.01):
    """Harris角点可视化 (在原图标记)"""
    if len(img.shape) == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        canvas = img.copy()
    else:
        gray = img.copy()
        canvas = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    gray_f = np.float32(gray)
    dst = cv2.cornerHarris(gray_f, block_size, ksize, k)
    dst = cv2.dilate(dst, None)
    corners = np.where(dst > threshold * dst.max())
    for y, x in zip(corners[0], corners[1]):
        cv2.circle(canvas, (x, y), 3, (0, 0, 255), -1)
    return canvas


def detect_shi_tomasi(img, max_corners=100, quality_level=0.01, min_distance=10):
    """Shi-Tomasi角点检测 (goodFeaturesToTrack)"""
    if len(img.shape) == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img.copy()
    corners = cv2.goodFeaturesToTrack(gray, max_corners, quality_level, min_distance)
    return corners


def detect_shi_tomasi_visual(img, max_corners=100, quality_level=0.01, min_distance=10):
    """Shi-Tomasi角点可视化"""
    if len(img.shape) == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        canvas = img.copy()
    else:
        gray = img.copy()
        canvas = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    corners = cv2.goodFeaturesToTrack(gray, max_corners, quality_level, min_distance)
    if corners is not None:
        for pt in corners:
            x, y = pt.ravel()
            cv2.circle(canvas, (int(x), int(y)), 5, (0, 255, 0), -1)
    return canvas


def detect_fast(img, threshold=20, nonmax_suppression=True, type=cv2.FAST_FEATURE_DETECTOR_TYPE_9_16):
    """FAST角点检测"""
    if len(img.shape) == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img.copy()
    fast = cv2.FastFeatureDetector_create(threshold=threshold,
                                           nonmaxSuppression=nonmax_suppression,
                                           type=type)
    keypoints = fast.detect(gray, None)
    return keypoints, fast


def detect_orb(img, nfeatures=500, scale_factor=1.2, nlevels=8, edge_threshold=31):
    """ORB特征检测与描述"""
    if len(img.shape) == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img.copy()
    orb = cv2.ORB_create(nfeatures=nfeatures, scaleFactor=scale_factor,
                          nlevels=nlevels, edgeThreshold=edge_threshold)
    keypoints, descriptors = orb.detectAndCompute(gray, None)
    return keypoints, descriptors


def detect_sift(img, nfeatures=0, contrast_threshold=0.04, edge_threshold=10, sigma=1.6):
    """SIFT特征检测与描述"""
    if len(img.shape) == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img.copy()
    sift = cv2.SIFT_create(nfeatures=nfeatures, contrastThreshold=contrast_threshold,
                            edgeThreshold=edge_threshold, sigma=sigma)
    keypoints, descriptors = sift.detectAndCompute(gray, None)
    return keypoints, descriptors


def detect_brisk(img, thresh=30, octaves=3, pattern_scale=1.0):
    """BRISK特征检测与描述"""
    if len(img.shape) == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img.copy()
    brisk = cv2.BRISK_create(thresh=thresh, octaves=octaves, patternScale=pattern_scale)
    keypoints, descriptors = brisk.detectAndCompute(gray, None)
    return keypoints, descriptors


def draw_keypoints(img, keypoints, color=(0, 255, 0), flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS):
    """绘制关键点"""
    return cv2.drawKeypoints(img, keypoints, None, color=color, flags=flags)


def match_features_orb(img1, img2, nfeatures=500, ratio_thresh=0.75):
    """ORB + BFMatcher 特征匹配"""
    kp1, des1 = detect_orb(img1, nfeatures=nfeatures)
    kp2, des2 = detect_orb(img2, nfeatures=nfeatures)
    if des1 is None or des2 is None:
        return [], [], []
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
    matches = bf.knnMatch(des1, des2, k=2)
    good = []
    for m_n in matches:
        if len(m_n) == 2:
            m, n = m_n
            if m.distance < ratio_thresh * n.distance:
                good.append(m)
    return kp1, kp2, good


def match_features_sift(img1, img2, nfeatures=0, ratio_thresh=0.75):
    """SIFT + FLANN特征匹配"""
    kp1, des1 = detect_sift(img1, nfeatures=nfeatures)
    kp2, des2 = detect_sift(img2, nfeatures=nfeatures)
    if des1 is None or des2 is None:
        return [], [], []
    FLANN_INDEX_KDTREE = 1
    index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
    search_params = dict(checks=50)
    flann = cv2.FlannBasedMatcher(index_params, search_params)
    matches = flann.knnMatch(des1, des2, k=2)
    good = []
    for m_n in matches:
        if len(m_n) == 2:
            m, n = m_n
            if m.distance < ratio_thresh * n.distance:
                good.append(m)
    return kp1, kp2, good


def draw_matches(img1, kp1, img2, kp2, matches, max_draw=50):
    """绘制匹配结果"""
    matches = sorted(matches, key=lambda x: x.distance)[:max_draw]
    return cv2.drawMatches(img1, kp1, img2, kp2, matches, None,
                           flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
