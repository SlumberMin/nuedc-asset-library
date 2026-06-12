"""
图像配准模块 - 特征点检测+匹配+仿射/透视变换
适用场景：图像拼接、运动补偿、模板对齐、多帧对齐
"""

import cv2
import numpy as np


def detect_and_match_orb(img1, img2, max_features=500, match_ratio=0.75):
    """
    ORB特征点检测与匹配 (速度快，适合实时场景)
    :param img1: 参考图像
    :param img2: 待配准图像
    :param max_features: 最大特征点数
    :param match_ratio: Lowe比率测试阈值
    :return: (good_matches, kp1, kp2, des1, des2)
    """
    gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY) if len(img1.shape) == 3 else img1
    gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY) if len(img2.shape) == 3 else img2

    orb = cv2.ORB_create(max_features)
    kp1, des1 = orb.detectAndCompute(gray1, None)
    kp2, des2 = orb.detectAndCompute(gray2, None)

    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
    matches = bf.knnMatch(des1, des2, k=2)

    good_matches = []
    for m, n in matches:
        if m.distance < match_ratio * n.distance:
            good_matches.append(m)

    return good_matches, kp1, kp2, des1, des2


def detect_and_match_sift(img1, img2, match_ratio=0.75):
    """
    SIFT特征点检测与匹配 (精度高，适合精细配准)
    :param img1: 参考图像
    :param img2: 待配准图像
    :param match_ratio: Lowe比率测试阈值
    :return: (good_matches, kp1, kp2)
    """
    gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY) if len(img1.shape) == 3 else img1
    gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY) if len(img2.shape) == 3 else img2

    sift = cv2.SIFT_create()
    kp1, des1 = sift.detectAndCompute(gray1, None)
    kp2, des2 = sift.detectAndCompute(gray2, None)

    bf = cv2.BFMatcher()
    matches = bf.knnMatch(des1, des2, k=2)

    good_matches = []
    for m, n in matches:
        if m.distance < match_ratio * n.distance:
            good_matches.append(m)

    return good_matches, kp1, kp2


def estimate_affine_transform(img1, img2, method="orb", min_matches=10):
    """
    估计仿射变换矩阵 (平移+旋转+缩放，保平行线)
    :param img1: 参考图像
    :param img2: 待配准图像
    :param method: 特征检测方法 "orb" 或 "sift"
    :param min_matches: 最少匹配点数
    :return: (M_2x3, warped_img, match_count) 或 (None, None, 0)
    """
    if method == "orb":
        good_matches, kp1, kp2, _, _ = detect_and_match_orb(img1, img2)
    else:
        good_matches, kp1, kp2 = detect_and_match_sift(img1, img2)

    if len(good_matches) < min_matches:
        print(f"匹配点不足: {len(good_matches)}/{min_matches}")
        return None, None, len(good_matches)

    pts1 = np.float32([kp1[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
    pts2 = np.float32([kp2[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

    M, inliers = cv2.estimateAffinePartial2D(pts2, pts1, method=cv2.RANSAC)

    if M is None:
        return None, None, len(good_matches)

    h, w = img1.shape[:2]
    warped = cv2.warpAffine(img2, M, (w, h))
    return M, warped, len(good_matches)


def estimate_homography(img1, img2, method="orb", min_matches=10):
    """
    估计透视变换矩阵 (允许投影变形，用于图像拼接)
    :param img1: 参考图像
    :param img2: 待配准图像
    :param method: "orb" 或 "sift"
    :param min_matches: 最少匹配点数
    :return: (H_3x3, warped_img, match_count)
    """
    if method == "orb":
        good_matches, kp1, kp2, _, _ = detect_and_match_orb(img1, img2)
    else:
        good_matches, kp1, kp2 = detect_and_match_sift(img1, img2)

    if len(good_matches) < min_matches:
        print(f"匹配点不足: {len(good_matches)}/{min_matches}")
        return None, None, len(good_matches)

    pts1 = np.float32([kp1[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
    pts2 = np.float32([kp2[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

    H, mask = cv2.findHomography(pts2, pts1, cv2.RANSAC, 5.0)

    if H is None:
        return None, None, len(good_matches)

    h, w = img1.shape[:2]
    warped = cv2.warpPerspective(img2, H, (w, h))
    return H, warped, len(good_matches)


def draw_matches(img1, kp1, img2, kp2, matches, max_draw=50):
    """
    绘制特征匹配结果
    :return: 匹配可视化图像
    """
    draw_kp = matches[:max_draw]
    vis = cv2.drawMatches(img1, kp1, img2, kp2, draw_kp, None,
                          flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    return vis


def align_images_simple(img1, img2, warp_mode=cv2.MOTION_EUCLIDEAN):
    """
    基于ECC的图像对齐 (无需特征点，适合小位移)
    :param img1: 参考图像
    :param img2: 待对齐图像
    :param warp_mode: 变换模式
    :return: (warped_image, warp_matrix)
    """
    gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY).astype(np.float32) if len(img1.shape) == 3 else img1.astype(np.float32)
    gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY).astype(np.float32) if len(img2.shape) == 3 else img2.astype(np.float32)

    if warp_mode == cv2.MOTION_HOMOGRAPHY:
        warp_matrix = np.eye(3, 3, dtype=np.float32)
    else:
        warp_matrix = np.eye(2, 3, dtype=np.float32)

    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 50, 1e-5)
    _, warp_matrix = cv2.findTransformECC(gray1, gray2, warp_matrix, warp_mode, criteria)

    h, w = img1.shape[:2]
    if warp_mode == cv2.MOTION_HOMOGRAPHY:
        warped = cv2.warpPerspective(img2, warp_matrix, (w, h))
    else:
        warped = cv2.warpAffine(img2, warp_matrix, (w, h))

    return warped, warp_matrix


# ===================== 示例与测试 =====================
if __name__ == "__main__":
    import sys

    if len(sys.argv) < 3:
        print("用法: python image_registration.py <参考图> <待配准图>")
        print("生成测试数据...")
        base = np.zeros((300, 400, 3), dtype=np.uint8)
        cv2.rectangle(base, (50, 50), (200, 200), (0, 255, 0), -1)
        cv2.circle(base, (300, 150), 60, (0, 0, 255), -1)
        img1 = base.copy()
        M_shift = np.float32([[1, 0, 30], [0, 1, 20]])
        img2 = cv2.warpAffine(base, M_shift, (400, 300))
    else:
        img1 = cv2.imread(sys.argv[1])
        img2 = cv2.imread(sys.argv[2])

    # 仿射变换配准
    M, warped_affine, n = estimate_affine_transform(img1, img2, method="orb")
    print(f"仿射配准: {n} 个匹配点")

    # 透视变换配准
    H, warped_persp, n = estimate_homography(img1, img2, method="orb")
    print(f"透视配准: {n} 个匹配点")

    if warped_affine is not None:
        diff = cv2.absdiff(img1, warped_affine)
        cv2.imshow("Affine Registration Result", warped_affine)
        cv2.imshow("Affine Diff", diff)

    if warped_persp is not None:
        cv2.imshow("Homography Result", warped_persp)

    print("按任意键关闭...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()
