"""
图像融合模块 - 多尺度融合(拉普拉斯金字塔) + HDR合成
适用场景：多曝光融合、多聚焦融合、全景拼接后处理
"""

import cv2
import numpy as np


def laplacian_pyramid_blend(img1, img2, mask, levels=5):
    """
    拉普拉斯金字塔融合 (经典多尺度融合方法)
    :param img1: 图像1
    :param img2: 图像2
    :param mask: 融合掩码 (0~255, 白色区域取img1)
    :param levels: 金字塔层数
    :return: 融合结果
    """
    assert img1.shape == img2.shape, "两幅图像尺寸必须一致"

    if len(mask.shape) == 2:
        mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    mask = mask.astype(np.float64) / 255.0
    img1 = img1.astype(np.float64)
    img2 = img2.astype(np.float64)

    # 构建高斯金字塔
    def build_gauss(img, lvls):
        G = [img.copy()]
        for _ in range(lvls - 1):
            img = cv2.pyrDown(img)
            G.append(img)
        return G

    # 构建拉普拉斯金字塔
    def build_laplace(img, lvls):
        G = build_gauss(img, lvls)
        L = []
        for i in range(lvls - 1):
            h, w = G[i].shape[:2]
            up = cv2.pyrUp(G[i + 1], dstsize=(w, h))
            L.append(G[i] - up)
        L.append(G[-1])  # 最低频层
        return L

    L1 = build_laplace(img1, levels)
    L2 = build_laplace(img2, levels)
    Gm = build_gauss(mask, levels)

    # 按层融合
    L_blend = []
    for l1, l2, gm in zip(L1, L2, Gm):
        blended = l1 * gm + l2 * (1.0 - gm)
        L_blend.append(blended)

    # 重建
    result = L_blend[-1]
    for i in range(levels - 2, -1, -1):
        h, w = L_blend[i].shape[:2]
        result = cv2.pyrUp(result, dstsize=(w, h))
        result += L_blend[i]

    return np.clip(result, 0, 255).astype(np.uint8)


def multi_focus_fusion(img_near, img_far, block_size=21):
    """
    多聚焦融合 (基于清晰度检测自动选择清晰区域)
    :param img_near: 近景清晰图像
    :param img_far: 远景清晰图像
    :param block_size: 清晰度评估块大小
    :return: (融合图像, 聚焦掩码)
    """
    gray1 = cv2.cvtColor(img_near, cv2.COLOR_BGR2GRAY) if len(img_near.shape) == 3 else img_near
    gray2 = cv2.cvtColor(img_far, cv2.COLOR_BGR2GRAY) if len(img_far.shape) == 3 else img_far

    # 使用拉普拉斯方差作为清晰度指标
    def laplacian_variance(gray, ksize=3):
        return cv2.Laplacian(gray, cv2.CV_64F, ksize=ksize) ** 2

    focus1 = laplacian_variance(gray1)
    focus2 = laplacian_variance(gray2)

    # 块级清晰度评估
    h, w = gray1.shape
    mask = np.zeros((h, w), dtype=np.float64)
    half = block_size // 2

    for y in range(half, h - half, block_size):
        for x in range(half, w - half, block_size):
            s1 = np.sum(focus1[y - half:y + half + 1, x - half:x + half + 1])
            s2 = np.sum(focus2[y - half:y + half + 1, x - half:x + half + 1])
            val = 255 if s1 >= s2 else 0
            mask[y - half:y + half + 1, x - half:x + half + 1] = val

    mask = mask.astype(np.uint8)
    mask = cv2.GaussianBlur(mask, (block_size | 1, block_size | 1), 0)

    # 使用金字塔融合
    result = laplacian_pyramid_blend(img_near, img_far, mask)
    return result, mask


def exposure_fusion(images, contrast_weight=1.0, saturation_weight=1.0, exposure_weight=1.0):
    """
    多曝光融合 (Mertens方法，无需HDR中间步骤)
    :param images: 不同曝光的图像列表
    :param contrast_weight: 对比度权重
    :param saturation_weight: 饱和度权重
    :param exposure_weight: 曝光权重
    :return: 融合结果
    """
    merge_mertens = cv2.createMergeMertens(
        contrastWeight=contrast_weight,
        saturationWeight=saturation_weight,
        exposureWeight=exposure_weight
    )
    result = merge_mertens.process(images)
    result = np.clip(result * 255, 0, 255).astype(np.uint8)
    return result


def hdr_tone_mapping(images, exposure_times, method="reinhard"):
    """
    HDR合成+色调映射
    :param images: 不同曝光的图像列表
    :param exposure_times: 对应曝光时间列表(秒)
    :param method: 色调映射方法 "reinhard"/"drago"/"mantiuk"/"linear"
    :return: 色调映射后的8位图像
    """
    # 校准HDR
    calibrate = cv2.createCalibrateDebevec()
    response = calibrate.process(images, np.array(exposure_times, dtype=np.float32))

    merge = cv2.createMergeDebevec()
    hdr = merge.process(images, np.array(exposure_times, dtype=np.float32), response)

    # 色调映射
    if method == "reinhard":
        tonemap = cv2.createTonemapReinhard(gamma=2.2)
    elif method == "drago":
        tonemap = cv2.createTonemapDrago(gamma=2.2)
    elif method == "mantiuk":
        tonemap = cv2.createTonemapMantiuk(gamma=2.2)
    else:
        tonemap = cv2.createTonemap(gamma=2.2)

    ldr = tonemap.process(hdr)
    result = np.clip(ldr * 255, 0, 255).astype(np.uint8)
    return result


def simple_hdr_from_bracket(images, method="reinhard"):
    """
    简化版HDR (自动估计曝光时间)
    :param images: 多曝光图像列表
    :param method: 色调映射方法
    :return: HDR色调映射结果
    """
    # 简单假设曝光时间等比递增
    n = len(images)
    exposure_times = [2 ** (i - n // 2) * 0.01 for i in range(n)]
    return hdr_tone_mapping(images, exposure_times, method)


def weighted_fusion(img1, img2, alpha=0.5):
    """
    简单加权融合
    :param alpha: img1的权重 (0~1)
    :return: 融合结果
    """
    return cv2.addWeighted(img1, alpha, img2, 1 - alpha, 0)


# ===================== 示例与测试 =====================
if __name__ == "__main__":
    import sys

    if len(sys.argv) >= 3:
        img1 = cv2.imread(sys.argv[1])
        img2 = cv2.imread(sys.argv[2])
    else:
        print("生成测试数据...")
        img1 = np.zeros((256, 256, 3), dtype=np.uint8)
        img2 = np.zeros((256, 256, 3), dtype=np.uint8)
        cv2.circle(img1, (128, 128), 80, (255, 100, 50), -1)
        cv2.rectangle(img2, (40, 40), (216, 216), (50, 100, 255), -1)

    h, w = img1.shape[:2]
    mask = np.zeros((h, w), dtype=np.uint8)
    mask[:, :w // 2] = 255  # 左半取img1，右半取img2

    # 金字塔融合
    result_pyr = laplacian_pyramid_blend(img1, img2, mask)

    # 加权融合
    result_wt = weighted_fusion(img1, img2, 0.6)

    cv2.imshow("Image1", img1)
    cv2.imshow("Image2", img2)
    cv2.imshow("Mask", mask)
    cv2.imshow("Pyramid Blend", result_pyr)
    cv2.imshow("Weighted Blend", result_wt)

    print("按任意键关闭...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()
