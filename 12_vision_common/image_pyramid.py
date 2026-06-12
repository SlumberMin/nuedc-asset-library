"""
图像金字塔模块 - 高斯金字塔、拉普拉斯金字塔、DOG金字塔
适用场景：多尺度特征提取、图像融合、图像压缩、目标检测
"""

import cv2
import numpy as np


def gaussian_pyramid(image, levels=5):
    """
    构建高斯金字塔
    :param image: 输入图像 (灰度或彩色)
    :param levels: 金字塔层数
    :return: 金字塔各层列表 [原始层, 缩小1层, ..., 缩小n层]
    """
    pyramid = [image.copy()]
    current = image.copy()
    for i in range(levels - 1):
        current = cv2.pyrDown(current)
        pyramid.append(current)
    return pyramid


def laplacian_pyramid(image, levels=5):
    """
    构建拉普拉斯金字塔 (用于图像融合和增强)
    :param image: 输入图像
    :param levels: 金字塔层数
    :return: 拉普拉斯金字塔各层列表
    """
    gauss_pyr = gaussian_pyramid(image, levels)
    laplacian_pyr = [gauss_pyr[-1]]  # 最高层(最小)直接作为拉普拉斯层

    for i in range(levels - 2, -1, -1):
        expanded = cv2.pyrUp(gauss_pyr[i + 1], dstsize=(gauss_pyr[i].shape[1], gauss_pyr[i].shape[0]))
        lap = cv2.subtract(gauss_pyr[i], expanded)
        laplacian_pyr.append(lap)

    laplacian_pyr.reverse()  # 从原始分辨率到最小
    return laplacian_pyr


def reconstruct_from_laplacian(lap_pyr):
    """
    从拉普拉斯金字塔重建图像
    :param lap_pyr: 拉普拉斯金字塔
    :return: 重建的图像
    """
    current = lap_pyr[-1]
    for i in range(len(lap_pyr) - 2, -1, -1):
        expanded = cv2.pyrUp(current, dstsize=(lap_pyr[i].shape[1], lap_pyr[i].shape[0]))
        current = cv2.add(expanded, lap_pyr[i])
    return current


def dog_pyramid(image, levels=5, sigma1=1.0, sigma2=1.6):
    """
    构建DOG(Difference of Gaussian)金字塔
    :param image: 输入图像(灰度)
    :param levels: 层数
    :param sigma1: 第一个高斯核sigma
    :param sigma2: 第二个高斯核sigma
    :return: DOG金字塔各层列表
    """
    if len(image.shape) == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image.copy()

    gray = gray.astype(np.float64)
    dog_pyr = []
    current = gray.copy()
    k = sigma2 / sigma1

    for i in range(levels - 1):
        blur1 = cv2.GaussianBlur(current, (0, 0), sigma1)
        blur2 = cv2.GaussianBlur(current, (0, 0), sigma2)
        dog = blur1 - blur2
        dog_pyr.append(dog)
        current = cv2.pyrDown(current)
        sigma1 *= k
        sigma2 *= k

    return dog_pyr


def visualize_pyramid(pyramid, title="Pyramid"):
    """
    拼接金字塔各层为一张图用于可视化
    :param pyramid: 金字塔列表
    :param title: 窗口标题
    :return: 拼接后的可视化图像
    """
    # 获取原始层的尺寸
    h0, w0 = pyramid[0].shape[:2]

    # 左侧放置各层(右对齐)
    vis_h = h0
    vis_w = w0 + sum(p.shape[1] for p in pyramid[1:])

    if len(pyramid[0].shape) == 3:
        canvas = np.zeros((vis_h, vis_w, 3), dtype=pyramid[0].dtype)
    else:
        canvas = np.zeros((vis_h, vis_w), dtype=pyramid[0].dtype)

    # 放置第一层(左侧)
    canvas[0:h0, 0:w0] = pyramid[0]

    # 放置后续层(从上对齐，向右排列)
    x_offset = w0
    for p in pyramid[1:]:
        h, w = p.shape[:2]
        if len(p.shape) == 3:
            canvas[0:h, x_offset:x_offset + w] = p
        else:
            canvas[0:h, x_offset:x_offset + w] = p
        x_offset += w

    return canvas


# ===================== 示例与测试 =====================
if __name__ == "__main__":
    import sys

    img_path = sys.argv[1] if len(sys.argv) > 1 else "test.jpg"
    img = cv2.imread(img_path)
    if img is None:
        print(f"无法读取图像: {img_path}，生成测试图像...")
        img = np.random.randint(0, 255, (256, 256, 3), dtype=np.uint8)

    levels = 5

    # 高斯金字塔
    gauss_pyr = gaussian_pyramid(img, levels)
    vis_gauss = visualize_pyramid(gauss_pyr, "Gaussian")
    cv2.imshow("Gaussian Pyramid", vis_gauss)

    # 拉普拉斯金字塔
    lap_pyr = laplacian_pyramid(img, levels)
    lap_vis = []
    for p in lap_pyr:
        if p.dtype != np.uint8:
            p = cv2.normalize(p, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        lap_vis.append(p)
    vis_lap = visualize_pyramid(lap_vis, "Laplacian")
    cv2.imshow("Laplacian Pyramid", vis_lap)

    # 重建验证
    recon = reconstruct_from_laplacian(lap_pyr)
    recon = np.clip(recon, 0, 255).astype(np.uint8) if recon.dtype != np.uint8 else recon
    cv2.imshow("Reconstructed", recon)

    # DOG金字塔
    dog_pyr = dog_pyramid(img, levels)
    dog_vis = []
    for d in dog_pyr:
        d = cv2.normalize(d, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        dog_vis.append(d)
    vis_dog = visualize_pyramid(dog_vis, "DOG")
    cv2.imshow("DOG Pyramid", vis_dog)

    print("按任意键关闭窗口...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()
