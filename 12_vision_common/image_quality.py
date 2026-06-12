"""
图像质量评估工具 - PSNR/SSIM/NIQE/清晰度评价
"""
import cv2
import numpy as np


def psnr(img1, img2):
    """峰值信噪比 (越高越好, 一般>30dB为好)
    Args:
        img1, img2: 两幅同尺寸图像
    Returns:
        PSNR值 (dB), float
    """
    img1 = img1.astype(np.float64)
    img2 = img2.astype(np.float64)
    mse = np.mean((img1 - img2) ** 2)
    if mse == 0:
        return float('inf')
    return 10 * np.log10(255.0 ** 2 / mse)


def ssim(img1, img2, k1=0.01, k2=0.03, L=255):
    """结构相似性指数 (越高越好, 范围[-1,1])
    Args:
        img1, img2: 灰度图或BGR图
    Returns:
        SSIM值, float; 若为彩色图返回平均SSIM
    """
    def _ssim_single(c1, c2):
        c1 = c1.astype(np.float64)
        c2 = c2.astype(np.float64)
        mu1 = cv2.GaussianBlur(c1, (11, 11), 1.5)
        mu2 = cv2.GaussianBlur(c2, (11, 11), 1.5)
        mu1_sq = mu1 ** 2
        mu2_sq = mu2 ** 2
        mu1_mu2 = mu1 * mu2
        sigma1_sq = cv2.GaussianBlur(c1 ** 2, (11, 11), 1.5) - mu1_sq
        sigma2_sq = cv2.GaussianBlur(c2 ** 2, (11, 11), 1.5) - mu2_sq
        sigma12 = cv2.GaussianBlur(c1 * c2, (11, 11), 1.5) - mu1_mu2

        c1_k = (k1 * L) ** 2
        c2_k = (k2 * L) ** 2
        num = (2 * mu1_mu2 + c1_k) * (2 * sigma12 + c2_k)
        den = (mu1_sq + mu2_sq + c1_k) * (sigma1_sq + sigma2_sq + c2_k)
        ssim_map = num / den
        return np.mean(ssim_map)

    if len(img1.shape) == 3:
        vals = [_ssim_single(img1[:, :, c], img2[:, :, c]) for c in range(3)]
        return np.mean(vals)
    return _ssim_single(img1, img2)


def niqe(img):
    """自然图像质量评估器 (无参考, 越低越好)
    基于自然场景统计 (NSS) 的简化实现
    Args:
        img: 灰度图
    Returns:
        NIQE分数, float
    """
    gray = img if len(img.shape) == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = gray.astype(np.float64)

    # 提取MSCN系数
    mu = cv2.GaussianBlur(gray, (7, 7), 7.0 / 6.0)
    mu_sq = mu * mu
    sigma = np.sqrt(np.abs(cv2.GaussianBlur(gray ** 2, (7, 7), 7.0 / 6.0) - mu_sq))
    mscn = (gray - mu) / (sigma + 1)

    # 提取特征: 均值、方差、偏度、峰度
    features = []
    features.append(np.mean(mscn))
    features.append(np.std(mscn))
    features.append(np.mean(mscn ** 3) / (np.std(mscn) ** 3 + 1e-10))
    features.append(np.mean(mscn ** 4) / (np.std(mscn) ** 4 + 1e-10))

    # 邻域对特征
    shifts = [(0, 1), (1, 0), (1, 1), (1, -1)]
    for dy, dx in shifts:
        shifted = np.roll(np.roll(mscn, dy, axis=0), dx, axis=1)
        pair = mscn * shifted
        features.append(np.mean(pair))
        features.append(np.std(pair))

    # 简化评分: 基于特征与自然图像统计先验的马氏距离
    # 这里用特征向量的范数作为简化指标
    feat = np.array(features)
    return float(np.sqrt(np.sum(feat ** 2)))


def sharpness_laplacian(img):
    """拉普拉斯方差法评估清晰度 (越大越清晰)
    Args:
        img: 灰度图
    Returns:
        清晰度分数, float
    """
    gray = img if len(img.shape) == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    laplacian = cv2.Laplacian(gray, cv2.CV_64F)
    return float(np.var(laplacian))


def sharpness_tenengrad(img, ksize=3):
    """Tenengrad梯度法评估清晰度 (越大越清晰)
    """
    gray = img if len(img.shape) == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=ksize)
    gy = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=ksize)
    return float(np.mean(gx ** 2 + gy ** 2))


def noise_estimate(img):
    """估计图像噪声水平 (高斯噪声标准差)
    使用拉普拉斯算子的鲁棒估计
    Args:
        img: 灰度图
    Returns:
        噪声标准差估计值, float
    """
    gray = img if len(img.shape) == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    H, W = gray.shape
    # 使用2x2块估计
    sigma = 1.4826 * np.median(np.abs(
        gray[:H-1, :W-1].astype(float) + gray[1:, 1:].astype(float) -
        gray[:H-1, 1:].astype(float) - gray[1:, :W-1].astype(float)
    ))
    return float(sigma / np.sqrt(2))


def contrast_metric(img):
    """RMS对比度"""
    gray = img if len(img.shape) == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return float(np.std(gray.astype(np.float64)))


if __name__ == '__main__':
    img1 = cv2.imread('test1.jpg')
    img2 = cv2.imread('test2.jpg')
    if img1 is not None and img2 is not None:
        print(f"PSNR: {psnr(img1, img2):.2f} dB")
        print(f"SSIM: {ssim(img1, img2):.4f}")
    img = cv2.imread('test.jpg')
    if img is not None:
        print(f"清晰度(Lap): {sharpness_laplacian(img):.2f}")
        print(f"清晰度(Ten): {sharpness_tenengrad(img):.2f}")
        print(f"噪声估计: {noise_estimate(img):.2f}")
        print(f"对比度: {contrast_metric(img):.2f}")
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        print(f"NIQE: {niqe(gray):.4f}")
