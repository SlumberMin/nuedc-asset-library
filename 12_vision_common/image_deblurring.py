"""
图像去模糊模块 - 运动模糊去卷积、高斯模糊恢复、自适应锐化
适用场景：运动模糊修复、对焦不准图像增强、PSF估计
"""

import cv2
import numpy as np
from numpy.fft import fft2, ifft2, fftshift, ifftshift


def estimate_psf_motion(size, angle, length):
    """
    生成运动模糊PSF(点扩散函数)
    :param size: PSF核大小 (应为奇数)
    :param angle: 运动角度(度)
    :param length: 运动长度(像素)
    :return: 归一化的PSF核
    """
    psf = np.zeros((size, size), dtype=np.float64)
    center = size // 2

    angle_rad = np.deg2rad(angle)
    cos_a = np.cos(angle_rad)
    sin_a = np.sin(angle_rad)

    for i in range(-length // 2, length // 2 + 1):
        x = int(center + i * cos_a)
        y = int(center + i * sin_a)
        if 0 <= x < size and 0 <= y < size:
            psf[y, x] = 1.0

    psf /= psf.sum() if psf.sum() > 0 else 1.0
    return psf


def create_gaussian_psf(size, sigma):
    """
    生成高斯模糊PSF
    :param size: 核大小
    :param sigma: 高斯标准差
    :return: 归一化的PSF
    """
    ax = np.arange(-size // 2 + 1, size // 2 + 1)
    xx, yy = np.meshgrid(ax, ax)
    kernel = np.exp(-(xx ** 2 + yy ** 2) / (2 * sigma ** 2))
    kernel /= kernel.sum()
    return kernel


def wiener_deconvolution(image, psf, noise_var=0.01, signal_var=1.0):
    """
    维纳去卷积 (经典频域去模糊)
    :param image: 输入模糊图像(灰度)
    :param psf: 点扩散函数
    :param noise_var: 噪声方差
    :param signal_var: 信号方差
    :return: 去模糊图像
    """
    if len(image.shape) == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY).astype(np.float64)
    else:
        gray = image.astype(np.float64)

    h, w = gray.shape
    ph, pw = psf.shape

    # 将PSF零填充到图像大小
    psf_padded = np.zeros_like(gray)
    psf_padded[:ph, :pw] = psf
    # 循环移位使PSF中心对齐
    psf_padded = np.roll(np.roll(psf_padded, -ph // 2, axis=0), -pw // 2, axis=1)

    # 频域计算
    G = fft2(gray)
    H = fft2(psf_padded)
    SNR = signal_var / noise_var

    # 维纳滤波: F_hat = (H* / (|H|^2 + 1/SNR)) * G
    H_conj = np.conj(H)
    H_abs2 = H * H_conj
    F_hat = (H_conj / (H_abs2 + 1.0 / SNR)) * G

    result = np.real(ifft2(F_hat))
    return np.clip(result, 0, 255).astype(np.uint8)


def lucy_richardson_deconvolution(image, psf, iterations=30):
    """
    Richardson-Lucy去卷积 (迭代方法，适合泊松噪声)
    :param image: 输入模糊图像(灰度)
    :param psf: 点扩散函数
    :param iterations: 迭代次数
    :return: 去模糊图像
    """
    if len(image.shape) == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY).astype(np.float64)
    else:
        gray = image.astype(np.float64)

    gray = gray / gray.max() if gray.max() > 0 else gray
    estimate = gray.copy()
    psf_mirror = psf[::-1, ::-1]

    eps = 1e-12
    for _ in range(iterations):
        convolved = cv2.filter2D(estimate, -1, psf, borderType=cv2.BORDER_REFLECT)
        convolved = np.maximum(convolved, eps)
        ratio = gray / convolved
        correction = cv2.filter2D(ratio, -1, psf_mirror, borderType=cv2.BORDER_REFLECT)
        estimate = estimate * correction

    return np.clip(estimate * 255, 0, 255).astype(np.uint8)


def deblur_with_unsharp_mask(image, sigma=1.0, strength=1.5):
    """
    反锐化掩模 (简单有效的模糊补偿)
    :param image: 输入图像
    :param sigma: 高斯模糊sigma
    :param strength: 锐化强度
    :return: 锐化结果
    """
    blurred = cv2.GaussianBlur(image, (0, 0), sigma)
    sharpened = cv2.addWeighted(image, 1.0 + strength, blurred, -strength, 0)
    return np.clip(sharpened, 0, 255).astype(np.uint8)


def deblur_bilateral_sharpen(image, d=9, sigma_color=75, sigma_space=75):
    """
    双边滤波+锐化 (去噪后锐化，保持边缘)
    :return: 去噪锐化图像
    """
    denoised = cv2.bilateralFilter(image, d, sigma_color, sigma_space)
    sharpened = cv2.addWeighted(image, 1.5, denoised, -0.5, 0)
    return np.clip(sharpened, 0, 255).astype(np.uint8)


def blind_deblur_simple(image, psf_size=15, iterations=50):
    """
    简化盲去卷积 (同时估计PSF和恢复图像)
    :param image: 模糊图像
    :param psf_size: PSF大小
    :param iterations: 迭代次数
    :return: (去模糊图像, 估计的PSF)
    """
    if len(image.shape) == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY).astype(np.float64)
    else:
        gray = image.astype(np.float64)

    gray = gray / gray.max() if gray.max() > 0 else gray

    # 初始化PSF为均匀核
    psf = np.ones((psf_size, psf_size)) / (psf_size ** 2)
    estimate = gray.copy()

    for it in range(iterations):
        # RL步：用当前PSF估计清晰图像
        convolved = cv2.filter2D(estimate, -1, psf, borderType=cv2.BORDER_REFLECT)
        convolved = np.maximum(convolved, 1e-12)
        ratio = gray / convolved
        correction = cv2.filter2D(ratio, -1, psf[::-1, ::-1], borderType=cv2.BORDER_REFLECT)
        estimate = estimate * correction
        estimate = np.clip(estimate, 0, 1)

        # 更新PSF
        conv_est = cv2.filter2D(estimate, -1, psf, borderType=cv2.BORDER_REFLECT)
        conv_est = np.maximum(conv_est, 1e-12)
        ratio2 = gray / conv_est
        psf_corr = cv2.filter2D(ratio2, -1, estimate[::-1, ::-1], borderType=cv2.BORDER_REFLECT)
        psf = psf * psf_corr[:psf_size, :psf_size]
        psf = np.maximum(psf, 0)
        psf /= psf.sum() if psf.sum() > 0 else 1.0

    return np.clip(estimate * 255, 0, 255).astype(np.uint8), psf


def simulate_motion_blur(image, length=15, angle=45):
    """
    模拟运动模糊 (用于测试)
    :param image: 清晰图像
    :param length: 模糊长度
    :param angle: 运动角度
    :return: 模糊图像
    """
    psf = estimate_psf_motion(max(length * 2 + 1, 31), angle, length)
    blurred = cv2.filter2D(image, -1, psf, borderType=cv2.BORDER_REFLECT)
    return blurred


# ===================== 示例与测试 =====================
if __name__ == "__main__":
    import sys

    img_path = sys.argv[1] if len(sys.argv) > 1 else None
    if img_path:
        img = cv2.imread(img_path)
    else:
        print("生成测试数据：创建运动模糊图像并恢复...")
        img = np.zeros((256, 256, 3), dtype=np.uint8)
        cv2.putText(img, "TEST", (30, 160), cv2.FONT_HERSHEY_SIMPLEX, 4, (255, 255, 255), 5)
        cv2.circle(img, (200, 80), 40, (0, 200, 255), -1)

    if img is None:
        print("无法读取图像")
        sys.exit(1)

    # 模拟运动模糊
    blurred = simulate_motion_blur(img, length=20, angle=30)
    cv2.imshow("Original", img)
    cv2.imshow("Motion Blurred", blurred)

    # 维纳去卷积
    psf_motion = estimate_psf_motion(31, 30, 20)
    deblur_wiener = wiener_deconvolution(blurred, psf_motion, noise_var=0.001)
    cv2.imshow("Wiener Deconv", deblur_wiener)

    # R-L去卷积
    deblur_rl = lucy_richardson_deconvolution(blurred, psf_motion, iterations=50)
    cv2.imshow("R-L Deconv", deblur_rl)

    # 锐化
    sharpened = deblur_with_unsharp_mask(blurred, sigma=2.0, strength=1.5)
    cv2.imshow("Unsharp Mask", sharpened)

    print("按任意键关闭...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()
