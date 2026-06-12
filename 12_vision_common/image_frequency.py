"""
频域处理工具 - FFT/IFFT/频域滤波/同态滤波
"""
import cv2
import numpy as np


def fft2(img):
    """二维FFT (取灰度图)
    Returns:
        f_shift: 中心化频谱 (复数)
        magnitude: 幅度谱 (对数增强, uint8)
    """
    gray = img if len(img.shape) == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    f = np.float32(gray)
    f_complex = np.fft.fft2(f)
    f_shift = np.fft.fftshift(f_complex)
    magnitude = 20 * np.log(np.abs(f_shift) + 1)
    magnitude = np.uint8(255 * magnitude / magnitude.max())
    return f_shift, magnitude


def ifft2(f_shift):
    """逆FFT还原图像
    Args:
        f_shift: 中心化频谱
    Returns:
        还原图像 (uint8)
    """
    f_ishift = np.fft.ifftshift(f_shift)
    img_back = np.fft.ifft2(f_ishift)
    img_back = np.abs(img_back)
    img_back = np.uint8(np.clip(img_back, 0, 255))
    return img_back


def fft_filter(img, filter_mask):
    """通用频域滤波
    Args:
        img: 灰度图
        filter_mask: 频域滤波器 (与频谱同尺寸)
    Returns:
        滤波后图像
    """
    gray = img if len(img.shape) == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    f = np.float32(gray)
    f_complex = np.fft.fft2(f)
    f_shift = np.fft.fftshift(f_complex)
    filtered = f_shift * filter_mask
    return ifft2(filtered)


def lowpass_ideal(shape, cutoff):
    """理想低通滤波器"""
    rows, cols = shape
    crow, ccol = rows // 2, cols // 2
    mask = np.zeros((rows, cols), np.float32)
    for i in range(rows):
        for j in range(cols):
            d = np.sqrt((i - crow) ** 2 + (j - ccol) ** 2)
            if d <= cutoff:
                mask[i, j] = 1
    return mask


def highpass_ideal(shape, cutoff):
    """理想高通滤波器"""
    return 1 - lowpass_ideal(shape, cutoff)


def lowpass_butterworth(shape, cutoff, order=2):
    """巴特沃斯低通滤波器"""
    rows, cols = shape
    crow, ccol = rows // 2, cols // 2
    u = np.arange(rows).reshape(-1, 1) - crow
    v = np.arange(cols).reshape(1, -1) - ccol
    d = np.sqrt(u ** 2 + v ** 2)
    mask = 1 / (1 + (d / cutoff) ** (2 * order))
    return mask.astype(np.float32)


def highpass_butterworth(shape, cutoff, order=2):
    """巴特沃斯高通滤波器"""
    return 1 - lowpass_butterworth(shape, cutoff, order)


def lowpass_gaussian(shape, cutoff):
    """高斯低通滤波器"""
    rows, cols = shape
    crow, ccol = rows // 2, cols // 2
    u = np.arange(rows).reshape(-1, 1) - crow
    v = np.arange(cols).reshape(1, -1) - ccol
    d2 = u ** 2 + v ** 2
    mask = np.exp(-d2 / (2 * cutoff ** 2))
    return mask.astype(np.float32)


def highpass_gaussian(shape, cutoff):
    """高斯高通滤波器"""
    return 1 - lowpass_gaussian(shape, cutoff)


def bandpass_filter(shape, low, high, filter_type='gaussian', order=2):
    """带通滤波器"""
    if filter_type == 'gaussian':
        return lowpass_gaussian(shape, high) - lowpass_gaussian(shape, low)
    elif filter_type == 'butterworth':
        return lowpass_butterworth(shape, high, order) - lowpass_butterworth(shape, low, order)
    else:
        return lowpass_ideal(shape, high) - lowpass_ideal(shape, low)


def homomorphic_filter(img, gamma_l=0.5, gamma_h=2.0, cutoff=30, c=1):
    """同态滤波 - 去除不均匀光照
    Args:
        img: 灰度图
        gamma_l: 低频增益 (抑制光照)
        gamma_h: 高频增益 (增强细节)
        cutoff: 截止频率
        c: 控制函数斜率
    Returns:
        滤波后图像
    """
    gray = img if len(img.shape) == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img_log = np.log1p(np.float64(gray))
    f = np.fft.fft2(img_log)
    f_shift = np.fft.fftshift(f)

    rows, cols = gray.shape
    crow, ccol = rows // 2, cols // 2
    u = np.arange(rows).reshape(-1, 1) - crow
    v = np.arange(cols).reshape(1, -1) - ccol
    d2 = u ** 2 + v ** 2
    H = (gamma_h - gamma_l) * (1 - np.exp(-c * d2 / (2 * cutoff ** 2))) + gamma_l

    filtered = f_shift * H
    f_ishift = np.fft.ifftshift(filtered)
    img_back = np.fft.ifft2(f_ishift)
    result = np.expm1(np.abs(img_back))
    result = np.uint8(np.clip(255 * result / result.max(), 0, 255))
    return result


def notch_filter(shape, centers, radius=5):
    """陷波滤波器 - 去除特定频率噪声
    Args:
        shape: 图像尺寸 (rows, cols)
        centers: 陷波中心列表 [(u1,v1), (u2,v2), ...]
        radius: 陷波半径
    Returns:
        滤波器掩码
    """
    mask = np.ones(shape, np.float32)
    rows, cols = shape
    for (u, v) in centers:
        # 对称点也要陷波
        for cu, cv in [(u, v), (rows - u, cols - v)]:
            if 0 <= cu < rows and 0 <= cv < cols:
                y, x = np.ogrid[max(0, cu-radius):min(rows, cu+radius+1),
                                 max(0, cv-radius):min(cols, cv+radius+1)]
                dist = (y - cu) ** 2 + (x - cv) ** 2
                mask[max(0, cu-radius):min(rows, cu+radius+1),
                     max(0, cv-radius):min(cols, cv+radius+1)][dist <= radius**2] = 0
    return mask


if __name__ == '__main__':
    img = cv2.imread('test.jpg', cv2.IMREAD_GRAYSCALE)
    if img is not None:
        f_shift, mag = fft2(img)
        # 低通滤波
        lp_mask = lowpass_gaussian(img.shape[:2], 50)
        blurred = fft_filter(img, lp_mask)
        # 高通滤波
        hp_mask = highpass_gaussian(img.shape[:2], 30)
        edges = fft_filter(img, hp_mask)
        # 同态滤波
        homo = homomorphic_filter(img)
        print(f"频谱: {mag.shape}, 低通: {blurred.shape}, 同态: {homo.shape}")
