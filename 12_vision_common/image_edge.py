"""
边缘检测通用工具库
支持: Canny / Sobel / Laplacian / Prewitt / Roberts
"""
import cv2
import numpy as np


def edge_canny(img, threshold1=50, threshold2=150, aperture_size=3, l2_gradient=False):
    """Canny边缘检测"""
    if len(img.shape) == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img.copy()
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    return cv2.Canny(gray, threshold1, threshold2, apertureSize=aperture_size, L2gradient=l2_gradient)


def edge_sobel(img, dx=1, dy=1, ksize=3, scale=1, delta=0):
    """Sobel边缘检测"""
    if len(img.shape) == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img.copy()
    grad_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=ksize, scale=scale, delta=delta)
    grad_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=ksize, scale=scale, delta=delta)
    abs_grad_x = cv2.convertScaleAbs(grad_x)
    abs_grad_y = cv2.convertScaleAbs(grad_y)
    return cv2.addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0)


def edge_sobel_xy(img, ksize=3):
    """Sobel分别返回x和y方向边缘"""
    if len(img.shape) == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img.copy()
    grad_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=ksize)
    grad_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=ksize)
    return cv2.convertScaleAbs(grad_x), cv2.convertScaleAbs(grad_y)


def edge_laplacian(img, ksize=3, scale=1, delta=0):
    """Laplacian边缘检测"""
    if len(img.shape) == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img.copy()
    lap = cv2.Laplacian(gray, cv2.CV_64F, ksize=ksize, scale=scale, delta=delta)
    return cv2.convertScaleAbs(lap)


def edge_prewitt(img):
    """Prewitt边缘检测 (通过卷积核实现)"""
    if len(img.shape) == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img.copy()
    kernel_x = np.array([[-1, 0, 1], [-1, 0, 1], [-1, 0, 1]], dtype=np.float32)
    kernel_y = np.array([[-1, -1, -1], [0, 0, 0], [1, 1, 1]], dtype=np.float32)
    grad_x = cv2.filter2D(gray, cv2.CV_64F, kernel_x)
    grad_y = cv2.filter2D(gray, cv2.CV_64F, kernel_y)
    abs_x = cv2.convertScaleAbs(grad_x)
    abs_y = cv2.convertScaleAbs(grad_y)
    return cv2.addWeighted(abs_x, 0.5, abs_y, 0.5, 0)


def edge_roberts(img):
    """Roberts交叉边缘检测"""
    if len(img.shape) == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img.copy()
    kernel_x = np.array([[1, 0], [0, -1]], dtype=np.float32)
    kernel_y = np.array([[0, 1], [-1, 0]], dtype=np.float32)
    grad_x = cv2.filter2D(gray, cv2.CV_64F, kernel_x)
    grad_y = cv2.filter2D(gray, cv2.CV_64F, kernel_y)
    abs_x = cv2.convertScaleAbs(grad_x)
    abs_y = cv2.convertScaleAbs(grad_y)
    return cv2.addWeighted(abs_x, 0.5, abs_y, 0.5, 0)


def auto_canny(img, sigma=0.33):
    """自动阈值Canny (基于中值)"""
    if len(img.shape) == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img.copy()
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    v = np.median(gray)
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    return cv2.Canny(gray, lower, upper)


def edge_gradient_magnitude(img, ksize=3):
    """梯度幅值图"""
    if len(img.shape) == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img.copy()
    grad_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=ksize)
    grad_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=ksize)
    magnitude = np.sqrt(grad_x ** 2 + grad_y ** 2)
    return cv2.convertScaleAbs(magnitude)
