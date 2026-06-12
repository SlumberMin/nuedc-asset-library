"""
图像变换工具 - 仿射变换/透视变换/极坐标变换/对数变换
"""
import cv2
import numpy as np


def affine_transform(img, src_pts, dst_pts, size=None):
    """仿射变换 (3点对应)
    Args:
        img: 输入图像
        src_pts: 源点 np.float32 [[x1,y1],[x2,y2],[x3,y3]]
        dst_pts: 目标点 np.float32 [[x1,y1],[x2,y2],[x3,y3]]
        size: 输出尺寸 (w, h)，None则同原图
    Returns:
        变换后图像
    """
    if size is None:
        size = (img.shape[1], img.shape[0])
    M = cv2.getAffineTransform(np.float32(src_pts), np.float32(dst_pts))
    return cv2.warpAffine(img, M, size)


def affine_rotate(img, angle, center=None, scale=1.0):
    """仿射旋转
    Args:
        img: 输入图像
        angle: 旋转角度(逆时针)
        center: 旋转中心 None为图像中心
        scale: 缩放比例
    Returns:
        旋转后图像
    """
    h, w = img.shape[:2]
    if center is None:
        center = (w // 2, h // 2)
    M = cv2.getRotationMatrix2D(center, angle, scale)
    return cv2.warpAffine(img, M, (w, h))


def affine_scale(img, fx, fy=None):
    """仿射缩放"""
    if fy is None:
        fy = fx
    return cv2.resize(img, None, fx=fx, fy=fy, interpolation=cv2.INTER_LINEAR)


def affine_shear(img, shear_x=0, shear_y=0):
    """仿射剪切变换"""
    h, w = img.shape[:2]
    M = np.float32([[1, shear_x, 0],
                     [shear_y, 1, 0]])
    new_w = int(w + abs(shear_x) * h)
    new_h = int(h + abs(shear_y) * w)
    return cv2.warpAffine(img, M, (new_w, new_h))


def perspective_transform(img, src_pts, dst_pts, size=None):
    """透视变换 (4点对应)
    Args:
        img: 输入图像
        src_pts: 源点 np.float32 4个点
        dst_pts: 目标点 np.float32 4个点
        size: 输出尺寸 (w, h)
    Returns:
        变换后图像
    """
    if size is None:
        size = (img.shape[1], img.shape[0])
    M = cv2.getPerspectiveTransform(np.float32(src_pts), np.float32(dst_pts))
    return cv2.warpPerspective(img, M, size)


def perspective_correct(img, pts):
    """透视校正 - 将四边形校正为矩形
    Args:
        img: 输入图像
        pts: 四边形顶点 np.float32 4个点
    Returns:
        校正后图像
    """
    pts = np.float32(pts)
    w1 = np.linalg.norm(pts[0] - pts[1])
    w2 = np.linalg.norm(pts[2] - pts[3])
    h1 = np.linalg.norm(pts[0] - pts[3])
    h2 = np.linalg.norm(pts[1] - pts[2])
    w = int(max(w1, w2))
    h = int(max(h1, h2))
    dst = np.float32([[0, 0], [w, 0], [w, h], [0, h]])
    return perspective_transform(img, pts, dst, (w, h))


def polar_transform(img, center=None, max_radius=None, dsize=None):
    """极坐标变换 (线性极坐标)
    Args:
        img: 输入图像
        center: 变换中心
        max_radius: 最大半径
        dsize: 输出尺寸 (w, h)
    Returns:
        极坐标图像
    """
    h, w = img.shape[:2]
    if center is None:
        center = (w / 2, h / 2)
    if max_radius is None:
        max_radius = min(w, h) / 2
    if dsize is None:
        dsize = (int(max_radius), 360)
    return cv2.linearPolar(img, center, max_radius, cv2.INTER_LINEAR)


def log_polar_transform(img, center=None, max_radius=None, dsize=None):
    """对数极坐标变换"""
    h, w = img.shape[:2]
    if center is None:
        center = (w / 2, h / 2)
    if max_radius is None:
        max_radius = min(w, h) / 2
    if dsize is None:
        dsize = (int(max_radius), 360)
    return cv2.logPolar(img, center, max_radius, cv2.INTER_LINEAR)


def inverse_polar(img_polar, center, max_radius, dsize):
    """极坐标逆变换"""
    return cv2.linearPolar(img_polar, center, max_radius,
                           cv2.INTER_LINEAR + cv2.WARP_INVERSE_MAP)


def log_transform(img, c=None):
    """对数变换 s = c * log(1 + r)
    Args:
        img: 输入图像
        c: 常数系数 None则自动计算
    Returns:
        变换后图像
    """
    img_f = img.astype(np.float64)
    if c is None:
        c = 255.0 / np.log(1 + np.max(img_f))
    result = c * np.log(1 + img_f)
    return np.uint8(np.clip(result, 0, 255))


def gamma_transform(img, gamma=1.0, c=1.0):
    """伽马变换 s = c * r^gamma
    gamma < 1 提亮, gamma > 1 压暗
    """
    table = np.array([c * (i / 255.0) ** gamma * 255 for i in range(256)]).astype(np.uint8)
    return cv2.LUT(img, table)


if __name__ == '__main__':
    img = cv2.imread('test.jpg')
    if img is not None:
        # 仿射旋转45度
        rotated = affine_rotate(img, 45)
        # 透视校正
        pts = np.float32([[50, 50], [400, 50], [400, 350], [50, 350]])
        corrected = perspective_correct(img, pts)
        # 极坐标
        polar = polar_transform(img)
        # 对数变换
        log_img = log_transform(img)
        # 伽马变换提亮
        bright = gamma_transform(img, gamma=0.5)
        print(f"旋转: {rotated.shape}, 极坐标: {polar.shape}")
