"""
条形码检测 - 定位 + 解码
适用于电赛中条码识别、物品标识读取等场景
依赖: numpy, opencv-python, pyzbar (需pip install pyzbar)
"""

import cv2
import numpy as np
from typing import List, Tuple, Optional, Dict

# 尝试导入 pyzbar
try:
    from pyzbar import pyzbar
    HAS_PYZBAR = True
except ImportError:
    HAS_PYZBAR = False
    print("[WARN] pyzbar 未安装, 运行: pip install pyzbar")


# ==================== 条形码定位（不依赖pyzbar） ====================

def locate_barcode_by_gradient(gray: np.ndarray) -> List[Tuple[int, int, int, int]]:
    """
    基于梯度特征定位条形码区域
    条形码区域具有强烈的水平梯度和较弱的垂直梯度
    返回: [(x, y, w, h), ...]
    """
    # Sobel梯度
    grad_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
    grad_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)

    # 梯度差值（条形码水平梯度远大于垂直梯度）
    diff = cv2.absdiff(np.abs(grad_x), np.abs(grad_y))
    diff = np.uint8(np.clip(diff, 0, 255))

    # 二值化 + 形态学操作
    _, thresh = cv2.threshold(diff, 50, 255, cv2.THRESH_BINARY)

    kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (21, 7))
    closed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel_close)

    kernel_open = cv2.getStructuringElement(cv2.MORPH_RECT, (11, 5))
    opened = cv2.morphologyEx(closed, cv2.MORPH_OPEN, kernel_open)

    # 查找轮廓
    contours, _ = cv2.findContours(opened, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    rects = []
    for cnt in contours:
        x, y, w, h = cv2.boundingRect(cnt)
        aspect_ratio = w / (h + 1e-10)
        area = w * h

        # 条形码通常宽大于高，面积适中
        if aspect_ratio > 1.5 and area > 2000:
            rects.append((x, y, w, h))

    return rects


def locate_barcode_by_morphology(gray: np.ndarray,
                                  kernel_length: int = 21) -> List[Tuple[int, int, int, int]]:
    """
    基于形态学的条形码定位
    利用条形码的条纹特征
    """
    # 自适应阈值
    binary = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                    cv2.THRESH_BINARY_INV, 15, 5)

    # 水平方向闭操作连接条码条纹
    kernel_h = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel_length, 1))
    closed = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel_h)

    # 垂直方向开操作去除噪声
    kernel_v = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 5))
    opened = cv2.morphologyEx(closed, cv2.MORPH_OPEN, kernel_v)

    # 膨胀合并
    kernel_d = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 15))
    dilated = cv2.dilate(opened, kernel_d, iterations=2)

    contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    rects = []
    for cnt in contours:
        x, y, w, h = cv2.boundingRect(cnt)
        if w > 50 and h > 20 and w / h > 1.0:
            rects.append((x, y, w, h))

    return rects


# ==================== 条形码解码 ====================

def decode_barcodes(image: np.ndarray) -> List[Dict]:
    """
    解码图像中所有条形码
    返回: [{'data': str, 'type': str, 'rect': (x,y,w,h), 'quality': int}, ...]
    """
    if not HAS_PYZBAR:
        raise ImportError("需要安装 pyzbar: pip install pyzbar")

    barcodes = pyzbar.decode(image)
    results = []

    for bc in barcodes:
        results.append({
            'data': bc.data.decode('utf-8', errors='replace'),
            'type': bc.type,
            'rect': (bc.rect.left, bc.rect.top, bc.rect.width, bc.rect.height),
            'quality': bc.quality,
            'polygon': [(p.x, p.y) for p in bc.polygon] if bc.polygon else []
        })

    return results


def decode_barcode_roi(image: np.ndarray,
                       roi: Tuple[int, int, int, int]) -> Optional[Dict]:
    """
    对指定ROI区域解码条形码
    roi: (x, y, w, h)
    """
    if not HAS_PYZBAR:
        raise ImportError("需要安装 pyzbar: pip install pyzbar")

    x, y, w, h = roi
    crop = image[y:y + h, x:x + w]

    barcodes = pyzbar.decode(crop)
    if not barcodes:
        return None

    bc = barcodes[0]
    return {
        'data': bc.data.decode('utf-8', errors='replace'),
        'type': bc.type,
        'rect': (x + bc.rect.left, y + bc.rect.top, bc.rect.width, bc.rect.height),
        'quality': bc.quality
    }


def enhance_for_barcode(gray: np.ndarray) -> np.ndarray:
    """
    针对条形码的图像增强
    """
    # CLAHE对比度增强
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    enhanced = clahe.apply(gray)

    # 锐化
    kernel = np.array([[-1, -1, -1],
                       [-1,  9, -1],
                       [-1, -1, -1]])
    sharpened = cv2.filter2D(enhanced, -1, kernel)

    return sharpened


def multi_scale_decode(image: np.ndarray) -> List[Dict]:
    """
    多尺度尝试解码条形码（对小/模糊条码有效）
    """
    if not HAS_PYZBAR:
        raise ImportError("需要安装 pyzbar: pip install pyzbar")

    all_results = []
    seen = set()

    gray = image if len(image.shape) == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    for scale in [1.0, 1.5, 2.0, 0.75, 0.5]:
        if scale != 1.0:
            h, w = gray.shape
            resized = cv2.resize(gray, (int(w * scale), int(h * scale)))
        else:
            resized = gray

        # 尝试原图和增强图
        for img_variant in [resized, enhance_for_barcode(resized)]:
            results = decode_barcodes(img_variant)
            for r in results:
                key = r['data']
                if key not in seen:
                    seen.add(key)
                    # 还原坐标到原图尺度
                    if scale != 1.0:
                        rect = r['rect']
                        r['rect'] = tuple(int(v / scale) for v in rect)
                    all_results.append(r)

    return all_results


def draw_barcode_results(image: np.ndarray, results: List[Dict]) -> np.ndarray:
    """绘制条形码检测结果"""
    vis = image.copy()

    for r in results:
        x, y, w, h = r['rect']
        cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 255, 0), 2)

        label = f"{r['type']}: {r['data']}"
        cv2.putText(vis, label, (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    return vis


# ==================== 演示 ====================
if __name__ == '__main__':
    # 创建模拟条形码图像
    img = np.ones((200, 400), dtype=np.uint8) * 255

    # 简单绘制条纹
    x_start = 50
    bar_widths = [2, 1, 1, 3, 1, 2, 1, 1, 3, 2, 1, 1, 2, 3, 1, 1, 2]
    x = x_start
    for i, w in enumerate(bar_widths):
        if i % 2 == 0:
            cv2.rectangle(img, (x, 50), (x + w * 3, 150), 0, -1)
        x += w * 3

    # 定位
    rects = locate_barcode_by_gradient(img)
    print(f"梯度法定位到 {len(rects)} 个条形码区域: {rects}")

    rects2 = locate_barcode_by_morphology(img)
    print(f"形态学法定位到 {len(rects2)} 个条形码区域: {rects2}")

    # 解码（需要 pyzbar）
    if HAS_PYZBAR:
        results = multi_scale_decode(img)
        for r in results:
            print(f"解码结果: {r['type']} = {r['data']}")
    else:
        print("安装 pyzbar 后可使用解码功能: pip install pyzbar")
