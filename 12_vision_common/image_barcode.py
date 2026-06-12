"""
image_barcode_v13.py - 条形码识别V13
支持多种条形码格式识别与解码，适用于电赛物品识别/计数场景
"""
import cv2
import numpy as np


class BarcodeDetector:
    """条形码识别器 V13"""

    # 常见条形码格式
    FORMATS = ["EAN13", "EAN8", "UPC-A", "UPC-E", "CODE128", "CODE39", "CODE93", "I25"]

    def __init__(self, use_zbar=True):
        """
        Args:
            use_zbar: True用pyzbar, False用OpenCV内置detectBarcode
        """
        self.use_zbar = use_zbar
        self._decoder = None

    def _init_decoder(self):
        if self._decoder is not None:
            return
        if self.use_zbar:
            from pyzbar import pyzbar
            self._decoder = pyzbar
        else:
            self._decoder = cv2.barcode.BarcodeDetector()

    def preprocess(self, img):
        """条形码图像预处理"""
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if len(img.shape) == 3 else img
        # 增强对比度
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        enhanced = clahe.apply(gray)
        # 锐化
        kernel = np.array([[-1, -1, -1], [-1, 9, -1], [-1, -1, -1]])
        sharpened = cv2.filter2D(enhanced, -1, kernel)
        return sharpened

    def detect_orientation(self, img):
        """
        检测条形码方向并旋转至水平
        Returns:
            旋转后的图像, 角度
        """
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if len(img.shape) == 3 else img
        # Sobel检测条纹方向
        sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
        sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
        angle = np.arctan2(np.sum(sobely), np.sum(sobelx)) * 180 / np.pi
        # 条形码竖条纹，Sobel主要响应在X方向
        if abs(angle) > 10:
            h, w = img.shape[:2]
            M = cv2.getRotationMatrix2D((w // 2, h // 2), angle, 1.0)
            img = cv2.warpAffine(img, M, (w, h), borderMode=cv2.BORDER_REPLICATE)
        return img, angle

    def decode_single(self, img):
        """
        解码单个条形码
        Returns:
            dict: {data, type, bbox, quality} 或 None
        """
        self._init_decoder()
        if self.use_zbar:
            results = self._decoder.decode(img)
            if results:
                r = results[0]
                return {
                    "data": r.data.decode("utf-8"),
                    "type": r.type,
                    "bbox": (r.rect.left, r.rect.top, r.rect.width, r.rect.height),
                    "quality": r.quality,
                    "polygon": [(p.x, p.y) for p in r.polygon] if r.polygon else []
                }
        else:
            retval, decoded_info, decoded_type, points = self._decoder.detectAndDecode(img)
            if retval and decoded_info:
                return {
                    "data": decoded_info,
                    "type": decoded_type,
                    "bbox": cv2.boundingRect(points.astype(int)),
                    "points": points.tolist()
                }
        return None

    def decode_all(self, img):
        """
        解码图像中所有条形码
        Returns:
            list of dict
        """
        self._init_decoder()
        results = []
        if self.use_zbar:
            decoded = self._decoder.decode(img)
            for r in decoded:
                results.append({
                    "data": r.data.decode("utf-8"),
                    "type": r.type,
                    "bbox": (r.rect.left, r.rect.top, r.rect.width, r.rect.height),
                    "quality": r.quality
                })
        else:
            retval, decoded_info, decoded_type, points = self._decoder.detectAndDecodeMulti(img)
            if retval:
                for i, info in enumerate(decoded_info):
                    if info:
                        results.append({
                            "data": info,
                            "type": decoded_type[i] if i < len(decoded_type) else "UNKNOWN",
                            "bbox": cv2.boundingRect(points[i].astype(int)) if points is not None else None
                        })
        return results

    def locate_barcode_regions(self, img):
        """
        定位图像中可能的条形码区域（无需解码）
        Returns:
            list of (x, y, w, h)
        """
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if len(img.shape) == 3 else img
        # Sobel X 条形码有强竖向边缘
        sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
        sobelx = np.abs(sobelx).astype(np.uint8)
        # 二值化 + 闭运算连接
        _, binary = cv2.threshold(sobelx, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (21, 7))
        closed = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        regions = []
        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            aspect = w / h if h > 0 else 0
            if aspect > 1.5 and w > 50:  # 条形码通常宽远大于高
                regions.append((x, y, w, h))
        return regions

    def scan_and_decode(self, img, preprocess=True):
        """
        完整流程：预处理 -> 定位 -> 解码
        Returns:
            list of dict with data, type, bbox
        """
        if preprocess:
            processed = self.preprocess(img)
        else:
            processed = img
        # 尝试直接解码
        results = self.decode_all(processed)
        if results:
            return results
        # 如果失败，旋转后重试
        rotated, _ = self.detect_orientation(img)
        return self.decode_all(rotated)


if __name__ == "__main__":
    img = cv2.imread("test_barcode.png")
    if img is not None:
        detector = BarcodeDetector(use_zbar=True)
        results = detector.scan_and_decode(img)
        for r in results:
            print(f"[{r['type']}] {r['data']} @ {r['bbox']}")
        if not results:
            print("未检测到条形码")
    else:
        print("请准备测试图片 test_barcode.png")
