# -*- coding: utf-8 -*-
"""
文档扫描V2模块 - 边缘检测 + 透视校正 + OCR预处理
适用于电赛中纸质文档、答题卡、标签等的数字化扫描
"""

import cv2
import numpy as np


class DocumentScanner:
    """文档扫描器：自动检测文档边界、透视校正、OCR预处理"""

    def __init__(self, target_width=800, target_height=1100,
                 canny_low=50, canny_high=150, approx_epsilon=0.02):
        """
        参数：
            target_width/height: 输出目标尺寸（像素）
            canny_low/high: Canny边缘检测阈值
            approx_epsilon: 多边形近似精度（相对于周长的比例）
        """
        self.target_width = target_width
        self.target_height = target_height
        self.canny_low = canny_low
        self.canny_high = canny_high
        self.approx_epsilon = approx_epsilon

    def find_document_contour(self, image):
        """
        检测图像中的文档四边形轮廓
        返回：四角点坐标 (4, 2) 的numpy数组，未找到返回None
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
        h, w = gray.shape[:2]

        # 高斯模糊降噪
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Canny边缘检测
        edged = cv2.Canny(blurred, self.canny_low, self.canny_high)

        # 膨胀操作连接断裂边缘
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        edged = cv2.dilate(edged, kernel, iterations=2)

        # 查找轮廓，按面积排序
        contours, _ = cv2.findContours(edged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=cv2.contourArea, reverse=True)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < (h * w * 0.1):  # 面积太小跳过
                continue

            # 多边形近似
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, self.approx_epsilon * peri, True)

            # 找到四边形
            if len(approx) == 4:
                return self._order_points(approx.reshape(4, 2))

        return None

    def _order_points(self, pts):
        """
        排序四角点：左上、右上、右下、左下
        """
        rect = np.zeros((4, 2), dtype=np.float32)
        s = pts.sum(axis=1)
        d = np.diff(pts, axis=1).flatten()
        rect[0] = pts[np.argmin(s)]   # 左上
        rect[2] = pts[np.argmax(s)]   # 右下
        rect[1] = pts[np.argmin(d)]   # 右上
        rect[3] = pts[np.argmax(d)]   # 左下
        return rect

    def perspective_transform(self, image, pts):
        """
        透视变换：将四边形区域校正为矩形
        """
        dst = np.array([
            [0, 0],
            [self.target_width - 1, 0],
            [self.target_width - 1, self.target_height - 1],
            [0, self.target_height - 1]
        ], dtype=np.float32)

        M = cv2.getPerspectiveTransform(pts.astype(np.float32), dst)
        warped = cv2.warpPerspective(image, M, (self.target_width, self.target_height))
        return warped

    def scan(self, image, auto_detect=True, manual_corners=None):
        """
        完整文档扫描流程
        参数：
            image: 输入图像
            auto_detect: 是否自动检测文档边界
            manual_corners: 手动指定四角点 [(x1,y1), (x2,y2), (x3,y3), (x4,y4)]
        返回：校正后的文档图像
        """
        if auto_detect:
            pts = self.find_document_contour(image)
            if pts is None:
                print("[警告] 未检测到文档轮廓，返回原图")
                return image
        else:
            pts = self._order_points(np.array(manual_corners, dtype=np.float32))

        return self.perspective_transform(image, pts)

    @staticmethod
    def preprocess_for_ocr(image, method='adaptive'):
        """
        OCR预处理：增强文字可读性
        参数：
            method: 'adaptive'自适应二值化 | 'otsu'大津法 | 'enhance'增强灰度
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image

        # 去噪
        denoised = cv2.fastNlMeansDenoising(gray, h=10)

        # 倾斜校正（简化版）
        denoised = DocumentScanner._deskew(denoised)

        if method == 'adaptive':
            # CLAHE + 自适应阈值
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            enhanced = clahe.apply(denoised)
            binary = cv2.adaptiveThreshold(enhanced, 255,
                                           cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                           cv2.THRESH_BINARY, 31, 10)
        elif method == 'otsu':
            _, binary = cv2.threshold(denoised, 0, 255,
                                      cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        elif method == 'enhance':
            clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
            binary = clahe.apply(denoised)
        else:
            binary = denoised

        # 形态学操作去除噪点
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=1)

        return binary

    @staticmethod
    def _deskew(image):
        """基于最小面积矩形的倾斜校正"""
        coords = np.column_stack(np.where(image > 0))
        if len(coords) < 50:
            return image
        angle = cv2.minAreaRect(coords)[-1]
        if angle < -45:
            angle = -(90 + angle)
        else:
            angle = -angle
        if abs(angle) < 0.5:  # 倾斜角太小不校正
            return image
        h, w = image.shape[:2]
        M = cv2.getRotationMatrix2D((w // 2, h // 2), angle, 1.0)
        rotated = cv2.warpAffine(image, M, (w, h), flags=cv2.INTER_CUBIC,
                                  borderMode=cv2.BORDER_REPLICATE)
        return rotated

    def draw_detection(self, image, pts):
        """绘制文档边界检测结果"""
        result = image.copy()
        if pts is not None:
            pts_int = pts.astype(int)
            cv2.polylines(result, [pts_int], True, (0, 255, 0), 3)
            for i, pt in enumerate(pts_int):
                cv2.circle(result, tuple(pt), 8, (0, 0, 255), -1)
                cv2.putText(result, str(i), tuple(pt + [10, -10]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        return result


# ========== 使用示例 ==========
if __name__ == '__main__':
    print("DocumentScanner V2 文档扫描模块")
    print("=" * 40)

    scanner = DocumentScanner(target_width=800, target_height=1100)

    import sys
    if len(sys.argv) > 1:
        img = cv2.imread(sys.argv[1])
        if img is not None:
            # 自动检测并扫描
            result = scanner.scan(img)
            # OCR预处理
            ocr_ready = DocumentScanner.preprocess_for_ocr(result, method='adaptive')
            cv2.imwrite("doc_scan_result.jpg", result)
            cv2.imwrite("doc_ocr_ready.jpg", ocr_ready)
            print("文档扫描完成，结果已保存")

            # 绘制检测框
            pts = scanner.find_document_contour(img)
            if pts is not None:
                debug = scanner.draw_detection(img, pts)
                cv2.imwrite("doc_detection.jpg", debug)
                print("检测结果已保存为 doc_detection.jpg")
        else:
            print(f"无法读取图片: {sys.argv[1]}")
    else:
        print("用法: python document_scanner_v2.py <图片路径>")
        print("\n编程调用示例：")
        print("  scanner = DocumentScanner()")
        print("  scanned = scanner.scan(image)")
        print("  ocr_img = DocumentScanner.preprocess_for_ocr(scanned)")
