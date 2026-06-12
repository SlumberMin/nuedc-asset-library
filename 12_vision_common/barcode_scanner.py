# -*- coding: utf-8 -*-
"""
条形码/二维码扫描模块 - ZBar + OpenCV + 多码检测
支持：EAN-13, EAN-8, UPC-A, Code128, Code39, QR Code, DataMatrix等
"""

import cv2
import numpy as np

try:
    import pyzbar.pyzbar as pyzbar
    HAS_PYZBAR = True
except ImportError:
    HAS_PYZBAR = False
    print("[警告] 未安装pyzbar，部分功能不可用。安装: pip install pyzbar")

try:
    from pylibdmtx.pylibdmtx import decode as dmtx_decode
    HAS_DMTX = True
except ImportError:
    HAS_DMTX = False


class BarcodeScanner:
    """条形码/二维码多码检测扫描器"""

    def __init__(self, enhance=True, min_confidence=0):
        """
        参数：
            enhance: 是否对图像做增强预处理
            min_confidence: 最低置信度过滤（0表示不过滤）
        """
        self.enhance = enhance
        self.min_confidence = min_confidence

    def preprocess(self, image):
        """图像增强预处理：灰度化→锐化→对比度增强"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image.copy()

        # CLAHE对比度增强
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        enhanced = clahe.apply(gray)

        # 锐化
        kernel = np.array([[-1, -1, -1], [-1, 9, -1], [-1, -1, -1]])
        sharpened = cv2.filter2D(enhanced, -1, kernel)
        return sharpened

    def decode_pyzbar(self, image):
        """使用pyzbar解码条形码/二维码"""
        if not HAS_PYZBAR:
            return []
        gray = self.preprocess(image) if self.enhance else \
               cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image

        results = []
        # 多尺度扫描提高检出率
        for scale in [1.0, 0.75, 1.5]:
            if scale != 1.0:
                h, w = gray.shape[:2]
                scaled = cv2.resize(gray, (int(w * scale), int(h * scale)))
            else:
                scaled = gray

            decoded = pyzbar.decode(scaled)
            for obj in decoded:
                data = obj.data.decode('utf-8', errors='replace')
                code_type = obj.type
                # 还原坐标到原始尺度
                pts = [(int(p.x / scale), int(p.y / scale)) for p in obj.polygon]

                # 去重：相同数据不同尺度只保留一个
                if not any(r['data'] == data for r in results):
                    results.append({
                        'data': data,
                        'type': code_type,
                        'quality': obj.quality,
                        'rect': obj.rect,
                        'polygon': pts,
                    })

        # 置信度过滤
        if self.min_confidence > 0:
            results = [r for r in results if r['quality'] >= self.min_confidence]

        return results

    def decode_datamatrix(self, image):
        """使用pylibdmtx解码DataMatrix码"""
        if not HAS_DMTX:
            return []
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
        results = []
        decoded = dmtx_decode(gray)
        for obj in decoded:
            results.append({
                'data': obj.data.decode('utf-8', errors='replace'),
                'type': 'DataMatrix',
                'rect': (obj.rect.left, obj.rect.top, obj.rect.width, obj.rect.height),
                'polygon': [],
            })
        return results

    def scan(self, image):
        """
        扫描图像中所有条形码/二维码
        返回：结果列表 [{'data': str, 'type': str, 'rect': ..., 'polygon': ...}, ...]
        """
        results = self.decode_pyzbar(image)
        # 补充DataMatrix检测
        if HAS_DMTX:
            dm_results = self.decode_datamatrix(image)
            for r in dm_results:
                if not any(existing['data'] == r['data'] for existing in results):
                    results.append(r)
        return results

    def scan_region(self, image, roi):
        """
        在指定ROI区域内扫描
        roi: (x, y, w, h)
        """
        x, y, w, h = roi
        cropped = image[y:y+h, x:x+w]
        results = self.scan(cropped)
        # 坐标偏移还原
        for r in results:
            if 'rect' in r and r['rect']:
                r['rect'] = (r['rect'][0] + x, r['rect'][1] + y,
                             r['rect'][2], r['rect'][3])
            r['polygon'] = [(p[0] + x, p[1] + y) for p in r.get('polygon', [])]
        return results

    def scan_video_frame(self, frame, roi=None):
        """视频帧扫描（可选ROI加速）"""
        if roi:
            return self.scan_region(frame, roi)
        return self.scan(frame)

    def draw_results(self, image, results, color=(0, 255, 0), thickness=2):
        """在图像上绘制扫描结果"""
        result = image.copy()
        for r in results:
            # 画多边形或矩形框
            if r.get('polygon') and len(r['polygon']) >= 4:
                pts = np.array(r['polygon'], dtype=np.int32)
                cv2.polylines(result, [pts], True, color, thickness)
            elif r.get('rect'):
                rx, ry, rw, rh = r['rect']
                cv2.rectangle(result, (rx, ry), (rx + rw, ry + rh), color, thickness)
            # 标注文字
            text = f"{r['type']}: {r['data']}"
            label_y = r['rect'][1] - 10 if r.get('rect') else 30
            label_x = r['rect'][0] if r.get('rect') else 10
            cv2.putText(result, text, (label_x, label_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        return result


# ========== 使用示例 ==========
if __name__ == '__main__':
    # 创建测试图像（需安装pyzbar才能正常运行）
    print("BarcodeScanner 条形码扫描模块")
    print("=" * 40)

    scanner = BarcodeScanner(enhance=True)

    # 示例：从图片文件扫描
    import sys
    if len(sys.argv) > 1:
        img = cv2.imread(sys.argv[1])
        if img is not None:
            results = scanner.scan(img)
            print(f"检测到 {len(results)} 个码：")
            for r in results:
                print(f"  类型: {r['type']}, 数据: {r['data']}")
            result_img = scanner.draw_results(img, results)
            cv2.imwrite("barcode_result.jpg", result_img)
            print("结果已保存为 barcode_result.jpg")
        else:
            print(f"无法读取图片: {sys.argv[1]}")
    else:
        print("用法: python barcode_scanner.py <图片路径>")
        print("支持码制: EAN-13, EAN-8, UPC-A, Code128, Code39, QR Code, DataMatrix等")
        print("\n编程调用示例：")
        print("  scanner = BarcodeScanner()")
        print("  results = scanner.scan(image)")
        print("  for r in results:")
        print("      print(r['type'], r['data'])")
