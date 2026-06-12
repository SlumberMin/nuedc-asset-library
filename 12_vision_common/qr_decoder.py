#!/usr/bin/env python3
"""
二维码/条形码解码
功能：QR码、DataMatrix、条形码检测与解码
适用：OpenCV + pyzbar + Orange Pi 5
依赖：pip install pyzbar opencv-python
"""

import cv2
import numpy as np
import json

try:
    from pyzbar import pyzbar
    HAS_PYZBAR = True
except ImportError:
    HAS_PYZBAR = False
    print("[警告] pyzbar未安装，仅支持OpenCV QR检测")


class QRDecoder:
    """二维码/条形码解码器"""

    def __init__(self, use_opencv_qr=True, use_pyzbar=True):
        """
        参数:
            use_opencv_qr: 使用OpenCV内置QR检测器
            use_pyzbar: 使用pyzbar库(支持更多格式)
        """
        self.use_opencv_qr = use_opencv_qr
        self.use_pyzbar = use_pyzbar and HAS_PYZBAR

        # OpenCV QR检测器
        if self.use_opencv_qr:
            self.qr_detector = cv2.QRCodeDetector()

        # 支持的编码格式(供显示)
        self.type_names = {
            'QRCODE': 'QR Code',
            'EAN13': 'EAN-13',
            'EAN8': 'EAN-8',
            'UPCA': 'UPC-A',
            'UPCE': 'UPC-E',
            'CODE128': 'Code 128',
            'CODE39': 'Code 39',
            'I25': 'Interleaved 2 of 5',
            'DATAMATRIX': 'Data Matrix',
        }

    def decode_opencv(self, frame):
        """OpenCV QR码检测"""
        results = []
        if not self.use_opencv_qr:
            return results

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if len(frame.shape) == 3 else frame

        try:
            # OpenCV 4.x 新API：detectAndDecodeMulti
            retval, decoded_info, points, _ = self.qr_detector.detectAndDecodeMulti(gray)
            if retval:
                for info, pts in zip(decoded_info, points):
                    if info:
                        cx = int(np.mean(pts[:, 0]))
                        cy = int(np.mean(pts[:, 1]))
                        results.append({
                            'type': 'QRCODE',
                            'data': info,
                            'points': pts.tolist(),
                            'center': (cx, cy),
                            'source': 'opencv',
                        })
        except Exception:
            # 回退到单个检测
            try:
                data, points, _ = self.qr_detector.detectAndDecode(gray)
                if data and points is not None:
                    cx = int(np.mean(points[:, 0]))
                    cy = int(np.mean(points[:, 1]))
                    results.append({
                        'type': 'QRCODE',
                        'data': data,
                        'points': points.tolist(),
                        'center': (cx, cy),
                        'source': 'opencv',
                    })
            except Exception:
                pass

        return results

    def decode_pyzbar(self, frame):
        """pyzbar解码(支持多种格式)"""
        results = []
        if not self.use_pyzbar:
            return results

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if len(frame.shape) == 3 else frame

        try:
            decoded_objects = pyzbar.decode(gray)
            for obj in decoded_objects:
                pts = obj.polygon
                if len(pts) >= 4:
                    pts_array = np.array([(p.x, p.y) for p in pts])
                    cx = int(np.mean(pts_array[:, 0]))
                    cy = int(np.mean(pts_array[:, 1]))
                else:
                    x, y, w, h = obj.rect
                    pts_array = np.array([[x, y], [x+w, y], [x+w, y+h], [x, y+h]])
                    cx, cy = x + w//2, y + h//2

                barcode_type = obj.type.decode('utf-8') if isinstance(obj.type, bytes) else str(obj.type)
                data = obj.data.decode('utf-8') if isinstance(obj.data, bytes) else str(obj.data)

                results.append({
                    'type': barcode_type,
                    'data': data,
                    'points': pts_array.tolist(),
                    'center': (cx, cy),
                    'rect': (obj.rect.left, obj.rect.top, obj.rect.width, obj.rect.height),
                    'quality': obj.quality,
                    'source': 'pyzbar',
                })
        except Exception as e:
            print(f"[pyzbar错误] {e}")

        return results

    def decode(self, frame):
        """
        完整解码流程

        返回:
            results: list of dict, 每个包含:
                - type: 编码类型
                - data: 解码数据
                - points: 定位点坐标
                - center: 中心坐标
                - source: 检测来源
        """
        all_results = []

        # OpenCV检测
        cv_results = self.decode_opencv(frame)
        all_results.extend(cv_results)

        # pyzbar检测
        pz_results = self.decode_pyzbar(frame)
        all_results.extend(pz_results)

        # 去重(基于数据内容)
        seen_data = set()
        unique_results = []
        for r in all_results:
            if r['data'] not in seen_data:
                seen_data.add(r['data'])
                unique_results.append(r)

        return unique_results

    def draw(self, frame, results):
        """绘制检测结果"""
        vis = frame.copy()

        for r in results:
            pts = np.array(r['points'], dtype=np.int32)

            # 绘制边框
            cv2.polylines(vis, [pts], True, (0, 255, 0), 2)

            # 绘制中心
            cx, cy = r['center']
            cv2.circle(vis, (cx, cy), 5, (0, 0, 255), -1)

            # 显示类型和数据
            type_name = self.type_names.get(r['type'], r['type'])
            label = f"{type_name}: {r['data'][:30]}"
            cv2.putText(vis, label, (cx - 50, cy - 15),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

        cv2.putText(vis, f"Detected: {len(results)}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        return vis

    def decode_image(self, image_path):
        """从图片文件解码"""
        frame = cv2.imread(image_path)
        if frame is None:
            print(f"[错误] 无法读取图片: {image_path}")
            return []
        return self.decode(frame)

    def generate_qr(self, data, output_path=None, size=300):
        """生成QR码(需要qrcode库)"""
        try:
            import qrcode
            qr = qrcode.QRCode(version=1, box_size=10, border=4)
            qr.add_data(data)
            qr.make(fit=True)
            img = qr.make_image(fill_color="black", back_color="white")

            if output_path:
                img.save(output_path)
                print(f"[生成] QR码 -> {output_path}")

            # 转为OpenCV格式
            import io
            buf = io.BytesIO()
            img.save(buf, format='PNG')
            buf.seek(0)
            img_array = np.frombuffer(buf.getvalue(), dtype=np.uint8)
            cv_img = cv2.imdecode(img_array, cv2.IMREAD_GRAYSCALE)
            return cv_img

        except ImportError:
            print("[警告] qrcode库未安装: pip install qrcode[pil]")
            return None


def run_demo(camera_id=0):
    """实时演示"""
    cap = cv2.VideoCapture(camera_id)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    decoder = QRDecoder()

    print("=" * 50)
    print("二维码/条形码解码器")
    print("q/ESC: 退出 | s: 截图保存")
    print("=" * 50)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        results = decoder.decode(frame)
        vis = decoder.draw(frame, results)

        # 打印解码结果
        for r in results:
            print(f"[检测] {r['type']}: {r['data']}")

        cv2.imshow('QR/Barcode Decoder', vis)

        key = cv2.waitKey(1) & 0xFF
        if key in [ord('q'), 27]:
            break
        elif key == ord('s'):
            cv2.imwrite('qr_capture.jpg', frame)
            print("[保存] 截图: qr_capture.jpg")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='二维码/条形码解码')
    parser.add_argument('--camera', type=int, default=0)
    parser.add_argument('--image', type=str, default=None, help='从图片解码')
    parser.add_argument('--generate', type=str, default=None, help='生成QR码(输入数据)')
    args = parser.parse_args()

    if args.image:
        decoder = QRDecoder()
        results = decoder.decode_image(args.image)
        for r in results:
            print(f"{r['type']}: {r['data']}")
    elif args.generate:
        decoder = QRDecoder()
        decoder.generate_qr(args.generate, 'qr_generated.png')
    else:
        run_demo(args.camera)
