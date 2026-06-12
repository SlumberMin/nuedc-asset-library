#!/usr/bin/env python3
"""
圆形目标检测
功能：霍夫圆检测 + 轮廓拟合圆检测
适用：OpenCV + Orange Pi 5
"""

import cv2
import numpy as np


class CircleDetector:
    """圆形目标检测器，支持霍夫变换和轮廓拟合两种方法"""

    def __init__(self, method='contour', min_radius=10, max_radius=200,
                 min_area=200, color_filter=None):
        """
        参数:
            method: 'hough' | 'contour' | 'both'
            min_radius: 最小半径
            max_radius: 最大半径
            min_area: 最小轮廓面积(轮廓法)
            color_filter: 颜色过滤字典 {'lower': [H,S,V], 'upper': [H,S,V]}
        """
        self.method = method
        self.min_radius = min_radius
        self.max_radius = max_radius
        self.min_area = min_area
        self.color_filter = color_filter

    def _preprocess(self, frame):
        """预处理：灰度化 + 高斯模糊"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (9, 9), 2)
        return gray, blurred

    def _apply_color_filter(self, frame):
        """应用颜色过滤"""
        if self.color_filter is None:
            return None
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower = np.array(self.color_filter['lower'])
        upper = np.array(self.color_filter['upper'])
        mask = cv2.inRange(hsv, lower, upper)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def detect_hough(self, blurred):
        """
        霍夫圆检测
        返回: circles list of (x, y, r, confidence)
        """
        circles = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            dp=1.2,
            minDist=50,
            param1=100,
            param2=40,
            minRadius=self.min_radius,
            maxRadius=self.max_radius
        )

        results = []
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for c in circles[0]:
                x, y, r = int(c[0]), int(c[1]), int(c[2])
                results.append({
                    'cx': x, 'cy': y, 'radius': r,
                    'method': 'hough',
                    'confidence': 0.8,
                    'circularity': 0.0,
                    'area': np.pi * r * r,
                })
        return results

    def detect_contour(self, frame, mask=None):
        """
        轮廓拟合圆检测
        返回: circles list of dict
        """
        if mask is not None:
            binary = mask
        else:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (9, 9), 2)
            # 自适应阈值
            binary = cv2.adaptiveThreshold(blurred, 255,
                                           cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                           cv2.THRESH_BINARY_INV, 11, 2)

        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)

        results = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_area:
                continue

            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0:
                continue

            # 圆度 = 4π*面积 / 周长²
            circularity = 4 * np.pi * area / (perimeter * perimeter)

            # 圆度阈值
            if circularity < 0.6:
                continue

            # 最小外接圆
            (cx, cy), radius = cv2.minEnclosingCircle(cnt)
            cx, cy, radius = int(cx), int(cy), int(radius)

            if radius < self.min_radius or radius > self.max_radius:
                continue

            # 拟合椭圆(更精确)
            ellipse_fit = None
            if len(cnt) >= 5:
                try:
                    ellipse = cv2.fitEllipse(cnt)
                    (ecx, ecy), (ma, Mi), angle = ellipse
                    ratio = min(ma, Mi) / max(ma, Mi) if max(ma, Mi) > 0 else 0
                    if ratio > 0.7:  # 接近圆形
                        ellipse_fit = {
                            'center': (int(ecx), int(ecy)),
                            'axes': (int(ma/2), int(Mi/2)),
                            'angle': angle,
                            'aspect_ratio': ratio,
                        }
                except:
                    pass

            results.append({
                'cx': cx, 'cy': cy,
                'radius': radius,
                'method': 'contour',
                'confidence': circularity,
                'circularity': circularity,
                'area': area,
                'ellipse': ellipse_fit,
                'contour': cnt,
            })

        # 按圆度排序
        results.sort(key=lambda r: r['confidence'], reverse=True)
        return results

    def detect(self, frame):
        """
        完整检测流程

        返回:
            results: 圆形检测结果列表
        """
        gray, blurred = self._preprocess(frame)
        color_mask = self._apply_color_filter(frame)

        all_results = []

        if self.method in ('hough', 'both'):
            hough_results = self.detect_hough(blurred)
            all_results.extend(hough_results)

        if self.method in ('contour', 'both'):
            contour_results = self.detect_contour(frame, color_mask)
            all_results.extend(contour_results)

        # 如果是both模式，去重(位置接近的)
        if self.method == 'both':
            all_results = self._deduplicate(all_results)

        return all_results

    def _deduplicate(self, results, dist_thresh=30):
        """去重：合并位置接近的检测结果"""
        if len(results) <= 1:
            return results

        keep = []
        used = set()
        # 按confidence降序排序
        results.sort(key=lambda r: r['confidence'], reverse=True)

        for i, r1 in enumerate(results):
            if i in used:
                continue
            keep.append(r1)
            for j, r2 in enumerate(results):
                if j <= i or j in used:
                    continue
                dist = np.sqrt((r1['cx'] - r2['cx'])**2 + (r1['cy'] - r2['cy'])**2)
                if dist < dist_thresh:
                    used.add(j)

        return keep

    def draw(self, frame, results, show_info=True):
        """绘制检测结果"""
        vis = frame.copy()

        for i, r in enumerate(results):
            cx, cy, radius = r['cx'], r['cy'], r['radius']
            color = (0, 255, 0)

            # 绘制圆
            cv2.circle(vis, (cx, cy), radius, color, 2)
            cv2.circle(vis, (cx, cy), 3, (0, 0, 255), -1)

            # 绘制十字准线
            cv2.drawMarker(vis, (cx, cy), (255, 0, 0), cv2.MARKER_CROSS, 20, 1)

            # 绘制拟合椭圆
            if 'ellipse' in r and r['ellipse'] is not None:
                e = r['ellipse']
                cv2.ellipse(vis, e['center'], e['axes'], e['angle'],
                           0, 360, (255, 255, 0), 1)

            # 显示信息
            if show_info:
                info = f"R:{radius} C:{r['circularity']:.2f}"
                cv2.putText(vis, info, (cx + radius + 5, cy - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                cv2.putText(vis, f"({cx},{cy})", (cx + radius + 5, cy + 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        cv2.putText(vis, f"Circles: {len(results)}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        return vis


def run_demo(camera_id=0, method='contour'):
    """实时演示"""
    cap = cv2.VideoCapture(camera_id)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    detector = CircleDetector(method=method)

    print("=" * 50)
    print(f"圆形目标检测 - 方法: {method}")
    print("q/ESC: 退出 | h: 霍夫 | c: 轮廓 | b: 两者")
    print("=" * 50)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        results = detector.detect(frame)
        vis = detector.draw(frame, results)

        cv2.imshow('Circle Detector', vis)

        key = cv2.waitKey(1) & 0xFF
        if key in [ord('q'), 27]:
            break
        elif key == ord('h'):
            detector.method = 'hough'
            print("方法: Hough")
        elif key == ord('c'):
            detector.method = 'contour'
            print("方法: Contour")
        elif key == ord('b'):
            detector.method = 'both'
            print("方法: Both")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='圆形目标检测')
    parser.add_argument('--camera', type=int, default=0)
    parser.add_argument('--method', type=str, default='contour',
                       choices=['hough', 'contour', 'both'])
    parser.add_argument('--min-radius', type=int, default=10)
    parser.add_argument('--max-radius', type=int, default=200)
    args = parser.parse_args()

    detector = CircleDetector(method=args.method,
                              min_radius=args.min_radius,
                              max_radius=args.max_radius)
    run_demo(args.camera, args.method)
