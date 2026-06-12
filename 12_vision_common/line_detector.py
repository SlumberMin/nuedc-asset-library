#!/usr/bin/env python3
"""
循迹线检测
功能：检测黑色/红色/蓝色循迹线，支持直线和曲线
适用：OpenCV + Orange Pi 5 / 摄像头朝下安装
"""

import cv2
import numpy as np
from collections import deque


class LineDetector:
    """循迹线检测器"""

    # 颜色阈值预设 (HSV)
    LINE_COLORS = {
        'black': {'lower': [0, 0, 0], 'upper': [180, 255, 60]},
        'red1':  {'lower': [0, 100, 100], 'upper': [10, 255, 255]},
        'red2':  {'lower': [160, 100, 100], 'upper': [180, 255, 255]},
        'blue':  {'lower': [100, 80, 80], 'upper': [130, 255, 255]},
        'green': {'lower': [35, 80, 80], 'upper': [85, 255, 255]},
    }

    def __init__(self, line_color='black', roi_ratio=0.5, binary_thresh=0,
                 min_line_length=30, max_line_gap=10):
        """
        参数:
            line_color: 线条颜色 ('black', 'red', 'blue', 'green')
            roi_ratio: 感兴趣区域占画面比例 (从底部开始)
            binary_thresh: 二值化阈值 (0=自适应)
            min_line_length: 最小线段长度 (HoughLinesP)
            max_line_gap: 最大线段间隙 (HoughLinesP)
        """
        self.line_color = line_color
        self.roi_ratio = roi_ratio
        self.binary_thresh = binary_thresh
        self.min_line_length = min_line_length
        self.max_line_gap = max_line_gap

        # 曲线追踪历史
        self.center_history = deque(maxlen=30)

    def _get_roi(self, frame):
        """提取感兴趣区域(画面底部)"""
        h, w = frame.shape[:2]
        y_start = int(h * (1 - self.roi_ratio))
        return frame[y_start:h, 0:w], y_start

    def _get_mask(self, hsv_roi):
        """生成线条颜色掩膜"""
        if self.line_color == 'red':
            lower1 = np.array(self.LINE_COLORS['red1']['lower'])
            upper1 = np.array(self.LINE_COLORS['red1']['upper'])
            lower2 = np.array(self.LINE_COLORS['red2']['lower'])
            upper2 = np.array(self.LINE_COLORS['red2']['upper'])
            mask1 = cv2.inRange(hsv_roi, lower1, upper1)
            mask2 = cv2.inRange(hsv_roi, lower2, upper2)
            mask = cv2.bitwise_or(mask1, mask2)
        elif self.line_color == 'black':
            # 黑色用V通道阈值
            cfg = self.LINE_COLORS['black']
            lower = np.array(cfg['lower'])
            upper = np.array(cfg['upper'])
            mask = cv2.inRange(hsv_roi, lower, upper)
        else:
            cfg = self.LINE_COLORS.get(self.line_color, self.LINE_COLORS['blue'])
            lower = np.array(cfg['lower'])
            upper = np.array(cfg['upper'])
            mask = cv2.inRange(hsv_roi, lower, upper)

        # 形态学处理
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        return mask

    def detect_lines_hough(self, mask):
        """
        HoughLinesP 检测直线段
        返回: lines list of (x1, y1, x2, y2)
        """
        edges = cv2.Canny(mask, 50, 150, apertureSize=3)
        lines = cv2.HoughLinesP(edges, 1, np.pi/180,
                                threshold=50,
                                minLineLength=self.min_line_length,
                                maxLineGap=self.max_line_gap)
        if lines is None:
            return []
        return [line[0] for line in lines]

    def detect_line_center(self, mask, n_slices=10):
        """
        切片法检测线条中心(支持曲线)
        将ROI分成n个水平切片，每个切片计算线条中心点

        返回: center_points list of (x, y), deviation(偏移量)
        """
        h, w = mask.shape[:2]
        slice_h = h // n_slices
        center_points = []

        for i in range(n_slices):
            y_start = i * slice_h
            y_end = (i + 1) * slice_h if i < n_slices - 1 else h
            slice_mask = mask[y_start:y_end, :]

            # 计算该切片中白色像素的x坐标均值
            white_pixels = np.where(slice_mask > 0)
            if len(white_pixels[0]) > 5:  # 至少5个像素点
                cx = int(np.mean(white_pixels[1]))
                cy = (y_start + y_end) // 2
                center_points.append((cx, cy))

        return center_points

    def calculate_deviation(self, center_points, frame_width):
        """
        计算线条偏移量(相对于画面中心)
        返回: deviation (正值=偏右, 负值=偏左), angle(偏转角)
        """
        if not center_points:
            return None, None

        # 使用底部的几个点计算偏差
        bottom_pts = center_points[-3:] if len(center_points) >= 3 else center_points
        avg_x = np.mean([p[0] for p in bottom_pts])
        center_x = frame_width / 2
        deviation = avg_x - center_x

        # 计算方向角
        angle = None
        if len(center_points) >= 2:
            p1 = center_points[0]
            p2 = center_points[-1]
            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            if dy != 0:
                angle = np.degrees(np.arctan2(dx, dy))

        return deviation, angle

    def fit_curve(self, center_points):
        """
        对中心点拟合多项式曲线
        返回: 系数 coeffs, 曲线点列表
        """
        if len(center_points) < 3:
            return None, center_points

        xs = np.array([p[0] for p in center_points])
        ys = np.array([p[1] for p in center_points])

        # 二次多项式拟合 x = f(y)
        try:
            coeffs = np.polyfit(ys, xs, 2)
            poly = np.poly1d(coeffs)
            curve_pts = []
            for y in range(int(ys.min()), int(ys.max()), 5):
                x = int(poly(y))
                curve_pts.append((x, y))
            return coeffs, curve_pts
        except:
            return None, center_points

    def detect(self, frame):
        """
        完整检测流程

        返回:
            result: dict 包含:
                - lines: HoughLinesP检测到的线段
                - center_points: 切片法中心点
                - curve_points: 拟合曲线点
                - deviation: 偏移量
                - angle: 方向角
                - mask: 二值掩膜
                - roi_offset: ROI的y偏移
        """
        roi, y_offset = self._get_roi(frame)
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = self._get_mask(hsv)

        # 直线检测
        lines = self.detect_lines_hough(mask)

        # 切片法中心点
        center_points = self.detect_line_center(mask)

        # 曲线拟合
        coeffs, curve_points = self.fit_curve(center_points)

        # 偏移计算
        deviation, angle = self.calculate_deviation(center_points, frame.shape[1])

        # 更新历史
        if center_points:
            self.center_history.append(center_points[-1])

        return {
            'lines': lines,
            'center_points': center_points,
            'curve_points': curve_points,
            'curve_coeffs': coeffs,
            'deviation': deviation,
            'angle': angle,
            'mask': mask,
            'roi_offset': y_offset,
        }

    def draw(self, frame, result):
        """可视化检测结果"""
        vis = frame.copy()
        y_off = result['roi_offset']
        h, w = frame.shape[:2]

        # 绘制ROI边界
        cv2.line(vis, (0, y_off), (w, y_off), (255, 255, 0), 1)

        # 绘制Hough线段
        for line in result['lines']:
            x1, y1, x2, y2 = line
            cv2.line(vis, (x1, y1 + y_off), (x2, y2 + y_off), (0, 255, 0), 2)

        # 绘制中心点
        for i, pt in enumerate(result['center_points']):
            cv2.circle(vis, (pt[0], pt[1] + y_off), 4, (0, 0, 255), -1)

        # 绘制拟合曲线
        curve_pts = result.get('curve_points', [])
        if curve_pts and len(curve_pts) > 1:
            for i in range(1, len(curve_pts)):
                p1 = (curve_pts[i-1][0], curve_pts[i-1][1] + y_off)
                p2 = (curve_pts[i][0], curve_pts[i][1] + y_off)
                cv2.line(vis, p1, p2, (255, 0, 255), 2)

        # 绘制画面中心线
        cv2.line(vis, (w//2, 0), (w//2, h), (200, 200, 200), 1, cv2.LINE_AA)

        # 显示偏移信息
        dev = result.get('deviation')
        angle = result.get('angle')
        if dev is not None:
            color = (0, 255, 0) if abs(dev) < 30 else (0, 0, 255)
            cv2.putText(vis, f"Deviation: {dev:.1f}px", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            if angle is not None:
                cv2.putText(vis, f"Angle: {angle:.1f} deg", (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

            # 方向指示
            if abs(dev) > 10:
                direction = "RIGHT" if dev > 0 else "LEFT"
                cv2.putText(vis, f"Turn {direction}", (10, 90),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        return vis


def run_demo(camera_id=0, color='black'):
    """实时演示"""
    cap = cv2.VideoCapture(camera_id)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    detector = LineDetector(line_color=color)

    print("=" * 50)
    print(f"循迹线检测 - 颜色: {color}")
    print("q/ESC: 退出 | 1-3: 切换ROI比例(30/50/70%)")
    print("=" * 50)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        result = detector.detect(frame)
        vis = detector.draw(frame, result)

        cv2.imshow('Line Detector', vis)
        cv2.imshow('Mask', result['mask'])

        key = cv2.waitKey(1) & 0xFF
        if key in [ord('q'), 27]:
            break
        elif key == ord('1'):
            detector.roi_ratio = 0.3
            print("ROI: 30%")
        elif key == ord('2'):
            detector.roi_ratio = 0.5
            print("ROI: 50%")
        elif key == ord('3'):
            detector.roi_ratio = 0.7
            print("ROI: 70%")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='循迹线检测')
    parser.add_argument('--camera', type=int, default=0)
    parser.add_argument('--color', type=str, default='black',
                       choices=['black', 'red', 'blue', 'green'])
    args = parser.parse_args()
    run_demo(args.camera, args.color)
