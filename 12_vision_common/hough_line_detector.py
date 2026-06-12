#!/usr/bin/env python3
"""
Hough变换线检测器
功能：标准Hough变换 + 概率Hough变换 + 自适应线段合并
适用：OpenCV + Orange Pi 5
支持：
  - 霍夫变换直线检测 (HoughLines)
  - 概率霍夫变换线段检测 (HoughLinesP)
  - 自适应阈值
  - 线段合并与过滤
"""

import cv2
import numpy as np
import math


class HoughLineDetector:
    """Hough变换线检测器"""

    def __init__(self, method='probabilistic', rho=1, theta=np.pi/180,
                 threshold=50, min_line_length=50, max_line_gap=10,
                 merge_angle_threshold=10, merge_distance_threshold=20):
        """
        参数:
            method: 'standard' | 'probabilistic'
            rho: 距离分辨率(像素)
            theta: 角度分辨率(弧度)
            threshold: 累加器阈值
            min_line_length: 最小线段长度(概率法)
            max_line_gap: 最大线段间隙(概率法)
            merge_angle_threshold: 合并角度阈值(度)
            merge_distance_threshold: 合并距离阈值(像素)
        """
        self.method = method
        self.rho = rho
        self.theta = theta
        self.threshold = threshold
        self.min_line_length = min_line_length
        self.max_line_gap = max_line_gap
        self.merge_angle_threshold = merge_angle_threshold
        self.merge_distance_threshold = merge_distance_threshold

    def preprocess(self, frame):
        """预处理：灰度化 + 高斯模糊 + Canny边缘"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if len(frame.shape) == 3 else frame
        blurred = cv2.GaussianBlur(gray, (5, 5), 1.5)
        edges = cv2.Canny(blurred, 50, 150)
        return edges

    def detect_raw(self, edges):
        """
        原始Hough线检测
        返回: lines列表
        """
        if self.method == 'probabilistic':
            lines = cv2.HoughLinesP(
                edges, self.rho, self.theta, self.threshold,
                minLineLength=self.min_line_length,
                maxLineGap=self.max_line_gap
            )
        else:
            lines = cv2.HoughLines(edges, self.rho, self.theta, self.threshold)

        return lines

    def _line_distance(self, line1, line2):
        """计算两条线段端点间的最小距离"""
        x1, y1, x2, y2 = line1
        x3, y3, x4, y4 = line2
        d1 = math.sqrt((x1-x3)**2 + (y1-y3)**2)
        d2 = math.sqrt((x1-x4)**2 + (y1-y4)**2)
        d3 = math.sqrt((x2-x3)**2 + (y2-y3)**2)
        d4 = math.sqrt((x2-x4)**2 + (y2-y4)**2)
        return min(d1, d2, d3, d4)

    def _line_angle(self, line):
        """计算线段角度(度)"""
        x1, y1, x2, y2 = line
        angle = math.degrees(math.atan2(y2-y1, x2-x1))
        return angle % 180  # 归一化到[0, 180)

    def merge_lines(self, lines):
        """
        合并相近的线段
        合并条件: 角度差 < 阈值 且 距离 < 阈值
        """
        if lines is None or len(lines) == 0:
            return []

        if self.method != 'probabilistic':
            # 标准HoughLines返回(rho, theta)格式，不合并
            return [(l[0], l[1]) for l in lines]

        # 转换为(x1,y1,x2,y2)格式
        segments = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            segments.append((x1, y1, x2, y2))

        merged = []
        used = [False] * len(segments)

        for i in range(len(segments)):
            if used[i]:
                continue
            group = [segments[i]]
            used[i] = True
            angle_i = self._line_angle(segments[i])

            for j in range(i + 1, len(segments)):
                if used[j]:
                    continue
                angle_j = self._line_angle(segments[j])
                angle_diff = abs(angle_i - angle_j)
                if angle_diff > 90:
                    angle_diff = 180 - angle_diff

                if angle_diff < self.merge_angle_threshold:
                    dist = self._line_distance(segments[i], segments[j])
                    if dist < self.merge_distance_threshold:
                        group.append(segments[j])
                        used[j] = True

            # 合并: 取端点最远的两个点作为合并后线段
            if len(group) > 1:
                all_pts = []
                for seg in group:
                    all_pts.extend([(seg[0], seg[1]), (seg[2], seg[3])])
                # 找最远的两个点
                max_dist = 0
                best_pair = (all_pts[0], all_pts[1])
                for p1 in all_pts:
                    for p2 in all_pts:
                        d = (p1[0]-p2[0])**2 + (p1[1]-p2[1])**2
                        if d > max_dist:
                            max_dist = d
                            best_pair = (p1, p2)
                merged.append((best_pair[0][0], best_pair[0][1],
                              best_pair[1][0], best_pair[1][1]))
            else:
                merged.append(group[0])

        return merged

    def detect(self, frame):
        """
        完整检测流程
        返回:
            merged_lines: 合并后的线段列表 [(x1,y1,x2,y2), ...]
            raw_lines: 原始检测结果
        """
        edges = self.preprocess(frame)
        raw_lines = self.detect_raw(edges)
        merged = self.merge_lines(raw_lines)
        return merged, raw_lines

    def detect_with_mask(self, frame, mask=None):
        """带掩膜的线检测"""
        if mask is not None:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if len(frame.shape) == 3 else frame
            edges = cv2.Canny(gray, 50, 150)
            edges = cv2.bitwise_and(edges, mask)
        else:
            edges = self.preprocess(frame)
        raw_lines = self.detect_raw(edges)
        merged = self.merge_lines(raw_lines)
        return merged, raw_lines

    def draw_lines(self, frame, lines, color=(0, 255, 0), thickness=2):
        """在图像上绘制线段"""
        vis = frame.copy()
        for line in lines:
            if self.method == 'probabilistic':
                x1, y1, x2, y2 = line
                cv2.line(vis, (int(x1), int(y1)), (int(x2), int(y2)), color, thickness)
            else:
                rho, theta = line
                a, b = math.cos(theta), math.sin(theta)
                x0, y0 = a * rho, b * rho
                pt1 = (int(x0 + 1000 * (-b)), int(y0 + 1000 * a))
                pt2 = (int(x0 - 1000 * (-b)), int(y0 - 1000 * a))
                cv2.line(vis, pt1, pt2, color, thickness)
        return vis

    def get_dominant_line(self, lines):
        """获取最长的主线段"""
        if not lines or self.method != 'probabilistic':
            return None
        max_len = 0
        dominant = None
        for line in lines:
            x1, y1, x2, y2 = line
            length = math.sqrt((x2-x1)**2 + (y2-y1)**2)
            if length > max_len:
                max_len = length
                dominant = line
        return dominant

    def get_line_angle(self, line):
        """获取线段角度(度, 相对于水平方向)"""
        if self.method == 'probabilistic':
            x1, y1, x2, y2 = line
            return math.degrees(math.atan2(y2-y1, x2-x1))
        else:
            rho, theta = line
            return math.degrees(theta) - 90


def demo():
    """演示Hough线检测"""
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("无法打开摄像头")
        return

    detector = HoughLineDetector(method='probabilistic',
                                  min_line_length=30, max_line_gap=10)
    print("Hough线检测器启动 (按q退出)")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        merged, raw = detector.detect(frame)
        vis = detector.draw_lines(frame, merged, color=(0, 255, 0))

        cv2.putText(vis, f"Lines: {len(merged)}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        cv2.imshow("Hough Line Detector", vis)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    demo()
