#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
车道偏离预警模块 (Lane Departure Warning)

功能：
    - 车道线检测（Canny边缘+Hough直线检测）
    - 车道线拟合（最小二乘拟合左右车道线）
    - 偏离量计算（车辆中心与车道中心偏移）
    - 预警判断（偏移阈值报警）

适用场景：智能车赛道、自动驾驶辅助、车道保持辅助系统
"""

import cv2
import numpy as np
from typing import Tuple, Optional, List, Dict


class LaneDepartureWarning:
    """车道偏离预警系统"""

    # 偏离等级
    LEVEL_SAFE = "安全"
    LEVEL_WARNING = "警告"
    LEVEL_DANGER = "危险"

    def __init__(self,
                 canny_low: int = 50,
                 canny_high: int = 150,
                 hough_threshold: int = 50,
                 min_line_length: int = 100,
                 max_line_gap: int = 50,
                 roi_ratio: Tuple[float, float, float, float] = (0.0, 0.5, 1.0, 1.0),
                 warning_threshold: float = 0.15,
                 danger_threshold: float = 0.30):
        """
        初始化车道偏离预警系统

        参数：
            canny_low: Canny边缘检测低阈值
            canny_high: Canny边缘检测高阈值
            hough_threshold: 霍夫变换累加器阈值
            min_line_length: 最小线段长度
            max_line_gap: 最大线段间隔
            roi_ratio: 感兴趣区域比例 (x, y, w, h)，归一化到[0,1]
            warning_threshold: 偏离警告阈值（相对图像宽度比例）
            danger_threshold: 偏离危险阈值（相对图像宽度比例）
        """
        self.canny_low = canny_low
        self.canny_high = canny_high
        self.hough_threshold = hough_threshold
        self.min_line_length = min_line_length
        self.max_line_gap = max_line_gap
        self.roi_ratio = roi_ratio
        self.warning_threshold = warning_threshold
        self.danger_threshold = danger_threshold

        # 上一帧车道线（用于平滑）
        self.prev_left_line: Optional[np.ndarray] = None
        self.prev_right_line: Optional[np.ndarray] = None
        self.smooth_factor = 0.7  # 平滑系数

    def _preprocess(self, frame: np.ndarray) -> np.ndarray:
        """图像预处理：灰度化 + 高斯模糊 + Canny边缘"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, self.canny_low, self.canny_high)
        return edges

    def _get_roi_mask(self, shape: Tuple[int, int], edges: np.ndarray) -> np.ndarray:
        """生成感兴趣区域掩码（梯形区域，模拟透视效果）"""
        h, w = shape
        rx, ry, rw, rh = self.roi_ratio

        # 梯形ROI：底部宽、顶部窄
        mask = np.zeros_like(edges)
        polygon = np.array([[
            (int(w * 0.05), h),          # 左下
            (int(w * 0.45), int(h * ry)), # 左上
            (int(w * 0.55), int(h * ry)), # 右上
            (int(w * 0.95), h)           # 右下
        ]], dtype=np.int32)
        cv2.fillPoly(mask, polygon, 255)
        masked = cv2.bitwise_and(edges, mask)
        return masked

    def _detect_lines(self, edges: np.ndarray) -> np.ndarray:
        """霍夫变换检测直线"""
        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi / 180,
            threshold=self.hough_threshold,
            minLineLength=self.min_line_length,
            maxLineGap=self.max_line_gap
        )
        return lines

    def _classify_lines(self, lines: np.ndarray, img_width: int) -> Tuple[List, List]:
        """
        将检测到的线段分为左车道线和右车道线

        参数：
            lines: 检测到的线段集合
            img_width: 图像宽度

        返回：
            (left_lines, right_lines): 左右车道线列表
        """
        left_lines = []
        right_lines = []

        if lines is None:
            return left_lines, right_lines

        for line in lines:
            x1, y1, x2, y2 = line[0]
            if x2 == x1:
                continue  # 跳过垂直线

            slope = (y2 - y1) / (x2 - x1)

            # 斜率过滤：排除近水平线
            if abs(slope) < 0.3:
                continue

            if slope < 0:
                left_lines.append(line[0])  # 左车道线斜率为负
            else:
                right_lines.append(line[0])  # 右车道线斜率为正

        return left_lines, right_lines

    def _fit_line(self, lines: List, img_shape: Tuple[int, int]) -> Optional[np.ndarray]:
        """
        最小二乘拟合车道线

        参数：
            lines: 线段列表
            img_shape: 图像形状 (h, w)

        返回：
            拟合后的直线参数 [slope, intercept] 或 None
        """
        if not lines:
            return None

        x_coords = []
        y_coords = []
        for line in lines:
            x1, y1, x2, y2 = line
            x_coords.extend([x1, x2])
            y_coords.extend([y1, y2])

        if len(x_coords) < 2:
            return None

        # 最小二乘拟合：y = slope*x + intercept
        coeffs = np.polyfit(y_coords, x_coords, 1)  # 注意：x作为y的函数
        return coeffs

    def _smooth_line(self, current: Optional[np.ndarray],
                     previous: Optional[np.ndarray]) -> Optional[np.ndarray]:
        """时序平滑，减少帧间抖动"""
        if current is None:
            return previous
        if previous is None:
            return current
        return self.smooth_factor * previous + (1 - self.smooth_factor) * current

    def _calculate_departure(self, left_coeffs: Optional[np.ndarray],
                             right_coeffs: Optional[np.ndarray],
                             img_shape: Tuple[int, int]) -> Dict:
        """
        计算偏离量

        参数：
            left_coeffs: 左车道线系数
            right_coeffs: 右车道线系数
            img_shape: 图像形状 (h, w)

        返回：
            偏离信息字典
        """
        h, w = img_shape
        y_eval = h  # 在图像底部评估

        result = {
            "left_lane_detected": left_coeffs is not None,
            "right_lane_detected": right_coeffs is not None,
            "offset": 0.0,
            "offset_ratio": 0.0,
            "direction": "居中",
            "level": self.LEVEL_SAFE,
            "lane_center": w / 2,
            "vehicle_center": w / 2
        }

        if left_coeffs is not None and right_coeffs is not None:
            # 左右车道线都检测到：计算车道中心
            left_x = np.polyval(left_coeffs, y_eval)
            right_x = np.polyval(right_coeffs, y_eval)
            lane_center = (left_x + right_x) / 2
            vehicle_center = w / 2

            offset = vehicle_center - lane_center
            offset_ratio = offset / w

            result["lane_center"] = lane_center
            result["vehicle_center"] = vehicle_center
            result["offset"] = offset
            result["offset_ratio"] = offset_ratio

            if offset_ratio < -self.danger_threshold:
                result["direction"] = "严重偏左"
                result["level"] = self.LEVEL_DANGER
            elif offset_ratio < -self.warning_threshold:
                result["direction"] = "偏左"
                result["level"] = self.LEVEL_WARNING
            elif offset_ratio > self.danger_threshold:
                result["direction"] = "严重偏右"
                result["level"] = self.LEVEL_DANGER
            elif offset_ratio > self.warning_threshold:
                result["direction"] = "偏右"
                result["level"] = self.LEVEL_WARNING
            else:
                result["direction"] = "居中"
                result["level"] = self.LEVEL_SAFE

        elif left_coeffs is not None:
            result["direction"] = "仅检测到左车道线"
            result["level"] = self.LEVEL_WARNING
        elif right_coeffs is not None:
            result["direction"] = "仅检测到右车道线"
            result["level"] = self.LEVEL_WARNING
        else:
            result["direction"] = "未检测到车道线"
            result["level"] = self.LEVEL_DANGER

        return result

    def _draw_results(self, frame: np.ndarray,
                      left_coeffs: Optional[np.ndarray],
                      right_coeffs: Optional[np.ndarray],
                      departure_info: Dict) -> np.ndarray:
        """在图像上绘制车道线和预警信息"""
        h, w = frame.shape[:2]
        overlay = frame.copy()

        # 绘制左侧车道线（绿色）
        if left_coeffs is not None:
            y_bottom = h
            y_top = int(h * self.roi_ratio[1])
            x_bottom = int(np.polyval(left_coeffs, y_bottom))
            x_top = int(np.polyval(left_coeffs, y_top))
            cv2.line(overlay, (x_bottom, y_bottom), (x_top, y_top), (0, 255, 0), 3)

        # 绘制右侧车道线（绿色）
        if right_coeffs is not None:
            y_bottom = h
            y_top = int(h * self.roi_ratio[1])
            x_bottom = int(np.polyval(right_coeffs, y_bottom))
            x_top = int(np.polyval(right_coeffs, y_top))
            cv2.line(overlay, (x_bottom, y_bottom), (x_top, y_top), (0, 255, 0), 3)

        # 绘制车道中心线（蓝色虚线）
        if left_coeffs is not None and right_coeffs is not None:
            for y in range(int(h * self.roi_ratio[1]), h, 20):
                left_x = int(np.polyval(left_coeffs, y))
                right_x = int(np.polyval(right_coeffs, y))
                center_x = (left_x + right_x) // 2
                cv2.circle(overlay, (center_x, y), 2, (255, 0, 0), -1)

        # 绘制车辆中心线（黄色）
        cv2.line(overlay, (w // 2, h), (w // 2, int(h * 0.6)), (0, 255, 255), 2)

        # 绘制预警信息
        level = departure_info["level"]
        direction = departure_info["direction"]
        offset_ratio = departure_info["offset_ratio"]

        # 颜色映射
        color_map = {
            self.LEVEL_SAFE: (0, 255, 0),
            self.LEVEL_WARNING: (0, 255, 255),
            self.LEVEL_DANGER: (0, 0, 255)
        }
        text_color = color_map.get(level, (255, 255, 255))

        # 状态文本
        info_text = f"State: {direction} | Offset: {offset_ratio:.2%}"
        level_text = f"Level: {level}"

        cv2.putText(overlay, info_text, (10, 30),
                     cv2.FONT_HERSHEY_SIMPLEX, 0.8, text_color, 2)
        cv2.putText(overlay, level_text, (10, 65),
                     cv2.FONT_HERSHEY_SIMPLEX, 0.8, text_color, 2)

        # 危险时绘制红色边框警告
        if level == self.LEVEL_DANGER:
            cv2.rectangle(overlay, (5, 5), (w - 5, h - 5), (0, 0, 255), 4)

        # 半透明叠加
        result = cv2.addWeighted(overlay, 0.8, frame, 0.2, 0)
        return result

    def process_frame(self, frame: np.ndarray) -> Tuple[np.ndarray, Dict]:
        """
        处理单帧图像

        参数：
            frame: BGR格式的输入图像

        返回：
            (result_frame, departure_info): 绘制结果的图像和偏离信息字典
        """
        h, w = frame.shape[:2]

        # 1. 预处理
        edges = self._preprocess(frame)

        # 2. ROI提取
        roi_edges = self._get_roi_mask((h, w), edges)

        # 3. 霍夫变换检测直线
        lines = self._detect_lines(roi_edges)

        # 4. 分类左右车道线
        left_lines, right_lines = self._classify_lines(lines, w)

        # 5. 拟合车道线
        left_coeffs = self._fit_line(left_lines, (h, w))
        right_coeffs = self._fit_line(right_lines, (h, w))

        # 6. 时序平滑
        left_coeffs = self._smooth_line(left_coeffs, self.prev_left_line)
        right_coeffs = self._smooth_line(right_coeffs, self.prev_right_line)
        self.prev_left_line = left_coeffs
        self.prev_right_line = right_coeffs

        # 7. 计算偏离量
        departure_info = self._calculate_departure(left_coeffs, right_coeffs, (h, w))

        # 8. 绘制结果
        result_frame = self._draw_results(frame, left_coeffs, right_coeffs, departure_info)

        return result_frame, departure_info

    def reset(self):
        """重置内部状态"""
        self.prev_left_line = None
        self.prev_right_line = None


# ======================== 使用示例 ========================

def example_camera():
    """摄像头实时车道偏离预警示例"""
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("无法打开摄像头")
        return

    ldw = LaneDepartureWarning(
        warning_threshold=0.12,
        danger_threshold=0.25
    )

    print("车道偏离预警系统已启动，按 'q' 退出")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        result, info = ldw.process_frame(frame)

        print(f"\r偏离: {info['direction']} | "
              f"偏移率: {info['offset_ratio']:.2%} | "
              f"等级: {info['level']}", end="")

        cv2.imshow("Lane Departure Warning", result)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


def example_image():
    """单张图片车道偏离检测示例"""
    # 读取测试图像
    img_path = "road.jpg"
    frame = cv2.imread(img_path)
    if frame is None:
        print(f"无法读取图像: {img_path}")
        print("生成模拟测试图像...")
        # 生成模拟道路图像
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        frame[:] = (100, 100, 100)  # 灰色路面
        # 绘制模拟车道线
        cv2.line(frame, (200, 480), (280, 240), (255, 255, 255), 3)
        cv2.line(frame, (440, 480), (360, 240), (255, 255, 255), 3)

    ldw = LaneDepartureWarning()
    result, info = ldw.process_frame(frame)

    print("=== 车道偏离检测结果 ===")
    for key, value in info.items():
        print(f"  {key}: {value}")

    cv2.imshow("Result", result)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "--camera":
        example_camera()
    else:
        example_image()
