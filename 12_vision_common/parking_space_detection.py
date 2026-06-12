#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
车位检测模块 (Parking Space Detection)

功能：
    - 透视变换（鸟瞰图生成）
    - 车位线检测（边缘+形态学+霍夫变换）
    - 空闲判断（车位区域纹理/颜色分析）
    - 支持平行/垂直/斜列三种车位类型

适用场景：智能停车场系统、自动泊车辅助、停车场管理
"""

import cv2
import numpy as np
from typing import Tuple, Optional, List, Dict
from dataclasses import dataclass, field
from enum import Enum


class ParkingType(Enum):
    """车位类型"""
    PARALLEL = "平行式"     # 侧方停车
    PERPENDICULAR = "垂直式"  # 倒车入库
    ANGULAR = "斜列式"       # 斜角停车


@dataclass
class ParkingSpace:
    """车位检测结果"""
    space_id: int                   # 车位编号
    corners: np.ndarray             # 四个角点坐标
    is_occupied: bool               # 是否被占用
    confidence: float               # 检测置信度
    occupancy_score: float          # 占用评分（0=空闲，1=占用）
    parking_type: ParkingType       # 车位类型
    center: Tuple[int, int] = (0, 0)  # 车位中心
    area: float = 0.0               # 车位面积


class ParkingSpaceDetector:
    """车位检测系统"""

    def __init__(self,
                 parking_type: ParkingType = ParkingType.PERPENDICULAR,
                 space_width: int = 80,
                 space_height: int = 160,
                 occupancy_threshold: float = 0.45,
                 edge_threshold: float = 0.15,
                 src_points: Optional[np.ndarray] = None,
                 dst_size: Tuple[int, int] = (400, 600)):
        """
        初始化车位检测系统

        参数：
            parking_type: 车位类型（平行/垂直/斜列）
            space_width: 透视变换后车位宽度（像素）
            space_height: 透视变换后车位高度（像素）
            occupancy_threshold: 占用判定阈值
            edge_threshold: 边缘密度阈值
            src_points: 透视变换源点（4个点），None则自动估计
            dst_size: 透视变换后图像尺寸 (w, h)
        """
        self.parking_type = parking_type
        self.space_width = space_width
        self.space_height = space_height
        self.occupancy_threshold = occupancy_threshold
        self.edge_threshold = edge_threshold
        self.src_points = src_points
        self.dst_size = dst_size

        # 预计算透视变换矩阵
        self.M = None
        self.M_inv = None
        if src_points is not None:
            self._compute_transform(src_points)

    def _compute_transform(self, src_points: np.ndarray):
        """计算透视变换矩阵"""
        dw, dh = self.dst_size
        dst_points = np.float32([
            [0, 0],
            [dw, 0],
            [dw, dh],
            [0, dh]
        ])
        src = np.float32(src_points)
        self.M = cv2.getPerspectiveTransform(src, dst_points)
        self.M_inv = cv2.getPerspectiveTransform(dst_points, src)

    def set_roi_points(self, points: List[Tuple[int, int]]):
        """
        设置透视变换的四个源点（图像中的梯形区域）

        参数：
            points: 4个点坐标，顺序为 [左上, 右上, 右下, 左下]
        """
        assert len(points) == 4, "需要恰好4个点"
        self.src_points = np.float32(points)
        self._compute_transform(self.src_points)

    def _warp_perspective(self, frame: np.ndarray) -> np.ndarray:
        """透视变换，生成鸟瞰图"""
        if self.M is None:
            # 自动估计：取图像中心区域的梯形
            h, w = frame.shape[:2]
            self.src_points = np.float32([
                [w * 0.15, h * 0.55],  # 左上
                [w * 0.85, h * 0.55],  # 右上
                [w * 1.0, h * 1.0],    # 右下
                [w * 0.0, h * 1.0]     # 左下
            ])
            self._compute_transform(self.src_points)

        warped = cv2.warpPerspective(frame, self.M, self.dst_size,
                                      flags=cv2.INTER_LINEAR)
        return warped

    def _detect_parking_lines(self, warped: np.ndarray) -> np.ndarray:
        """
        在鸟瞰图中检测车位线

        参数：
            warped: 鸟瞰图

        返回：
            车位线掩码
        """
        # 转灰度
        gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)

        # 自适应阈值（适应不同光照）
        binary = cv2.adaptiveThreshold(
            gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV, 25, 10
        )

        # 形态学操作：强化车位线
        # 水平方向核（提取水平车位线）
        h_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (25, 1))
        h_lines = cv2.morphologyEx(binary, cv2.MORPH_OPEN, h_kernel)

        # 垂直方向核（提取垂直车位线）
        v_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 25))
        v_lines = cv2.morphologyEx(binary, cv2.MORPH_OPEN, v_kernel)

        # 合并
        line_mask = cv2.bitwise_or(h_lines, v_lines)

        # 膨胀使线条更明显
        dilate_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        line_mask = cv2.dilate(line_mask, dilate_kernel, iterations=1)

        return line_mask

    def _find_parking_spaces(self, line_mask: np.ndarray,
                              warped_shape: Tuple[int, int]) -> List[Dict]:
        """
        从车位线掩码中提取车位区域

        参数：
            line_mask: 车位线掩码
            warped_shape: 鸟瞰图尺寸 (h, w)

        返回：
            车位区域列表
        """
        h, w = warped_shape
        spaces = []

        # 霍夫变换检测直线段
        lines = cv2.HoughLinesP(line_mask, 1, np.pi / 180,
                                 threshold=30, minLineLength=30, maxLineGap=10)

        if lines is None:
            # 备选方案：基于网格划分
            return self._grid_based_detection(warped_shape)

        # 提取水平线和垂直线
        h_lines = []
        v_lines = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            angle = abs(np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi)
            length = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

            if angle < 20 and length > 30:
                h_lines.append((y1 + y2) / 2)  # 水平线y坐标
            elif angle > 70 and length > 30:
                v_lines.append((x1 + x2) / 2)  # 垂直线x坐标

        # 排序去重
        h_lines = sorted(set(int(y) for y in h_lines))
        v_lines = sorted(set(int(x) for x in v_lines))

        # 合并相近的线（间距<20像素视为同一条）
        h_lines = self._merge_close_lines(h_lines, min_gap=20)
        v_lines = self._merge_close_lines(v_lines, min_gap=20)

        # 根据车位类型生成车位区域
        if self.parking_type == ParkingType.PERPENDICULAR:
            # 垂直式：垂直线之间的区域
            for i in range(len(v_lines) - 1):
                x_left = v_lines[i]
                x_right = v_lines[i + 1]
                space_w = x_right - x_left

                if space_w < self.space_width * 0.5 or space_w > self.space_width * 2:
                    continue

                # 从水平线确定y范围
                y_top = h_lines[0] if h_lines else int(h * 0.1)
                y_bottom = h_lines[-1] if h_lines else int(h * 0.9)

                corners = np.array([
                    [x_left, y_top],
                    [x_right, y_top],
                    [x_right, y_bottom],
                    [x_left, y_bottom]
                ])
                spaces.append({
                    "corners": corners,
                    "center": ((x_left + x_right) // 2, (y_top + y_bottom) // 2),
                    "width": space_w,
                    "height": y_bottom - y_top
                })

        elif self.parking_type == ParkingType.PARALLEL:
            # 平行式：水平线之间的区域
            for i in range(len(h_lines) - 1):
                y_top = h_lines[i]
                y_bottom = h_lines[i + 1]
                space_h = y_bottom - y_top

                if space_h < self.space_height * 0.3 or space_h > self.space_height * 2:
                    continue

                x_left = v_lines[0] if v_lines else int(w * 0.1)
                x_right = v_lines[-1] if v_lines else int(w * 0.9)

                # 按车位宽度分割
                num_spaces = max(1, int((x_right - x_left) / self.space_width))
                seg_width = (x_right - x_left) // num_spaces

                for j in range(num_spaces):
                    x1 = x_left + j * seg_width
                    x2 = x1 + seg_width
                    corners = np.array([
                        [x1, y_top], [x2, y_top],
                        [x2, y_bottom], [x1, y_bottom]
                    ])
                    spaces.append({
                        "corners": corners,
                        "center": ((x1 + x2) // 2, (y_top + y_bottom) // 2),
                        "width": seg_width,
                        "height": space_h
                    })

        return spaces

    def _grid_based_detection(self, shape: Tuple[int, int]) -> List[Dict]:
        """基于网格的车位检测（备用方案）"""
        h, w = shape
        spaces = []

        if self.parking_type == ParkingType.PERPENDICULAR:
            cols = max(1, w // self.space_width)
            rows = max(1, h // self.space_height)
            for r in range(rows):
                for c in range(cols):
                    x1 = c * self.space_width
                    y1 = r * self.space_height
                    x2 = x1 + self.space_width
                    y2 = y1 + self.space_height
                    corners = np.array([
                        [x1, y1], [x2, y1], [x2, y2], [x1, y2]
                    ])
                    spaces.append({
                        "corners": corners,
                        "center": ((x1 + x2) // 2, (y1 + y2) // 2),
                        "width": self.space_width,
                        "height": self.space_height
                    })
        return spaces

    @staticmethod
    def _merge_close_lines(lines: List[int], min_gap: int = 20) -> List[int]:
        """合并间距过近的线"""
        if not lines:
            return lines

        merged = [lines[0]]
        for line in lines[1:]:
            if line - merged[-1] >= min_gap:
                merged.append(line)
        return merged

    def _check_occupancy(self, warped: np.ndarray,
                          corners: np.ndarray) -> Tuple[bool, float]:
        """
        判断车位是否被占用

        分析方法：
            1. 边缘密度：占用的车位内部边缘更多（车辆轮廓）
            2. 颜色方差：占用的车位内部颜色变化更大
            3. 纹理复杂度：占用的车位纹理更复杂

        参数：
            warped: 鸟瞰图
            corners: 车位角点

        返回：
            (is_occupied, score): 是否占用，占用评分
        """
        h, w = warped.shape[:2]

        # 创建车位区域掩码
        mask = np.zeros((h, w), dtype=np.uint8)
        pts = corners.reshape((-1, 1, 2)).astype(np.int32)
        cv2.fillPoly(mask, [pts], 255)

        # 提取车位区域
        roi_mask = mask > 0
        if np.sum(roi_mask) == 0:
            return False, 0.0

        gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)

        # 1. 边缘密度分析
        edges = cv2.Canny(gray, 50, 150)
        edge_in_roi = edges[roi_mask]
        edge_density = np.sum(edge_in_roi > 0) / edge_in_roi.size if edge_in_roi.size > 0 else 0

        # 2. 颜色方差分析
        gray_roi = gray[roi_mask]
        color_std = np.std(gray_roi) / 255.0 if gray_roi.size > 0 else 0

        # 3. 颜色均值分析（空车位通常颜色均匀）
        color_mean = np.mean(gray_roi) / 255.0 if gray_roi.size > 0 else 0

        # 综合评分
        # 边缘密度权重较高（车辆有丰富的边缘信息）
        score = 0.5 * min(edge_density / self.edge_threshold, 1.0) + \
                0.3 * min(color_std * 3, 1.0) + \
                0.2 * (1.0 - abs(color_mean - 0.4))  # 偏暗/偏亮都可能有车

        score = np.clip(score, 0.0, 1.0)
        is_occupied = score > self.occupancy_threshold

        return is_occupied, float(score)

    def detect(self, frame: np.ndarray) -> Tuple[np.ndarray, List[ParkingSpace]]:
        """
        检测车位

        参数：
            frame: BGR格式输入图像

        返回：
            (bird_view, spaces): 鸟瞰图和车位列表
        """
        # 1. 透视变换
        warped = self._warp_perspective(frame)

        # 2. 检测车位线
        line_mask = self._detect_parking_lines(warped)

        # 3. 提取车位区域
        raw_spaces = self._find_parking_spaces(line_mask, warped.shape[:2])

        # 4. 判断每个车位状态
        spaces = []
        for i, raw in enumerate(raw_spaces):
            corners = raw["corners"]
            is_occupied, score = self._check_occupancy(warped, corners)

            space = ParkingSpace(
                space_id=i + 1,
                corners=corners,
                is_occupied=is_occupied,
                confidence=0.7 + 0.3 * (1 - abs(score - 0.5) * 2),
                occupancy_score=score,
                parking_type=self.parking_type,
                center=raw["center"],
                area=raw["width"] * raw["height"]
            )
            spaces.append(space)

        return warped, spaces

    def draw_results(self, warped: np.ndarray,
                     spaces: List[ParkingSpace]) -> np.ndarray:
        """在鸟瞰图上绘制检测结果"""
        result = warped.copy()

        for space in spaces:
            pts = space.corners.reshape((-1, 1, 2)).astype(np.int32)

            # 空闲=绿色，占用=红色
            color = (0, 200, 0) if not space.is_occupied else (0, 0, 200)
            fill_color = (0, 255, 0, 50) if not space.is_occupied else (0, 0, 255, 50)

            # 绘制车位边界
            cv2.polylines(result, [pts], True, color, 2)

            # 半透明填充
            overlay = result.copy()
            cv2.fillPoly(overlay, [pts], color[:3])
            cv2.addWeighted(overlay, 0.3, result, 0.7, 0, result)

            # 标注信息
            cx, cy = space.center
            status = "FREE" if not space.is_occupied else "OCCUPIED"
            label = f"#{space.space_id} {status}"
            cv2.putText(result, label, (cx - 30, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(result, f"{space.occupancy_score:.0%}",
                        (cx - 15, cy + 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        # 统计信息
        total = len(spaces)
        free = sum(1 for s in spaces if not s.is_occupied)
        info = f"Parking: {free}/{total} free"
        cv2.putText(result, info, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)

        return result

    def draw_on_original(self, frame: np.ndarray,
                         spaces: List[ParkingSpace]) -> np.ndarray:
        """将检测结果映射回原图"""
        if self.M_inv is None:
            return frame

        # 先在鸟瞰图上绘制
        warped = self._warp_perspective(frame)
        drawn = self.draw_results(warped, spaces)

        # 逆透视变换
        result = cv2.warpPerspective(drawn, self.M_inv,
                                      (frame.shape[1], frame.shape[0]))
        return result

    def process_frame(self, frame: np.ndarray) -> Tuple[np.ndarray, List[ParkingSpace]]:
        """
        处理单帧（一体化接口）

        参数：
            frame: BGR格式输入图像

        返回：
            (result_frame, spaces): 鸟瞰图结果和车位列表
        """
        warped, spaces = self.detect(frame)
        result = self.draw_results(warped, spaces)
        return result, spaces


# ======================== 使用示例 ========================

def example_image():
    """单张图片车位检测示例"""
    img_path = "parking_lot.jpg"
    frame = cv2.imread(img_path)

    if frame is None:
        print(f"无法读取图像: {img_path}")
        print("生成模拟停车场鸟瞰图...")
        # 创建模拟鸟瞰图
        frame = np.zeros((600, 400, 3), dtype=np.uint8)
        frame[:] = (60, 60, 60)  # 深灰路面

        # 绘制车位线
        for i in range(5):
            x = 20 + i * 80
            cv2.rectangle(frame, (x, 50), (x + 60, 250), (200, 200, 200), 2)
            cv2.rectangle(frame, (x, 300), (x + 60, 500), (200, 200, 200), 2)

        # 模拟一辆车（在车位2中）
        cv2.rectangle(frame, (105, 55), (155, 245), (80, 80, 120), -1)
        cv2.rectangle(frame, (265, 305), (315, 495), (80, 80, 120), -1)

    detector = ParkingSpaceDetector(
        parking_type=ParkingType.PERPENDICULAR,
        space_width=80,
        space_height=200,
        occupancy_threshold=0.40
    )

    result, spaces = detector.process_frame(frame)

    print("=== 车位检测结果 ===")
    print(f"共检测到 {len(spaces)} 个车位")
    for space in spaces:
        status = "占用" if space.is_occupied else "空闲"
        print(f"  车位#{space.space_id}: {status} "
              f"(评分: {space.occupancy_score:.2%})")

    free_count = sum(1 for s in spaces if not s.is_occupied)
    print(f"\n空闲车位: {free_count}/{len(spaces)}")

    cv2.imshow("Parking Space Detection", result)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def example_camera():
    """摄像头实时车位检测"""
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("无法打开摄像头")
        return

    detector = ParkingSpaceDetector(
        parking_type=ParkingType.PERPENDICULAR
    )

    print("车位检测系统已启动，按 'q' 退出")
    print("首次运行请用鼠标点击4个点设置ROI（左上->右上->右下->左下）")

    roi_points = []
    setup_done = False

    def mouse_callback(event, x, y, flags, param):
        nonlocal roi_points, setup_done
        if event == cv2.EVENT_LBUTTONDOWN and not setup_done:
            roi_points.append((x, y))
            print(f"  点 {len(roi_points)}: ({x}, {y})")
            if len(roi_points) == 4:
                detector.set_roi_points(roi_points)
                setup_done = True
                print("ROI设置完成！")

    cv2.namedWindow("Parking Detection")
    cv2.setMouseCallback("Parking Detection", mouse_callback)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        if setup_done:
            result, spaces = detector.process_frame(frame)
            free = sum(1 for s in spaces if not s.is_occupied)
            print(f"\r空闲车位: {free}/{len(spaces)}", end="")
        else:
            result = frame.copy()
            for pt in roi_points:
                cv2.circle(result, pt, 5, (0, 255, 0), -1)
            cv2.putText(result, f"Click ROI points ({len(roi_points)}/4)",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        cv2.imshow("Parking Detection", result)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "--camera":
        example_camera()
    else:
        example_image()
