#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
人行横道检测模块 (Pedestrian Crossing Detection)

功能：
    - 斑马线检测（条纹纹理分析+方向滤波+连通域分析）
    - 行人等待检测（人形轮廓检测+等待区域判定）
    - 安全通行判断（行人位置+移动趋势分析）

适用场景：智能交通信号控制、自动驾驶行人保护、过街安全预警
"""

import cv2
import numpy as np
from typing import Tuple, Optional, List, Dict
from dataclasses import dataclass
from enum import Enum


class CrossingState(Enum):
    """人行横道状态"""
    CLEAR = "clear"            # 畅通
    PEDESTRIAN_WAITING = "waiting"   # 有行人等待
    PEDESTRIAN_CROSSING = "crossing"  # 有行人正在通过
    WARNING = "warning"         # 预警状态


@dataclass
class Pedestrian:
    """行人检测结果"""
    pedestrian_id: int
    bbox: Tuple[int, int, int, int]  # (x, y, w, h)
    center: Tuple[int, int]
    is_on_crossing: bool       # 是否在斑马线上
    is_waiting: bool           # 是否在等待
    velocity: Tuple[float, float] = (0.0, 0.0)  # 估计速度


@dataclass
class CrossingDetection:
    """人行横道检测结果"""
    crossing_found: bool
    crossing_mask: Optional[np.ndarray]    # 斑马线掩码
    crossing_bbox: Optional[Tuple[int, int, int, int]]  # 斑马线边界框
    crossing_angle: float                   # 斑马线角度
    pedestrians: List[Pedestrian]
    state: CrossingState
    confidence: float


class PedestrianCrossingDetector:
    """人行横道检测系统"""

    def __init__(self,
                 stripe_min_length: int = 30,
                 stripe_max_gap: int = 15,
                 min_stripes: int = 4,
                 pedestrian_detector: str = "hog",
                 waiting_roi_ratio: Tuple[float, float, float, float] = (0.0, 0.3, 1.0, 0.7)):
        """
        初始化人行横道检测系统

        参数：
            stripe_min_length: 最小条纹长度
            stripe_max_gap: 条纹最大间隔
            min_stripes: 最少条纹数量（判定为斑马线的最小值）
            pedestrian_detector: 行人检测器类型 ("hog" 或 "contour")
            waiting_roi_ratio: 等待区域比例 (x, y, w, h)
        """
        self.stripe_min_length = stripe_min_length
        self.stripe_max_gap = stripe_max_gap
        self.min_stripes = min_stripes
        self.pedestrian_detector = pedestrian_detector
        self.waiting_roi_ratio = waiting_roi_ratio

        # HOG行人检测器
        self.hog = None
        if pedestrian_detector == "hog":
            self.hog = cv2.HOGDescriptor()
            self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        # 上一帧行人跟踪
        self.prev_pedestrians: List[Pedestrian] = []
        self.next_id = 1

    def _detect_crossing_stripes(self, frame: np.ndarray) -> Tuple[np.ndarray, float]:
        """
        检测斑马线条纹

        算法思路：
            1. 灰度化 + 边缘检测
            2. 方向滤波（提取水平方向的条纹）
            3. 形态学操作连接条纹
            4. 连通域分析过滤

        参数：
            frame: BGR输入图像

        返回：
            (crossing_mask, angle): 斑马线掩码和角度
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 自适应阈值处理
        binary = cv2.adaptiveThreshold(
            gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY, 31, 10
        )

        # 形态学操作：提取水平条纹
        # 使用较长的水平核
        h_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (25, 3))
        h_stripes = cv2.morphologyEx(binary, cv2.MORPH_OPEN, h_kernel)

        # 使用稍短的核保留更多条纹
        h_kernel2 = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 2))
        h_stripes2 = cv2.morphologyEx(binary, cv2.MORPH_OPEN, h_kernel2)

        # 合并
        stripes = cv2.bitwise_or(h_stripes, h_stripes2)

        # 膨胀连接断裂的条纹
        dilate_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        stripes = cv2.dilate(stripes, dilate_kernel, iterations=2)

        # 连通域分析
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
            stripes, connectivity=8
        )

        # 过滤连通域
        valid_mask = np.zeros_like(stripes)
        valid_count = 0

        for i in range(1, num_labels):  # 跳过背景
            area = stats[i, cv2.CC_STAT_AREA]
            width = stats[i, cv2.CC_STAT_WIDTH]
            height = stats[i, cv2.CC_STAT_HEIGHT]

            # 条纹应该是宽而窄的
            if area < 200:
                continue
            if width < self.stripe_min_length:
                continue
            aspect = width / max(height, 1)
            if aspect < 2.0:  # 条纹宽高比应较大
                continue

            valid_mask[labels == i] = 255
            valid_count += 1

        # 判断是否为斑马线
        crossing_mask = np.zeros_like(stripes)
        angle = 0.0

        if valid_count >= self.min_stripes:
            crossing_mask = valid_mask

            # 计算条纹平均角度
            lines = cv2.HoughLinesP(valid_mask, 1, np.pi / 180,
                                     threshold=20, minLineLength=20, maxLineGap=10)
            if lines is not None:
                angles = []
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    a = np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi
                    if abs(a) < 30:  # 近水平
                        angles.append(a)
                if angles:
                    angle = np.median(angles)

        return crossing_mask, angle

    def _find_crossing_bbox(self, crossing_mask: np.ndarray) -> Optional[Tuple[int, int, int, int]]:
        """计算斑马线边界框"""
        contours, _ = cv2.findContours(crossing_mask, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        # 合并所有轮廓
        all_points = np.vstack(contours)
        x, y, w, h = cv2.boundingRect(all_points)
        return (x, y, w, h)

    def _detect_pedestrians_hog(self, frame: np.ndarray) -> List[Tuple[int, int, int, int]]:
        """使用HOG+SVM检测行人"""
        if self.hog is None:
            return []

        # 缩小图像提高速度
        h, w = frame.shape[:2]
        scale = min(1.0, 640 / max(w, h))
        if scale < 1.0:
            small = cv2.resize(frame, None, fx=scale, fy=scale)
        else:
            small = frame

        # 检测
        rects, weights = self.hog.detectMultiScale(
            small,
            winStride=(8, 8),
            padding=(4, 4),
            scale=1.05
        )

        # NMS
        if len(rects) == 0:
            return []

        # 缩放回原尺寸
        detections = []
        for (x, y, w, h) in rects:
            detections.append((
                int(x / scale), int(y / scale),
                int(w / scale), int(h / scale)
            ))

        # 简单NMS
        detections = self._nms_rects(detections, 0.4)
        return detections

    def _detect_pedestrians_contour(self, frame: np.ndarray) -> List[Tuple[int, int, int, int]]:
        """基于轮廓的行人检测（适用于简单场景）"""
        # 背景差分或运动检测
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)

        # 边缘检测
        edges = cv2.Canny(blur, 50, 150)

        # 膨胀连接
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 10))
        dilated = cv2.dilate(edges, kernel, iterations=2)

        # 查找轮廓
        contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)

        detections = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 1000 or area > 50000:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            aspect = h / max(w, 1)

            # 人形轮廓：高大于宽，宽高比在合理范围
            if 1.5 < aspect < 5.0 and w > 20 and h > 50:
                detections.append((x, y, w, h))

        return detections

    def _detect_pedestrians(self, frame: np.ndarray) -> List[Tuple[int, int, int, int]]:
        """检测行人"""
        if self.pedestrian_detector == "hog":
            return self._detect_pedestrians_hog(frame)
        else:
            return self._detect_pedestrians_contour(frame)

    @staticmethod
    def _nms_rects(rects: List[Tuple[int, int, int, int]],
                   threshold: float = 0.4) -> List[Tuple[int, int, int, int]]:
        """矩形NMS"""
        if not rects:
            return []

        rects_array = np.array(rects)
        x1 = rects_array[:, 0]
        y1 = rects_array[:, 1]
        x2 = x1 + rects_array[:, 2]
        y2 = y1 + rects_array[:, 3]
        areas = rects_array[:, 2] * rects_array[:, 3]

        # 按面积排序
        order = areas.argsort()[::-1]
        keep = []

        while order.size > 0:
            i = order[0]
            keep.append(i)

            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])

            inter = np.maximum(0, xx2 - xx1) * np.maximum(0, yy2 - yy1)
            iou = inter / (areas[i] + areas[order[1:]] - inter)

            inds = np.where(iou <= threshold)[0]
            order = order[inds + 1]

        return [rects[i] for i in keep]

    def _check_pedestrian_on_crossing(self, ped_bbox: Tuple[int, int, int, int],
                                       crossing_mask: np.ndarray) -> bool:
        """检查行人是否在斑马线上"""
        if crossing_mask is None or crossing_mask.size == 0:
            return False

        x, y, w, h = ped_bbox
        ch, cw = crossing_mask.shape[:2]

        # 行人脚部区域（底部1/4）
        foot_y = min(y + int(h * 0.75), ch - 1)
        foot_y_end = min(y + h, ch)
        foot_x = max(x, 0)
        foot_x_end = min(x + w, cw)

        if foot_y >= foot_y_end or foot_x >= foot_x_end:
            return False

        foot_region = crossing_mask[foot_y:foot_y_end, foot_x:foot_x_end]
        if foot_region.size == 0:
            return False

        # 如果脚部区域有斑马线像素，则认为在斑马线上
        white_ratio = np.sum(foot_region > 0) / foot_region.size
        return white_ratio > 0.1

    def _check_waiting(self, ped: Pedestrian,
                       crossing_bbox: Optional[Tuple[int, int, int, int]],
                       frame_shape: Tuple[int, int]) -> bool:
        """判断行人是否在等待"""
        if crossing_bbox is None:
            return False

        h, w = frame_shape
        cx, cy = ped.center
        bx, by, bw, bh = crossing_bbox

        # 等待区域：斑马线两侧
        margin = int(bh * 0.5)
        waiting_zone = (
            bx - margin,
            max(0, by - margin),
            bw + 2 * margin,
            bh + 2 * margin
        )

        # 行人在等待区域但不在斑马线上
        in_zone = (waiting_zone[0] <= cx <= waiting_zone[0] + waiting_zone[2] and
                   waiting_zone[1] <= cy <= waiting_zone[1] + waiting_zone[3])

        return in_zone and not ped.is_on_crossing

    def _update_pedestrians(self, detections: List[Tuple[int, int, int, int]],
                            crossing_mask: np.ndarray,
                            crossing_bbox: Optional[Tuple[int, int, int, int]],
                            frame_shape: Tuple[int, int]) -> List[Pedestrian]:
        """更新行人列表（含简单跟踪）"""
        pedestrians = []

        for bbox in detections:
            x, y, w, h = bbox
            center = (x + w // 2, y + h // 2)

            on_crossing = self._check_pedestrian_on_crossing(bbox, crossing_mask)

            ped = Pedestrian(
                pedestrian_id=self.next_id,
                bbox=bbox,
                center=center,
                is_on_crossing=on_crossing,
                is_waiting=False
            )
            ped.is_waiting = self._check_waiting(ped, crossing_bbox, frame_shape)

            # 简单跟踪：与上一帧最近的行人匹配
            if self.prev_pedestrians:
                min_dist = float('inf')
                matched = None
                for prev in self.prev_pedestrians:
                    dist = np.sqrt((center[0] - prev.center[0]) ** 2 +
                                   (center[1] - prev.center[1]) ** 2)
                    if dist < min_dist and dist < 100:
                        min_dist = dist
                        matched = prev

                if matched:
                    ped.pedestrian_id = matched.pedestrian_id
                    ped.velocity = (
                        center[0] - matched.center[0],
                        center[1] - matched.center[1]
                    )
                else:
                    self.next_id += 1
            else:
                self.next_id += 1

            pedestrians.append(ped)

        self.prev_pedestrians = pedestrians
        return pedestrians

    def _determine_state(self, pedestrians: List[Pedestrian],
                         crossing_found: bool) -> CrossingState:
        """判断人行横道整体状态"""
        if not crossing_found:
            return CrossingState.CLEAR

        crossing_peds = [p for p in pedestrians if p.is_on_crossing]
        waiting_peds = [p for p in pedestrians if p.is_waiting]

        if crossing_peds:
            return CrossingState.PEDESTRIAN_CROSSING
        elif waiting_peds:
            return CrossingState.PEDESTRIAN_WAITING
        elif pedestrians:
            return CrossingState.WARNING
        else:
            return CrossingState.CLEAR

    def detect(self, frame: np.ndarray) -> CrossingDetection:
        """
        检测人行横道和行人

        参数：
            frame: BGR格式输入图像

        返回：
            CrossingDetection 检测结果
        """
        h, w = frame.shape[:2]

        # 1. 检测斑马线条纹
        crossing_mask, angle = self._detect_crossing_stripes(frame)
        crossing_found = np.sum(crossing_mask > 0) > 500

        # 2. 计算斑马线边界框
        crossing_bbox = None
        if crossing_found:
            crossing_bbox = self._find_crossing_bbox(crossing_mask)

        # 3. 检测行人
        detections = self._detect_pedestrians(frame)

        # 4. 更新行人列表
        pedestrians = self._update_pedestrians(
            detections, crossing_mask, crossing_bbox, (h, w)
        )

        # 5. 判断状态
        state = self._determine_state(pedestrians, crossing_found)

        # 6. 计算置信度
        if crossing_found:
            stripe_area = np.sum(crossing_mask > 0)
            total_area = h * w
            confidence = min(stripe_area / (total_area * 0.05), 1.0)
        else:
            confidence = 0.0

        return CrossingDetection(
            crossing_found=crossing_found,
            crossing_mask=crossing_mask,
            crossing_bbox=crossing_bbox,
            crossing_angle=angle,
            pedestrians=pedestrians,
            state=state,
            confidence=confidence
        )

    def draw_results(self, frame: np.ndarray,
                     detection: CrossingDetection) -> np.ndarray:
        """在图像上绘制检测结果"""
        result = frame.copy()

        # 绘制斑马线区域
        if detection.crossing_found and detection.crossing_bbox:
            x, y, w, h = detection.crossing_bbox

            # 半透明覆盖
            overlay = result.copy()
            state_colors = {
                CrossingState.CLEAR: (0, 200, 0),
                CrossingState.PEDESTRIAN_WAITING: (0, 200, 200),
                CrossingState.PEDESTRIAN_CROSSING: (0, 0, 200),
                CrossingState.WARNING: (0, 200, 200),
            }
            color = state_colors.get(detection.state, (200, 200, 200))
            cv2.rectangle(overlay, (x, y), (x + w, y + h), color, -1)
            cv2.addWeighted(overlay, 0.2, result, 0.8, 0, result)
            cv2.rectangle(result, (x, y), (x + w, y + h), color, 2)

        # 绘制行人
        for ped in detection.pedestrians:
            px, py, pw, ph = ped.bbox

            if ped.is_on_crossing:
                color = (0, 0, 255)     # 红色：在斑马线上
                label = "CROSSING"
            elif ped.is_waiting:
                color = (0, 200, 200)   # 黄色：等待中
                label = "WAITING"
            else:
                color = (200, 200, 200) # 白色：其他
                label = f"#{ped.pedestrian_id}"

            cv2.rectangle(result, (px, py), (px + pw, py + ph), color, 2)
            cv2.putText(result, label, (px, py - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

            # 绘制速度箭头
            if abs(ped.velocity[0]) > 1 or abs(ped.velocity[1]) > 1:
                cx, cy = ped.center
                end_x = int(cx + ped.velocity[0] * 3)
                end_y = int(cy + ped.velocity[1] * 3)
                cv2.arrowedLine(result, (cx, cy), (end_x, end_y),
                                (0, 255, 0), 2)

        # 状态文本
        state_text = {
            CrossingState.CLEAR: "状态: 畅通",
            CrossingState.PEDESTRIAN_WAITING: "状态: 有行人等待",
            CrossingState.PEDESTRIAN_CROSSING: "状态: 有行人通过!",
            CrossingState.WARNING: "状态: 预警"
        }

        text = state_text.get(detection.state, "状态: 未知")
        text_color = (0, 200, 0) if detection.state == CrossingState.CLEAR else (0, 0, 255)
        cv2.putText(result, text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, text_color, 2)

        ped_count = len(detection.pedestrians)
        cv2.putText(result, f"Pedestrians: {ped_count}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        return result

    def process_frame(self, frame: np.ndarray) -> Tuple[np.ndarray, CrossingDetection]:
        """
        处理单帧（一体化接口）

        参数：
            frame: BGR格式输入图像

        返回：
            (result_frame, detection): 绘制结果的图像和检测结果
        """
        detection = self.detect(frame)
        result = self.draw_results(frame, detection)
        return result, detection


# ======================== 使用示例 ========================

def example_image():
    """单张图片人行横道检测示例"""
    img_path = "crossing.jpg"
    frame = cv2.imread(img_path)

    if frame is None:
        print(f"无法读取图像: {img_path}")
        print("生成模拟斑马线图像...")
        # 创建模拟图像
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        frame[:] = (60, 60, 60)  # 深灰路面

        # 绘制斑马线（白色条纹）
        for i in range(6):
            y = 180 + i * 25
            cv2.rectangle(frame, (100, y), (540, y + 15), (220, 220, 220), -1)

        # 模拟一个行人
        cv2.rectangle(frame, (300, 140), (320, 200), (0, 100, 200), -1)
        cv2.circle(frame, (310, 130), 12, (0, 100, 200), -1)

    detector = PedestrianCrossingDetector(
        min_stripes=3,
        pedestrian_detector="contour"
    )

    result, detection = detector.detect(frame)
    result = detector.draw_results(frame, detection)

    print("=== 人行横道检测结果 ===")
    print(f"检测到斑马线: {'是' if detection.crossing_found else '否'}")
    print(f"置信度: {detection.confidence:.2%}")
    print(f"斑马线角度: {detection.crossing_angle:.1f}度")
    print(f"检测到行人: {len(detection.pedestrians)}个")

    state_names = {
        CrossingState.CLEAR: "畅通",
        CrossingState.PEDESTRIAN_WAITING: "有行人等待",
        CrossingState.PEDESTRIAN_CROSSING: "有行人正在通过",
        CrossingState.WARNING: "预警"
    }
    print(f"横道状态: {state_names.get(detection.state, '未知')}")

    for ped in detection.pedestrians:
        on = "在斑马线上" if ped.is_on_crossing else "不在斑马线上"
        waiting = "等待中" if ped.is_waiting else ""
        print(f"  行人#{ped.pedestrian_id}: {on} {waiting}")

    cv2.imshow("Pedestrian Crossing Detection", result)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def example_camera():
    """摄像头实时人行横道检测"""
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("无法打开摄像头")
        return

    detector = PedestrianCrossingDetector(
        min_stripes=3,
        pedestrian_detector="hog"
    )

    print("人行横道检测系统已启动，按 'q' 退出")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        result, detection = detector.process_frame(frame)

        state_names = {
            CrossingState.CLEAR: "CLEAR",
            CrossingState.PEDESTRIAN_WAITING: "WAITING",
            CrossingState.PEDESTRIAN_CROSSING: "CROSSING",
            CrossingState.WARNING: "WARNING"
        }
        print(f"\r状态: {state_names.get(detection.state, '?')} | "
              f"行人: {len(detection.pedestrians)}", end="")

        cv2.imshow("Pedestrian Crossing", result)
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
