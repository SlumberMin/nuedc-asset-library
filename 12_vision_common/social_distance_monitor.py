#!/usr/bin/env python3
"""
社交距离监测模块
================
功能：检测行人并估计其距离，对过近的行人发出预警。
依赖：opencv-python, mediapipe (可选), numpy
用法：python social_distance_monitor.py

说明：使用 HOG 行人检测器 + 透视变换估计距离。
     生产环境建议使用 YOLOv8 等深度学习模型提升精度。
"""

import cv2
import numpy as np
import math


class SocialDistanceMonitor:
    """社交距离监测器：行人检测 + 距离估计 + 预警"""

    def __init__(self, cam_id=0, width=640, height=480,
                 safe_distance=150, warn_distance=100):
        """
        参数：
            cam_id: 摄像头ID
            width, height: 画面尺寸
            safe_distance: 安全距离（像素），低于此值显示警告
            warn_distance: 危险距离（像素），低于此值显示红色警告
        """
        self.cam_id = cam_id
        self.width = width
        self.height = height
        self.safe_distance = safe_distance
        self.warn_distance = warn_distance

        # HOG 行人检测器
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        # 参考参数：用于像素到实际距离的近似转换
        # 假设摄像头高度约1.5m，视角约60度
        self.reference_height = 1.7  # 参考行人身高（米）
        self.focal_length = 600      # 估算焦距（像素）

    def _detect_pedestrians(self, frame):
        """使用 HOG+SVM 检测行人"""
        # 多尺度检测
        boxes, weights = self.hog.detectMultiScale(
            frame,
            winStride=(8, 8),
            padding=(4, 4),
            scale=1.05,
        )
        # 非极大值抑制
        indices = cv2.dnn.NMSBoxes(
            [tuple(b) for b in boxes],
            weights.tolist(),
            score_threshold=0.5,
            nms_threshold=0.4,
        )
        if len(indices) == 0:
            return []
        return [boxes[i] for i in indices.flatten()]

    def _estimate_distance(self, box_height):
        """根据行人框高度估算实际距离（米）"""
        if box_height < 10:
            return float('inf')
        # 简单针孔模型：距离 = (实际身高 × 焦距) / 像素高度
        distance = (self.reference_height * self.focal_length) / box_height
        return distance

    def _calc_box_center(self, box):
        """计算检测框中心点"""
        x, y, w, h = box
        return (x + w // 2, y + h)

    def _calc_pixel_distance(self, center1, center2):
        """计算两个中心点的像素距离"""
        return math.hypot(center1[0] - center2[0], center1[1] - center2[1])

    def _check_pairwise_distances(self, centers, distances_real):
        """检查所有行人对的距离，找出违规对"""
        violations = []
        n = len(centers)
        for i in range(n):
            for j in range(i + 1, n):
                pixel_dist = self._calc_pixel_distance(centers[i], centers[j])
                # 使用两个行人距离的平均值作为参考
                avg_real_dist = (distances_real[i] + distances_real[j]) / 2

                # 基于实际距离判断（更准确的方式）
                # 简化：用像素距离和行框高度综合判断
                if pixel_dist < self.warn_distance:
                    violations.append((i, j, 'danger', pixel_dist))
                elif pixel_dist < self.safe_distance:
                    violations.append((i, j, 'warning', pixel_dist))
        return violations

    def _draw_info_panel(self, frame, num_people, violations):
        """绘制信息面板"""
        panel_h = 80
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (350, panel_h), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)

        # 人数统计
        cv2.putText(frame, f'People: {num_people}', (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # 违规统计
        danger_count = sum(1 for v in violations if v[2] == 'danger')
        warn_count = sum(1 for v in violations if v[2] == 'warning')

        if danger_count > 0:
            cv2.putText(frame, f'DANGER: {danger_count}', (10, 55),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        elif warn_count > 0:
            cv2.putText(frame, f'WARNING: {warn_count}', (10, 55),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
        else:
            cv2.putText(frame, 'SAFE', (10, 55),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        return frame

    def run(self):
        """主循环"""
        cap = cv2.VideoCapture(self.cam_id)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        print("[社交距离监测] 按 'q' 退出")
        print(f"  安全距离阈值: {self.safe_distance}px")
        print(f"  危险距离阈值: {self.warn_distance}px")

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            frame = cv2.flip(frame, 1)

            # 检测行人
            boxes = self._detect_pedestrians(frame)

            centers = []
            real_distances = []

            # 绘制每个行人
            for box in boxes:
                x, y, w, h = box
                center = self._calc_box_center(box)
                centers.append(center)

                # 估算实际距离
                real_dist = self._estimate_distance(h)
                real_distances.append(real_dist)

                # 默认绿色框
                color = (0, 255, 0)
                cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
                cv2.putText(frame, f'{real_dist:.1f}m', (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                # 绘制底部中心点
                cv2.circle(frame, center, 4, (255, 0, 0), -1)

            # 检查行人间距离
            violations = self._check_pairwise_distances(centers, real_distances)

            # 绘制违规连线
            for i, j, level, dist in violations:
                color = (0, 0, 255) if level == 'danger' else (0, 165, 255)
                cv2.line(frame, centers[i], centers[j], color, 2)
                mid_x = (centers[i][0] + centers[j][0]) // 2
                mid_y = (centers[i][1] + centers[j][1]) // 2
                cv2.putText(frame, f'{int(dist)}px', (mid_x - 20, mid_y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

                # 高亮违规行人
                for idx in [i, j]:
                    bx, by, bw, bh = boxes[idx]
                    cv2.rectangle(frame, (bx, by), (bx + bw, by + bh), color, 3)

            # 绘制信息面板
            self._draw_info_panel(frame, len(boxes), violations)

            cv2.imshow("Social Distance Monitor", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()


# ════════════════════════════════════════════
#  使用示例
# ════════════════════════════════════════════
if __name__ == '__main__':
    monitor = SocialDistanceMonitor(
        cam_id=0,
        safe_distance=150,   # 安全距离阈值（像素）
        warn_distance=100,   # 危险距离阈值（像素）
    )
    monitor.run()
