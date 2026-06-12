#!/usr/bin/env python3
"""
口罩检测模块
============
功能：检测人脸是否佩戴口罩，分析口罩区域并分类。
依赖：opencv-python, mediapipe, numpy
用法：python face_mask_detection.py

说明：基于肤色检测+下半脸遮挡分析的轻量方案。
     生产环境建议使用 YOLO/MobileNet 训练专用口罩检测模型。
"""

import cv2
import numpy as np
import mediapipe as mp


class FaceMaskDetector:
    """口罩检测器：基于面部关键点和肤色分析"""

    def __init__(self, cam_id=0, width=640, height=480):
        self.cam_id = cam_id
        self.width = width
        self.height = height

        # MediaPipe 面部网格（468个关键点）
        self.mp_face_mesh = mp.solutions.face_mesh
        self.face_mesh = self.mp_face_mesh.FaceMesh(
            static_image_mode=False,
            max_num_faces=5,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        self.mp_draw = mp.solutions.drawing_utils
        self.mp_styles = mp.solutions.drawing_styles

        # 口罩区域关键点索引（面部下半部分）
        # 下巴到鼻尖区域的关键点
        self.mask_region_indices = [
            10, 338, 297, 332, 284, 251, 389, 356, 454, 323, 361, 288,
            397, 365, 379, 378, 400, 377, 152, 148, 176, 149, 150, 136,
            172, 58, 132, 93, 234, 127, 162, 21, 54, 103, 67, 109
        ]

    def _extract_mask_region(self, frame, landmarks, w, h):
        """提取口罩区域（下半脸）并分析"""
        # 收集口罩区域顶点
        points = []
        for idx in self.mask_region_indices:
            if idx < len(landmarks):
                x = int(landmarks[idx].x * w)
                y = int(landmarks[idx].y * h)
                points.append([x, y])

        if len(points) < 10:
            return None, None

        points = np.array(points, dtype=np.int32)

        # 创建掩码
        mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        cv2.fillPoly(mask, [points], 255)

        # 提取区域
        roi = cv2.bitwise_and(frame, frame, mask=mask)

        return roi, points

    def _analyze_skin_ratio(self, roi, mask):
        """分析口罩区域的肤色占比（口罩会遮挡肤色）"""
        # HSV 肤色范围
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower_skin = np.array([0, 20, 70], dtype=np.uint8)
        upper_skin = np.array([20, 255, 255], dtype=np.uint8)
        skin_mask = cv2.inRange(hsv, lower_skin, upper_skin)

        # 在口罩区域内计算肤色像素占比
        total_pixels = cv2.countNonZero(mask)
        if total_pixels == 0:
            return 0
        skin_pixels = cv2.countNonZero(cv2.bitwise_and(skin_mask, mask))
        return skin_pixels / total_pixels

    def _analyze_texture(self, frame, landmarks, w, h):
        """分析下半脸区域的纹理特征（口罩=低纹理，皮肤=高纹理）"""
        # 取鼻尖下方到下巴的矩形区域
        nose_tip = landmarks[4]
        chin = landmarks[152]
        left_cheek = landmarks[234]
        right_cheek = landmarks[454]

        x1 = int(left_cheek.x * w) - 10
        x2 = int(right_cheek.x * w) + 10
        y1 = int(nose_tip.y * h) + 5
        y2 = int(chin.y * h)

        x1, x2 = max(0, x1), min(w, x2)
        y1, y2 = max(0, y1), min(h, y2)

        if x2 - x1 < 10 or y2 - y1 < 10:
            return 0

        roi = frame[y1:y2, x1:x2]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        laplacian_var = cv2.Laplacian(gray, cv2.CV_64F).var()
        return laplacian_var

    def _classify_mask(self, skin_ratio, texture_var):
        """根据肤色比例和纹理特征分类"""
        # 综合判断逻辑
        if skin_ratio < 0.15 and texture_var < 500:
            return 'masked', (0, 255, 0)     # 绿色=佩戴口罩
        elif skin_ratio < 0.3 and texture_var < 800:
            return 'partial', (0, 165, 255)  # 橙色=部分遮挡
        else:
            return 'no_mask', (0, 0, 255)    # 红色=未佩戴

    def run(self):
        """主循环"""
        cap = cv2.VideoCapture(self.cam_id)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        status_text = {
            'masked': '已佩戴口罩 ✓',
            'partial': '部分遮挡 ⚠',
            'no_mask': '未佩戴口罩 ✗',
        }

        print("[口罩检测] 按 'q' 退出")

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            frame = cv2.flip(frame, 1)
            h, w, _ = frame.shape
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.face_mesh.process(rgb)

            if results.multi_face_landmarks:
                for face_lm in results.multi_face_landmarks:
                    landmarks = face_lm.landmark

                    # 提取口罩区域并分析
                    roi, points = self._extract_mask_region(frame, landmarks, w, h)

                    if roi is not None:
                        # 分析肤色比例
                        mask_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
                        cv2.fillPoly(mask_mask, [points], 255)
                        skin_ratio = self._analyze_skin_ratio(roi, mask_mask)

                        # 分析纹理
                        texture_var = self._analyze_texture(frame, landmarks, w, h)

                        # 分类
                        status, color = self._classify_mask(skin_ratio, texture_var)

                        # 绘制口罩区域轮廓
                        cv2.polylines(frame, [points], True, color, 2)

                        # 显示状态
                        text = status_text[status]
                        cv2.putText(frame, text, (10, 40),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)

                        # 显示分析数据
                        cv2.putText(frame, f'Skin: {skin_ratio:.2f}  Tex: {texture_var:.0f}',
                                    (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
                    else:
                        # 无法提取区域，绘制面部网格
                        self.mp_draw.draw_landmarks(
                            image=frame,
                            landmark_list=face_lm,
                            connections=self.mp_face_mesh.FACEMESH_TESSELATION,
                            landmark_drawing_spec=None,
                            connection_drawing_spec=self.mp_styles.get_default_face_mesh_tesselation_style(),
                        )

            cv2.imshow("Face Mask Detection", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()


# ════════════════════════════════════════════
#  使用示例
# ════════════════════════════════════════════
if __name__ == '__main__':
    detector = FaceMaskDetector(cam_id=0)
    detector.run()
