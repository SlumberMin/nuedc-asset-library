#!/usr/bin/env python3
"""
手语识别模块
============
功能：通过手指姿态识别基本手语手势，输出对应文字。
依赖：opencv-python, mediapipe, numpy
用法：python sign_language_recognition.py

说明：基于手指伸展/弯曲状态进行简单手势分类，
     可扩展为深度学习模型实现更复杂的识别。
"""

import cv2
import numpy as np
import mediapipe as mp


class SignLanguageRecognizer:
    """手语识别器：基于手指关键点的姿态分类"""

    # 手势标签映射（基于手指伸展模式）
    GESTURE_MAP = {
        'fist': '握拳',
        'open_palm': '你好/停止',
        'thumbs_up': '好的/赞同',
        'peace': '胜利/二',
        'point_up': '一/指',
        'ok': 'OK/可以',
        'rock': '摇滚/六',
        'three': '三',
        'four': '四',
        'call_me': '打电话',
    }

    def __init__(self, cam_id=0, width=640, height=480):
        self.cam_id = cam_id
        self.width = width
        self.height = height

        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5,
        )
        self.mp_draw = mp.solutions.drawing_utils

        # 手指关键点索引
        # 拇指：tip=4, ip=3, mcp=2
        # 食指：tip=8, dip=7, pip=6, mcp=5
        # 中指：tip=12, dip=11, pip=10, mcp=9
        # 无名指：tip=16, dip=15, pip=14, mcp=13
        # 小指：tip=20, dip=19, pip=18, mcp=17
        self.tip_ids = [4, 8, 12, 16, 20]

    def _is_finger_extended(self, landmarks, finger_tip, finger_pip, finger_mcp):
        """判断手指是否伸展（tip 在 pip 上方）"""
        return landmarks[finger_tip].y < landmarks[finger_pip].y

    def _is_thumb_extended(self, landmarks):
        """拇指特殊处理：水平方向判断"""
        # 拇指尖 x 坐标与拇指 MCP 比较（右手：tip.x < mcp.x 表示伸展）
        return abs(landmarks[4].x - landmarks[3].x) > 0.04

    def _get_finger_states(self, landmarks):
        """获取五根手指的伸展状态"""
        fingers = []
        # 拇指（水平判断）
        fingers.append(1 if self._is_thumb_extended(landmarks) else 0)
        # 其余四指（垂直判断）
        for tip, pip, mcp in [(8, 6, 5), (12, 10, 9), (16, 14, 13), (20, 18, 17)]:
            fingers.append(1 if self._is_finger_extended(landmarks, tip, pip, mcp) else 0)
        return fingers  # [拇指, 食指, 中指, 无名指, 小指]

    def _classify_gesture(self, fingers):
        """根据手指状态分类手势"""
        thumb, index, middle, ring, pinky = fingers

        if sum(fingers) == 0:
            return 'fist'
        if sum(fingers) == 5:
            return 'open_palm'
        if thumb and not index and not middle and not ring and not pinky:
            return 'thumbs_up'
        if not thumb and index and middle and not ring and not pinky:
            return 'peace'
        if not thumb and index and not middle and not ring and not pinky:
            return 'point_up'
        if thumb and index and not middle and not ring and not pinky:
            return 'ok'
        if thumb and index and not middle and not ring and pinky:
            return 'rock'
        if not thumb and index and middle and ring and not pinky:
            return 'three'
        if not thumb and index and middle and ring and pinky:
            return 'four'
        if thumb and not index and not middle and not ring and pinky:
            return 'call_me'
        return 'unknown'

    def run(self):
        """主循环"""
        cap = cv2.VideoCapture(self.cam_id)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        print("[手语识别] 按 'q' 退出")
        print("  支持手势：握拳、你好、OK、胜利、点赞 等")

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            frame = cv2.flip(frame, 1)
            h, w, _ = frame.shape
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.hands.process(rgb)

            gesture_key = 'none'
            gesture_text = '未检测到手'

            if results.multi_hand_landmarks:
                hand_lm = results.multi_hand_landmarks[0]
                self.mp_draw.draw_landmarks(frame, hand_lm, self.mp_hands.HAND_CONNECTIONS)
                landmarks = hand_lm.landmark

                fingers = self._get_finger_states(landmarks)
                gesture_key = self._classify_gesture(fingers)
                gesture_text = self.GESTURE_MAP.get(gesture_key, '未知手势')

                # 显示手指状态
                finger_names = ['拇指', '食指', '中指', '无名指', '小指']
                for i, (name, state) in enumerate(zip(finger_names, fingers)):
                    color = (0, 255, 0) if state else (0, 0, 255)
                    cv2.putText(frame, f'{name}:{"伸" if state else "曲"}',
                                (w - 200, 30 + i * 25),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

            # 显示识别结果
            cv2.rectangle(frame, (10, 10), (350, 70), (0, 0, 0), -1)
            cv2.putText(frame, f'{gesture_text}', (20, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 2)

            cv2.imshow("Sign Language Recognition", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()


# ════════════════════════════════════════════
#  使用示例
# ════════════════════════════════════════════
if __name__ == '__main__':
    recognizer = SignLanguageRecognizer(cam_id=0)
    recognizer.run()
