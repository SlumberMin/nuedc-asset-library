#!/usr/bin/env python3
"""
虚拟鼠标模块
============
功能：通过手指追踪控制鼠标光标，捏合手势实现点击。
依赖：opencv-python, mediapipe, pyautogui
用法：python virtual_mouse.py
"""

import cv2
import math
import numpy as np
import mediapipe as mp
import pyautogui

# pyautogui 安全设置
pyautogui.FAILSAFE = True
pyautogui.PAUSE = 0.01


class VirtualMouse:
    """虚拟鼠标：食指移动光标，拇指+食指捏合点击"""

    def __init__(self, cam_id=0, width=640, height=480, smoothing=5):
        self.cam_id = cam_id
        self.width = width
        self.height = height
        self.smoothing = smoothing  # 坐标平滑窗口

        # 屏幕分辨率
        self.screen_w, self.screen_h = pyautogui.size()

        # MediaPipe
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5,
        )
        self.mp_draw = mp.solutions.drawing_utils

        # 坐标平滑缓冲
        self.pos_buffer = []
        self.click_cooldown = 0  # 防止连续点击

    def _smooth_pos(self, x, y):
        """对光标坐标做滑动平均平滑"""
        self.pos_buffer.append((x, y))
        if len(self.pos_buffer) > self.smoothing:
            self.pos_buffer.pop(0)
        avg_x = int(np.mean([p[0] for p in self.pos_buffer]))
        avg_y = int(np.mean([p[1] for p in self.pos_buffer]))
        return avg_x, avg_y

    def run(self):
        """主循环"""
        cap = cv2.VideoCapture(self.cam_id)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        # 定义工作区域边距（避免到达屏幕边缘时失控）
        margin = 80

        print("[虚拟鼠标] 按 'q' 退出")
        print("  ☝️ 食指移动 → 控制光标")
        print("  🤏 拇指+食指捏合 → 点击")

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            frame = cv2.flip(frame, 1)
            h, w, _ = frame.shape
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.hands.process(rgb)

            if results.multi_hand_landmarks:
                hand_lm = results.multi_hand_landmarks[0]
                self.mp_draw.draw_landmarks(frame, hand_lm, self.mp_hands.HAND_CONNECTIONS)
                landmarks = hand_lm.landmark

                # 食指尖(8)坐标 → 映射到屏幕
                ix, iy = int(landmarks[8].x * w), int(landmarks[8].y * h)
                # 将摄像头坐标映射到屏幕坐标（带边距）
                screen_x = np.interp(ix, (margin, w - margin), (0, self.screen_w))
                screen_y = np.interp(iy, (margin, h - margin), (0, self.screen_h))
                screen_x, screen_y = self._smooth_pos(screen_x, screen_y)

                # 移动光标
                pyautogui.moveTo(screen_x, screen_y)

                # 拇指尖(4)与食指尖(8)距离 → 判断点击
                tx, ty = int(landmarks[4].x * w), int(landmarks[4].y * h)
                dist = math.hypot(ix - tx, iy - ty)

                # 绘制指尖
                cv2.circle(frame, (ix, iy), 10, (0, 255, 0), -1)
                cv2.circle(frame, (tx, ty), 10, (0, 0, 255), -1)
                cv2.line(frame, (ix, iy), (tx, ty), (255, 255, 0), 2)

                # 捏合检测
                if dist < 40:
                    if self.click_cooldown == 0:
                        pyautogui.click()
                        self.click_cooldown = 15  # 冷却帧数
                        cv2.putText(frame, "CLICK!", (ix - 30, iy - 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)

                # 绘制距离信息
                cv2.putText(frame, f'Dist: {int(dist)}', (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)

            if self.click_cooldown > 0:
                self.click_cooldown -= 1

            cv2.imshow("Virtual Mouse", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()


# ════════════════════════════════════════════
#  使用示例
# ════════════════════════════════════════════
if __name__ == '__main__':
    mouse = VirtualMouse(cam_id=0, smoothing=5)
    mouse.run()
