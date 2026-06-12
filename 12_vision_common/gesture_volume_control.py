#!/usr/bin/env python3
"""
手势音量控制模块
================
功能：通过手部拇指与食指距离映射系统音量，实时可视化。
依赖：opencv-python, mediapipe, pycaw, comtypes
用法：python gesture_volume_control.py
"""

import cv2
import math
import numpy as np
import mediapipe as mp
from ctypes import cast, POINTER
from comtypes import CLSCTX_ALL
from pycaw.pycaw import AudioUtilities, IAudioEndpointVolume


class GestureVolumeController:
    """手势音量控制器：通过拇指-食指距离控制音量"""

    def __init__(self, cam_id=0, width=640, height=480):
        # ── 摄像头参数 ──
        self.cam_id = cam_id
        self.width = width
        self.height = height

        # ── MediaPipe 手部检测 ──
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5,
        )
        self.mp_draw = mp.solutions.drawing_utils

        # ── 系统音量控制（Windows pycaw） ──
        devices = AudioUtilities.GetSpeakers()
        interface = devices.Activate(IAudioEndpointVolume._iid_, CLSCTX_ALL, None)
        self.volume_ctrl = cast(interface, POINTER(IAudioEndpointVolume))
        self.vol_range = self.volume_ctrl.GetVolumeRange()  # (min, max, step)
        self.vol_min, self.vol_max = self.vol_range[0], self.vol_range[1]

        # ── 距离映射范围（像素） ──
        self.dist_min = 30    # 拇指-食指最近距离（对应最大音量）
        self.dist_max = 200   # 拇指-食指最远距离（对应最小音量）

        # ── 平滑滤波 ──
        self.smooth_vol = 0   # 上一次音量百分比

    def _calc_distance(self, p1, p2):
        """计算两个关键点之间的欧氏距离"""
        return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

    def _map_range(self, value, in_min, in_max, out_min, out_max):
        """将值从一个范围线性映射到另一个范围"""
        value = max(in_min, min(in_max, value))
        return (value - in_min) / (in_max - in_min) * (out_max - out_min) + out_min

    def _draw_volume_bar(self, img, vol_percent):
        """在画面左侧绘制音量条"""
        bar_x, bar_y = 50, 100
        bar_w, bar_h = 35, 300
        fill_h = int(bar_h * vol_percent / 100)

        # 背景框
        cv2.rectangle(img, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (200, 200, 200), 2)
        # 填充（颜色随音量变化：低=绿，高=红）
        color = (0, int(255 * (1 - vol_percent / 100)), int(255 * vol_percent / 100))
        cv2.rectangle(img, (bar_x, bar_y + bar_h - fill_h),
                      (bar_x + bar_w, bar_y + bar_h), color, -1)
        # 百分比文字
        cv2.putText(img, f'{int(vol_percent)}%', (bar_x - 5, bar_y - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        return img

    def run(self):
        """主循环：打开摄像头并实时处理"""
        cap = cv2.VideoCapture(self.cam_id)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        print("[手势音量控制] 按 'q' 退出")
        print("  👍 拇指与食指靠近 → 音量增大")
        print("  🤏 拇指与食指远离 → 音量减小")

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            frame = cv2.flip(frame, 1)  # 镜像翻转
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.hands.process(rgb)

            vol_percent = self.smooth_vol  # 默认保持上一帧

            if results.multi_hand_landmarks:
                hand_lm = results.multi_hand_landmarks[0]
                self.mp_draw.draw_landmarks(frame, hand_lm, self.mp_hands.HAND_CONNECTIONS)

                # 提取拇指尖(4)和食指尖(8)坐标
                h, w, _ = frame.shape
                landmarks = hand_lm.landmark
                thumb_tip = (int(landmarks[4].x * w), int(landmarks[4].y * h))
                index_tip = (int(landmarks[8].x * w), int(landmarks[8].y * h))

                # 绘制指尖连线
                cv2.circle(frame, thumb_tip, 8, (255, 0, 255), -1)
                cv2.circle(frame, index_tip, 8, (255, 0, 255), -1)
                cv2.line(frame, thumb_tip, index_tip, (0, 255, 0), 2)

                # 计算距离并映射音量
                dist = self._calc_distance(thumb_tip, index_tip)
                raw_vol = self._map_range(dist, self.dist_min, self.dist_max, 0, 100)

                # 指尖越近 → 音量越大（反转映射逻辑，保持直觉）
                vol_percent = 100 - self._map_range(dist, self.dist_min, self.dist_max, 0, 100)

                # 平滑滤波（避免抖动）
                self.smooth_vol = self.smooth_vol * 0.7 + vol_percent * 0.3
                vol_percent = self.smooth_vol

                # 设置系统音量
                sys_vol = self._map_range(vol_percent, 0, 100, self.vol_min, self.vol_max)
                self.volume_ctrl.SetMasterVolumeLevel(sys_vol, None)

            # 绘制音量条
            self._draw_volume_bar(frame, vol_percent)

            cv2.imshow("Gesture Volume Control", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()


# ════════════════════════════════════════════
#  使用示例
# ════════════════════════════════════════════
if __name__ == '__main__':
    controller = GestureVolumeController(cam_id=0)
    controller.run()
