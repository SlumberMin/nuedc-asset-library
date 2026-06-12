#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
手势识别模块 - Gesture Recognition
====================================
针对 Orange Pi 5 优化的手势识别算法
包含：肤色检测、轮廓分析、凸包缺陷、手势分类

技术栈：OpenCV + NumPy + 多线程优化
适配：Orange Pi 5 (RK3588S) / Linux ARM64

作者：nuedc-asset-library
"""

import cv2
import numpy as np
import threading
import time
from collections import deque


class GestureRecognizer:
    """
    手势识别器

    支持手势类型：
    - 拳头（握拳）
    - 张开手掌（五指）
    - 竖起食指（数字1）
    - 竖起两指（数字2/V字手势）
    - 竖起三指（数字3）
    - OK手势
    - 挥手动作

    使用示例：
        recognizer = GestureRecognizer()
        gesture, info = recognizer.recognize(frame)
        print(f"识别到: {gesture}")
    """

    def __init__(self, roi=None):
        """
        初始化手势识别器

        参数：
            roi: 感兴趣区域 (x, y, w, h)，None表示全图
        """
        # ==================== ROI设置 ====================
        self.roi = roi or (100, 100, 400, 400)  # 默认ROI

        # ==================== 肤色检测参数（HSV空间）====================
        # 通用肤色范围
        self.skin_lower_hsv = np.array([0, 20, 70])
        self.skin_upper_hsv = np.array([20, 255, 255])

        # YCrCb空间肤色范围
        self.skin_lower_ycrcb = np.array([0, 133, 77])
        self.skin_upper_ycrcb = np.array([255, 173, 127])

        # ==================== 手势识别参数 ====================
        self.min_contour_area = 5000     # 最小轮廓面积
        self.max_contour_area = 100000   # 最大轮廓面积
        self.convexity_defect_depth_min = 20   # 凸缺陷最小深度
        self.finger_angle_thresh = 80    # 手指角度阈值（度）

        # ==================== 状态历史 ====================
        self.gesture_history = deque(maxlen=10)
        self.hand_positions = deque(maxlen=30)  # 用于挥手检测

        # ==================== 手势映射 ====================
        self.gesture_names = {
            'fist': '握拳',
            'palm': '张开手掌',
            'one': '数字1/食指',
            'two': '数字2/V字',
            'three': '数字3',
            'four': '数字4',
            'five': '数字5',
            'ok': 'OK手势',
            'wave': '挥手',
            'none': '未检测到',
        }

        # ==================== 多线程 ====================
        self._lock = threading.Lock()

        print("[手势识别] 初始化完成")
        print(f"[手势识别] ROI区域: {self.roi}")

    def set_roi(self, x, y, w, h):
        """设置感兴趣区域"""
        self.roi = (x, y, w, h)
        print(f"[手势识别] ROI更新: {self.roi}")

    def detect_skin(self, frame):
        """
        肤色检测（多色彩空间融合）

        使用 HSV + YCrCb 双色彩空间提高鲁棒性

        参数：
            frame: BGR图像

        返回：
            mask: 肤色掩码
        """
        # 转换色彩空间
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        ycrcb = cv2.cvtColor(frame, cv2.COLOR_BGR2YCrCb)

        # HSV肤色检测
        mask_hsv = cv2.inRange(hsv, self.skin_lower_hsv, self.skin_upper_hsv)

        # YCrCb肤色检测
        mask_ycrcb = cv2.inRange(ycrcb, self.skin_lower_ycrcb, self.skin_upper_ycrcb)

        # 融合两个掩码
        mask = cv2.bitwise_and(mask_hsv, mask_ycrcb)

        # 形态学操作：去噪和平滑
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        # 高斯模糊平滑
        mask = cv2.GaussianBlur(mask, (5, 5), 0)

        return mask

    def find_hand_contour(self, mask):
        """
        找到最大的手部轮廓

        参数：
            mask: 肤色掩码

        返回：
            contour: 手部轮廓（最大轮廓）
            area: 轮廓面积
        """
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None, 0

        # 找到最大轮廓
        max_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(max_contour)

        # 面积过滤
        if area < self.min_contour_area or area > self.max_contour_area:
            return None, area

        # 凸包平滑
        epsilon = 0.001 * cv2.arcLength(max_contour, True)
        smoothed = cv2.approxPolyDP(max_contour, epsilon, True)

        return smoothed, area

    def analyze_hand(self, contour):
        """
        分析手部轮廓特征

        参数：
            contour: 手部轮廓

        返回：
            info: 手部特征字典 {
                'hull': 凸包,
                'hull_defects': 凸缺陷,
                'fingers': 手指数,
                'center': 中心点,
                'radius': 外接圆半径,
                'convexity': 凸性,
            }
        """
        if contour is None:
            return None

        # 凸包
        hull = cv2.convexHull(contour, returnPoints=False)

        # 凸缺陷
        try:
            defects = cv2.convexityDefects(contour, hull)
        except cv2.error:
            defects = None

        # 手部中心
        M = cv2.moments(contour)
        if M['m00'] == 0:
            return None

        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])

        # 外接圆
        (rx, ry), radius = cv2.minEnclosingCircle(contour)
        radius = int(radius)

        # 计算手指数量
        fingers, finger_points = self._count_fingers(contour, defects)

        # 计算凸性
        hull_area = cv2.contourArea(cv2.convexHull(contour))
        contour_area = cv2.contourArea(contour)
        convexity = contour_area / hull_area if hull_area > 0 else 0

        return {
            'hull': cv2.convexHull(contour),
            'hull_indices': hull,
            'defects': defects,
            'fingers': fingers,
            'finger_points': finger_points,
            'center': (cx, cy),
            'radius': radius,
            'convexity': convexity,
            'contour': contour,
        }

    def _count_fingers(self, contour, defects):
        """
        通过凸缺陷计算手指数

        算法：
        1. 遍历凸缺陷
        2. 计算缺陷深度
        3. 计算缺陷角度
        4. 判断是否为手指间隙

        参数：
            contour: 轮廓
            defects: 凸缺陷

        返回：
            finger_count: 手指数
            finger_points: 手指尖端点列表
        """
        finger_count = 0
        finger_points = []

        if defects is None:
            return 0, []

        for i in range(defects.shape[0]):
            s, e, f, d = defects[i, 0]

            start = tuple(contour[s][0])
            end = tuple(contour[e][0])
            far = tuple(contour[f][0])

            # 计算三角形边长
            a = np.sqrt((end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2)
            b = np.sqrt((far[0] - start[0]) ** 2 + (far[1] - start[1]) ** 2)
            c = np.sqrt((end[0] - far[0]) ** 2 + (end[1] - far[1]) ** 2)

            # 计算角度（使用余弦定理）
            if b * c == 0:
                continue

            angle = np.arccos((b ** 2 + c ** 2 - a ** 2) / (2 * b * c)) * 180 / np.pi

            # 深度过滤（过滤太浅的缺陷）
            depth = d / 256.0
            if depth < self.convexity_defect_depth_min:
                continue

            # 角度过滤（手指间隙角度通常 < 90度）
            if angle < self.finger_angle_thresh:
                finger_count += 1
                finger_points.append(start)
                finger_points.append(end)

        # 去重手指点（取唯一点）
        if finger_points:
            # 手指数量 = (唯一手指点数 + 1) / 2
            unique_points = []
            for p in finger_points:
                is_unique = True
                for up in unique_points:
                    if np.sqrt((p[0] - up[0]) ** 2 + (p[1] - up[1]) ** 2) < 30:
                        is_unique = False
                        break
                if is_unique:
                    unique_points.append(p)
            finger_points = unique_points

        return min(finger_count + 1, 5), finger_points  # 最多5个手指

    def classify_gesture(self, hand_info):
        """
        根据手部特征分类手势

        参数：
            hand_info: 手部特征字典

        返回：
            gesture: 手势名称
            confidence: 置信度
        """
        if hand_info is None:
            return 'none', 0.0

        fingers = hand_info['fingers']
        convexity = hand_info['convexity']
        radius = hand_info['radius']

        # 基于手指数分类
        if fingers == 0:
            return 'fist', 0.8

        elif fingers == 1:
            # 判断是食指还是其他单指
            if hand_info['finger_points']:
                fp = hand_info['finger_points'][0]
                center = hand_info['center']
                # 食指通常在上方
                if fp[1] < center[1] - radius * 0.3:
                    return 'one', 0.85
            return 'one', 0.7

        elif fingers == 2:
            return 'two', 0.8

        elif fingers == 3:
            return 'three', 0.8

        elif fingers == 4:
            return 'four', 0.75

        elif fingers >= 5:
            # 判断是五指张开还是其他
            if convexity > 0.85:
                return 'palm', 0.9
            return 'five', 0.7

        return 'none', 0.0

    def detect_wave(self, hand_info):
        """
        检测挥手动作

        通过分析手部中心的连续移动判断挥手

        参数：
            hand_info: 手部特征

        返回：
            is_wave: 是否在挥手
        """
        if hand_info is None:
            return False

        # 记录手部位置
        self.hand_positions.append(hand_info['center'])

        if len(self.hand_positions) < 10:
            return False

        # 分析最近10帧的水平移动
        recent = list(self.hand_positions)[-10:]
        x_positions = [p[0] for p in recent]

        # 计算移动方向变化次数
        direction_changes = 0
        for i in range(2, len(x_positions)):
            d1 = x_positions[i - 1] - x_positions[i - 2]
            d2 = x_positions[i] - x_positions[i - 1]
            if d1 * d2 < 0 and abs(d2) > 5:  # 方向改变且幅度足够
                direction_changes += 1

        # 方向变化 >= 3 次认为是挥手
        return direction_changes >= 3

    def smooth_gesture(self, gesture):
        """
        平滑手势识别结果（投票机制）

        参数：
            gesture: 当前手势

        返回：
            smoothed: 平滑后的手势
        """
        self.gesture_history.append(gesture)

        if len(self.gesture_history) < 3:
            return gesture

        # 投票
        from collections import Counter
        counter = Counter(self.gesture_history)
        most_common = counter.most_common(1)[0][0]

        return most_common

    def recognize(self, frame):
        """
        完整的手势识别流水线

        参数：
            frame: 输入BGR图像

        返回：
            gesture: 识别到的手势名称
            info: 手部特征信息字典
        """
        # 提取ROI
        x, y, w, h = self.roi
        roi = frame[y:y + h, x:x + w]

        if roi.size == 0:
            return 'none', None

        # 1. 肤色检测
        skin_mask = self.detect_skin(roi)

        # 2. 找到手部轮廓
        contour, area = self.find_hand_contour(skin_mask)

        # 3. 分析手部
        hand_info = self.analyze_hand(contour)

        # 4. 分类手势
        gesture, confidence = self.classify_gesture(hand_info)

        # 5. 检测挥手
        if self.detect_wave(hand_info):
            gesture = 'wave'
            confidence = 0.85

        # 6. 平滑处理
        gesture = self.smooth_gesture(gesture)

        # 将坐标转换回原图
        if hand_info is not None:
            hand_info['roi_offset'] = (x, y)
            # 调整中心点到原图坐标
            cx, cy = hand_info['center']
            hand_info['center_global'] = (cx + x, cy + y)
            # 调整手指点
            hand_info['finger_points_global'] = [
                (fp[0] + x, fp[1] + y) for fp in hand_info['finger_points']
            ]

        return gesture, {
            'hand_info': hand_info,
            'confidence': confidence,
            'skin_mask': skin_mask,
            'contour_area': area,
        }

    def draw_results(self, frame, gesture, info):
        """
        在图像上绘制识别结果

        参数：
            frame: 输入图像
            gesture: 手势名称
            info: 识别信息

        返回：
            annotated: 标注后的图像
        """
        annotated = frame.copy()

        # 绘制ROI区域
        x, y, w, h = self.roi
        cv2.rectangle(annotated, (x, y), (x + w, y + h), (255, 255, 0), 2)
        cv2.putText(annotated, 'ROI', (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        if info is None or info['hand_info'] is None:
            cv2.putText(annotated, 'No hand detected', (x, y + h + 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            return annotated

        hand = info['hand_info']

        # 绘制手部轮廓
        if 'contour' in hand:
            # 调整轮廓坐标到原图
            contour_global = hand['contour'].copy()
            contour_global[:, :, 0] += x
            contour_global[:, :, 1] += y
            cv2.drawContours(annotated, [contour_global], -1, (0, 255, 0), 2)

        # 绘制凸包
        if 'hull' in hand:
            hull_global = hand['hull'].copy()
            hull_global[:, :, 0] += x
            hull_global[:, :, 1] += y
            cv2.drawContours(annotated, [hull_global], -1, (0, 255, 255), 2)

        # 绘制手部中心
        if 'center_global' in hand:
            cv2.circle(annotated, hand['center_global'], 10, (0, 0, 255), -1)

        # 绘制手指尖端
        if 'finger_points_global' in hand:
            for fp in hand['finger_points_global']:
                cv2.circle(annotated, fp, 8, (255, 0, 0), -1)

        # 绘制手势名称
        gesture_cn = self.gesture_names.get(gesture, gesture)
        confidence = info.get('confidence', 0)

        cv2.putText(annotated, f'Gesture: {gesture_cn}',
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        cv2.putText(annotated, f'Confidence: {confidence:.0%}',
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(annotated, f'Fingers: {hand.get("fingers", 0)}',
                    (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        return annotated


class GestureControlInterface:
    """
    手势控制接口（将手势映射到控制命令）

    使用示例：
        controller = GestureControlInterface()
        gesture, info = recognizer.recognize(frame)
        command = controller.map_to_command(gesture)
        print(f"命令: {command}")
    """

    def __init__(self):
        """初始化手势控制映射"""
        # 手势到命令的映射
        self.command_map = {
            'fist': {'action': 'stop', 'desc': '停止'},
            'palm': {'action': 'go', 'desc': '前进'},
            'one': {'action': 'turn_left', 'desc': '左转'},
            'two': {'action': 'turn_right', 'desc': '右转'},
            'three': {'action': 'speed_up', 'desc': '加速'},
            'four': {'action': 'slow_down', 'desc': '减速'},
            'wave': {'action': 'emergency_stop', 'desc': '紧急停止'},
        }

        self.last_command = None
        self.command_count = 0
        self.required_frames = 5  # 需要连续N帧相同手势

    def map_to_command(self, gesture):
        """
        将手势映射到控制命令

        参数：
            gesture: 手势名称

        返回：
            command: 控制命令字典（或None）
        """
        if gesture == self.last_command:
            self.command_count += 1
        else:
            self.last_command = gesture
            self.command_count = 1

        # 需要连续多帧确认
        if self.command_count >= self.required_frames:
            command = self.command_map.get(gesture)
            if command:
                return command

        return None


# ================================================================
#                          使用示例
# ================================================================
if __name__ == '__main__':
    print("=" * 60)
    print("手势识别 - Gesture Recognition")
    print("针对 Orange Pi 5 优化")
    print("=" * 60)

    # 创建识别器
    recognizer = GestureRecognizer(roi=(150, 50, 350, 350))
    controller = GestureControlInterface()

    # 从摄像头读取
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    print("\n按 'q' 退出")
    print("按 'r' 重设ROI区域")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        t_start = time.time()
        gesture, info = recognizer.recognize(frame)
        annotated = recognizer.draw_results(frame, gesture, info)

        # 手势控制
        command = controller.map_to_command(gesture)
        if command:
            cv2.putText(annotated, f'CMD: {command["desc"]}',
                        (400, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

        fps = 1.0 / max(time.time() - t_start, 1e-6)
        cv2.putText(annotated, f'FPS: {fps:.1f}', (10, 450),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

        # 显示肤色掩码
        if info and info['skin_mask'] is not None:
            mask_small = cv2.resize(info['skin_mask'], (160, 120))
            annotated[0:120, 480:640] = cv2.cvtColor(mask_small, cv2.COLOR_GRAY2BGR)

        cv2.imshow('Gesture Recognition', annotated)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r'):
            # 简单的ROI重设（居中）
            h, w = frame.shape[:2]
            size = min(w, h) // 2
            recognizer.set_roi((w - size) // 2, (h - size) // 2, size, size)

    cap.release()
    cv2.destroyAllWindows()
