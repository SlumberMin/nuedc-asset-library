#!/usr/bin/env python3
"""
颜色追踪器
功能：多目标颜色追踪 + Kalman滤波预测 + 轨迹绘制
适用：OpenCV + Orange Pi 5
"""

import cv2
import numpy as np
from collections import deque


class KalmanTracker:
    """单目标Kalman滤波器"""

    def __init__(self, init_x, init_y, dt=1.0):
        # 状态: [x, y, vx, vy]
        self.kf = cv2.KalmanFilter(4, 2)
        self.kf.measurementMatrix = np.array([[1, 0, 0, 0],
                                               [0, 1, 0, 0]], np.float32)
        self.kf.transitionMatrix = np.array([[1, 0, dt, 0],
                                              [0, 1, 0, dt],
                                              [0, 0, 1, 0],
                                              [0, 0, 0, 1]], np.float32)
        self.kf.processNoiseCov = np.eye(4, dtype=np.float32) * 0.03
        self.kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * 0.5
        self.kf.statePost = np.array([[init_x], [init_y], [0], [0]], np.float32)

        self.trajectory = deque(maxlen=64)  # 轨迹历史
        self.trajectory.append((int(init_x), int(init_y)))
        self.lost_count = 0  # 丢失帧计数
        self.max_lost = 15   # 最大允许丢失帧数

    def predict(self):
        """预测下一帧位置"""
        pred = self.kf.predict()
        return int(pred[0]), int(pred[1])

    def update(self, x, y):
        """用观测值更新"""
        self.kf.correct(np.array([[np.float32(x)], [np.float32(y)]]))
        self.trajectory.append((int(x), int(y)))
        self.lost_count = 0

    def get_position(self):
        """获取当前估计位置"""
        state = self.kf.statePost
        return int(state[0]), int(state[1])

    def get_velocity(self):
        """获取当前速度估计"""
        state = self.kf.statePost
        return float(state[2]), float(state[3])

    def is_lost(self):
        """判断目标是否已丢失"""
        return self.lost_count > self.max_lost


class ColorTracker:
    """多目标颜色追踪器"""

    # 预定义HSV阈值
    PRESETS = {
        'red': {'lower1': [0, 100, 100], 'upper1': [10, 255, 255],
                'lower2': [160, 100, 100], 'upper2': [180, 255, 255]},
        'blue': {'lower1': [100, 100, 100], 'upper1': [130, 255, 255]},
        'green': {'lower1': [35, 80, 80], 'upper1': [85, 255, 255]},
        'yellow': {'lower1': [20, 100, 100], 'upper1': [35, 255, 255]},
        'black': {'lower1': [0, 0, 0], 'upper1': [180, 255, 50]},
    }

    def __init__(self, color_name='red', min_area=300, max_targets=5,
                 use_kalman=True, trail_length=64):
        """
        参数:
            color_name: 追踪颜色名(使用预设)或自定义HSV阈值字典
            min_area: 最小轮廓面积
            max_targets: 最大同时追踪目标数
            use_kalman: 是否使用Kalman滤波
            trail_length: 轨迹历史长度
        """
        self.min_area = min_area
        self.max_targets = max_targets
        self.use_kalman = use_kalman
        self.trail_length = trail_length
        self.trackers = []  # KalmanTracker列表
        self.next_id = 0
        self.match_distance = 80  # 匹配距离阈值(像素)

        # 设置颜色阈值
        if isinstance(color_name, str) and color_name in self.PRESETS:
            self.color_config = self.PRESETS[color_name]
        elif isinstance(color_name, dict):
            self.color_config = color_name
        else:
            self.color_config = self.PRESETS['red']

    def _get_mask(self, hsv_frame):
        """生成颜色掩膜"""
        cfg = self.color_config
        lower1 = np.array(cfg['lower1'])
        upper1 = np.array(cfg['upper1'])
        mask = cv2.inRange(hsv_frame, lower1, upper1)

        # 处理双区间颜色(如红色)
        if 'lower2' in cfg:
            lower2 = np.array(cfg['lower2'])
            upper2 = np.array(cfg['upper2'])
            mask2 = cv2.inRange(hsv_frame, lower2, upper2)
            mask = cv2.bitwise_or(mask, mask2)

        # 形态学处理
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def _find_detections(self, mask):
        """从掩膜中提取检测结果"""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detections = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area >= self.min_area:
                M = cv2.moments(cnt)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    x, y, w, h = cv2.boundingRect(cnt)
                    detections.append({
                        'cx': cx, 'cy': cy,
                        'x': x, 'y': y, 'w': w, 'h': h,
                        'area': area, 'contour': cnt
                    })
        # 按面积降序排序，取前max_targets个
        detections.sort(key=lambda d: d['area'], reverse=True)
        return detections[:self.max_targets]

    def _match_detections(self, detections):
        """匈牙利匹配(简化版)：最近邻匹配"""
        matched_det = set()
        matched_trk = set()

        if not self.trackers or not detections:
            return matched_det, matched_trk

        # 构建距离矩阵
        n_trk = len(self.trackers)
        n_det = len(detections)
        cost = np.full((n_trk, n_det), fill_value=1e6)

        for i, trk in enumerate(self.trackers):
            pred_x, pred_y = trk.get_position()
            for j, det in enumerate(detections):
                dist = np.sqrt((pred_x - det['cx'])**2 + (pred_y - det['cy'])**2)
                if dist < self.match_distance:
                    cost[i, j] = dist

        # 贪心匹配
        for _ in range(min(n_trk, n_det)):
            idx = np.unravel_index(np.argmin(cost), cost.shape)
            if cost[idx] >= 1e6:
                break
            i, j = idx
            matched_trk.add(i)
            matched_det.add(j)
            cost[i, :] = 1e6
            cost[:, j] = 1e6

        return matched_det, matched_trk

    def update(self, frame):
        """
        处理一帧图像，返回追踪结果

        返回:
            results: list of dict, 每个包含:
                - id: 目标ID
                - cx, cy: 当前/预测位置
                - vx, vy: 速度估计
                - bbox: (x, y, w, h)
                - area: 面积
                - trajectory: 轨迹点列表
                - lost: 是否丢失
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = self._get_mask(hsv)
        detections = self._find_detections(mask)

        # 匹配
        matched_det, matched_trk = self._match_detections(detections)

        # 构建正确的追踪器-检测映射
        trk_to_det = {}
        # 使用之前构建的cost矩阵进行精确匹配
        n_trk = len(self.trackers)
        n_det = len(detections)
        if n_trk > 0 and n_det > 0:
            cost_map = np.full((n_trk, n_det), fill_value=1e6)
            for i, trk in enumerate(self.trackers):
                pred_x, pred_y = trk.get_position()
                for j, det in enumerate(detections):
                    dist = np.sqrt((pred_x - det['cx'])**2 + (pred_y - det['cy'])**2)
                    if dist < self.match_distance:
                        cost_map[i, j] = dist

            # 贪心匹配，构建映射
            for _ in range(min(n_trk, n_det)):
                idx = np.unravel_index(np.argmin(cost_map), cost_map.shape)
                if cost_map[idx] >= 1e6:
                    break
                i, j = idx
                trk_to_det[i] = j
                cost_map[i, :] = 1e6
                cost_map[:, j] = 1e6

        # 更新已匹配的追踪器
        for i, trk in enumerate(self.trackers):
            if i in trk_to_det:
                det_idx = trk_to_det[i]
                det = detections[det_idx]
                trk.update(det['cx'], det['cy'])
            else:
                trk.predict()
                trk.lost_count += 1

        # 删除丢失的追踪器
        self.trackers = [t for t in self.trackers if not t.is_lost()]

        # 为未匹配的检测创建新追踪器
        matched_det_indices = set(trk_to_det.values())

        for j, det in enumerate(detections):
            if j not in matched_det_indices and len(self.trackers) < self.max_targets:
                new_trk = KalmanTracker(det['cx'], det['cy'])
                new_trk._id = self.next_id
                self.next_id += 1
                self.trackers.append(new_trk)

        # 构建结果
        results = []
        for trk in self.trackers:
            cx, cy = trk.get_position()
            vx, vy = trk.get_velocity()
            results.append({
                'id': getattr(trk, '_id', -1),
                'cx': cx, 'cy': cy,
                'vx': vx, 'vy': vy,
                'trajectory': list(trk.trajectory),
                'lost': trk.is_lost(),
            })

        return results, mask

    def draw(self, frame, results):
        """在帧上绘制追踪结果"""
        vis = frame.copy()
        colors = [(0, 255, 0), (0, 0, 255), (255, 0, 0), (0, 255, 255),
                  (255, 0, 255), (255, 255, 0), (128, 255, 0), (0, 128, 255)]

        for res in results:
            color = colors[res['id'] % len(colors)]

            # 绘制轨迹
            pts = res['trajectory']
            if len(pts) > 1:
                for i in range(1, len(pts)):
                    thickness = max(1, int(2 * i / len(pts)) + 1)
                    cv2.line(vis, pts[i-1], pts[i], color, thickness)

            # 绘制当前位置
            cv2.circle(vis, (res['cx'], res['cy']), 8, color, 2)
            cv2.drawMarker(vis, (res['cx'], res['cy']), color,
                          cv2.MARKER_CROSS, 20, 2)

            # 显示信息
            info = f"ID:{res['id']} ({res['cx']},{res['cy']})"
            cv2.putText(vis, info, (res['cx'] + 10, res['cy'] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

            # 速度箭头
            if abs(res['vx']) > 0.5 or abs(res['vy']) > 0.5:
                end_x = int(res['cx'] + res['vx'] * 3)
                end_y = int(res['cy'] + res['vy'] * 3)
                cv2.arrowedLine(vis, (res['cx'], res['cy']),
                               (end_x, end_y), color, 2, tipLength=0.3)

        return vis


def run_demo(camera_id=0, color='red'):
    """实时演示"""
    cap = cv2.VideoCapture(camera_id)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    tracker = ColorTracker(color_name=color, use_kalman=True)

    print("=" * 50)
    print(f"颜色追踪器 - 追踪颜色: {color}")
    print("q/ESC: 退出 | c: 清除追踪器")
    print("=" * 50)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        results, mask = tracker.update(frame)
        vis = tracker.draw(frame, results)

        cv2.putText(vis, f"Targets: {len(results)}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.imshow('Color Tracker', vis)
        cv2.imshow('Mask', mask)

        key = cv2.waitKey(1) & 0xFF
        if key in [ord('q'), 27]:
            break
        elif key == ord('c'):
            tracker.trackers.clear()
            tracker.next_id = 0

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='颜色追踪器')
    parser.add_argument('--camera', type=int, default=0)
    parser.add_argument('--color', type=str, default='red',
                       choices=['red', 'blue', 'green', 'yellow', 'black'])
    args = parser.parse_args()
    run_demo(args.camera, args.color)
