#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
高级运动检测模块 - Advanced Motion Detection
===============================================
针对 Orange Pi 5 优化的运动检测算法
包含：背景建模、前景提取、目标跟踪、轨迹分析

技术栈：OpenCV + NumPy + 多线程优化
适配：Orange Pi 5 (RK3588S) / Linux ARM64

作者：nuedc-asset-library
"""

import cv2
import numpy as np
import threading
import time
from collections import deque, defaultdict


class BackgroundModeler:
    """
    背景建模器

    支持方法：
    1. 帧差法（Frame Difference）
    2. 均值背景法（Average Background）
    3. 高斯混合模型（GMM / MOG2）
    4. KNN背景减除

    使用示例：
        modeler = BackgroundModeler(method='mog2')
        fg_mask = modeler.update(frame)
    """

    def __init__(self, method='mog2', history=500, threshold=16):
        """
        初始化背景建模器

        参数：
            method: 背景建模方法 ('frame_diff', 'average', 'mog2', 'knn')
            history: 背景历史帧数
            threshold: 阈值
        """
        self.method = method
        self.history = history
        self.threshold = threshold

        # 背景减除器
        self.bg_subtractor = None

        if method == 'mog2':
            self.bg_subtractor = cv2.createBackgroundSubtractorMOG2(
                history=history,
                varThreshold=threshold,
                detectShadows=True
            )
            # 设置阴影参数
            self.bg_subtractor.setShadowValue(127)
            self.bg_subtractor.setShadowThreshold(0.5)

        elif method == 'knn':
            self.bg_subtractor = cv2.createBackgroundSubtractorKNN(
                history=history,
                dist2Threshold=400,
                detectShadows=True
            )

        # 平均背景（用于均值法）
        self.avg_background = None
        self.frame_count = 0
        self.alpha = 0.01  # 学习率

        # 前一帧（用于帧差法）
        self.prev_frame = None

        print(f"[背景建模] 方法: {method}")

    def update(self, frame):
        """
        更新背景模型并获取前景掩码

        参数：
            frame: 输入BGR图像

        返回：
            fg_mask: 前景掩码（255=前景，0=背景）
        """
        if self.method in ('mog2', 'knn'):
            fg_mask = self.bg_subtractor.apply(frame)
            # 去除阴影（值为127的像素）
            fg_mask = cv2.threshold(fg_mask, 200, 255, cv2.THRESH_BINARY)[1]

        elif self.method == 'frame_diff':
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (21, 21), 0)

            if self.prev_frame is None:
                self.prev_frame = gray
                return np.zeros_like(gray)

            # 帧差
            diff = cv2.absdiff(self.prev_frame, gray)
            fg_mask = cv2.threshold(diff, 30, 255, cv2.THRESH_BINARY)[1]
            self.prev_frame = gray

        elif self.method == 'average':
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).astype(np.float32)

            if self.avg_background is None:
                self.avg_background = gray.copy()
                return np.zeros_like(gray, dtype=np.uint8)

            # 更新背景
            cv2.accumulateWeighted(gray, self.avg_background, self.alpha)

            # 计算差值
            diff = cv2.absdiff(gray, self.avg_background.astype(np.uint8))
            fg_mask = cv2.threshold(diff, 30, 255, cv2.THRESH_BINARY)[1]

        else:
            raise ValueError(f"不支持的方法: {self.method}")

        # 形态学操作去噪
        fg_mask = self._morphological_cleanup(fg_mask)

        return fg_mask

    def _morphological_cleanup(self, mask):
        """
        形态学操作清理掩码

        参数：
            mask: 输入掩码

        返回：
            cleaned: 清理后的掩码
        """
        # 开运算：去除小噪点
        kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        cleaned = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open, iterations=2)

        # 闭运算：填充空洞
        kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, kernel_close, iterations=2)

        # 膨胀：扩展前景区域
        kernel_dilate = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        cleaned = cv2.dilate(cleaned, kernel_dilate, iterations=1)

        return cleaned

    def reset(self):
        """重置背景模型"""
        if self.bg_subtractor is not None:
            self.bg_subtractor.clear()
        self.avg_background = None
        self.prev_frame = None
        self.frame_count = 0


class MotionDetector:
    """
    运动检测器

    功能：
    - 运动区域检测
    - 运动目标提取
    - 轮廓分析
    - 面积/速度过滤

    使用示例：
        detector = MotionDetector()
        motions = detector.detect(frame)
        for m in motions:
            print(m['position'], m['area'], m['speed'])
    """

    def __init__(self, bg_method='mog2', min_area=500, max_area=50000,
                 min_speed=0, max_speed=1000):
        """
        初始化运动检测器

        参数：
            bg_method: 背景建模方法
            min_area: 最小运动区域面积
            max_area: 最大运动区域面积
            min_speed: 最小运动速度（像素/帧）
            max_speed: 最大运动速度
        """
        self.bg_modeler = BackgroundModeler(method=bg_method)
        self.min_area = min_area
        self.max_area = max_area
        self.min_speed = min_speed
        self.max_speed = max_speed

        # 运动历史
        self.motion_history = deque(maxlen=30)
        self.prev_positions = {}  # {id: position}
        self.next_id = 0

        # 连通域分析
        self.connectivity = 8

        print(f"[运动检测] 初始化完成")

    def detect(self, frame):
        """
        检测运动目标

        参数：
            frame: 输入BGR图像

        返回：
            motions: 运动目标列表 [{
                'id': 目标ID,
                'position': (x, y, w, h),
                'center': (cx, cy),
                'area': 面积,
                'speed': 速度,
                'direction': 运动方向(度),
                'contour': 轮廓,
            }]
        """
        # 1. 获取前景掩码
        fg_mask = self.bg_modeler.update(frame)

        # 2. 查找轮廓
        contours, _ = cv2.findContours(fg_mask, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)

        motions = []

        for contour in contours:
            area = cv2.contourArea(contour)

            # 面积过滤
            if area < self.min_area or area > self.max_area:
                continue

            # 边界矩形
            x, y, w, h = cv2.boundingRect(contour)

            # 纵横比过滤（去除噪声）
            aspect = float(w) / h if h > 0 else 0
            if aspect > 5 or aspect < 0.2:
                continue

            # 中心点
            M = cv2.moments(contour)
            if M['m00'] == 0:
                continue
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            # 计算紧凑度
            hull = cv2.convexHull(contour)
            hull_area = cv2.contourArea(hull)
            solidity = area / hull_area if hull_area > 0 else 0

            motions.append({
                'position': (x, y, w, h),
                'center': (cx, cy),
                'area': area,
                'contour': contour,
                'solidity': solidity,
            })

        # 3. 目标关联和速度计算
        motions = self._associate_targets(motions)

        return motions, fg_mask

    def _associate_targets(self, current_motions):
        """
        目标关联（基于最近邻匹配）

        参数：
            current_motions: 当前帧运动目标

        返回：
            associated: 关联后的目标列表
        """
        if not self.prev_positions:
            # 第一帧，分配新ID
            for m in current_motions:
                m['id'] = self.next_id
                m['speed'] = 0
                m['direction'] = 0
                self.prev_positions[self.next_id] = m['center']
                self.next_id += 1
            return current_motions

        # 计算距离矩阵
        prev_ids = list(self.prev_positions.keys())
        prev_centers = [self.prev_positions[pid] for pid in prev_ids]

        for m in current_motions:
            best_id = None
            best_dist = float('inf')

            for i, (pid, pcenter) in enumerate(zip(prev_ids, prev_centers)):
                dist = np.sqrt((m['center'][0] - pcenter[0]) ** 2 +
                               (m['center'][1] - pcenter[1]) ** 2)

                if dist < best_dist and dist < 100:  # 最大关联距离
                    best_dist = dist
                    best_id = pid

            if best_id is not None:
                # 关联成功
                m['id'] = best_id
                m['speed'] = best_dist
                dx = m['center'][0] - self.prev_positions[best_id][0]
                dy = m['center'][1] - self.prev_positions[best_id][1]
                m['direction'] = np.degrees(np.arctan2(dy, dx))
                self.prev_positions[best_id] = m['center']
            else:
                # 新目标
                m['id'] = self.next_id
                m['speed'] = 0
                m['direction'] = 0
                self.prev_positions[self.next_id] = m['center']
                self.next_id += 1

        # 清理消失的目标
        current_ids = {m['id'] for m in current_motions}
        lost_ids = [pid for pid in self.prev_positions if pid not in current_ids]
        for pid in lost_ids:
            del self.prev_positions[pid]

        return current_motions


class OpticalFlowTracker:
    """
    光流跟踪器

    使用稀疏光流（Lucas-Kanade）跟踪特征点

    使用示例：
        tracker = OpticalFlowTracker()
        tracked = tracker.track(frame)
    """

    def __init__(self, max_points=100, quality_level=0.3,
                 min_distance=7, block_size=7):
        """
        初始化光流跟踪器

        参数：
            max_points: 最大特征点数
            quality_level: 特征点质量阈值
            min_distance: 最小特征点距离
            block_size: 块大小
        """
        self.max_points = max_points
        self.quality_level = quality_level
        self.min_distance = min_distance
        self.block_size = block_size

        # 特征点参数
        self.feature_params = dict(
            maxCorners=max_points,
            qualityLevel=quality_level,
            minDistance=min_distance,
            blockSize=block_size
        )

        # Lucas-Kanade光流参数
        self.lk_params = dict(
            winSize=(15, 15),
            maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03)
        )

        self.prev_gray = None
        self.prev_points = None
        self.tracks = []  # 跟踪轨迹
        self.track_len = 10  # 轨迹长度

        print("[光流跟踪器] 初始化完成")

    def track(self, frame):
        """
        跟踪特征点

        参数：
            frame: 输入BGR图像

        返回：
            tracks: 跟踪轨迹列表
            flow: 光流场
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.prev_gray is None:
            self.prev_gray = gray
            return [], None

        # 计算光流
        if self.prev_points is not None and len(self.prev_points) > 0:
            next_points, status, error = cv2.calcOpticalFlowPyrLK(
                self.prev_gray, gray, self.prev_points, None, **self.lk_params
            )

            # 过滤好的跟踪点
            good_mask = status.flatten() == 1
            good_new = next_points[good_mask]
            good_old = self.prev_points[good_mask]

            # 更新轨迹
            new_tracks = []
            for i, (new, old) in enumerate(zip(good_new, good_old)):
                if i < len(self.tracks):
                    track = self.tracks[i] + [new.ravel().tolist()]
                    if len(track) > self.track_len:
                        track = track[-self.track_len:]
                    new_tracks.append(track)
                else:
                    new_tracks.append([new.ravel().tolist()])

            self.tracks = new_tracks
            self.prev_points = good_new.reshape(-1, 1, 2)
        else:
            good_new = np.array([])

        # 检测新的特征点
        if self.prev_points is None or len(self.prev_points) < self.max_points // 2:
            new_pts = cv2.goodFeaturesToTrack(gray, mask=None, **self.feature_params)
            if new_pts is not None:
                if self.prev_points is not None and len(self.prev_points) > 0:
                    self.prev_points = np.vstack([self.prev_points, new_pts])
                else:
                    self.prev_points = new_pts
                for pt in new_pts:
                    self.tracks.append([pt.ravel().tolist()])

        self.prev_gray = gray

        # 计算密集光流（可选）
        flow = cv2.calcOpticalFlowFarneback(
            self.prev_gray, gray, None,
            pyr_scale=0.5, levels=3, winsize=15,
            iterations=3, poly_n=5, poly_sigma=1.2, flags=0
        ) if len(self.tracks) > 0 else None

        return self.tracks, flow

    def reset(self):
        """重置跟踪器"""
        self.prev_gray = None
        self.prev_points = None
        self.tracks = []


class MotionHeatmap:
    """
    运动热力图生成器

    累积运动信息，生成热力图可视化

    使用示例：
        heatmap = MotionHeatmap()
        heatmap.update(fg_mask)
        vis = heatmap.get_heatmap()
    """

    def __init__(self, decay_factor=0.95):
        """
        初始化热力图

        参数：
            decay_factor: 衰减因子（越接近1衰减越慢）
        """
        self.decay_factor = decay_factor
        self.heatmap = None

    def update(self, fg_mask):
        """
        更新热力图

        参数：
            fg_mask: 前景掩码
        """
        mask_float = fg_mask.astype(np.float32) / 255.0

        if self.heatmap is None:
            self.heatmap = mask_float
        else:
            self.heatmap = self.heatmap * self.decay_factor + mask_float

        # 归一化
        max_val = np.max(self.heatmap)
        if max_val > 0:
            self.heatmap = self.heatmap / max_val

    def get_heatmap(self):
        """
        获取热力图可视化

        返回：
            heatmap_vis: BGR热力图图像
        """
        if self.heatmap is None:
            return None

        # 转换为uint8
        heatmap_uint8 = (self.heatmap * 255).astype(np.uint8)

        # 应用色彩映射
        heatmap_vis = cv2.applyColorMap(heatmap_uint8, cv2.COLORMAP_JET)

        return heatmap_vis

    def reset(self):
        """重置热力图"""
        self.heatmap = None


class MotionDetectionPipeline:
    """
    运动检测流水线（整合所有组件）

    使用示例：
        pipeline = MotionDetectionPipeline()
        pipeline.start(camera_id=0)

        while True:
            result = pipeline.get_result()
            if result:
                cv2.imshow('Motion', result['visualization'])
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        pipeline.stop()
    """

    def __init__(self, camera_id=0, resolution=(640, 480), bg_method='mog2'):
        """
        初始化流水线

        参数：
            camera_id: 摄像头ID
            resolution: 分辨率
            bg_method: 背景建模方法
        """
        self.camera_id = camera_id
        self.resolution = resolution

        # 组件
        self.motion_detector = MotionDetector(bg_method=bg_method)
        self.optical_flow = OpticalFlowTracker()
        self.heatmap = MotionHeatmap()

        # 线程
        self._cap = None
        self._frame = None
        self._result = None
        self._running = False
        self._lock = threading.Lock()
        self._threads = []

        # 统计
        self.frame_count = 0
        self.total_motions = 0

    def start(self):
        """启动流水线"""
        self._cap = cv2.VideoCapture(self.camera_id)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])

        self._running = True

        t1 = threading.Thread(target=self._capture_loop, daemon=True)
        t1.start()
        self._threads.append(t1)

        t2 = threading.Thread(target=self._process_loop, daemon=True)
        t2.start()
        self._threads.append(t2)

        print("[运动检测流水线] 已启动")

    def stop(self):
        """停止流水线"""
        self._running = False
        for t in self._threads:
            t.join(timeout=2)
        if self._cap:
            self._cap.release()
        print("[运动检测流水线] 已停止")

    def _capture_loop(self):
        """采集循环"""
        while self._running:
            ret, frame = self._cap.read()
            if ret:
                with self._lock:
                    self._frame = frame

    def _process_loop(self):
        """处理循环"""
        while self._running:
            frame = None
            with self._lock:
                if self._frame is not None:
                    frame = self._frame.copy()

            if frame is not None:
                self._process_frame(frame)

    def _process_frame(self, frame):
        """处理单帧"""
        # 运动检测
        motions, fg_mask = self.motion_detector.detect(frame)

        # 光流跟踪
        tracks, flow = self.optical_flow.track(frame)

        # 更新热力图
        self.heatmap.update(fg_mask)

        # 生成可视化
        vis = self._draw_results(frame, motions, fg_mask, tracks)

        self.frame_count += 1
        self.total_motions += len(motions)

        with self._lock:
            self._result = {
                'motions': motions,
                'fg_mask': fg_mask,
                'tracks': tracks,
                'flow': flow,
                'visualization': vis,
                'frame_count': self.frame_count,
            }

    def _draw_results(self, frame, motions, fg_mask, tracks):
        """绘制检测结果"""
        vis = frame.copy()

        # 绘制运动区域
        for m in motions:
            x, y, w, h = m['position']
            color = (0, 255, 0)
            cv2.rectangle(vis, (x, y), (x + w, y + h), color, 2)

            # 标注信息
            label = f"ID:{m.get('id', '?')} Area:{m['area']}"
            cv2.putText(vis, label, (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            # 运动方向箭头
            if 'direction' in m and m.get('speed', 0) > 2:
                cx, cy = m['center']
                angle = np.radians(m['direction'])
                arrow_len = min(50, m.get('speed', 0) * 2)
                ex = int(cx + arrow_len * np.cos(angle))
                ey = int(cy + arrow_len * np.sin(angle))
                cv2.arrowedLine(vis, (cx, cy), (ex, ey), (0, 0, 255), 2)

        # 绘制光流轨迹
        for track in tracks:
            if len(track) > 1:
                pts = np.int32(track)
                cv2.polylines(vis, [pts], False, (255, 255, 0), 2)
                # 绘制当前点
                cv2.circle(vis, tuple(pts[-1]), 5, (0, 255, 255), -1)

        # 叠加热力图
        heatmap_vis = self.heatmap.get_heatmap()
        if heatmap_vis is not None:
            h, w = vis.shape[:2]
            heatmap_small = cv2.resize(heatmap_vis, (w // 4, h // 4))
            vis[0:h // 4, w - w // 4:w] = cv2.addWeighted(
                vis[0:h // 4, w - w // 4:w], 0.5, heatmap_small, 0.5, 0
            )

        # 前景掩码小图
        fg_small = cv2.resize(fg_mask, (160, 120))
        vis[0:120, 0:160] = cv2.cvtColor(fg_small, cv2.COLOR_GRAY2BGR)

        # 统计信息
        cv2.putText(vis, f'Frame: {self.frame_count}', (10, 140),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(vis, f'Motions: {len(motions)}', (10, 165),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        return vis

    def get_result(self):
        """获取最新结果"""
        with self._lock:
            return self._result


# ================================================================
#                          使用示例
# ================================================================
if __name__ == '__main__':
    print("=" * 60)
    print("高级运动检测 - Advanced Motion Detection")
    print("针对 Orange Pi 5 优化")
    print("=" * 60)

    # 选择背景建模方法
    print("\n选择背景建模方法:")
    print("  1 - GMM/高斯混合模型 (默认)")
    print("  2 - KNN")
    print("  3 - 帧差法")
    print("  4 - 均值背景法")

    method_choice = input("请选择 (1-4, 默认1): ").strip() or '1'
    methods = {'1': 'mog2', '2': 'knn', '3': 'frame_diff', '4': 'average'}
    method = methods.get(method_choice, 'mog2')

    # 创建检测器
    detector = MotionDetector(bg_method=method)
    flow_tracker = OpticalFlowTracker()
    heatmap = MotionHeatmap()

    # 从摄像头读取
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    print(f"\n背景建模方法: {method}")
    print("按 'q' 退出")
    print("按 'r' 重置背景模型")
    print("按 'h' 显示/隐藏热力图")

    show_heatmap = True
    frame_count = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        t_start = time.time()

        # 运动检测
        motions, fg_mask = detector.detect(frame)

        # 光流跟踪
        tracks, flow = flow_tracker.track(frame)

        # 热力图
        heatmap.update(fg_mask)

        # 绘制结果
        vis = frame.copy()

        for m in motions:
            x, y, w, h = m['position']
            cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 255, 0), 2)
            label = f"ID:{m.get('id', '?')} S:{m.get('speed', 0):.0f}"
            cv2.putText(vis, label, (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        for track in tracks:
            if len(track) > 1:
                pts = np.int32(track)
                cv2.polylines(vis, [pts], False, (255, 255, 0), 2)

        # 叠加热力图
        if show_heatmap:
            hv = heatmap.get_heatmap()
            if hv is not None:
                hv_small = cv2.resize(hv, (vis.shape[1] // 4, vis.shape[0] // 4))
                vis[0:hv_small.shape[0], vis.shape[1] - hv_small.shape[1]:] = cv2.addWeighted(
                    vis[0:hv_small.shape[0], vis.shape[1] - hv_small.shape[1]:],
                    0.5, hv_small, 0.5, 0
                )

        # 前景掩码
        fg_vis = cv2.resize(fg_mask, (160, 120))
        vis[0:120, 0:160] = cv2.cvtColor(fg_vis, cv2.COLOR_GRAY2BGR)

        fps = 1.0 / max(time.time() - t_start, 1e-6)
        cv2.putText(vis, f'FPS: {fps:.1f}  Objects: {len(motions)}',
                    (10, vis.shape[0] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        cv2.imshow('Motion Detection', vis)

        frame_count += 1

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r'):
            detector.bg_modeler.reset()
            flow_tracker.reset()
            heatmap.reset()
            print("已重置")
        elif key == ord('h'):
            show_heatmap = not show_heatmap

    cap.release()
    cv2.destroyAllWindows()
