#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
高级车道线检测模块 - Advanced Lane Detection
==============================================
针对 Orange Pi 5 优化的车道线检测算法
包含：透视变换、多项式拟合、滑动窗口搜索

技术栈：OpenCV + NumPy + 多线程优化
适配：Orange Pi 5 (RK3588S) / Linux ARM64

作者：nuedc-asset-library
"""

import cv2
import numpy as np
import threading
import time
from collections import deque


class AdvancedLaneDetector:
    """
    高级车道线检测器

    功能特性：
    1. 透视变换（鸟瞰图）
    2. Sobel边缘检测 + 颜色阈值
    3. 滑动窗口搜索车道线像素
    4. 二阶多项式拟合车道线
    5. 车道曲率和车辆偏移计算
    6. 多线程加速处理

    使用示例：
        detector = AdvancedLaneDetector()
        result = detector.detect(frame)
        cv2.imshow('Lane', result['visualization'])
    """

    def __init__(self, config=None):
        """
        初始化车道线检测器

        参数：
            config: 配置字典，可覆盖默认参数
        """
        # ==================== 摄像头参数 ====================
        # 默认透视变换源点（根据实际摄像头调整）
        self.src_points = np.float32([
            [200, 720],   # 左下
            [580, 460],   # 左上
            [700, 460],   # 右上
            [1100, 720]   # 右下
        ])

        # 透视变换目标点（鸟瞰图坐标）
        self.dst_points = np.float32([
            [300, 720],   # 左下
            [300, 0],     # 左上
            [980, 0],     # 右上
            [980, 720]    # 右下
        ])

        # ==================== 滑动窗口参数 ====================
        self.n_windows = 9          # 窗口数量
        self.window_width = 100     # 窗口宽度（像素）
        self.min_pixels = 50        # 重新定位的最小像素数

        # ==================== 像素到米的转换 ====================
        self.ym_per_pix = 30 / 720   # y方向：30m对应720像素
        self.xm_per_pix = 3.7 / 700  # x方向：3.7m对应700像素

        # ==================== Sobel参数 ====================
        self.sobel_kernel = 7
        self.sobel_thresh_x = (20, 100)
        self.sobel_thresh_mag = (30, 150)
        self.sobel_thresh_dir = (0.7, 1.3)

        # ==================== 颜色阈值参数 ====================
        self.hls_s_thresh = (170, 255)   # S通道阈值
        self.lab_b_thresh = (155, 200)   # B通道阈值（Lab色彩空间）
        self.luv_l_thresh = (215, 255)   # L通道阈值（LUV色彩空间）

        # ==================== 检测结果历史 ====================
        self.left_fit_history = deque(maxlen=10)
        self.right_fit_history = deque(maxlen=10)
        self.curvature_history = deque(maxlen=20)

        # ==================== 多线程 ====================
        self._lock = threading.Lock()
        self._result_cache = None

        # 应用自定义配置
        if config:
            for key, value in config.items():
                if hasattr(self, key):
                    setattr(self, key, value)

        # 预计算透视变换矩阵
        self._update_transform_matrix()

        print("[车道线检测] 初始化完成")
        print(f"[车道线检测] 滑动窗口数: {self.n_windows}, 窗口宽度: {self.window_width}px")

    def _update_transform_matrix(self):
        """计算并缓存透视变换矩阵"""
        self.M = cv2.getPerspectiveTransform(self.src_points, self.dst_points)
        self.M_inv = cv2.getPerspectiveTransform(self.dst_points, self.src_points)

    def set_perspective_points(self, src_pts, dst_pts):
        """
        设置透视变换点

        参数：
            src_pts: 源点 [[x1,y1], [x2,y2], [x3,y3], [x4,y4]]
            dst_pts: 目标点 [[x1,y1], [x2,y2], [x3,y3], [x4,y4]]
        """
        self.src_points = np.float32(src_pts)
        self.dst_points = np.float32(dst_pts)
        self._update_transform_matrix()
        print("[车道线检测] 透视变换矩阵已更新")

    def perspective_transform(self, img):
        """
        透视变换（俯视图/鸟瞰图）

        参数：
            img: 输入图像

        返回：
            warped: 透视变换后的图像
        """
        h, w = img.shape[:2]
        warped = cv2.warpPerspective(img, self.M, (w, h), flags=cv2.INTER_LINEAR)
        return warped

    def sobel_edge_detection(self, gray):
        """
        Sobel边缘检测（多方向组合）

        包含：X方向梯度 + Y方向梯度 + 梯度幅值 + 梯度方向

        参数：
            gray: 灰度图像

        返回：
            combined: 组合边缘图
        """
        # X方向梯度
        sobel_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=self.sobel_kernel)
        abs_sobel_x = np.absolute(sobel_x)
        scaled_x = np.uint8(255 * abs_sobel_x / np.max(abs_sobel_x))
        binary_x = np.zeros_like(scaled_x)
        binary_x[(scaled_x >= self.sobel_thresh_x[0]) &
                  (scaled_x <= self.sobel_thresh_x[1])] = 1

        # Y方向梯度
        sobel_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=self.sobel_kernel)
        abs_sobel_y = np.absolute(sobel_y)
        scaled_y = np.uint8(255 * abs_sobel_y / np.max(abs_sobel_y))
        binary_y = np.zeros_like(scaled_y)
        binary_y[(scaled_y >= self.sobel_thresh_x[0]) &
                  (scaled_y <= self.sobel_thresh_x[1])] = 1

        # 梯度幅值
        mag = np.sqrt(sobel_x ** 2 + sobel_y ** 2)
        scaled_mag = np.uint8(255 * mag / np.max(mag))
        binary_mag = np.zeros_like(scaled_mag)
        binary_mag[(scaled_mag >= self.sobel_thresh_mag[0]) &
                    (scaled_mag <= self.sobel_thresh_mag[1])] = 1

        # 梯度方向
        direction = np.arctan2(abs_sobel_y, abs_sobel_x)
        binary_dir = np.zeros_like(direction)
        binary_dir[(direction >= self.sobel_thresh_dir[0]) &
                    (direction <= self.sobel_thresh_dir[1])] = 1

        # 组合：X方向 + 幅值 & 方向
        combined = np.zeros_like(binary_x)
        combined[((binary_x == 1) & (binary_y == 1)) |
                 ((binary_mag == 1) & (binary_dir == 1))] = 1

        return combined

    def color_threshold(self, img):
        """
        颜色阈值分割（多色彩空间）

        使用 HLS、Lab、LUV 三个色彩空间提取白色和黄色车道线

        参数：
            img: BGR图像

        返回：
            combined: 颜色阈值二值图
        """
        # HLS色彩空间 - 提取S通道（饱和度，黄色线）
        hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        s_channel = hls[:, :, 2]
        s_binary = np.zeros_like(s_channel)
        s_binary[(s_channel >= self.hls_s_thresh[0]) &
                 (s_channel <= self.hls_s_thresh[1])] = 1

        # Lab色彩空间 - 提取B通道（黄色分量）
        lab = cv2.cvtColor(img, cv2.COLOR_BGR2Lab)
        b_channel = lab[:, :, 2]
        b_binary = np.zeros_like(b_channel)
        b_binary[(b_channel >= self.lab_b_thresh[0]) &
                 (b_channel <= self.lab_b_thresh[1])] = 1

        # LUV色彩空间 - 提取L通道（亮度，白色线）
        luv = cv2.cvtColor(img, cv2.COLOR_BGR2LUV)
        l_channel = luv[:, :, 0]
        l_binary = np.zeros_like(l_channel)
        l_binary[(l_channel >= self.luv_l_thresh[0]) &
                 (l_channel <= self.luv_l_thresh[1])] = 1

        # 组合
        combined = np.zeros_like(s_channel)
        combined[(s_binary == 1) | (b_binary == 1) | (l_binary == 1)] = 1

        return combined

    def combined_threshold(self, img):
        """
        组合阈值（Sobel + 颜色）

        参数：
            img: BGR图像

        返回：
            binary: 组合二值图
        """
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Sobel边缘
        sobel_binary = self.sobel_edge_detection(gray)

        # 颜色阈值
        color_binary = self.color_threshold(img)

        # 组合
        combined = np.zeros_like(sobel_binary)
        combined[(sobel_binary == 1) | (color_binary == 1)] = 1

        return combined

    def sliding_window_search(self, binary_warped, n_windows=None, window_width=None,
                               min_pix=None, visualize=False):
        """
        滑动窗口搜索车道线像素

        算法流程：
        1. 统计底部直方图，找到左右车道线起始位置
        2. 从底部向上逐窗口搜索
        3. 计算窗口内非零像素均值，更新窗口中心
        4. 收集所有车道线像素点

        参数：
            binary_warped: 透视变换后的二值图
            n_windows: 窗口数量（可选）
            window_width: 窗口宽度（可选）
            min_pix: 最小像素数阈值（可选）
            visualize: 是否返回可视化窗口

        返回：
            left_fit, right_fit: 左右车道线多项式系数
            left_lane, right_lane: 左右车道线像素索引
            out_img: 可视化图像（如果visualize=True）
        """
        if n_windows is None:
            n_windows = self.n_windows
        if window_width is None:
            window_width = self.window_width
        if min_pix is None:
            min_pix = self.min_pixels

        h, w = binary_warped.shape
        window_height = h // n_windows

        # 统计底部1/3的直方图
        histogram = np.sum(binary_warped[h * 2 // 3:, :], axis=0)

        # 找到左右峰值（中点分开）
        midpoint = w // 2
        left_x_base = np.argmax(histogram[:midpoint])
        right_x_base = np.argmax(histogram[midpoint:]) + midpoint

        # 获取所有非零像素
        nonzero = binary_warped.nonzero()
        nonzero_y = np.array(nonzero[0])
        nonzero_x = np.array(nonzero[1])

        # 当前窗口中心
        left_x_current = left_x_base
        right_x_current = right_x_base

        # 收集像素索引
        left_lane_inds = []
        right_lane_inds = []

        # 可视化
        if visualize:
            out_img = np.dstack((binary_warped, binary_warped, binary_warped)) * 255

        # 滑动窗口
        for i in range(n_windows):
            # 窗口边界
            y_low = h - (i + 1) * window_height
            y_high = h - i * window_height

            x_left_low = left_x_current - window_width // 2
            x_left_high = left_x_current + window_width // 2
            x_right_low = right_x_current - window_width // 2
            x_right_high = right_x_current + window_width // 2

            # 可视化窗口
            if visualize:
                cv2.rectangle(out_img,
                              (x_left_low, y_low), (x_left_high, y_high),
                              (0, 255, 0), 2)
                cv2.rectangle(out_img,
                              (x_right_low, y_low), (x_right_high, y_high),
                              (0, 255, 0), 2)

            # 找到窗口内的非零像素
            good_left = ((nonzero_y >= y_low) & (nonzero_y < y_high) &
                         (nonzero_x >= x_left_low) & (nonzero_x < x_left_high)).nonzero()[0]
            good_right = ((nonzero_y >= y_low) & (nonzero_y < y_high) &
                          (nonzero_x >= x_right_low) & (nonzero_x < x_right_high)).nonzero()[0]

            left_lane_inds.append(good_left)
            right_lane_inds.append(good_right)

            # 如果足够多像素，更新窗口中心
            if len(good_left) > min_pix:
                left_x_current = int(np.mean(nonzero_x[good_left]))
            if len(good_right) > min_pix:
                right_x_current = int(np.mean(nonzero_x[good_right]))

        # 合并索引
        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)

        # 提取像素坐标
        left_x = nonzero_x[left_lane_inds]
        left_y = nonzero_y[left_lane_inds]
        right_x = nonzero_x[right_lane_inds]
        right_y = nonzero_y[right_lane_inds]

        # 二阶多项式拟合
        left_fit = None
        right_fit = None

        if len(left_x) > 0:
            left_fit = np.polyfit(left_y, left_x, 2)
        if len(right_x) > 0:
            right_fit = np.polyfit(right_y, right_x, 2)

        # 可视化标记像素
        if visualize:
            out_img[left_y, left_x] = [255, 0, 0]   # 红色：左车道
            out_img[right_y, right_x] = [0, 0, 255]  # 蓝色：右车道
            return left_fit, right_fit, (left_lane_inds, right_lane_inds), out_img

        return left_fit, right_fit, (left_lane_inds, right_lane_inds), None

    def search_around_poly(self, binary_warped, left_fit, right_fit, margin=100):
        """
        基于已有多项式的区域搜索（比滑动窗口更快）

        参数：
            binary_warped: 透视变换后的二值图
            left_fit: 左车道线多项式系数
            right_fit: 右车道线多项式系数
            margin: 搜索边距

        返回：
            left_fit, right_fit: 更新后的多项式系数
            left_lane, right_lane: 像素索引
        """
        nonzero = binary_warped.nonzero()
        nonzero_y = np.array(nonzero[0])
        nonzero_x = np.array(nonzero[1])

        # 根据多项式计算搜索区域
        left_lane_inds = ((nonzero_x > (left_fit[0] * nonzero_y ** 2 +
                                         left_fit[1] * nonzero_y +
                                         left_fit[2] - margin)) &
                          (nonzero_x < (left_fit[0] * nonzero_y ** 2 +
                                         left_fit[1] * nonzero_y +
                                         left_fit[2] + margin)))

        right_lane_inds = ((nonzero_x > (right_fit[0] * nonzero_y ** 2 +
                                          right_fit[1] * nonzero_y +
                                          right_fit[2] - margin)) &
                           (nonzero_x < (right_fit[0] * nonzero_y ** 2 +
                                          right_fit[1] * nonzero_y +
                                          right_fit[2] + margin)))

        # 提取像素
        left_x = nonzero_x[left_lane_inds]
        left_y = nonzero_y[left_lane_inds]
        right_x = nonzero_x[right_lane_inds]
        right_y = nonzero_y[right_lane_inds]

        # 重新拟合
        new_left_fit = left_fit
        new_right_fit = right_fit

        if len(left_x) > 0:
            new_left_fit = np.polyfit(left_y, left_x, 2)
        if len(right_x) > 0:
            new_right_fit = np.polyfit(right_y, right_x, 2)

        return new_left_fit, new_right_fit, (left_lane_inds, right_lane_inds)

    def calculate_curvature(self, left_fit, right_fit, y_eval=720):
        """
        计算车道曲率半径

        参数：
            left_fit: 左车道多项式系数
            right_fit: 右车道多项式系数
            y_eval: 计算曲率的y坐标（通常是图像底部）

        返回：
            left_curvature: 左车道曲率半径（米）
            right_curvature: 右车道曲率半径（米）
        """
        # 转换到实际世界坐标
        left_fit_world = np.polyfit(
            np.array([0, 360, 720]) * self.ym_per_pix,
            np.array([left_fit[0] * 720 ** 2 + left_fit[1] * 720 + left_fit[2],
                       left_fit[0] * 360 ** 2 + left_fit[1] * 360 + left_fit[2],
                       left_fit[2]]) * self.xm_per_pix,
            2
        ) if left_fit is not None else None

        right_fit_world = np.polyfit(
            np.array([0, 360, 720]) * self.ym_per_pix,
            np.array([right_fit[0] * 720 ** 2 + right_fit[1] * 720 + right_fit[2],
                       right_fit[0] * 360 ** 2 + right_fit[1] * 360 + right_fit[2],
                       right_fit[2]]) * self.xm_per_pix,
            2
        ) if right_fit is not None else None

        y_eval_m = y_eval * self.ym_per_pix

        left_curv = None
        right_curv = None

        if left_fit_world is not None:
            left_curv = ((1 + (2 * left_fit_world[0] * y_eval_m + left_fit_world[1]) ** 2) ** 1.5
                         / np.absolute(2 * left_fit_world[0]))

        if right_fit_world is not None:
            right_curv = ((1 + (2 * right_fit_world[0] * y_eval_m + right_fit_world[1]) ** 2) ** 1.5
                          / np.absolute(2 * right_fit_world[0]))

        return left_curv, right_curv

    def calculate_offset(self, left_fit, right_fit, img_width=1280):
        """
        计算车辆相对于车道中心的偏移

        参数：
            left_fit: 左车道多项式
            right_fit: 右车道多项式
            img_width: 图像宽度

        返回：
            offset: 偏移量（米），正=右偏，负=左偏
        """
        if left_fit is None or right_fit is None:
            return 0.0

        y_eval = 720
        left_x = left_fit[0] * y_eval ** 2 + left_fit[1] * y_eval + left_fit[2]
        right_x = right_fit[0] * y_eval ** 2 + right_fit[1] * y_eval + right_fit[2]

        lane_center = (left_x + right_x) / 2
        car_center = img_width / 2

        offset = (car_center - lane_center) * self.xm_per_pix
        return offset

    def smooth_fit(self, left_fit, right_fit):
        """
        平滑多项式系数（使用历史均值）

        参数：
            left_fit: 当前左车道多项式
            right_fit: 当前右车道多项式

        返回：
            smoothed_left, smoothed_right: 平滑后的多项式
        """
        with self._lock:
            if left_fit is not None:
                self.left_fit_history.append(left_fit)
            if right_fit is not None:
                self.right_fit_history.append(right_fit)

            smoothed_left = np.mean(self.left_fit_history, axis=0) if self.left_fit_history else left_fit
            smoothed_right = np.mean(self.right_fit_history, axis=0) if self.right_fit_history else right_fit

        return smoothed_left, smoothed_right

    def draw_lane(self, img, binary_warped, left_fit, right_fit):
        """
        在原图上绘制检测到的车道

        参数：
            img: 原始BGR图像
            binary_warped: 透视变换后的二值图
            left_fit: 左车道多项式
            right_fit: 右车道多项式

        返回：
            result: 绘制了车道的图像
        """
        h, w = img.shape[:2]
        plot_y = np.linspace(0, h - 1, h)

        # 创建空白图
        warp_zero = np.zeros_like(binary_warped).astype(np.uint8)
        color_warp = np.dstack((warp_zero, warp_zero, warp_zero))

        if left_fit is not None and right_fit is not None:
            left_fit_x = left_fit[0] * plot_y ** 2 + left_fit[1] * plot_y + left_fit[2]
            right_fit_x = right_fit[0] * plot_y ** 2 + right_fit[1] * plot_y + right_fit[2]

            # 构造多边形
            pts_left = np.array([np.transpose(np.vstack([left_fit_x, plot_y]))])
            pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fit_x, plot_y])))])
            pts = np.hstack((pts_left, pts_right))

            # 绘制填充区域（绿色半透明）
            cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))

            # 绘制车道线
            pts_left_int = np.array([np.transpose(np.vstack([left_fit_x, plot_y]))], dtype=np.int32)
            pts_right_int = np.array([np.transpose(np.vstack([right_fit_x, plot_y]))], dtype=np.int32)
            cv2.polylines(color_warp, pts_left_int, False, (255, 0, 0), 5)
            cv2.polylines(color_warp, pts_right_int, False, (0, 0, 255), 5)

        # 逆透视变换
        newwarp = cv2.warpPerspective(color_warp, self.M_inv, (w, h))

        # 叠加
        result = cv2.addWeighted(img, 1, newwarp, 0.3, 0)
        return result

    def draw_info(self, img, curvature, offset, fps=0):
        """
        在图像上绘制信息文字

        参数：
            img: 输入图像
            curvature: 曲率半径（米）
            offset: 偏移量（米）
            fps: 帧率

        返回：
            img: 绘制了信息的图像
        """
        # 曲率信息
        if curvature is not None:
            cv2.putText(img, f'Curvature: {curvature:.0f}m',
                        (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 2)

        # 偏移信息
        direction = "Right" if offset > 0 else "Left"
        cv2.putText(img, f'Offset: {abs(offset):.2f}m {direction}',
                    (30, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 2)

        # FPS
        if fps > 0:
            cv2.putText(img, f'FPS: {fps:.1f}',
                        (30, 150), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)

        return img

    def detect(self, frame, use_previous=True, visualize=True):
        """
        完整的车道线检测流水线

        参数：
            frame: 输入BGR图像
            use_previous: 是否使用上一帧结果加速搜索
            visualize: 是否生成可视化结果

        返回：
            dict: {
                'left_fit': 左车道多项式,
                'right_fit': 右车道多项式,
                'curvature': 曲率半径,
                'offset': 偏移量,
                'visualization': 可视化图像
            }
        """
        t_start = time.time()

        # 1. 组合阈值
        binary = self.combined_threshold(frame)

        # 2. 透视变换
        binary_warped = self.perspective_transform(binary)

        # 3. 车道线搜索
        with self._lock:
            has_previous = (len(self.left_fit_history) > 0 and
                            len(self.right_fit_history) > 0 and
                            use_previous)

        if has_previous:
            with self._lock:
                prev_left = self.left_fit_history[-1]
                prev_right = self.right_fit_history[-1]
            left_fit, right_fit, lane_inds = self.search_around_poly(
                binary_warped, prev_left, prev_right
            )
        else:
            left_fit, right_fit, lane_inds, _ = self.sliding_window_search(binary_warped)

        # 4. 平滑处理
        left_fit, right_fit = self.smooth_fit(left_fit, right_fit)

        # 5. 计算曲率和偏移
        left_curv, right_curv = self.calculate_curvature(left_fit, right_fit)
        curvature = None
        if left_curv is not None and right_curv is not None:
            curvature = (left_curv + right_curv) / 2

        offset = self.calculate_offset(left_fit, right_fit)

        # 6. 可视化
        vis = None
        if visualize:
            vis = self.draw_lane(frame, binary_warped, left_fit, right_fit)
            fps = 1.0 / max(time.time() - t_start, 1e-6)
            vis = self.draw_info(vis, curvature, offset, fps)

        return {
            'left_fit': left_fit,
            'right_fit': right_fit,
            'curvature': curvature,
            'offset': offset,
            'binary': binary,
            'binary_warped': binary_warped,
            'visualization': vis
        }


class LaneDetectionPipeline:
    """
    车道线检测流水线（支持多线程预处理）

    使用示例：
        pipeline = LaneDetectionPipeline()
        pipeline.start(camera_id=0)

        while True:
            result = pipeline.get_result()
            if result:
                cv2.imshow('Lane', result['visualization'])
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        pipeline.stop()
    """

    def __init__(self, camera_id=0, resolution=(1280, 720), config=None):
        """
        初始化流水线

        参数：
            camera_id: 摄像头ID
            resolution: 分辨率 (宽, 高)
            config: 检测器配置
        """
        self.camera_id = camera_id
        self.resolution = resolution
        self.detector = AdvancedLaneDetector(config)

        self._cap = None
        self._frame = None
        self._result = None
        self._running = False
        self._lock = threading.Lock()

        self._capture_thread = None
        self._process_thread = None

    def start(self):
        """启动检测流水线"""
        self._cap = cv2.VideoCapture(self.camera_id)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])

        self._running = True

        # 采集线程
        self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._capture_thread.start()

        # 处理线程
        self._process_thread = threading.Thread(target=self._process_loop, daemon=True)
        self._process_thread.start()

        print(f"[流水线] 已启动 (摄像头: {self.camera_id})")

    def stop(self):
        """停止流水线"""
        self._running = False
        if self._capture_thread:
            self._capture_thread.join(timeout=2)
        if self._process_thread:
            self._process_thread.join(timeout=2)
        if self._cap:
            self._cap.release()
        print("[流水线] 已停止")

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
                result = self.detector.detect(frame, use_previous=True, visualize=True)
                with self._lock:
                    self._result = result

    def get_result(self):
        """获取最新结果"""
        with self._lock:
            return self._result


# ================================================================
#                          使用示例
# ================================================================
if __name__ == '__main__':
    print("=" * 60)
    print("高级车道线检测 - Advanced Lane Detection")
    print("针对 Orange Pi 5 优化")
    print("=" * 60)

    # 示例1：基本使用
    detector = AdvancedLaneDetector()

    # 从摄像头读取
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    print("\n按 'q' 退出")
    print("按 'p' 调整透视变换点")
    print("按 's' 切换滑动窗口/区域搜索")

    use_sliding = True

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        result = detector.detect(frame, use_previous=not use_sliding)

        if result['visualization'] is not None:
            # 显示二值图
            binary_small = cv2.resize(result['binary_warped'], (320, 180))
            result['visualization'][0:180, 0:320] = cv2.cvtColor(
                binary_small * 255, cv2.COLOR_GRAY2BGR)

            cv2.imshow('Lane Detection', result['visualization'])

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            use_sliding = not use_sliding
            print(f"搜索模式: {'滑动窗口' if use_sliding else '区域搜索'}")

    cap.release()
    cv2.destroyAllWindows()
