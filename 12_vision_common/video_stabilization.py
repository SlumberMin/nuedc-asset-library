# -*- coding: utf-8 -*-
"""
视频稳定模块 - 光流 + 仿射变换 + 平滑
适用于电赛中手持/车载摄像头的实时视频稳定
"""

import cv2
import numpy as np
from collections import deque


class VideoStabilizer:
    """视频稳定器：基于光流跟踪的实时帧间稳定"""

    def __init__(self, smooth_radius=30, crop_ratio=0.05,
                 lk_win_size=21, max_corners=200):
        """
        参数：
            smooth_radius: 轨迹平滑窗口半径（帧数）
            crop_ratio: 稳定后裁剪黑边比例
            lk_win_size: 光流跟踪窗口大小
            max_corners: 最大特征点数
        """
        self.smooth_radius = smooth_radius
        self.crop_ratio = crop_ratio
        self.lk_win_size = lk_win_size

        # Shi-Tomasi角点检测参数
        self.feature_params = dict(
            maxCorners=max_corners,
            qualityLevel=0.01,
            minDistance=30,
            blockSize=7
        )
        # 光流参数
        self.lk_params = dict(
            winSize=(lk_win_size, lk_win_size),
            maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
        )

        self.prev_gray = None
        self.transforms = []       # 历史变换列表
        self.smoothed_trajectory = None
        self.frame_buffer = deque(maxlen=smooth_radius * 2 + 1)

    def _get_transform(self, prev_gray, curr_gray):
        """计算两帧之间的仿射变换矩阵"""
        # 检测角点
        prev_pts = cv2.goodFeaturesToTrack(prev_gray, **self.feature_params)
        if prev_pts is None or len(prev_pts) < 10:
            return np.array([0.0, 0.0, 0.0])  # 无足够特征点

        # Lucas-Kanade光流跟踪
        curr_pts, status, _ = cv2.calcOpticalFlowPyrLK(
            prev_gray, curr_gray, prev_pts, None, **self.lk_params
        )

        # 过滤有效跟踪点
        idx = np.where(status == 1)[0]
        if len(idx) < 6:
            return np.array([0.0, 0.0, 0.0])

        prev_pts = prev_pts[idx]
        curr_pts = curr_pts[idx]

        # 估计仿射变换（RANSAC去除外点）
        M, inliers = cv2.estimateAffinePartial2D(prev_pts, curr_pts, method=cv2.RANSAC)

        if M is None:
            return np.array([0.0, 0.0, 0.0])

        # 提取dx, dy, dθ
        dx = M[0, 2]
        dy = M[1, 2]
        da = np.arctan2(M[1, 0], M[0, 0])

        return np.array([dx, dy, da])

    def _smooth_trajectory(self, trajectory):
        """滑动窗口平滑轨迹"""
        smoothed = np.copy(trajectory)
        n = len(trajectory)
        for i in range(n):
            start = max(0, i - self.smooth_radius)
            end = min(n, i + self.smooth_radius + 1)
            smoothed[i] = np.mean(trajectory[start:end], axis=0)
        return smoothed

    def stabilize_frame(self, frame):
        """
        稳定单帧图像（实时处理模式）
        参数：frame - 当前帧 BGR图像
        返回：稳定后的帧
        """
        curr_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.prev_gray is None:
            self.prev_gray = curr_gray
            self.transforms.append(np.array([0.0, 0.0, 0.0]))
            return frame

        # 计算帧间变换
        dx, dy, da = self._get_transform(self.prev_gray, curr_gray)
        self.transforms.append(np.array([dx, dy, da]))
        self.prev_gray = curr_gray

        # 累积轨迹
        trajectory = np.cumsum(self.transforms, axis=0)

        # 平滑轨迹
        smoothed = self._smooth_trajectory(trajectory)

        # 计算补偿变换
        diff = smoothed[-1] - trajectory[-1]
        dx_comp = diff[0]
        dy_comp = diff[1]
        da_comp = diff[2]

        # 构建补偿矩阵
        h, w = frame.shape[:2]
        cx, cy = w // 2, h // 2
        M = cv2.getRotationMatrix2D((cx, cy), np.rad2deg(da_comp), 1.0)
        M[0, 2] += dx_comp
        M[1, 2] += dy_comp

        stabilized = cv2.warpAffine(frame, M, (w, h), borderMode=cv2.BORDER_REFLECT)

        # 裁剪黑边
        if self.crop_ratio > 0:
            margin_x = int(w * self.crop_ratio)
            margin_y = int(h * self.crop_ratio)
            stabilized = stabilized[margin_y:h - margin_y, margin_x:w - margin_x]
            stabilized = cv2.resize(stabilized, (w, h))

        return stabilized

    def stabilize_video(self, input_path, output_path=None):
        """
        离线稳定整个视频文件
        参数：
            input_path: 输入视频路径
            output_path: 输出视频路径（None则自动生成）
        """
        cap = cv2.VideoCapture(input_path)
        if not cap.isOpened():
            print(f"[错误] 无法打开视频: {input_path}")
            return None

        fps = cap.get(cv2.CAP_PROP_FPS)
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

        if output_path is None:
            output_path = input_path.rsplit('.', 1)[0] + '_stabilized.mp4'

        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        writer = cv2.VideoWriter(output_path, fourcc, fps, (w, h))

        # 第一遍：收集所有变换
        print(f"第一遍：分析视频帧间运动 ({total} 帧)...")
        transforms = []
        prev_gray = None
        frame_idx = 0

        while True:
            ret, frame = cap.read()
            if not ret:
                break
            curr_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            if prev_gray is not None:
                t = self._get_transform(prev_gray, curr_gray)
                transforms.append(t)
            else:
                transforms.append(np.array([0.0, 0.0, 0.0]))
            prev_gray = curr_gray
            frame_idx += 1
            if frame_idx % 100 == 0:
                print(f"  已分析 {frame_idx}/{total} 帧")

        # 计算平滑轨迹
        trajectory = np.cumsum(transforms, axis=0)
        smoothed = self._smooth_trajectory(trajectory)
        corrections = smoothed - trajectory

        # 第二遍：应用稳定
        print("第二遍：应用稳定变换...")
        cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
        frame_idx = 0
        margin_x = int(w * self.crop_ratio)
        margin_y = int(h * self.crop_ratio)

        while True:
            ret, frame = cap.read()
            if not ret:
                break
            dx, dy, da = corrections[frame_idx]
            cx, cy = w // 2, h // 2
            M = cv2.getRotationMatrix2D((cx, cy), np.rad2deg(da), 1.0)
            M[0, 2] += dx
            M[1, 2] += dy
            stabilized = cv2.warpAffine(frame, M, (w, h), borderMode=cv2.BORDER_REFLECT)
            if margin_x > 0 and margin_y > 0:
                stabilized = stabilized[margin_y:h - margin_y, margin_x:w - margin_x]
                stabilized = cv2.resize(stabilized, (w, h))
            writer.write(stabilized)
            frame_idx += 1
            if frame_idx % 100 == 0:
                print(f"  已处理 {frame_idx}/{total} 帧")

        cap.release()
        writer.release()
        print(f"稳定视频已保存: {output_path}")
        return output_path


# ========== 使用示例 ==========
if __name__ == '__main__':
    print("VideoStabilizer 视频稳定模块")
    print("=" * 40)

    import sys
    if len(sys.argv) > 1:
        stabilizer = VideoStabilizer(smooth_radius=30, crop_ratio=0.05)
        stabilizer.stabilize_video(sys.argv[1])
    else:
        print("用法: python video_stabilization.py <视频文件路径>")
        print("\n实时模式示例：")
        print("  stabilizer = VideoStabilizer()")
        print("  cap = cv2.VideoCapture(0)")
        print("  while True:")
        print("      ret, frame = cap.read()")
        print("      stable = stabilizer.stabilize_frame(frame)")
        print("      cv2.imshow('Stable', stable)")
        print("\n离线模式示例：")
        print("  stabilizer = VideoStabilizer()")
        print("  stabilizer.stabilize_video('input.mp4', 'output.mp4')")
