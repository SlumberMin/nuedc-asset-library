"""
速度估计模块 - 基于视觉的目标运动速度计算
适用于电赛中跟踪目标并估算其运动速度
"""

import cv2
import numpy as np
import math
import time
from collections import deque


class SpeedEstimator:
    """基于视觉的速度估计器"""

    def __init__(self, focal_length_px=350, fps=30, pixel_to_meter=None, history_len=10):
        """
        初始化速度估计器

        Args:
            focal_length_px: 焦距(像素)
            fps: 帧率
            pixel_to_meter: 像素到米的换算比例(已知距离时可预设)
            history_len: 位置历史长度
        """
        self.focal_length_px = focal_length_px
        self.fps = fps
        self.pixel_to_meter = pixel_to_meter
        self.history_len = history_len
        # 多目标轨迹 {track_id: deque([(x, y, t), ...])}
        self.tracks = {}

    def set_pixel_to_meter(self, known_distance_m, known_size_m, known_pixel_size):
        """
        设置像素到实际距离的换算比例

        Args:
            known_distance_m: 已知距离(米)
            known_size_m: 已知物体实际尺寸(米)
            known_pixel_size: 该物体在图像中的像素尺寸
        """
        # 实际距离/像素 = 换算系数
        self.pixel_to_meter = (known_size_m / known_pixel_size) * \
                              (known_distance_m / known_distance_m)  # 简化为比例
        self.pixel_to_meter = known_size_m / known_pixel_size

    def _get_scale_at_distance(self, distance_m):
        """
        获取指定距离下的像素-米换算比例

        Args:
            distance_m: 距离(米)

        Returns:
            每像素对应的实际距离(米)
        """
        if self.pixel_to_meter:
            return self.pixel_to_meter
        return distance_m / self.focal_length_px

    def update_track(self, track_id, x, y, timestamp=None):
        """
        更新目标轨迹

        Args:
            track_id: 目标ID
            x, y: 当前位置(像素)
            timestamp: 时间戳(秒), 默认使用当前时间
        """
        if timestamp is None:
            timestamp = time.time()

        if track_id not in self.tracks:
            self.tracks[track_id] = deque(maxlen=self.history_len)

        self.tracks[track_id].append((x, y, timestamp))

    def update_track_center(self, track_id, contour, timestamp=None):
        """
        通过轮廓中心更新轨迹

        Args:
            track_id: 目标ID
            contour: OpenCV轮廓
            timestamp: 时间戳
        """
        M = cv2.moments(contour)
        if M['m00'] == 0:
            return
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        self.update_track(track_id, cx, cy, timestamp)

    def get_pixel_speed(self, track_id):
        """
        获取目标像素速度

        Args:
            track_id: 目标ID

        Returns:
            (speed_px_per_sec, dx, dy) - 像素速度和分量
        """
        if track_id not in self.tracks or len(self.tracks[track_id]) < 2:
            return 0.0, 0.0, 0.0

        pts = list(self.tracks[track_id])
        x0, y0, t0 = pts[-2]
        x1, y1, t1 = pts[-1]

        dt = t1 - t0
        if dt <= 0:
            return 0.0, 0.0, 0.0

        dx = x1 - x0
        dy = y1 - y0
        speed = math.sqrt(dx ** 2 + dy ** 2) / dt

        return speed, dx / dt, dy / dt

    def get_actual_speed(self, track_id, distance_m=None):
        """
        获取目标实际速度(m/s)

        Args:
            track_id: 目标ID
            distance_m: 目标距离(米)，用于换算

        Returns:
            (speed_mps, vx, vy) - 实际速度(m/s)和分量
        """
        speed_px, vx_px, vy_px = self.get_pixel_speed(track_id)

        if distance_m:
            scale = self._get_scale_at_distance(distance_m)
        elif self.pixel_to_meter:
            scale = self.pixel_to_meter
        else:
            # 无法换算，返回像素速度
            return speed_px, vx_px, vy_px

        return speed_px * scale, vx_px * scale, vy_px * scale

    def get_average_speed(self, track_id, n_frames=5, distance_m=None):
        """
        获取多帧平均速度(平滑)

        Args:
            track_id: 目标ID
            n_frames: 用于平均的帧数
            distance_m: 距离(米)

        Returns:
            平均速度(m/s 或 px/s)
        """
        if track_id not in self.tracks or len(self.tracks[track_id]) < 2:
            return 0.0

        pts = list(self.tracks[track_id])
        speeds = []
        for i in range(max(1, len(pts) - n_frames), len(pts)):
            x0, y0, t0 = pts[i - 1]
            x1, y1, t1 = pts[i]
            dt = t1 - t0
            if dt > 0:
                dist = math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)
                speeds.append(dist / dt)

        if not speeds:
            return 0.0

        avg_speed = np.mean(speeds)

        if distance_m:
            scale = self._get_scale_at_distance(distance_m)
            return avg_speed * scale
        elif self.pixel_to_meter:
            return avg_speed * self.pixel_to_meter

        return avg_speed

    def get_direction(self, track_id):
        """
        获取运动方向(角度)

        Args:
            track_id: 目标ID

        Returns:
            方向角度(度), 0=向右, 90=向下
        """
        speed, vx, vy = self.get_pixel_speed(track_id)
        if speed == 0:
            return None
        return math.degrees(math.atan2(vy, vx))

    def get_displacement(self, track_id):
        """
        获取总位移

        Args:
            track_id: 目标ID

        Returns:
            (displacement_px, displacement_x, displacement_y)
        """
        if track_id not in self.tracks or len(self.tracks[track_id]) < 2:
            return 0.0, 0.0, 0.0

        pts = list(self.tracks[track_id])
        dx = pts[-1][0] - pts[0][0]
        dy = pts[-1][1] - pts[0][1]
        return math.sqrt(dx ** 2 + dy ** 2), dx, dy

    def get_path_length(self, track_id):
        """
        获取轨迹总长度(累计路径)

        Args:
            track_id: 目标ID

        Returns:
            路径长度(像素)
        """
        if track_id not in self.tracks or len(self.tracks[track_id]) < 2:
            return 0.0

        pts = list(self.tracks[track_id])
        total = 0.0
        for i in range(1, len(pts)):
            dx = pts[i][0] - pts[i - 1][0]
            dy = pts[i][1] - pts[i - 1][1]
            total += math.sqrt(dx ** 2 + dy ** 2)
        return total

    def remove_track(self, track_id):
        """删除轨迹"""
        self.tracks.pop(track_id, None)

    def clear_all(self):
        """清除所有轨迹"""
        self.tracks.clear()

    def draw_tracks(self, frame, color=(0, 255, 0)):
        """
        绘制所有轨迹和速度信息

        Args:
            frame: 输入图像
            color: 轨迹颜色

        Returns:
            绘制后的图像
        """
        for tid, pts in self.tracks.items():
            pts_list = list(pts)
            # 绘制轨迹线
            for i in range(1, len(pts_list)):
                pt1 = (int(pts_list[i - 1][0]), int(pts_list[i - 1][1]))
                pt2 = (int(pts_list[i][0]), int(pts_list[i][1]))
                cv2.line(frame, pt1, pt2, color, 2)

            # 当前位置
            if pts_list:
                cx, cy = int(pts_list[-1][0]), int(pts_list[-1][1])
                cv2.circle(frame, (cx, cy), 5, color, -1)

                # 速度标注
                speed, _, _ = self.get_pixel_speed(tid)
                cv2.putText(frame, f"ID{tid}: {speed:.0f}px/s",
                            (cx + 10, cy - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        return frame


class OpticalFlowSpeedEstimator:
    """基于光流的速度估计器(适用于无明确目标ID的场景)"""

    def __init__(self, pixel_to_meter=1.0):
        """
        Args:
            pixel_to_meter: 像素到米的换算系数
        """
        self.pixel_to_meter = pixel_to_meter
        self.prev_gray = None
        self.prev_time = None

    def estimate_dense_flow_speed(self, frame):
        """
        稠密光流估计整体运动速度

        Args:
            frame: 当前帧(BGR)

        Returns:
            (avg_speed, angle, flow) - 平均速度、方向角、光流场
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        now = time.time()

        if self.prev_gray is None:
            self.prev_gray = gray
            self.prev_time = now
            return 0.0, 0.0, None

        dt = now - self.prev_time
        if dt <= 0:
            dt = 1.0 / 30

        flow = cv2.calcOpticalFlowFarneback(
            self.prev_gray, gray, None,
            pyr_scale=0.5, levels=3, winsize=15,
            iterations=3, poly_n=5, poly_sigma=1.2, flags=0
        )

        # 计算速度分量
        mag, ang = cv2.cartToPolar(flow[..., 0], flow[..., 1], angleInDegrees=True)

        avg_speed = np.mean(mag) / dt * self.pixel_to_meter
        avg_angle = np.mean(ang)

        self.prev_gray = gray
        self.prev_time = now

        return avg_speed, avg_angle, flow


# ==================== 使用示例 ====================
if __name__ == '__main__':
    estimator = SpeedEstimator(focal_length_px=350, pixel_to_meter=0.001)

    # 模拟目标移动
    positions = [(100, 100), (105, 102), (112, 105), (120, 110), (130, 115)]
    t = time.time()
    for i, (x, y) in enumerate(positions):
        estimator.update_track(0, x, y, timestamp=t + i * 0.033)

    speed, vx, vy = estimator.get_actual_speed(0)
    print(f"速度: {speed:.4f} m/s")
    print(f"分量: vx={vx:.4f}, vy={vy:.4f}")
    print(f"方向: {estimator.get_direction(0):.1f}°")
    print(f"轨迹长度: {estimator.get_path_length(0):.1f} px")
    print(f"平均速度: {estimator.get_average_speed(0):.1f} px/s")
