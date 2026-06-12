#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
摄像头透视变换深度实现 - 电赛视觉核心
======================================
核心内容:
  1. 针孔相机模型 + 畸变模型
  2. 单应性矩阵(Homography)求解
     - 4点对应法 (DLT)
     - RANSAC鲁棒估计
  3. 相机标定 (张正友标定法原理)
  4. 图像坐标 ↔ 世界坐标转换
  5. 鸟瞰图变换
  6. 电赛典型应用: 巡线/定位/测量

数学基础:
  相机内参: K = [f  0 cx; 0  f cy; 0  0  1]
  投影方程: [u,v,1]^T = K * [R|t] * [X,Y,Z,1]^T
  单应性:   [u,v,1]^T = H * [X,Y,1]^T  (平面场景)
  畸变:     r^2 = x'^2 + y'^2
            xd = x'(1 + k1*r^2 + k2*r^4 + k3*r^6)
            yd = y'(1 + k1*r^2 + k2*r^4 + k3*r^6)
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib
from matplotlib.patches import FancyArrowPatch, Rectangle
import warnings
warnings.filterwarnings('ignore')

matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False


# ==============================================================================
# 1. 相机模型
# ==============================================================================
class PinholeCamera:
    """针孔相机模型"""
    def __init__(self, fx=800, fy=800, cx=320, cy=240, img_width=640, img_height=480):
        """
        fx, fy: 焦距 [像素]
        cx, cy: 主点坐标 [像素]
        img_width, img_height: 图像尺寸
        """
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.img_width = img_width
        self.img_height = img_height

        # 内参矩阵
        self.K = np.array([
            [fx, 0,  cx],
            [0,  fy, cy],
            [0,  0,  1]
        ])

        # 畸变系数 [k1, k2, p1, p2, k3]
        self.dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

    def project(self, points_3d):
        """
        3D世界坐标 -> 2D图像坐标
        points_3d: Nx3 array
        返回: Nx2 array (像素坐标)
        """
        points_3d = np.array(points_3d)
        if points_3d.ndim == 1:
            points_3d = points_3d.reshape(1, -1)

        # 投影到归一化平面
        X = points_3d[:, 0]
        Y = points_3d[:, 1]
        Z = points_3d[:, 2]

        # 避免除零
        Z = np.where(np.abs(Z) < 1e-6, 1e-6, Z)

        x = X / Z
        y = Y / Z

        # 应用畸变
        x_d, y_d = self._apply_distortion(x, y)

        # 转换为像素坐标
        u = self.fx * x_d + self.cx
        v = self.fy * y_d + self.cy

        return np.column_stack([u, v])

    def _apply_distortion(self, x, y):
        """应用径向和切向畸变"""
        k1, k2, p1, p2, k3 = self.dist_coeffs
        r2 = x**2 + y**2
        r4 = r2**2
        r6 = r4 * r2

        # 径向畸变
        radial = 1 + k1 * r2 + k2 * r4 + k3 * r6

        # 切向畸变
        dx = 2 * p1 * x * y + p2 * (r2 + 2 * x**2)
        dy = p1 * (r2 + 2 * y**2) + 2 * p2 * x * y

        x_d = x * radial + dx
        y_d = y * radial + dy

        return x_d, y_d

    def undistort(self, points_2d):
        """
        去畸变: 2D畸变坐标 -> 2D无畸变坐标
        使用迭代法近似
        """
        points_2d = np.array(points_2d)
        if points_2d.ndim == 1:
            points_2d = points_2d.reshape(1, -1)

        # 转到归一化坐标
        x_d = (points_2d[:, 0] - self.cx) / self.fx
        y_d = (points_2d[:, 1] - self.cy) / self.fy

        # 迭代求解
        x, y = x_d.copy(), y_d.copy()
        for _ in range(10):
            r2 = x**2 + y**2
            r4 = r2**2
            r6 = r4 * r2
            k1, k2, p1, p2, k3 = self.dist_coeffs

            radial = 1 + k1 * r2 + k2 * r4 + k3 * r6
            dx = 2 * p1 * x * y + p2 * (r2 + 2 * x**2)
            dy = p1 * (r2 + 2 * y**2) + 2 * p2 * x * y

            x_new = (x_d - dx) / radial
            y_new = (y_d - dy) / radial

            if np.max(np.abs(x_new - x)) < 1e-8:
                break
            x, y = x_new, y_new

        return np.column_stack([x, y])

    def set_distortion(self, k1, k2, p1=0, p2=0, k3=0):
        """设置畸变系数"""
        self.dist_coeffs = np.array([k1, k2, p1, p2, k3])


# ==============================================================================
# 2. 单应性矩阵 (Homography)
# ==============================================================================
class HomographySolver:
    """单应性矩阵求解器"""

    @staticmethod
    def dlt(src_points, dst_points):
        """
        DLT (Direct Linear Transform) 求解单应性矩阵
        src_points: Nx2 源点
        dst_points: Nx2 目标点
        返回: 3x3 单应性矩阵 H
        """
        src = np.array(src_points, dtype=float)
        dst = np.array(dst_points, dtype=float)

        assert len(src) >= 4, "至少需要4个点对"
        assert len(src) == len(dst), "点对数量必须一致"

        # 构建A矩阵
        n = len(src)
        A = np.zeros((2 * n, 9))

        for i in range(n):
            x, y = src[i]
            u, v = dst[i]
            A[2*i] = [-x, -y, -1, 0, 0, 0, u*x, u*y, u]
            A[2*i+1] = [0, 0, 0, -x, -y, -1, v*x, v*y, v]

        # SVD求解
        _, _, Vt = np.linalg.svd(A)
        H = Vt[-1].reshape(3, 3)
        H = H / H[2, 2]  # 归一化

        return H

    @staticmethod
    def compute_error(src, dst, H):
        """计算单应性变换的重投影误差"""
        src = np.array(src, dtype=float)
        dst = np.array(dst, dtype=float)

        # 变换源点
        src_h = np.column_stack([src, np.ones(len(src))])
        projected = (H @ src_h.T).T
        projected = projected[:, :2] / projected[:, 2:3]

        # 计算误差
        errors = np.linalg.norm(projected - dst, axis=1)
        return np.mean(errors), errors

    @staticmethod
    def ransac(src_points, dst_points, n_iterations=1000, threshold=3.0):
        """
        RANSAC鲁棒估计单应性矩阵
        threshold: 内点阈值 [像素]
        """
        src = np.array(src_points, dtype=float)
        dst = np.array(dst_points, dtype=float)
        n_points = len(src)

        best_H = None
        best_inliers = 0
        best_error = float('inf')

        for _ in range(n_iterations):
            # 随机选择4个点
            indices = np.random.choice(n_points, 4, replace=False)
            src_sample = src[indices]
            dst_sample = dst[indices]

            # DLT求解
            H = HomographySolver.dlt(src_sample, dst_sample)

            # 计算所有点的误差
            mean_err, errors = HomographySolver.compute_error(src, dst, H)

            # 统计内点
            inliers = np.sum(errors < threshold)

            if inliers > best_inliers or (inliers == best_inliers and mean_err < best_error):
                best_inliers = inliers
                best_H = H
                best_error = mean_err

                # 如果内点足够多，用所有内点重新拟合
                if inliers > n_points * 0.6:
                    inlier_mask = errors < threshold
                    if np.sum(inlier_mask) >= 4:
                        best_H = HomographySolver.dlt(src[inlier_mask], dst[inlier_mask])
                        best_error, _ = HomographySolver.compute_error(
                            src[inlier_mask], dst[inlier_mask], best_H
                        )

        return best_H, best_inliers, best_error

    @staticmethod
    def normalize_points(points):
        """DLT前的点归一化"""
        points = np.array(points, dtype=float)
        centroid = np.mean(points, axis=0)
        shifted = points - centroid
        avg_dist = np.mean(np.linalg.norm(shifted, axis=1))
        scale = np.sqrt(2) / avg_dist if avg_dist > 0 else 1.0

        T = np.array([
            [scale, 0, -scale * centroid[0]],
            [0, scale, -scale * centroid[1]],
            [0, 0, 1]
        ])

        points_h = np.column_stack([points, np.ones(len(points))])
        normalized = (T @ points_h.T).T

        return normalized[:, :2], T


# ==============================================================================
# 3. 相机标定 (张正友标定法原理)
# ==============================================================================
class CameraCalibration:
    """相机标定 (张正友标定法简化实现)"""

    @staticmethod
    def create_chessboard_points(square_size=0.025, rows=7, cols=9):
        """
        生成棋盘格角点的3D世界坐标
        square_size: 方格边长 [m]
        """
        obj_points = []
        for i in range(rows):
            for j in range(cols):
                obj_points.append([j * square_size, i * square_size, 0])

        return np.array(obj_points, dtype=np.float32)

    @staticmethod
    def find_chessboard_corners_simple(image, rows=7, cols=9):
        """
        简化的棋盘格角点检测
        (实际应用中应使用cv2.findChessboardCorners)
        这里模拟检测结果
        """
        # 模拟: 基于图像大小生成合理的角点
        h, w = image.shape[:2] if image is not None else (480, 640)

        corners = []
        margin_x = w * 0.15
        margin_y = h * 0.15

        for i in range(rows):
            for j in range(cols):
                x = margin_x + j * (w - 2 * margin_x) / (cols - 1)
                y = margin_y + i * (h - 2 * margin_y) / (rows - 1)
                corners.append([x, y])

        return np.array(corners, dtype=np.float32)

    @staticmethod
    def calibrate_single_view(obj_points, img_points):
        """
        单幅图像标定: 求解H矩阵和内参
        """
        # 归一化
        obj_norm, T_obj = HomographySolver.normalize_points(obj_points[:, :2])
        img_norm, T_img = HomographySolver.normalize_points(img_points)

        # DLT求H
        H_norm = HomographySolver.dlt(obj_norm, img_norm)

        # 反归一化
        H = np.linalg.inv(T_img) @ H_norm @ T_obj

        return H

    @staticmethod
    def extract_intrinsics_from_H(H_list):
        """
        从多个H矩阵提取内参 (张正友标定法)
        """
        V = []
        for H in H_list:
            h1, h2, h3 = H[:, 0], H[:, 1], H[:, 2]

            # 约束方程 (Zhang Zhengyou法: vij 向量, 6维)
            v12 = np.array([
                h1[0]*h2[0],
                h1[0]*h2[1] + h1[1]*h2[0],
                h1[1]*h2[1],
                h1[2]*h2[0] + h1[0]*h2[2],
                h1[2]*h2[1] + h1[1]*h2[2],
                h1[2]*h2[2]
            ])
            v11_minus_v22 = np.array([
                h1[0]*h1[0] - h2[0]*h2[0],
                2*(h1[0]*h1[1] - h2[0]*h2[1]),
                h1[1]*h1[1] - h2[1]*h2[1],
                2*(h1[2]*h1[0] - h2[2]*h2[0]),
                2*(h1[2]*h1[1] - h2[2]*h2[1]),
                h1[2]*h1[2] - h2[2]*h2[2]
            ])
            V.append(v12)
            V.append(v11_minus_v22)

        V = np.array(V)

        # SVD求解
        _, _, Vt = np.linalg.svd(V)
        b = Vt[-1]

        # 从b求内参
        B11, B12, B22 = b[0], b[1], b[2]

        cy = B12 * B11 / (B11 * B22 - B12**2) if abs(B11 * B22 - B12**2) > 1e-6 else 240
        cx = (B12 * cy / B11 - B12) * B11 / B22 if abs(B22) > 1e-6 else 320

        fy = np.sqrt(B11 / (B11 * B22 - B12**2)) if (B11 * B22 - B12**2) > 0 else 800
        fx = np.sqrt(B11) if B11 > 0 else 800

        K = np.array([
            [fx, 0,  cx],
            [0,  fy, cy],
            [0,  0,  1]
        ])

        return K


# ==============================================================================
# 4. 鸟瞰图变换
# ==============================================================================
class BirdseyeView:
    """鸟瞰图变换"""

    def __init__(self, camera=None):
        self.camera = camera or PinholeCamera()
        self.H = None
        self.output_size = (640, 480)

    def compute_homography_from_points(self, src_points, dst_points):
        """
        从4对对应点计算单应性矩阵
        src_points: 图像中的4个点
        dst_points: 对应的俯视图坐标
        """
        self.H = HomographySolver.dlt(src_points, dst_points)
        return self.H

    def compute_homography_ransac(self, src_points, dst_points, threshold=3.0):
        """RANSAC鲁棒估计"""
        self.H, n_inliers, error = HomographySolver.ransac(
            src_points, dst_points, threshold=threshold
        )
        return self.H, n_inliers, error

    def warp_image(self, image, output_size=None):
        """
        将图像变换为鸟瞰图
        使用逆映射 (避免空洞)
        """
        if self.H is None:
            raise ValueError("未计算单应性矩阵")

        out_size = output_size or self.output_size
        H_inv = np.linalg.inv(self.H)

        # 逆映射
        h_out, w_out = out_size[1], out_size[0]
        output = np.zeros((h_out, w_out, 3), dtype=np.uint8)

        # 生成输出网格
        Y, X = np.meshgrid(np.arange(h_out), np.arange(w_out), indexing='ij')
        ones = np.ones_like(X)
        coords = np.stack([X, Y, ones], axis=-1).reshape(-1, 3).T

        # 映射到源图像坐标
        src_coords = H_inv @ coords
        src_coords = src_coords[:2] / src_coords[2:3]

        # 双线性插值
        src_x = src_coords[0]
        src_y = src_coords[1]

        h_img, w_img = image.shape[:2]
        valid = (src_x >= 0) & (src_x < w_img - 1) & (src_y >= 0) & (src_y < h_img - 1)

        # 简化: 最近邻插值
        src_x_int = np.clip(np.round(src_x).astype(int), 0, w_img - 1)
        src_y_int = np.clip(np.round(src_y).astype(int), 0, h_img - 1)

        for c in range(3):
            output_flat = output[:, :, c].reshape(-1)
            source_flat = image[src_y_int, src_x_int, c]
            output_flat[valid] = source_flat[valid]
            output[:, :, c] = output_flat.reshape(h_out, w_out)

        return output

    def image_to_world(self, image_point, z=0):
        """
        图像坐标 -> 世界坐标 (已知z平面)
        """
        if self.H is None:
            raise ValueError("未计算单应性矩阵")

        # 归一化图像坐标
        img_pt = np.array([image_point[0], image_point[1], 1.0])
        world_pt = self.H @ img_pt
        world_pt = world_pt[:2] / world_pt[2]

        return world_pt

    def world_to_image(self, world_point):
        """
        世界坐标 -> 图像坐标
        """
        if self.H is None:
            raise ValueError("未计算单应性矩阵")

        world_pt = np.array([world_point[0], world_point[1], 1.0])
        img_pt = self.H @ world_pt
        img_pt = img_pt[:2] / img_pt[2]

        return img_pt


# ==============================================================================
# 5. 电赛应用示例
# ==============================================================================
class CompetitionApplications:
    """电赛典型应用场景"""

    @staticmethod
    def line_following_undistort(camera, image_shape):
        """
        巡线机器人: 消除透视畸变
        将摄像头拍摄的倾斜地面图像变换为俯视图
        """
        h_img, w_img = image_shape[:2]

        # 图像中的4个路面角点 (近大远小)
        src_points = np.array([
            [w_img * 0.2, h_img * 0.9],   # 左下
            [w_img * 0.8, h_img * 0.9],   # 右下
            [w_img * 0.6, h_img * 0.3],   # 右上 (远)
            [w_img * 0.4, h_img * 0.3],   # 左上 (远)
        ])

        # 对应的实际地面坐标 (矩形区域)
        dst_points = np.array([
            [0, 400],    # 左下
            [320, 400],  # 右下
            [320, 0],    # 右上
            [0, 0],      # 左上
        ])

        bev = BirdseyeView(camera)
        H = bev.compute_homography_from_points(src_points, dst_points)

        return bev, H

    @staticmethod
    def position_localization(camera, bev, image_target_point):
        """
        定位系统: 将图像中的目标位置转换为实际物理坐标
        """
        world_pos = bev.image_to_world(image_target_point)
        return world_pos

    @staticmethod
    def size_measurement(camera, bev, image_point1, image_point2):
        """
        测量系统: 从图像中测量物体的实际尺寸
        """
        world_p1 = bev.image_to_world(image_point1)
        world_p2 = bev.image_to_world(image_point2)

        real_distance = np.linalg.norm(world_p2 - world_p1)
        return real_distance, world_p1, world_p2


# ==============================================================================
# 6. 主仿真与可视化
# ==============================================================================
def run_perspective_transform_demo():
    """运行透视变换完整演示"""
    print("=" * 70)
    print("摄像头透视变换深度实现 - 电赛视觉核心")
    print("=" * 70)

    # === 1. 相机模型演示 ===
    print("\n[1/6] 相机模型演示...")
    camera = PinholeCamera(fx=800, fy=800, cx=320, cy=240)

    # 生成3D测试点 (地面上的矩形)
    world_points = np.array([
        [0.5, 0.5, 2.0],
        [1.0, 0.5, 2.0],
        [1.0, 1.0, 2.0],
        [0.5, 1.0, 2.0],
        [0.75, 0.75, 1.5],
        [0.75, 0.75, 3.0],
    ])

    image_points = camera.project(world_points)
    print(f"  世界坐标 -> 图像坐标投影结果:")
    for i in range(len(world_points)):
        print(f"    3D({world_points[i, 0]:.2f}, {world_points[i, 1]:.2f}, "
              f"{world_points[i, 2]:.2f}) -> "
              f"2D({image_points[i, 0]:.1f}, {image_points[i, 1]:.1f})")

    # === 2. 畸变演示 ===
    print("\n[2/6] 畸变模型演示...")
    camera_distorted = PinholeCamera()
    camera_distorted.set_distortion(k1=-0.2, k2=0.05, p1=0.01, p2=-0.01)

    # 测试网格点
    grid_x, grid_y = np.meshgrid(np.linspace(-0.5, 0.5, 10), np.linspace(-0.5, 0.5, 10))
    grid_points = np.column_stack([grid_x.ravel(), grid_y.ravel(), np.ones(100) * 2.0])

    img_undistorted = camera.project(grid_points)
    img_distorted = camera_distorted.project(grid_points)

    print(f"  畸变引入的最大偏移: "
          f"{np.max(np.abs(img_distorted - img_undistorted)):.2f} 像素")

    # === 3. 单应性矩阵求解 ===
    print("\n[3/6] 单应性矩阵求解...")

    # 源点 (图像坐标) 和目标点 (俯视图坐标)
    src_pts = np.array([
        [100, 400],
        [540, 400],
        [450, 100],
        [190, 100],
    ], dtype=float)

    dst_pts = np.array([
        [0, 400],
        [320, 400],
        [320, 0],
        [0, 0],
    ], dtype=float)

    # DLT求解
    H_dlt = HomographySolver.dlt(src_pts, dst_pts)
    mean_err, errors = HomographySolver.compute_error(src_pts, dst_pts, H_dlt)
    print(f"  DLT求解 H 矩阵:")
    print(f"    {H_dlt[0, 0]:.6f}  {H_dlt[0, 1]:.6f}  {H_dlt[0, 2]:.6f}")
    print(f"    {H_dlt[1, 0]:.6f}  {H_dlt[1, 1]:.6f}  {H_dlt[1, 2]:.6f}")
    print(f"    {H_dlt[2, 0]:.6f}  {H_dlt[2, 1]:.6f}  {H_dlt[2, 2]:.6f}")
    print(f"  DLT重投影误差: {mean_err:.4f} 像素")

    # RANSAC (添加噪声)
    noise = np.random.randn(len(src_pts), 2) * 5.0
    src_noisy = src_pts + noise
    H_ransac, n_inliers, ransac_err = HomographySolver.ransac(
        src_noisy, dst_pts, threshold=5.0
    )
    print(f"  RANSAC内点数: {n_inliers}/{len(src_pts)}")
    print(f"  RANSAC重投影误差: {ransac_err:.4f} 像素")

    # === 4. 鸟瞰图变换 ===
    print("\n[4/6] 鸟瞰图变换...")
    bev = BirdseyeView(camera)
    bev.compute_homography_from_points(src_pts, dst_pts)

    # 测试点变换
    test_image_pts = np.array([[320, 250], [200, 350], [450, 200]])
    for pt in test_image_pts:
        world_pt = bev.image_to_world(pt)
        print(f"  图像({pt[0]}, {pt[1]}) -> 俯视图({world_pt[0]:.1f}, {world_pt[1]:.1f})")

    # === 5. 电赛应用演示 ===
    print("\n[5/6] 电赛应用演示...")

    # 巡线应用
    bev_line, H_line = CompetitionApplications.line_following_undistort(
        camera, (480, 640, 3)
    )
    print(f"  巡线畸变校正 H矩阵计算完成")

    # 定位应用
    target_img_pt = np.array([320, 300])
    world_pos = CompetitionApplications.position_localization(camera, bev, target_img_pt)
    print(f"  定位: 图像({target_img_pt[0]}, {target_img_pt[1]}) -> "
          f"物理坐标({world_pos[0]:.2f}m, {world_pos[1]:.2f}m)")

    # 测量应用
    p1_img = np.array([150, 350])
    p2_img = np.array([450, 350])
    real_dist, wp1, wp2 = CompetitionApplications.size_measurement(camera, bev, p1_img, p2_img)
    print(f"  测量: 图像两点距离 = {real_dist:.4f}m ({real_dist*1000:.1f}mm)")

    # === 6. 可视化 ===
    print("\n[6/6] 生成可视化图表...")

    fig, axes = plt.subplots(2, 3, figsize=(18, 12))
    fig.suptitle('摄像头透视变换深度实现 - 电赛视觉核心', fontsize=14, fontweight='bold')

    # 6.1 相机投影
    ax = axes[0, 0]
    ax.scatter(image_points[:, 0], image_points[:, 1], c='blue', s=50, zorder=5)
    for i, pt in enumerate(image_points):
        ax.annotate(f'P{i+1}', pt, fontsize=8)
    ax.set_xlim(0, 640)
    ax.set_ylim(480, 0)
    ax.set_xlabel('u [像素]')
    ax.set_ylabel('v [像素]')
    ax.set_title('针孔相机投影')
    ax.grid(True, alpha=0.3)

    # 6.2 畸变效果
    ax = axes[0, 1]
    ax.scatter(img_undistorted[:, 0], img_undistorted[:, 1],
               c='green', s=20, alpha=0.6, label='无畸变')
    ax.scatter(img_distorted[:, 0], img_distorted[:, 1],
               c='red', s=20, alpha=0.6, label='有畸变')
    ax.set_xlim(0, 640)
    ax.set_ylim(480, 0)
    ax.set_title('径向+切向畸变效果')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 6.3 单应性变换可视化
    ax = axes[0, 2]
    # 绘制源四边形
    src_quad = np.vstack([src_pts, src_pts[0]])
    ax.plot(src_quad[:, 0], src_quad[:, 1], 'b-o', linewidth=2, label='图像坐标')
    # 绘制目标四边形
    dst_quad = np.vstack([dst_pts, dst_pts[0]])
    ax.plot(dst_quad[:, 0], dst_quad[:, 1], 'r-s', linewidth=2, label='俯视图坐标')
    # 绘制对应关系
    for i in range(4):
        ax.annotate('', xy=dst_pts[i], xytext=src_pts[i],
                    arrowprops=dict(arrowstyle='->', color='gray', alpha=0.5))
    ax.set_xlim(-50, 600)
    ax.set_ylim(450, -50)
    ax.set_title('单应性对应点')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 6.4 4点对应法原理图
    ax = axes[1, 0]
    # 绘制DLT矩阵结构
    ax.text(0.5, 0.5,
            'DLT: A*h = 0\n'
            'A = [ -x -y -1  0  0  0  ux uy u ]\n'
            '    [  0  0  0 -x -y -1  vx vy v ]\n'
            'h = argmin ||A*h|| s.t. ||h||=1',
            fontsize=10, ha='center', va='center',
            bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.5))
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.set_title('DLT求解原理')
    ax.axis('off')

    # 6.5 RANSAC示意
    ax = axes[1, 1]
    # 模拟内点和外点
    np.random.seed(42)
    n_inliers = 20
    n_outliers = 5
    inlier_src = src_pts[0] + np.random.randn(n_inliers, 2) * 3
    inlier_dst = dst_pts[0] + np.random.randn(n_inliers, 2) * 3
    outlier_src = np.random.uniform(0, 640, (n_outliers, 2))
    outlier_dst = np.random.uniform(0, 400, (n_outliers, 2))

    ax.scatter(inlier_src[:, 0], inlier_src[:, 1], c='green', s=30, label='内点')
    ax.scatter(outlier_src[:, 0], outlier_src[:, 1], c='red', s=30, label='外点')
    ax.set_title(f'RANSAC: {n_inliers}内点 + {n_outliers}外点')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 6.6 应用场景
    ax = axes[1, 2]
    # 巡线场景示意
    # 绘制地面
    ground = plt.Polygon([[50, 350], [590, 350], [450, 50], [190, 50]],
                          facecolor='lightgray', edgecolor='gray')
    ax.add_patch(ground)

    # 绘制线条
    line_x = np.linspace(200, 440, 100)
    line_y = 350 - (line_x - 200) * (300/240)
    ax.plot(line_x, line_y, 'k-', linewidth=3, label='黑线')

    # 标注透视效果
    ax.annotate('近大', xy=(320, 350), fontsize=12, fontweight='bold', color='blue')
    ax.annotate('远小', xy=(320, 80), fontsize=12, fontweight='bold', color='red')
    ax.set_xlim(0, 640)
    ax.set_ylim(400, 0)
    ax.set_title('巡线机器人透视效果')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'perspective_transform.png'),
                dpi=150, bbox_inches='tight')
    print("  图表已保存: perspective_transform.png")

    # === 额外: 畸变校正对比 ===
    fig2, axes2 = plt.subplots(1, 3, figsize=(15, 5))
    fig2.suptitle('畸变模型与校正', fontsize=14, fontweight='bold')

    # 网格图: 无畸变
    ax = axes2[0]
    grid_size = 10
    for i in range(grid_size):
        x_line = np.linspace(0, 640, grid_size)
        y_line = np.full(grid_size, i * 480 / (grid_size - 1))
        ax.plot(x_line, y_line, 'b-', linewidth=0.5)
        y_line = np.linspace(0, 480, grid_size)
        x_line = np.full(grid_size, i * 640 / (grid_size - 1))
        ax.plot(x_line, y_line, 'b-', linewidth=0.5)
    ax.set_xlim(0, 640)
    ax.set_ylim(480, 0)
    ax.set_title('无畸变网格')
    ax.set_aspect('equal')

    # 网格图: 有畸变
    ax = axes2[1]
    camera_test = PinholeCamera()
    camera_test.set_distortion(k1=-0.15, k2=0.03)
    grid_points_3d = []
    for i in range(grid_size):
        for j in range(grid_size):
            x = j * 1.0 / (grid_size - 1) - 0.5
            y = i * 1.0 / (grid_size - 1) - 0.5
            grid_points_3d.append([x, y, 2.0])
    grid_points_3d = np.array(grid_points_3d)
    grid_img = camera_test.project(grid_points_3d)

    for i in range(grid_size):
        start = i * grid_size
        end = start + grid_size
        ax.plot(grid_img[start:end, 0], grid_img[start:end, 1], 'r-', linewidth=0.5)
        ax.plot(grid_img[i::grid_size, 0], grid_img[i::grid_size, 1], 'r-', linewidth=0.5)
    ax.set_xlim(0, 640)
    ax.set_ylim(480, 0)
    ax.set_title('有畸变网格 (k1=-0.15)')
    ax.set_aspect('equal')

    # 去畸变效果
    ax = axes2[2]
    camera_undistort = PinholeCamera()
    undistorted_pts = camera_undistort.undistort(grid_img)
    # 转回像素坐标
    undistorted_px = undistorted_pts * np.array([800, 800]) + np.array([320, 240])
    for i in range(grid_size):
        start = i * grid_size
        end = start + grid_size
        ax.plot(undistorted_px[start:end, 0], undistorted_px[start:end, 1],
                'g-', linewidth=0.5)
        ax.plot(undistorted_px[i::grid_size, 0], undistorted_px[i::grid_size, 1],
                'g-', linewidth=0.5)
    ax.set_xlim(0, 640)
    ax.set_ylim(480, 0)
    ax.set_title('去畸变后网格')
    ax.set_aspect('equal')

    plt.tight_layout()
    plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'distortion_correction.png'),
                dpi=150, bbox_inches='tight')
    print("  畸变校正图已保存: distortion_correction.png")

    print("\n" + "=" * 70)
    print("透视变换深度实现完成!")
    print("=" * 70)
    print("\n关键公式总结:")
    print("  1. 投影方程: [u,v,1]^T = K * [R|t] * [X,Y,Z,1]^T")
    print("  2. 单应性:   [u,v,1]^T = H * [X,Y,1]^T")
    print("  3. DLT:      Ah = 0 (SVD求最小奇异值对应的特征向量)")
    print("  4. 畸变:     xd = x(1+k1*r^2+k2*r^4+k3*r^6) + 2p1*x*y + p2*(r^2+2x^2)")
    print("  5. RANSAC:   随机采样4点 -> DLT -> 内点检验 -> 重新拟合")

    return camera, bev, H_dlt


if __name__ == '__main__':
    camera, bev, H = run_perspective_transform_demo()
    plt.close('all')
