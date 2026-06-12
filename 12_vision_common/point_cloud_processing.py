#!/usr/bin/env python3
"""
点云处理模块 - 深度图转点云 + 滤波 + 分割 + 配准
适用于电赛三维重建、障碍物检测、场景理解等任务
依赖: numpy, opencv-python
"""

import cv2
import numpy as np
from typing import Tuple, Optional, List


class PointCloudProcessor:
    """点云处理器：支持深度图转点云、滤波、分割、配准"""

    def __init__(self, camera_matrix: np.ndarray, depth_scale: float = 1000.0):
        """
        初始化点云处理器
        Args:
            camera_matrix: 3x3相机内参矩阵 [[fx,0,cx],[0,fy,cy],[0,0,1]]
            depth_scale: 深度缩放系数（如Kinect深度图单位mm转m用1000）
        """
        self.fx = camera_matrix[0, 0]
        self.fy = camera_matrix[1, 1]
        self.cx = camera_matrix[0, 2]
        self.cy = camera_matrix[1, 2]
        self.K = camera_matrix
        self.depth_scale = depth_scale

    def depth_to_pointcloud(self, depth_img: np.ndarray,
                            color_img: Optional[np.ndarray] = None,
                            max_depth: float = 5.0) -> dict:
        """
        深度图转点云
        Args:
            depth_img: 深度图 (H,W), 单位由depth_scale决定
            color_img: 可选彩色图 (H,W,3), BGR格式
            max_depth: 最大有效深度（米）
        Returns:
            {'points': (N,3), 'colors': (N,3) or None, 'mask': (N,)}
        """
        h, w = depth_img.shape[:2]
        depth = depth_img.astype(np.float64) / self.depth_scale

        # 生成像素坐标网格
        u, v = np.meshgrid(np.arange(w), np.arange(h))

        # 反投影到3D
        z = depth
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy

        # 有效深度掩码
        mask = (z > 0) & (z < max_depth) & np.isfinite(z)

        points = np.stack([x[mask], y[mask], z[mask]], axis=-1)

        colors = None
        if color_img is not None:
            colors = color_img[mask][:, ::-1]  # BGR -> RGB

        return {'points': points, 'colors': colors, 'mask': mask}

    def voxel_downsample(self, points: np.ndarray, voxel_size: float = 0.05,
                         colors: Optional[np.ndarray] = None) -> dict:
        """
        体素下采样 - 降低点云密度
        Args:
            points: (N,3) 点云
            voxel_size: 体素边长（米）
            colors: 可选颜色 (N,3)
        Returns:
            下采样后的点云和颜色
        """
        # 计算每个点所属体素索引
        voxel_indices = np.floor(points / voxel_size).astype(np.int32)

        # 唯一体素编码（哈希）
        min_vals = voxel_indices.min(axis=0)
        voxel_indices -= min_vals
        max_vals = voxel_indices.max(axis=0)
        voxel_hash = (voxel_indices[:, 0] * (max_vals[1] + 1) * (max_vals[2] + 1) +
                      voxel_indices[:, 1] * (max_vals[2] + 1) +
                      voxel_indices[:, 2])

        # 对每个体素取质心
        unique_hashes, inverse = np.unique(voxel_hash, return_inverse=True)
        counts = np.bincount(inverse)

        # 累加坐标求均值
        sum_points = np.zeros((len(unique_hashes), 3), dtype=np.float64)
        np.add.at(sum_points, inverse, points)
        down_points = sum_points / counts[:, None]

        down_colors = None
        if colors is not None:
            sum_colors = np.zeros((len(unique_hashes), 3), dtype=np.float64)
            np.add.at(sum_colors, inverse, colors.astype(np.float64))
            down_colors = (sum_colors / counts[:, None]).astype(np.uint8)

        return {'points': down_points, 'colors': down_colors}

    def statistical_filter(self, points: np.ndarray, k: int = 20,
                           std_ratio: float = 2.0) -> Tuple[np.ndarray, np.ndarray]:
        """
        统计滤波 - 去除离群点
        Args:
            points: (N,3) 点云
            k: 邻域点数
            std_ratio: 标准差倍数阈值
        Returns:
            (滤波后点云, 保留索引)
        """
        n = len(points)
        if n <= k:
            return points, np.arange(n)

        # 计算每个点到k近邻的平均距离
        # 使用分块计算避免内存溢出
        mean_dists = np.zeros(n)
        chunk_size = max(1, min(1000, n))

        for i in range(0, n, chunk_size):
            end = min(i + chunk_size, n)
            # 计算当前块到所有点的距离
            diff = points[i:end, None, :] - points[None, :, :]
            dists = np.sqrt(np.sum(diff ** 2, axis=-1))
            dists.sort(axis=1)
            mean_dists[i:end] = dists[:, 1:k + 1].mean(axis=1)

        # 去除超过阈值的点
        global_mean = mean_dists.mean()
        global_std = mean_dists.std()
        threshold = global_mean + std_ratio * global_std
        keep_mask = mean_dists < threshold
        keep_idx = np.where(keep_mask)[0]

        return points[keep_mask], keep_idx

    def ransac_plane_segmentation(self, points: np.ndarray,
                                  distance_threshold: float = 0.02,
                                  max_iterations: int = 1000) -> Tuple[List, List]:
        """
        RANSAC平面分割 - 从点云中提取多个平面
        Args:
            points: (N,3) 点云
            distance_threshold: 点到平面距离阈值（米）
            max_iterations: 每个平面最大迭代次数
        Returns:
            (平面列表 [{'normal':(3,), 'point':(3,), 'indices':array}], 剩余点索引)
        """
        remaining = np.arange(len(points))
        planes = []

        for _ in range(5):  # 最多提取5个平面
            if len(remaining) < 3:
                break

            pts = points[remaining]
            best_inliers = []
            best_normal = None
            best_point = None

            for _ in range(max_iterations):
                # 随机选3个点
                idx = np.random.choice(len(pts), 3, replace=False)
                p0, p1, p2 = pts[idx]

                # 计算平面法向量
                v1 = p1 - p0
                v2 = p2 - p0
                normal = np.cross(v1, v2)
                norm = np.linalg.norm(normal)
                if norm < 1e-10:
                    continue
                normal /= norm

                # 计算内点
                dists = np.abs(np.dot(pts - p0, normal))
                inlier_mask = dists < distance_threshold
                n_inliers = inlier_mask.sum()

                if n_inliers > len(best_inliers):
                    best_inliers = np.where(inlier_mask)[0]
                    best_normal = normal
                    best_point = p0

            if len(best_inliers) < 10:
                break

            planes.append({
                'normal': best_normal,
                'point': best_point,
                'indices': remaining[best_inliers]
            })

            # 移除已分割的点
            remaining = np.delete(remaining, best_inliers)

        return planes, remaining

    def icp_registration(self, source: np.ndarray, target: np.ndarray,
                         max_iterations: int = 50,
                         tolerance: float = 1e-6) -> Tuple[np.ndarray, float]:
        """
        ICP（迭代最近点）配准
        Args:
            source: (N,3) 源点云
            target: (M,3) 目标点云
            max_iterations: 最大迭代次数
            tolerance: 收敛阈值
        Returns:
            (4x4变换矩阵, 最终误差)
        """
        src = source.copy().astype(np.float64)
        tgt = target.copy().astype(np.float64)
        T_total = np.eye(4)

        # 简易KD树用numpy实现（小规模适用）
        for iteration in range(max_iterations):
            # 最近点搜索
            distances = np.linalg.norm(
                src[:, None, :] - tgt[None, :, :], axis=2
            )
            indices = distances.argmin(axis=1)

            matched_tgt = tgt[indices]

            # 计算质心
            src_centroid = src.mean(axis=0)
            tgt_centroid = matched_tgt.mean(axis=0)

            # 中心化
            src_centered = src - src_centroid
            tgt_centered = matched_tgt - tgt_centroid

            # SVD求旋转矩阵
            H = src_centered.T @ tgt_centered
            U, _, Vt = np.linalg.svd(H)
            R = Vt.T @ U.T

            # 修正反射情况
            if np.linalg.det(R) < 0:
                Vt[-1, :] *= -1
                R = Vt.T @ U.T

            t = tgt_centroid - R @ src_centroid

            # 更新源点云
            src = (R @ src.T).T + t

            # 累积变换
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = t
            T_total = T @ T_total

            # 检查收敛
            error = np.mean(np.linalg.norm(src - matched_tgt, axis=1))
            if iteration > 0 and abs(prev_error - error) < tolerance:
                break
            prev_error = error

        return T_total, error


def create_test_depth_map(width=640, height=480):
    """生成测试深度图：前方有一个倾斜平面"""
    depth = np.zeros((height, width), dtype=np.uint16)
    # 倾斜平面，距离1~3米
    for v in range(height):
        for u in range(width):
            depth[v, u] = int((1000 + v * 3 + u * 1) % 2000 + 500)
    # 添加噪声
    noise = np.random.randint(-20, 20, depth.shape, dtype=np.int16)
    depth = np.clip(depth.astype(np.int16) + noise, 1, 65535).astype(np.uint16)
    return depth


if __name__ == '__main__':
    # ==================== 使用示例 ====================
    print("=== 点云处理模块使用示例 ===\n")

    # 1. 设置相机内参（以640x480分辨率为例）
    K = np.array([[525.0, 0, 319.5],
                  [0, 525.0, 239.5],
                  [0, 0, 1.0]])
    processor = PointCloudProcessor(K, depth_scale=1000.0)

    # 2. 生成测试深度图
    depth = create_test_depth_map()
    print(f"深度图尺寸: {depth.shape}, 深度范围: {depth.min()}~{depth.max()}")

    # 3. 深度图转点云
    result = processor.depth_to_pointcloud(depth, max_depth=3.0)
    print(f"生成点云: {result['points'].shape[0]} 个点")

    # 4. 统计滤波去噪
    filtered_pts, keep_idx = processor.statistical_filter(
        result['points'], k=10, std_ratio=2.0
    )
    print(f"统计滤波后: {filtered_pts.shape[0]} 个点 "
          f"(保留{len(keep_idx)/len(result['points'])*100:.1f}%)")

    # 5. 体素下采样
    down = processor.voxel_downsample(filtered_pts, voxel_size=0.05)
    print(f"体素下采样后: {down['points'].shape[0]} 个点")

    # 6. 平面分割
    planes, remaining = processor.ransac_plane_segmentation(
        filtered_pts, distance_threshold=0.03
    )
    print(f"检测到 {len(planes)} 个平面")
    for i, plane in enumerate(planes):
        print(f"  平面{i}: 法向量{plane['normal']}, 内点{len(plane['indices'])}个")
    print(f"  剩余非平面点: {len(remaining)} 个")

    # 7. ICP配准示例
    src = filtered_pts[:200] + np.array([0.05, 0.03, 0.0])  # 模拟轻微偏移
    T, err = processor.icp_registration(src, filtered_pts[:200])
    print(f"\nICP配准变换矩阵:\n{T}")
    print(f"ICP最终误差: {err:.6f}")

    print("\n示例完成！")
