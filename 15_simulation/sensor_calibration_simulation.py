#!/usr/bin/env python3
"""
传感器标定仿真
- 六面标定法 (加速度计)
- 椭球拟合 (磁力计)
- 最小二乘法通用标定
- 陀螺仪零偏标定
- 九轴传感器联合标定
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from dataclasses import dataclass
from typing import Tuple, List

# ============================================================
# 1. 传感器模型
# ============================================================

@dataclass
class SensorModel:
    """含误差的传感器模型: y = T * K * x + b + noise"""
    scale: np.ndarray       # 3x3 对角: 比例因子
    misalignment: np.ndarray # 3x3: 安装误差矩阵
    bias: np.ndarray         # 3x1: 零偏
    noise_std: float = 0.01

    def measure(self, true_value: np.ndarray) -> np.ndarray:
        """生成含误差的测量值"""
        x = true_value.reshape(-1, 1) if true_value.ndim == 1 else true_value
        y = self.misalignment @ self.scale @ x + self.bias.reshape(-1, 1)
        noise = np.random.randn(3, 1) * self.noise_std
        return (y + noise).flatten()

def create_random_sensor_model(noise_std=0.02) -> SensorModel:
    """创建随机传感器误差模型"""
    scale = np.diag([1.0 + np.random.uniform(-0.1, 0.1) for _ in range(3)])
    misalignment = np.eye(3)
    for i in range(3):
        for j in range(3):
            if i != j:
                misalignment[i, j] = np.random.uniform(-0.05, 0.05)
    bias = np.random.randn(3) * 0.2
    return SensorModel(scale, misalignment, bias, noise_std)


# ============================================================
# 2. 六面标定法 (加速度计)
# ============================================================

def six_face_calibration(measurements: List[np.ndarray]) -> Tuple[np.ndarray, np.ndarray]:
    """
    六面标定法
    输入: 6个面的测量值 (每个面静止采集多次平均)
    输出: 标定矩阵 K 和偏置 b

    原理: 理想情况下6个面测量值为 ±g 在三个轴上
    """
    # measurements: list of 6 arrays, each (3,)
    # 理想值: [+g,0,0], [-g,0,0], [0,+g,0], [0,-g,0], [0,0,+g], [0,0,-g]
    g = 9.80665

    ideal = np.array([
        [g, 0, 0], [-g, 0, 0],
        [0, g, 0], [0, -g, 0],
        [0, 0, g], [0, 0, -g]
    ])

    measured = np.array(measurements)  # (6, 3)

    # 最小二乘拟合: measured = K * ideal + b
    # 构建增广矩阵 [ideal, 1] -> measured
    A = np.column_stack([ideal, np.ones(6)])  # (6, 4)
    # 对每个轴分别拟合
    calib_matrix = np.zeros((3, 4))
    for i in range(3):
        result = np.linalg.lstsq(A, measured[:, i], rcond=None)
        calib_matrix[i] = result[0]

    K = calib_matrix[:, :3]  # 标定矩阵
    b = calib_matrix[:, 3]   # 偏置
    return K, b


# ============================================================
# 3. 椭球拟合 (磁力计)
# ============================================================

def ellipsoid_fit(points: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    椭球拟合
    输入: N个3D测量点
    输出: 中心, 半轴长度, 旋转矩阵

    椭球方程: (x-c)^T R^T diag(1/a^2, 1/b^2, 1/c^2) R (x-c) = 1
    """
    N = points.shape[0]
    x, y, z = points[:, 0], points[:, 1], points[:, 2]

    # 设计矩阵: ax² + by² + cz² + 2dxy + 2exz + 2fyz + 2gx + 2hy + 2kz = 1
    D = np.column_stack([
        x**2, y**2, z**2,
        2*x*y, 2*x*z, 2*y*z,
        2*x, 2*y, 2*z,
        np.ones(N)
    ])

    # 最小二乘: D @ v = 1
    v = np.linalg.lstsq(D, np.ones(N), rcond=None)[0]

    # 提取椭球参数
    A_mat = np.array([
        [v[0], v[3], v[4]],
        [v[3], v[1], v[5]],
        [v[4], v[5], v[2]]
    ])
    b_vec = np.array([v[6], v[7], v[8]])

    # 中心
    center = -np.linalg.solve(A_mat, b_vec)

    # 归一化
    T = np.eye(4)
    T[:3, :3] = A_mat
    T[:3, 3] = b_vec
    T[3, :3] = b_vec
    T[3, 3] = v[9] if len(v) > 9 else 1.0

    # 计算半轴
    A_centered = A_mat / (center @ A_mat @ center - v[-1] if len(v) > 9 else 1.0)
    eigenvalues, eigenvectors = np.linalg.eigh(A_mat)
    # 半轴长度
    radii = 1.0 / np.sqrt(np.abs(eigenvalues))

    return center, radii, eigenvectors


def magnetometer_calibration(raw_data: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    磁力计标定
    输入: 原始磁力计数据 (N, 3)
    输出: 标定矩阵 M 和偏置 b
    标定后: calibrated = M @ (raw - b)
    """
    center, radii, rotation = ellipsoid_fit(raw_data)

    # 缩放: 使椭球变成球
    scale = np.diag(1.0 / radii)
    # 标定矩阵: 将椭球映射到单位球
    M = scale @ rotation.T
    b = center

    return M, b


# ============================================================
# 4. 陀螺仪零偏标定
# ============================================================

def gyro_bias_calibration(static_data: np.ndarray) -> np.ndarray:
    """
    陀螺仪静态零偏标定
    输入: 静止状态下的陀螺仪数据 (N, 3)
    输出: 零偏向量
    """
    return np.mean(static_data, axis=0)


# ============================================================
# 5. 最小二乘通用标定
# ============================================================

def least_squares_calibration(true_values: np.ndarray,
                               measured_values: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    通用最小二乘标定
    measured = A @ true + b
    返回 (A, b)
    """
    N = true_values.shape[0]
    A_aug = np.column_stack([true_values, np.ones(N)])
    result = np.linalg.lstsq(A_aug, measured_values, rcond=None)
    A = result[0][:3].T  # (3, 3)
    b = result[0][3]     # (3,)
    return A, b


# ============================================================
# 6. 仿真与验证
# ============================================================

def simulate_accelerometer_calibration():
    """加速度计六面标定仿真"""
    print("\n--- 加速度计六面标定 ---")

    model = create_random_sensor_model(noise_std=0.02)
    g = 9.80665

    # 模拟六面测量 (每个面采集100次取平均)
    ideal_faces = np.array([
        [g,0,0], [-g,0,0], [0,g,0], [0,-g,0], [0,0,g], [0,0,-g]
    ])

    measurements = []
    for face in ideal_faces:
        samples = np.array([model.measure(face) for _ in range(100)])
        measurements.append(np.mean(samples, axis=0))

    # 标定
    K, b = six_face_calibration(measurements)

    # 真实标定参数
    true_K = model.misalignment @ model.scale
    true_b = model.bias

    print(f"  真实比例因子: {np.diag(model.scale)}")
    print(f"  估计比例因子: {np.diag(K)}")
    print(f"  真实偏置: {true_b}")
    print(f"  估计偏置: {b}")
    print(f"  偏置误差: {np.linalg.norm(b - true_b):.6f}")

    # 验证
    test_points = np.random.randn(50, 3) * g
    errors_before = []
    errors_after = []
    for tp in test_points:
        raw = model.measure(tp)
        calib = np.linalg.solve(K, raw - b)
        errors_before.append(np.linalg.norm(raw - tp))
        errors_after.append(np.linalg.norm(calib - tp))

    print(f"  标定前 RMS 误差: {np.sqrt(np.mean(np.array(errors_before)**2)):.4f} m/s²")
    print(f"  标定后 RMS 误差: {np.sqrt(np.mean(np.array(errors_after)**2)):.4f} m/s²")

    return model, K, b

def simulate_magnetometer_calibration():
    """磁力计椭球拟合标定仿真"""
    print("\n--- 磁力计椭球拟合 ---")

    model = create_random_sensor_model(noise_std=0.05)

    # 在球面上均匀采样 (模拟转动磁力计)
    n_points = 500
    phi = np.random.uniform(0, 2*np.pi, n_points)
    theta = np.arccos(np.random.uniform(-1, 1, n_points))
    r = 50.0  # 磁场强度 (μT)

    ideal = np.column_stack([
        r * np.sin(theta) * np.cos(phi),
        r * np.sin(theta) * np.sin(phi),
        r * np.cos(theta)
    ])

    measured = np.array([model.measure(p) for p in ideal])

    # 椭球拟合
    M, b = magnetometer_calibration(measured)

    # 标定
    calib = np.array([M @ (m - b) for m in measured])

    # 验证: 应该是球形
    radii_est = np.linalg.norm(calib, axis=1)
    print(f"  理想半径: {r:.1f} μT")
    print(f"  标定后半径均值: {np.mean(radii_est):.2f} μT")
    print(f"  标定后半径标准差: {np.std(radii_est):.4f} μT")

    # 绘图
    fig = plt.figure(figsize=(15, 5))

    ax1 = fig.add_subplot(131, projection='3d')
    ax1.scatter(measured[:, 0], measured[:, 1], measured[:, 2], s=1, c='red', alpha=0.5)
    ax1.set_title('标定前 (椭球)')
    ax1.set_xlabel('X'); ax1.set_ylabel('Y'); ax1.set_zlabel('Z')

    ax2 = fig.add_subplot(132, projection='3d')
    ax2.scatter(calib[:, 0], calib[:, 1], calib[:, 2], s=1, c='blue', alpha=0.5)
    ax2.set_title('标定后 (球形)')
    ax2.set_xlabel('X'); ax2.set_ylabel('Y'); ax2.set_zlabel('Z')

    # 拟合椭球
    center, radii, rotation = ellipsoid_fit(measured)
    ax3 = fig.add_subplot(133, projection='3d')
    ax3.scatter(measured[:, 0], measured[:, 1], measured[:, 2], s=1, c='red', alpha=0.3)

    # 绘制拟合椭球
    u = np.linspace(0, 2*np.pi, 30)
    v = np.linspace(0, np.pi, 20)
    x = radii[0] * np.outer(np.cos(u), np.sin(v))
    y = radii[1] * np.outer(np.sin(u), np.sin(v))
    z = radii[2] * np.outer(np.ones_like(u), np.cos(v))
    for i in range(len(u)):
        for j in range(len(v)):
            point = rotation @ np.array([x[i,j], y[i,j], z[i,j]]) + center
            x[i,j], y[i,j], z[i,j] = point
    ax3.plot_surface(x, y, z, alpha=0.2, color='blue')
    ax3.set_title('椭球拟合')
    ax3.set_xlabel('X'); ax3.set_ylabel('Y'); ax3.set_zlabel('Z')

    plt.tight_layout()
    plt.savefig('./nuedc-asset-library/15_simulation/mag_calibration.png', dpi=150)
    plt.show()

    return model, M, b


def simulate_gyro_calibration():
    """陀螺仪零偏标定仿真"""
    print("\n--- 陀螺仪零偏标定 ---")

    true_bias = np.array([0.05, -0.03, 0.02])  # rad/s
    noise_std = 0.001

    # 静态采集
    n_samples = 1000
    data = np.tile(true_bias, (n_samples, 1)) + np.random.randn(n_samples, 3) * noise_std

    estimated_bias = gyro_bias_calibration(data)

    print(f"  真实零偏: {true_bias}")
    print(f"  估计零偏: {estimated_bias}")
    print(f"  估计误差: {np.linalg.norm(estimated_bias - true_bias):.6f} rad/s")

    return estimated_bias


def simulate_least_squares_calibration():
    """最小二乘通用标定仿真"""
    print("\n--- 最小二乘通用标定 ---")

    # 真实参数
    true_A = np.array([
        [1.02, 0.03, -0.01],
        [0.02, 0.98, 0.04],
        [-0.01, 0.02, 1.05]
    ])
    true_b = np.array([0.1, -0.2, 0.15])

    # 生成校准数据
    n_points = 100
    true_values = np.random.randn(n_points, 3) * 10
    measured = (true_A @ true_values.T).T + true_b + np.random.randn(n_points, 3) * 0.01

    # 标定
    A_est, b_est = least_squares_calibration(true_values, measured)

    print(f"  真实矩阵:\n{true_A}")
    print(f"  估计矩阵:\n{A_est}")
    print(f"  矩阵误差: {np.linalg.norm(A_est - true_A):.6f}")
    print(f"  偏置误差: {np.linalg.norm(b_est - true_b):.6f}")


if __name__ == '__main__':
    print("=" * 60)
    print("  传感器标定仿真")
    print("=" * 60)

    simulate_accelerometer_calibration()
    simulate_magnetometer_calibration()
    simulate_gyro_calibration()
    simulate_least_squares_calibration()

    print("\n✅ 所有传感器标定仿真完成")
