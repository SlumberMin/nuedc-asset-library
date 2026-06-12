#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IMU标定仿真 — 六面标定 + 椭球拟合 + 零偏估计
==============================================
模拟三轴加速度计和陀螺仪的六面标定过程，
通过椭球拟合求解比例因子、非正交误差和零偏。
依赖: numpy, matplotlib
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.gridspec import GridSpec

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

# ═══════════════════════════════════════════════════════════════
# 1. 模拟真实IMU参数 (制造误差)
# ═══════════════════════════════════════════════════════════════

# 加速度计真实参数
# 真实模型: a_meas = M * a_true + b
# M = 比例因子矩阵(含非正交), b = 零偏
np.random.seed(42)

# 真实比例因子 (理想为1)
scale_true = np.array([1.02, 0.98, 1.01])
# 真实非正交角 (弧度, 理想为0)
misalign_true = np.array([0.02, -0.01, 0.015])  # xy, xz, yz 交叉耦合
# 真实零偏 (m/s^2, 理想为0)
bias_true = np.array([0.15, -0.08, 0.12])

# 构造真实变换矩阵 M
M_true = np.array([
    [scale_true[0],              scale_true[0]*np.tan(misalign_true[0]), scale_true[0]*np.tan(misalign_true[1])],
    [0,                          scale_true[1],                          scale_true[1]*np.tan(misalign_true[2])],
    [0,                          0,                                      scale_true[2]]
])

print("=" * 60)
print("IMU标定仿真")
print("=" * 60)
print("\n真实参数:")
print(f"  比例因子: {scale_true}")
print(f"  非正交角(度): {np.degrees(misalign_true)}")
print(f"  零偏(m/s²): {bias_true}")

# ═══════════════════════════════════════════════════════════════
# 2. 六面标定数据生成
# ═══════════════════════════════════════════════════════════════

# 标准重力加速度
g = 9.81

# 六个面的理想测量值 (±X, ±Y, ±Z)
# 每个面放置时, 理想加速度计读数
faces_ideal = np.array([
    [+g, 0, 0],   # +X面朝上
    [-g, 0, 0],   # -X面朝上
    [0, +g, 0],   # +Y面朝上
    [0, -g, 0],   # -Y面朝上
    [0, 0, +g],   # +Z面朝上
    [0, 0, -g],   # -Z面朝上
])

# 模拟多次测量 (每个面取多个采样点, 含噪声)
N_SAMPLES_PER_FACE = 50
NOISE_STD = 0.02  # 测量噪声标准差 (m/s²)

all_ideal = []   # 理想值
all_measured = []  # 测量值

for face_idx, face_ideal in enumerate(faces_ideal):
    for _ in range(N_SAMPLES_PER_FACE):
        # 理想值 + 微小姿态偏差
        theta = np.random.normal(0, 0.02)  # 微小倾角
        phi = np.random.normal(0, 0.02)
        
        # 旋转后的理想值
        Rx = np.array([[1, 0, 0],
                       [0, np.cos(theta), -np.sin(theta)],
                       [0, np.sin(theta), np.cos(theta)]])
        Ry = np.array([[np.cos(phi), 0, np.sin(phi)],
                       [0, 1, 0],
                       [-np.sin(phi), 0, np.cos(phi)]])
        
        a_rotated = Rx @ Ry @ face_ideal
        
        # 加上真实误差: 测量值 = M * 理想值 + 零偏 + 噪声
        noise = np.random.normal(0, NOISE_STD, 3)
        a_measured = M_true @ a_rotated + bias_true + noise
        
        all_ideal.append(a_rotated)
        all_measured.append(a_measured)

all_ideal = np.array(all_ideal)
all_measured = np.array(all_measured)

print(f"\n六面标定数据: {len(all_measured)} 个采样点 (每面 {N_SAMPLES_PER_FACE} 个)")

# ═══════════════════════════════════════════════════════════════
# 3. 椭球拟合 (最小二乘法)
# ═══════════════════════════════════════════════════════════════

def ellipsoid_fit(data):
    """
    椭球拟合 — 求解椭球方程:
    ax² + by² + cz² + 2dxy + 2exz + 2fyz + 2gx + 2hy + 2kz = 1
    
    返回: 椭球参数向量 [a, b, c, d, e, f, g, h, k]
    """
    x, y, z = data[:, 0], data[:, 1], data[:, 2]
    
    # 构建设计矩阵
    D = np.column_stack([
        x**2, y**2, z**2,
        2*x*y, 2*x*z, 2*y*z,
        2*x, 2*y, 2*z
    ])
    
    # 最小二乘求解: D @ params = 1
    ones = np.ones(len(x))
    params, _, _, _ = np.linalg.lstsq(D, ones, rcond=None)
    
    return params

def extract_calibration(params):
    """
    从椭球参数提取标定矩阵:
    M_cal, b_cal 使得: a_true ≈ M_cal @ (a_meas - b_cal)
    """
    a, b, c, d, e, f, g, h, k = params
    
    # 二次型矩阵
    A = np.array([
        [a, d, e],
        [d, b, f],
        [e, f, c]
    ])
    
    # 线性项
    v = np.array([g, h, k])
    
    # 椭球中心 (零偏估计)
    center = -np.linalg.solve(A, v)
    
    # 将椭球方程写成 (x-c)^T A (x-c) = r^2
    r2 = 1.0 + center @ A @ center
    
    # 特征分解求半轴
    eigenvalues, eigenvectors = np.linalg.eigh(A / r2)
    
    # 标定矩阵: 将椭球映射到单位球
    # 半轴长度 = 1/sqrt(eigenvalue)
    half_axes = 1.0 / np.sqrt(np.abs(eigenvalues))
    M_cal = eigenvectors @ np.diag(half_axes) @ eigenvectors.T
    
    return center, M_cal

print("\n椭球拟合中...")
params = ellipsoid_fit(all_measured)
bias_est, M_cal = extract_calibration(params)

print(f"\n零偏估计 (m/s²):")
print(f"  真实: {bias_true}")
print(f"  估计: {bias_est}")
print(f"  误差: {np.abs(bias_true - bias_est)}")

# ═══════════════════════════════════════════════════════════════
# 4. 标定前后对比
# ═══════════════════════════════════════════════════════════════

# 标定后数据
all_calibrated = np.zeros_like(all_measured)
for i in range(len(all_measured)):
    all_calibrated[i] = M_cal @ (all_measured[i] - bias_est)

# 计算标定前后的模长误差
norms_before = np.linalg.norm(all_measured, axis=1)
norms_after = np.linalg.norm(all_calibrated, axis=1)
norms_ideal = np.linalg.norm(all_ideal, axis=1)

err_before = np.abs(norms_before - g)
err_after = np.abs(norms_after - g)

print(f"\n标定效果:")
print(f"  标定前模长误差: 均值={np.mean(err_before):.4f}, 最大={np.max(err_before):.4f} m/s²")
print(f"  标定后模长误差: 均值={np.mean(err_after):.6f}, 最大={np.max(err_after):.6f} m/s²")
print(f"  改善倍数: {np.mean(err_before)/max(np.mean(err_after),1e-10):.1f}x")

# ═══════════════════════════════════════════════════════════════
# 5. 陀螺仪零偏估计仿真
# ═══════════════════════════════════════════════════════════════

# 陀螺仪真实零偏 (°/s)
gyro_bias_true = np.array([0.5, -0.3, 0.8])
gyro_noise_std = 0.01  # °/s

# 静态采集 (陀螺仪静止时)
GYRO_SAMPLES = 1000
gyro_static = np.tile(gyro_bias_true, (GYRO_SAMPLES, 1)) + \
              np.random.normal(0, gyro_noise_std, (GYRO_SAMPLES, 3))

gyro_bias_est = np.mean(gyro_static, axis=0)

print(f"\n陀螺仪零偏估计:")
print(f"  真实 (°/s): {gyro_bias_true}")
print(f"  估计 (°/s): {gyro_bias_est}")
print(f"  误差 (°/s): {np.abs(gyro_bias_true - gyro_bias_est)}")

# ═══════════════════════════════════════════════════════════════
# 6. 绘图
# ═══════════════════════════════════════════════════════════════

fig = plt.figure(figsize=(20, 16))
fig.suptitle('IMU标定仿真 — 六面标定 + 椭球拟合 + 零偏估计',
             fontsize=16, fontweight='bold')

gs = GridSpec(3, 3, hspace=0.4, wspace=0.35)

# --- 子图1: 标定前3D散点 ---
ax1 = fig.add_subplot(gs[0, 0], projection='3d')
face_colors = ['#e74c3c', '#3498db', '#2ecc71', '#f39c12', '#9b59b6', '#1abc9c']
for i in range(6):
    start = i * N_SAMPLES_PER_FACE
    end = start + N_SAMPLES_PER_FACE
    ax1.scatter(all_measured[start:end, 0],
                all_measured[start:end, 1],
                all_measured[start:end, 2],
                c=face_colors[i], s=10, alpha=0.6)
# 画单位球参考
u = np.linspace(0, 2*np.pi, 30)
v = np.linspace(0, np.pi, 20)
xs = g * np.outer(np.cos(u), np.sin(v))
ys = g * np.outer(np.sin(u), np.sin(v))
zs = g * np.outer(np.ones_like(u), np.cos(v))
ax1.plot_wireframe(xs, ys, zs, alpha=0.1, color='gray')
ax1.set_xlabel('X (m/s²)')
ax1.set_ylabel('Y (m/s²)')
ax1.set_zlabel('Z (m/s²)')
ax1.set_title('标定前测量数据')

# --- 子图2: 标定后3D散点 ---
ax2 = fig.add_subplot(gs[0, 1], projection='3d')
for i in range(6):
    start = i * N_SAMPLES_PER_FACE
    end = start + N_SAMPLES_PER_FACE
    ax2.scatter(all_calibrated[start:end, 0],
                all_calibrated[start:end, 1],
                all_calibrated[start:end, 2],
                c=face_colors[i], s=10, alpha=0.6)
ax2.plot_wireframe(xs, ys, zs, alpha=0.2, color='green')
ax2.set_xlabel('X (m/s²)')
ax2.set_ylabel('Y (m/s²)')
ax2.set_zlabel('Z (m/s²)')
ax2.set_title('标定后测量数据 (应接近球面)')

# --- 子图3: 模长误差对比 ---
ax3 = fig.add_subplot(gs[0, 2])
x_idx = np.arange(len(err_before))
ax3.bar(x_idx[:30], err_before[:30], alpha=0.6, color='#e74c3c', label='标定前')
ax3.bar(x_idx[:30], err_after[:30], alpha=0.8, color='#2ecc71', label='标定后')
ax3.set_xlabel('采样点')
ax3.set_ylabel('|模长 - g| (m/s²)')
ax3.set_title('模长误差对比 (前30个点)')
ax3.legend()
ax3.grid(True, alpha=0.3)

# --- 子图4: 标定前各轴时间序列 ---
ax4 = fig.add_subplot(gs[1, 0])
t_samples = np.arange(len(all_measured))
ax4.plot(t_samples, all_measured[:, 0], 'r-', alpha=0.6, label='X')
ax4.plot(t_samples, all_measured[:, 1], 'g-', alpha=0.6, label='Y')
ax4.plot(t_samples, all_measured[:, 2], 'b-', alpha=0.6, label='Z')
ax4.axhline(y=g, color='k', linestyle='--', alpha=0.3)
ax4.axhline(y=-g, color='k', linestyle='--', alpha=0.3)
ax4.axhline(y=0, color='k', linestyle='--', alpha=0.3)
ax4.set_xlabel('采样点')
ax4.set_ylabel('加速度 (m/s²)')
ax4.set_title('标定前各轴加速度')
ax4.legend()
ax4.grid(True, alpha=0.3)

# --- 子图5: 标定后各轴时间序列 ---
ax5 = fig.add_subplot(gs[1, 1])
ax5.plot(t_samples, all_calibrated[:, 0], 'r-', alpha=0.6, label='X')
ax5.plot(t_samples, all_calibrated[:, 1], 'g-', alpha=0.6, label='Y')
ax5.plot(t_samples, all_calibrated[:, 2], 'b-', alpha=0.6, label='Z')
ax5.axhline(y=g, color='k', linestyle='--', alpha=0.3)
ax5.axhline(y=-g, color='k', linestyle='--', alpha=0.3)
ax5.axhline(y=0, color='k', linestyle='--', alpha=0.3)
ax5.set_xlabel('采样点')
ax5.set_ylabel('加速度 (m/s²)')
ax5.set_title('标定后各轴加速度')
ax5.legend()
ax5.grid(True, alpha=0.3)

# --- 子图6: 陀螺仪零偏估计 ---
ax6 = fig.add_subplot(gs[1, 2])
gyro_t = np.arange(GYRO_SAMPLES) * 0.01
ax6.plot(gyro_t, gyro_static[:, 0], 'r-', alpha=0.4, label='X轴')
ax6.plot(gyro_t, gyro_static[:, 1], 'g-', alpha=0.4, label='Y轴')
ax6.plot(gyro_t, gyro_static[:, 2], 'b-', alpha=0.4, label='Z轴')
ax6.axhline(y=gyro_bias_true[0], color='r', linestyle='--', alpha=0.8)
ax6.axhline(y=gyro_bias_true[1], color='g', linestyle='--', alpha=0.8)
ax6.axhline(y=gyro_bias_true[2], color='b', linestyle='--', alpha=0.8)
ax6.set_xlabel('时间 (s)')
ax6.set_ylabel('角速度 (°/s)')
ax6.set_title('陀螺仪静态零偏采集')
ax6.legend()
ax6.grid(True, alpha=0.3)

# --- 子图7: 零偏参数对比 ---
ax7 = fig.add_subplot(gs[2, 0])
labels = ['X轴', 'Y轴', 'Z轴']
x_pos = np.arange(3)
width = 0.35
bars1 = ax7.bar(x_pos - width/2, bias_true, width, label='真实零偏',
                color='#e74c3c', alpha=0.8)
bars2 = ax7.bar(x_pos + width/2, bias_est, width, label='估计零偏',
                color='#2ecc71', alpha=0.8)
ax7.set_xticks(x_pos)
ax7.set_xticklabels(labels)
ax7.set_ylabel('零偏 (m/s²)')
ax7.set_title('加速度计零偏对比')
ax7.legend()
ax7.grid(True, alpha=0.3, axis='y')

# --- 子图8: 比例因子对比 ---
ax8 = fig.add_subplot(gs[2, 1])
# 从M_cal对角线提取估计的比例因子
scale_est = np.diag(M_cal) / np.mean(np.diag(M_cal))  # 归一化
scale_true_norm = scale_true / np.mean(scale_true)
bars1 = ax8.bar(x_pos - width/2, scale_true_norm, width, label='真实比例因子',
                color='#3498db', alpha=0.8)
bars2 = ax8.bar(x_pos + width/2, scale_est, width, label='估计比例因子',
                color='#f39c12', alpha=0.8)
ax8.set_xticks(x_pos)
ax8.set_xticklabels(labels)
ax8.set_ylabel('归一化比例因子')
ax8.set_title('比例因子对比')
ax8.legend()
ax8.grid(True, alpha=0.3, axis='y')

# --- 子图9: 说明文字 ---
ax9 = fig.add_subplot(gs[2, 2])
ax9.axis('off')
info = (
    "IMU标定方法说明\n"
    "═══════════════════\n\n"
    "【六面标定法】\n"
    "将IMU分别朝6个方向放置\n"
    "(±X, ±Y, ±Z朝上)\n"
    "每个面采集多个数据点\n\n"
    "【椭球拟合】\n"
    "测量数据理论上应在球面上\n"
    "实际因误差呈椭球分布\n"
    "拟合椭球→提取标定参数\n\n"
    "【求解参数】\n"
    "• 零偏: 椭球中心偏移\n"
    "• 比例因子: 椭球半轴长度\n"
    "• 非正交: 椭球轴方向\n\n"
    f"标定改善: {np.mean(err_before)/max(np.mean(err_after),1e-10):.0f}x"
)
ax9.text(0.05, 0.95, info, transform=ax9.transAxes, fontsize=10,
         verticalalignment='top', fontfamily='monospace',
         bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))

plt.savefig('imu_calibration_simulation_result.png', dpi=150, bbox_inches='tight')
print("\n图表已保存: imu_calibration_simulation_result.png")
print("仿真完成!")
