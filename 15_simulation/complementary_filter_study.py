# -*- coding: utf-8 -*-
"""
互补滤波器研究仿真 — 不同时间常数的效果
========================================
场景：融合加速度计（低频可靠）与陀螺仪（高频可靠）估计姿态角。
对比时间常数 τ = 0.5, 1.0, 2.0, 5.0 s 的响应差异。

用法：python complementary_filter_study.py
"""

import numpy as np
import matplotlib.pyplot as plt
import os, sys

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

# ── 仿真参数 ──────────────────────────────────────────────
DT = 0.01          # 采样周期 10ms (100Hz)
T_END = 20.0       # 总时长
N = int(T_END / DT)
t = np.linspace(0, T_END, N)

# ── 真实姿态：正弦摆动 ───────────────────────────────────
true_angle = 45 * np.sin(2 * np.pi * 0.2 * t)  # ±45°, 0.2Hz

# ── 传感器仿真 ───────────────────────────────────────────
np.random.seed(42)
gyro_bias = 2.0  # deg/s 零偏
gyro_noise_std = 1.5
acc_noise_std = 3.0

# 陀螺仪：角速度 + 零偏 + 噪声
gyro_rate = np.gradient(true_angle, DT) + gyro_bias + np.random.randn(N) * gyro_noise_std

# 加速度计：角度 + 噪声（低频可靠，但有高频噪声）
acc_angle = true_angle + np.random.randn(N) * acc_noise_std

# ── 互补滤波器 ──────────────────────────────────────────
def complementary_filter(gyro_rate, acc_angle, dt, tau):
    """alpha = tau / (tau + dt)"""
    alpha = tau / (tau + dt)
    angle = np.zeros_like(gyro_rate)
    angle[0] = acc_angle[0]
    for i in range(1, len(gyro_rate)):
        angle[i] = alpha * (angle[i-1] + gyro_rate[i] * dt) + (1 - alpha) * acc_angle[i]
    return angle

taus = [0.5, 1.0, 2.0, 5.0]
results = {}
for tau in taus:
    results[tau] = complementary_filter(gyro_rate, acc_angle, DT, tau)

# ── 计算误差指标 ──────────────────────────────────────────
print("=" * 60)
print("互补滤波器 — 不同时间常数 τ 对比")
print("=" * 60)
print(f"{'τ (s)':>8} {'RMSE (°)':>12} {'最大误差 (°)':>14} {'α':>8}")
print("-" * 46)
for tau in taus:
    alpha = tau / (tau + DT)
    err = results[tau] - true_angle
    rmse = np.sqrt(np.mean(err**2))
    max_err = np.max(np.abs(err))
    print(f"{tau:>8.1f} {rmse:>12.2f} {max_err:>14.2f} {alpha:>8.4f}")

# ── 绘图 ──────────────────────────────────────────────────
fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)

# 子图1：原始传感器数据
ax = axes[0]
ax.plot(t, true_angle, 'k-', lw=2, label='真实角度')
ax.plot(t, acc_angle, 'g.', markersize=1, alpha=0.3, label='加速度计')
# 累积陀螺仪（会漂移）
gyro_integrated = np.cumsum(gyro_rate * DT)
gyro_integrated -= gyro_integrated[0] - true_angle[0]
ax.plot(t, gyro_integrated, 'r-', alpha=0.6, label='陀螺仪积分（漂移）')
ax.set_ylabel('角度 (°)')
ax.set_title('原始传感器数据')
ax.legend(loc='upper right')
ax.grid(True, alpha=0.3)

# 子图2：不同τ的滤波结果
ax = axes[1]
colors = ['#2196F3', '#4CAF50', '#FF9800', '#F44336']
for tau, color in zip(taus, colors):
    ax.plot(t, results[tau], color=color, lw=1, label=f'τ={tau}s')
ax.plot(t, true_angle, 'k--', lw=2, label='真实角度')
ax.set_ylabel('角度 (°)')
ax.set_title('互补滤波器输出（不同时间常数）')
ax.legend(loc='upper right')
ax.grid(True, alpha=0.3)

# 子图3：误差对比
ax = axes[2]
for tau, color in zip(taus, colors):
    err = results[tau] - true_angle
    ax.plot(t, err, color=color, lw=1, label=f'τ={tau}s')
ax.set_xlabel('时间 (s)')
ax.set_ylabel('误差 (°)')
ax.set_title('滤波误差')
ax.legend(loc='upper right')
ax.grid(True, alpha=0.3)

plt.tight_layout()

out_dir = os.path.dirname(os.path.abspath(__file__))
out_path = os.path.join(out_dir, 'complementary_filter_study.png')
plt.savefig(out_path, dpi=150, bbox_inches='tight')
print(f"\n图表已保存: {out_path}")
plt.show()

print("\n结论：")
print("  τ 小 → 更信任加速度计 → 响应快，但高频噪声大")
print("  τ 大 → 更信任陀螺仪   → 噪声小，但响应慢、收敛慢")
print("  推荐：τ = 1~2s (100Hz采样下 α ≈ 0.99~0.995)")
