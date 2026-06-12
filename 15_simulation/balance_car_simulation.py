#!/usr/bin/env python3
"""
平衡车仿真 — 倒立摆模型 + LQR控制器
======================================
模拟两轮平衡车在受到扰动后，通过LQR控制器恢复直立状态。
使用倒立摆(Inverted Pendulum)动力学模型。
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
import matplotlib.patches as mpatches

# ============ 物理参数 ============
M = 1.0       # 车体质量 (kg)
m = 0.5       # 摆杆质量 (kg)
l = 0.5       # 摆杆半长 (m)
g = 9.81      # 重力加速度 (m/s^2)
J = m * l**2  # 摆杆转动惯量 (简化)

# ============ 状态空间模型 ============
# 状态: [x, x_dot, theta, theta_dot]
# 线性化后在竖直位置(theta=0)附近

# 系统矩阵 A (4x4)
A = np.array([
    [0, 1, 0, 0],
    [0, 0, -m*g/M, 0],
    [0, 0, 0, 1],
    [0, 0, (M+m)*g/(M*l), 0]
])

# 控制矩阵 B (4x1)
B = np.array([
    [0],
    [1/M],
    [0],
    [-1/(M*l)]
])

# ============ LQR控制器设计 ============
from scipy.linalg import solve_continuous_are

# 权重矩阵
Q = np.diag([10, 1, 100, 10])   # 状态权重：角度误差最重要
R = np.array([[0.1]])            # 控制输入权重

# 求解Riccati方程获得LQR增益
P = solve_continuous_are(A, B, Q, R)
K = np.linalg.inv(R) @ B.T @ P
print(f"LQR增益矩阵 K = {K.flatten()}")

# ============ 仿真参数 ============
dt = 0.005     # 时间步长 (s)
T = 5.0        # 仿真时长 (s)
N = int(T / dt)

# 初始状态 [x, x_dot, theta, theta_dot]
x0 = np.array([0.0, 0.0, 0.3, 0.0])  # 初始偏角0.3rad ≈ 17°

# ============ 仿真循环（含物理引擎） ============
t = np.zeros(N)
x = np.zeros((N, 4))
u = np.zeros(N)

x[0] = x0

for i in range(N - 1):
    # LQR控制力
    u[i] = -(K @ x[i])[0]
    u[i] = np.clip(u[i], -20, 20)  # 限幅

    # 非线性动力学（更真实的倒立摆）
    theta = x[i, 2]
    theta_dot = x[i, 3]
    x_dot = x[i, 1]

    # 非线性加速度
    sin_t = np.sin(theta)
    cos_t = np.cos(theta)
    denom = M + m - m * cos_t**2

    x_ddot = (u[i] + m * l * theta_dot**2 * sin_t - m * g * sin_t * cos_t / 1) / (denom + 1e-6)
    theta_ddot = ((M + m) * g * sin_t - u[i] * cos_t - m * l * theta_dot**2 * sin_t * cos_t) / (l * denom + 1e-6)

    # 加入地面摩擦
    friction = -0.1 * x_dot
    x_ddot += friction

    # Euler积分
    x[i+1, 0] = x[i, 0] + x_dot * dt
    x[i+1, 1] = x[i, 1] + x_ddot * dt
    x[i+1, 2] = x[i, 2] + theta_dot * dt
    x[i+1, 3] = x[i, 3] + theta_ddot * dt
    t[i+1] = t[i] + dt

u[-1] = -(K @ x[-1])[0]

# ============ 绘图 ============
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

fig, axes = plt.subplots(2, 2, figsize=(14, 10))
fig.suptitle('平衡车仿真 — 倒立摆 + LQR控制', fontsize=16, fontweight='bold')

# 图1：摆角
axes[0, 0].plot(t, np.degrees(x[:, 2]), 'b-', linewidth=1.5)
axes[0, 0].axhline(y=0, color='r', linestyle='--', alpha=0.5, label='目标')
axes[0, 0].set_xlabel('时间 (s)')
axes[0, 0].set_ylabel('摆角 (°)')
axes[0, 0].set_title('摆角响应')
axes[0, 0].legend()
axes[0, 0].grid(True, alpha=0.3)

# 图2：车体位置
axes[0, 1].plot(t, x[:, 0], 'g-', linewidth=1.5)
axes[0, 1].set_xlabel('时间 (s)')
axes[0, 1].set_ylabel('位置 (m)')
axes[0, 1].set_title('车体位移')
axes[0, 1].grid(True, alpha=0.3)

# 图3：控制力
axes[1, 0].plot(t, u, 'r-', linewidth=1)
axes[1, 0].set_xlabel('时间 (s)')
axes[1, 0].set_ylabel('控制力 (N)')
axes[1, 0].set_title('LQR控制力输出')
axes[1, 0].grid(True, alpha=0.3)

# 图4：相平面
axes[1, 1].plot(np.degrees(x[:, 2]), np.degrees(x[:, 3]), 'b-', linewidth=0.8)
axes[1, 1].plot(np.degrees(x[0, 2]), np.degrees(x[0, 3]), 'ro', markersize=10, label='起点')
axes[1, 1].plot(0, 0, 'g*', markersize=15, label='平衡点')
axes[1, 1].set_xlabel('摆角 (°)')
axes[1, 1].set_ylabel('角速度 (°/s)')
axes[1, 1].set_title('相平面轨迹')
axes[1, 1].legend()
axes[1, 1].grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('./nuedc-asset-library/15_simulation/balance_car_result.png', dpi=150, bbox_inches='tight')
print("✅ 平衡车仿真完成，图表已保存")

# 打印关键指标
settling_idx = np.where(np.abs(np.degrees(x[:, 2])) > 1.0)[0]
if len(settling_idx) > 0:
    # 找到最后一次超过1°的时间
    for j in range(len(settling_idx)-1, -1, -1):
        if j == len(settling_idx)-1 or settling_idx[j] != settling_idx[j+1]-1:
            settle_time = t[settling_idx[j]]
            break
    else:
        settle_time = t[settling_idx[0]]
    print(f"  调节时间(±1°): {settle_time:.2f} s")
print(f"  最大摆角: {np.degrees(np.max(np.abs(x[:, 2]))):.1f}°")
print(f"  最大控制力: {np.max(np.abs(u)):.1f} N")
print(f"  最终位置偏移: {x[-1, 0]:.3f} m")
