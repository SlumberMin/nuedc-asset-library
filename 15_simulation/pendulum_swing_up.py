#!/usr/bin/env python3
"""
倒立摆摆起仿真 - 能量控制 + LQR 切换
==============================================
功能：
  1. 能量控制将摆从下垂位置摆起
  2. 当角度足够小时自动切换到 LQR 稳定控制
  3. 实时动画显示摆的状态
使用：
  python pendulum_swing_up.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle, Circle
import matplotlib.patches as mpatches

# ========== 物理参数 ==========


def main():
    M = 0.5     # 小车质量 (kg)
    m = 0.1     # 摆杆质量 (kg)
    l = 0.5     # 摆杆半长 (m)  质心到转轴距离
    g = 9.81    # 重力加速度
    b = 0.1     # 摩擦系数

    dt = 0.005  # 仿真时间步长
    total_time = 15.0
    steps = int(total_time / dt)

    # ========== LQR 控制器 ==========
    def lqr(A, B, Q, R, max_iter=500, tol=1e-9):
        """离散代数Riccati方程求解"""
        P = Q.copy()
        for _ in range(max_iter):
            K = np.linalg.inv(R + B.T @ P @ B) @ B.T @ P @ A
            Pn = Q + A.T @ P @ A - A.T @ P @ B @ K
            if np.max(np.abs(Pn - P)) < tol:
                P = Pn
                break
            P = Pn
        return K

    # 线性化在 theta=pi (倒立), theta_dot=0
    J = (1/3) * m * (2*l)**2  # 转动惯量 ≈ m*l^2 for thin rod
    # 状态: [x, x_dot, theta - pi, theta_dot]
    A = np.array([
        [0, 1, 0, 0],
        [0, -b/M, -m*g/M, 0],
        [0, 0, 0, 1],
        [0, b/(M*l), (M+m)*g/(M*l), 0]
    ])
    B = np.array([[0], [1/M], [0], [-1/(M*l)]])

    Q = np.diag([100, 1, 200, 1])
    R = np.array([[0.01]])
    K_lqr = lqr(A, B, Q, R)

    # ========== 能量控制 ==========
    def energy_control(state):
        """能量泵送控制 - 将摆从下方摆起"""
        x, x_dot, theta, theta_dot = state
        # 摆杆能量 (以转轴为参考)
        E = 0.5 * m * l**2 * theta_dot**2 + m * g * l * (1 - np.cos(theta))
        E_desired = m * g * l * 2  # 目标能量: 摆到顶端
        k_energy = 2.0
        # 速度限幅
        u = k_energy * (E - E_desired) * theta_dot * np.cos(theta)
        u = np.clip(u, -15, 15)
        return u

    # ========== 动力学 ==========
    def dynamics(state, u):
        """倒立摆非线性动力学"""
        x, x_dot, theta, theta_dot = state
        sin_t = np.sin(theta)
        cos_t = np.cos(theta)

        den = M + m - m * cos_t**2
        x_ddot = (u - b * x_dot + m * l * theta_dot**2 * sin_t - m * g * sin_t * cos_t) / den
        theta_ddot = (-u * cos_t + b * x_dot * cos_t - m * l * theta_dot**2 * sin_t * cos_t
                      + (M + m) * g * sin_t) / (l * den)

        new_x_dot = x_dot + x_ddot * dt
        new_theta_dot = theta_dot + theta_ddot * dt
        new_x = x + new_x_dot * dt
        new_theta = theta + new_theta_dot * dt

        return np.array([new_x, new_x_dot, new_theta, new_theta_dot])

    # ========== 仿真主循环 ==========
    print("开始倒立摆摆起仿真...")
    print("阶段: 能量控制摆起 -> LQR 稳定")

    # 初始状态: 摆杆下垂 (theta=0)
    state = np.array([0.0, 0.0, 0.0, 0.0])

    history = {'t': [], 'x': [], 'theta': [], 'u': [], 'E': [], 'phase': []}
    switch_time = None
    swing_up_count = 0

    for i in range(steps):
        t = i * dt
        theta = state[2]
        theta_dot = state[3]

        # 判断是否切换到LQR (角度接近倒立 ±15°)
        angle_from_upright = abs(np.pi - theta) if theta > 0 else abs(-np.pi - theta)
        # 统一到 [0, 2pi]
        theta_mod = theta % (2 * np.pi)
        angle_from_top = min(abs(theta_mod - 2*np.pi), abs(theta_mod))

        if angle_from_top < 0.25 and abs(theta_dot) < 3.0:  # ~14度, 3 rad/s
            # LQR 控制
            state_lqr = np.array([state[0], state[1], theta_mod - np.pi, state[3]])
            u = float(-(K_lqr @ state_lqr))
            phase = 'LQR'
            if switch_time is None:
                switch_time = t
        else:
            # 能量控制
            u = energy_control(state)
            phase = '能量摆起'
            swing_up_count += 1

        u = np.clip(u, -20, 20)
        state = dynamics(state, u)

        E = 0.5 * m * l**2 * state[3]**2 + m * g * l * (1 - np.cos(state[2]))

        history['t'].append(t)
        history['x'].append(state[0])
        history['theta'].append(state[2])
        history['u'].append(u)
        history['E'].append(E)
        history['phase'].append(phase)

    print(f"仿真完成! 摆起阶段耗时: {switch_time:.2f}s" if switch_time else "摆起失败")

    # ========== 可视化 ==========
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('倒立摆摆起仿真 - 能量控制 + LQR', fontsize=14, fontweight='bold')

    t_arr = np.array(history['t'])
    theta_arr = np.array(history['theta'])
    u_arr = np.array(history['u'])
    E_arr = np.array(history['E'])

    # 角度曲线
    ax1 = axes[0, 0]
    ax1.plot(t_arr, np.degrees(theta_arr), 'b-', linewidth=0.5)
    ax1.axhline(y=180, color='r', linestyle='--', alpha=0.5, label='目标(180°)')
    ax1.axhline(y=0, color='gray', linestyle=':', alpha=0.5)
    if switch_time:
        ax1.axvline(x=switch_time, color='green', linestyle='--', alpha=0.7, label=f'切换LQR ({switch_time:.1f}s)')
    ax1.set_xlabel('时间 (s)')
    ax1.set_ylabel('角度 (°)')
    ax1.set_title('摆杆角度')
    ax1.legend(fontsize=8)
    ax1.grid(True, alpha=0.3)

    # 控制力
    ax2 = axes[0, 1]
    ax2.plot(t_arr, u_arr, 'r-', linewidth=0.5)
    ax2.set_xlabel('时间 (s)')
    ax2.set_ylabel('控制力 (N)')
    ax2.set_title('控制输入')
    ax2.grid(True, alpha=0.3)

    # 能量
    ax3 = axes[1, 0]
    E_target = m * g * l * 2
    ax3.plot(t_arr, E_arr, 'g-', linewidth=0.8)
    ax3.axhline(y=E_target, color='r', linestyle='--', alpha=0.5, label='目标能量')
    ax3.set_xlabel('时间 (s)')
    ax3.set_ylabel('能量 (J)')
    ax3.set_title('摆杆能量')
    ax3.legend(fontsize=8)
    ax3.grid(True, alpha=0.3)

    # 相图
    ax4 = axes[1, 1]
    ax4.plot(np.degrees(theta_arr), np.degrees(np.gradient(theta_arr, dt)), 'b-', linewidth=0.3, alpha=0.5)
    ax4.scatter([180], [0], color='red', s=100, marker='*', zorder=5, label='目标点')
    ax4.set_xlabel('角度 (°)')
    ax4.set_ylabel('角速度 (°/s)')
    ax4.set_title('相图 (角度 vs 角速度)')
    ax4.legend(fontsize=8)
    ax4.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'pendulum_swing_up_result.png'), dpi=150)
    print("图表已保存: pendulum_swing_up_result.png")
    plt.close('all')



if __name__ == '__main__':
    main()
