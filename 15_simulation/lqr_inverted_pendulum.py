#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LQR 倒立摆仿真
===============
仿真内容：摆起控制 + LQR平衡控制
被控对象：倒立摆 (Inverted Pendulum on Cart)
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')


def main():
    import matplotlib.pyplot as plt
    matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
    matplotlib.rcParams['axes.unicode_minus'] = False

    # ============ 倒立摆物理参数 ============
    M = 0.5     # 小车质量 (kg)
    m = 0.2     # 摆杆质量 (kg)
    l = 0.3     # 摆杆半长 (m)
    g = 9.81    # 重力加速度 (m/s^2)
    b_fric = 0.1  # 摩擦系数

    dt = 0.001  # 仿真步长
    T_total = 10.0
    N = int(T_total / dt)
    t = np.arange(N) * dt

    # ============ 倒立摆动力学模型 ============
    def pendulum_dynamics(state, F):
        """
        倒立摆非线性动力学
        state = [x, x_dot, theta, theta_dot]
        F: 施加在小车上的力
        """
        x, x_dot, theta, theta_dot = state

        sin_t = np.sin(theta)
        cos_t = np.cos(theta)

        # 质量矩阵求解
        M_total = M + m
        D = M_total * m * l**2 - (m * l * cos_t)**2

        # 加速度
        theta_ddot = (m * g * l * sin_t * M_total - m * l * cos_t * (F - b_fric * x_dot + m * l * theta_dot**2 * sin_t)) / D
        x_ddot = (F - b_fric * x_dot + m * l * (theta_dot**2 * sin_t - theta_ddot * cos_t)) / M_total

        return np.array([x_dot, x_ddot, theta_dot, theta_ddot])

    def rk4_step(state, F, dt):
        """四阶龙格-库塔积分"""
        k1 = pendulum_dynamics(state, F)
        k2 = pendulum_dynamics(state + 0.5*dt*k1, F)
        k3 = pendulum_dynamics(state + 0.5*dt*k2, F)
        k4 = pendulum_dynamics(state + dt*k3, F)
        return state + (dt/6)*(k1 + 2*k2 + 2*k3 + k4)

    # ============ LQR控制器设计 ============
    def compute_lqr_gain():
        """
        计算LQR增益矩阵 K
        线性化倒立摆在竖直位置 (theta=0) 附近的状态空间模型
        状态: [x, x_dot, theta, theta_dot]
        """
        # 线性化后的状态空间矩阵 (在theta=0处)
        # x' = A*x + B*u
        M_total = M + m
        D = M_total * m * l**2  # cos(0)=1时的分母简化

        A = np.array([
            [0, 1, 0, 0],
            [0, -b_fric * m * l**2 / D, m**2 * g * l**2 / D, 0],
            [0, 0, 0, 1],
            [0, -m * l * b_fric / (D), M_total * m * g * l / D, 0]
        ])

        B = np.array([
            [0],
            [m * l**2 / D],
            [0],
            [m * l / D]
        ])

        # LQR权重矩阵
        Q = np.diag([100, 1, 100, 1])  # 状态权重
        R = np.array([[0.1]])            # 控制权重

        # 通过迭代求解代数Riccati方程
        P = np.eye(4) * 100
        for _ in range(1000):
            K = np.linalg.inv(R + B.T @ P @ B) @ B.T @ P @ A
            P_new = Q + A.T @ P @ A - A.T @ P @ B @ K
            if np.max(np.abs(P_new - P)) < 1e-10:
                break
            P = P_new

        return K, A, B

    # ============ 摆起控制器（能量控制） ============
    def swing_up_energy_control(state, K_lqr, lqr_region=0.3):
        """
        基于能量的摆起控制 + LQR切换
        当摆角小于阈值时切换到LQR控制
        """
        theta = state[2]
        theta_dot = state[3]

        # 角度归一化到[-pi, pi]
        while theta > np.pi:
            theta -= 2 * np.pi
        while theta < -np.pi:
            theta += 2 * np.pi

        # 如果摆角足够小，使用LQR
        if abs(theta) < lqr_region and abs(theta_dot) < 2.0:
            x_ref = np.array([0, 0, 0, 0])  # 目标状态
            F = -K_lqr @ (state - x_ref)
            return float(F[0])

        # 否则使用能量控制摆起
        # 摆杆能量
        E = 0.5 * m * l**2 * theta_dot**2 + m * g * l * (np.cos(theta) - 1)
        E_desired = 0  # 竖直位置的能量

        # 能量误差
        E_err = E - E_desired

        # 控制律：根据能量差和角速度确定力的方向
        k_energy = 5.0
        F = k_energy * E_err * np.sign(theta_dot * np.cos(theta))

        # 限制力的大小
        F = np.clip(F, -50, 50)
        return F

    # ============ 运行仿真 ============
    print("LQR倒立摆仿真开始...")

    K_lqr, A, B = compute_lqr_gain()
    print(f"LQR增益矩阵 K = {K_lqr.flatten()}")

    # 初始状态：摆杆倒下（theta = pi）
    state = np.array([0.0, 0.0, np.pi - 0.1, 0.0])  # 初始角度接近倒立

    # 记录数组
    x_log = np.zeros(N)
    theta_log = np.zeros(N)
    F_log = np.zeros(N)
    mode_log = np.zeros(N)  # 0=摆起, 1=LQR

    for i in range(N):
        x_log[i] = state[0]
        theta_log[i] = state[2]

        # 控制
        F = swing_up_energy_control(state, K_lqr, lqr_region=0.4)
        F_log[i] = F

        # 记录控制模式
        theta = state[2]
        while theta > np.pi: theta -= 2*np.pi
        while theta < -np.pi: theta += 2*np.pi
        mode_log[i] = 1 if abs(theta) < 0.4 else 0

        # 状态更新
        state = rk4_step(state, F, dt)

        # 位置限幅
        state[0] = np.clip(state[0], -2.0, 2.0)

    # 计算性能指标（LQR平衡阶段）
    # 找到切换到LQR的时间点
    lqr_start_idx = 0
    for i in range(N):
        if mode_log[i] == 1:
            lqr_start_idx = i
            break

    if lqr_start_idx < N - 100:
        theta_lqr = theta_log[lqr_start_idx:]
        t_lqr = t[lqr_start_idx:]
        IAE = np.sum(np.abs(theta_lqr)) * dt
        ISE = np.sum(theta_lqr**2) * dt
        settling_idx = -1
        for i in range(len(theta_lqr)):
            if np.abs(theta_lqr[i]) < 0.02:  # 2% 调节
                settling_idx = i
                break
        settling_time = t_lqr[settling_idx] if settling_idx > 0 else t_lqr[-1]
        print(f"LQR平衡阶段 - IAE:{IAE:.4f} ISE:{ISE:.6f} 调节时间:{settling_time:.3f}s")

    # ============ 绘图 ============
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('LQR倒立摆仿真（摆起 + 平衡）', fontsize=16, fontweight='bold')

    # 小车位置
    ax = axes[0, 0]
    ax.plot(t, x_log, 'b-', linewidth=1)
    ax.set_title('小车位置')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('位置 (m)')
    ax.grid(True, alpha=0.3)

    # 摆杆角度
    ax = axes[0, 1]
    ax.plot(t, np.degrees(theta_log), 'r-', linewidth=1)
    ax.axhline(y=0, color='k', linestyle=':', alpha=0.5, label='竖直位置')
    ax.axhline(y=180, color='gray', linestyle=':', alpha=0.5, label='倒立位置')
    ax.set_title('摆杆角度')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('角度 (°)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 控制力
    ax = axes[1, 0]
    ax.plot(t, F_log, 'g-', linewidth=0.5, alpha=0.7)
    ax.set_title('控制力')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('力 (N)')
    ax.grid(True, alpha=0.3)

    # 相平面图 (theta vs theta_dot)
    ax = axes[1, 1]
    scatter = ax.scatter(np.degrees(theta_log[::10]), 
                          np.gradient(theta_log, dt)[::10] / (2*np.pi),
                          c=t[::10], cmap='viridis', s=1, alpha=0.5)
    ax.set_title('相平面图 (角度 vs 角速度)')
    ax.set_xlabel('角度 (°)')
    ax.set_ylabel('角速度 (rps)')
    ax.grid(True, alpha=0.3)
    plt.colorbar(scatter, ax=ax, label='时间 (s)')

    plt.tight_layout()
    plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'lqr_inverted_pendulum_result.png'), dpi=150, bbox_inches='tight')
    print("\n图表已保存: lqr_inverted_pendulum_result.png")
    print("LQR倒立摆仿真完成!")



if __name__ == '__main__':
    main()
