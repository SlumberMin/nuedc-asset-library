#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
观测器仿真 - 状态估计 + 观测器-状态反馈控制
============================================
功能：能观性判断、全阶观测器设计、降阶观测器、分离原理验证
作者：nuedc-asset-library
"""

import numpy as np
from scipy import signal
from scipy.linalg import eigvals
from scipy.integrate import odeint
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
plt.rcParams['axes.unicode_minus'] = False


# ============================================================
# 1. 系统定义
# ============================================================
A = np.array([[0, 1],
              [-2, -3]])
B = np.array([[0],
              [1]])
C = np.array([[1, 0]])  # 只能观测 x1
D = np.array([[0]])


# ============================================================
# 2. 工具函数
# ============================================================
def observability_matrix(A, C):
    """计算能观性矩阵"""
    n = A.shape[0]
    Ob = C.copy()
    for i in range(1, n):
        Ob = np.vstack([Ob, C @ np.linalg.matrix_power(A, i)])
    return Ob


def place_poles_k(A, B, desired_poles):
    """极点配置求反馈增益 K"""
    n = A.shape[0]
    poly_desired = np.poly(desired_poles)
    Co = B.copy()
    for i in range(1, n):
        Co = np.hstack([Co, np.linalg.matrix_power(A, i) @ B])
    Co_inv = np.linalg.inv(Co)
    phi_A = np.zeros_like(A, dtype=float)
    for i, coeff in enumerate(poly_desired):
        power = n - i
        if power == 0:
            phi_A += coeff * np.eye(n)
        else:
            phi_A += coeff * np.linalg.matrix_power(A, power)
    e_n = np.zeros((1, n))
    e_n[0, -1] = 1.0
    K = e_n @ Co_inv @ phi_A
    return K.real


def design_observer(A, C, desired_poles):
    """
    设计全阶 Luenberger 观测器
    观测器增益 L: (A - LC) 的极点 = desired_poles
    利用对偶性: A^T, C^T 的极点配置
    """
    A_T = A.T
    C_T = C.T.reshape(-1, 1)
    L_T = place_poles_k(A_T, C_T, desired_poles)
    return L_T.T


# ============================================================
# 3. 主程序
# ============================================================
def main():
    print("=" * 60)
    print("观测器仿真 - 状态估计 + 控制")
    print("=" * 60)

    n = A.shape[0]

    # --- 能观性分析 ---
    Ob = observability_matrix(A, C)
    rank_Ob = np.linalg.matrix_rank(Ob)
    print(f"\n能观性矩阵:\n{Ob}")
    print(f"秩 = {rank_Ob}, 系统{'能观' if rank_Ob == n else '不能观'}")

    # --- 设计状态反馈 K ---
    desired_ctrl_poles = [-3, -5]
    K = place_poles_k(A, B, desired_ctrl_poles)
    print(f"\n状态反馈 K = {K}")
    print(f"  闭环极点: {np.round(eigvals(A - B @ K), 4)}")

    # --- 设计观测器 L ---
    desired_obs_poles = [-10, -12]  # 观测器比控制器快
    L = design_observer(A, C, desired_obs_poles)
    print(f"观测器增益 L = {L.flatten()}")
    print(f"  观测器极点: {np.round(eigvals(A - L * C), 4)}")

    # --- 仿真: 真实系统 + 观测器 ---
    t = np.linspace(0, 5, 1000)
    x0 = np.array([2.0, 1.0])     # 真实初始状态
    x_hat0 = np.array([0.0, 0.0]) # 观测器初始估计

    def augmented_dynamics(state, t):
        """增广系统: [x; x_hat]"""
        x = state[:n]
        x_hat = state[n:]
        y = (C @ x).item()
        y_hat = (C @ x_hat).item()
        u = -(K @ x_hat).item()  # 用估计状态反馈
        dx = (A @ x + B.flatten() * u).flatten()
        dx_hat = (A @ x_hat + B.flatten() * u + L.flatten() * (y - y_hat)).flatten()
        return np.concatenate([dx, dx_hat])

    state0 = np.concatenate([x0, x_hat0])
    states = odeint(augmented_dynamics, state0, t)
    x_true = states[:, :n]
    x_hat = states[:, n:]
    error = x_true - x_hat

    # --- 绘图 ---
    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    fig.suptitle('观测器仿真 - 状态估计与控制', fontsize=14, fontweight='bold')

    # 真实状态 vs 估计状态
    axes[0, 0].plot(t, x_true[:, 0], 'b-', linewidth=2, label='$x_1$ 真实')
    axes[0, 0].plot(t, x_hat[:, 0], 'r--', linewidth=2, label='$\\hat{x}_1$ 估计')
    axes[0, 0].plot(t, x_true[:, 1], 'g-', linewidth=2, label='$x_2$ 真实')
    axes[0, 0].plot(t, x_hat[:, 1], 'm--', linewidth=2, label='$\\hat{x}_2$ 估计')
    axes[0, 0].set_title('状态与估计对比')
    axes[0, 0].set_ylabel('状态值')
    axes[0, 0].legend(fontsize=9)
    axes[0, 0].grid(True, alpha=0.3)

    # 估计误差
    axes[0, 1].plot(t, error[:, 0], 'b-', linewidth=1.5, label='$e_1 = x_1 - \\hat{x}_1$')
    axes[0, 1].plot(t, error[:, 1], 'r-', linewidth=1.5, label='$e_2 = x_2 - \\hat{x}_2$')
    axes[0, 1].set_title('状态估计误差')
    axes[0, 1].set_ylabel('误差')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)

    # 极点配置示意
    ctrl_poles = eigvals(A - B @ K)
    obs_poles = eigvals(A - L * C)
    axes[1, 0].scatter(ctrl_poles.real, ctrl_poles.imag, c='blue', s=120,
                       marker='x', linewidths=3, label='控制器极点', zorder=5)
    axes[1, 0].scatter(obs_poles.real, obs_poles.imag, c='red', s=120,
                       marker='s', linewidths=2, label='观测器极点', zorder=5)
    axes[1, 0].axhline(0, color='k', linewidth=0.5)
    axes[1, 0].axvline(0, color='k', linewidth=0.5)
    axes[1, 0].set_title('分离原理: 控制器极点 + 观测器极点')
    axes[1, 0].set_xlabel('实部 σ')
    axes[1, 0].set_ylabel('虚部 jω')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)

    # 控制量
    u = np.array([-(K @ x_hat[i]).item() for i in range(len(t))])
    axes[1, 1].plot(t, u, 'k-', linewidth=1.5, label='控制量 u(t)')
    axes[1, 1].set_title('控制量')
    axes[1, 1].set_xlabel('时间 (s)')
    axes[1, 1].set_ylabel('u')
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('observer_result.png', dpi=150, bbox_inches='tight')
    plt.close('all')
    print("\n仿真结果已保存: observer_result.png")


if __name__ == '__main__':
    main()
