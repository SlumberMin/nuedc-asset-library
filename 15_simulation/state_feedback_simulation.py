#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
状态反馈控制仿真 - 极点配置
===========================
功能：能控性判断、极点配置、状态反馈设计
作者：nuedc-asset-library
"""

import numpy as np
from scipy import signal
from scipy.linalg import eigvals
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
plt.rcParams['axes.unicode_minus'] = False


# ============================================================
# 1. 系统定义 (二阶系统示例)
# ============================================================
# 状态空间: x' = Ax + Bu, y = Cx
A = np.array([[0, 1],
              [-2, -3]])   # 开环极点: -1, -2
B = np.array([[0],
              [1]])
C = np.array([[1, 0]])
D = np.array([[0]])


# ============================================================
# 2. 工具函数
# ============================================================
def controllability_matrix(A, B):
    """计算能控性矩阵 [B, AB, A^2B, ...]"""
    n = A.shape[0]
    Co = B.copy()
    for i in range(1, n):
        Co = np.hstack([Co, np.linalg.matrix_power(A, i) @ B])
    return Co


def place_poles(A, B, desired_poles):
    """
    Ackermann公式进行极点配置
    desired_poles: 期望闭环极点列表
    """
    n = A.shape[0]
    # 期望特征多项式
    poly_desired = np.poly(desired_poles)
    # Ackermann公式
    # K = [0 ... 0 1] * Co^{-1} * phi(A)
    Co = controllability_matrix(A, B)
    Co_inv = np.linalg.inv(Co)

    # 计算 phi(A) = A^n + a1*A^(n-1) + ... + an*I
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


# ============================================================
# 3. 主程序
# ============================================================
def main():
    print("=" * 60)
    print("状态反馈控制仿真 - 极点配置")
    print("=" * 60)

    # --- 能控性分析 ---
    Co = controllability_matrix(A, B)
    rank_Co = np.linalg.matrix_rank(Co)
    print(f"\n系统矩阵 A:\n{A}")
    print(f"输入矩阵 B:\n{B}")
    print(f"\n能控性矩阵:\n{Co}")
    print(f"能控性矩阵秩 = {rank_Co} (系统阶数 n={A.shape[0]})")
    print(f"系统{'能控' if rank_Co == A.shape[0] else '不能控'}")

    open_poles = eigvals(A)
    print(f"\n开环极点: {open_poles}")

    # --- 极点配置 ---
    desired_poles = [-3, -5]  # 期望闭环极点
    print(f"期望闭环极点: {desired_poles}")

    K = place_poles(A, B, desired_poles)
    print(f"状态反馈增益 K = {K}")

    A_cl = A - B @ K
    closed_poles = eigvals(A_cl)
    print(f"实际闭环极点: {np.round(closed_poles, 6)}")

    # --- 仿真对比 ---
    t = np.linspace(0, 5, 500)
    sys_open = signal.StateSpace(A, B, C, D)
    sys_closed = signal.StateSpace(A_cl, B, C, D)

    _, y_open = signal.step(sys_open, T=t)
    _, y_closed = signal.step(sys_closed, T=t)

    # --- 绘图 ---
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle('状态反馈控制 - 极点配置仿真', fontsize=14, fontweight='bold')

    # 阶跃响应对比
    axes[0, 0].plot(t, y_open, 'b-', linewidth=1.5, label='开环')
    axes[0, 0].plot(t, y_closed, 'r-', linewidth=1.5, label='状态反馈闭环')
    axes[0, 0].set_title('阶跃响应对比')
    axes[0, 0].set_ylabel('输出 y')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)

    # 极点位置
    axes[0, 1].scatter(open_poles.real, open_poles.imag, c='blue', s=100,
                       marker='x', linewidths=3, label='开环极点', zorder=5)
    axes[0, 1].scatter(np.array(desired_poles).real, np.array(desired_poles).imag,
                       c='red', s=100, marker='o', linewidths=3,
                       label='期望闭环极点', zorder=5)
    axes[0, 1].axhline(0, color='k', linewidth=0.5)
    axes[0, 1].axvline(0, color='k', linewidth=0.5)
    axes[0, 1].set_title('极点配置 (s平面)')
    axes[0, 1].set_xlabel('实部 σ')
    axes[0, 1].set_ylabel('虚部 jω')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)

    # 不同极点配置对比
    pole_sets = [[-2, -3], [-3, -5], [-5, -8], [-4+2j, -4-2j]]
    labels = ['[-2,-3]', '[-3,-5]', '[-5,-8]', '[-4±2j]']
    for poles, label in zip(pole_sets, labels):
        K_i = place_poles(A, B, poles)
        A_cl_i = A - B @ K_i
        sys_i = signal.StateSpace(A_cl_i, B, C, D)
        _, y_i = signal.step(sys_i, T=t)
        axes[1, 0].plot(t, y_i, linewidth=1.5, label=f'极点={label}')

    axes[1, 0].set_title('不同极点配置的响应对比')
    axes[1, 0].set_xlabel('时间 (s)')
    axes[1, 0].set_ylabel('输出 y')
    axes[1, 0].legend(fontsize=8)
    axes[1, 0].grid(True, alpha=0.3)

    # 控制量 (无参考输入)
    t_sim = np.linspace(0, 5, 500)
    x0 = np.array([1.0, 0.0])  # 初始状态
    from scipy.integrate import odeint

    def open_loop_dynamics(x, t):
        return (A @ x).flatten()

    def closed_loop_dynamics(x, t):
        return (A_cl @ x).flatten()

    x_open = odeint(open_loop_dynamics, x0, t_sim)
    x_closed = odeint(closed_loop_dynamics, x0, t_sim)
    u_closed = -K @ x_closed.T

    axes[1, 1].plot(t_sim, u_closed.flatten(), 'r-', linewidth=1.5, label='控制量 u(t)')
    axes[1, 1].plot(t_sim, x_closed[:, 0], 'b--', linewidth=1.5, label='$x_1$(状态)')
    axes[1, 1].set_title('闭环控制量与状态响应')
    axes[1, 1].set_xlabel('时间 (s)')
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('state_feedback_result.png', dpi=150, bbox_inches='tight')
    plt.close('all')
    print("\n仿真结果已保存: state_feedback_result.png")


if __name__ == '__main__':
    main()
