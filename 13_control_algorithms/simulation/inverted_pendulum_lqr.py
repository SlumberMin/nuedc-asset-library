#!/usr/bin/env python3
"""
倒立摆 LQR 仿真 - 含 Riccati 方程求解

倒立摆模型:
  状态: x = [θ, θ_dot, x, x_dot]^T
  输入: u = 施加在小车上的力

  线性化后:  x_dot = A*x + B*u
  输出:      y = C*x

使用 LQR 最优控制器:
  u = -K*x
  K = R^{-1} * B^T * P
  其中 P 是代数 Riccati 方程的解:
    A^T*P + P*A - P*B*R^{-1}*B^T*P + Q = 0

依赖: numpy, scipy, matplotlib
"""

import numpy as np
from scipy import linalg
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def solve_riccati(A, B, Q, R):
    """
    求解连续代数 Riccati 方程 (CARE):
        A^T*P + P*A - P*B*R^{-1}*B^T*P + Q = 0

    使用 scipy.linalg.solve_continuous_are
    """
    P = linalg.solve_continuous_are(A, B, Q, R)
    return P


def lqr_gain(A, B, Q, R):
    """
    计算 LQR 最优增益 K
    """
    P = solve_riccati(A, B, Q, R)
    R_inv = linalg.inv(R)
    K = R_inv @ B.T @ P
    return K, P


def inverted_pendulum_model(M=0.5, m=0.2, l=0.3, g=9.81):
    """
    倒立摆线性化模型参数

    Parameters:
        M: 小车质量 (kg)
        m: 摆杆质量 (kg)
        l: 摆杆半长 (m)
        g: 重力加速度 (m/s^2)

    Returns:
        A: 系统矩阵 (4x4)
        B: 输入矩阵 (4x1)
    """
    # 在平衡点 (θ=0) 附近的线性化
    den = M + m  # 简化的分母

    A = np.array([
        [0, 1, 0, 0],
        [(M + m) * g / (M * l), 0, 0, 0],
        [0, 0, 0, 1],
        [-m * g / M, 0, 0, 0]
    ], dtype=float)

    B = np.array([
        [0],
        [-1 / (M * l)],
        [0],
        [1 / M]
    ], dtype=float)

    C = np.eye(4)

    return A, B, C


def simulate_closed_loop(A, B, K, x0, t_span, dt=0.001):
    """
    仿真闭环系统

    x_dot = (A - B*K)*x
    """
    t = np.arange(t_span[0], t_span[1], dt)
    n_steps = len(t)
    n_states = A.shape[0]

    x = np.zeros((n_steps, n_states))
    u = np.zeros((n_steps, 1))
    x[0] = x0

    A_cl = A - B @ K

    for i in range(n_steps - 1):
        u[i] = (-K @ x[i].reshape(-1, 1)).flatten()
        # 欧拉积分
        x_dot = A_cl @ x[i]
        x[i + 1] = x[i] + x_dot * dt

    u[-1] = (-K @ x[-1].reshape(-1, 1)).flatten()

    return t, x, u


def simulate_nonlinear(M, m, l, g, K, x0, t_span, dt=0.001):
    """
    使用非线性模型仿真(验证LQR在大角度下的鲁棒性)
    """
    t = np.arange(t_span[0], t_span[1], dt)
    n_steps = len(t)
    x = np.zeros((n_steps, 4))
    u = np.zeros(n_steps)
    x[0] = x0

    for i in range(n_steps - 1):
        theta, theta_dot, pos, vel = x[i]

        # LQR 控制力
        u[i] = float(-K @ x[i])

        # 非线性动力学 (简化)
        sin_t = np.sin(theta)
        cos_t = np.cos(theta)

        den = M + m - m * cos_t ** 2

        theta_ddot = ((M + m) * g * sin_t - cos_t * (u[i] + m * l * theta_dot ** 2 * sin_t)) / (l * den)
        x_ddot = (u[i] + m * l * (theta_dot ** 2 * sin_t - theta_ddot * cos_t)) / den

        x[i + 1, 0] = theta + theta_dot * dt
        x[i + 1, 1] = theta_dot + theta_ddot * dt
        x[i + 1, 2] = pos + vel * dt
        x[i + 1, 3] = vel + x_ddot * dt

    return t, x, u


def check_stability(A, B, K):
    """检查闭环极点"""
    A_cl = A - B @ K
    eigenvalues = np.linalg.eigvals(A_cl)
    print("闭环极点:")
    for i, ev in enumerate(eigenvalues):
        status = "稳定" if ev.real < 0 else "不稳定!"
        print(f"  极点 {i+1}: {ev:.4f}  ({status})")
    return eigenvalues


def main():
    print("=" * 60)
    print("  倒立摆 LQR 最优控制仿真")
    print("=" * 60)

    # 系统参数
    M, m, l, g = 0.5, 0.2, 0.3, 9.81
    A, B, C = inverted_pendulum_model(M, m, l, g)

    print(f"\n系统参数: M={M}kg, m={m}kg, l={l}m, g={g}m/s²")
    print(f"\n系统矩阵 A:\n{A}")
    print(f"\n输入矩阵 B:\n{B}")

    # 开环极点
    open_poles = np.linalg.eigvals(A)
    print(f"\n开环极点: {open_poles}")
    print("  -> 存在正实部极点,系统开环不稳定!")

    # LQR 权重设计
    # Q: 状态权重(越大越重视该状态)
    # R: 输入权重(越大控制越保守)
    Q = np.diag([100, 10, 10, 10])   # 重点抑制摆角偏差
    R = np.array([[1.0]])

    print(f"\nLQR 权重:")
    print(f"  Q = diag({np.diag(Q)})")
    print(f"  R = {R[0,0]}")

    # 求解 Riccati 方程 & LQR 增益
    K, P = lqr_gain(A, B, Q, R)
    print(f"\nRiccati 方程解 P:\n{np.round(P, 4)}")
    print(f"\nLQR 最优增益 K: {np.round(K, 4)}")

    # 稳定性检查
    check_stability(A, B, K)

    # 仿真 - 小角度初始偏差
    print("\n--- 仿真1: 小角度初始偏差 (5°) ---")
    x0_small = np.array([5 * np.pi / 180, 0, 0, 0])
    t1, x1, u1 = simulate_closed_loop(A, B, K, x0_small, (0, 2))

    # 仿真 - 大角度初始偏差(非线性模型)
    print("--- 仿真2: 大角度初始偏差 (30°, 非线性模型) ---")
    x0_large = np.array([30 * np.pi / 180, 0, 0, 0])
    t2, x2, u2 = simulate_nonlinear(M, m, l, g, K, x0_large, (0, 3))

    # 绘图
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    # 线性模型 - 摆角
    axes[0, 0].plot(t1, np.degrees(x1[:, 0]), 'b-', linewidth=2)
    axes[0, 0].set_title('线性模型 - 摆角 θ (小角度)')
    axes[0, 0].set_xlabel('时间 (s)')
    axes[0, 0].set_ylabel('角度 (°)')
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].axhline(y=0, color='k', linestyle='--', alpha=0.3)

    # 线性模型 - 小车位置
    axes[0, 1].plot(t1, x1[:, 2], 'r-', linewidth=2)
    axes[0, 1].set_title('线性模型 - 小车位置 x')
    axes[0, 1].set_xlabel('时间 (s)')
    axes[0, 1].set_ylabel('位置 (m)')
    axes[0, 1].grid(True, alpha=0.3)

    # 非线性模型 - 摆角
    axes[1, 0].plot(t2, np.degrees(x2[:, 0]), 'b-', linewidth=2)
    axes[1, 0].set_title('非线性模型 - 摆角 θ (大角度30°)')
    axes[1, 0].set_xlabel('时间 (s)')
    axes[1, 0].set_ylabel('角度 (°)')
    axes[1, 0].grid(True, alpha=0.3)
    axes[1, 0].axhline(y=0, color='k', linestyle='--', alpha=0.3)

    # 控制力
    axes[1, 1].plot(t2, u2, 'g-', linewidth=2)
    axes[1, 1].set_title('非线性模型 - 控制力 F')
    axes[1, 1].set_xlabel('时间 (s)')
    axes[1, 1].set_ylabel('力 (N)')
    axes[1, 1].grid(True, alpha=0.3)

    plt.suptitle('倒立摆 LQR 最优控制', fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig('inverted_pendulum_lqr.png', dpi=150)
    plt.close('all')

    print("\n仿真完成! 图形已保存为 inverted_pendulum_lqr.png")


if __name__ == "__main__":
    main()
