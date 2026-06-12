# -*- coding: utf-8 -*-
"""
Luenberger观测器设计仿真
=======================
使用Luenberger观测器估计线性系统的内部状态
应用场景: 电机系统中估计不可直接测量的状态(如电流导数)
"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False


def design_observer(A, B, C, poles_desired):
    """
    设计Luenberger观测器增益L

    Parameters
    ----------
    A, B, C : np.ndarray
        系统状态空间矩阵
    poles_desired : np.ndarray
        期望观测器极点

    Returns
    -------
    L : np.ndarray
        观测器增益向量
    A_obs : np.ndarray
        观测器闭环矩阵 (A - L*C)
    """
    # Ackermann公式: 通过匹配特征多项式求解L
    # 对于2阶系统: 设 L = [l1, l2]^T
    # (A - L*C)的特征多项式 = s^2 + (a11+a22+l1*C11) s + ...
    # 简化求解: 直接用期望极点构造特征多项式系数
    n = A.shape[0]
    # 期望特征多项式系数
    poly_coeffs = np.poly(poles_desired)  # [1, -(p1+p2), p1*p2] for 2nd order

    # 使用place (pole placement) 求解 L
    # 对于可观系统，L = acker(A^T, C^T, poles)^T
    from numpy.linalg import matrix_power
    # Ackermann公式 for observer: L = (W_o)^{-1} * (desired_poly - actual_poly)
    # 简化: 直接构造可观性矩阵求解
    W = C.copy()
    CA = C @ A
    W = np.vstack([W, CA])

    # 构造目标: (A-LC)的特征多项式 = desired_poly
    # 展开后得到线性方程组
    # 对于2阶: l1, l2 使得特征多项式匹配
    # (A-LC) = [[a11-l1*C1, a12-l1*C2], [a21-l2*C1, a22-l2*C2]]
    # trace(A-LC) = (a11+a22) - (l1*C1 + l2*C2) = -(p1+p2)
    # det(A-LC) = ... = p1*p2

    # 直接用SVD求解 Lyapunov-like方程
    # 简单2阶情况: 直接计算
    if n == 2:
        a11, a12 = A[0, 0], A[0, 1]
        a21, a22 = A[1, 0], A[1, 1]
        c1, c2 = C[0, 0], C[0, 1]

        # 期望: s^2 - trace_desired * s + det_desired
        trace_desired = poles_desired[0] + poles_desired[1]
        det_desired = poles_desired[0] * poles_desired[1]

        # (A-LC) 的迹 = (a11-l1*c1) + (a22-l2*c2)
        # (A-LC) 的行列式 = (a11-l1*c1)(a22-l2*c2) - (a12-l1*c2)(a21-l2*c1)

        # 从迹约束: l1*c1 + l2*c2 = (a11+a22) - trace_desired
        # 从行列式约束: 展开后第二个方程
        sum_lc = (a11 + a22) - trace_desired

        # 构造方程组 [c1, c2; ...] * [l1; l2] = [sum_lc; ...]
        # 行列式展开:
        # (a11*a22 - a12*a21) - l1*c1*a22 - l2*c2*a11 + l1*l2*c1*c2
        #   - (-l1*c2*a21 - l2*c1*a12 + l1*l2*c2*c1)
        # = det(A) - a22*l1*c1 - a11*l2*c2 + a21*l1*c2 + a12*l2*c1
        # = det(A) - l1*(a22*c1 - a21*c2) - l2*(a11*c2 - a12*c1)
        # = det_desired

        # 两个方程:
        # c1*l1 + c2*l2 = sum_lc
        # (a22*c1 - a21*c2)*l1 + (a11*c2 - a12*c1)*l2 = det(A) - det_desired
        det_A = a11 * a22 - a12 * a21
        coeff_mat = np.array([
            [c1, c2],
            [a22 * c1 - a21 * c2, a11 * c2 - a12 * c1]
        ])
        rhs = np.array([sum_lc, det_A - det_desired])

        L = np.linalg.solve(coeff_mat, rhs).reshape(n, 1)
    else:
        # 通用: 使用对偶系统的极点配置
        # L = place(A^T, C^T, poles)^T
        from scipy.signal import place_poles
        result = place_poles(A.T, C.T, poles_desired)
        L = result.gain_matrix.T

    A_obs = A - L @ C
    return L, A_obs


def run_simulation(dt=0.001, T=2.0, save_path=None):
    """
    运行Luenberger观测器仿真

    Parameters
    ----------
    dt : float
        仿真步长 (s)
    T : float
        仿真总时长 (s)
    save_path : str or None
        图表保存路径
    """
    t = np.arange(0, T, dt)
    N = len(t)

    # 二阶系统: 电机模型
    # x_dot = A*x + B*u, y = C*x
    A = np.array([[0, 1], [-10, -2]])
    B = np.array([[0], [1]])
    C = np.array([[1, 0]])  # 只能观测位置(角度)

    # ========== 设计Luenberger观测器 ==========
    poles_desired = np.array([-5, -5])
    L, A_obs = design_observer(A, B, C, poles_desired)
    l1, l2 = L[0, 0], L[1, 0]

    print("=== Luenberger 观测器参数 ===")
    print(f"  系统矩阵 A:\n{A}")
    print(f"  观测器增益 L = [{l1:.1f}, {l2:.1f}]^T")
    print(f"  期望极点: {poles_desired}")

    eig_obs = np.linalg.eigvals(A_obs)
    print(f"  实际观测器极点: {eig_obs}")

    # ========== 仿真 ==========
    u = 2.0 * np.sin(2 * np.pi * 5 * t) + 1.0

    # 真实系统
    x_true = np.zeros((N, 2))
    x_true[0] = [1.0, 0.0]
    meas_noise = np.random.randn(N) * 0.02

    # 观测器
    x_hat = np.zeros((N, 2))
    x_hat[0] = [0.0, 0.5]  # 初始估计有偏差

    for i in range(N - 1):
        x_true[i + 1] = x_true[i] + (A @ x_true[i:i + 1].T + B * u[i]).flatten() * dt
        y_meas = C @ x_true[i:i + 1].T + meas_noise[i]
        y_hat = C @ x_hat[i:i + 1].T
        x_hat[i + 1] = x_hat[i] + (A @ x_hat[i:i + 1].T + B * u[i] + L * (y_meas - y_hat)).flatten() * dt

    # ========== 不同观测器增益对比 ==========
    L_fast = np.array([[20], [50]])
    L_slow = np.array([[3], [5]])
    x_hat_fast = np.zeros((N, 2))
    x_hat_slow = np.zeros((N, 2))
    x_hat_fast[0] = [0.0, 0.5]
    x_hat_slow[0] = [0.0, 0.5]

    for i in range(N - 1):
        y_meas = C @ x_true[i:i + 1].T + meas_noise[i]
        y_hat_f = C @ x_hat_fast[i:i + 1].T
        x_hat_fast[i + 1] = x_hat_fast[i] + (A @ x_hat_fast[i:i + 1].T + B * u[i] + L_fast * (y_meas - y_hat_f)).flatten() * dt
        y_hat_s = C @ x_hat_slow[i:i + 1].T
        x_hat_slow[i + 1] = x_hat_slow[i] + (A @ x_hat_slow[i:i + 1].T + B * u[i] + L_slow * (y_meas - y_hat_s)).flatten() * dt

    # ========== 性能指标 ==========
    rmse_x1 = np.sqrt(np.mean((x_hat[:, 0] - x_true[:, 0]) ** 2))
    rmse_x2 = np.sqrt(np.mean((x_hat[:, 1] - x_true[:, 1]) ** 2))
    rmse_fast_x2 = np.sqrt(np.mean((x_hat_fast[:, 1] - x_true[:, 1]) ** 2))
    rmse_slow_x2 = np.sqrt(np.mean((x_hat_slow[:, 1] - x_true[:, 1]) ** 2))

    print(f"\n=== 观测性能 ===")
    print(f"  x1 RMSE: {rmse_x1:.4f}")
    print(f"  x2 RMSE: {rmse_x2:.4f}")
    print(f"  快观测器 x2 RMSE: {rmse_fast_x2:.4f}")
    print(f"  慢观测器 x2 RMSE: {rmse_slow_x2:.4f}")

    # ========== 绘图 ==========
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))

    axes[0].plot(t, x_true[:, 0], 'k-', linewidth=1.5, label='真实状态 x1')
    axes[0].plot(t, x_hat[:, 0], 'r--', linewidth=1.2, label='观测器估计 x1')
    axes[0].set_ylabel('状态 x1 (位置)')
    axes[0].set_title('Luenberger 观测器 — 状态x1估计')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t, x_true[:, 1], 'k-', linewidth=1.5, label='真实状态 x2')
    axes[1].plot(t, x_hat[:, 1], 'r--', linewidth=1.2, label='观测器估计 x2')
    axes[1].plot(t, x_hat_fast[:, 1], 'b:', linewidth=1.0, alpha=0.7, label='快增益估计 x2')
    axes[1].set_ylabel('状态 x2 (速度)')
    axes[1].set_title('Luenberger 观测器 — 状态x2估计（不可直接测量）')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(t, x_hat[:, 0] - x_true[:, 0], 'r-', linewidth=0.8, label='x1估计误差')
    axes[2].plot(t, x_hat[:, 1] - x_true[:, 1], 'b-', linewidth=0.8, label='x2估计误差')
    axes[2].set_xlabel('时间 (s)')
    axes[2].set_ylabel('估计误差')
    axes[2].set_title('观测器估计误差')
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    if save_path is None:
        save_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                 'luenberger_observer_design.png')
    plt.savefig(save_path, dpi=150)
    plt.close()
    print(f"\n已保存: {save_path}")


if __name__ == '__main__':
    run_simulation()
