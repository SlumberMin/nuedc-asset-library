#!/usr/bin/env python3
"""
状态反馈控制仿真 - 极点配置法

仿真内容:
  1. 二阶系统极点配置
  2. 观测器+状态反馈控制器
  3. 对比: 开环 vs 状态反馈 vs PID

系统模型: 直流电机速度控制
  状态方程: x_dot = A*x + B*u, y = C*x
  x = [theta, omega]^T (角度, 角速度)
  A = [0, 1; 0, -b/J], B = [0; Kt/J], C = [1, 0]
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
plt.rcParams['axes.unicode_minus'] = False


def simulate_state_feedback(A, B, C, K, N_ref, x0, ref, dt, steps):
    """状态反馈控制仿真"""
    n = A.shape[0]
    m = B.shape[1]
    x = x0.copy()
    x_hist = np.zeros((steps, n))
    y_hist = np.zeros(steps)
    u_hist = np.zeros((steps, m))

    for k in range(steps):
        y = C @ x
        u = -K @ x + N_ref * ref[k]
        x = x + dt * (A @ x + B @ u)

        x_hist[k] = x
        y_hist[k] = y[0] if y.ndim > 0 else y
        u_hist[k] = u.flatten()

    return x_hist, y_hist, u_hist


def simulate_observer_controller(A, B, C, K, L, N_ref, x0, x_hat0,
                                  ref, y_meas, dt, steps):
    """观测器+状态反馈仿真"""
    n = A.shape[0]
    m = B.shape[1]
    p = C.shape[0]
    x = x0.copy()
    x_hat = x_hat0.copy()

    x_hist = np.zeros((steps, n))
    x_hat_hist = np.zeros((steps, n))
    y_hist = np.zeros(steps)
    u_hist = np.zeros((steps, m))

    for k in range(steps):
        y = C @ x
        # 加入测量噪声
        y_noisy = y + np.random.randn(p) * y_meas

        u = -K @ x_hat + N_ref * ref[k]

        # 观测器更新
        e_y = y_noisy - C @ x_hat
        x_hat = x_hat + dt * (A @ x_hat + B @ u + L @ e_y)

        # 系统状态更新
        x = x + dt * (A @ x + B @ u)

        x_hist[k] = x
        x_hat_hist[k] = x_hat
        y_hist[k] = y[0] if y.ndim > 0 else y
        u_hist[k] = u.flatten()

    return x_hist, x_hat_hist, y_hist, u_hist


def simulate_open_loop(A, B, C, x0, u_const, dt, steps):
    """开环仿真"""
    n = A.shape[0]
    x = x0.copy()
    x_hist = np.zeros((steps, n))
    y_hist = np.zeros(steps)

    for k in range(steps):
        y = C @ x
        x = x + dt * (A @ x + B * u_const)

        x_hist[k] = x
        y_hist[k] = y[0] if y.ndim > 0 else y

    return x_hist, y_hist


def simulate_pid(Kp, Ki, Kd, ref, dt, steps, plant_A, plant_B, plant_C, x0):
    """PID控制仿真(用于对比)"""
    n = plant_A.shape[0]
    x = x0.copy()
    integral = 0.0
    prev_err = 0.0
    y_hist = np.zeros(steps)
    u_hist = np.zeros(steps)

    for k in range(steps):
        y = (plant_C @ x)[0]
        err = ref[k] - y
        integral += err * dt
        derivative = (err - prev_err) / dt
        u = Kp * err + Ki * integral + Kd * derivative
        prev_err = err

        x = x + dt * (plant_A @ x + plant_B.flatten() * u)
        y_hist[k] = y
        u_hist[k] = u

    return y_hist, u_hist


if __name__ == "__main__":
    # ============ 直流电机模型参数 ============
    J = 0.01     # 转动惯量 (kg·m²)
    b = 0.1      # 阻尼系数
    Kt = 0.01    # 电机转矩常数
    Ke = 0.01    # 反电动势常数
    R = 1.0      # 电阻
    V = 12.0     # 额定电压

    dt = 0.001   # 采样周期 1ms
    T = 2.0      # 仿真时长
    steps = int(T / dt)
    t = np.arange(steps) * dt

    # 状态空间模型: x = [theta, omega]
    A = np.array([[0, 1],
                  [0, -b / J]])
    B = np.array([[0],
                  [Kt / J]])
    C = np.array([[1, 0]])
    x0 = np.array([0.0, 0.0])

    # 参考信号: 阶跃 + 正弦
    ref_step = np.ones(steps)
    ref_sine = np.sin(2 * np.pi * 0.5 * t)  # 0.5Hz正弦

    # ============ 极点配置 ============
    # 期望闭环极点: -10 ± 10j (阻尼比0.707, 自然频率14.14 rad/s)
    desired_p1 = -10.0
    desired_p2 = -15.0

    # 利用可控性矩阵计算K (手动求解二阶系统)
    # A_cl = A - B*K, 特征多项式 = (s-p1)(s-p2)
    a11, a12 = A[0, 0], A[0, 1]
    a21, a22 = A[1, 0], A[1, 1]
    b1, b2 = B[0, 0], B[1, 0]

    # 期望特征多项式系数
    alpha1 = -(desired_p1 + desired_p2)  # = 25
    alpha0 = desired_p1 * desired_p2      # = 150

    # 可控标准型变换求K
    # 原系统特征多项式: s^2 + (b/J)*s
    a1_orig = b / J
    a0_orig = 0

    # K = [(alpha0 - a0_orig)/(b2), (alpha1 - a1_orig - a21/b2*(alpha0-a0_orig))/???]
    # 用 Ackermann 公式更简单:
    # phi(A) = A^2 + alpha1*A + alpha0*I
    A2 = A @ A
    phi = A2 + alpha1 * A + alpha0 * np.eye(2)

    # 可控性矩阵
    M = np.hstack([B, A @ B])
    M_inv = np.linalg.inv(M)

    # K = [0, 1] @ M_inv @ phi
    e = np.array([[0, 1]])
    K = e @ M_inv @ phi
    K = np.array(K).flatten().reshape(1, 2)

    print(f"状态反馈增益 K = [{K[0,0]:.4f}, {K[0,1]:.4f}]")

    # 前馈增益(稳态无差)
    A_cl = A - B @ K
    N_ref = -1.0 / (C @ np.linalg.inv(A_cl) @ B)[0, 0]
    print(f"前馈增益 N = {N_ref:.4f}")

    # ============ 仿真1: 阶跃响应对比 ============
    x_ol, y_ol = simulate_open_loop(A, B, C, x0, V, dt, steps)
    x_sf, y_sf, u_sf = simulate_state_feedback(A, B, C, K, N_ref, x0,
                                                 ref_step * 1.0, dt, steps)
    y_pid, u_pid = simulate_pid(10.0, 50.0, 0.5, ref_step, dt, steps, A, B, C, x0)

    # ============ 仿真2: 观测器+控制器 ============
    L = np.array([[50, 0],
                  [0, 200]])
    x_oc, x_hat_oc, y_oc, u_oc = simulate_observer_controller(
        A, B, C, K, L, N_ref, x0, np.array([0.0, 0.0]),
        ref_step * 1.0, 0.001, dt, steps)

    # ============ 仿真3: 正弦跟踪 ============
    x_sf_s, y_sf_s, u_sf_s = simulate_state_feedback(
        A, B, C, K, N_ref, x0, ref_sine, dt, steps)

    # ============ 绘图 ============
    fig, axes = plt.subplots(3, 2, figsize=(14, 10))

    # 图1: 阶跃响应对比
    axes[0, 0].plot(t, ref_step, 'k--', label='参考', linewidth=1)
    axes[0, 0].plot(t, y_ol, 'r', label='开环', alpha=0.7)
    axes[0, 0].plot(t, y_sf, 'b', label='状态反馈', linewidth=2)
    axes[0, 0].plot(t, y_pid, 'g', label='PID', alpha=0.7)
    axes[0, 0].set_title('阶跃响应对比')
    axes[0, 0].set_xlabel('时间 (s)')
    axes[0, 0].set_ylabel('输出')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)

    # 图2: 控制量对比
    axes[0, 1].plot(t, u_sf[:, 0], 'b', label='状态反馈', linewidth=1)
    axes[0, 1].plot(t, u_pid, 'g', label='PID', alpha=0.7)
    axes[0, 1].set_title('控制量对比')
    axes[0, 1].set_xlabel('时间 (s)')
    axes[0, 1].set_ylabel('u')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)

    # 图3: 观测器+控制器
    axes[1, 0].plot(t, ref_step, 'k--', label='参考', linewidth=1)
    axes[1, 0].plot(t, y_oc, 'b', label='观测器+状态反馈')
    axes[1, 0].set_title('观测器+状态反馈控制')
    axes[1, 0].set_xlabel('时间 (s)')
    axes[1, 0].set_ylabel('输出')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)

    # 图4: 状态估计误差
    axes[1, 1].plot(t, x[:, 0] - x_hat_oc[:, 0], 'r', label='θ估计误差')
    axes[1, 1].plot(t, x[:, 1] - x_hat_oc[:, 1], 'b', label='ω估计误差')
    axes[1, 1].set_title('观测器状态估计误差')
    axes[1, 1].set_xlabel('时间 (s)')
    axes[1, 1].set_ylabel('误差')
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3)

    # 图5: 正弦跟踪
    axes[2, 0].plot(t, ref_sine, 'k--', label='参考正弦', linewidth=1)
    axes[2, 0].plot(t, y_sf_s, 'b', label='状态反馈', linewidth=2)
    axes[2, 0].set_title('正弦信号跟踪')
    axes[2, 0].set_xlabel('时间 (s)')
    axes[2, 0].set_ylabel('输出')
    axes[2, 0].legend()
    axes[2, 0].grid(True, alpha=0.3)

    # 图6: 极点位置
    axes[2, 1].plot(np.real([desired_p1, desired_p2]),
                     np.imag([desired_p1, desired_p2]),
                     'rx', markersize=15, markeredgewidth=3, label='期望极点')
    # 闭环极点
    eig_cl = np.linalg.eig(A_cl)[0]
    axes[2, 1].plot(np.real(eig_cl), np.imag(eig_cl),
                     'bo', markersize=10, label='闭环极点')
    # 开环极点
    eig_ol = np.linalg.eig(A)[0]
    axes[2, 1].plot(np.real(eig_ol), np.imag(eig_ol),
                     'gs', markersize=10, label='开环极点')
    axes[2, 1].axhline(0, color='k', linewidth=0.5)
    axes[2, 1].axvline(0, color='k', linewidth=0.5)
    axes[2, 1].set_title('极点配置')
    axes[2, 1].set_xlabel('实部')
    axes[2, 1].set_ylabel('虚部')
    axes[2, 1].legend()
    axes[2, 1].grid(True, alpha=0.3)

    # 修正观测器仿真中的引用
    x = x0.copy()
    x_hat = np.array([0.0, 0.0])
    x_hist = np.zeros((steps, 2))
    for k in range(steps):
        y = (C @ x)[0]
        y_noisy = y + np.random.randn() * 0.001
        u = -(K @ x_hat)[0] + N_ref
        e_y = y_noisy - (C @ x_hat)[0]
        x_hat = x_hat + dt * (A @ x_hat + B.flatten() * u + L[:, 0] * e_y)
        x = x + dt * (A @ x + B.flatten() * u)
        x_hist[k] = x

    # 重新绘制图4
    axes[1, 1].clear()
    axes[1, 1].plot(t, x_hist[:, 0] - x_hat_oc[:, 0], 'r', label='θ估计误差')
    axes[1, 1].set_title('观测器状态估计误差')
    axes[1, 1].set_xlabel('时间 (s)')
    axes[1, 1].set_ylabel('误差')
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3)

    plt.suptitle('状态反馈控制仿真 (极点配置法)', fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig('state_feedback_simulation.png', dpi=150, bbox_inches='tight')
    plt.close('all')
    print("仿真完成! 图片已保存为 state_feedback_simulation.png")
