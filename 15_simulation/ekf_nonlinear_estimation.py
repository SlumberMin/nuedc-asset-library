# -*- coding: utf-8 -*-
"""
EKF 非线性状态估计仿真
=====================
使用扩展卡尔曼滤波器估计非线性系统的状态
应用场景: 倒立摆角度/角速度估计
"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os



def main():
    plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False

    # ========== 系统参数 ==========
    dt = 0.01
    T = 10.0
    t = np.arange(0, T, dt)
    N = len(t)
    g = 9.81
    L = 1.0   # 摆长
    b = 0.1   # 阻尼

    # ========== 真实系统 (非线性倒立摆) ==========
    # 状态: [theta, omega]
    x_true = np.zeros((N, 2))
    x_true[0] = [0.5, 0.0]  # 初始角度0.5rad

    process_noise = np.random.randn(N, 2) * 0.01
    meas_noise = np.random.randn(N) * 0.05

    # 输入力矩
    u_input = 0.5 * np.sin(2 * np.pi * 0.5 * t)  # 正弦力矩

    # 真实状态演化
    for i in range(N-1):
        theta, omega = x_true[i]
        theta_dot = omega
        omega_dot = -(g/L)*np.sin(theta) - b*omega + u_input[i]/(L**2)
        x_true[i+1, 0] = x_true[i, 0] + theta_dot * dt + process_noise[i, 0] * dt
        x_true[i+1, 1] = x_true[i, 1] + omega_dot * dt + process_noise[i, 1] * dt

    # 观测: 只能观测角度 + 噪声
    z_meas = x_true[:, 0] + meas_noise

    # ========== EKF ==========
    def f_func(x, u):
        """状态转移函数 (非线性)"""
        theta, omega = x
        theta_dot = omega
        omega_dot = -(g/L)*np.sin(theta) - b*omega + u/(L**2)
        return np.array([theta + theta_dot*dt, omega + omega_dot*dt])

    def F_jacobian(x, u):
        """状态转移雅可比矩阵"""
        theta, omega = x
        F = np.array([
            [1.0, dt],
            [-(g/L)*np.cos(theta)*dt, 1.0 - b*dt]
        ])
        return F

    def h_func(x):
        """观测函数"""
        return np.array([x[0]])

    def H_jacobian(x):
        """观测雅可比矩阵"""
        return np.array([[1.0, 0.0]])

    # EKF初始化
    x_est = np.zeros((N, 2))
    x_est[0] = [0.3, 0.1]  # 初始估计有偏差
    P = np.eye(2) * 0.1
    Q = np.diag([1e-4, 1e-4])  # 过程噪声协方差
    R = np.array([[0.05**2]])   # 观测噪声协方差

    for i in range(N-1):
        # ---- 预测 ----
        x_pred = f_func(x_est[i], u_input[i])
        F = F_jacobian(x_est[i], u_input[i])
        P_pred = F @ P @ F.T + Q

        # ---- 更新 ----
        H = H_jacobian(x_pred)
        y_res = z_meas[i+1] - h_func(x_pred)  # 新息
        S = H @ P_pred @ H.T + R              # 新息协方差 (1x1)
        K = P_pred @ H.T * (1.0 / S[0, 0])   # 卡尔曼增益 (S为1x1，直接求倒)

        x_est[i+1] = x_pred + (K @ y_res).flatten()
        P = (np.eye(2) - K @ H) @ P_pred

    # ========== 性能指标 ==========
    rmse_theta = np.sqrt(np.mean((x_est[:, 0] - x_true[:, 0])**2))
    rmse_omega = np.sqrt(np.mean((x_est[:, 1] - x_true[:, 1])**2))
    rmse_meas = np.sqrt(np.mean((z_meas - x_true[:, 0])**2))

    print("=== EKF 状态估计性能 ===")
    print(f"  角度 RMSE (原始测量): {rmse_meas:.4f} rad")
    print(f"  角度 RMSE (EKF估计):  {rmse_theta:.4f} rad")
    print(f"  角速度 RMSE (EKF):    {rmse_omega:.4f} rad/s")
    print(f"  角度估计改善:         {(1-rmse_theta/rmse_meas)*100:.1f}%")

    # ========== 绘图 ==========
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))

    # 角度估计
    axes[0].plot(t, x_true[:, 0], 'k-', linewidth=1.5, label='真实角度')
    axes[0].plot(t, z_meas, 'g.', markersize=1, alpha=0.3, label='含噪测量')
    axes[0].plot(t, x_est[:, 0], 'r-', linewidth=1.2, label='EKF估计')
    axes[0].set_ylabel('角度 (rad)')
    axes[0].set_title('EKF 非线性状态估计 — 角度')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    # 角速度估计 (无直接测量)
    axes[1].plot(t, x_true[:, 1], 'k-', linewidth=1.5, label='真实角速度')
    axes[1].plot(t, x_est[:, 1], 'r-', linewidth=1.2, label='EKF估计')
    axes[1].set_ylabel('角速度 (rad/s)')
    axes[1].set_title('EKF 非线性状态估计 — 角速度 (无直接测量)')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    # 估计误差
    axes[2].plot(t, x_est[:, 0] - x_true[:, 0], 'b-', linewidth=0.8, label='角度误差')
    axes[2].plot(t, x_est[:, 1] - x_true[:, 1], 'm-', linewidth=0.8, label='角速度误差')
    axes[2].set_xlabel('时间 (s)')
    axes[2].set_ylabel('估计误差')
    axes[2].set_title('EKF 估计误差')
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'ekf_nonlinear_estimation.png')
    plt.savefig(out, dpi=150)
    plt.close()
    print(f"\n已保存: {out}")



if __name__ == '__main__':
    main()
