# -*- coding: utf-8 -*-
"""
反步法非线性控制仿真
===================
Backstepping控制设计用于非线性系统
应用场景: 永磁同步电机(PMSM)速度控制
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
    dt = 0.0001    # 100us 步长
    T = 0.5        # 仿真时长
    t = np.arange(0, T, dt)
    N = len(t)

    # PMSM参数
    J = 0.001     # 转动惯量
    B = 0.005     # 粘滞摩擦
    Kt = 1.5      # 转矩常数
    Tl = 0.5      # 负载转矩

    # ========== 参考信号 ==========
    w_ref = np.ones(N) * 100.0  # 期望转速 100 rad/s
    w_ref[t > 0.25] = 150.0     # 250ms后变为150

    # ========== 反步法控制器设计 ==========
    # 系统: J*dw/dt = -B*w + Kt*iq - Tl
    # 定义状态: x1 = w (转速), x2 = iq (q轴电流)
    # 电流环简化为一阶: diq/dt = (-R*iq + u) / L
    R_s = 1.0     # 电阻
    L_s = 0.005   # 电感

    # 反步法增益
    k1 = 50.0
    k2 = 100.0

    x1 = np.zeros(N)  # 转速
    x2 = np.zeros(N)  # 电流
    u_bs = np.zeros(N)  # 控制电压

    x1[0] = 0.0
    x2[0] = 0.0

    # 扰动
    dist = np.zeros(N)
    dist[int(0.3/dt):int(0.35/dt)] = 2.0  # 负载扰动

    for i in range(N-1):
        e1 = w_ref[i] - x1[i]

        # Step1: 虚拟控制 alpha1 (期望电流)
        # J*x1_dot = -B*x1 + Kt*x2 - Tl
        # 令误差动态: e1_dot = -k1*e1
        # => alpha1 = (J*(-k1*e1) + B*x1 + Tl) / Kt
        alpha1 = (J * k1 * e1 + B * x1[i] + Tl + dist[i]) / Kt

        # Step2: 电流误差
        e2 = x2[i] - alpha1

        # 反步法控制律: 使e2收敛
        # L_s * x2_dot = -R_s * x2 + u
        # 令 e2_dot = -k2 * e2
        # => u = L_s * (-k2*e2) + R_s*x2[i]
        u = L_s * (-k2 * e2) + R_s * x2[i]
        u = np.clip(u, -50, 50)
        u_bs[i] = u

        # 状态更新
        x1_dot = (-B*x1[i] + Kt*x2[i] - Tl - dist[i]) / J
        x2_dot = (-R_s*x2[i] + u) / L_s

        x1[i+1] = x1[i] + x1_dot * dt
        x2[i+1] = x2[i] + x2_dot * dt

    # ========== PID对比 ==========
    Kp, Ki, Kd = 0.5, 20.0, 0.001
    x1_pid = np.zeros(N)
    x2_pid = np.zeros(N)
    u_pid = np.zeros(N)
    e_int = 0
    e_prev = 0

    for i in range(N-1):
        e = w_ref[i] - x1_pid[i]
        e_int += e * dt
        e_int = np.clip(e_int, -10, 10)
        e_d = (e - e_prev) / dt

        iq_ref = Kp * e + Ki * e_int + Kd * e_d
        u = (iq_ref - x2_pid[i]) * 100  # 简单电流环
        u = np.clip(u, -50, 50)
        u_pid[i] = u

        x1_dot = (-B*x1_pid[i] + Kt*x2_pid[i] - Tl - dist[i]) / J
        x2_dot = (-R_s*x2_pid[i] + u) / L_s
        x1_pid[i+1] = x1_pid[i] + x1_dot * dt
        x2_pid[i+1] = x2_pid[i] + x2_dot * dt
        e_prev = e

    # ========== 性能指标 ==========
    def metrics(y, ref, t, t0=0.0):
        idx = t >= t0
        err = ref[idx] - y[idx]
        return {
            'IAE': np.sum(np.abs(err)) * dt,
            'ISE': np.sum(err**2) * dt,
            '最大误差': np.max(np.abs(err))
        }

    m_bs = metrics(x1, w_ref, t)
    m_pid = metrics(x1_pid, w_ref, t)

    print("=== 反步法控制性能 ===")
    for k, v in m_bs.items():
        print(f"  {k}: {v:.4f}")
    print("\n=== PID控制性能 ===")
    for k, v in m_pid.items():
        print(f"  {k}: {v:.4f}")

    # ========== 绘图 ==========
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))

    axes[0].plot(t*1000, w_ref, 'k--', linewidth=1.5, label='参考转速')
    axes[0].plot(t*1000, x1, 'r-', linewidth=1.0, label='反步法')
    axes[0].plot(t*1000, x1_pid, 'b-', linewidth=1.0, label='PID')
    axes[0].set_ylabel('转速 (rad/s)')
    axes[0].set_title('反步法 vs PID 速度跟踪对比')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t*1000, x2, 'r-', linewidth=0.8, label='反步法 iq')
    axes[1].plot(t*1000, x2_pid, 'b-', linewidth=0.8, label='PID iq')
    axes[1].set_ylabel('q轴电流 (A)')
    axes[1].set_title('电流响应对比')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(t*1000, u_bs, 'r-', linewidth=0.6, label='反步法')
    axes[2].plot(t*1000, u_pid, 'b-', linewidth=0.6, label='PID')
    axes[2].set_xlabel('时间 (ms)')
    axes[2].set_ylabel('控制电压 (V)')
    axes[2].set_title('控制量对比')
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'backstepping_nonlinear.png')
    plt.savefig(out, dpi=150)
    plt.close()
    print(f"\n已保存: {out}")



if __name__ == '__main__':
    main()
