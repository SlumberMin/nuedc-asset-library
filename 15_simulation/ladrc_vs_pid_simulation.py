# -*- coding: utf-8 -*-
"""
LADRC vs PID 对比仿真
====================
比较线性自抗扰控制(LADRC)与传统PID的跟踪与抗扰性能
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
    dt = 0.001          # 仿真步长 1ms
    T = 5.0             # 仿真总时长 5s
    t = np.arange(0, T, dt)
    N = len(t)

    # 被控对象: G(s) = 1 / (s^2 + 2s + 1)  二阶欠阻尼系统
    # 状态空间: x1_dot = x2, x2_dot = -x1 - 2*x2 + u
    a1, a2 = 1.0, 2.0

    # ========== 参考信号 ==========
    ref = np.ones(N)  # 阶跃信号
    ref[t > 2.5] = 1.5  # 2.5s后变为1.5

    # 扰动
    disturbance = np.zeros(N)
    disturbance[(t > 1.5) & (t < 2.0)] = 0.3  # 脉冲扰动
    disturbance[t > 3.5] = 0.15  # 阶跃扰动

    # ========== PID控制器 ==========
    Kp, Ki, Kd = 50.0, 100.0, 10.0
    x_pid = np.zeros(2)
    y_pid = np.zeros(N)
    u_pid = np.zeros(N)
    e_int_pid = 0
    e_prev_pid = 0

    for i in range(N):
        y_pid[i] = x_pid[0]
        e = ref[i] - x_pid[0]
        e_int_pid += e * dt
        e_d = (e - e_prev_pid) / dt
        # 积分抗饱和
        e_int_pid = np.clip(e_int_pid, -5, 5)
        u = Kp * e + Ki * e_int_pid + Kd * e_d
        u_pid[i] = u
        # 状态更新 (欧拉法)
        x1_dot = x_pid[1]
        x2_dot = -a1 * x_pid[0] - a2 * x_pid[1] + u + disturbance[i]
        x_pid[0] += x1_dot * dt
        x_pid[1] += x2_dot * dt
        e_prev_pid = e

    # ========== LADRC控制器 ==========
    # LADRC: LESO + 状态误差反馈
    wc = 100   # 控制器带宽
    wo = 100   # 观测器带宽
    b0 = 1.0   # 系统增益估计

    # LADRC状态
    x_lad = np.zeros(2)
    z1, z2, z3 = 0.0, 0.0, 0.0  # LESO状态
    y_lad = np.zeros(N)
    u_lad = np.zeros(N)

    # LESO增益 (三阶)
    beta1 = 3 * wo
    beta2 = 3 * wo**2
    beta3 = wo**3

    for i in range(N):
        y_lad[i] = x_lad[0]
        # 观测误差
        e_obs = z1 - x_lad[0]
        # LESO更新
        z1 += (z2 - beta1 * e_obs) * dt
        z2 += (z3 - beta2 * e_obs + b0 * u_lad[max(0, i-1)]) * dt
        z3 += (-beta3 * e_obs) * dt

        # 控制律: PD + 扰动补偿
        e_lad = ref[i] - z1
        e_dot = -z2  # 期望导数为0时
        u0 = wc**2 * e_lad + 2 * wc * e_dot
        u = (u0 - z3) / b0
        u = np.clip(u, -50, 50)
        u_lad[i] = u

        # 被控对象更新
        x1_dot = x_lad[1]
        x2_dot = -a1 * x_lad[0] - a2 * x_lad[1] + u + disturbance[i]
        x_lad[0] += x1_dot * dt
        x_lad[1] += x2_dot * dt

    # ========== 性能指标 ==========
    def calc_metrics(y, ref, t, t_start=0.0):
        """计算性能指标"""
        idx = t >= t_start
        y_s, r_s, t_s = y[idx], ref[idx], t[idx]
        err = r_s - y_s

        # 上升时间 (10%~90%)
        final_val = r_s[-1]
        t_10 = t_s[np.where(y_s >= 0.1 * final_val)[0][0]] if np.any(y_s >= 0.1 * final_val) else t_s[-1]
        t_90 = t_s[np.where(y_s >= 0.9 * final_val)[0][0]] if np.any(y_s >= 0.9 * final_val) else t_s[-1]
        t_rise = t_90 - t_10

        # 超调量
        overshoot = (np.max(y_s) - final_val) / abs(final_val) * 100 if final_val != 0 else 0

        # 调节时间 (±2%)
        settled = np.abs(err) < 0.02 * abs(final_val) if final_val != 0 else np.abs(err) < 0.02
        t_settle = t_s[-1]
        for j in range(len(t_s)-1, -1, -1):
            if not settled[j]:
                t_settle = t_s[min(j+1, len(t_s)-1)]
                break

        # IAE / ISE
        iae = np.sum(np.abs(err)) * dt
        ise = np.sum(err**2) * dt

        return {'上升时间': t_rise, '超调量%': overshoot, '调节时间': t_settle, 'IAE': iae, 'ISE': ise}

    m_pid = calc_metrics(y_pid, ref, t)
    m_lad = calc_metrics(y_lad, ref, t)

    print("=== PID 性能指标 ===")
    for k, v in m_pid.items():
        print(f"  {k}: {v:.4f}")
    print("\n=== LADRC 性能指标 ===")
    for k, v in m_lad.items():
        print(f"  {k}: {v:.4f}")

    # ========== 绘图 ==========
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))

    # 跟踪对比
    axes[0].plot(t, ref, 'k--', linewidth=1.5, label='参考信号')
    axes[0].plot(t, y_pid, 'b-', linewidth=1.2, label='PID')
    axes[0].plot(t, y_lad, 'r-', linewidth=1.2, label='LADRC')
    axes[0].set_ylabel('输出')
    axes[0].set_title('LADRC vs PID 阶跃跟踪对比')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    # 控制量
    axes[1].plot(t, u_pid, 'b-', linewidth=0.8, label='PID控制量')
    axes[1].plot(t, u_lad, 'r-', linewidth=0.8, label='LADRC控制量')
    axes[1].set_ylabel('控制量 u')
    axes[1].set_title('控制量对比')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    # 误差
    axes[2].plot(t, ref - y_pid, 'b-', linewidth=0.8, label='PID误差')
    axes[2].plot(t, ref - y_lad, 'r-', linewidth=0.8, label='LADRC误差')
    axes[2].set_xlabel('时间 (s)')
    axes[2].set_ylabel('跟踪误差')
    axes[2].set_title('跟踪误差对比')
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'ladrc_vs_pid_simulation.png')
    plt.savefig(out, dpi=150)
    plt.close()
    print(f"\n已保存: {out}")



if __name__ == '__main__':
    main()
