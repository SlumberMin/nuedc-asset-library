# -*- coding: utf-8 -*-
"""
串级控制对比仿真
===============
对比单环PID与串级PID控制的性能差异
应用场景: 电机速度-电流双闭环控制
"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False


def calc_metrics(y, ref, t, dt, t0=0.0):
    """计算控制性能指标"""
    idx = t >= t0
    err = ref[idx] - y[idx]
    final_val = ref[idx][-1]
    overshoot = (np.max(y[idx]) - final_val) / abs(final_val) * 100 if final_val > 0 else 0
    return {
        'IAE': np.sum(np.abs(err)) * dt,
        'ISE': np.sum(err ** 2) * dt,
        '超调量%': overshoot,
        '最大误差': np.max(np.abs(err))
    }


def run_simulation(dt=0.0001, T=0.5, save_path=None):
    """
    运行串级控制对比仿真

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

    # 电机模型参数
    J = 0.001       # 转动惯量
    B_f = 0.005     # 摩擦系数
    Kt = 1.5        # 转矩常数
    R_s = 1.0       # 定子电阻
    L_s = 0.005     # 定子电感

    # 参考转速
    w_ref = np.ones(N) * 100.0
    w_ref[t > 0.2] = 150.0

    # 扰动
    dist = np.zeros(N)
    dist[int(0.3 / dt):] = 1.0  # 300ms后加负载

    # ========== 方案1: 单环PID (速度环) ==========
    Kp1, Ki1, Kd1 = 2.0, 50.0, 0.01
    w_single = np.zeros(N)
    iq_single = np.zeros(N)
    u_single = np.zeros(N)
    ei1 = 0
    ed1_prev = 0

    for i in range(N - 1):
        e = w_ref[i] - w_single[i]
        ei1 += e * dt
        ei1 = np.clip(ei1, -10, 10)
        ed1 = (e - ed1_prev) / dt if dt > 0 else 0

        # 单环PID直接输出电压
        u = Kp1 * e + Ki1 * ei1 + Kd1 * ed1
        u = np.clip(u, -50, 50)
        u_single[i] = u

        # 电流环简化为比例
        iq_cmd = u
        iq_single[i] = iq_cmd

        # 系统更新
        w_dot = (-B_f * w_single[i] + Kt * iq_single[i] - dist[i]) / J
        w_single[i + 1] = w_single[i] + w_dot * dt
        ed1_prev = e

    # ========== 方案2: 串级PID (速度外环 + 电流内环) ==========
    Kp_out, Ki_out, Kd_out = 0.5, 10.0, 0.001
    Kp_in, Ki_in = 100.0, 5000.0

    w_cascade = np.zeros(N)
    iq_cascade = np.zeros(N)
    u_cascade = np.zeros(N)
    ei_out = 0
    ed_out_prev = 0
    ei_in = 0

    for i in range(N - 1):
        # 外环: 速度 -> 电流指令
        e_out = w_ref[i] - w_cascade[i]
        ei_out += e_out * dt
        ei_out = np.clip(ei_out, -5, 5)
        ed_out = (e_out - ed_out_prev) / dt if dt > 0 else 0
        iq_cmd = Kp_out * e_out + Ki_out * ei_out + Kd_out * ed_out
        iq_cmd = np.clip(iq_cmd, -20, 20)

        # 内环: 电流 -> 电压
        e_in = iq_cmd - iq_cascade[i]
        ei_in += e_in * dt
        ei_in = np.clip(ei_in, -50, 50)
        u = Kp_in * e_in + Ki_in * ei_in
        u = np.clip(u, -50, 50)
        u_cascade[i] = u

        # 电流回路更新 (电感效应)
        iq_dot = (-R_s * iq_cascade[i] + u) / L_s
        iq_cascade[i + 1] = iq_cascade[i] + iq_dot * dt

        # 速度更新
        w_dot = (-B_f * w_cascade[i] + Kt * iq_cascade[i] - dist[i]) / J
        w_cascade[i + 1] = w_cascade[i] + w_dot * dt

        ed_out_prev = e_out

    # ========== 性能指标 ==========
    m_single = calc_metrics(w_single, w_ref, t, dt)
    m_cascade = calc_metrics(w_cascade, w_ref, t, dt)

    print("=== 单环PID性能 ===")
    for k, v in m_single.items():
        print(f"  {k}: {v:.4f}")
    print("\n=== 串级PID性能 ===")
    for k, v in m_cascade.items():
        print(f"  {k}: {v:.4f}")

    # ========== 绘图 ==========
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))

    axes[0].plot(t * 1000, w_ref, 'k--', linewidth=1.5, label='参考转速')
    axes[0].plot(t * 1000, w_single, 'b-', linewidth=1.0, label='单环PID')
    axes[0].plot(t * 1000, w_cascade, 'r-', linewidth=1.0, label='串级PID')
    axes[0].set_ylabel('转速 (rad/s)')
    axes[0].set_title('单环PID vs 串级PID 速度控制对比')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t * 1000, iq_single, 'b-', linewidth=0.6, label='单环PID电流')
    axes[1].plot(t * 1000, iq_cascade, 'r-', linewidth=0.6, label='串级PID电流')
    axes[1].set_ylabel('q轴电流 (A)')
    axes[1].set_title('电流响应对比')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(t * 1000, u_single, 'b-', linewidth=0.5, alpha=0.7, label='单环PID')
    axes[2].plot(t * 1000, u_cascade, 'r-', linewidth=0.5, alpha=0.7, label='串级PID')
    axes[2].set_xlabel('时间 (ms)')
    axes[2].set_ylabel('控制电压 (V)')
    axes[2].set_title('控制电压对比')
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    if save_path is None:
        save_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                 'cascade_control_comparison.png')
    plt.savefig(save_path, dpi=150)
    plt.close()
    print(f"\n已保存: {save_path}")


if __name__ == '__main__':
    run_simulation()
