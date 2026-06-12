# -*- coding: utf-8 -*-
"""
增益调度自适应仿真
================
根据运行工况自动调整PID参数的增益调度控制
应用场景: 电机不同转速段需要不同PID参数
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
    dt = 0.001
    T = 5.0
    t = np.arange(0, T, dt)
    N = len(t)

    # 非线性系统: 参数随工作点变化
    # 模拟电机在不同转速下的参数变化
    def system_params(w):
        """根据转速返回系统参数 (模拟参数变化)"""
        w_abs = abs(w) + 1.0
        J_eff = 0.001 + 0.0005 * (w_abs / 100)  # 惯量随转速增大
        B_eff = 0.005 + 0.001 * (w_abs / 100)   # 摩擦随转速增大
        Kt_eff = 1.5 - 0.001 * w_abs             # 转矩常数随转速下降
        return J_eff, B_eff, Kt_eff

    # 参考信号: 从低速到高速变化
    w_ref = np.concatenate([
        np.ones(N//5) * 50,
        np.ones(N//5) * 100,
        np.ones(N//5) * 200,
        np.ones(N//5) * 300,
        np.ones(N//5) * 150
    ])[:N]

    # ========== 增益调度PID ==========
    def gain_schedule(w):
        """根据转速调度PID参数"""
        w_abs = abs(w)
        if w_abs < 75:
            # 低速段: 高增益, 快响应
            return 1.5, 80.0, 0.05
        elif w_abs < 150:
            # 中速段: 中增益
            return 1.0, 50.0, 0.03
        elif w_abs < 250:
            # 高速段: 低增益, 避免振荡
            return 0.6, 30.0, 0.02
        else:
            # 超高速段: 最低增益
            return 0.4, 20.0, 0.01

    # 固定PID参数 (取中值)
    Kp_fixed = 0.8
    Ki_fixed = 40.0
    Kd_fixed = 0.03

    # ========== 仿真: 增益调度 vs 固定PID ==========
    w_gs = np.zeros(N)
    u_gs = np.zeros(N)
    ei_gs = 0; ed_prev_gs = 0
    params_log = np.zeros((N, 3))  # 记录参数变化

    w_fixed = np.zeros(N)
    u_fixed = np.zeros(N)
    ei_fx = 0; ed_prev_fx = 0

    for i in range(N-1):
        # --- 增益调度PID ---
        Kp_g, Ki_g, Kd_g = gain_schedule(w_ref[i])
        params_log[i] = [Kp_g, Ki_g, Kd_g]

        e_gs = w_ref[i] - w_gs[i]
        ei_gs += e_gs * dt
        ei_gs = np.clip(ei_gs, -10, 10)
        ed_gs = (e_gs - ed_prev_gs) / dt
        u_g = Kp_g * e_gs + Ki_g * ei_gs + Kd_g * ed_gs
        u_g = np.clip(u_g, -100, 100)
        u_gs[i] = u_g

        # --- 固定PID ---
        e_fx = w_ref[i] - w_fixed[i]
        ei_fx += e_fx * dt
        ei_fx = np.clip(ei_fx, -10, 10)
        ed_fx = (e_fx - ed_prev_fx) / dt
        u_f = Kp_fixed * e_fx + Ki_fixed * ei_fx + Kd_fixed * ed_fx
        u_f = np.clip(u_f, -100, 100)
        u_fixed[i] = u_f

        # 系统更新 (使用变参数)
        J_g, B_g, Kt_g = system_params(w_gs[i])
        w_gs_dot = (-B_g * w_gs[i] + Kt_g * u_g) / J_g
        w_gs[i+1] = w_gs[i] + w_gs_dot * dt

        J_f, B_f, Kt_f = system_params(w_fixed[i])
        w_fx_dot = (-B_f * w_fixed[i] + Kt_f * u_f) / J_f
        w_fixed[i+1] = w_fixed[i] + w_fx_dot * dt

        ed_prev_gs = e_gs
        ed_prev_fx = e_fx

    # ========== 性能指标 ==========
    def calc_metrics(y, ref, t):
        err = ref - y
        iae = np.sum(np.abs(err)) * dt
        ise = np.sum(err**2) * dt
        max_err = np.max(np.abs(err))
        return {'IAE': iae, 'ISE': ise, '最大误差': max_err}

    m_gs = calc_metrics(w_gs, w_ref, t)
    m_fx = calc_metrics(w_fixed, w_ref, t)

    print("=== 增益调度PID性能 ===")
    for k, v in m_gs.items():
        print(f"  {k}: {v:.4f}")
    print("\n=== 固定PID性能 ===")
    for k, v in m_fx.items():
        print(f"  {k}: {v:.4f}")
    print(f"\nIAE改善: {(1-m_gs['IAE']/m_fx['IAE'])*100:.1f}%")

    # ========== 绘图 ==========
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))

    axes[0].plot(t, w_ref, 'k--', linewidth=1.5, label='参考转速')
    axes[0].plot(t, w_gs, 'r-', linewidth=1.0, label='增益调度PID')
    axes[0].plot(t, w_fixed, 'b-', linewidth=1.0, label='固定PID')
    axes[0].set_ylabel('转速 (rad/s)')
    axes[0].set_title('增益调度PID vs 固定PID 速度跟踪')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    # 增益调度参数变化
    axes[1].plot(t, params_log[:, 0], linewidth=1.2, label='Kp')
    axes[1].plot(t, params_log[:, 1]/100, linewidth=1.2, label='Ki/100')
    axes[1].plot(t, params_log[:, 2]*100, linewidth=1.2, label='Kd*100')
    axes[1].set_ylabel('增益值')
    axes[1].set_title('增益调度参数变化曲线')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    # 误差对比
    axes[2].plot(t, w_ref-w_gs, 'r-', linewidth=0.8, label='增益调度误差')
    axes[2].plot(t, w_ref-w_fixed, 'b-', linewidth=0.8, label='固定PID误差')
    axes[2].set_xlabel('时间 (s)')
    axes[2].set_ylabel('跟踪误差')
    axes[2].set_title('跟踪误差对比')
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'gain_scheduling_adaptive.png')
    plt.savefig(out, dpi=150)
    plt.close()
    print(f"\n已保存: {out}")



if __name__ == '__main__':
    main()
