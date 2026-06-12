# -*- coding: utf-8 -*-
"""
全算法综合对比仿真 V2
====================
对比6种主流控制算法在同一被控对象上的综合性能
包括: PID, LADRC, 反步法, 滑模, 前馈+PID, 串级PID
"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os



def main():
    plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False

    # ========== 统一仿真环境 ==========
    dt = 0.0005
    T = 3.0
    t = np.arange(0, T, dt)
    N = len(t)

    # 统一被控对象: 二阶电机模型
    J = 0.001; B = 0.005; Kt = 1.5
    def plant(x, u, dist=0):
        """被控对象模型"""
        w_dot = (-B * x[0] + Kt * x[1] - dist) / J
        iq_dot = (-1.0 * x[1] + u) / 0.005
        return np.array([w_dot, iq_dot])

    # 参考信号
    w_ref = np.ones(N) * 100.0
    w_ref[t > 1.0] = 200.0
    w_ref[t > 2.0] = 50.0

    # 扰动
    dist = np.zeros(N)
    dist[int(1.5/dt):int(1.6/dt)] = 3.0  # 脉冲扰动
    dist[int(2.5/dt):] = 1.0  # 阶跃扰动

    # ========== 算法1: PID ==========
    def run_pid(Kp, Ki, Kd):
        x = np.zeros(2)
        y = np.zeros(N); u_out = np.zeros(N)
        ei = 0; ed_prev = 0
        for i in range(N):
            y[i] = x[0]
            e = w_ref[i] - x[0]
            ei += e * dt; ei = np.clip(ei, -5, 5)
            ed = (e - ed_prev) / dt
            u = Kp * e + Ki * ei + Kd * ed
            u = np.clip(u, -50, 50)
            u_out[i] = u
            dx = plant(x, u, dist[i])
            x += dx * dt
            ed_prev = e
        return y, u_out

    # ========== 算法2: LADRC (一阶速度环 + 电流环) ==========
    def run_ladrc(wc, wo, b0=1.0):
        x = np.zeros(2)  # [speed, current]
        y = np.zeros(N); u_out = np.zeros(N)
        # 一阶LESO: 估计速度和总扰动
        z1, z2 = 0.0, 0.0
        beta1, beta2 = 2*wo, wo**2
        for i in range(N):
            y[i] = x[0]
            eo = z1 - x[0]
            # LESO更新
            z1 += (z2 - beta1*eo + b0*x[1]) * dt
            z2 += (-beta2*eo) * dt
            # 控制律: 速度环输出电流指令
            e = w_ref[i] - z1
            iq_cmd = wc * e - z2 / b0
            iq_cmd = np.clip(iq_cmd, -20, 20)
            # 简单电流环P控制
            u = (iq_cmd - x[1]) * 100
            u = np.clip(u, -50, 50)
            u_out[i] = u
            dx = plant(x, u, dist[i])
            x += dx * dt
        return y, u_out

    # ========== 算法3: 反步法 ==========
    def run_backstepping(k1, k2):
        x = np.zeros(2)
        y = np.zeros(N); u_out = np.zeros(N)
        for i in range(N):
            y[i] = x[0]
            e1 = w_ref[i] - x[0]
            # 虚拟控制: 期望电流
            alpha1 = (B*x[0] + dist[i] + J*k1*e1) / Kt
            e2 = x[1] - alpha1
            # 控制律 (简化为电流环)
            u = -k2 * e2 + 1.0 * x[1]
            u = np.clip(u, -50, 50)
            u_out[i] = u
            dx = plant(x, u, dist[i])
            x += dx * dt
        return y, u_out

    # ========== 算法4: 滑模控制 ==========
    def run_smc(lam, eta, k):
        x = np.zeros(2)
        y = np.zeros(N); u_out = np.zeros(N)
        for i in range(N):
            y[i] = x[0]
            e = w_ref[i] - x[0]
            de = -x[0]  # 简化: 误差导数
            s = lam * e + de  # 滑模面
            u = k * np.sign(s) + eta * s  # 趋近律
            u = np.clip(u, -50, 50)
            u_out[i] = u
            dx = plant(x, u, dist[i])
            x += dx * dt
        return y, u_out

    # ========== 算法5: 前馈+PID ==========
    def run_ff_pid(Kp, Ki, Kd):
        x = np.zeros(2)
        y = np.zeros(N); u_out = np.zeros(N)
        ei = 0; ed_prev = 0
        for i in range(N):
            y[i] = x[0]
            e = w_ref[i] - x[0]
            ei += e * dt; ei = np.clip(ei, -5, 5)
            ed = (e - ed_prev) / dt
            # 前馈: 估计系统逆模型
            w_ref_dot = 0
            if i > 0 and i < N-1:
                w_ref_dot = (w_ref[i+1] - w_ref[i-1]) / (2*dt)
            u_ff = (J * w_ref_dot + B * w_ref[i]) / Kt
            u_pid = Kp * e + Ki * ei + Kd * ed
            u = u_ff + u_pid
            u = np.clip(u, -50, 50)
            u_out[i] = u
            dx = plant(x, u, dist[i])
            x += dx * dt
            ed_prev = e
        return y, u_out

    # ========== 算法6: 串级PID ==========
    def run_cascade():
        x = np.zeros(2)
        y = np.zeros(N); u_out = np.zeros(N)
        ei_out = 0; ed_out_prev = 0; ei_in = 0
        for i in range(N):
            y[i] = x[0]
            # 外环
            e_out = w_ref[i] - x[0]
            ei_out += e_out * dt; ei_out = np.clip(ei_out, -5, 5)
            ed_out = (e_out - ed_out_prev) / dt
            iq_cmd = 0.5*e_out + 10*ei_out + 0.001*ed_out
            iq_cmd = np.clip(iq_cmd, -20, 20)
            # 内环
            e_in = iq_cmd - x[1]
            ei_in += e_in * dt; ei_in = np.clip(ei_in, -50, 50)
            u = 100*e_in + 5000*ei_in
            u = np.clip(u, -50, 50)
            u_out[i] = u
            dx = plant(x, u, dist[i])
            x += dx * dt
            ed_out_prev = e_out
        return y, u_out

    # ========== 运行所有算法 ==========
    results = {}
    results['PID'] = run_pid(0.5, 20.0, 0.01)
    results['LADRC'] = run_ladrc(50, 100, b0=1500.0)
    results['反步法'] = run_backstepping(50, 100)
    results['滑模控制'] = run_smc(10, 50, 20)
    results['前馈+PID'] = run_ff_pid(0.3, 15.0, 0.005)
    results['串级PID'] = run_cascade()

    # ========== 性能指标计算 ==========
    print("=" * 70)
    print(f"{'算法':>10} | {'IAE':>10} | {'ISE':>10} | {'最大误差':>10} | {'控制能耗':>10}")
    print("-" * 70)

    metrics_all = {}
    for name, (y, u) in results.items():
        err = w_ref - y
        iae = np.sum(np.abs(err)) * dt
        ise = np.sum(err**2) * dt
        max_e = np.max(np.abs(err))
        energy = np.sum(u**2) * dt
        metrics_all[name] = {'IAE': iae, 'ISE': ise, '最大误差': max_e, '控制能耗': energy}
        print(f"{name:>10} | {iae:>10.4f} | {ise:>10.4f} | {max_e:>10.4f} | {energy:>10.4f}")

    print("=" * 70)

    # ========== 绘图 ==========
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b']
    linestyles = ['-', '-', '-', '--', '-.', ':']

    fig, axes = plt.subplots(3, 1, figsize=(14, 12))

    # 跟踪对比
    axes[0].plot(t, w_ref, 'k--', linewidth=2, label='参考信号', zorder=10)
    for idx, (name, (y, _)) in enumerate(results.items()):
        axes[0].plot(t, y, color=colors[idx], linestyle=linestyles[idx], 
                    linewidth=1.0, label=name)
    axes[0].set_ylabel('转速 (rad/s)')
    axes[0].set_title('全算法综合对比V2 — 速度跟踪')
    axes[0].legend(loc='upper right', fontsize=9, ncol=3)
    axes[0].grid(True, alpha=0.3)

    # 跟踪误差
    for idx, (name, (y, _)) in enumerate(results.items()):
        axes[1].plot(t, w_ref-y, color=colors[idx], linewidth=0.6, label=name)
    axes[1].set_ylabel('跟踪误差')
    axes[1].set_title('跟踪误差对比')
    axes[1].legend(fontsize=9, ncol=3)
    axes[1].grid(True, alpha=0.3)

    # 性能指标柱状图
    algo_names = list(metrics_all.keys())
    iae_vals = [metrics_all[n]['IAE'] for n in algo_names]
    ise_vals = [metrics_all[n]['ISE'] for n in algo_names]
    x_pos = np.arange(len(algo_names))
    width = 0.35
    bars1 = axes[2].bar(x_pos - width/2, iae_vals, width, label='IAE', color='steelblue')
    bars2 = axes[2].bar(x_pos + width/2, ise_vals, width, label='ISE', color='coral')
    axes[2].set_xticks(x_pos)
    axes[2].set_xticklabels(algo_names)
    axes[2].set_ylabel('指标值')
    axes[2].set_title('各算法性能指标对比')
    axes[2].legend()
    axes[2].grid(True, alpha=0.3, axis='y')

    # 在柱子上标注数值
    for bar in bars1:
        axes[2].text(bar.get_x() + bar.get_width()/2, bar.get_height(),
                    f'{bar.get_height():.1f}', ha='center', va='bottom', fontsize=7)
    for bar in bars2:
        axes[2].text(bar.get_x() + bar.get_width()/2, bar.get_height(),
                    f'{bar.get_height():.1f}', ha='center', va='bottom', fontsize=7)

    plt.tight_layout()
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'all_algorithms_comparison_v2.png')
    plt.savefig(out, dpi=150)
    plt.close()
    print(f"\n已保存: {out}")



if __name__ == '__main__':
    main()
