#!/usr/bin/env python3
"""
完整系统辨识仿真
================
方法: 阶跃响应法、频率响应法、最小二乘法、递推最小二乘法
被控对象: 电机系统 G(s) = K / (Ts + 1) * e^(-Ls)
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
matplotlib.rcParams['axes.unicode_minus'] = False


def main():
    dt = 0.001
    T = 10.0
    N = int(T / dt)
    t = np.arange(N) * dt

    # ===== 真实系统参数 =====
    K_true = 2.0    # 增益
    T_true = 0.5    # 时间常数
    L_true = 0.05   # 纯滞后
    print(f"真实系统参数: K={K_true}, T={T_true}, L={L_true}")

    # ===== 生成系统响应 =====
    def first_order_with_delay(u, K, T, L, dt, N):
        """一阶+纯滞后系统仿真"""
        y = np.zeros(N)
        delay_steps = int(L / dt)
        for i in range(1, N):
            # 纯滞后
            i_delayed = max(0, i - delay_steps)
            u_d = u[i_delayed]
            # 一阶惯性: T*dy/dt + y = K*u
            y[i] = y[i-1] + dt / T * (K * u_d - y[i-1])
        return y

    # ===== 实验1: 阶跃响应法 =====
    u_step = np.ones(N)
    y_step = first_order_with_delay(u_step, K_true, T_true, L_true, dt, N)
    # 加噪声
    np.random.seed(42)
    y_step_noisy = y_step + np.random.normal(0, 0.02, N)

    # 阶跃响应法辨识
    # K = y(∞) / u(∞)
    K_est_step = y_step_noisy[-1] / u_step[-1]
    # 找63.2%上升时间 → T
    target_63 = 0.632 * K_est_step
    idx_63 = np.argmax(y_step_noisy >= target_63)
    T_est_step = t[idx_63] - L_true  # 减去滞后
    # 找滞后: 输出开始上升的时刻
    threshold = 0.01 * K_est_step
    idx_lag = np.argmax(y_step_noisy >= threshold)
    L_est_step = t[idx_lag]

    print(f"\n=== 阶跃响应法 ===")
    print(f"  K={K_est_step:.3f} (真值{K_true})")
    print(f"  T={T_est_step:.3f} (真值{T_true})")
    print(f"  L={L_est_step:.3f} (真值{L_true})")

    # ===== 实验2: 频率响应法 =====
    freqs = np.array([0.1, 0.2, 0.5, 1.0, 2.0, 5.0, 10.0])
    magnitudes = []
    phases = []
    for f in freqs:
        omega = 2 * np.pi * f
        # 理论频率响应: G(jω) = K / (jωT + 1) * e^(-jωL)
        s = 1j * omega
        G = K_true / (s * T_true + 1) * np.exp(-s * L_true)
        magnitudes.append(np.abs(G))
        phases.append(np.angle(G, deg=True))

    # 用频率响应辨识
    # K = |G(0)| ≈ 低频增益
    K_est_freq = magnitudes[0]
    # 找-3dB带宽 → ωc → T = 1/ωc
    mag_3db = K_est_freq / np.sqrt(2)
    # 线性插值找-3dB频率
    for i in range(len(magnitudes)-1):
        if magnitudes[i] >= mag_3db and magnitudes[i+1] < mag_3db:
            w_3db = 2*np.pi * (freqs[i] + (freqs[i+1]-freqs[i]) *
                    (magnitudes[i]-mag_3db)/(magnitudes[i]-magnitudes[i+1]))
            T_est_freq = 1.0 / w_3db
            break
    else:
        T_est_freq = T_true

    # 从相位滞后估计延迟
    # 在-3dB频率处: phase ≈ -arctan(ωT) - ωL (rad)
    phase_at_3db = np.interp(1.0/T_est_freq, 2*np.pi*freqs, phases)
    expected_phase = -np.arctan(1.0) * 180/np.pi  # -45° from first order
    L_est_freq = max(0, (expected_phase - phase_at_3db) / (360 * 1.0/(2*np.pi*T_est_freq)))

    print(f"\n=== 频率响应法 ===")
    print(f"  K={K_est_freq:.3f} (真值{K_true})")
    print(f"  T={T_est_freq:.3f} (真值{T_true})")

    # ===== 实验3: 最小二乘法 =====
    # y(k) = a*y(k-1) + b*u(k-d)
    # 其中 a = 1-dt/T, b = K*dt/T, d = L/dt
    delay_steps = int(L_true / dt)
    Y = y_step_noisy[delay_steps+1:]
    Phi = np.column_stack([
        y_step_noisy[delay_steps:-1],
        u_step[:len(Y)]
    ])
    theta = np.linalg.lstsq(Phi, Y, rcond=None)[0]
    a_ls, b_ls = theta
    T_est_ls = -dt / np.log(max(a_ls, 0.001))
    K_est_ls = b_ls * T_est_ls / dt

    print(f"\n=== 最小二乘法 ===")
    print(f"  K={K_est_ls:.3f} (真值{K_true})")
    print(f"  T={T_est_ls:.3f} (真值{T_true})")

    # ===== 实验4: 递推最小二乘法 =====
    theta_rls = np.array([0.5, 0.5])  # 初始估计
    P_rls = np.eye(2) * 100
    lam = 0.99  # 遗忘因子
    theta_log = np.zeros((N, 2))

    for k in range(delay_steps+1, N):
        phi = np.array([y_step_noisy[k-1], u_step[max(0, k-delay_steps)]])
        y_k = y_step_noisy[k]

        # RLS更新
        e = y_k - phi @ theta_rls
        denom = lam + phi @ P_rls @ phi
        K_rls = P_rls @ phi / denom
        theta_rls = theta_rls + K_rls * e
        P_rls = (P_rls - np.outer(K_rls, phi @ P_rls)) / lam

        theta_log[k] = theta_rls

    a_rls, b_rls = theta_rls
    T_est_rls = -dt / np.log(max(a_rls, 0.001))
    K_est_rls = b_rls * T_est_rls / dt

    print(f"\n=== 递推最小二乘法 ===")
    print(f"  K={K_est_rls:.3f} (真值{K_true})")
    print(f"  T={T_est_rls:.3f} (真值{T_true})")

    # ===== 绘图 =====
    fig, axes = plt.subplots(3, 2, figsize=(16, 14))
    fig.suptitle('完整系统辨识仿真', fontsize=16, fontweight='bold')

    # 阶跃响应
    ax = axes[0, 0]
    ax.plot(t, y_step_noisy, 'b-', linewidth=0.5, alpha=0.5, label='含噪声响应')
    ax.plot(t, y_step, 'r-', linewidth=2, label='真实响应')
    ax.axhline(K_true, color='k', linestyle=':', alpha=0.5, label=f'K={K_true}')
    ax.axhline(0.632*K_true, color='g', linestyle=':', alpha=0.5, label='63.2%')
    ax.axvline(L_true, color='orange', linestyle=':', alpha=0.5, label=f'L={L_true}')
    ax.set_title('阶跃响应')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('输出')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # 频率响应Bode图
    ax = axes[0, 1]
    ax.semilogx(freqs, 20*np.log10(np.array(magnitudes)+1e-10), 'bo-', linewidth=2)
    ax.axhline(20*np.log10(K_true/np.sqrt(2)), color='g', linestyle=':', alpha=0.5, label='-3dB')
    ax.set_title('频率响应 (幅频)')
    ax.set_xlabel('频率 (Hz)')
    ax.set_ylabel('增益 (dB)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 相频
    ax = axes[1, 0]
    ax.semilogx(freqs, phases, 'ro-', linewidth=2)
    ax.set_title('频率响应 (相频)')
    ax.set_xlabel('频率 (Hz)')
    ax.set_ylabel('相位 (°)')
    ax.grid(True, alpha=0.3)

    # RLS参数收敛
    ax = axes[1, 1]
    ax.plot(t, theta_log[:, 0], 'b-', linewidth=1, label=f'a (终值={theta_rls[0]:.4f})')
    ax.plot(t, theta_log[:, 1], 'r-', linewidth=1, label=f'b (终值={theta_rls[1]:.4f})')
    ax.axhline(1-dt/T_true, color='b', linestyle=':', alpha=0.5, label=f'a_true={1-dt/T_true:.4f}')
    ax.axhline(K_true*dt/T_true, color='r', linestyle=':', alpha=0.5, label=f'b_true={K_true*dt/T_true:.4f}')
    ax.set_title('RLS参数收敛过程')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('参数值')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # 辨识结果对比
    ax = axes[2, 0]
    methods = ['阶跃法', '最小二乘', 'RLS']
    K_ests = [K_est_step, K_est_ls, K_est_rls]
    T_ests = [T_est_step, T_est_ls, T_est_rls]
    x_pos = np.arange(len(methods))
    width = 0.35
    bars1 = ax.bar(x_pos-width/2, K_ests, width, label='K估计', color='steelblue')
    bars2 = ax.bar(x_pos+width/2, T_ests, width, label='T估计', color='coral')
    ax.axhline(K_true, color='steelblue', linestyle='--', alpha=0.5)
    ax.axhline(T_true, color='coral', linestyle='--', alpha=0.5)
    ax.set_xticks(x_pos)
    ax.set_xticklabels(methods)
    ax.set_title('辨识结果对比')
    ax.legend()
    ax.grid(True, alpha=0.3, axis='y')

    # 误差分析
    ax = axes[2, 1]
    ax.axis('off')
    err_K = [abs(k-K_true)/K_true*100 for k in K_ests]
    err_T = [abs(t_-T_true)/T_true*100 for t_ in T_ests]
    summary = (
        "系统辨识结果总结\n"
        "================\n\n"
        f"真实参数: K={K_true}, T={T_true}, L={L_true}\n\n"
        "方法         K估计    K误差%   T估计    T误差%\n"
        "─" * 50 + "\n"
    )
    for i, m in enumerate(methods):
        summary += f"{m:8s}   {K_ests[i]:6.3f}   {err_K[i]:5.1f}%   {T_ests[i]:6.3f}   {err_T[i]:5.1f}%\n"
    summary += (
        "\n建议:\n"
        "• 阶跃法: 简单快速，适合现场调试\n"
        "• 最小二乘: 需要数据充足\n"
        "• RLS: 适合在线辨识，实时更新"
    )
    ax.text(0.05, 0.95, summary, transform=ax.transAxes, fontsize=10,
            verticalalignment='top', fontfamily='monospace',
            bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))

    plt.tight_layout()
    out_dir = os.path.dirname(os.path.abspath(__file__))
    plt.savefig(os.path.join(out_dir, 'system_identification_complete_result.png'), dpi=150, bbox_inches='tight')
    print("\n图表已保存: system_identification_complete_result.png")
    plt.close('all')
    print("完整系统辨识仿真完成!")


if __name__ == '__main__':
    main()
