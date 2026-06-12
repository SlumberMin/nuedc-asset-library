#!/usr/bin/env python3
"""
PID vs ADRC 对比仿真 (V2)
==========================
仿真实验:
  1. 阶跃响应对比
  2. 阶跃扰动抑制
  3. 正弦扰动抑制
  4. 参数变化鲁棒性
  5. 噪声环境性能
被控对象: 二阶系统 G(s) = 1 / (s^2 + 0.5s + 1)
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
matplotlib.rcParams['axes.unicode_minus'] = False


class PID:
    def __init__(self, kp, ki, kd, dt):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dt = dt
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, ref, y):
        e = ref - y
        self.integral += e * self.dt
        de = (e - self.prev_error) / self.dt
        self.prev_error = e
        return self.kp * e + self.ki * self.integral + self.kd * de

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


class LADRC:
    def __init__(self, wc, wo, b0, dt):
        self.wc = wc
        self.wo = wo
        self.b0 = b0 if abs(b0) > 1e-6 else 1e-6
        self.dt = dt
        self.z1 = self.z2 = self.z3 = 0.0
        self.prev_y = 0.0
        self.u_prev = 0.0

    def compute(self, ref, y):
        e = self.prev_y - self.z1
        self.z1 += (self.z2 + 3*self.wo*e) * self.dt
        self.z2 += (self.z3 + 3*self.wo**2*e + self.b0*self.u_prev) * self.dt
        self.z3 += (self.wo**3*e) * self.dt
        self.prev_y = y
        e1 = ref - self.z1
        e2 = -self.z2
        u = (self.wc**2*e1 + 2*self.wc*e2 - self.z3) / self.b0
        self.u_prev = u
        return u

    def reset(self):
        self.z1 = self.z2 = self.z3 = 0.0
        self.prev_y = self.u_prev = 0.0


def plant_step(x, u, m, b, k, dt):
    pos, vel = x
    acc = (u - b*vel - k*pos) / m
    vel += acc * dt
    pos += vel * dt
    return np.array([pos, vel])


def run_sim(ctrl, m, b, k, ref_f, dist_f, dt, N, noise_std=0.0):
    x = np.array([0.0, 0.0])
    y_log = np.zeros(N)
    u_log = np.zeros(N)
    rng = np.random.default_rng(42)
    for i in range(N):
        ref = ref_f(i*dt)
        y = x[0] + (rng.normal(0, noise_std) if noise_std > 0 else 0)
        u = ctrl.compute(ref, y)
        d = dist_f(i*dt)
        x = plant_step(x, u+d, m, b, k, dt)
        y_log[i] = x[0]
        u_log[i] = u
    return y_log, u_log


def calc_metrics(t, y, ref, t0=0.0):
    idx = int(t0 / (t[1]-t[0]))
    e = ref - y[idx:]
    iae = np.sum(np.abs(e)) * (t[1]-t[0])
    ise = np.sum(e**2) * (t[1]-t[0])
    peak = np.max(y)
    os_pct = max(0, (peak-ref)/ref*100) if ref > 0 else 0
    return {'IAE': iae, 'ISE': ise, 'OS%': os_pct}


def main():
    dt = 0.001
    T = 5.0
    N = int(T/dt)
    t = np.arange(N)*dt
    m, b, k = 1.0, 0.5, 1.0

    ref_f = lambda t: 1.0 if t >= 0.5 else 0.0

    fig, axes = plt.subplots(3, 3, figsize=(18, 14))
    fig.suptitle('PID vs ADRC 全面对比仿真', fontsize=16, fontweight='bold')

    results = {}

    # ===== 实验1: 阶跃响应 =====
    pid = PID(15.0, 5.0, 3.0, dt)
    adrc = LADRC(10, 30, 1.0, dt)
    y1p, u1p = run_sim(pid, m, b, k, ref_f, lambda t: 0, dt, N)
    adrc.reset()
    y1a, u1a = run_sim(adrc, m, b, k, ref_f, lambda t: 0, dt, N)

    m1p = calc_metrics(t, y1p, 1.0, 0.5)
    m1a = calc_metrics(t, y1a, 1.0, 0.5)
    results['step'] = (m1p, m1a)

    ax = axes[0, 0]
    ax.plot(t, y1p, 'b-', label=f'PID (IAE={m1p["IAE"]:.2f})', linewidth=1.5)
    ax.plot(t, y1a, 'r--', label=f'ADRC (IAE={m1a["IAE"]:.2f})', linewidth=1.5)
    ax.axhline(1.0, color='k', linestyle=':', alpha=0.4)
    ax.set_title('实验1: 阶跃响应')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('输出')
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    # ===== 实验2: 阶跃扰动 =====
    pid.reset(); adrc.reset()
    dist_f2 = lambda t: 0.5 if 2.0 <= t < 3.0 else 0.0
    y2p, _ = run_sim(pid, m, b, k, ref_f, dist_f2, dt, N)
    adrc2 = LADRC(10, 30, 1.0, dt)
    y2a, _ = run_sim(adrc2, m, b, k, ref_f, dist_f2, dt, N)
    m2p = calc_metrics(t, y2p, 1.0, 0.5)
    m2a = calc_metrics(t, y2a, 1.0, 0.5)

    ax = axes[0, 1]
    ax.plot(t, y2p, 'b-', label=f'PID (IAE={m2p["IAE"]:.2f})', linewidth=1.5)
    ax.plot(t, y2a, 'r--', label=f'ADRC (IAE={m2a["IAE"]:.2f})', linewidth=1.5)
    ax.axhline(1.0, color='k', linestyle=':', alpha=0.4)
    ax.axvspan(2.0, 3.0, alpha=0.1, color='green')
    ax.set_title('实验2: 阶跃扰动抑制')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('输出')
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    # ===== 实验3: 正弦扰动 =====
    pid.reset()
    dist_f3 = lambda t: 0.3*np.sin(2*np.pi*2*t) if t >= 1.5 else 0.0
    y3p, _ = run_sim(pid, m, b, k, ref_f, dist_f3, dt, N)
    adrc3 = LADRC(10, 30, 1.0, dt)
    y3a, _ = run_sim(adrc3, m, b, k, ref_f, dist_f3, dt, N)
    m3p = calc_metrics(t, y3p, 1.0, 0.5)
    m3a = calc_metrics(t, y3a, 1.0, 0.5)

    ax = axes[0, 2]
    ax.plot(t, y3p, 'b-', label=f'PID (IAE={m3p["IAE"]:.2f})', linewidth=1.5)
    ax.plot(t, y3a, 'r--', label=f'ADRC (IAE={m3a["IAE"]:.2f})', linewidth=1.5)
    ax.axhline(1.0, color='k', linestyle=':', alpha=0.4)
    ax.set_title('实验3: 正弦扰动抑制')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('输出')
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    # ===== 实验4: 参数变化 =====
    m2, b2, k2 = 2.0, 0.25, 1.5
    pid.reset()
    y4p, _ = run_sim(pid, m2, b2, k2, ref_f, lambda t: 0, dt, N)
    adrc4 = LADRC(10, 30, 1.0, dt)
    y4a, _ = run_sim(adrc4, m2, b2, k2, ref_f, lambda t: 0, dt, N)
    m4p = calc_metrics(t, y4p, 1.0, 0.5)
    m4a = calc_metrics(t, y4a, 1.0, 0.5)

    ax = axes[1, 0]
    ax.plot(t, y4p, 'b-', label=f'PID (IAE={m4p["IAE"]:.2f})', linewidth=1.5)
    ax.plot(t, y4a, 'r--', label=f'ADRC (IAE={m4a["IAE"]:.2f})', linewidth=1.5)
    ax.axhline(1.0, color='k', linestyle=':', alpha=0.4)
    ax.set_title('实验4: 参数变化鲁棒性')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('输出')
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    # ===== 实验5: 噪声环境 =====
    pid.reset()
    y5p, u5p = run_sim(pid, m, b, k, ref_f, lambda t: 0, dt, N, noise_std=0.05)
    adrc5 = LADRC(10, 30, 1.0, dt)
    y5a, u5a = run_sim(adrc5, m, b, k, ref_f, lambda t: 0, dt, N, noise_std=0.05)

    ax = axes[1, 1]
    ax.plot(t, y5p, 'b-', label='PID', linewidth=0.8, alpha=0.8)
    ax.plot(t, y5a, 'r-', label='ADRC', linewidth=0.8, alpha=0.8)
    ax.axhline(1.0, color='k', linestyle=':', alpha=0.4)
    ax.set_title('实验5: 噪声环境响应')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('输出')
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    # ===== 控制量对比 =====
    ax = axes[1, 2]
    ax.plot(t, u5p, 'b-', label='PID', linewidth=0.5, alpha=0.7)
    ax.plot(t, u5a, 'r-', label='ADRC', linewidth=0.5, alpha=0.7)
    ax.set_title('实验5: 控制量(含噪声)')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('控制量 u')
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    # ===== 综合IAE柱状图 =====
    ax = axes[2, 0]
    exp_names = ['阶跃响应', '阶跃扰动', '正弦扰动', '参数变化']
    pid_iaes = [m1p['IAE'], m2p['IAE'], m3p['IAE'], m4p['IAE']]
    adrc_iaes = [m1a['IAE'], m2a['IAE'], m3a['IAE'], m4a['IAE']]
    x_pos = np.arange(len(exp_names))
    width = 0.35
    ax.bar(x_pos-width/2, pid_iaes, width, label='PID', color='steelblue')
    ax.bar(x_pos+width/2, adrc_iaes, width, label='ADRC', color='coral')
    ax.set_xticks(x_pos)
    ax.set_xticklabels(exp_names, fontsize=9)
    ax.set_title('各实验IAE对比')
    ax.set_ylabel('IAE')
    ax.legend()
    ax.grid(True, alpha=0.3, axis='y')

    # ===== PID参数敏感度 =====
    ax = axes[2, 1]
    kp_range = np.arange(5, 30, 2)
    iaes_kp = []
    for kp in kp_range:
        p = PID(kp, 5.0, 3.0, dt)
        y, _ = run_sim(p, m, b, k, ref_f, lambda t: 0, dt, N)
        iaes_kp.append(calc_metrics(t, y, 1.0, 0.5)['IAE'])
    ax.plot(kp_range, iaes_kp, 'b-o', linewidth=1.5)
    ax.set_title('PID Kp敏感度')
    ax.set_xlabel('Kp')
    ax.set_ylabel('IAE')
    ax.grid(True, alpha=0.3)

    # ===== 总结 =====
    ax = axes[2, 2]
    ax.axis('off')
    win_count = sum(1 for p, a in zip(pid_iaes, adrc_iaes) if a < p)
    summary = (
        "PID vs ADRC 对比总结\n"
        "====================\n\n"
        f"ADRC在{win_count}/{len(exp_names)}个实验中优于PID\n\n"
        "• ADRC优势: 扰动抑制、鲁棒性\n"
        "• PID优势: 简单、噪声抑制\n"
        "• 建议: 扰动大的场景用ADRC\n"
        "         精度要求高且模型已知用PID\n"
        "         可结合: PID + ESO扰动补偿"
    )
    ax.text(0.1, 0.9, summary, transform=ax.transAxes, fontsize=11,
            verticalalignment='top', fontfamily='monospace',
            bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))

    plt.tight_layout()
    out_dir = os.path.dirname(os.path.abspath(__file__))
    plt.savefig(os.path.join(out_dir, 'pid_vs_adrc_comparison_result.png'), dpi=150, bbox_inches='tight')
    print("图表已保存: pid_vs_adrc_comparison_result.png")
    plt.close('all')

    print(f"\n仿真完成! ADRC胜{win_count}/{len(exp_names)}场")


if __name__ == '__main__':
    main()
