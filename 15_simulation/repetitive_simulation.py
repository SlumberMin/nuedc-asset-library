#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
重复控制仿真 — 周期性扰动消除
================================
应用场景: 逆变器输出波形控制、电网谐波抑制、机器人周期运动

重复控制原理:
  内模原理: 在反馈回路中嵌入周期信号发生器
  传递函数: Grc(z) = Q(z)*z^{-N} / (1 - Q(z)*z^{-N})
  其中 N = 周期对应的采样点数, Q(z) 为低通滤波器

对比:
  1) 常规PID (对周期扰动抑制能力有限)
  2) 重复控制 + PID
  3) 不同Q(z)滤波器的影响
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib


def main():
    matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
    matplotlib.rcParams['axes.unicode_minus'] = False

    # ── 系统参数 ──
    Ts = 0.001         # 1kHz采样
    T_sim = 0.3        # 300ms仿真 (3个基波周期)
    N = int(T_sim / Ts)
    t = np.arange(N) * Ts

    f_ref = 50.0       # 基波频率 50Hz
    f_sample = 1/Ts
    N_period = int(f_sample / f_ref)  # 每周期采样点数 = 20

    # ── 参考信号: 50Hz正弦 ──
    r = np.sin(2 * np.pi * f_ref * t)

    # ── 周期性扰动: 3次+5次谐波 ──
    d = 0.3 * np.sin(2 * np.pi * 3 * f_ref * t) + \
        0.2 * np.sin(2 * np.pi * 5 * f_ref * t) + \
        0.1 * np.sin(2 * np.pi * 7 * f_ref * t)

    # ── 被控对象: 二阶低通 (逆变器模型) ──
    # G(s) = ωn^2 / (s^2 + 2*zeta*ωn*s + ωn^2)
    wn = 2 * np.pi * 200   # 截止频率200Hz
    zeta = 0.4
    # 离散化 (双线性变换)
    a0 = 4 + 4*zeta*wn*Ts + (wn*Ts)**2
    a1 = (2*(wn*Ts)**2 - 8) / a0
    a2 = (4 - 4*zeta*wn*Ts + (wn*Ts)**2) / a0
    b0 = (wn*Ts)**2 / a0
    b1 = 2 * (wn*Ts)**2 / a0
    b2 = (wn*Ts)**2 / a0

    class DiscretePlant:
        def __init__(self):
            self.y = [0.0, 0.0]
            self.u = [0.0, 0.0, 0.0]

        def step(self, u_new):
            self.u = [u_new] + self.u[:2]
            y_new = b0*self.u[0] + b1*self.u[1] + b2*self.u[2] - a1*self.y[0] - a2*self.y[1]
            self.y = [y_new] + self.y[:1]
            return y_new

    # ── PID控制器 ──
    class PID:
        def __init__(self, Kp, Ki, Kd, dt, umin=-10, umax=10):
            self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
            self.dt = dt
            self.umin, self.umax = umin, umax
            self.ei = 0.0
            self.e_prev = 0.0

        def update(self, e):
            self.ei += e * self.dt
            self.ei = np.clip(self.ei, -5, 5)
            ed = (e - self.e_prev) / self.dt if self.dt > 0 else 0
            self.e_prev = e
            u = self.Kp * e + self.Ki * self.ei + self.Kd * ed
            return np.clip(u, self.umin, self.umax)

    # ── 重复控制器 ──
    class RepetitiveController:
        """
        重复控制器:
        u_rc(k) = Q(z) * u_rc(k-N) + Kr * e(k-N)
        Q(z): 零相移低通滤波器 (移动平均)
        Kr: 重复控制增益
        """
        def __init__(self, N_period, Kr=0.5, Q_order=3):
            self.N = N_period
            self.Kr = Kr
            self.Q_order = Q_order  # Q滤波器阶数 (移动平均窗口半宽)
            # 延迟缓冲区
            self.buffer = [0.0] * (N_period + Q_order + 1)
            self.e_buffer = [0.0] * (N_period + Q_order + 1)

        def _Q_filter(self, buf, idx):
            """零相移低通滤波器 (移动平均)"""
            n = self.Q_order
            total = 0.0
            count = 0
            for j in range(-n, n+1):
                pos = idx + j
                if 0 <= pos < len(buf):
                    total += buf[pos]
                    count += 1
            return total / max(count, 1)

        def update(self, e):
            self.e_buffer.append(e)
            if len(self.e_buffer) > 2 * self.N + 10:
                self.e_buffer.pop(0)

            # 取N步前的误差
            if len(self.e_buffer) > self.N:
                e_delayed = self.e_buffer[-self.N-1]
            else:
                e_delayed = 0.0

            # 取N步前的控制量
            if len(self.buffer) > self.N:
                u_delayed = self.buffer[-self.N-1]
            else:
                u_delayed = 0.0

            # Q滤波
            u_q = self._Q_filter(self.buffer, len(self.buffer)-1)

            # 重复控制律: u_rc = Q*u_rc(k-N) + Kr*e(k-N)
            u_rc = u_q + self.Kr * e_delayed

            self.buffer.append(u_rc)
            if len(self.buffer) > 2 * self.N + 10:
                self.buffer.pop(0)

            return u_rc


    # ── 仿真1: 仅PID ──
    def run_pid_only():
        plant = DiscretePlant()
        pid = PID(Kp=3.0, Ki=500.0, Kd=0.01, dt=Ts)
        y_hist, u_hist, e_hist = np.zeros(N), np.zeros(N), np.zeros(N)

        for i in range(N):
            y = plant.y[0]
            e = r[i] - y
            u_pid = pid.update(e)
            y_new = plant.step(u_pid + d[i])  # 扰动叠加
            y_hist[i] = y_new
            u_hist[i] = u_pid
            e_hist[i] = e
        return y_hist, u_hist, e_hist

    # ── 仿真2: 重复控制 + PID ──
    def run_repetitive_pid(Kr=0.5, Q_order=3):
        plant = DiscretePlant()
        pid = PID(Kp=3.0, Ki=500.0, Kd=0.01, dt=Ts)
        rc = RepetitiveController(N_period, Kr=Kr, Q_order=Q_order)
        y_hist, u_hist, e_hist = np.zeros(N), np.zeros(N), np.zeros(N)

        for i in range(N):
            y = plant.y[0]
            e = r[i] - y
            u_pid = pid.update(e)
            u_rc = rc.update(e)
            u_total = u_pid + u_rc
            y_new = plant.step(u_total + d[i])
            y_hist[i] = y_new
            u_hist[i] = u_total
            e_hist[i] = e
        return y_hist, u_hist, e_hist

    # ── 运行仿真 ──
    y_pid, u_pid, e_pid = run_pid_only()
    y_rc1, u_rc1, e_rc1 = run_repetitive_pid(Kr=0.5, Q_order=2)
    y_rc2, u_rc2, e_rc2 = run_repetitive_pid(Kr=0.8, Q_order=5)

    # ── 绘图 ──
    fig, axes = plt.subplots(3, 2, figsize=(14, 12))
    fig.suptitle('重复控制仿真 — 周期性扰动消除', fontsize=14, fontweight='bold')

    # (0,0) 输出波形对比
    axes[0, 0].plot(t*1000, r, 'k--', lw=1, alpha=0.5, label='参考')
    axes[0, 0].plot(t*1000, y_pid, 'r-', lw=0.8, label='仅PID')
    axes[0, 0].plot(t*1000, y_rc1, 'b-', lw=0.8, label='PID+RC(Kr=0.5)')
    axes[0, 0].plot(t*1000, y_rc2, 'g-', lw=0.8, label='PID+RC(Kr=0.8)')
    axes[0, 0].set_ylabel('输出 y(t)')
    axes[0, 0].set_title('输出波形对比')
    axes[0, 0].legend(fontsize=8)
    axes[0, 0].grid(True, alpha=0.3)

    # (0,1) 扰动信号
    axes[0, 1].plot(t*1000, d, 'r-', lw=1)
    axes[0, 1].set_ylabel('扰动 d(t)')
    axes[0, 1].set_title('周期性谐波扰动 (3+5+7次)')
    axes[0, 1].grid(True, alpha=0.3)

    # (1,0) 误差对比
    axes[1, 0].plot(t*1000, e_pid, 'r-', lw=0.8, label='仅PID')
    axes[1, 0].plot(t*1000, e_rc1, 'b-', lw=0.8, label='PID+RC(Kr=0.5)')
    axes[1, 0].plot(t*1000, e_rc2, 'g-', lw=0.8, label='PID+RC(Kr=0.8)')
    axes[1, 0].set_ylabel('误差 e(t)')
    axes[1, 0].set_title('跟踪误差对比')
    axes[1, 0].legend(fontsize=8)
    axes[1, 0].grid(True, alpha=0.3)

    # (1,1) 误差收敛过程 (RMS每周期)
    periods = int(T_sim * f_ref)
    rms_pid = []
    rms_rc1 = []
    rms_rc2 = []
    for p in range(periods):
        idx_s = int(p / f_ref / Ts)
        idx_e = int((p+1) / f_ref / Ts)
        if idx_e <= N:
            rms_pid.append(np.sqrt(np.mean(e_pid[idx_s:idx_e]**2)))
            rms_rc1.append(np.sqrt(np.mean(e_rc1[idx_s:idx_e]**2)))
            rms_rc2.append(np.sqrt(np.mean(e_rc2[idx_s:idx_e]**2)))

    p_idx = np.arange(len(rms_pid))
    axes[1, 1].bar(p_idx - 0.2, rms_pid, 0.2, color='red', alpha=0.7, label='仅PID')
    axes[1, 1].bar(p_idx, rms_rc1, 0.2, color='blue', alpha=0.7, label='PID+RC(Kr=0.5)')
    axes[1, 1].bar(p_idx + 0.2, rms_rc2, 0.2, color='green', alpha=0.7, label='PID+RC(Kr=0.8)')
    axes[1, 1].set_xlabel('周期数')
    axes[1, 1].set_ylabel('RMS误差')
    axes[1, 1].set_title('每周期RMS误差收敛过程')
    axes[1, 1].legend(fontsize=8)
    axes[1, 1].grid(True, alpha=0.3, axis='y')

    # (2,0) 频谱分析 (最后一个周期的误差)
    idx_s = int(2 / f_ref / Ts)
    idx_e = min(N, idx_s + N_period)
    e_pid_last = e_pid[idx_s:idx_e]
    e_rc1_last = e_rc1[idx_s:idx_e]
    e_rc2_last = e_rc2[idx_s:idx_e]

    freqs = np.fft.rfftfreq(len(e_pid_last), Ts)
    spec_pid = np.abs(np.fft.rfft(e_pid_last)) * 2 / len(e_pid_last)
    spec_rc1 = np.abs(np.fft.rfft(e_rc1_last)) * 2 / len(e_rc1_last)
    spec_rc2 = np.abs(np.fft.rfft(e_rc2_last)) * 2 / len(e_rc2_last)

    axes[2, 0].plot(freqs, spec_pid, 'r-', lw=1.2, label='仅PID')
    axes[2, 0].plot(freqs, spec_rc1, 'b-', lw=1.2, label='PID+RC(Kr=0.5)')
    axes[2, 0].plot(freqs, spec_rc2, 'g-', lw=1.2, label='PID+RC(Kr=0.8)')
    axes[2, 0].set_xlabel('频率 (Hz)')
    axes[2, 0].set_ylabel('幅值')
    axes[2, 0].set_title('误差频谱 (最后一个周期)')
    axes[2, 0].legend(fontsize=8)
    axes[2, 0].set_xlim(0, 500)
    axes[2, 0].grid(True, alpha=0.3)

    # (2,1) 性能指标汇总
    iae_pid_val = np.sum(np.abs(e_pid)) * Ts
    iae_rc1_val = np.sum(np.abs(e_rc1)) * Ts
    iae_rc2_val = np.sum(np.abs(e_rc2)) * Ts

    names = ['仅PID', 'PID+RC\n(Kr=0.5)', 'PID+RC\n(Kr=0.8)']
    values = [iae_pid_val, iae_rc1_val, iae_rc2_val]
    colors = ['red', 'blue', 'green']
    bars = axes[2, 1].bar(names, values, color=colors, alpha=0.7)
    for bar, val in zip(bars, values):
        axes[2, 1].text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.001,
                         f'{val:.4f}', ha='center', fontsize=10)
    axes[2, 1].set_ylabel('IAE')
    axes[2, 1].set_title('IAE性能指标对比')
    axes[2, 1].grid(True, alpha=0.3, axis='y')

    plt.tight_layout()
    plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'repetitive_result.png'), dpi=150, bbox_inches='tight')
    plt.close('all')

    print(f"✅ 重复控制仿真完成")
    print(f"   仅PID         IAE = {iae_pid_val:.4f}")
    print(f"   PID+RC(Kr=0.5) IAE = {iae_rc1_val:.4f}")
    print(f"   PID+RC(Kr=0.8) IAE = {iae_rc2_val:.4f}")
    print(f"   谐波扰动抑制提升: {(1-iae_rc2_val/iae_pid_val)*100:.1f}%")



if __name__ == '__main__':
    main()
