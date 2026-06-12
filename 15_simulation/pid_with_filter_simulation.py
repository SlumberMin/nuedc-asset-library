#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PID + 滤波器联合仿真
=====================
仿真内容:
  1. 微分项低通滤波 (一阶IIR) 对噪声抑制的效果
  2. 控制输出低通滤波 (执行器侧) 对抖动的平滑效果
  3. 不同滤波截止频率的对比

作者: nuedc-asset-library
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
plt.rcParams['axes.unicode_minus'] = False


# ── 系统模型 (一阶惯性) ──────────────────────────────
class FirstOrderPlant:
    def __init__(self, K=1.0, tau=0.5, dt=0.001):
        self.K, self.tau, self.dt = K, tau, dt
        self.y = 0.0

    def update(self, u):
        self.y += (self.dt / self.tau) * (self.K * u - self.y)
        return self.y


# ── PID + 可选滤波器 ─────────────────────────────────
class PIDWithFilter:
    def __init__(self, Kp, Ki, Kd, dt,
                 d_filter_fc=None, out_filter_fc=None):
        """
        d_filter_fc:  微分项一阶低通截止频率 (Hz), None=不滤波
        out_filter_fc: 输出一阶低通截止频率 (Hz), None=不滤波
        """
        self.Kp, self.Ki, self.Kd, self.dt = Kp, Ki, Kd, dt
        self.integral = 0.0
        self.prev_error = 0.0
        # 微分滤波状态
        self.d_alpha = self._fc_to_alpha(d_filter_fc) if d_filter_fc else None
        self.d_filtered = 0.0
        # 输出滤波状态
        self.out_alpha = self._fc_to_alpha(out_filter_fc) if out_filter_fc else None
        self.out_filtered = 0.0

    @staticmethod
    def _fc_to_alpha(fc, dt=None):
        """一阶IIR: alpha = dt / (RC + dt), RC = 1/(2*pi*fc)"""
        # dt 在 update 时传入，这里先返回 lambda
        return fc  # 实际 alpha 在 update 中计算

    def _get_alpha(self, fc):
        rc = 1.0 / (2 * np.pi * fc)
        return self.dt / (rc + self.dt)

    def update(self, error):
        # P
        p = self.Kp * error
        # I
        self.integral += error * self.dt
        i = self.Ki * self.integral
        # D (原始)
        d_raw = self.Kd * (error - self.prev_error) / self.dt
        # D 滤波
        if self.d_alpha is not None:
            alpha = self._get_alpha(self.d_alpha)
            self.d_filtered = alpha * d_raw + (1 - alpha) * self.d_filtered
            d = self.d_filtered
        else:
            d = d_raw
        self.prev_error = error

        u = p + i + d
        # 输出滤波
        if self.out_alpha is not None:
            alpha = self._get_alpha(self.out_alpha)
            self.out_filtered = alpha * u + (1 - alpha) * self.out_filtered
            return self.out_filtered
        return u


# ── 仿真函数 ─────────────────────────────────────────
def run_sim(pid_cfg, label, T=5.0, dt=0.001, noise_std=0.0):
    n = int(T / dt)
    t = np.arange(n) * dt
    ref = np.ones(n)  # 阶跃目标
    plant = FirstOrderPlant(K=1.0, tau=0.3, dt=dt)
    pid = PIDWithFilter(**pid_cfg, dt=dt)
    y = np.zeros(n)
    u = np.zeros(n)
    for i in range(n):
        measurement = plant.y + np.random.randn() * noise_std
        error = ref[i] - measurement
        u[i] = pid.update(error)
        y[i] = plant.update(u[i])
    return t, y, u


# ── 主仿真 ──────────────────────────────────────────
if __name__ == '__main__':
    dt = 0.001
    noise_std = 0.02  # 测量噪声

    configs = {
        '无滤波': dict(Kp=2.0, Ki=5.0, Kd=0.3, d_filter_fc=None, out_filter_fc=None),
        '微分滤波(fc=50Hz)': dict(Kp=2.0, Ki=5.0, Kd=0.3, d_filter_fc=50, out_filter_fc=None),
        '微分滤波(fc=10Hz)': dict(Kp=2.0, Ki=5.0, Kd=0.3, d_filter_fc=10, out_filter_fc=None),
        '输出滤波(fc=30Hz)': dict(Kp=2.0, Ki=5.0, Kd=0.3, d_filter_fc=None, out_filter_fc=30),
        '双重滤波(D=50,O=30)': dict(Kp=2.0, Ki=5.0, Kd=0.3, d_filter_fc=50, out_filter_fc=30),
    }

    results = {}
    for label, cfg in configs.items():
        t, y, u = run_sim(cfg, label, T=5.0, dt=dt, noise_std=noise_std)
        results[label] = (t, y, u)

    # ── 绘图 ──
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))

    for label, (t, y, u) in results.items():
        axes[0].plot(t, y, label=label, linewidth=0.8)
        axes[1].plot(t, u, label=label, linewidth=0.8)

    axes[0].axhline(1.0, color='k', linestyle='--', alpha=0.3, label='参考值')
    axes[0].set_ylabel('输出 y')
    axes[0].set_title('PID + 滤波器仿真 (σ_noise=0.02)')
    axes[0].legend(fontsize=8)
    axes[0].grid(True, alpha=0.3)

    axes[1].set_ylabel('控制量 u')
    axes[1].legend(fontsize=8)
    axes[1].grid(True, alpha=0.3)

    # 局部放大 (稳态噪声区域)
    mask = (t > 3.0) & (t < 3.5)
    for label, (t, y, u) in results.items():
        axes[2].plot(t[mask], u[mask], label=label, linewidth=0.8)
    axes[2].set_xlabel('时间 (s)')
    axes[2].set_ylabel('控制量 u (放大)')
    axes[2].set_title('稳态控制量放大 (3.0~3.5s)')
    axes[2].legend(fontsize=8)
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('pid_filter_comparison.png', dpi=150)
    print('[OK] 图像已保存: pid_filter_comparison.png')

    # ── 定量指标 ──
    print('\n=== 稳态控制量标准差 (噪声引起) ===')
    for label, (t, y, u) in results.items():
        mask = t > 3.0
        print(f'  {label:30s}  σ_u = {np.std(u[mask]):.4f}')

    plt.close('all')
