#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
自适应PID仿真 — 对比固定参数PID
=================================
场景: 二阶系统参数突变（增益变化+负载扰动）
比较:
  1) 固定参数PID
  2) 自适应PID（基于继电辨识 + 参数在线更新）
输出: 阶跃响应对比、Kp/Ki/Kd自适应曲线、误差指标
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
matplotlib.rcParams['axes.unicode_minus'] = False

# ── 仿真参数 ──
Ts = 0.01          # 采样周期 (s)
T_sim = 20.0       # 仿真时长 (s)
N = int(T_sim / Ts)
t = np.arange(N) * Ts

# ── 参考信号 ──
r = np.ones(N) * 1.0
r[int(8/Ts):] = 0.5   # t=8s 设定值变化

# ── 被控对象: G(s) = K / (T^2*s^2 + 2*zeta*T*s + 1) ──
# t<10s: K=1, T=1, zeta=0.3
# t>=10s: K=1.5, T=0.8, zeta=0.5  (参数突变)
def plant_step(x, u, dt, K, Tn, zeta):
    """二阶系统离散化 (状态空间: x=[y, dy])"""
    x1, x2 = x
    dx1 = x2
    dx2 = (-2*zeta/Tn)*x2 - (1/Tn**2)*x1 + (K/Tn**2)*u
    x1_new = x1 + dx1 * dt
    x2_new = x2 + dx2 * dt
    return np.array([x1_new, x2_new])

# ── 固定参数PID ──
class FixedPID:
    def __init__(self, Kp, Ki, Kd, dt, umin=-10, umax=10):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.dt = dt
        self.umin, self.umax = umin, umax
        self.ei = 0.0
        self.e_prev = 0.0

    def update(self, e):
        self.ei += e * self.dt
        ed = (e - self.e_prev) / self.dt if self.dt > 0 else 0
        self.e_prev = e
        u = self.Kp * e + self.Ki * self.ei + self.Kd * ed
        return np.clip(u, self.umin, self.umax)

# ── 自适应PID ──
class AdaptivePID:
    def __init__(self, Kp0, Ki0, Kd0, dt, umin=-10, umax=10):
        self.Kp, self.Ki, self.Kd = Kp0, Ki0, Kd0
        self.dt = dt
        self.umin, self.umax = umin, umax
        self.ei = 0.0
        self.e_prev = 0.0
        self.e_window = []       # 误差窗口用于辨识
        self.y_window = []       # 输出窗口
        self.u_window = []       # 控制量窗口
        self.win_size = 100
        self.update_counter = 0
        self.Kp_hist, self.Ki_hist, self.Kd_hist = [], [], []
        self.gain_est = 1.0      # 估计的系统增益
        self.tau_est = 1.0       # 估计的时间常数

    def _estimate_params(self):
        """基于窗口数据在线估计系统参数并调整PID"""
        if len(self.y_window) < self.win_size:
            return
        y_arr = np.array(self.y_window[-self.win_size:])
        u_arr = np.array(self.u_window[-self.win_size:])

        # 简单增益估计: Δy / Δu 的稳态比
        dy = y_arr[-1] - y_arr[0]
        du = u_arr[-1] - u_arr[0]
        if abs(du) > 1e-6:
            self.gain_est = 0.9 * self.gain_est + 0.1 * abs(dy / du)

        # 基于增益估计自适应调参 (目标: 保持稳定裕度)
        gain_ratio = 1.0 / max(self.gain_est, 0.1)
        self.Kp = np.clip(2.0 * gain_ratio, 0.5, 10.0)
        self.Ki = np.clip(1.0 * gain_ratio, 0.1, 5.0)
        self.Kd = np.clip(0.5 * gain_ratio, 0.05, 3.0)

    def update(self, e, y, u):
        self.ei += e * self.dt
        ed = (e - self.e_prev) / self.dt if self.dt > 0 else 0
        self.e_prev = e

        self.e_window.append(e)
        self.y_window.append(y)
        self.u_window.append(u)

        self.update_counter += 1
        if self.update_counter % self.win_size == 0:
            self._estimate_params()

        self.Kp_hist.append(self.Kp)
        self.Ki_hist.append(self.Ki)
        self.Kd_hist.append(self.Kd)

        u_out = self.Kp * e + self.Ki * self.ei + self.Kd * ed
        return np.clip(u_out, self.umin, self.umax)

# ── 仿真运行 ──
def run_simulation(pid, is_adaptive=False):
    x = np.zeros(2)
    y_hist = np.zeros(N)
    u_hist = np.zeros(N)
    for i in range(N):
        y = x[0]
        e = r[i] - y
        if is_adaptive:
            u_prev = u_hist[i-1] if i > 0 else 0
            u = pid.update(e, y, u_prev)
        else:
            u = pid.update(e)
        # 参数突变
        if t[i] < 10:
            x = plant_step(x, u, Ts, K=1.0, Tn=1.0, zeta=0.3)
        else:
            x = plant_step(x, u, Ts, K=1.5, Tn=0.8, zeta=0.5)
        # t=5s 加负载扰动
        if abs(t[i] - 5.0) < Ts/2:
            x[0] += 0.2
        y_hist[i] = x[0]
        u_hist[i] = u
    return y_hist, u_hist

# 创建控制器
pid_fixed = FixedPID(Kp=2.0, Ki=1.0, Kd=0.5, dt=Ts)
pid_adapt = AdaptivePID(Kp0=2.0, Ki0=1.0, Kd0=0.5, dt=Ts)

y_fixed, u_fixed = run_simulation(pid_fixed, is_adaptive=False)
y_adapt, u_adapt = run_simulation(pid_adapt, is_adaptive=True)

# ── 性能指标 ──
iae_fixed = np.sum(np.abs(r - y_fixed)) * Ts
iae_adapt = np.sum(np.abs(r - y_adapt)) * Ts

# ── 绘图 ──
fig, axes = plt.subplots(3, 2, figsize=(14, 10))
fig.suptitle('自适应PID vs 固定参数PID 仿真对比', fontsize=14, fontweight='bold')

# (0,0) 阶跃响应对比
axes[0, 0].plot(t, r, 'k--', lw=1, label='设定值')
axes[0, 0].plot(t, y_fixed, 'r-', lw=1.2, label=f'固定PID (IAE={iae_fixed:.3f})')
axes[0, 0].plot(t, y_adapt, 'b-', lw=1.2, label=f'自适应PID (IAE={iae_adapt:.3f})')
axes[0, 0].axvline(5, color='gray', ls=':', alpha=0.5, label='负载扰动')
axes[0, 0].axvline(10, color='gray', ls='--', alpha=0.5, label='参数突变')
axes[0, 0].set_ylabel('输出 y(t)')
axes[0, 0].set_title('阶跃响应对比')
axes[0, 0].legend(fontsize=8)
axes[0, 0].grid(True, alpha=0.3)

# (0,1) 误差对比
axes[0, 1].plot(t, r - y_fixed, 'r-', lw=1, label='固定PID误差')
axes[0, 1].plot(t, r - y_adapt, 'b-', lw=1, label='自适应PID误差')
axes[0, 1].set_ylabel('误差 e(t)')
axes[0, 1].set_title('跟踪误差对比')
axes[0, 1].legend(fontsize=8)
axes[0, 1].grid(True, alpha=0.3)

# (1,0) 控制量对比
axes[1, 0].plot(t, u_fixed, 'r-', lw=1, label='固定PID')
axes[1, 0].plot(t, u_adapt, 'b-', lw=1, label='自适应PID')
axes[1, 0].set_ylabel('控制量 u(t)')
axes[1, 0].set_title('控制量对比')
axes[1, 0].legend(fontsize=8)
axes[1, 0].grid(True, alpha=0.3)

# (1,1) Kp自适应曲线
axes[1, 1].plot(t, np.ones(N)*2.0, 'r--', lw=1, label='固定Kp=2.0')
axes[1, 1].plot(t[:len(pid_adapt.Kp_hist)], pid_adapt.Kp_hist, 'b-', lw=1.2, label='自适应Kp')
axes[1, 1].set_ylabel('Kp')
axes[1, 1].set_title('比例增益自适应过程')
axes[1, 1].legend(fontsize=8)
axes[1, 1].grid(True, alpha=0.3)

# (2,0) Ki自适应曲线
axes[2, 0].plot(t, np.ones(N)*1.0, 'r--', lw=1, label='固定Ki=1.0')
axes[2, 0].plot(t[:len(pid_adapt.Ki_hist)], pid_adapt.Ki_hist, 'b-', lw=1.2, label='自适应Ki')
axes[2, 0].set_xlabel('时间 (s)')
axes[2, 0].set_ylabel('Ki')
axes[2, 0].set_title('积分增益自适应过程')
axes[2, 0].legend(fontsize=8)
axes[2, 0].grid(True, alpha=0.3)

# (2,1) Kd自适应曲线
axes[2, 1].plot(t, np.ones(N)*0.5, 'r--', lw=1, label='固定Kd=0.5')
axes[2, 1].plot(t[:len(pid_adapt.Kd_hist)], pid_adapt.Kd_hist, 'b-', lw=1.2, label='自适应Kd')
axes[2, 1].set_xlabel('时间 (s)')
axes[2, 1].set_ylabel('Kd')
axes[2, 1].set_title('微分增益自适应过程')
axes[2, 1].legend(fontsize=8)
axes[2, 1].grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'adaptive_pid_result.png'), dpi=150, bbox_inches='tight')
plt.close('all')
print(f"✅ 自适应PID仿真完成")
print(f"   固定PID  IAE = {iae_fixed:.4f}")
print(f"   自适应PID IAE = {iae_adapt:.4f}")
print(f"   性能提升 = {(1-iae_adapt/iae_fixed)*100:.1f}%")

if __name__ == '__main__':
    run_simulation()
