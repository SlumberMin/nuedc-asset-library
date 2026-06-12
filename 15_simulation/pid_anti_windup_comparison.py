# -*- coding: utf-8 -*-
"""
PID抗饱和方法对比仿真
======================
对比5种常见抗积分饱和（Anti-Windup）策略：
1. 积分限幅 (Clamping)
2. 条件积分 (Conditional Integration)
3. 反馈退饱和 (Back-Calculation)
4. 积分泄漏 (Integral Decay)
5. 无抗饱和（对照组）

用法：python pid_anti_windup_comparison.py
"""

import numpy as np
import matplotlib.pyplot as plt
import os

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

# ── 被控对象：一阶惯性 + 执行器饱和 ─────────────────────
class Plant:
    """一阶系统 y' = (K*u - y) / tau, 执行器饱和 ±U_MAX"""
    def __init__(self, K=1.0, tau=1.0, u_max=10.0, dt=0.01):
        self.K = K
        self.tau = tau
        self.u_max = u_max
        self.dt = dt
        self.y = 0.0

    def update(self, u_cmd):
        u_sat = np.clip(u_cmd, -self.u_max, self.u_max)
        self.y += (self.K * u_sat - self.y) / self.tau * self.dt
        return self.y, u_sat

# ── PID控制器 ─────────────────────────────────────────────
class PIDController:
    def __init__(self, kp, ki, kd, dt, u_max=10.0, anti_windup='none'):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.u_max = u_max
        self.anti_windup = anti_windup

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_u_sat = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_u_sat = 0.0

    def compute(self, error):
        # 积分
        self.integral += error * self.dt

        # ── 抗饱和策略 ──
        u_raw = self.kp * error + self.ki * self.integral
        u_sat = np.clip(u_raw, -self.u_max, self.u_max)

        if self.anti_windup == 'clamping':
            # 1. 积分限幅：限制积分项
            max_integral = self.u_max / self.ki if self.ki > 0 else 1e10
            self.integral = np.clip(self.integral, -max_integral, max_integral)

        elif self.anti_windup == 'conditional':
            # 2. 条件积分：饱和时停止积分
            if (u_sat >= self.u_max and error > 0) or (u_sat <= -self.u_max and error < 0):
                self.integral -= error * self.dt  # 撤销本次积分

        elif self.anti_windup == 'back_calculation':
            # 3. 反馈退饱和：用饱和差修正积分
            kb = 0.5  # 退饱和增益
            self.integral += kb * (u_sat - u_raw) * self.dt

        elif self.anti_windup == 'decay':
            # 4. 积分泄漏：指数衰减
            decay = 0.99
            self.integral *= decay

        # 重新计算输出
        self.prev_error = error
        u_raw = self.kp * error + self.ki * self.integral
        u_sat = np.clip(u_raw, -self.u_max, self.u_max)
        self.prev_u_sat = u_sat
        return u_raw, u_sat

# ── 仿真 ──────────────────────────────────────────────────
DT = 0.01
T_END = 10.0
N = int(T_END / DT)
t = np.linspace(0, T_END, N)

# 阶跃设定值（大幅值确保饱和）
setpoint = np.ones(N) * 50.0

methods = {
    '无抗饱和':          'none',
    '积分限幅':          'clamping',
    '条件积分':          'conditional',
    '反馈退饱和':        'back_calculation',
    '积分泄漏':          'decay',
}

colors = ['#F44336', '#2196F3', '#4CAF50', '#FF9800', '#9C27B0']

results = {}
for name, method in methods.items():
    plant = Plant(K=2.0, tau=1.0, u_max=10.0, dt=DT)
    pid = PIDController(kp=2.0, ki=5.0, kd=0.1, dt=DT, u_max=10.0, anti_windup=method)

    outputs = []
    u_commands = []
    integrals = []
    for i in range(N):
        error = setpoint[i] - plant.y
        u_raw, u_sat = pid.compute(error)
        y, _ = plant.update(u_raw)
        outputs.append(y)
        u_commands.append(u_sat)
        integrals.append(pid.integral)
    results[name] = (np.array(outputs), np.array(u_commands), np.array(integrals))

# ── 绘图 ──────────────────────────────────────────────────
fig, axes = plt.subplots(3, 1, figsize=(14, 11), sharex=True)

# 子图1：系统输出
ax = axes[0]
ax.plot(t, setpoint, 'k--', lw=2, label='设定值')
for (name, (y, _, _)), color in zip(results.items(), colors):
    ax.plot(t, y, color=color, lw=1.5, label=name)
ax.set_ylabel('输出')
ax.set_title('PID抗饱和方法对比 — 系统响应')
ax.legend(loc='lower right', fontsize=9)
ax.grid(True, alpha=0.3)

# 子图2：控制量
ax = axes[1]
for (name, (_, u, _)), color in zip(results.items(), colors):
    ax.plot(t, u, color=color, lw=1, alpha=0.8, label=name)
ax.axhline(10, color='gray', ls=':', lw=1)
ax.axhline(-10, color='gray', ls=':', lw=1)
ax.set_ylabel('控制量')
ax.set_title('控制量输出（执行器饱和 ±10）')
ax.legend(loc='upper right', fontsize=9)
ax.grid(True, alpha=0.3)

# 子图3：积分项
ax = axes[2]
for (name, (_, _, integ)), color in zip(results.items(), colors):
    ax.plot(t, integ, color=color, lw=1, alpha=0.8, label=name)
ax.set_xlabel('时间 (s)')
ax.set_ylabel('积分项')
ax.set_title('积分项累积对比（Windup可见）')
ax.legend(loc='upper right', fontsize=9)
ax.grid(True, alpha=0.3)

plt.tight_layout()
out_dir = os.path.dirname(os.path.abspath(__file__))
out_path = os.path.join(out_dir, 'pid_anti_windup_comparison.png')
plt.savefig(out_path, dpi=150, bbox_inches='tight')
print(f"图表已保存: {out_path}")
plt.show()

# 性能指标
print("\n" + "=" * 65)
print("PID抗饱和方法 — 性能对比")
print("=" * 65)
print(f"{'方法':>14} {'上升时间(s)':>12} {'超调量(%)':>10} {'调节时间(s)':>12} {'最终误差':>10}")
print("-" * 65)
for name, (y, _, _) in results.items():
    # 上升时间 (10%~90%)
    idx_10 = np.argmax(y > 0.1 * 50)
    idx_90 = np.argmax(y > 0.9 * 50)
    rise_time = (idx_90 - idx_10) * DT if idx_90 > idx_10 else float('nan')
    # 超调
    overshoot = (np.max(y) - 50) / 50 * 100
    # 调节时间 (±2%)
    settled = N
    for i in range(N-1, 0, -1):
        if abs(y[i] - 50) > 0.02 * 50:
            settled = i + 1
            break
    settle_time = settled * DT
    final_err = abs(y[-1] - 50)
    print(f"{name:>14} {rise_time:>12.2f} {overshoot:>10.1f} {settle_time:>12.2f} {final_err:>10.2f}")

print("\n建议：")
print("  - 简单场景用'条件积分'即可")
print("  - 高性能场景用'反馈退饱和'（需调 kb）")
print("  - '积分限幅'最简单但效果一般")
print("  - '积分泄漏'适合持续扰动场景")
