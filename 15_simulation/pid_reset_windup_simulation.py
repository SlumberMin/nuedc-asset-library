#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
积分重置 / 抗积分饱和 (Anti-Windup) 仿真
==========================================
仿真内容:
  1. 无抗饱和: 积分饱和导致大幅超调
  2. 限幅钳位 (Clamping): 输出饱和时冻结积分
  3. 条件积分 (Conditional Integration): 仅在未饱和或误差方向有利时积分
  4. Back-Calculation: 反馈实际输出与限幅差值修正积分项
  5. 积分重置: 阶跃变化时清零积分

作者: nuedc-asset-library
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
plt.rcParams['axes.unicode_minus'] = False


class FirstOrderPlant:
    def __init__(self, K=1.0, tau=0.5, dt=0.001):
        self.K, self.tau, self.dt = K, tau, dt
        self.y = 0.0

    def update(self, u):
        self.y += (self.dt / self.tau) * (self.K * u - self.y)
        return self.y


class PIDAntiWindup:
    """支持多种抗积分饱和策略的PID"""

    def __init__(self, Kp, Ki, Kd, dt, u_min=-10, u_max=10,
                 mode='none', kb=0.5):
        self.Kp, self.Ki, self.Kd, self.dt = Kp, Ki, Kd, dt
        self.u_min, self.u_max = u_min, u_max
        self.mode = mode  # none / clamping / conditional / backcalc / reset
        self.kb = kb  # back-calculation 增益
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_saturated = False

    def update(self, error, setpoint_changed=False):
        # P
        p = self.Kp * error
        # D
        d = self.Kd * (error - self.prev_error) / self.dt
        self.prev_error = error

        # ── 积分策略 ──
        if self.mode == 'none':
            # 无抗饱和
            self.integral += error * self.dt

        elif self.mode == 'clamping':
            # 钳位: 输出饱和且误差方向会加剧饱和时冻结积分
            u_unsat = p + self.Ki * self.integral + d
            saturated = (u_unsat > self.u_max) or (u_unsat < self.u_min)
            if not (saturated and (error * u_unsat > 0)):
                self.integral += error * self.dt

        elif self.mode == 'conditional':
            # 条件积分: 仅当未饱和，或误差使输出远离饱和方向时积分
            u_unsat = p + self.Ki * self.integral + d
            if (u_unsat < self.u_max and error > 0) or \
               (u_unsat > self.u_min and error < 0) or \
               (self.u_min <= u_unsat <= self.u_max):
                self.integral += error * self.dt

        elif self.mode == 'backcalc':
            # Back-Calculation
            self.integral += error * self.dt
            u_unsat = p + self.Ki * self.integral + d
            u_sat = np.clip(u_unsat, self.u_min, self.u_max)
            # 用限幅差修正积分
            self.integral += self.kb * (u_sat - u_unsat) * self.dt

        elif self.mode == 'reset':
            # 积分重置: 阶跃变化时清零
            if setpoint_changed:
                self.integral = 0.0
            else:
                self.integral += error * self.dt

        i = self.Ki * self.integral
        u = p + i + d
        return np.clip(u, self.u_min, self.u_max)


def run_sim(mode, T=8.0, dt=0.001):
    n = int(T / dt)
    t = np.arange(n) * dt
    plant = FirstOrderPlant(K=1.0, tau=0.5, dt=dt)
    pid = PIDAntiWindup(Kp=1.0, Ki=8.0, Kd=0.05, dt=dt,
                         u_min=-5, u_max=5, mode=mode, kb=2.0)
    ref = np.zeros(n)
    ref[0:int(3/dt)] = 1.0          # 0~3s: 目标1
    ref[int(3/dt):int(5/dt)] = 0.2   # 3~5s: 目标0.2 (大幅下降)
    ref[int(5/dt):] = 1.5            # 5~8s: 目标1.5

    y, u_arr = np.zeros(n), np.zeros(n)
    for i in range(n):
        sp_changed = (i > 0) and (abs(ref[i] - ref[i-1]) > 0.01)
        error = ref[i] - plant.y
        u_arr[i] = pid.update(error, setpoint_changed=sp_changed)
        y[i] = plant.update(u_arr[i])

    return t, ref, y, u_arr


if __name__ == '__main__':
    modes = {
        'none':        '无抗饱和',
        'clamping':    '钳位(Clamping)',
        'conditional': '条件积分',
        'backcalc':    'Back-Calculation',
        'reset':       '积分重置',
    }

    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    for mode, label in modes.items():
        t, ref, y, u_arr = run_sim(mode)
        axes[0].plot(t, y, label=label, linewidth=1.0)
        axes[1].plot(t, u_arr, label=label, linewidth=0.8)

    axes[0].plot(t, ref, 'k--', alpha=0.4, label='参考值')
    axes[0].set_ylabel('输出 y')
    axes[0].set_title('积分抗饱和策略对比 (Ki=8, u∈[-5,5])')
    axes[0].legend(fontsize=9)
    axes[0].grid(True, alpha=0.3)

    axes[1].set_ylabel('控制量 u')
    axes[1].set_xlabel('时间 (s)')
    axes[1].legend(fontsize=9)
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('pid_anti_windup.png', dpi=150)
    print('[OK] 图像已保存: pid_anti_windup.png')

    # 定量分析
    print('\n=== 性能指标 ===')
    for mode, label in modes.items():
        t, ref, y, u_arr = run_sim(mode)
        overshoot_1 = (np.max(y[:int(3/0.001)]) - 1.0) / 1.0 * 100
        overshoot_2 = (np.max(y[int(5/0.001):]) - 1.5) / 1.5 * 100
        print(f'  {label:20s}  阶跃1超调: {overshoot_1:5.1f}%  阶跃2超调: {overshoot_2:5.1f}%')

    plt.close('all')
