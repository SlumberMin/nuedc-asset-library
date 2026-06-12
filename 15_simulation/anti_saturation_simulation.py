#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
抗饱和策略对比仿真
==================
对比: 无抗饱和 / 积分限幅 / 条件积分 / 反馈抗饱和(back-calculation)
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
plt.rcParams['axes.unicode_minus'] = False


class IntegratorAntiWindup:
    """带抗饱和的积分器"""
    def __init__(self, dt, limits=(-100, 100), method='none'):
        """
        method: 'none'/'clamp'/'conditional'/'back_calc'
        """
        self.dt = dt
        self.limits = limits
        self.method = method
        self.state = 0.0

    def update(self, input_val, output_clamped=None):
        if self.method == 'none':
            self.state += input_val * self.dt
        elif self.method == 'clamp':
            self.state += input_val * self.dt
            self.state = np.clip(self.state, self.limits[0], self.limits[1])
        elif self.method == 'conditional':
            # 只在输出未饱和或误差方向使输出减小时积分
            self.state += input_val * self.dt
        elif self.method == 'back_calc':
            # 反馈抗饱和: Ki*e + Ka*(u_sat - u_unsat)
            Ka = 5.0
            self.state += (input_val + Ka * (output_clamped - input_val)) * self.dt
        return self.state

    def reset(self):
        self.state = 0.0


def simulate_pid_with_antiwindup(method, Kp=2.0, Ki=10.0, Kd=0.5,
                                  dt=0.01, T=8.0, setpoint=1.0):
    """仿真带抗饱和的PID"""
    t = np.arange(0, T, dt)
    y_list, u_list, u_sat_list = [], [], []

    # 被控对象: 一阶惯性 G(s)=1/(0.5s+1)
    y = 0.0
    prev_e = 0.0
    integ = IntegratorAntiWindup(dt, limits=(-5, 5), method=method)
    u_limits = (-5, 5)  # 执行器饱和限

    for ti in t:
        e = setpoint - y
        P = Kp * e
        D = Kd * (e - prev_e) / dt
        I_state = integ.state + Ki * e * dt

        u_unsat = P + Ki * integ.state + D
        u_sat = np.clip(u_unsat, u_limits[0], u_limits[1])

        if method == 'none':
            integ.state += e * dt
        elif method == 'clamp':
            integ.state += e * dt
            integ.state = np.clip(integ.state, -5/Ki, 5/Ki)
        elif method == 'conditional':
            if (u_unsat >= u_limits[1] and e > 0) or (u_unsat <= u_limits[0] and e < 0):
                pass  # 不积分
            else:
                integ.state += e * dt
        elif method == 'back_calc':
            Ka = 2.0
            integ.state += (e + Ka * (u_sat - u_unsat)) * dt

        # 重新计算实际输出
        u_actual = np.clip(P + Ki * integ.state + D, u_limits[0], u_limits[1])

        # 被控对象更新 (带大惯性模拟饱和场景)
        tau = 0.8
        y += (u_actual - y) / tau * dt

        prev_e = e
        y_list.append(y)
        u_list.append(u_actual)

    return t, y_list, u_list


def main():
    methods = {
        'none':        '无抗饱和',
        'clamp':       '积分限幅',
        'conditional': '条件积分',
        'back_calc':   '反馈抗饱和',
    }

    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    t_ref = None

    for method, label in methods.items():
        t, y, u = simulate_pid_with_antiwindup(method)
        t_ref = t
        axes[0].plot(t, y, label=label)
        axes[1].plot(t, u, label=label)

    axes[0].axhline(y=1.0, color='k', ls='--', lw=1.5, label='设定值')
    axes[0].set_ylabel('输出')
    axes[0].set_title('抗饱和策略对比 - 系统响应')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].axhline(y=5, color='r', ls=':', lw=1, alpha=0.5)
    axes[1].axhline(y=-5, color='r', ls=':', lw=1, alpha=0.5)
    axes[1].set_xlabel('时间 (s)')
    axes[1].set_ylabel('控制量')
    axes[1].set_title('控制量 (饱和限 ±5)')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('anti_saturation_result.png', dpi=150)
    plt.close('all')


if __name__ == '__main__':
    main()
