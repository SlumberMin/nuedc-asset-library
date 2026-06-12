#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
轨迹跟踪仿真 - 梯形/S曲线轨迹规划 + PID跟踪
=============================================
功能:
  1. 生成梯形速度曲线轨迹
  2. 生成S曲线(7段)轨迹
  3. PID跟踪 + 跟踪误差分析
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
plt.rcParams['axes.unicode_minus'] = False


def trapezoidal_profile(q0, qf, v_max, a_max, dt, t_total):
    """梯形速度曲线: 三段(加速-匀速-减速)"""
    # 计算各段时间
    t_accel = v_max / a_max
    d_accel = 0.5 * a_max * t_accel**2
    d_total = qf - q0
    d_cruise = d_total - 2 * d_accel

    if d_cruise < 0:
        # 距离不够, 三角形曲线
        t_accel = np.sqrt(d_total / a_max)
        t_cruise = 0
    else:
        t_cruise = d_cruise / v_max

    t_decel = t_accel
    t_sum = t_accel + t_cruise + t_decel

    t = np.arange(0, t_total, dt)
    pos, vel, acc = [], [], []

    for ti in t:
        if ti < t_accel:
            a = a_max
            v = a_max * ti
            p = q0 + 0.5 * a_max * ti**2
        elif ti < t_accel + t_cruise:
            a = 0
            v = v_max
            p = q0 + d_accel + v_max * (ti - t_accel)
        elif ti < t_sum:
            a = -a_max
            v = v_max - a_max * (ti - t_accel - t_cruise)
            p = qf - 0.5 * a_max * (t_sum - ti)**2
        else:
            a = 0
            v = 0
            p = qf
        pos.append(p)
        vel.append(v)
        acc.append(a)

    return t, np.array(pos), np.array(vel), np.array(acc)


def s_curve_profile(q0, qf, v_max, a_max, j_max, dt, t_total):
    """S曲线(7段)轨迹规划"""
    # 简化: 用多项式近似S曲线
    # q(t) = q0 + (qf-q0) * [10(t/T)^3 - 15(t/T)^4 + 6(t/T)^5]
    T = t_total * 0.8
    t = np.arange(0, t_total, dt)
    pos, vel, acc = [], [], []

    for ti in t:
        tau = np.clip(ti / T, 0, 1)
        # 5次多项式
        s = 10*tau**3 - 15*tau**4 + 6*tau**5
        ds = (30*tau**2 - 60*tau**3 + 30*tau**4) / T
        dds = (60*tau - 180*tau**2 + 120*tau**3) / T**2
        pos.append(q0 + (qf - q0) * s)
        vel.append((qf - q0) * ds)
        acc.append((qf - q0) * dds)

    return t, np.array(pos), np.array(vel), np.array(acc)


class PID:
    def __init__(self, kp, ki, kd, dt):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dt = dt
        self.ei = 0.0
        self.ep = 0.0

    def compute(self, ref, fb):
        e = ref - fb
        self.ei += e * self.dt
        ed = (e - self.ep) / self.dt
        self.ep = e
        return self.kp * e + self.ki * self.ei + self.kd * ed

    def reset(self):
        self.ei = self.ep = 0.0


def track_trajectory(t, ref_pos, ref_vel, kp=50, ki=10, kd=5):
    """一阶系统跟踪轨迹"""
    pid = PID(kp, ki, kd, t[1] - t[0])
    y = 0.0
    y_list, e_list = [], []
    for i, ti in enumerate(t):
        u = pid.compute(ref_pos[i], y) + 0.3 * ref_vel[i]  # 前馈
        y += (u - y) * (t[1] - t[0]) / 0.3
        y_list.append(y)
        e_list.append(ref_pos[i] - y)
    return np.array(y_list), np.array(e_list)


def main():
    dt = 0.005
    T = 3.0
    q0, qf = 0.0, 100.0

    # 梯形轨迹
    t1, pos1, vel1, acc1 = trapezoidal_profile(q0, qf, v_max=60, a_max=200, dt=dt, t_total=T)
    # S曲线轨迹
    t2, pos2, vel2, acc2 = s_curve_profile(q0, qf, v_max=60, a_max=200, j_max=500, dt=dt, t_total=T)

    # 跟踪
    y_trap, e_trap = track_trajectory(t1, pos1, vel1)
    y_s, e_s = track_trajectory(t2, pos2, vel2)

    fig, axes = plt.subplots(3, 2, figsize=(14, 10))

    # 梯形
    axes[0, 0].plot(t1, pos1, 'b-', label='参考')
    axes[0, 0].plot(t1, y_trap, 'r--', label='跟踪')
    axes[0, 0].set_title('梯形轨迹 - 位置')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)

    axes[1, 0].plot(t1, vel1)
    axes[1, 0].set_title('梯形轨迹 - 速度')
    axes[1, 0].grid(True, alpha=0.3)

    axes[2, 0].plot(t1, e_trap)
    axes[2, 0].set_title('梯形轨迹 - 跟踪误差')
    axes[2, 0].set_xlabel('时间 (s)')
    axes[2, 0].grid(True, alpha=0.3)

    # S曲线
    axes[0, 1].plot(t2, pos2, 'b-', label='参考')
    axes[0, 1].plot(t2, y_s, 'r--', label='跟踪')
    axes[0, 1].set_title('S曲线轨迹 - 位置')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)

    axes[1, 1].plot(t2, vel2)
    axes[1, 1].set_title('S曲线轨迹 - 速度')
    axes[1, 1].grid(True, alpha=0.3)

    axes[2, 1].plot(t2, e_s)
    axes[2, 1].set_title('S曲线轨迹 - 跟踪误差')
    axes[2, 1].set_xlabel('时间 (s)')
    axes[2, 1].grid(True, alpha=0.3)

    plt.suptitle('轨迹跟踪仿真', fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig('trajectory_tracking_result.png', dpi=150)
    plt.close('all')

    print(f"梯形轨迹最大跟踪误差: {np.max(np.abs(e_trap)):.4f}")
    print(f"S曲线最大跟踪误差:   {np.max(np.abs(e_s)):.4f}")
    print(f"梯形轨迹RMSE: {np.sqrt(np.mean(e_trap**2)):.4f}")
    print(f"S曲线RMSE:   {np.sqrt(np.mean(e_s**2)):.4f}")


if __name__ == '__main__':
    main()
