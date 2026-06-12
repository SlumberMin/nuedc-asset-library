#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
运动控制仿真 - 速度规划与路径插补
====================================
仿真内容:
  1. 梯形速度规划 vs S形速度规划 对比
  2. 三次多项式轨迹规划
  3. 点到点运动控制 (规划+PID闭环)
  4. 2D连续路径插补 (直线+圆弧)
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
matplotlib.rcParams['axes.unicode_minus'] = False


class TrapezoidalProfile:
    """梯形速度规划"""

    def __init__(self, q0, qf, v_max, a_max, dt):
        self.q0 = q0
        self.qf = qf
        self.v_max = abs(v_max)
        self.a_max = abs(a_max)
        self.dt = dt
        self.t = 0.0
        self.dir = 1.0 if qf >= q0 else -1.0
        self.dist = abs(qf - q0)

        t_acc = self.v_max / self.a_max
        d_acc = 0.5 * self.a_max * t_acc ** 2

        if 2 * d_acc > self.dist:
            t_acc = np.sqrt(self.dist / self.a_max)
            self.T_accel = t_acc
            self.T_const = 0.0
            self.T_decel = t_acc
        else:
            self.T_accel = t_acc
            self.T_decel = t_acc
            self.T_const = (self.dist - 2 * d_acc) / self.v_max

        self.T_total = self.T_accel + self.T_const + self.T_decel

    def get_state(self, t):
        t = min(t, self.T_total)
        if t < self.T_accel:
            pos = 0.5 * self.a_max * t ** 2
            vel = self.a_max * t
            acc = self.a_max
        elif t < self.T_accel + self.T_const:
            tc = t - self.T_accel
            d_acc = 0.5 * self.a_max * self.T_accel ** 2
            pos = d_acc + self.v_max * tc
            vel = self.v_max
            acc = 0.0
        else:
            td = t - self.T_accel - self.T_const
            d_acc = 0.5 * self.a_max * self.T_accel ** 2
            pos = d_acc + self.v_max * self.T_const + self.v_max * td - 0.5 * self.a_max * td ** 2
            vel = self.v_max - self.a_max * td
            acc = -self.a_max

        return self.q0 + self.dir * pos, self.dir * vel, self.dir * acc


class SCurveProfile:
    """S形速度规划 (简化版: 梯形加速度)"""

    def __init__(self, q0, qf, v_max, a_max, j_max, dt):
        self.q0 = q0
        self.qf = qf
        self.v_max = abs(v_max)
        self.a_max = abs(a_max)
        self.j_max = abs(j_max)
        self.dt = dt
        self.dir = 1.0 if qf >= q0 else -1.0
        self.dist = abs(qf - qf + q0) if False else abs(qf - q0)

        # 7段时间
        Tj = self.a_max / self.j_max
        Ta = self.v_max / self.a_max

        # 检查是否能达到最大速度
        d_acc = self.v_max * Ta
        if 2 * d_acc > self.dist:
            self.v_max = np.sqrt(self.dist * self.a_max)
            d_acc = self.v_max * self.v_max / self.a_max
            Ta = self.v_max / self.a_max

        self.T = [Tj, Ta - Tj, Tj, 0.0, Tj, Ta - Tj, Tj]
        d_const = self.dist - 2 * d_acc
        self.T[3] = max(0, d_const / self.v_max) if self.v_max > 0 else 0
        self.T_total = sum(self.T)

    def get_state(self, t):
        t = min(t, self.T_total)
        # 简化: 用数值积分
        cumT = [0]
        for ti in self.T:
            cumT.append(cumT[-1] + ti)

        # 分段计算加速度
        if t < cumT[1]:
            acc = self.j_max * t
        elif t < cumT[2]:
            acc = self.a_max
        elif t < cumT[3]:
            acc = self.a_max - self.j_max * (t - cumT[2])
        elif t < cumT[4]:
            acc = 0.0
        elif t < cumT[5]:
            acc = -self.j_max * (t - cumT[4])
        elif t < cumT[6]:
            acc = -self.a_max
        elif t < cumT[7]:
            acc = -self.a_max + self.j_max * (t - cumT[6])
        else:
            acc = 0.0

        # 数值积分求速度和位置
        n = int(t / self.dt) + 1
        vel, pos = 0.0, 0.0
        for i in range(n):
            ti = i * self.dt
            if ti > t:
                break
            if ti < cumT[1]:
                a = self.j_max * ti
            elif ti < cumT[2]:
                a = self.a_max
            elif ti < cumT[3]:
                a = self.a_max - self.j_max * (ti - cumT[2])
            elif ti < cumT[4]:
                a = 0.0
            elif ti < cumT[5]:
                a = -self.j_max * (ti - cumT[4])
            elif ti < cumT[6]:
                a = -self.a_max
            elif ti < cumT[7]:
                a = -self.a_max + self.j_max * (ti - cumT[6])
            else:
                a = 0.0
            vel += a * self.dt
            pos += vel * self.dt

        return self.q0 + self.dir * pos, self.dir * vel, self.dir * acc


class CubicPolynomial:
    """三次多项式轨迹"""

    def __init__(self, q0, dq0, qf, dqf, T, dt):
        self.T = T
        self.dt = dt
        self.a0 = q0
        self.a1 = dq0
        self.a2 = (3 * (qf - q0) - (2 * dq0 + dqf) * T) / (T ** 2)
        self.a3 = (2 * (q0 - qf) + (dq0 + dqf) * T) / (T ** 3)

    def get_state(self, t):
        t = min(t, self.T)
        q = self.a0 + self.a1 * t + self.a2 * t ** 2 + self.a3 * t ** 3
        dq = self.a1 + 2 * self.a2 * t + 3 * self.a3 * t ** 2
        ddq = 2 * self.a2 + 6 * self.a3 * t
        return q, dq, ddq


class PIDController:
    """PID控制器"""

    def __init__(self, Kp, Ki, Kd, dt):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.err_sum = 0.0
        self.err_prev = 0.0

    def update(self, ref, meas):
        err = ref - meas
        self.err_sum += err * self.dt
        d_err = (err - self.err_prev) / self.dt
        self.err_prev = err
        return self.Kp * err + self.Ki * self.err_sum + self.Kd * d_err


def simulate_profiles():
    """对比梯形和S形速度规划"""
    dt = 0.001
    trap = TrapezoidalProfile(0, 10, v_max=5.0, a_max=10.0, dt=dt)
    scurve = SCurveProfile(0, 10, v_max=5.0, a_max=10.0, j_max=50.0, dt=dt)

    t_arr = np.arange(0, trap.T_total + 0.5, dt)
    trap_pos, trap_vel, trap_acc = [], [], []
    s_pos, s_vel, s_acc = [], [], []

    for t in t_arr:
        p, v, a = trap.get_state(t)
        trap_pos.append(p)
        trap_vel.append(v)
        trap_acc.append(a)
        p, v, a = scurve.get_state(t)
        s_pos.append(p)
        s_vel.append(v)
        s_acc.append(a)

    fig, axes = plt.subplots(3, 1, figsize=(10, 8))
    fig.suptitle('梯形 vs S形速度规划对比', fontsize=14)

    axes[0].plot(t_arr, trap_pos, 'b-', label='梯形')
    axes[0].plot(t_arr, s_pos, 'r--', label='S形')
    axes[0].set_ylabel('位置')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t_arr, trap_vel, 'b-', label='梯形')
    axes[1].plot(t_arr, s_vel, 'r--', label='S形')
    axes[1].set_ylabel('速度')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(t_arr, trap_acc, 'b-', label='梯形')
    axes[2].plot(t_arr, s_acc, 'r--', label='S形')
    axes[2].set_ylabel('加速度')
    axes[2].set_xlabel('时间 (s)')
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('profile_comparison.png', dpi=150)
    plt.close('all')
    print("速度规划对比仿真完成, 图片已保存: profile_comparison.png")


def simulate_cubic_polynomial():
    """三次多项式轨迹仿真"""
    dt = 0.001
    T = 2.0
    cubic = CubicPolynomial(q0=0, dq0=0, qf=10, dqf=0, T=T, dt=dt)

    t_arr = np.arange(0, T + dt, dt)
    pos, vel, acc = [], [], []
    for t in t_arr:
        q, dq, ddq = cubic.get_state(t)
        pos.append(q)
        vel.append(dq)
        acc.append(ddq)

    fig, axes = plt.subplots(3, 1, figsize=(10, 7))
    fig.suptitle('三次多项式轨迹规划', fontsize=14)

    axes[0].plot(t_arr, pos, 'b-')
    axes[0].set_ylabel('位置')
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t_arr, vel, 'g-')
    axes[1].set_ylabel('速度')
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(t_arr, acc, 'r-')
    axes[2].set_ylabel('加速度')
    axes[2].set_xlabel('时间 (s)')
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('cubic_polynomial.png', dpi=150)
    plt.close('all')
    print("三次多项式轨迹仿真完成, 图片已保存: cubic_polynomial.png")


def simulate_ptp():
    """点到点运动控制 (规划+PID闭环)"""
    dt = 0.001
    T = 3.0

    trap = TrapezoidalProfile(0, 5, v_max=3.0, a_max=10.0, dt=dt)
    pid = PIDController(Kp=50.0, Ki=10.0, Kd=5.0, dt=dt)

    # 二阶系统模型
    m, b = 1.0, 2.0
    x, dx = 0.0, 0.0

    t_arr = np.arange(0, T, dt)
    ref_arr, pos_arr, ctrl_arr = [], [], []

    for t in t_arr:
        ref, _, _ = trap.get_state(t)
        u = pid.update(ref, x)
        ddx = (u - b * dx) / m
        dx += ddx * dt
        x += dx * dt
        ref_arr.append(ref)
        pos_arr.append(x)
        ctrl_arr.append(u)

    fig, axes = plt.subplots(2, 1, figsize=(10, 6))
    fig.suptitle('点到点运动控制 (梯形规划 + PID)', fontsize=14)

    axes[0].plot(t_arr, ref_arr, 'b--', label='参考轨迹')
    axes[0].plot(t_arr, pos_arr, 'r-', label='实际轨迹')
    axes[0].set_ylabel('位置')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t_arr, ctrl_arr, 'm-')
    axes[1].set_ylabel('控制量')
    axes[1].set_xlabel('时间 (s)')
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('ptp_control.png', dpi=150)
    plt.close('all')
    print("点到点控制仿真完成, 图片已保存: ptp_control.png")


def simulate_2d_path():
    """2D连续路径插补仿真 (直线+圆弧)"""
    dt = 0.01
    feed_rate = 2.0

    # 路径: 直线1 → 圆弧 → 直线2
    line1_start = np.array([0.0, 0.0])
    line1_end = np.array([5.0, 0.0])
    arc_center = np.array([5.0, 3.0])
    arc_radius = 3.0
    line2_start = np.array([5.0 + 3.0, 3.0])  # 圆弧终点
    line2_end = np.array([11.0, 3.0])

    path_x, path_y = [], []

    # 直线1
    t = 0
    while True:
        ratio = min(feed_rate * t * dt / np.linalg.norm(line1_end - line1_start), 1.0)
        p = line1_start + ratio * (line1_end - line1_start)
        path_x.append(p[0])
        path_y.append(p[1])
        if ratio >= 1.0:
            break
        t += 1

    # 圆弧 (从角度-π/2到0, 顺时针)
    arc_len = abs(-np.pi / 2) * arc_radius
    t = 0
    while True:
        dist = feed_rate * t * dt
        if dist >= arc_len:
            break
        angle = -np.pi / 2 + (dist / arc_radius)
        x = arc_center[0] + arc_radius * np.cos(angle)
        y = arc_center[1] + arc_radius * np.sin(angle)
        path_x.append(x)
        path_y.append(y)
        t += 1

    # 直线2
    t = 0
    while True:
        ratio = min(feed_rate * t * dt / np.linalg.norm(line2_end - line2_start), 1.0)
        p = line2_start + ratio * (line2_end - line2_start)
        path_x.append(p[0])
        path_y.append(p[1])
        if ratio >= 1.0:
            break
        t += 1

    fig, ax = plt.subplots(1, 1, figsize=(10, 6))
    ax.plot(path_x, path_y, 'b-', linewidth=2, label='规划路径')
    ax.plot(line1_start[0], line1_start[1], 'go', markersize=10, label='起点')
    ax.plot(line2_end[0], line2_end[1], 'rs', markersize=10, label='终点')

    # 绘制圆弧中心
    ax.plot(arc_center[0], arc_center[1], 'k+', markersize=10)
    circle = plt.Circle(arc_center, arc_radius, fill=False, color='gray', linestyle='--')
    ax.add_patch(circle)

    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_title('2D连续路径插补 (直线+圆弧)')
    ax.set_aspect('equal')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('2d_path_interpolation.png', dpi=150)
    plt.close('all')
    print("2D路径插补仿真完成, 图片已保存: 2d_path_interpolation.png")


if __name__ == '__main__':
    simulate_profiles()
    simulate_cubic_polynomial()
    simulate_ptp()
    simulate_2d_path()
    print("\n所有运动控制仿真完成!")
