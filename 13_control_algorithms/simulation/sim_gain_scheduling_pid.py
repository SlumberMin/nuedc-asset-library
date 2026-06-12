#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
增益调度PID (Gain Scheduling PID) 仿真演示

演示内容：
1. 增益调度 vs 固定PID 对比
2. 硬切换 vs 软切换
3. 电机全速域控制示例
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial Unicode MS']
plt.rcParams['axes.unicode_minus'] = False


class VariablePlant:
    """变参数被控对象（模拟电机在不同转速下的特性变化）"""
    def __init__(self, dt):
        self.dt = dt
        self.y = 0.0
        self.dy = 0.0

    def update(self, u, speed):
        """speed 越大，系统惯量越大，响应越慢"""
        # 惯量随转速变化
        J = 1.0 + 0.02 * abs(speed)
        # 阻尼随转速变化
        D = 0.3 + 0.005 * abs(speed)
        ddy = (-D * self.dy + u) / J
        self.dy += ddy * self.dt
        self.y += self.dy * self.dt
        return self.y

    def reset(self):
        self.y = 0.0
        self.dy = 0.0


class GainSchedulingPID:
    """增益调度PID控制器"""
    def __init__(self, dt, u_max=100, u_min=-100, mode='soft'):
        self.dt = dt
        self.u_max = u_max
        self.u_min = u_min
        self.mode = mode
        self.points = []  # [(sv, Kp, Ki, Kd), ...]
        self.integral = 0.0
        self.last_error = 0.0
        self.last_feedback = 0.0
        self.Kp = self.Ki = self.Kd = 0.0
        self.integral_max = 500.0

    def add_point(self, sv, Kp, Ki, Kd):
        self.points.append((sv, Kp, Ki, Kd))
        self.points.sort(key=lambda p: p[0])

    def lookup(self, sv):
        n = len(self.points)
        if n == 0:
            return 0, 0, 0
        if sv <= self.points[0][0]:
            return self.points[0][1], self.points[0][2], self.points[0][3]
        if sv >= self.points[-1][0]:
            return self.points[-1][1], self.points[-1][2], self.points[-1][3]

        for i in range(n - 1):
            sv0, kp0, ki0, kd0 = self.points[i]
            sv1, kp1, ki1, kd1 = self.points[i + 1]
            if sv0 <= sv < sv1:
                if self.mode == 'hard':
                    return kp0, ki0, kd0
                else:
                    t = (sv - sv0) / (sv1 - sv0)
                    return (kp0 + t * (kp1 - kp0),
                            ki0 + t * (ki1 - ki0),
                            kd0 + t * (kd1 - kd0))
        return self.points[-1][1], self.points[-1][2], self.points[-1][3]

    def update(self, setpoint, feedback, sv):
        self.Kp, self.Ki, self.Kd = self.lookup(sv)
        error = setpoint - feedback
        self.integral += error * self.dt
        self.integral = np.clip(self.integral, -self.integral_max, self.integral_max)
        derivative = (error - self.last_error) / self.dt
        u = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        u = np.clip(u, self.u_min, self.u_max)
        self.last_error = error
        self.last_feedback = feedback
        return u

    def reset(self):
        self.integral = 0.0
        self.last_error = 0.0
        self.last_feedback = 0.0


class FixedPID:
    """固定参数PID"""
    def __init__(self, Kp, Ki, Kd, dt, u_max=100, u_min=-100):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.dt = dt
        self.u_max, self.u_min = u_max, u_min
        self.integral = 0.0
        self.last_error = 0.0
        self.integral_max = 500.0

    def update(self, setpoint, feedback):
        error = setpoint - feedback
        self.integral += error * self.dt
        self.integral = np.clip(self.integral, -self.integral_max, self.integral_max)
        derivative = (error - self.last_error) / self.dt
        u = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.last_error = error
        return np.clip(u, self.u_min, self.u_max)

    def reset(self):
        self.integral = 0.0
        self.last_error = 0.0


def demo1_gs_vs_fixed():
    """演示1：增益调度 vs 固定PID"""
    dt = 0.001
    T = 15.0
    n = int(T / dt)
    t = np.arange(n) * dt

    # 调度变量：转速从0逐步增加
    speed = np.piecewise(t,
        [t < 3, (t >= 3) & (t < 6), (t >= 6) & (t < 9), t >= 9],
        [500, 1000, 2000, 3000])

    sp = 1.0  # 设定值

    # 增益调度PID
    gs_pid = GainSchedulingPID(dt)
    gs_pid.add_point(500, 8.0, 2.0, 0.5)
    gs_pid.add_point(1000, 10.0, 3.0, 0.8)
    gs_pid.add_point(2000, 15.0, 4.0, 1.2)
    gs_pid.add_point(3000, 20.0, 5.0, 1.5)

    # 固定PID（中等转速整定的参数）
    fixed_pid = FixedPID(12.0, 3.5, 1.0, dt)

    plant_gs = VariablePlant(dt)
    plant_fixed = VariablePlant(dt)

    y_gs = np.zeros(n)
    y_fixed = np.zeros(n)

    for i in range(n):
        y_gs[i] = plant_gs.y
        u_gs = gs_pid.update(sp, plant_gs.y, speed[i])
        plant_gs.update(u_gs, speed[i])

        y_fixed[i] = plant_fixed.y
        u_f = fixed_pid.update(sp, plant_fixed.y)
        plant_fixed.update(u_f, speed[i])

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7))

    ax1.plot(t, np.ones(n) * sp, 'k--', label='设定值', linewidth=1)
    ax1.plot(t, y_gs, 'b-', label='增益调度PID', linewidth=1.5)
    ax1.plot(t, y_fixed, 'r-', label='固定PID', linewidth=1.5)
    ax1.set_ylabel('输出')
    ax1.set_title('增益调度PID vs 固定PID（转速变化场景）')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    ax2.plot(t, speed, 'g-', linewidth=1.5)
    ax2.set_xlabel('时间 (s)')
    ax2.set_ylabel('转速 (rpm)')
    ax2.set_title('调度变量（转速）变化')
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('gs_pid_vs_fixed.png', dpi=150, bbox_inches='tight')
    plt.close('all')


def demo2_hard_vs_soft():
    """演示2：硬切换 vs 软切换"""
    dt = 0.001
    T = 10.0
    n = int(T / dt)
    t = np.arange(n) * dt

    # 调度变量缓慢变化
    sv = 500 + 2500 * (1 - np.cos(2 * np.pi * t / T)) / 2

    sp = 1.0

    gs_hard = GainSchedulingPID(dt, mode='hard')
    gs_soft = GainSchedulingPID(dt, mode='soft')
    for gs in [gs_hard, gs_soft]:
        gs.add_point(500, 8.0, 2.0, 0.5)
        gs.add_point(1000, 10.0, 3.0, 0.8)
        gs.add_point(2000, 15.0, 4.0, 1.2)
        gs.add_point(3000, 20.0, 5.0, 1.5)

    plant_hard = VariablePlant(dt)
    plant_soft = VariablePlant(dt)

    y_hard = np.zeros(n)
    y_soft = np.zeros(n)

    for i in range(n):
        y_hard[i] = plant_hard.y
        u_h = gs_hard.update(sp, plant_hard.y, sv[i])
        plant_hard.update(u_h, sv[i])

        y_soft[i] = plant_soft.y
        u_s = gs_soft.update(sp, plant_soft.y, sv[i])
        plant_soft.update(u_s, sv[i])

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7))

    ax1.plot(t, np.ones(n) * sp, 'k--', label='设定值', linewidth=1)
    ax1.plot(t, y_hard, 'r-', label='硬切换', linewidth=1, alpha=0.8)
    ax1.plot(t, y_soft, 'b-', label='软切换（插值）', linewidth=1.5)
    ax1.set_ylabel('输出')
    ax1.set_title('硬切换 vs 软切换（增益调度PID）')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    ax2.plot(t, sv, 'g-', linewidth=1.5)
    ax2.set_xlabel('时间 (s)')
    ax2.set_ylabel('调度变量')
    ax2.set_title('调度变量变化')
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('gs_hard_vs_soft.png', dpi=150, bbox_inches='tight')
    plt.close('all')


if __name__ == '__main__':
    print("=== 增益调度PID仿真演示 ===")
    print("演示1: 增益调度 vs 固定PID...")
    demo1_gs_vs_fixed()
    print("演示2: 硬切换 vs 软切换...")
    demo2_hard_vs_soft()
    print("仿真完成！")
