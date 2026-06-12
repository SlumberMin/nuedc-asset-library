#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PID融合控制仿真 - 对比纯PID vs 前馈+PID vs 串级PID
====================================================
适用场景: 电机速度/位置控制、温度控制等
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
plt.rcParams['axes.unicode_minus'] = False


class PIDController:
    """标准PID控制器"""
    def __init__(self, kp, ki, kd, dt, output_limits=(-100, 100)):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dt = dt
        self.limits = output_limits
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, setpoint, measurement):
        error = setpoint - measurement
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        output = np.clip(output, self.limits[0], self.limits[1])
        return output

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


class Plant:
    """二阶系统 G(s) = K / (T1*s^2 + T2*s + 1)"""
    def __init__(self, K=1.0, T1=0.1, T2=0.5, dt=0.01):
        self.K, self.T1, self.T2, self.dt = K, T1, T2, dt
        self.x1 = 0.0  # 位置
        self.x2 = 0.0  # 速度

    def update(self, u):
        dx2 = (-self.T2 * self.x2 - self.x1 + self.K * u) / self.T1
        self.x2 += dx2 * self.dt
        self.x1 += self.x2 * self.dt
        return self.x1

    def reset(self):
        self.x1 = self.x2 = 0.0


def simulate_pure_pid(setpoint_func, t):
    """纯PID控制"""
    plant = Plant()
    pid = PIDController(kp=20.0, ki=5.0, kd=2.0, dt=0.01)
    y_list, u_list = [], []
    for ti in t:
        y = plant.x1
        u = pid.compute(setpoint_func(ti), y)
        plant.update(u)
        y_list.append(y)
        u_list.append(u)
    return y_list, u_list


def simulate_feedforward_pid(setpoint_func, d_sp_func, t):
    """前馈+PID控制: u = Kff * d(SP)/dt + PID(e)"""
    plant = Plant()
    pid = PIDController(kp=10.0, ki=3.0, kd=1.0, dt=0.01)
    Kff = 0.3
    y_list, u_list = [], []
    for ti in t:
        y = plant.x1
        ff = Kff * d_sp_func(ti)
        fb = pid.compute(setpoint_func(ti), y)
        u = np.clip(ff + fb, -100, 100)
        plant.update(u)
        y_list.append(y)
        u_list.append(u)
    return y_list, u_list


def simulate_cascade_pid(setpoint_func, t):
    """串级PID: 外环(位置)输出作为内环(速度)设定值"""
    plant = Plant()
    outer_pid = PIDController(kp=15.0, ki=2.0, kd=3.0, dt=0.01)
    inner_pid = PIDController(kp=30.0, ki=10.0, kd=0.5, dt=0.01)
    y_list, u_list = [], []
    for ti in t:
        y = plant.x1
        vel_ref = outer_pid.compute(setpoint_func(ti), y)
        vel_meas = plant.x2
        u = inner_pid.compute(vel_ref, vel_meas)
        plant.update(u)
        y_list.append(y)
        u_list.append(u)
    return y_list, u_list


def main():
    dt = 0.01
    t = np.arange(0, 5, dt)

    # 设定值: 阶跃 + 斜坡混合
    setpoint = np.where(t < 2.0, 1.0, 2.0 + 0.5 * (t - 2.0))
    sp_func = lambda ti: 1.0 if ti < 2.0 else 2.0 + 0.5 * (ti - 2.0)
    dsp_func = lambda ti: 0.0 if ti < 2.0 else 0.5

    y1, u1 = simulate_pure_pid(sp_func, t)
    y2, u2 = simulate_feedforward_pid(sp_func, dsp_func, t)
    y3, u3 = simulate_cascade_pid(sp_func, t)

    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    axes[0].plot(t, setpoint, 'k--', lw=2, label='设定值')
    axes[0].plot(t, y1, label='纯PID')
    axes[0].plot(t, y2, label='前馈+PID')
    axes[0].plot(t, y3, label='串级PID')
    axes[0].set_ylabel('输出')
    axes[0].set_title('PID融合控制仿真 - 输出对比')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t, u1, label='纯PID')
    axes[1].plot(t, u2, label='前馈+PID')
    axes[1].plot(t, u3, label='串级PID')
    axes[1].set_xlabel('时间 (s)')
    axes[1].set_ylabel('控制量')
    axes[1].set_title('控制量对比')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('pid_fusion_result.png', dpi=150)
    plt.close('all')

    # 计算性能指标
    print("\n===== 性能指标 =====")
    sp = np.array([sp_func(ti) for ti in t])
    for name, y in [('纯PID', y1), ('前馈+PID', y2), ('串级PID', y3)]:
        e = np.array(sp) - np.array(y)
        ise = np.sum(e**2) * dt
        iae = np.sum(np.abs(e)) * dt
        print(f"{name:10s} | ISE={ise:.4f}  IAE={iae:.4f}")


if __name__ == '__main__':
    main()
