#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LADRC（线性自抗扰控制器）仿真演示

演示内容：
1. LADRC 控制二阶系统
2. 与传统 PID 对比
3. 抗扰动能力测试
4. 参数灵敏度分析
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial Unicode MS']
plt.rcParams['axes.unicode_minus'] = False


class SecondOrderPlant:
    """二阶被控对象 y'' + a1*y' + a0*y = b*u + d(t)"""
    def __init__(self, a0=1.0, a1=0.5, b=1.0, dt=0.001):
        self.a0, self.a1, self.b = a0, a1, b
        self.dt = dt
        self.y = 0.0
        self.dy = 0.0

    def update(self, u, disturbance=0.0):
        ddy = -self.a1 * self.dy - self.a0 * self.y + self.b * u + disturbance
        self.dy += ddy * self.dt
        self.y += self.dy * self.dt
        return self.y

    def reset(self):
        self.y = 0.0
        self.dy = 0.0


class LADRC:
    """线性自抗扰控制器"""
    def __init__(self, wc, wo, b0, dt, u_max=100, u_min=-100):
        self.wc = wc
        self.wo = wo
        self.b0 = b0
        self.dt = dt
        self.u_max = u_max
        self.u_min = u_min

        # LESO 增益
        self.beta1 = 3 * wo
        self.beta2 = 3 * wo**2
        self.beta3 = wo**3

        # LSEF 增益
        self.Kp = wc**2
        self.Kd = 2 * wc

        # TD
        self.r0 = 100.0
        self.h0 = dt
        self.v1 = 0.0
        self.v2 = 0.0

        # LESO 状态
        self.z1 = 0.0
        self.z2 = 0.0
        self.z3 = 0.0

        self.u = 0.0

    def fhan(self, x1, x2):
        """最速综合函数"""
        d = self.r0 * self.h0
        d0 = d * self.h0
        y = x1 + self.h0 * x2
        a0 = np.sqrt(d**2 + 8 * self.r0 * abs(y))
        a = x2 + (a0 - d) / 2 * np.sign(y) if abs(y) > d0 else x2 + y / self.h0
        fh = -self.r0 * np.sign(a) if abs(a) > d else -self.r0 * a / d
        return fh

    def update(self, setpoint, feedback):
        # 1. TD
        fh = self.fhan(self.v1 - setpoint, self.v2)
        self.v1 += self.h0 * self.v2
        self.v2 += self.h0 * fh

        # 2. LESO
        e = self.z1 - feedback
        self.z1 += (self.z2 - self.beta1 * e) * self.dt
        self.z2 += (self.z3 - self.beta2 * e + self.b0 * self.u) * self.dt
        self.z3 += (-self.beta3 * e) * self.dt

        # 3. LSEF + 扰动补偿
        u0 = self.Kp * (self.v1 - self.z1) + self.Kd * (self.v2 - self.z2)
        u = (u0 - self.z3) / self.b0
        self.u = np.clip(u, self.u_min, self.u_max)
        return self.u

    def reset(self):
        self.v1 = self.v2 = 0.0
        self.z1 = self.z2 = self.z3 = 0.0
        self.u = 0.0


class PID:
    """传统 PID 控制器"""
    def __init__(self, Kp, Ki, Kd, dt, u_max=100, u_min=-100):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.dt = dt
        self.u_max, self.u_min = u_max, u_min
        self.integral = 0.0
        self.last_error = 0.0

    def update(self, setpoint, feedback):
        error = setpoint - feedback
        self.integral += error * self.dt
        self.integral = np.clip(self.integral, -1000, 1000)
        derivative = (error - self.last_error) / self.dt
        u = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.last_error = error
        return np.clip(u, self.u_min, self.u_max)

    def reset(self):
        self.integral = 0.0
        self.last_error = 0.0


def simulate(controller, plant, setpoint_func, dist_func, T, dt):
    """通用仿真函数"""
    n = int(T / dt)
    t = np.arange(n) * dt
    y_arr, u_arr, sp_arr = np.zeros(n), np.zeros(n), np.zeros(n)

    for i in range(n):
        sp = setpoint_func(t[i])
        y = plant.y
        u = controller.update(sp, y)
        d = dist_func(t[i])
        plant.update(u, d)
        sp_arr[i] = sp
        y_arr[i] = plant.y
        u_arr[i] = u

    return t, sp_arr, y_arr, u_arr


def demo1_ladrc_vs_pid():
    """演示1：LADRC vs PID 阶跃响应对比"""
    dt = 0.001
    T = 5.0

    # 被控对象: y'' + 0.5*y' + 1.0*y = 1.0*u
    plant1 = SecondOrderPlant(1.0, 0.5, 1.0, dt)
    plant2 = SecondOrderPlant(1.0, 0.5, 1.0, dt)

    ladrc = LADRC(wc=5.0, wo=15.0, b0=1.0, dt=dt)
    pid = PID(Kp=5.0, Ki=2.0, Kd=1.0, dt=dt)

    sp_func = lambda t: 1.0 if t > 0.5 else 0.0
    dist_func = lambda t: 0.0

    t, sp, y_ladrc, u_ladrc = simulate(ladrc, plant1, sp_func, dist_func, T, dt)
    pid.reset(); plant2.reset()
    _, _, y_pid, u_pid = simulate(pid, plant2, sp_func, dist_func, T, dt)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7))
    ax1.plot(t, sp, 'k--', label='设定值', linewidth=1)
    ax1.plot(t, y_ladrc, 'b-', label='LADRC', linewidth=1.5)
    ax1.plot(t, y_pid, 'r-', label='PID', linewidth=1.5)
    ax1.set_ylabel('输出')
    ax1.set_title('LADRC vs PID 阶跃响应对比')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    ax2.plot(t, u_ladrc, 'b-', label='LADRC 控制量', linewidth=1)
    ax2.plot(t, u_pid, 'r-', label='PID 控制量', linewidth=1)
    ax2.set_xlabel('时间 (s)')
    ax2.set_ylabel('控制输出')
    ax2.set_title('控制量对比')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('ladrc_vs_pid.png', dpi=150, bbox_inches='tight')
    plt.close('all')


def demo2_disturbance_rejection():
    """演示2：抗扰动能力测试"""
    dt = 0.001
    T = 8.0

    plant = SecondOrderPlant(1.0, 0.5, 1.0, dt)
    ladrc = LADRC(wc=5.0, wo=15.0, b0=1.0, dt=dt)

    sp_func = lambda t: 1.0 if t > 0.5 else 0.0
    dist_func = lambda t: 2.0 if 3.0 < t < 4.0 else 0.0

    t, sp, y, u = simulate(ladrc, plant, sp_func, dist_func, T, dt)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7))
    ax1.plot(t, sp, 'k--', label='设定值', linewidth=1)
    ax1.plot(t, y, 'b-', label='LADRC 输出', linewidth=1.5)
    ax1.axvspan(3.0, 4.0, alpha=0.2, color='red', label='扰动区间')
    ax1.set_ylabel('输出')
    ax1.set_title('LADRC 抗扰动能力测试 (阶跃扰动 d=2.0)')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    ax2.plot(t, u, 'g-', label='控制量', linewidth=1)
    d_arr = np.array([dist_func(ti) for ti in t])
    ax2.plot(t, d_arr, 'r--', label='扰动', linewidth=1)
    ax2.set_xlabel('时间 (s)')
    ax2.set_ylabel('控制输出')
    ax2.set_title('控制量与扰动')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('ladrc_disturbance.png', dpi=150, bbox_inches='tight')
    plt.close('all')


def demo3_parameter_sensitivity():
    """演示3：参数灵敏度分析"""
    dt = 0.001
    T = 4.0
    sp_func = lambda t: 1.0 if t > 0.5 else 0.0
    dist_func = lambda t: 0.0

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))

    # 不同 wc
    for wc in [2, 5, 10, 20]:
        plant = SecondOrderPlant(1.0, 0.5, 1.0, dt)
        ctrl = LADRC(wc=wc, wo=3*wc, b0=1.0, dt=dt)
        t, sp, y, _ = simulate(ctrl, plant, sp_func, dist_func, T, dt)
        axes[0].plot(t, y, label=f'ωc={wc}')
    axes[0].plot(t, sp, 'k--', linewidth=1)
    axes[0].set_title('不同 ωc 的响应')
    axes[0].set_xlabel('时间 (s)')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    # 不同 wo
    for wo in [5, 15, 30, 60]:
        plant = SecondOrderPlant(1.0, 0.5, 1.0, dt)
        ctrl = LADRC(wc=5.0, wo=wo, b0=1.0, dt=dt)
        t, sp, y, _ = simulate(ctrl, plant, sp_func, dist_func, T, dt)
        axes[1].plot(t, y, label=f'ωo={wo}')
    axes[1].plot(t, sp, 'k--', linewidth=1)
    axes[1].set_title('不同 ωo 的响应')
    axes[1].set_xlabel('时间 (s)')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    # 不同 b0
    for b0 in [0.5, 1.0, 1.5, 2.0]:
        plant = SecondOrderPlant(1.0, 0.5, 1.0, dt)
        ctrl = LADRC(wc=5.0, wo=15.0, b0=b0, dt=dt)
        t, sp, y, _ = simulate(ctrl, plant, sp_func, dist_func, T, dt)
        axes[2].plot(t, y, label=f'b0={b0}')
    axes[2].plot(t, sp, 'k--', linewidth=1)
    axes[2].set_title('不同 b0 的响应')
    axes[2].set_xlabel('时间 (s)')
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)

    plt.suptitle('LADRC 参数灵敏度分析', fontsize=14)
    plt.tight_layout()
    plt.savefig('ladrc_sensitivity.png', dpi=150, bbox_inches='tight')
    plt.close('all')


if __name__ == '__main__':
    print("=== LADRC 仿真演示 ===")
    print("演示1: LADRC vs PID 对比...")
    demo1_ladrc_vs_pid()
    print("演示2: 抗扰动能力...")
    demo2_disturbance_rejection()
    print("演示3: 参数灵敏度...")
    demo3_parameter_sensitivity()
    print("仿真完成！图片已保存。")
