#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
自适应控制仿真 - MRAC + 增益调度 + 自校正PID对比
用于电赛控制系统设计与参数自整定
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
matplotlib.rcParams['axes.unicode_minus'] = False

# ======================== 通用被控对象 ========================
class SecondOrderPlant:
    """二阶时变被控对象: G(s) = K / (s^2 + a*s + b)
    参数a, b随时间缓慢漂移，模拟工况变化"""
    def __init__(self, a0=2.0, b0=5.0, K0=1.0):
        self.a0, self.b0, self.K0 = a0, b0, K0
        self.x = np.zeros(2)  # [x, dx]
        self.a, self.b, self.K = a0, b0, K0

    def update_params(self, t):
        """参数漂移模拟"""
        self.a = self.a0 + 0.5 * np.sin(0.05 * t)
        self.b = self.b0 + 1.0 * (1 - np.exp(-t / 30))

    def step(self, u, dt):
        self.update_params(self.t if hasattr(self, 't') else 0)
        dx0 = self.x[1]
        dx1 = -self.a * self.x[1] - self.b * self.x[0] + self.K * u
        self.x[0] += dx0 * dt
        self.x[1] += dx1 * dt
        self.t = getattr(self, 't', 0) + dt
        return self.x[0]

    def reset(self):
        self.x = np.zeros(2)
        self.a, self.b, self.K = self.a0, self.b0, self.K0
        self.t = 0


# ======================== MRAC控制器 ========================
class MRACController:
    """模型参考自适应控制 (MIT规则)"""
    def __init__(self, am=3.0, bm=9.0, gamma=0.5):
        self.am, self.bm = am, bm
        self.gamma = gamma
        self.theta = np.array([0.0, 0.0])  # [kr, a_adapt]
        self.xm = np.zeros(2)

    def reference_model(self, r, dt):
        dx0 = self.xm[1]
        dx1 = -self.am * self.xm[1] - self.bm * self.xm[0] + self.bm * r
        self.xm[0] += dx0 * dt
        self.xm[1] += dx1 * dt
        return self.xm[0]

    def compute(self, r, y, dt):
        ym = self.reference_model(r, dt)
        e = y - ym
        # MIT律
        self.theta[0] -= self.gamma * e * r * dt
        self.theta[1] -= self.gamma * e * y * dt
        self.theta = np.clip(self.theta, -50, 50)
        u = self.theta[0] * r - self.theta[1] * y
        return u, ym, e

    def reset(self):
        self.theta = np.array([0.0, 0.0])
        self.xm = np.zeros(2)


# ======================== 增益调度PID ========================
class GainSchedulingPID:
    """基于误差幅值的增益调度PID"""
    def __init__(self):
        # 不同误差区间的PID参数 [Kp, Ki, Kd]
        self.schedules = [
            {'threshold': 0.3, 'Kp': 15.0, 'Ki': 2.0, 'Kd': 3.0},   # 大误差：快响应
            {'threshold': 0.1, 'Kp': 8.0,  'Ki': 1.5, 'Kd': 2.0},   # 中误差
            {'threshold': 0.0, 'Kp': 4.0,  'Ki': 1.0, 'Kd': 1.0},   # 小误差：精细调节
        ]
        self.integral = 0
        self.prev_error = 0

    def get_gains(self, error):
        abs_e = abs(error)
        for s in self.schedules:
            if abs_e >= s['threshold']:
                return s['Kp'], s['Ki'], s['Kd']
        return self.schedules[-1]['Kp'], self.schedules[-1]['Ki'], self.schedules[-1]['Kd']

    def compute(self, error, dt):
        Kp, Ki, Kd = self.get_gains(error)
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error
        return Kp * error + Ki * self.integral + Kd * derivative

    def reset(self):
        self.integral = 0
        self.prev_error = 0


# ======================== 自校正PID ========================
class SelfTuningPID:
    """递推最小二乘 + 自校正PID (极点配置)"""
    def __init__(self):
        self.theta = np.array([1.0, -0.5, 0.3, 0.1])  # [b0, b1, a1, a2]
        self.P = np.eye(4) * 100
        self.lam = 0.98  # 遗忘因子
        self.phi = np.zeros(4)
        self.u_buf = [0, 0]
        self.y_buf = [0, 0, 0]
        self.Kp, self.Ki, self.Kd = 5.0, 1.0, 1.5
        self.integral = 0
        self.prev_error = 0

    def rls_update(self, u, y):
        self.phi = np.array([-self.y_buf[1], -self.y_buf[2], self.u_buf[0], self.u_buf[1]])
        y_hat = self.phi @ self.theta
        err = y - y_hat
        denom = self.lam + self.phi @ self.P @ self.phi
        K = self.P @ self.phi / max(denom, 1e-6)
        self.theta += K * err
        self.P = (self.P - np.outer(K, self.phi @ self.P)) / self.lam
        # 更新缓冲
        self.y_buf = [y, self.y_buf[0], self.y_buf[1]]
        self.u_buf = [u, self.u_buf[0]]
        return err

    def update_pid_gains(self):
        """基于辨识参数计算PID增益（简化极点配置）"""
        b0 = max(abs(self.theta[0]), 0.01)
        self.Kp = 2.0 / b0
        self.Ki = self.Kp * 0.3
        self.Kd = self.Kp * 0.4

    def compute(self, error, dt):
        self.rls_update(0, 0)  # 预更新
        self.update_pid_gains()
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative

    def reset(self):
        self.theta = np.array([1.0, -0.5, 0.3, 0.1])
        self.P = np.eye(4) * 100
        self.integral = 0
        self.prev_error = 0


# ======================== 固定PID(基线) ========================
class FixedPID:
    def __init__(self, Kp=5, Ki=1, Kd=2):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.integral = 0
        self.prev_error = 0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative

    def reset(self):
        self.integral = 0
        self.prev_error = 0


# ======================== 仿真主循环 ========================
def simulate(controller, plant, ref_func, T=30, dt=0.01):
    N = int(T / dt)
    t_arr = np.linspace(0, T, N)
    y_arr, u_arr, r_arr = np.zeros(N), np.zeros(N), np.zeros(N)
    ym_arr = np.zeros(N)

    plant.reset()
    if hasattr(controller, 'reset'):
        controller.reset()

    for i in range(N):
        t = i * dt
        r = ref_func(t)
        y = plant.x[0]
        r_arr[i] = r

        if isinstance(controller, MRACController):
            u, ym, _ = controller.compute(r, y, dt)
            ym_arr[i] = ym
        else:
            error = r - y
            u = controller.compute(error, dt)

        u = np.clip(u, -20, 20)
        plant.step(u, dt)
        y_arr[i] = plant.x[0]
        u_arr[i] = u

    return t_arr, y_arr, u_arr, r_arr, ym_arr


# ======================== 主程序 ========================
def main():
    T, dt = 30, 0.01

    # 参考信号：方波
    def ref_func(t):
        return 1.0 if (int(t / 10) % 2 == 0) else -1.0

    # 控制器
    controllers = {
        '固定PID': FixedPID(Kp=5, Ki=1, Kd=2),
        '增益调度PID': GainSchedulingPID(),
        'MRAC': MRACController(am=3, bm=9, gamma=0.3),
        '自校正PID': SelfTuningPID(),
    }

    results = {}
    for name, ctrl in controllers.items():
        plant = SecondOrderPlant(a0=2.0, b0=5.0, K0=1.0)
        t, y, u, r, ym = simulate(ctrl, plant, ref_func, T, dt)
        results[name] = {'t': t, 'y': y, 'u': u, 'r': r, 'ym': ym}

    # ======================== 绘图 ========================
    fig, axes = plt.subplots(3, 1, figsize=(14, 10))

    # 输出响应
    ax = axes[0]
    ax.plot(results['固定PID']['t'], results['固定PID']['r'], 'k--', lw=1.5, label='参考信号')
    colors = ['#2196F3', '#4CAF50', '#FF9800', '#E91E63']
    for (name, data), color in zip(results.items(), colors):
        ax.plot(data['t'], data['y'], color=color, lw=1.2, label=name)
    ax.set_ylabel('输出 y(t)')
    ax.set_title('自适应控制方法对比 - 输出响应')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, alpha=0.3)

    # 误差
    ax = axes[1]
    for (name, data), color in zip(results.items(), colors):
        e = data['r'] - data['y']
        ax.plot(data['t'], e, color=color, lw=0.8, label=name)
    ax.set_ylabel('跟踪误差 e(t)')
    ax.set_title('跟踪误差对比')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, alpha=0.3)

    # 控制量
    ax = axes[2]
    for (name, data), color in zip(results.items(), colors):
        ax.plot(data['t'], data['u'], color=color, lw=0.8, label=name)
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('控制量 u(t)')
    ax.set_title('控制量对比')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('adaptive_control_comparison.png', dpi=150, bbox_inches='tight')
    plt.show()

    # MRAC参考模型跟踪图
    fig2, ax2 = plt.subplots(figsize=(10, 5))
    d = results['MRAC']
    ax2.plot(d['t'], d['r'], 'k--', lw=1.5, label='参考信号')
    ax2.plot(d['t'], d['y'], '#FF9800', lw=1.2, label='MRAC输出')
    ax2.plot(d['t'], d['ym'], '#9C27B0', lw=1.2, ls='-.', label='参考模型输出')
    ax2.set_xlabel('时间 (s)')
    ax2.set_ylabel('输出')
    ax2.set_title('MRAC参考模型跟踪效果')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('mrac_model_tracking.png', dpi=150, bbox_inches='tight')
    plt.show()

    # 性能指标汇总
    print("\n" + "="*70)
    print("自适应控制方法性能指标对比")
    print("="*70)
    print(f"{'方法':<12} {'ISE':>10} {'IAE':>10} {'ITAE':>10} {'超调%':>10} {'控制能量':>10}")
    print("-"*70)
    for name, data in results.items():
        e = data['r'] - data['y']
        ise = np.sum(e**2) * dt
        iae = np.sum(np.abs(e)) * dt
        itae = np.sum(data['t'] * np.abs(e)) * dt
        overshoot = (np.max(data['y']) - 1.0) / 1.0 * 100 if np.max(data['y']) > 1 else 0
        control_energy = np.sum(data['u']**2) * dt
        print(f"{name:<12} {ise:>10.3f} {iae:>10.3f} {itae:>10.3f} {overshoot:>9.1f}% {control_energy:>10.1f}")
    print("="*70)


if __name__ == '__main__':
    main()
