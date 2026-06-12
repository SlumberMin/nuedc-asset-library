#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
电机速度控制仿真 - PID + 前馈 + 扰动观测器
============================================
适用于电赛直流电机速度控制类题目
包含三种控制策略对比：纯PID、PID+前馈、PID+前馈+DOB(扰动观测器)
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False


class DCMotorModel:
    """直流电机数学模型 (二阶系统)
    传递函数: K / (T_m * s + 1)(T_e * s + 1)
    状态方程: dx/dt = Ax + Bu + Ed
    """
    def __init__(self, K=1.0, Tm=0.5, Te=0.01, dt=0.001):
        self.K = K      # 电机增益
        self.Tm = Tm    # 机械时间常数
        self.Te = Te    # 电气时间常数
        self.dt = dt
        # 状态: [电流, 转速]
        self.current = 0.0
        self.speed = 0.0
        self.disturbance = 0.0

    def update(self, voltage, disturbance=0.0):
        """更新电机状态"""
        self.disturbance = disturbance
        # 电流环: di/dt = (-i*R + v - Ke*w) / L  简化为一阶
        di = (-self.current + voltage) / self.Te
        # 速度环: dw/dt = (-w + K*i + d) / Tm
        dw = (-self.speed + self.K * self.current + disturbance) / self.Tm

        self.current += di * self.dt
        self.speed += dw * self.dt
        return self.speed


class PIDController:
    """增量式PID控制器"""
    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.0, dt=0.001, output_limit=24.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.output_limit = output_limit

        self.error_prev = 0.0
        self.error_prev2 = 0.0
        self.output = 0.0

    def compute(self, setpoint, measurement):
        error = setpoint - measurement
        # 增量式PID
        delta_u = (self.Kp * (error - self.error_prev)
                   + self.Ki * error * self.dt
                   + self.Kd * (error - 2 * self.error_prev + self.error_prev2) / self.dt)

        self.output += delta_u
        self.output = np.clip(self.output, -self.output_limit, self.output_limit)

        self.error_prev2 = self.error_prev
        self.error_prev = error
        return self.output


class FeedForwardController:
    """前馈控制器 - 基于模型逆"""
    def __init__(self, K=1.0, Tm=0.5, dt=0.001):
        self.K = K
        self.Tm = Tm
        self.dt = dt
        self.ref_prev = 0.0

    def compute(self, ref, ref_dot=None):
        """前馈量 = (Tm * d_ref/dt + ref) / K"""
        if ref_dot is None:
            ref_dot = (ref - self.ref_prev) / self.dt
        self.ref_prev = ref
        feedforward = (self.Tm * ref_dot + ref) / self.K
        return feedforward


class DisturbanceObserver:
    """扰动观测器 (DOB)
    观测扰动 d_hat = (u - y/K) 的低通滤波
    """
    def __init__(self, K=1.0, cutoff_freq=50.0, dt=0.001):
        self.K = K
        self.dt = dt
        # 一阶低通滤波器系数
        tau = 1.0 / (2 * np.pi * cutoff_freq)
        self.alpha = dt / (tau + dt)
        self.d_hat = 0.0
        self.u_prev = 0.0
        self.y_prev = 0.0

    def update(self, u, y):
        """估计扰动"""
        # 简化的DOB: 估计模型输出，与实际输出比较
        y_model = self.K * u  # 简化模型
        d_raw = y - y_model
        self.d_hat = (1 - self.alpha) * self.d_hat + self.alpha * d_raw
        return self.d_hat


def generate_reference(t, pattern='step', amplitude=100.0):
    """生成参考信号"""
    if pattern == 'step':
        return amplitude if t > 0.1 else 0.0
    elif pattern == 'ramp':
        return min(amplitude * t / 2.0, amplitude) if t > 0.1 else 0.0
    elif pattern == 'sine':
        return amplitude * 0.5 * (np.sin(2 * np.pi * 0.5 * t) + 1) if t > 0.1 else 0.0
    elif pattern == 'multi_step':
        if t > 3.0:
            return amplitude * 0.5
        elif t > 1.0:
            return amplitude
        else:
            return amplitude * 0.3
    return 0.0


def simulate(duration=5.0, dt=0.001, ref_pattern='step', add_disturbance=True):
    """运行仿真"""
    steps = int(duration / dt)
    t = np.arange(steps) * dt

    # 创建对象
    pid1 = PIDController(Kp=2.0, Ki=5.0, Kd=0.01, dt=dt, output_limit=24.0)
    pid2 = PIDController(Kp=2.0, Ki=5.0, Kd=0.01, dt=dt, output_limit=24.0)
    pid3 = PIDController(Kp=1.5, Ki=4.0, Kd=0.01, dt=dt, output_limit=24.0)
    ff = FeedForwardController(K=1.0, Tm=0.5, dt=dt)
    dob = DisturbanceObserver(K=1.0, cutoff_freq=50.0, dt=dt)

    # 结果存储
    ref_signal = np.zeros(steps)
    y_pid = np.zeros(steps)
    y_pid_ff = np.zeros(steps)
    y_pid_ff_dob = np.zeros(steps)
    u_pid = np.zeros(steps)
    u_pid_ff = np.zeros(steps)
    u_pid_ff_dob = np.zeros(steps)
    dist_est = np.zeros(steps)
    disturbance = np.zeros(steps)

    # 仿真主循环
    motors = [DCMotorModel(K=1.0, Tm=0.5, Te=0.01, dt=dt) for _ in range(3)]

    for i in range(steps):
        ref = generate_reference(t[i], ref_pattern, amplitude=100.0)
        ref_signal[i] = ref

        # 扰动：在t=2s时加入阶跃扰动
        d = -5.0 if (add_disturbance and 2.0 < t[i] < 3.5) else 0.0
        disturbance[i] = d

        # 方案1: 纯PID
        u1 = pid1.compute(ref, motors[0].speed)
        y_pid[i] = motors[0].update(u1, d)
        u_pid[i] = u1

        # 方案2: PID + 前馈
        u_ff = ff.compute(ref)
        u2 = pid2.compute(ref, motors[1].speed) + u_ff
        u2 = np.clip(u2, -24.0, 24.0)
        y_pid_ff[i] = motors[1].update(u2, d)
        u_pid_ff[i] = u2

        # 方案3: PID + 前馈 + DOB
        d_hat = dob.update(u_pid_ff_dob[max(0, i-1)], motors[2].speed)
        dist_est[i] = d_hat
        u3 = pid3.compute(ref, motors[2].speed) + u_ff - d_hat
        u3 = np.clip(u3, -24.0, 24.0)
        y_pid_ff_dob[i] = motors[2].update(u3, d)
        u_pid_ff_dob[i] = u3

    return {
        't': t, 'ref': ref_signal, 'disturbance': disturbance,
        'pid': y_pid, 'pid_ff': y_pid_ff, 'pid_ff_dob': y_pid_ff_dob,
        'u_pid': u_pid, 'u_pid_ff': u_pid_ff, 'u_pid_ff_dob': u_pid_ff_dob,
        'dist_est': dist_est
    }


def plot_results(data):
    """绘制仿真结果"""
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

    # 速度响应对比
    axes[0].plot(data['t'], data['ref'], 'k--', linewidth=2, label='参考信号')
    axes[0].plot(data['t'], data['pid'], 'b-', linewidth=1, label='纯PID', alpha=0.8)
    axes[0].plot(data['t'], data['pid_ff'], 'r-', linewidth=1, label='PID+前馈', alpha=0.8)
    axes[0].plot(data['t'], data['pid_ff_dob'], 'g-', linewidth=1.5, label='PID+前馈+DOB')
    axes[0].set_ylabel('转速 (rpm)')
    axes[0].set_title('电机速度控制仿真 - 三种控制策略对比')
    axes[0].legend(loc='best')
    axes[0].grid(True, alpha=0.3)

    # 扰动与估计
    axes[1].plot(data['t'], data['disturbance'], 'r-', linewidth=2, label='实际扰动')
    axes[1].plot(data['t'], data['dist_est'], 'b--', linewidth=1, label='DOB估计扰动')
    axes[1].set_ylabel('扰动力矩')
    axes[1].set_title('扰动观测器效果')
    axes[1].legend(loc='best')
    axes[1].grid(True, alpha=0.3)

    # 控制量
    axes[2].plot(data['t'], data['u_pid'], 'b-', linewidth=1, label='PID输出', alpha=0.7)
    axes[2].plot(data['t'], data['u_pid_ff'], 'r-', linewidth=1, label='PID+前馈输出', alpha=0.7)
    axes[2].plot(data['t'], data['u_pid_ff_dob'], 'g-', linewidth=1.5, label='PID+FF+DOB输出')
    axes[2].set_xlabel('时间 (s)')
    axes[2].set_ylabel('控制电压 (V)')
    axes[2].set_title('控制量对比')
    axes[2].legend(loc='best')
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('motor_speed_control_result.png', dpi=150, bbox_inches='tight')
    plt.close('all')

    # 计算性能指标
    t = data['t']
    ref = data['ref']
    for name, y in [('PID', data['pid']), ('PID+FF', data['pid_ff']), ('PID+FF+DOB', data['pid_ff_dob'])]:
        mask = t > 0.1
        error = ref[mask] - y[mask]
        mae = np.mean(np.abs(error))
        rmse = np.sqrt(np.mean(error**2))
        # 调节时间 (5%误差带)
        settling = None
        for j in range(len(error)-1, -1, -1):
            if np.abs(error[j]) > 0.05 * np.max(ref):
                settling = t[mask][j] if j < len(t[mask])-1 else t[mask][-1]
                break
        settling_str = f"{settling:.2f}" if settling is not None else "N/A(始终在误差带内)"
        print(f"{name:15s} | MAE={mae:.2f} | RMSE={rmse:.2f} | 调节时间≈{settling_str}s")


if __name__ == '__main__':
    print("=" * 60)
    print("  电机速度控制仿真 (PID + 前馈 + 扰动观测器)")
    print("=" * 60)
    data = simulate(duration=5.0, dt=0.001, ref_pattern='step')
    plot_results(data)
