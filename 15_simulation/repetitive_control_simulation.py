#!/usr/bin/env python3
"""
重复控制仿真 (Repetitive Control)
====================================
重复控制基于内模原理，将周期信号发生器嵌入反馈回路，
实现对周期性参考信号或周期性干扰的无静差跟踪/抑制。

核心思想: G_rc(z) = z^(-N) / (1 - z^(-N))
  其中 N 为一个参考周期内的采样点数

对比：重复控制 vs 常规PID
场景：跟踪周期性信号，抑制周期性扰动

运行方式: python repetitive_control_simulation.py
输出图表: repetitive_control_result.png
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False


class RepetitiveController:
    """重复控制器"""
    def __init__(self, N_period, kr=0.1, Q=0.95, out_min=-10, out_max=10):
        self.N = N_period        # 周期采样点数
        self.kr = kr             # 重复增益
        self.Q = Q               # 低通滤波器系数(稳定化)
        self.out_min = out_min
        self.out_max = out_max
        self.buffer = np.zeros(N_period)  # 延迟缓冲
        self.idx = 0
        self.prev_output = 0.0

    def compute(self, error):
        # Q滤波后的延迟信号
        delayed = self.buffer[self.idx]
        # 重复控制律: u(k) = Q*u(k-N) + kr*e(k-N)
        output = self.Q * self.prev_output + self.kr * delayed
        # 更新缓冲（存入当前误差）
        self.buffer[self.idx] = error
        self.idx = (self.idx + 1) % self.N
        self.prev_output = output
        return np.clip(output, self.out_min, self.out_max)

    def reset(self):
        self.buffer[:] = 0
        self.idx = 0
        self.prev_output = 0.0


class PIDController:
    def __init__(self, kp, ki, kd, out_min=-10, out_max=10):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt=1.0):
        self.integral += error * dt
        self.integral = np.clip(self.integral,
                                self.out_min / max(self.ki, 1e-8),
                                self.out_max / max(self.ki, 1e-8))
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        out = self.kp * error + self.ki * self.integral + self.kd * derivative
        return np.clip(out, self.out_min, self.out_max)

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


def run_simulation():
    # 采样频率 1kHz，参考信号频率 50Hz
    fs = 1000
    f_ref = 50
    N_period = fs // f_ref  # 每周期20个采样点
    T_total = 0.5           # 0.5秒 = 25个周期
    N_samples = int(T_total * fs)
    t = np.arange(N_samples) / fs

    # 参考信号: 50Hz正弦
    ref = np.sin(2 * np.pi * f_ref * t)

    # 周期性扰动: 150Hz（3次谐波）
    dist = 0.3 * np.sin(2 * np.pi * 3 * f_ref * t + 0.5)

    # 被控对象: 简单一阶离散系统 y(k) = 0.9*y(k-1) + 0.1*u(k-1)
    a_plant, b_plant = 0.9, 0.1

    # ===== 方案1: 纯PID =====
    pid1 = PIDController(kp=5.0, ki=1.0, kd=0.5)
    y_pid = np.zeros(N_samples)
    u_pid = np.zeros(N_samples)

    for k in range(1, N_samples):
        error = ref[k-1] - y_pid[k-1]
        u_pid[k] = pid1.compute(error)
        u_applied = u_pid[k] + dist[k]
        y_pid[k] = a_plant * y_pid[k-1] + b_plant * u_applied

    # ===== 方案2: PID + 重复控制 =====
    pid2 = PIDController(kp=5.0, ki=1.0, kd=0.5)
    rc = RepetitiveController(N_period=N_period, kr=0.3, Q=0.98)
    y_rc = np.zeros(N_samples)
    u_rc = np.zeros(N_samples)

    for k in range(1, N_samples):
        error = ref[k-1] - y_rc[k-1]
        u_pid_part = pid2.compute(error)
        u_rc_part = rc.compute(error)
        u_rc[k] = u_pid_part + u_rc_part
        u_applied = u_rc[k] + dist[k]
        y_rc[k] = a_plant * y_rc[k-1] + b_plant * u_applied

    # ===== 方案3: 纯重复控制 =====
    rc_only = RepetitiveController(N_period=N_period, kr=0.5, Q=0.98)
    y_rc_only = np.zeros(N_samples)
    u_rc_only = np.zeros(N_samples)

    for k in range(1, N_samples):
        error = ref[k-1] - y_rc_only[k-1]
        u_rc_only[k] = rc_only.compute(error)
        u_applied = u_rc_only[k] + dist[k]
        y_rc_only[k] = a_plant * y_rc_only[k-1] + b_plant * u_applied

    # 计算各周期RMSE
    def periodic_rmse(err, n_per):
        rmses = []
        for p in range(len(err) // n_per):
            seg = err[p*n_per:(p+1)*n_per]
            rmses.append(np.sqrt(np.mean(seg**2)))
        return rmses

    rmses_pid = periodic_rmse(ref - y_pid, N_period)
    rmses_rc = periodic_rmse(ref - y_rc, N_period)
    rmses_rco = periodic_rmse(ref - y_rc_only, N_period)
    periods = np.arange(len(rmses_pid))

    # ===== 绘图 =====
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=False)
    fig.suptitle('重复控制仿真 - 周期信号跟踪与扰动抑制',
                 fontsize=14, fontweight='bold')

    # 波形对比
    ax0 = axes[0]
    ax0.plot(t*1000, ref, 'k--', linewidth=1.5, label='参考(50Hz)')
    ax0.plot(t*1000, y_pid, 'b-', linewidth=0.8, alpha=0.7, label='纯PID')
    ax0.plot(t*1000, y_rc, 'r-', linewidth=1.2, label='PID+重复控制')
    ax0.set_ylabel('输出')
    ax0.set_xlabel('时间 (ms)')
    ax0.legend(loc='upper right')
    ax0.grid(True, alpha=0.3)
    ax0.set_title('跟踪波形对比')

    # 误差波形
    ax1 = axes[1]
    ax1.plot(t*1000, ref - y_pid, 'b-', linewidth=0.6, alpha=0.7, label='纯PID误差')
    ax1.plot(t*1000, ref - y_rc, 'r-', linewidth=0.8, label='PID+RC误差')
    ax1.set_ylabel('跟踪误差')
    ax1.set_xlabel('时间 (ms)')
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)
    ax1.set_title('跟踪误差波形')

    # 各周期RMSE收敛曲线
    ax2 = axes[2]
    ax2.plot(periods, rmses_pid, 'b-o', markersize=4, linewidth=1.5, label='纯PID')
    ax2.plot(periods, rmses_rc, 'r-s', markersize=4, linewidth=1.5, label='PID+RC')
    ax2.plot(periods, rmses_rco, 'g-^', markersize=4, linewidth=1.5, label='纯RC')
    ax2.set_xlabel('周期数')
    ax2.set_ylabel('RMSE')
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3)
    ax2.set_title('逐周期RMSE收敛曲线')

    plt.tight_layout()
    plt.savefig('repetitive_control_result.png', dpi=150, bbox_inches='tight')
    print('[OK] 仿真完成，图表已保存: repetitive_control_result.png')
    plt.close()


if __name__ == '__main__':
    run_simulation()
