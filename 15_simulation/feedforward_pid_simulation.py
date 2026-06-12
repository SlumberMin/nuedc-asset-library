#!/usr/bin/env python3
"""
前馈PID仿真 - 前馈+反馈对比
================================
比较三种控制方案：
  1. 纯PID反馈
  2. 纯前馈
  3. 前馈+PID反馈组合

典型应用场景：电机位置跟踪、温度控制、运动平台

运行方式: python feedforward_pid_simulation.py
输出图表: feedforward_pid_result.png
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False


class SecondOrderPlant:
    """二阶被控对象: G(s) = 1 / (Js^2 + bs + k)"""
    def __init__(self, J=1.0, b=0.5, k=2.0):
        self.J = J
        self.b = b
        self.k = k
        self.x = 0.0      # 位置
        self.dx = 0.0      # 速度

    def update(self, u, dt):
        ddx = (u - self.b * self.dx - self.k * self.x) / self.J
        self.dx += ddx * dt
        self.x += self.dx * dt
        return self.x

    def reset(self):
        self.x = 0.0
        self.dx = 0.0


class PIDController:
    def __init__(self, kp, ki, kd, out_min=-50, out_max=50):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        self.integral = np.clip(self.integral,
                                self.out_min / max(self.ki, 1e-8),
                                self.out_max / max(self.ki, 1e-8))
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error
        out = self.kp * error + self.ki * self.integral + self.kd * derivative
        return np.clip(out, self.out_min, self.out_max)

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


def compute_feedforward(plant, r, dr, ddr):
    """计算前馈量: u_ff = J*ddr + b*dr + k*r"""
    return plant.J * ddr + plant.b * dr + plant.k * r


def generate_trajectory(t_total, dt, amplitude=5.0, freq=0.5):
    """生成正弦参考轨迹"""
    t = np.arange(0, t_total, dt)
    omega = 2 * np.pi * freq
    r = amplitude * np.sin(omega * t)
    dr = amplitude * omega * np.cos(omega * t)
    ddr = -amplitude * omega**2 * np.sin(omega * t)
    return t, r, dr, ddr


def run_single(pid, plant, t, r, dr, ddr, use_ff, dt):
    """运行单个控制方案"""
    plant.reset()
    pid.reset()
    n = len(t)
    y = np.zeros(n)
    u_log = np.zeros(n)

    for i in range(n):
        u_pid = pid.compute(r[i] - plant.x, dt)
        u_ff = compute_feedforward(plant, r[i], dr[i], ddr[i]) if use_ff else 0.0
        u = u_pid + u_ff
        u_log[i] = u
        y[i] = plant.update(u, dt)

    return y, u_log


def run_simulation():
    dt = 0.001
    t_total = 6.0
    t, r, dr, ddr = generate_trajectory(t_total, dt)
    n = len(t)

    # 三种方案
    plant1 = SecondOrderPlant(J=1.0, b=0.5, k=2.0)
    plant2 = SecondOrderPlant(J=1.0, b=0.5, k=2.0)
    plant3 = SecondOrderPlant(J=1.0, b=0.5, k=2.0)

    pid1 = PIDController(kp=20, ki=5, kd=8)
    pid2 = PIDController(kp=20, ki=5, kd=8)
    pid3 = PIDController(kp=20, ki=5, kd=8)

    y_pid, u_pid = run_single(pid1, plant1, t, r, dr, ddr, use_ff=False, dt=dt)
    y_ff, u_ff_only = run_single(pid2, plant2, t, r, dr, ddr, use_ff=True, dt=dt)
    y_combined, u_combined = run_single(pid3, plant3, t, r, dr, ddr, use_ff=True, dt=dt)

    # 纯前馈（不加PID）
    plant_ff = SecondOrderPlant(J=1.0, b=0.5, k=2.0)
    y_ffonly = np.zeros(n)
    for i in range(n):
        u = compute_feedforward(plant_ff, r[i], dr[i], ddr[i])
        y_ffonly[i] = plant_ff.update(u, dt)

    # ===== 绘图 =====
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    fig.suptitle('前馈+PID vs 纯PID vs 纯前馈 控制效果对比', fontsize=14, fontweight='bold')

    axes[0].plot(t, r, 'k--', linewidth=2, label='参考轨迹')
    axes[0].plot(t, y_pid, 'b-', linewidth=1.2, label='纯PID')
    axes[0].plot(t, y_ffonly, 'g-', linewidth=1.2, label='纯前馈')
    axes[0].plot(t, y_combined, 'r-', linewidth=1.5, label='前馈+PID')
    axes[0].set_ylabel('位置')
    axes[0].legend(loc='upper right')
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title('跟踪响应对比')

    # 跟踪误差
    err_pid = r - y_pid
    err_ffonly = r - y_ffonly
    err_combined = r - y_combined
    axes[1].plot(t, err_pid, 'b-', linewidth=1.0, label=f'纯PID (RMSE={np.sqrt(np.mean(err_pid**2)):.4f})')
    axes[1].plot(t, err_ffonly, 'g-', linewidth=1.0, label=f'纯前馈 (RMSE={np.sqrt(np.mean(err_ffonly**2)):.4f})')
    axes[1].plot(t, err_combined, 'r-', linewidth=1.2, label=f'前馈+PID (RMSE={np.sqrt(np.mean(err_combined**2)):.4f})')
    axes[1].set_ylabel('跟踪误差')
    axes[1].legend(loc='upper right')
    axes[1].grid(True, alpha=0.3)
    axes[1].set_title('跟踪误差对比')

    # 控制量
    axes[2].plot(t, u_pid, 'b-', linewidth=0.8, label='纯PID', alpha=0.7)
    axes[2].plot(t, u_combined, 'r-', linewidth=1.0, label='前馈+PID', alpha=0.8)
    axes[2].set_ylabel('控制量 u')
    axes[2].set_xlabel('时间 (s)')
    axes[2].legend(loc='upper right')
    axes[2].grid(True, alpha=0.3)
    axes[2].set_title('控制输出对比')

    plt.tight_layout()
    plt.savefig('feedforward_pid_result.png', dpi=150, bbox_inches='tight')
    print('[OK] 仿真完成，图表已保存: feedforward_pid_result.png')
    plt.close()


if __name__ == '__main__':
    run_simulation()
