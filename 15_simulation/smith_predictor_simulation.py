#!/usr/bin/env python3
"""
Smith预估器仿真 - 纯滞后系统
================================
对比：
  1. 常规PID控制含纯滞后系统
  2. Smith预估器 + PID控制

被控对象: G(s) = K * e^(-L*s) / (Ts + 1)
  其中 L 为纯滞后时间

Smith预估器原理：
  用模型 Gm(s)*(1-e^(-Lm*s)) 补偿滞后，
  使等效开环传递函数不含滞后项。

运行方式: python smith_predictor_simulation.py
输出图表: smith_predictor_result.png
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False


class FOPDT:
    """一阶加纯滞后模型: y(s) = K*e^(-L*s)/(Ts+1) * u(s)"""
    def __init__(self, K=1.0, T=2.0, L=1.5):
        self.K = K      # 增益
        self.T = T      # 时间常数
        self.L = L      # 纯滞后
        self.y = 0.0
        # 滞迟缓冲区
        self.delay_steps = 0
        self.buffer = []
        self.initialized = False

    def init_buffer(self, dt):
        self.delay_steps = int(self.L / dt)
        self.buffer = [0.0] * (self.delay_steps + 1)
        self.initialized = True

    def update(self, u, dt):
        if not self.initialized:
            self.init_buffer(dt)
        # 一阶惯性
        dy = (-self.y + self.K * self.buffer[0]) / self.T * dt
        self.y += dy
        # 更新延迟缓冲
        self.buffer.append(u)
        self.buffer.pop(0)
        return self.y

    def reset(self):
        self.y = 0.0
        self.buffer = [0.0] * (self.delay_steps + 1) if self.initialized else []


class PIDController:
    def __init__(self, kp, ki, kd, out_min=-10, out_max=10):
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


def run_simulation():
    dt = 0.01
    t_total = 30.0
    n = int(t_total / dt)
    t = np.arange(n) * dt

    # 设定点（阶跃）
    setpoint = np.ones(n)

    # ===== 方案1: 纯PID =====
    plant1 = FOPDT(K=1.0, T=2.0, L=1.5)
    pid1 = PIDController(kp=2.0, ki=0.8, kd=0.5)
    y_pid = np.zeros(n)
    u_pid = np.zeros(n)

    for i in range(n):
        u_pid[i] = pid1.compute(setpoint[i] - plant1.y, dt)
        y_pid[i] = plant1.update(u_pid[i], dt)

    # ===== 方案2: Smith预估器 + PID =====
    plant2 = FOPDT(K=1.0, T=2.0, L=1.5)       # 实际对象
    model_no_delay = FOPDT(K=1.0, T=2.0, L=0.0)  # 无滞后模型
    model_with_delay = FOPDT(K=1.0, T=2.0, L=1.5) # 有滞后模型

    pid2 = PIDController(kp=2.0, ki=0.8, kd=0.5)
    y_smith = np.zeros(n)
    u_smith = np.zeros(n)
    ym_nd = 0.0  # 无滞后模型输出
    ym_wd = 0.0  # 有滞后模型输出

    for i in range(n):
        # Smith预估器等效反馈信号
        ym_nd = model_no_delay.update(u_smith[i-1] if i > 0 else 0, dt)
        ym_wd = model_with_delay.update(u_smith[i-1] if i > 0 else 0, dt)

        # 等效误差 = 设定值 - 实际输出 - (无滞后模型 - 有滞后模型)
        smith_compensation = ym_nd - ym_wd
        feedback = plant2.y + smith_compensation
        error = setpoint[i] - feedback

        u_smith[i] = pid2.compute(error, dt)
        y_smith[i] = plant2.update(u_smith[i], dt)

    # ===== 方案3: 优化PID参数的Smith预估器 =====
    plant3 = FOPDT(K=1.0, T=2.0, L=1.5)
    model_nd3 = FOPDT(K=1.0, T=2.0, L=0.0)
    model_wd3 = FOPDT(K=1.0, T=2.0, L=1.5)
    pid3 = PIDController(kp=4.0, ki=2.0, kd=1.0)  # 更激进的参数
    y_smith_opt = np.zeros(n)

    for i in range(n):
        ym_nd3 = model_nd3.update(u_smith[i-1] if i > 0 else 0, dt)
        ym_wd3 = model_wd3.update(u_smith[i-1] if i > 0 else 0, dt)
        comp3 = ym_nd3 - ym_wd3
        fb3 = plant3.y + comp3
        u3 = pid3.compute(setpoint[i] - fb3, dt)
        y_smith_opt[i] = plant3.update(u3, dt)

    # ===== 绘图 =====
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    fig.suptitle('Smith预估器 vs 常规PID（纯滞后系统 L=1.5s, T=2.0s）', fontsize=14, fontweight='bold')

    axes[0].plot(t, setpoint, 'k--', linewidth=2, label='设定值')
    axes[0].plot(t, y_pid, 'b-', linewidth=1.5, label='常规PID')
    axes[0].plot(t, y_smith, 'r-', linewidth=1.5, label='Smith预估器+PID')
    axes[0].plot(t, y_smith_opt, 'm-', linewidth=1.5, label='Smith预估器+优化PID')
    axes[0].set_ylabel('输出')
    axes[0].legend(loc='lower right')
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title('阶跃响应对比')

    # 误差
    axes[1].plot(t, setpoint - y_pid, 'b-', linewidth=1.0, label='常规PID误差')
    axes[1].plot(t, setpoint - y_smith, 'r-', linewidth=1.0, label='Smith误差')
    axes[1].plot(t, setpoint - y_smith_opt, 'm-', linewidth=1.0, label='Smith优化误差')
    axes[1].set_ylabel('误差')
    axes[1].legend(loc='upper right')
    axes[1].grid(True, alpha=0.3)
    axes[1].set_title('跟踪误差')

    axes[2].plot(t, u_pid, 'b-', linewidth=0.8, label='常规PID控制量', alpha=0.7)
    axes[2].plot(t, u_smith, 'r-', linewidth=0.8, label='Smith控制量', alpha=0.7)
    axes[2].set_ylabel('控制量')
    axes[2].set_xlabel('时间 (s)')
    axes[2].legend(loc='upper right')
    axes[2].grid(True, alpha=0.3)
    axes[2].set_title('控制输出')

    plt.tight_layout()
    plt.savefig('smith_predictor_result.png', dpi=150, bbox_inches='tight')
    print('[OK] 仿真完成，图表已保存: smith_predictor_result.png')
    plt.close()


if __name__ == '__main__':
    run_simulation()
