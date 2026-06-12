#!/usr/bin/env python3
"""
串级PID仿真 - 位置环+速度环+电流环
=====================================
模拟典型的电机三环串级PID控制系统：
  位置环(外环) → 速度环(中环) → 电流环(内环) → 电机模型

运行方式: python cascade_pid_simulation.py
输出图表: cascade_pid_result.png
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False


class PIDController:
    """通用PID控制器"""
    def __init__(self, kp, ki, kd, output_min=-100, output_max=100):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        self.integral = np.clip(self.integral,
                                self.output_min / max(self.ki, 1e-6),
                                self.output_max / max(self.ki, 1e-6))
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return np.clip(output, self.output_min, self.output_max)

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


class MotorModel:
    """简化的直流电机模型：电流→角速度→角度"""
    def __init__(self, J=0.01, b=0.1, Kt=0.1, R=1.0, L=0.01):
        self.J = J      # 转动惯量
        self.b = b      # 粘滞摩擦系数
        self.Kt = Kt    # 转矩常数
        self.R = R      # 电阻
        self.L = L      # 电感
        self.omega = 0.0   # 角速度
        self.theta = 0.0   # 角位置
        self.current = 0.0  # 电流

    def update(self, voltage, dt):
        """给定电压，更新电机状态"""
        # 电流环（电气模型）
        di = (voltage - self.R * self.current - self.Kt * self.omega) / self.L
        self.current += di * dt
        # 力矩
        torque = self.Kt * self.current
        # 机械模型
        domega = (torque - self.b * self.omega) / self.J
        self.omega += domega * dt
        self.theta += self.omega * dt
        return self.theta, self.omega, self.current


def run_simulation():
    """运行串级PID仿真"""
    dt = 0.001  # 1ms 采样
    t_total = 2.0
    steps = int(t_total / dt)
    t = np.linspace(0, t_total, steps)

    # 目标位置（方波）
    target_pos = np.zeros(steps)
    target_pos[steps//4:] = 10.0
    target_pos[steps//2:] = -5.0
    target_pos[3*steps//4:] = 8.0

    # 控制器
    pos_pid = PIDController(kp=2.0, ki=0.5, kd=0.3, output_min=-50, output_max=50)
    vel_pid = PIDController(kp=5.0, ki=2.0, kd=0.1, output_min=-12, output_max=12)
    cur_pid = PIDController(kp=10.0, ki=50.0, kd=0.0, output_min=-24, output_max=24)

    motor = MotorModel()

    # 记录数据
    pos_log = np.zeros(steps)
    vel_log = np.zeros(steps)
    cur_log = np.zeros(steps)
    vel_cmd_log = np.zeros(steps)
    cur_cmd_log = np.zeros(steps)

    for i in range(steps):
        # 位置环
        pos_error = target_pos[i] - motor.theta
        vel_cmd = pos_pid.compute(pos_error, dt)

        # 速度环
        vel_error = vel_cmd - motor.omega
        cur_cmd = vel_pid.compute(vel_error, dt)

        # 电流环
        cur_error = cur_cmd - motor.current
        voltage = cur_pid.compute(cur_error, dt)

        # 电机更新
        motor.update(voltage, dt)

        pos_log[i] = motor.theta
        vel_log[i] = motor.omega
        cur_log[i] = motor.current
        vel_cmd_log[i] = vel_cmd
        cur_cmd_log[i] = cur_cmd

    # ===== 绘图 =====
    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    fig.suptitle('串级PID控制仿真（位置环+速度环+电流环）', fontsize=14, fontweight='bold')

    # 位置环
    axes[0].plot(t, target_pos, 'r--', linewidth=2, label='目标位置')
    axes[0].plot(t, pos_log, 'b-', linewidth=1.2, label='实际位置')
    axes[0].set_ylabel('位置 (rad)')
    axes[0].legend(loc='upper right')
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title('位置环响应')

    # 速度环
    axes[1].plot(t, vel_cmd_log, 'r--', linewidth=1.5, label='速度指令')
    axes[1].plot(t, vel_log, 'b-', linewidth=1.2, label='实际速度')
    axes[1].set_ylabel('角速度 (rad/s)')
    axes[1].legend(loc='upper right')
    axes[1].grid(True, alpha=0.3)
    axes[1].set_title('速度环响应')

    # 电流环
    axes[2].plot(t, cur_cmd_log, 'r--', linewidth=1.5, label='电流指令')
    axes[2].plot(t, cur_log, 'b-', linewidth=1.2, label='实际电流')
    axes[2].set_ylabel('电流 (A)')
    axes[2].legend(loc='upper right')
    axes[2].grid(True, alpha=0.3)
    axes[2].set_title('电流环响应')

    # 位置误差
    error = target_pos - pos_log
    axes[3].plot(t, error, 'g-', linewidth=1.2)
    axes[3].set_ylabel('位置误差 (rad)')
    axes[3].set_xlabel('时间 (s)')
    axes[3].grid(True, alpha=0.3)
    axes[3].set_title('位置跟踪误差')

    plt.tight_layout()
    plt.savefig('cascade_pid_result.png', dpi=150, bbox_inches='tight')
    print('[OK] 仿真完成，图表已保存: cascade_pid_result.png')
    plt.close()


if __name__ == '__main__':
    run_simulation()
