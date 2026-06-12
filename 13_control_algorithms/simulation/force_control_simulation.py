#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
力控制仿真 - 阻抗控制与导纳控制
====================================
仿真内容:
  1. 阻抗控制: 机器人末端跟踪期望轨迹, 遇到环境后产生接触力
  2. 导纳控制: 机器人受到外力后, 通过导纳模型修正运动
  3. 力/位混合控制: 某轴力控、某轴位控
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
matplotlib.rcParams['axes.unicode_minus'] = False


class ImpedanceController:
    """阻抗控制器: 设定运动 → 输出力"""

    def __init__(self, M, B, K, dt):
        self.M = M
        self.B = B
        self.K = K
        self.dt = dt
        self.x = 0.0
        self.dx = 0.0

    def update(self, x_des, dx_des, F_ext):
        x_err = x_des - self.x
        dx_err = dx_des - self.dx
        ddx = (F_ext - self.B * self.dx - self.K * x_err) / self.M
        self.dx += ddx * self.dt
        self.x += self.dx * self.dt
        F_out = self.K * x_err + self.B * dx_err + F_ext
        return F_out

    def reset(self):
        self.x = 0.0
        self.dx = 0.0


class AdmittanceController:
    """导纳控制器: 测量力 → 输出位置修正"""

    def __init__(self, Md, Bd, Kd, dt):
        self.Md = Md
        self.Bd = Bd
        self.Kd = Kd
        self.dt = dt
        self.x_cmd = 0.0
        self.dx_cmd = 0.0

    def update(self, F_des, F_meas):
        F_err = F_des - F_meas
        ddx = (F_err - self.Bd * self.dx_cmd - self.Kd * self.x_cmd) / self.Md
        dx_new = self.dx_cmd + ddx * self.dt
        self.x_cmd += 0.5 * (self.dx_cmd + dx_new) * self.dt
        self.dx_cmd = dx_new
        return self.x_cmd

    def reset(self):
        self.x_cmd = 0.0
        self.dx_cmd = 0.0


class Environment:
    """环境模型 (弹簧-阻尼)"""

    def __init__(self, Ke, Be, x_wall):
        self.Ke = Ke  # 环境刚度
        self.Be = Be  # 环境阻尼
        self.x_wall = x_wall  # 墙壁位置

    def contact_force(self, x, dx):
        if x >= self.x_wall:
            F = self.Ke * (x - self.x_wall) + self.Be * dx
            return max(F, 0)
        return 0.0


def simulate_impedance():
    """阻抗控制仿真: 跟踪正弦轨迹, 遇到墙壁"""
    dt = 0.001
    T = 4.0
    steps = int(T / dt)

    # 控制器参数
    ctrl = ImpedanceController(M=1.0, B=50.0, K=500.0, dt=dt)
    env = Environment(Ke=10000, Be=100, x_wall=0.5)

    # 机器人简化模型 (质量-阻尼)
    m_robot = 2.0
    b_robot = 10.0
    x_robot = 0.0
    dx_robot = 0.0

    time_arr = np.zeros(steps)
    x_des_arr = np.zeros(steps)
    x_robot_arr = np.zeros(steps)
    F_contact_arr = np.zeros(steps)
    F_ctrl_arr = np.zeros(steps)

    for i in range(steps):
        t = i * dt
        time_arr[i] = t

        # 期望轨迹: 正弦运动
        x_des = 0.3 + 0.3 * np.sin(2 * np.pi * 0.5 * t)
        dx_des = 0.3 * 2 * np.pi * 0.5 * np.cos(2 * np.pi * 0.5 * t)

        # 环境接触力
        F_env = env.contact_force(x_robot, dx_robot)

        # 阻抗控制器
        F_ctrl = ctrl.update(x_des, dx_des, F_env)

        # 机器人动力学
        ddx = (F_ctrl - F_env - b_robot * dx_robot) / m_robot
        dx_robot += ddx * dt
        x_robot += dx_robot * dt

        x_des_arr[i] = x_des
        x_robot_arr[i] = x_robot
        F_contact_arr[i] = F_env
        F_ctrl_arr[i] = F_ctrl

    # 绘图
    fig, axes = plt.subplots(2, 1, figsize=(10, 7))
    fig.suptitle('阻抗控制仿真 - 跟踪正弦轨迹遇到墙壁', fontsize=14)

    axes[0].plot(time_arr, x_des_arr, 'b--', label='期望轨迹')
    axes[0].plot(time_arr, x_robot_arr, 'r-', label='实际轨迹')
    axes[0].axhline(y=0.5, color='k', linestyle=':', label='墙壁位置')
    axes[0].set_ylabel('位置 (m)')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(time_arr, F_contact_arr, 'g-', label='接触力')
    axes[1].plot(time_arr, F_ctrl_arr, 'm-', alpha=0.6, label='控制力')
    axes[1].set_xlabel('时间 (s)')
    axes[1].set_ylabel('力 (N)')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('impedance_control.png', dpi=150)
    plt.close('all')
    print("阻抗控制仿真完成, 图片已保存: impedance_control.png")


def simulate_admittance():
    """导纳控制仿真: 受外力后修正运动"""
    dt = 0.001
    T = 3.0
    steps = int(T / dt)

    ctrl = AdmittanceController(Md=2.0, Bd=100.0, Kd=500.0, dt=dt)

    # 期望接触力
    F_des = 10.0

    # 机器人
    x_robot = 0.0
    x_wall = 0.3

    time_arr = np.zeros(steps)
    x_arr = np.zeros(steps)
    F_meas_arr = np.zeros(steps)
    x_cmd_arr = np.zeros(steps)

    for i in range(steps):
        t = i * dt
        time_arr[i] = t

        # 模拟测量力 (接触时)
        if x_robot >= x_wall:
            F_meas = 5000.0 * (x_robot - x_wall)
        else:
            F_meas = 0.0

        # 导纳控制器输出位置修正
        x_cmd = ctrl.update(F_des, F_meas)

        # 机器人位置 = 基础运动 + 导纳修正
        x_base = 0.5 * t  # 匀速前进
        x_robot = min(x_base + x_cmd, x_wall + 0.01)

        x_arr[i] = x_robot
        F_meas_arr[i] = F_meas
        x_cmd_arr[i] = x_cmd

    fig, axes = plt.subplots(2, 1, figsize=(10, 7))
    fig.suptitle('导纳控制仿真 - 接触力跟踪', fontsize=14)

    axes[0].plot(time_arr, x_arr, 'r-', label='实际位置')
    axes[0].axhline(y=x_wall, color='k', linestyle=':', label='墙壁位置')
    axes[0].set_ylabel('位置 (m)')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(time_arr, F_meas_arr, 'g-', label='测量力')
    axes[1].axhline(y=F_des, color='b', linestyle='--', label='期望力')
    axes[1].set_xlabel('时间 (s)')
    axes[1].set_ylabel('力 (N)')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('admittance_control.png', dpi=150)
    plt.close('all')
    print("导纳控制仿真完成, 图片已保存: admittance_control.png")


def simulate_hybrid_force_position():
    """力/位混合控制仿真"""
    dt = 0.001
    T = 2.0
    steps = int(T / dt)

    Kp_pos = 200.0
    Kp_force = 0.01
    Ki_force = 0.5
    max_force = 50.0

    # X轴: 位控, Y轴: 力控
    x_pos, y_pos = 0.0, 0.0
    force_integral = 0.0

    F_des_y = 10.0  # Y轴期望力
    Ke_y = 5000.0    # Y方向环境刚度

    time_arr = np.zeros(steps)
    x_arr = np.zeros(steps)
    y_arr = np.zeros(steps)
    F_y_arr = np.zeros(steps)
    u_x_arr = np.zeros(steps)
    u_y_arr = np.zeros(steps)

    for i in range(steps):
        t = i * dt
        time_arr[i] = t

        # X轴位控: 跟踪正弦
        x_des = 0.5 * np.sin(2 * np.pi * t)
        x_err = x_des - x_pos
        u_x = Kp_pos * x_err
        x_pos += u_x * dt * 0.01

        # Y轴力控: 接触力跟踪
        y_wall = 0.1
        if y_pos >= y_wall:
            F_meas_y = Ke_y * (y_pos - y_wall)
        else:
            F_meas_y = 0.0

        force_err = F_des_y - F_meas_y
        force_integral += force_err * dt
        u_y = Kp_force * force_err + Ki_force * force_integral
        u_y = np.clip(u_y, -max_force, max_force)
        y_pos += u_y * dt * 0.001

        x_arr[i] = x_pos
        y_arr[i] = y_pos
        F_y_arr[i] = F_meas_y
        u_x_arr[i] = u_x
        u_y_arr[i] = u_y

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle('力/位混合控制仿真 - X轴位控 + Y轴力控', fontsize=14)

    axes[0, 0].plot(time_arr, x_arr, 'b-')
    axes[0, 0].set_ylabel('X位置 (m)')
    axes[0, 0].set_title('X轴 (位控)')
    axes[0, 0].grid(True, alpha=0.3)

    axes[0, 1].plot(time_arr, y_arr, 'r-')
    axes[0, 1].axhline(y=0.1, color='k', linestyle=':')
    axes[0, 1].set_ylabel('Y位置 (m)')
    axes[0, 1].set_title('Y轴 (力控)')
    axes[0, 1].grid(True, alpha=0.3)

    axes[1, 0].plot(time_arr, u_x_arr, 'b-')
    axes[1, 0].set_xlabel('时间 (s)')
    axes[1, 0].set_ylabel('X控制量')
    axes[1, 0].grid(True, alpha=0.3)

    axes[1, 1].plot(time_arr, F_y_arr, 'r-', label='测量力')
    axes[1, 1].axhline(y=F_des_y, color='b', linestyle='--', label='期望力')
    axes[1, 1].set_xlabel('时间 (s)')
    axes[1, 1].set_ylabel('Y轴力 (N)')
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('hybrid_force_position.png', dpi=150)
    plt.close('all')
    print("力/位混合控制仿真完成, 图片已保存: hybrid_force_position.png")


if __name__ == '__main__':
    simulate_impedance()
    simulate_admittance()
    simulate_hybrid_force_position()
    print("\n所有力控制仿真完成!")
