#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
自适应控制仿真 (Adaptive Control Simulation)

本脚本仿真以下自适应控制策略:
1. Model Reference Adaptive Control (MRAC) - 模型参考自适应控制
2. Self-Tuning Regulator (STR) - 自校正调节器
3. Gain Scheduling - 增益调度

适用场景:
- 参数未知或时变的系统
- 电机参数漂移
- 负载变化补偿

Author: nuedc-asset-library
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import rcParams

# 中文显示设置
rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
rcParams['axes.unicode_minus'] = False


class PlantModel:
    """被控对象: 二阶系统 G(s) = b / (s^2 + a1*s + a0)
    参数可能时变"""

    def __init__(self, a0=2.0, a1=1.5, b=1.0):
        self.a0 = a0
        self.a1 = a1
        self.b = b
        # 状态: [x, dx]
        self.x = np.zeros(2)

    def update(self, u, dt, disturbance=0.0):
        """状态更新 (RK4积分)"""
        def deriv(state, u_in):
            x1, x2 = state
            dx1 = x2
            dx2 = -self.a0 * x1 - self.a1 * x2 + self.b * u_in + disturbance
            return np.array([dx1, dx2])

        k1 = deriv(self.x, u)
        k2 = deriv(self.x + 0.5 * dt * k1, u)
        k3 = deriv(self.x + 0.5 * dt * k2, u)
        k4 = deriv(self.x + dt * k3, u)
        self.x = self.x + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
        return self.x[0]

    def set_params(self, a0=None, a1=None, b=None):
        if a0 is not None: self.a0 = a0
        if a1 is not None: self.a1 = a1
        if b is not None: self.b = b

    def reset(self):
        self.x = np.zeros(2)


class MRACController:
    """模型参考自适应控制 (MRAC)

    参考模型: x_m'' + a_m1*x_m' + a_m0*x_m = b_m * r
    控制律:   u = theta1*x1 + theta2*x2 + theta3*r  (前馈+反馈)
    自适应律: dtheta/dt = -gamma * e * phi  (MIT规则)

    其中 e = x - x_m, phi = [x1, x2, r] 为回归向量
    """

    def __init__(self, dt, am0=4.0, am1=4.0, bm=4.0, gamma=5.0):
        self.dt = dt
        # 参考模型参数
        self.am0 = am0
        self.am1 = am1
        self.bm = bm
        # 参考模型状态
        self.xm = np.zeros(2)

        # 自适应增益
        self.gamma = gamma

        # 可调参数 theta = [theta1, theta2, theta3]
        self.theta = np.array([0.0, 0.0, 1.0])

        # 参数限幅
        self.theta_max = 20.0
        self.theta_min = -20.0

        # 输出限幅
        self.u_max = 50.0

        # 积分项 (消除稳态误差)
        self.e_integral = 0.0
        self.ki = 0.5

    def reference_model_update(self, r):
        """更新参考模型状态"""
        def deriv(state, r_in):
            xm1, xm2 = state
            dxm1 = xm2
            dxm2 = -self.am0 * xm1 - self.am1 * xm2 + self.bm * r_in
            return np.array([dxm1, dxm2])

        dt = self.dt
        k1 = deriv(self.xm, r)
        k2 = deriv(self.xm + 0.5 * dt * k1, r)
        k3 = deriv(self.xm + 0.5 * dt * k2, r)
        k4 = deriv(self.xm + dt * k3, r)
        self.xm = self.xm + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
        return self.xm[0]

    def update(self, x_state, r):
        """
        计算控制量
        x_state: [位置, 速度]
        r: 参考输入
        """
        x1, x2 = x_state

        # 参考模型输出
        xm1 = self.reference_model_update(r)
        xm2 = self.xm[1]

        # 跟踪误差
        e = x1 - xm1
        e_dot = x2 - xm2

        # 回归向量
        phi = np.array([x1, x2, r])

        # 自适应律 (MIT规则 + σ修正防止参数漂移)
        sigma = 0.01  # σ修正项
        dtheta = -self.gamma * e * phi - sigma * self.gamma * self.theta
        self.theta = self.theta + dtheta * self.dt

        # 参数限幅
        self.theta = np.clip(self.theta, self.theta_min, self.theta_max)

        # 控制律
        u = np.dot(self.theta, phi) + self.ki * self.e_integral

        # 积分更新
        self.e_integral += e * self.dt
        self.e_integral = np.clip(self.e_integral, -10.0, 10.0)

        # 输出限幅
        u = np.clip(u, -self.u_max, self.u_max)

        return u, xm1

    def reset(self):
        self.xm = np.zeros(2)
        self.theta = np.array([0.0, 0.0, 1.0])
        self.e_integral = 0.0


class STRController:
    """自校正调节器 (Self-Tuning Regulator)

    使用递推最小二乘 (RLS) 在线辨识系统参数,
    然后基于辨识结果计算控制律。

    系统模型: y(k) = -a1*y(k-1) - a0*y(k-2) + b*u(k-1)
    RLS辨识: theta_hat = [a1, a0, b]
    控制律: u = (a1*y(k-1) + a0*y(k-2) + r_desired) / b
    """

    def __init__(self, dt):
        self.dt = dt

        # RLS参数
        self.theta_hat = np.array([0.5, 0.5, 0.5])  # 初始估计
        self.P = np.eye(3) * 100.0  # 协方差矩阵
        self.lambda_rls = 0.98  # 遗忘因子

        # 历史数据
        self.y_history = [0.0, 0.0]
        self.u_history = [0.0]

        # 输出限幅
        self.u_max = 50.0

        # 期望闭环极点
        self.desired_poles = [0.6, 0.6]

    def rls_update(self, y_current):
        """递推最小二乘参数更新"""
        # 回归向量: phi = [-y(k-1), -y(k-2), u(k-1)]
        phi = np.array([
            -self.y_history[-1],
            -self.y_history[-2],
            self.u_history[-1]
        ])

        # 预测误差
        y_pred = np.dot(self.theta_hat, phi)
        e = y_current - y_pred

        # RLS更新
        denom = self.lambda_rls + np.dot(phi, np.dot(self.P, phi))
        K = np.dot(self.P, phi) / denom  # 增益向量
        self.theta_hat = self.theta_hat + K * e
        self.P = (self.P - np.outer(K, np.dot(phi, self.P))) / self.lambda_rls

        # 协方差限幅 (防止爆炸)
        self.P = np.clip(self.P, -1e6, 1e6)

        return e

    def update(self, y_current, r):
        """计算控制量"""
        # RLS参数辨识
        self.rls_update(y_current)

        # 辨识出的参数
        a1_hat = self.theta_hat[0]
        a0_hat = self.theta_hat[1]
        b_hat = self.theta_hat[2]

        # 避免除零
        if abs(b_hat) < 0.01:
            b_hat = 0.01 if b_hat >= 0 else -0.01

        # 极点配置控制律
        # 期望闭环: y(k) = p1*y(k-1) + p2*y(k-2) + (1-p1-p2)*r
        p1, p2 = self.desired_poles
        u = (p1 * self.y_history[-1] + p2 * self.y_history[-2]
             + (1 - p1 - p2) * r + a1_hat * self.y_history[-1]
             + a0_hat * self.y_history[-2]) / b_hat

        # 简化控制律 (直接求逆)
        u = (r - a1_hat * self.y_history[-1] - a0_hat * self.y_history[-2]) / b_hat

        # 输出限幅
        u = np.clip(u, -self.u_max, self.u_max)

        # 更新历史
        self.y_history.append(y_current)
        if len(self.y_history) > 10:
            self.y_history = self.y_history[-10:]
        self.u_history.append(u)
        if len(self.u_history) > 10:
            self.u_history = self.u_history[-10:]

        return u

    def reset(self):
        self.theta_hat = np.array([0.5, 0.5, 0.5])
        self.P = np.eye(3) * 100.0
        self.y_history = [0.0, 0.0]
        self.u_history = [0.0]


class GainSchedulingController:
    """增益调度控制器 (Gain Scheduling)

    根据调度变量(如工作点速度)选择不同的控制器参数。
    参数表通过插值实现平滑切换。

    适用场景: 工作点变化已知的系统(如变桨距风力发电、不同飞行状态)
    """

    def __init__(self, dt):
        self.dt = dt

        # 调度变量 → 控制参数表
        # (调度值, kp, ki, kd)
        self.schedule_table = [
            (0.0,  2.0, 0.5, 0.3),
            (5.0,  3.0, 0.8, 0.5),
            (10.0, 4.0, 1.0, 0.7),
            (20.0, 3.5, 0.9, 0.6),
            (30.0, 2.5, 0.6, 0.4),
        ]

        self.error_prev = 0.0
        self.e_integral = 0.0
        self.u_max = 50.0

    def get_params(self, scheduling_var):
        """根据调度变量插值获取PID参数"""
        sv = scheduling_var

        # 边界处理
        if sv <= self.schedule_table[0][0]:
            return self.schedule_table[0][1], self.schedule_table[0][2], self.schedule_table[0][3]
        if sv >= self.schedule_table[-1][0]:
            return self.schedule_table[-1][1], self.schedule_table[-1][2], self.schedule_table[-1][3]

        # 线性插值
        for i in range(len(self.schedule_table) - 1):
            sv0, kp0, ki0, kd0 = self.schedule_table[i]
            sv1, kp1, ki1, kd1 = self.schedule_table[i + 1]
            if sv0 <= sv <= sv1:
                alpha = (sv - sv0) / (sv1 - sv0)
                kp = kp0 + alpha * (kp1 - kp0)
                ki = ki0 + alpha * (ki1 - ki0)
                kd = kd0 + alpha * (kd1 - kd0)
                return kp, ki, kd

        return self.schedule_table[-1][1], self.schedule_table[-1][2], self.schedule_table[-1][3]

    def update(self, error, scheduling_var):
        """计算控制量"""
        kp, ki, kd = self.get_params(scheduling_var)

        self.e_integral += error * self.dt
        self.e_integral = np.clip(self.e_integral, -20.0, 20.0)

        e_dot = (error - self.error_prev) / self.dt
        self.error_prev = error

        u = kp * error + ki * self.e_integral + kd * e_dot
        u = np.clip(u, -self.u_max, self.u_max)

        return u

    def reset(self):
        self.error_prev = 0.0
        self.e_integral = 0.0


def run_simulation():
    """运行自适应控制仿真"""
    print("=" * 60)
    print("自适应控制仿真 (Adaptive Control Simulation)")
    print("=" * 60)

    # 仿真参数
    dt = 0.001
    T = 10.0
    N = int(T / dt)
    t = np.linspace(0, T, N)

    # 参考信号
    freq = 0.5
    r = np.sin(2 * np.pi * freq * t) * 2.0

    # ====== 1. MRAC 仿真 ======
    print("\n[1] 仿真 MRAC (模型参考自适应控制)...")
    plant_mrac = PlantModel(a0=2.0, a1=1.5, b=1.0)
    mrac = MRACController(dt, am0=9.0, am1=6.0, bm=9.0, gamma=3.0)

    y_mrac = np.zeros(N)
    xm_mrac = np.zeros(N)
    u_mrac = np.zeros(N)
    theta_hist = np.zeros((N, 3))

    for i in range(N):
        y_mrac[i] = plant_mrac.x[0]

        # 在t=5s时改变被控对象参数 (模拟参数突变)
        if i == int(5.0 / dt):
            plant_mrac.set_params(a0=3.5, a1=2.5, b=1.2)
            print(f"  t=5.0s: 参数突变 a0: 2.0→3.5, a1: 1.5→2.5, b: 1.0→1.2")

        u_val, xm_val = mrac.update(plant_mrac.x, r[i])
        u_mrac[i] = u_val
        xm_mrac[i] = xm_val
        theta_hist[i] = mrac.theta.copy()

        disturbance = 0.1 * np.sin(2 * np.pi * 5 * t[i])  # 高频扰动
        plant_mrac.update(u_val, dt, disturbance)

    mrac_error = np.mean(np.abs(y_mrac - xm_mrac))
    print(f"  MRAC 平均跟踪误差: {mrac_error:.4f}")

    # ====== 2. STR 仿真 ======
    print("\n[2] 仿真 STR (自校正调节器)...")
    plant_str = PlantModel(a0=2.0, a1=1.5, b=1.0)
    str_ctrl = STRController(dt)

    y_str = np.zeros(N)
    u_str = np.zeros(N)
    param_hist = np.zeros((N, 3))

    for i in range(N):
        y_str[i] = plant_str.x[0]

        # t=3s参数变化
        if i == int(3.0 / dt):
            plant_str.set_params(a0=1.0, a1=0.8, b=1.5)
            print(f"  t=3.0s: 参数变化 a0: 2.0→1.0, a1: 1.5→0.8, b: 1.0→1.5")

        u_val = str_ctrl.update(y_str[i], r[i])
        u_str[i] = u_val
        param_hist[i] = str_ctrl.theta_hat.copy()

        plant_str.update(u_val, dt)

    str_error = np.mean(np.abs(y_str - r))
    print(f"  STR 平均跟踪误差: {str_error:.4f}")

    # ====== 3. 增益调度仿真 ======
    print("\n[3] 仿真 增益调度 (Gain Scheduling)...")
    plant_gs = PlantModel(a0=2.0, a1=1.5, b=1.0)
    gs = GainSchedulingController(dt)

    y_gs = np.zeros(N)
    u_gs = np.zeros(N)
    scheduling_var = np.zeros(N)

    for i in range(N):
        y_gs[i] = plant_gs.x[0]

        # 调度变量 = |速度| (模拟不同工作点)
        scheduling_var[i] = abs(plant_gs.x[1]) * 3.0

        error = r[i] - y_gs[i]
        u_val = gs.update(error, scheduling_var[i])
        u_gs[i] = u_val

        plant_gs.update(u_val, dt)

    gs_error = np.mean(np.abs(y_gs - r))
    print(f"  增益调度 平均跟踪误差: {gs_error:.4f}")

    # ====== 绘图 ======
    print("\n生成仿真图表...")

    fig, axes = plt.subplots(3, 2, figsize=(16, 14))

    # --- MRAC ---
    ax = axes[0, 0]
    ax.plot(t, r, 'k--', alpha=0.5, label='参考信号 r')
    ax.plot(t, xm_mrac, 'g-', linewidth=1.5, label='参考模型 x_m')
    ax.plot(t, y_mrac, 'b-', linewidth=1.0, label='实际输出 y (MRAC)')
    ax.axvline(x=5.0, color='r', linestyle=':', alpha=0.5, label='参数突变时刻')
    ax.set_title('MRAC 跟踪效果')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('输出')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[0, 1]
    ax.plot(t, theta_hist[:, 0], label=r'$\theta_1$ (反馈增益1)')
    ax.plot(t, theta_hist[:, 1], label=r'$\theta_2$ (反馈增益2)')
    ax.plot(t, theta_hist[:, 2], label=r'$\theta_3$ (前馈增益)')
    ax.axvline(x=5.0, color='r', linestyle=':', alpha=0.5)
    ax.set_title('MRAC 自适应参数变化')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('参数值')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # --- STR ---
    ax = axes[1, 0]
    ax.plot(t, r, 'k--', alpha=0.5, label='参考信号 r')
    ax.plot(t, y_str, 'b-', linewidth=1.0, label='实际输出 y (STR)')
    ax.axvline(x=3.0, color='r', linestyle=':', alpha=0.5, label='参数变化时刻')
    ax.set_title('STR 跟踪效果')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('输出')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[1, 1]
    ax.plot(t, param_hist[:, 0], label=r'$\hat{a}_1$ (辨识)')
    ax.plot(t, param_hist[:, 1], label=r'$\hat{a}_0$ (辨识)')
    ax.plot(t, param_hist[:, 2], label=r'$\hat{b}$ (辨识)')
    ax.axvline(x=3.0, color='r', linestyle=':', alpha=0.5)
    ax.axhline(y=0.8, color='gray', linestyle='--', alpha=0.3)
    ax.axhline(y=1.0, color='gray', linestyle='--', alpha=0.3)
    ax.axhline(y=1.5, color='gray', linestyle='--', alpha=0.3)
    ax.set_title('STR 在线参数辨识')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('参数值')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # --- Gain Scheduling ---
    ax = axes[2, 0]
    ax.plot(t, r, 'k--', alpha=0.5, label='参考信号 r')
    ax.plot(t, y_gs, 'b-', linewidth=1.0, label='实际输出 y (增益调度)')
    ax.set_title('增益调度控制 跟踪效果')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('输出')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[2, 1]
    ax.plot(t, scheduling_var, 'r-', alpha=0.7, label='调度变量 (|速度|)')
    ax.set_title('增益调度变量')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('调度变量值')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    plt.suptitle('自适应控制仿真对比', fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig('adaptive_control_simulation.png', dpi=150, bbox_inches='tight')
    print("图表已保存: adaptive_control_simulation.png")
    plt.close('all')


if __name__ == '__main__':
    run_simulation()
