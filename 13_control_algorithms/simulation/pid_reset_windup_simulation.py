#!/usr/bin/env python3
"""
积分重置抗饱和PID仿真

研究内容:
  1. 精确重置(Exact Reset) vs 条件积分 vs 无抗饱和
  2. 不同饱和程度下的恢复速度对比
  3. 积分重置对超调量的抑制效果
  4. 死区重置消除稳态微振荡
  5. 双向饱和场景下的表现
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from dataclasses import dataclass
from typing import Callable

# ============================================================
# 被控对象: 二阶系统
# ============================================================
@dataclass
class SecondOrderPlant:
    """二阶传递函数: wn^2 / (s^2 + 2*zeta*wn*s + wn^2)"""
    wn: float = 15.0
    zeta: float = 0.2
    dt: float = 0.001
    x1: float = 0.0
    x2: float = 0.0

    def update(self, u: float) -> float:
        dx1 = self.x2
        dx2 = -self.wn**2 * self.x1 - 2*self.zeta*self.wn*self.x2 + self.wn**2 * u
        self.x1 += dx1 * self.dt
        self.x2 += dx2 * self.dt
        return self.x1

    def reset(self):
        self.x1 = 0.0
        self.x2 = 0.0


# ============================================================
# PID + 积分重置抗饱和
# ============================================================
class PIDResetWindup:
    """积分重置抗饱和PID控制器"""

    # 抗饱和模式
    MODE_NONE         = 0  # 无抗饱和
    MODE_EXACT_RESET  = 1  # 精确重置
    MODE_CONDITIONAL  = 2  # 条件积分
    MODE_DEADZONE     = 3  # 死区重置

    def __init__(self, kp, ki, kd, dt, out_min=-5.0, out_max=5.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.out_min = out_min
        self.out_max = out_max

        # 状态
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_d_term = 0.0
        self.first_run = True

        # 配置
        self.mode = self.MODE_EXACT_RESET
        self.deadzone = 0.0
        self.d_alpha = 0.2  # 微分滤波

        # 记录
        self.saturated = False

    def compute(self, setpoint, pv):
        error = setpoint - pv

        # P
        p_term = self.kp * error

        # I (梯形积分)
        self.integral += 0.5 * (error + self.prev_error) * self.dt
        i_term = self.ki * self.integral

        # 死区
        if self.mode == self.MODE_DEADZONE and abs(error) < self.deadzone:
            self.integral = 0.0
            i_term = 0.0

        # D
        if self.first_run:
            d_raw = 0.0
        else:
            d_raw = self.kd * (error - self.prev_error) / self.dt
        d_filtered = self.d_alpha * d_raw + (1 - self.d_alpha) * self.prev_d_term
        self.prev_d_term = d_filtered

        # 原始输出
        raw_output = p_term + i_term + d_filtered

        # 抗饱和
        self.saturated = False
        final_output = raw_output

        if raw_output > self.out_max:
            self.saturated = True
            final_output = self.out_max
            if self.mode == self.MODE_EXACT_RESET and abs(self.ki) > 1e-9:
                self.integral = (self.out_max - p_term - d_filtered) / self.ki
            elif self.mode == self.MODE_CONDITIONAL:
                self.integral -= 0.5 * (error + self.prev_error) * self.dt
        elif raw_output < self.out_min:
            self.saturated = True
            final_output = self.out_min
            if self.mode == self.MODE_EXACT_RESET and abs(self.ki) > 1e-9:
                self.integral = (self.out_min - p_term - d_filtered) / self.ki
            elif self.mode == self.MODE_CONDITIONAL:
                self.integral -= 0.5 * (error + self.prev_error) * self.dt

        self.prev_error = error
        self.first_run = False

        return final_output, p_term, i_term, d_filtered

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_d_term = 0.0
        self.first_run = True


# ============================================================
# 仿真运行器
# ============================================================
def run_sim(pid, plant, sp_func, n_steps):
    t = np.zeros(n_steps)
    sp = np.zeros(n_steps)
    pv = np.zeros(n_steps)
    u = np.zeros(n_steps)
    i_term = np.zeros(n_steps)
    sat = np.zeros(n_steps)

    plant.reset()
    pid.reset()

    for i in range(n_steps):
        t[i] = i * pid.dt
        sp[i] = sp_func(t[i])
        pv[i] = plant.x1

        output, _, i_val, _ = pid.compute(sp[i], pv[i])
        u[i] = output
        i_term[i] = i_val
        sat[i] = 1.0 if pid.saturated else 0.0

        plant.update(output)

    return t, sp, pv, u, i_term, sat


# ============================================================
# 实验1: 三种抗饱和策略对比
# ============================================================
def experiment_compare_strategies():
    """对比无抗饱和、条件积分、精确重置"""
    dt = 0.001
    n_steps = 5000

    def sp_func(t):
        """阶跃设定值, 故意让输出饱和"""
        return 1.5 if t > 0.3 else 0.0

    modes = [
        (PIDResetWindup.MODE_NONE, '无抗饱和', 'red'),
        (PIDResetWindup.MODE_CONDITIONAL, '条件积分', 'blue'),
        (PIDResetWindup.MODE_EXACT_RESET, '精确重置', 'green'),
    ]

    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    fig.suptitle('实验1: 积分重置抗饱和策略对比', fontsize=14, fontweight='bold')

    for mode, label, color in modes:
        plant = SecondOrderPlant(dt=dt)
        pid = PIDResetWindup(kp=4.0, ki=8.0, kd=0.3, dt=dt,
                             out_min=-3.0, out_max=3.0)
        pid.mode = mode
        t, sp, pv, u, i_arr, sat = run_sim(pid, plant, sp_func, n_steps)

        axes[0].plot(t, pv, color=color, label=label, linewidth=1.0)
        axes[1].plot(t, u, color=color, label=label, linewidth=1.0)
        axes[2].plot(t, i_arr, color=color, label=label, linewidth=1.0)

    axes[0].plot(t, sp, 'k--', label='设定值', linewidth=1.5)
    axes[0].set_ylabel('PV')
    axes[0].set_title('系统响应 (注意超调和恢复速度)')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].axhline(y=3.0, color='gray', linestyle=':', alpha=0.5, label='输出上限')
    axes[1].axhline(y=-3.0, color='gray', linestyle=':', alpha=0.5, label='输出下限')
    axes[1].set_ylabel('控制输出 u')
    axes[1].set_title('控制输出 (观察饱和区域)')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    axes[2].set_ylabel('积分项值')
    axes[2].set_xlabel('时间 (s)')
    axes[2].set_title('积分项演变 (windup vs 重置)')
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'pid_rw_exp1_compare.png'), dpi=150)
    plt.close('all')


# ============================================================
# 实验2: 不同饱和程度下的恢复速度
# ============================================================
def experiment_saturation_levels():
    """对比不同程度饱和下的恢复效果"""
    dt = 0.001
    n_steps = 5000

    setpoints = [0.8, 1.5, 3.0]  # 轻度、中度、重度饱和
    labels = ['轻度饱和', '中度饱和', '重度饱和']

    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    fig.suptitle('实验2: 不同饱和程度下的积分重置效果', fontsize=14, fontweight='bold')

    for sp_val, label in zip(setpoints, labels):
        def sp_func(t, sv=sp_val):
            return sv if t > 0.2 else 0.0

        # 精确重置
        plant = SecondOrderPlant(dt=dt)
        pid = PIDResetWindup(kp=4.0, ki=8.0, kd=0.3, dt=dt,
                             out_min=-3.0, out_max=3.0)
        pid.mode = PIDResetWindup.MODE_EXACT_RESET
        t, sp, pv, u, i_arr, sat = run_sim(pid, plant, sp_func, n_steps)

        axes[0].plot(t, pv, label=f'{label} (SP={sp_val})', linewidth=1.0)
        axes[1].plot(t, i_arr, label=f'{label} (SP={sp_val})', linewidth=1.0)

    axes[0].set_ylabel('PV')
    axes[0].set_title('系统响应')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].set_ylabel('积分项值')
    axes[1].set_xlabel('时间 (s)')
    axes[1].set_title('积分项 (重置效果)')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'pid_rw_exp2_levels.png'), dpi=150)
    plt.close('all')


# ============================================================
# 实验3: 死区重置消除稳态振荡
# ============================================================
def experiment_deadzone():
    """对比有无死区重置的稳态表现"""
    dt = 0.001
    n_steps = 8000
    noise_std = 0.02

    def sp_func(t):
        return 1.0 if t > 0.3 else 0.0

    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    fig.suptitle('实验3: 死区重置消除稳态微振荡', fontsize=14, fontweight='bold')

    # 无死区
    plant = SecondOrderPlant(dt=dt)
    pid = PIDResetWindup(kp=3.0, ki=6.0, kd=0.2, dt=dt)
    pid.mode = PIDResetWindup.MODE_EXACT_RESET

    t_arr = np.zeros(n_steps)
    pv_no_dz = np.zeros(n_steps)
    plant.reset()
    pid.reset()
    for i in range(n_steps):
        t_arr[i] = i * dt
        sp = sp_func(t_arr[i])
        noisy_pv = plant.x1 + np.random.normal(0, noise_std)
        pv_no_dz[i] = noisy_pv
        u, _, _, _ = pid.compute(sp, noisy_pv)
        plant.update(u)

    # 有死区
    plant = SecondOrderPlant(dt=dt)
    pid = PIDResetWindup(kp=3.0, ki=6.0, kd=0.2, dt=dt)
    pid.mode = PIDResetWindup.MODE_DEADZONE
    pid.deadzone = 0.05

    pv_dz = np.zeros(n_steps)
    plant.reset()
    pid.reset()
    for i in range(n_steps):
        sp = sp_func(t_arr[i])
        noisy_pv = plant.x1 + np.random.normal(0, noise_std)
        pv_dz[i] = noisy_pv
        u, _, _, _ = pid.compute(sp, noisy_pv)
        plant.update(u)

    axes[0].plot(t_arr, pv_no_dz, label='无死区', linewidth=0.8, alpha=0.8)
    axes[0].plot(t_arr, pv_dz, label='有死区(±0.05)', linewidth=0.8, alpha=0.8)
    axes[0].set_ylabel('PV')
    axes[0].set_title('全程响应')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    # 放大稳态区域
    mask = t_arr > 1.0
    axes[1].plot(t_arr[mask], pv_no_dz[mask], label='无死区', linewidth=0.8)
    axes[1].plot(t_arr[mask], pv_dz[mask], label='有死区(±0.05)', linewidth=0.8)
    axes[1].set_ylabel('PV')
    axes[1].set_xlabel('时间 (s)')
    axes[1].set_title('稳态区域放大 (t>1s, 观察振荡幅度)')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'pid_rw_exp3_deadzone.png'), dpi=150)
    plt.close('all')


# ============================================================
# 实验4: 双向饱和(正负交替设定值)
# ============================================================
def experiment_bidirectional():
    """双向饱和场景"""
    dt = 0.001
    n_steps = 6000

    def sp_func(t):
        if t < 1.0:
            return 0.0
        elif t < 2.5:
            return 2.0
        elif t < 4.0:
            return -2.0
        else:
            return 1.0

    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    fig.suptitle('实验4: 双向饱和场景', fontsize=14, fontweight='bold')

    for mode, label, color in [
        (PIDResetWindup.MODE_NONE, '无抗饱和', 'red'),
        (PIDResetWindup.MODE_EXACT_RESET, '精确重置', 'green'),
    ]:
        plant = SecondOrderPlant(dt=dt)
        pid = PIDResetWindup(kp=4.0, ki=8.0, kd=0.3, dt=dt,
                             out_min=-3.0, out_max=3.0)
        pid.mode = mode
        t, sp, pv, u, i_arr, sat = run_sim(pid, plant, sp_func, n_steps)

        axes[0].plot(t, pv, color=color, label=label, linewidth=1.0)
        axes[1].plot(t, u, color=color, label=label, linewidth=1.0)
        axes[2].plot(t, i_arr, color=color, label=label, linewidth=1.0)

    axes[0].plot(t, sp, 'k--', label='设定值', linewidth=1.5)
    axes[0].set_ylabel('PV')
    axes[0].set_title('系统响应')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].set_ylabel('控制输出')
    axes[1].set_title('控制输出')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    axes[2].set_ylabel('积分项')
    axes[2].set_xlabel('时间 (s)')
    axes[2].set_title('积分项演变')
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'pid_rw_exp4_bidir.png'), dpi=150)
    plt.close('all')


# ============================================================
# 主函数
# ============================================================
if __name__ == '__main__':
    plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False

    print("=" * 60)
    print("积分重置抗饱和PID仿真")
    print("=" * 60)

    print("\n实验1: 三种抗饱和策略对比...")
    experiment_compare_strategies()

    print("实验2: 不同饱和程度下的恢复速度...")
    experiment_saturation_levels()

    print("实验3: 死区重置消除稳态振荡...")
    experiment_deadzone()

    print("实验4: 双向饱和场景...")
    experiment_bidirectional()

    print("\n仿真完成! 图片已保存到 simulation/ 目录。")
