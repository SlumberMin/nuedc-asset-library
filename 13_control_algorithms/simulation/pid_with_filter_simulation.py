#!/usr/bin/env python3
"""
PID+滤波器仿真 - 微分滤波与输出滤波效果对比

研究内容:
  1. 微分项低通滤波对噪声抑制的效果
  2. 输出低通滤波对控制信号平滑的作用
  3. 不同滤波系数(alpha)对系统动态响应的影响
  4. PV微分 vs 误差微分的对比
  5. 滤波器截止频率与响应速度的权衡
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from dataclasses import dataclass
from typing import List, Tuple

# ============================================================
# 被控对象: 二阶系统 (电机/弹簧-质量-阻尼)
# ============================================================
@dataclass
class SecondOrderPlant:
    """二阶传递函数: wn^2 / (s^2 + 2*zeta*wn*s + wn^2)"""
    wn: float = 20.0
    zeta: float = 0.3
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
# PID+滤波器控制器
# ============================================================
class PIDWithFilter:
    """PID控制器 + 微分低通滤波 + 输出低通滤波"""

    def __init__(self, kp, ki, kd, dt, out_min=-10.0, out_max=10.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.out_min = out_min
        self.out_max = out_max

        # 积分状态
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_pv = 0.0

        # 滤波状态
        self.prev_d_filtered = 0.0
        self.prev_out_filtered = 0.0

        # 滤波系数
        self.d_alpha = 0.2      # 微分滤波
        self.out_alpha = 0.3    # 输出滤波

        # 配置
        self.d_from_pv = False  # True=对PV微分, False=对误差微分
        self.enable_d_filter = True
        self.enable_out_filter = True

        self.first_run = True

    def compute(self, setpoint, pv):
        error = setpoint - pv

        # P
        p_term = self.kp * error

        # I (梯形积分)
        self.integral += 0.5 * (error + self.prev_error) * self.dt
        i_term = self.ki * self.integral

        # D
        if self.first_run:
            d_raw = 0.0
        elif self.d_from_pv:
            d_raw = self.kd * (-(pv - self.prev_pv)) / self.dt
        else:
            d_raw = self.kd * (error - self.prev_error) / self.dt

        # D项滤波
        if self.enable_d_filter:
            d_filtered = self.d_alpha * d_raw + (1 - self.d_alpha) * self.prev_d_filtered
        else:
            d_filtered = d_raw
        self.prev_d_filtered = d_filtered

        # 合成
        raw_output = p_term + i_term + d_filtered

        # 输出滤波
        if self.enable_out_filter:
            if self.first_run:
                out_filtered = raw_output
            else:
                out_filtered = self.out_alpha * raw_output + (1 - self.out_alpha) * self.prev_out_filtered
        else:
            out_filtered = raw_output
        self.prev_out_filtered = out_filtered

        # 限幅
        final_output = np.clip(out_filtered, self.out_min, self.out_max)

        self.prev_error = error
        self.prev_pv = pv
        self.first_run = False

        return final_output, p_term, d_raw, d_filtered

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_pv = 0.0
        self.prev_d_filtered = 0.0
        self.prev_out_filtered = 0.0
        self.first_run = True


# ============================================================
# 仿真运行器
# ============================================================
def run_simulation(pid, plant, setpoint_func, noise_std, n_steps):
    """运行仿真, 返回时间序列数据"""
    t = np.zeros(n_steps)
    sp = np.zeros(n_steps)
    pv = np.zeros(n_steps)
    u = np.zeros(n_steps)
    d_raw_arr = np.zeros(n_steps)
    d_filt_arr = np.zeros(n_steps)
    p_arr = np.zeros(n_steps)

    plant.reset()
    pid.reset()

    for i in range(n_steps):
        t[i] = i * pid.dt
        sp[i] = setpoint_func(t[i])
        clean_pv = plant.x1
        noisy_pv = clean_pv + np.random.normal(0, noise_std)
        pv[i] = noisy_pv

        output, p_term, d_raw, d_filt = pid.compute(sp[i], noisy_pv)
        u[i] = output
        d_raw_arr[i] = d_raw
        d_filt_arr[i] = d_filt
        p_arr[i] = p_term

        plant.update(output)

    return t, sp, pv, u, d_raw_arr, d_filt_arr, p_arr


# ============================================================
# 实验1: 微分滤波效果对比
# ============================================================
def experiment_d_filter():
    """对比不同微分滤波系数的效果"""
    dt = 0.001
    n_steps = 3000
    noise_std = 0.05

    def sp_func(t):
        return 1.0 if t > 0.2 else 0.0

    alphas = [1.0, 0.3, 0.1, 0.03]
    labels = ['无滤波(α=1.0)', '轻度(α=0.3)', '中度(α=0.1)', '重度(α=0.03)']

    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    fig.suptitle('实验1: 微分项低通滤波效果对比', fontsize=14, fontweight='bold')

    for alpha, label in zip(alphas, labels):
        plant = SecondOrderPlant(dt=dt)
        pid = PIDWithFilter(kp=3.0, ki=5.0, kd=0.5, dt=dt)
        pid.d_alpha = alpha
        pid.enable_d_filter = (alpha < 1.0)
        pid.enable_out_filter = False
        t, sp, pv, u, d_raw, d_filt, p_arr = run_simulation(pid, plant, sp_func, noise_std, n_steps)

        axes[0].plot(t, pv, label=label, linewidth=0.8)
        axes[1].plot(t, d_filt, label=label, linewidth=0.8)
        axes[2].plot(t, u, label=label, linewidth=0.8)

    axes[0].plot(t, sp, 'k--', label='设定值', linewidth=1.5)
    axes[0].set_ylabel('PV (过程变量)')
    axes[0].set_title('系统响应')
    axes[0].legend(fontsize=8)
    axes[0].grid(True, alpha=0.3)

    axes[1].set_ylabel('微分项值')
    axes[1].set_title('D项滤波效果 (噪声抑制)')
    axes[1].legend(fontsize=8)
    axes[1].grid(True, alpha=0.3)

    axes[2].set_ylabel('控制输出 u')
    axes[2].set_xlabel('时间 (s)')
    axes[2].set_title('控制输出')
    axes[2].legend(fontsize=8)
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'pid_wf_exp1_d_filter.png'), dpi=150)
    plt.close('all')


# ============================================================
# 实验2: 输出滤波效果对比
# ============================================================
def experiment_output_filter():
    """对比不同输出滤波系数的效果"""
    dt = 0.001
    n_steps = 3000
    noise_std = 0.03

    def sp_func(t):
        return 1.0 if t > 0.2 else 0.0

    alphas = [1.0, 0.5, 0.3, 0.1]
    labels = ['无滤波(α=1.0)', '轻度(α=0.5)', '中度(α=0.3)', '重度(α=0.1)']

    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    fig.suptitle('实验2: 输出低通滤波效果对比', fontsize=14, fontweight='bold')

    for alpha, label in zip(alphas, labels):
        plant = SecondOrderPlant(dt=dt)
        pid = PIDWithFilter(kp=3.0, ki=5.0, kd=0.5, dt=dt)
        pid.d_alpha = 0.3
        pid.enable_d_filter = True
        pid.out_alpha = alpha
        pid.enable_out_filter = (alpha < 1.0)
        t, sp, pv, u, d_raw, d_filt, p_arr = run_simulation(pid, plant, sp_func, noise_std, n_steps)

        axes[0].plot(t, u, label=label, linewidth=0.8)
        axes[1].plot(t, pv, label=label, linewidth=0.8)

    axes[0].plot(t, sp, 'k--', label='设定值', linewidth=1.5)
    axes[0].set_ylabel('控制输出 u')
    axes[0].set_title('控制输出平滑度')
    axes[0].legend(fontsize=8)
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t, sp, 'k--', label='设定值', linewidth=1.5)
    axes[1].set_ylabel('PV')
    axes[1].set_xlabel('时间 (s)')
    axes[1].set_title('系统响应 (滤波带来的延迟)')
    axes[1].legend(fontsize=8)
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'pid_wf_exp2_output_filter.png'), dpi=150)
    plt.close('all')


# ============================================================
# 实验3: PV微分 vs 误差微分
# ============================================================
def experiment_d_source():
    """对比对误差微分和对PV微分的效果"""
    dt = 0.001
    n_steps = 3000
    noise_std = 0.03

    def sp_func(t):
        return 1.0 if t > 0.2 else 0.0

    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    fig.suptitle('实验3: 误差微分 vs PV微分', fontsize=14, fontweight='bold')

    # 误差微分
    plant = SecondOrderPlant(dt=dt)
    pid = PIDWithFilter(kp=3.0, ki=5.0, kd=0.5, dt=dt)
    pid.d_from_pv = False
    pid.d_alpha = 0.2
    t, sp, pv1, u1, d_raw1, d_filt1, _ = run_simulation(pid, plant, sp_func, noise_std, n_steps)

    # PV微分
    plant = SecondOrderPlant(dt=dt)
    pid = PIDWithFilter(kp=3.0, ki=5.0, kd=0.5, dt=dt)
    pid.d_from_pv = True
    pid.d_alpha = 0.2
    t, sp, pv2, u2, d_raw2, d_filt2, _ = run_simulation(pid, plant, sp_func, noise_std, n_steps)

    axes[0].plot(t, sp, 'k--', label='设定值', linewidth=1.5)
    axes[0].plot(t, pv1, label='误差微分', linewidth=0.8)
    axes[0].plot(t, pv2, label='PV微分', linewidth=0.8)
    axes[0].set_ylabel('PV')
    axes[0].set_title('系统响应对比')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t, d_filt1, label='误差微分 D项', linewidth=0.8)
    axes[1].plot(t, d_filt2, label='PV微分 D项', linewidth=0.8)
    axes[1].set_ylabel('D项值')
    axes[1].set_title('设定值突变时D项冲击对比 (注意t=0.2s)')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(t, u1, label='误差微分', linewidth=0.8)
    axes[2].plot(t, u2, label='PV微分', linewidth=0.8)
    axes[2].set_ylabel('控制输出 u')
    axes[2].set_xlabel('时间 (s)')
    axes[2].set_title('控制输出对比')
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'pid_wf_exp3_d_source.png'), dpi=150)
    plt.close('all')


# ============================================================
# 实验4: 截止频率与响应速度权衡
# ============================================================
def experiment_bandwidth_tradeoff():
    """展示滤波强度与响应速度的权衡关系"""
    dt = 0.001

    # 计算alpha与截止频率的关系
    alphas = np.linspace(0.01, 1.0, 100)
    fc = -np.log(1 - alphas) / (2 * np.pi * dt)

    fig, ax = plt.subplots(1, 1, figsize=(10, 6))
    ax.plot(alphas, fc, 'b-', linewidth=2)
    ax.set_xlabel('滤波系数 α')
    ax.set_ylabel('截止频率 fc (Hz)')
    ax.set_title('一阶低通滤波器: 滤波系数 vs 截止频率 (dt=1ms)')
    ax.grid(True, alpha=0.3)

    # 标注常用值
    for a in [0.1, 0.2, 0.3, 0.5]:
        f = -np.log(1 - a) / (2 * np.pi * dt)
        ax.annotate(f'α={a}\nfc={f:.0f}Hz', xy=(a, f),
                    fontsize=9, ha='center', va='bottom',
                    arrowprops=dict(arrowstyle='->', color='red'),
                    color='red')

    # 右侧y轴显示时间常数
    ax2 = ax.twinx()
    tau = 1.0 / (2 * np.pi * fc)
    ax2.plot(alphas, tau * 1000, 'g--', linewidth=1, alpha=0.5)
    ax2.set_ylabel('时间常数 τ (ms)', color='green')
    ax2.tick_params(axis='y', labelcolor='green')

    plt.tight_layout()
    plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'pid_wf_exp4_bandwidth.png'), dpi=150)
    plt.close('all')


# ============================================================
# 主函数
# ============================================================
if __name__ == '__main__':
    plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False

    print("=" * 60)
    print("PID+滤波器仿真")
    print("=" * 60)
    print("\n实验1: 微分滤波效果对比...")
    experiment_d_filter()

    print("实验2: 输出滤波效果对比...")
    experiment_output_filter()

    print("实验3: PV微分 vs 误差微分...")
    experiment_d_source()

    print("实验4: 截止频率与响应速度权衡...")
    experiment_bandwidth_tradeoff()

    print("\n仿真完成! 图片已保存到 simulation/ 目录。")
