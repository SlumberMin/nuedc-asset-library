#!/usr/bin/env python3
"""
阻抗控制仿真 - 机器人接触力控制
===============================
模拟机器人末端执行器与环境接触时的阻抗控制策略。
支持质量-弹簧-阻尼模型，可调参数：M(惯量), B(阻尼), K(刚度)。

运行: python impedance_control_simulation.py
依赖: numpy, matplotlib
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

# ========== 阻抗控制器 ==========
class ImpedanceController:
    """阻抗控制器: F = M*x_ddot_des + B*(x_dot_des - x_dot) + K*(x_des - x)"""

    def __init__(self, M=1.0, B=20.0, K=100.0):
        self.M = M   # 期望惯量 (kg)
        self.B = B   # 期望阻尼 (N·s/m)
        self.K = K   # 期望刚度 (N/m)

    def compute(self, x_des, x_dot_des, x_ddot_des, x, x_dot, F_ext=0.0):
        """计算阻抗控制力矩/力"""
        # 阻抗关系: M*x_ddot + B*(x_dot - x_dot_des) + K*(x - x_des) = -F_ext
        # 解出控制输入: F_ctrl = M*x_ddot_des + B*(x_dot_des - x_dot) + K*(x_des - x) - F_ext
        F_ctrl = (self.M * x_ddot_des
                  + self.B * (x_dot_des - x_dot)
                  + self.K * (x_des - x)
                  - F_ext)
        return F_ctrl


# ========== 环境模型 ==========
class CompliantEnvironment:
    """柔性环境模型: 线性弹簧+阻尼"""

    def __init__(self, K_e=5000.0, B_e=50.0, x_surface=0.5):
        self.K_e = K_e         # 环境刚度
        self.B_e = B_e         # 环境阻尼
        self.x_surface = x_surface  # 环境表面位置

    def contact_force(self, x, x_dot):
        """计算接触力 (仅在接触时产生)"""
        penetration = self.x_surface - x
        if penetration > 0:  # 发生接触
            F = self.K_e * penetration + self.B_e * max(0, -x_dot)
            return F
        return 0.0


# ========== 单关节机器人臂 ==========
class SingleJointArm:
    """简化的单关节机械臂动力学模型"""

    def __init__(self, m=2.0, l=0.5, damping=1.0):
        self.m = m           # 质量 (kg)
        self.l = l           # 臂长 (m)
        self.damping = damping  # 关节阻尼
        self.I = m * l**2 / 3  # 转动惯量
        self.x = 0.3          # 初始位置
        self.x_dot = 0.0      # 初始速度
        self.g = 9.81

    def step(self, F_ctrl, F_ext, dt):
        """更新状态 (简化为1D直线运动)"""
        # m * x_ddot = F_ctrl + F_ext - damping * x_dot
        x_ddot = (F_ctrl + F_ext - self.damping * self.x_dot) / self.m
        self.x_dot += x_ddot * dt
        self.x += self.x_dot * dt
        return self.x, self.x_dot


# ========== 仿真主循环 ==========
def run_simulation(controller_params, x_target=0.5, duration=2.0, dt=0.0001):
    """运行一次阻抗控制仿真"""
    controller = ImpedanceController(**controller_params)
    env = CompliantEnvironment(x_surface=0.5)
    arm = SingleJointArm(m=2.0, l=0.5, damping=1.0)

    # 重置
    arm.x, arm.x_dot = 0.3, 0.0

    steps = int(duration / dt)
    results = {
        'time': np.zeros(steps),
        'position': np.zeros(steps),
        'velocity': np.zeros(steps),
        'force': np.zeros(steps),
        'force_ext': np.zeros(steps),
        'target': np.full(steps, x_target),
    }

    for i in range(steps):
        t = i * dt
        F_ext = env.contact_force(arm.x, arm.x_dot)
        F_ctrl = controller.compute(x_target, 0, 0, arm.x, arm.x_dot, F_ext)
        arm.step(F_ctrl, F_ext, dt)

        results['time'][i] = t
        results['position'][i] = arm.x
        results['velocity'][i] = arm.x_dot
        results['force'][i] = F_ctrl
        results['force_ext'][i] = F_ext

    return results


# ========== 可视化 ==========
def plot_results(all_results):
    """绘制多组参数对比图"""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('阻抗控制仿真 - 机器人接触力控制', fontsize=16, fontweight='bold')

    colors = ['#2196F3', '#F44336', '#4CAF50', '#FF9800']

    for idx, (label, res) in enumerate(all_results.items()):
        c = colors[idx % len(colors)]

        # 位置
        axes[0, 0].plot(res['time'], res['position'], color=c, label=label, linewidth=1.5)
        axes[0, 0].set_title('末端位置')
        axes[0, 0].set_ylabel('位置 (m)')

        # 速度
        axes[0, 1].plot(res['time'], res['velocity'], color=c, label=label, linewidth=1.5)
        axes[0, 1].set_title('末端速度')
        axes[0, 1].set_ylabel('速度 (m/s)')

        # 接触力
        axes[1, 0].plot(res['time'], res['force_ext'], color=c, label=label, linewidth=1.5)
        axes[1, 0].set_title('环境接触力')
        axes[1, 0].set_ylabel('力 (N)')

        # 控制力
        axes[1, 1].plot(res['time'], res['force'], color=c, label=label, linewidth=1.5)
        axes[1, 1].set_title('控制力')
        axes[1, 1].set_ylabel('力 (N)')

    for ax in axes.flat:
        ax.set_xlabel('时间 (s)')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    # 目标位置虚线
    axes[0, 0].axhline(y=0.5, color='k', linestyle='--', alpha=0.5, label='目标/环境表面')

    plt.tight_layout()
    plt.savefig('impedance_control_results.png', dpi=150, bbox_inches='tight')
    plt.close('all')
    print("结果已保存: impedance_control_results.png")


def plot_phase_portrait(all_results):
    """绘制力-位移相图"""
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    fig.suptitle('阻抗控制 - 相图分析', fontsize=14, fontweight='bold')

    colors = ['#2196F3', '#F44336', '#4CAF50', '#FF9800']

    for idx, (label, res) in enumerate(all_results.items()):
        c = colors[idx % len(colors)]
        axes[0].plot(res['position'], res['velocity'], color=c, label=label, linewidth=1)
        axes[1].plot(res['position'], res['force_ext'], color=c, label=label, linewidth=1)

    axes[0].set_xlabel('位置 (m)'); axes[0].set_ylabel('速度 (m/s)')
    axes[0].set_title('位置-速度相图'); axes[0].legend(fontsize=8); axes[0].grid(True, alpha=0.3)

    axes[1].set_xlabel('位置 (m)'); axes[1].set_ylabel('接触力 (N)')
    axes[1].set_title('位置-力相图'); axes[1].legend(fontsize=8); axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('impedance_phase_portrait.png', dpi=150, bbox_inches='tight')
    plt.close('all')


# ========== 主程序 ==========
if __name__ == '__main__':
    print("=" * 60)
    print("阻抗控制仿真 - 机器人接触力控制")
    print("=" * 60)

    configs = {
        '低刚度 (软接触)': {'M': 1.0, 'B': 30.0, 'K': 50.0},
        '中等刚度 (平衡)': {'M': 1.0, 'B': 40.0, 'K': 200.0},
        '高刚度 (硬接触)': {'M': 1.0, 'B': 60.0, 'K': 800.0},
        '高阻尼':          {'M': 1.0, 'B': 100.0, 'K': 200.0},
    }

    all_results = {}
    for name, params in configs.items():
        print(f"  仿真: {name} (K={params['K']}, B={params['B']})")
        all_results[name] = run_simulation(params, duration=2.0)

    # 性能指标
    print("\n性能指标:")
    print(f"{'配置':<20} {'最大接触力(N)':<15} {'稳态误差(mm)':<15} {'调节时间(s)':<12}")
    print("-" * 65)
    for name, res in all_results.items():
        max_force = np.max(res['force_ext'])
        # 取后10%数据算稳态
        ss_idx = int(0.9 * len(res['position']))
        ss_error = np.mean(np.abs(res['position'][ss_idx:] - 0.5)) * 1000
        # 调节时间: 误差进入±2%的时间
        target = 0.5
        tol = 0.02 * target
        settle_t = 2.0
        for i in range(len(res['position'])):
            if np.all(np.abs(res['position'][i:] - target) < tol):
                settle_t = res['time'][i]
                break
        print(f"{name:<20} {max_force:<15.1f} {ss_error:<15.2f} {settle_t:<12.3f}")

    plot_results(all_results)
    plot_phase_portrait(all_results)

    print("\n仿真完成！")
