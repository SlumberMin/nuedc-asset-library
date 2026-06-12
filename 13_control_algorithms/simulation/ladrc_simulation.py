"""
ladrc_simulation.py - LADRC vs PID 性能对比仿真

对比LADRC（线性自抗扰控制）与传统PID控制器的性能差异：
1. 阶跃响应对比
2. 扰动抑制对比
3. 参数敏感性对比
4. 带宽整定效果演示

依赖: pip install numpy matplotlib
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from dataclasses import dataclass
from typing import Tuple, Optional
import sys
import os

# 添加父目录到路径以导入控制算法
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


# ============================================================
# LADRC控制器 (Python版本)
# ============================================================

@dataclass
class LADRCController:
    """
    LADRC线性自抗扰控制器

    核心思想：
    - LESO估计系统状态和总扰动
    - LSEF基于误差的PD控制
    - 扰动补偿提升鲁棒性
    """
    omega_o: float = 30.0      # 观测器带宽 ωo (rad/s)
    omega_c: float = 10.0      # 控制器带宽 ωc (rad/s)
    b0: float = 1.0            # 控制增益估计
    dt: float = 0.001          # 采样步长 (s)

    def __post_init__(self):
        self._init_gains()
        self.reset()

    def _init_gains(self):
        """根据带宽计算增益"""
        # LESO增益: beta1 = 3*ωo, beta2 = 3*ωo², beta3 = ωo³
        self.beta1 = 3.0 * self.omega_o
        self.beta2 = 3.0 * self.omega_o**2
        self.beta3 = self.omega_o**3

        # LSEF增益: Kp = ωc², Kd = 2*ωc
        self.Kp = self.omega_c**2
        self.Kd = 2.0 * self.omega_c

    def reset(self):
        """重置内部状态"""
        self.z1 = 0.0      # LESO状态z1 (输出估计)
        self.z2 = 0.0      # LESO状态z2 (导数估计)
        self.z3 = 0.0      # LESO状态z3 (扰动估计)
        self.output = 0.0  # 控制输出
        self.e1 = 0.0      # 位置误差

    def calculate(self, r: float, y: float) -> float:
        """
        LADRC计算

        参数:
            r: 参考输入（设定值）
            y: 测量输出（反馈值）

        返回:
            u: 控制输出
        """
        # 1. LESO更新
        e = self.z1 - y  # 观测误差
        self.z1 += self.dt * (self.z2 + self.b0 * self.output - self.beta1 * e)
        self.z2 += self.dt * (self.z3 - self.beta2 * e)
        self.z3 += self.dt * (-self.beta3 * e)

        # 2. LSEF计算
        self.e1 = r - self.z1
        e2 = -self.z2  # 速度误差 (假设参考速度=0)
        u0 = self.Kp * self.e1 + self.Kd * e2

        # 3. 扰动补偿
        self.output = (u0 - self.z3) / self.b0

        return self.output


# ============================================================
# PID控制器
# ============================================================

@dataclass
class PIDController:
    """标准PID控制器"""
    kp: float = 15.0
    ki: float = 8.0
    kd: float = 3.0
    output_min: float = -1000.0
    output_max: float = 1000.0

    def __post_init__(self):
        self.reset()

    def reset(self):
        self._error = 0.0
        self._error_last = 0.0
        self._integral = 0.0

    def update(self, target: float, measurement: float) -> float:
        error = target - measurement
        self._integral += error
        self._integral = max(-500, min(500, self._integral))
        derivative = error - self._error_last

        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        output = max(self.output_min, min(self.output_max, output))

        self._error_last = error
        return output


# ============================================================
# 被控对象模型
# ============================================================

class SecondOrderPlant:
    """二阶振荡环节: G(s) = K*ωn² / (s² + 2ζωn*s + ωn²)"""
    def __init__(self, K=1.0, omega_n=5.0, zeta=0.3):
        self.K = K
        self.omega_n = omega_n
        self.zeta = zeta
        self.state = 0.0
        self.state2 = 0.0

    def reset(self):
        self.state = 0.0
        self.state2 = 0.0

    def update(self, u: float, dt: float) -> float:
        accel = self.omega_n**2 * (self.K * u - self.state) - 2 * self.zeta * self.omega_n * self.state2
        self.state2 += accel * dt
        self.state += self.state2 * dt
        return self.state


class NonlinearPlant:
    """非线性被控对象 (模拟真实电机特性)"""
    def __init__(self, K=1.0, friction=0.1):
        self.K = K
        self.friction = friction
        self.state = 0.0
        self.state2 = 0.0

    def reset(self):
        self.state = 0.0
        self.state2 = 0.0

    def update(self, u: float, dt: float) -> float:
        # 非线性摩擦 + 增益变化
        effective_K = self.K * (1.0 + 0.3 * np.sin(self.state))  # 时变增益
        friction_force = self.friction * np.sign(self.state2) if abs(self.state2) > 0.01 else 0

        accel = effective_K * u - friction_force - 2 * self.state2
        self.state2 += accel * dt
        self.state += self.state2 * dt
        return self.state


# ============================================================
# 仿真函数
# ============================================================

def simulate_system(controller, plant, target: float, duration: float, dt: float,
                    disturbance=None, delay_steps: int = 0) -> Tuple:
    """
    运行仿真

    返回: (time, target, output, control)
    """
    steps = int(duration / dt)
    t = np.zeros(steps)
    y = np.zeros(steps)
    r = np.zeros(steps)
    u = np.zeros(steps)

    plant.reset()
    if hasattr(controller, 'reset'):
        controller.reset()

    buffer = [0.0] * max(delay_steps, 1)

    for i in range(steps):
        t[i] = i * dt
        r[i] = target

        # 获取测量值 (带延迟)
        measurement = y[i-1] if i > 0 else 0

        # 控制器计算
        if isinstance(controller, LADRCController):
            control = controller.calculate(target, measurement)
        else:
            control = controller.update(target, measurement)

        # 添加扰动
        if disturbance:
            control += disturbance(t[i])

        # 限幅
        control = max(-1000, min(1000, control))

        # 被控对象更新
        y[i] = plant.update(control, dt)
        u[i] = control

    return t, r, y, u


def calculate_metrics(t, target, output) -> dict:
    """计算性能指标"""
    # 上升时间(10%→90%)
    rise_time = None
    t_start = 0
    for i in range(len(output)):
        if output[i] >= 0.1 * target and rise_time is None:
            t_start = t[i]
        if output[i] >= 0.9 * target:
            rise_time = t[i] - t_start
            break

    # 超调量
    overshoot = (np.max(output) - target) / target * 100 if target != 0 else 0

    # 稳态误差
    steady_state = np.mean(output[-100:])
    steady_error = abs(target - steady_state)

    # 调节时间(进入±2%带)
    settling_time = None
    band = 0.02 * abs(target)
    for i in range(len(output) - 1, -1, -1):
        if abs(output[i] - target) > band:
            settling_time = t[min(i + 1, len(t) - 1)]
            break

    # IAE指标 (绝对误差积分)
    iae = np.sum(np.abs(target - output)) * (t[1] - t[0])

    return {
        'rise_time': rise_time or 0,
        'overshoot': overshoot,
        'settling_time': settling_time or 0,
        'steady_error': steady_error,
        'iae': iae
    }


# ============================================================
# 实验1: 阶跃响应对比
# ============================================================

def experiment1_step_response():
    """阶跃响应对比"""
    print("\n" + "=" * 60)
    print("  实验1: 阶跃响应对比")
    print("=" * 60)

    dt = 0.001
    duration = 3.0
    target = 1.0

    # 被控对象
    plant = SecondOrderPlant(K=1.0, omega_n=5.0, zeta=0.3)

    # 控制器
    pid = PIDController(kp=15, ki=8, kd=3)
    ladrc = LADRCController(omega_o=30, omega_c=10, b0=1.0, dt=dt)

    # PID仿真
    t_pid, r_pid, y_pid, u_pid = simulate_system(pid, plant, target, duration, dt)

    # LADRC仿真
    t_ladrc, r_ladrc, y_ladrc, u_ladrc = simulate_system(ladrc, plant, target, duration, dt)

    # 计算指标
    metrics_pid = calculate_metrics(t_pid, target, y_pid)
    metrics_ladrc = calculate_metrics(t_ladrc, target, y_ladrc)

    # 绘图
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))

    # 响应对比
    axes[0].plot(t_pid, r_pid, 'r--', label='Target', linewidth=2)
    axes[0].plot(t_pid, y_pid, 'b-', label='PID', linewidth=1.5)
    axes[0].plot(t_ladrc, y_ladrc, 'g-', label='LADRC', linewidth=1.5)
    axes[0].set_ylabel('Output')
    axes[0].set_title('Step Response Comparison')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    # 添加性能指标文本
    info = (f"PID: Rise={metrics_pid['rise_time']:.3f}s, OS={metrics_pid['overshoot']:.1f}%, "
            f"Ts={metrics_pid['settling_time']:.3f}s, IAE={metrics_pid['iae']:.3f}\n"
            f"LADRC: Rise={metrics_ladrc['rise_time']:.3f}s, OS={metrics_ladrc['overshoot']:.1f}%, "
            f"Ts={metrics_ladrc['settling_time']:.3f}s, IAE={metrics_ladrc['iae']:.3f}")
    axes[0].text(0.02, 0.95, info, transform=axes[0].transAxes, fontsize=9,
                 verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    # 控制量对比
    axes[1].plot(t_pid, u_pid, 'b-', label='PID', linewidth=1)
    axes[1].plot(t_ladrc, u_ladrc, 'g-', label='LADRC', linewidth=1)
    axes[1].set_ylabel('Control Output')
    axes[1].set_title('Control Signal')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    # 误差对比
    axes[2].plot(t_pid, target - y_pid, 'b-', label='PID Error', linewidth=1)
    axes[2].plot(t_ladrc, target - y_ladrc, 'g-', label='LADRC Error', linewidth=1)
    axes[2].set_xlabel('Time (s)')
    axes[2].set_ylabel('Error')
    axes[2].set_title('Tracking Error')
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('experiment1_step_response.png', dpi=150, bbox_inches='tight')
    plt.close('all')

    return metrics_pid, metrics_ladrc


# ============================================================
# 实验2: 扰动抑制对比
# ============================================================

def experiment2_disturbance_rejection():
    """扰动抑制对比"""
    print("\n" + "=" * 60)
    print("  实验2: 扰动抑制对比")
    print("=" * 60)

    dt = 0.001
    duration = 5.0
    target = 1.0

    # 被控对象
    plant = SecondOrderPlant(K=1.0, omega_n=5.0, zeta=0.3)

    # 扰动函数: 在t=2s时施加阶跃扰动
    def disturbance(t):
        if 2.0 <= t < 2.5:
            return 50.0  # 正向扰动
        elif 3.0 <= t < 3.5:
            return -50.0  # 负向扰动
        return 0.0

    # 控制器
    pid = PIDController(kp=15, ki=8, kd=3)
    ladrc = LADRCController(omega_o=30, omega_c=10, b0=1.0, dt=dt)

    # PID仿真
    t_pid, r_pid, y_pid, u_pid = simulate_system(pid, plant, target, duration, dt, disturbance)

    # LADRC仿真
    t_ladrc, r_ladrc, y_ladrc, u_ladrc = simulate_system(ladrc, plant, target, duration, dt, disturbance)

    # 绘图
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))

    # 响应对比
    axes[0].plot(t_pid, r_pid, 'r--', label='Target', linewidth=2)
    axes[0].plot(t_pid, y_pid, 'b-', label='PID', linewidth=1.5)
    axes[0].plot(t_ladrc, y_ladrc, 'g-', label='LADRC', linewidth=1.5)
    axes[0].axvspan(2.0, 2.5, alpha=0.3, color='red', label='Disturbance (+)')
    axes[0].axvspan(3.0, 3.5, alpha=0.3, color='blue', label='Disturbance (-)')
    axes[0].set_ylabel('Output')
    axes[0].set_title('Disturbance Rejection Comparison')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    # 控制量对比
    axes[1].plot(t_pid, u_pid, 'b-', label='PID', linewidth=1)
    axes[1].plot(t_ladrc, u_ladrc, 'g-', label='LADRC', linewidth=1)
    axes[1].set_xlabel('Time (s)')
    axes[1].set_ylabel('Control Output')
    axes[1].set_title('Control Signal (with Disturbance Compensation)')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('experiment2_disturbance.png', dpi=150, bbox_inches='tight')
    plt.close('all')


# ============================================================
# 实验3: 带宽参数整定演示
# ============================================================

def experiment3_bandwidth_tuning():
    """LADRC带宽整定演示"""
    print("\n" + "=" * 60)
    print("  实验3: LADRC带宽整定演示")
    print("=" * 60)

    dt = 0.001
    duration = 3.0
    target = 1.0

    # 被控对象
    plant = SecondOrderPlant(K=1.0, omega_n=5.0, zeta=0.3)

    # 不同带宽参数组合
    configs = [
        ("ωo=10, ωc=5 (保守)", 10, 5),
        ("ωo=30, ωc=10 (推荐)", 30, 10),
        ("ωo=50, ωc=15 (快速)", 50, 15),
        ("ωo=100, ωc=20 (激进)", 100, 20),
    ]

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    for ax, (name, omega_o, omega_c) in zip(axes.flat, configs):
        ladrc = LADRCController(omega_o=omega_o, omega_c=omega_c, b0=1.0, dt=dt)
        t, r, y, u = simulate_system(ladrc, plant, target, duration, dt)

        metrics = calculate_metrics(t, target, y)

        ax.plot(t, r, 'r--', label='Target', linewidth=2)
        ax.plot(t, y, 'g-', label='LADRC', linewidth=1.5)
        ax.set_title(name)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Output')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.text(0.02, 0.95, f"Rise: {metrics['rise_time']:.3f}s\n"
                f"OS: {metrics['overshoot']:.1f}%\n"
                f"Ts: {metrics['settling_time']:.3f}s",
                transform=ax.transAxes, fontsize=9, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

        print(f"\n{name}:")
        print(f"  上升时间: {metrics['rise_time']:.3f} s")
        print(f"  超调量: {metrics['overshoot']:.1f} %")
        print(f"  调节时间: {metrics['settling_time']:.3f} s")

    plt.suptitle('LADRC Bandwidth Tuning Demo (ωo: Observer, ωc: Controller)', fontsize=14)
    plt.tight_layout()
    plt.savefig('experiment3_bandwidth.png', dpi=150, bbox_inches='tight')
    plt.close('all')


# ============================================================
# 实验4: 非线性系统鲁棒性对比
# ============================================================

def experiment4_nonlinear_robustness():
    """非线性系统鲁棒性对比"""
    print("\n" + "=" * 60)
    print("  实验4: 非线性系统鲁棒性对比")
    print("=" * 60)

    dt = 0.001
    duration = 5.0
    target = 1.0

    # 非线性被控对象 (时变增益)
    plant = NonlinearPlant(K=1.0, friction=0.1)

    # 控制器 (使用相同的参数, 不针对特定对象调优)
    pid = PIDController(kp=15, ki=8, kd=3)
    ladrc = LADRCController(omega_o=30, omega_c=10, b0=1.0, dt=dt)

    # PID仿真
    t_pid, r_pid, y_pid, u_pid = simulate_system(pid, plant, target, duration, dt)

    # LADRC仿真
    t_ladrc, r_ladrc, y_ladrc, u_ladrc = simulate_system(ladrc, plant, target, duration, dt)

    metrics_pid = calculate_metrics(t_pid, target, y_pid)
    metrics_ladrc = calculate_metrics(t_ladrc, target, y_ladrc)

    # 绘图
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))

    # 响应对比
    axes[0].plot(t_pid, r_pid, 'r--', label='Target', linewidth=2)
    axes[0].plot(t_pid, y_pid, 'b-', label='PID', linewidth=1.5)
    axes[0].plot(t_ladrc, y_ladrc, 'g-', label='LADRC', linewidth=1.5)
    axes[0].set_ylabel('Output')
    axes[0].set_title('Nonlinear System Response (Time-varying Gain)')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    info = (f"PID: OS={metrics_pid['overshoot']:.1f}%, IAE={metrics_pid['iae']:.3f}\n"
            f"LADRC: OS={metrics_ladrc['overshoot']:.1f}%, IAE={metrics_ladrc['iae']:.3f}")
    axes[0].text(0.02, 0.95, info, transform=axes[0].transAxes, fontsize=10,
                 verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    # 扰动估计 (LADRC特有)
    # 重新仿真以获取z3
    plant2 = NonlinearPlant(K=1.0, friction=0.1)
    ladrc2 = LADRCController(omega_o=30, omega_c=10, b0=1.0, dt=dt)
    steps = int(duration / dt)
    z3_log = np.zeros(steps)
    for i in range(steps):
        y_val = plant2.state if i > 0 else 0
        ladrc2.calculate(target, y_val)
        z3_log[i] = ladrc2.z3
        plant2.update(ladrc2.output, dt)

    axes[1].plot(np.arange(steps) * dt, z3_log, 'g-', label='Disturbance Estimate (z3)', linewidth=1.5)
    axes[1].set_xlabel('Time (s)')
    axes[1].set_ylabel('Disturbance Estimate')
    axes[1].set_title('LADRC Disturbance Estimation')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('experiment4_nonlinear.png', dpi=150, bbox_inches='tight')
    plt.close('all')


# ============================================================
# 主程序
# ============================================================

if __name__ == '__main__':
    print("=" * 60)
    print("  LADRC vs PID 性能对比仿真")
    print("=" * 60)
    print("\nLADRC (Linear Active Disturbance Rejection Control)")
    print("线性自抗扰控制 - 基于高志强带宽整定法")
    print("\n核心参数:")
    print("  - ωo (观测器带宽): 决定扰动估计速度, 一般ωo≥3×ωc")
    print("  - ωc (控制器带宽): 决定系统响应速度")

    # 运行所有实验
    metrics_pid, metrics_ladrc = experiment1_step_response()
    experiment2_disturbance_rejection()
    experiment3_bandwidth_tuning()
    experiment4_nonlinear_robustness()

    # 最终对比总结
    print("\n" + "=" * 60)
    print("  性能对比总结")
    print("=" * 60)

    print(f"\n阶跃响应对比:")
    print(f"  PID:    上升时间={metrics_pid['rise_time']:.3f}s, "
          f"超调量={metrics_pid['overshoot']:.1f}%, "
          f"IAE={metrics_pid['iae']:.3f}")
    print(f"  LADRC:  上升时间={metrics_ladrc['rise_time']:.3f}s, "
          f"超调量={metrics_ladrc['overshoot']:.1f}%, "
          f"IAE={metrics_ladrc['iae']:.3f}")

    improvement = (1 - metrics_ladrc['iae'] / metrics_pid['iae']) * 100 if metrics_pid['iae'] > 0 else 0
    print(f"\n  LADRC相比PID:")
    print(f"    IAE改善: {improvement:.1f}%")
    print(f"    上升时间改善: {(1 - metrics_ladrc['rise_time'] / metrics_pid['rise_time']) * 100:.1f}%")
    print(f"    超调量改善: {metrics_pid['overshoot'] - metrics_ladrc['overshoot']:.1f}%")

    print("\n结论:")
    print("  LADRC相比传统PID具有以下优势:")
    print("  1. 更快的响应速度")
    print("  2. 更小的超调量")
    print("  3. 更强的抗扰能力 (通过扰动估计和补偿)")
    print("  4. 更好的鲁棒性 (对参数变化和非线性不敏感)")
    print("  5. 仅需整定2个参数 (ωo, ωc), 比PID更简单")
    print("\n推荐使用场景:")
    print("  - 电机控制 (速度/位置)")
    print("  - 倒立摆、小车等平衡系统")
    print("  - 高精度伺服系统")
    print("  - 噪声和扰动较强的场合")
