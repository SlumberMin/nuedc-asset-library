"""
pid_simulation.py - PID仿真程序

模拟不同系统特性(一阶惯性、二阶振荡、积分+惯性、纯延迟)
可视化PID响应曲线, 帮助理解参数作用

依赖: pip install matplotlib numpy
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from dataclasses import dataclass
from typing import Callable, Tuple


# ============================================================
# 被控对象模型
# ============================================================

class Plant:
    """被控对象基类"""
    def __init__(self):
        self.state = 0.0
        self.state2 = 0.0
    
    def update(self, u: float, dt: float) -> float:
        raise NotImplementedError
    
    def reset(self):
        self.state = 0.0
        self.state2 = 0.0


class FirstOrderPlant(Plant):
    """一阶惯性环节: G(s) = K / (Ts + 1)"""
    def __init__(self, K=1.0, T=1.0):
        super().__init__()
        self.K = K  # 增益
        self.T = T  # 时间常数
    
    def update(self, u, dt):
        self.state += dt / self.T * (self.K * u - self.state)
        return self.state


class SecondOrderPlant(Plant):
    """二阶振荡环节: G(s) = K*ωn² / (s² + 2ζωn*s + ωn²)"""
    def __init__(self, K=1.0, omega_n=5.0, zeta=0.3):
        super().__init__()
        self.K = K
        self.omega_n = omega_n
        self.zeta = zeta
    
    def update(self, u, dt):
        accel = self.omega_n**2 * (self.K * u - self.state) - 2 * self.zeta * self.omega_n * self.state2
        self.state2 += accel * dt
        self.state += self.state2 * dt
        return self.state


class IntegralPlant(Plant):
    """积分+惯性: G(s) = K / (s(Ts + 1))"""
    def __init__(self, K=1.0, T=1.0):
        super().__init__()
        self.K = K
        self.T = T
    
    def update(self, u, dt):
        self.state2 += dt / self.T * (self.K * u - self.state2)
        self.state += self.state2 * dt
        return self.state


class DelayPlant(Plant):
    """一阶惯性+纯延迟: G(s) = K*e^(-Ls) / (Ts + 1)"""
    def __init__(self, K=1.0, T=1.0, L=0.5, dt=0.01):
        super().__init__()
        self.K = K
        self.T = T
        self.delay_steps = int(L / dt)
        self.buffer = [0.0] * max(self.delay_steps, 1)
    
    def update(self, u, dt):
        self.state += dt / self.T * (self.K * u - self.state)
        self.buffer.append(self.state)
        return self.buffer.pop(0)


# ============================================================
# PID控制器
# ============================================================

@dataclass
class PIDController:
    kp: float = 10.0
    ki: float = 0.0
    kd: float = 0.0
    output_min: float = -1000.0
    output_max: float = 1000.0
    integral_max: float = 500.0
    
    def __post_init__(self):
        self.reset()
    
    def reset(self):
        self._error = 0.0
        self._error_last = 0.0
        self._integral = 0.0
        self._derivative = 0.0
    
    def update(self, target: float, measurement: float) -> float:
        error = target - measurement
        self._integral += error
        self._integral = max(-self.integral_max, min(self.integral_max, self._integral))
        derivative = error - self._error_last
        
        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        output = max(self.output_min, min(self.output_max, output))
        
        self._error_last = error
        return output


# ============================================================
# 仿真引擎
# ============================================================

def simulate(pid: PIDController, plant: Plant, 
             target: float, duration: float = 5.0, dt: float = 0.01,
             disturbance: Callable[[float], float] = None) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    运行仿真
    返回: (time, target_array, output_array, control_array)
    """
    steps = int(duration / dt)
    t = np.zeros(steps)
    y = np.zeros(steps)
    r = np.zeros(steps)
    u = np.zeros(steps)
    
    plant.reset()
    pid.reset()
    
    for i in range(steps):
        t[i] = i * dt
        r[i] = target
        
        measurement = y[i-1] if i > 0 else 0
        control = pid.update(target, measurement)
        
        # 添加扰动
        if disturbance:
            control += disturbance(t[i])
        
        y[i] = plant.update(control, dt)
        u[i] = control
    
    return t, r, y, u


def calculate_metrics(t, target, output) -> dict:
    """计算性能指标"""
    # 上升时间(10%→90%)
    rise_time = None
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
    
    return {
        'rise_time': rise_time,
        'overshoot': overshoot,
        'settling_time': settling_time,
        'steady_error': steady_error,
    }


# ============================================================
# 可视化
# ============================================================

def plot_response(t, target, output, control, title="PID Response", metrics=None):
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7))
    
    ax1.plot(t, target, 'r--', label='Target', linewidth=1.5)
    ax1.plot(t, output, 'b-', label='Output', linewidth=1)
    ax1.set_ylabel('Output')
    ax1.set_title(title)
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    if metrics:
        info = (f"Rise: {metrics['rise_time']:.3f}s | "
                f"Overshoot: {metrics['overshoot']:.1f}% | "
                f"Settling: {metrics['settling_time']:.3f}s | "
                f"Steady Err: {metrics['steady_error']:.3f}")
        ax1.text(0.02, 0.95, info, transform=ax1.transAxes, fontsize=9,
                verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    ax2.plot(t, control, 'g-', label='Control', linewidth=1)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Control')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('pid_response.png', dpi=150)
    plt.close('all')


# ============================================================
# 对比实验
# ============================================================

def compare_p_params():
    """对比不同Kp值的效果"""
    plant = SecondOrderPlant(K=1.0, omega_n=5.0, zeta=0.3)
    
    fig, axes = plt.subplots(2, 2, figsize=(12, 9))
    kp_values = [5, 10, 20, 50]
    
    for ax, kp in zip(axes.flat, kp_values):
        pid = PIDController(kp=kp, ki=0, kd=0)
        t, r, y, u = simulate(pid, plant, target=1.0, duration=3.0)
        metrics = calculate_metrics(t, r, y)
        
        ax.plot(t, r, 'r--', label='Target')
        ax.plot(t, y, 'b-', label='Output')
        ax.set_title(f'Kp={kp}')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.text(0.02, 0.95, f"Overshoot: {metrics['overshoot']:.1f}%\nSteady Err: {metrics['steady_error']:.3f}",
                transform=ax.transAxes, fontsize=9, verticalalignment='top')
    
    plt.suptitle('P Controller - Effect of Kp', fontsize=14)
    plt.tight_layout()
    plt.savefig('pid_compare_p.png', dpi=150)
    plt.close('all')


def compare_plants():
    """对比不同被控对象"""
    plants = {
        'First Order (K=1, T=0.5)': FirstOrderPlant(K=1.0, T=0.5),
        'Second Order (ζ=0.3)': SecondOrderPlant(K=1.0, omega_n=5.0, zeta=0.3),
        'Integral + Inertia': IntegralPlant(K=1.0, T=1.0),
        'Delay (L=0.3s)': DelayPlant(K=1.0, T=1.0, L=0.3, dt=0.01),
    }
    
    fig, axes = plt.subplots(2, 2, figsize=(12, 9))
    
    for ax, (name, plant) in zip(axes.flat, plants.items()):
        pid = PIDController(kp=10, ki=5, kd=2)
        t, r, y, u = simulate(pid, plant, target=1.0, duration=5.0)
        
        ax.plot(t, r, 'r--', label='Target')
        ax.plot(t, y, 'b-', label='Output')
        ax.set_title(name)
        ax.legend()
        ax.grid(True, alpha=0.3)
    
    plt.suptitle('PID on Different Plants', fontsize=14)
    plt.tight_layout()
    plt.savefig('pid_compare_plants.png', dpi=150)
    plt.close('all')


# ============================================================
# 主程序
# ============================================================

if __name__ == '__main__':
    print("=" * 60)
    print("  PID控制仿真程序")
    print("=" * 60)
    
    # 基本仿真
    plant = SecondOrderPlant(K=1.0, omega_n=5.0, zeta=0.3)
    pid = PIDController(kp=15, ki=8, kd=3)
    
    t, r, y, u = simulate(pid, plant, target=1.0, duration=3.0)
    metrics = calculate_metrics(t, r, y)
    
    print(f"\n性能指标:")
    print(f"  上升时间: {metrics['rise_time']:.3f} s")
    print(f"  超调量: {metrics['overshoot']:.1f} %")
    print(f"  调节时间: {metrics['settling_time']:.3f} s")
    print(f"  稳态误差: {metrics['steady_error']:.4f}")
    
    # 绘图
    plot_response(t, r, y, u, 
                  title="PID Response (Kp=15, Ki=8, Kd=3)",
                  metrics=metrics)
    
    # 对比实验
    compare_p_params()
    compare_plants()
