#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
磁悬浮系统仿真 - PID控制稳定性验证
====================================
数学模型:
  m*d²h/dt² = m*g - k*I²/(h²)
  线性化在平衡点(h0, I0):
    Δh'' = (2k*I0²/(m*h0³))*Δh - (2k*I0/(m*h0²))*ΔI

控制策略:
  1. 传统PID
  2. 串级PID (位置环+电流环)
  3. 前馈+PID

仿真内容:
  - 非线性模型仿真
  - 三种控制策略对比
  - 参数扫描与最优参数搜索
  - 位置跟踪性能
  - 扰动注入与鲁棒性测试
"""

import os
import numpy as np

# numpy兼容：np.trapz在1.x废弃，2.x移除，统一用np.trapezoid
if hasattr(np, 'trapezoid'):
    _trapz = np.trapezoid
else:
    _trapz = np.trapezoid

from scipy.integrate import odeint
from scipy.optimize import differential_evolution
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib
import warnings
warnings.filterwarnings('ignore')

matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False

# ==============================================================================
# 磁悬浮模型参数
# ==============================================================================
class MagneticLevitationParams:
    """磁悬浮物理参数"""
    m = 0.02        # 钢球质量 [kg]
    k = 7.848e-5      # 电磁力常数 [N.m^2/A^2] (满足m*g=k*I0^2/h0^2平衡)
    h0 = 0.02       # 平衡位置 [m]
    I0 = 1.0        # 平衡电流 [A]
    g = 9.81        # 重力加速度 [m/s^2]
    R = 1.0          # 线圈电阻 [Ohm]
    L = 0.01         # 线圈电感 [H]

    @classmethod
    def linearized_ss(cls):
        """线性化状态空间: x = [Δh, Δh', ΔI]"""
        p = cls
        # 平衡点: m*g = k*I0^2/h0^2 => 验证
        assert abs(p.m * p.g - p.k * p.I0**2 / p.h0**2) < 0.01, "平衡点不满足!"

        # 线性化系数
        a = 2 * p.k * p.I0**2 / (p.m * p.h0**3)  # Δh'' = a*Δh + b*ΔI
        b = -2 * p.k * p.I0 / (p.m * p.h0**2)     # 注意负号

        # 状态: [Δh, Δh', ΔI]
        # dΔh/dt = Δh'
        # dΔh'/dt = a*Δh + b*ΔI
        # dΔI/dt = -R/L*ΔI + 1/L*ΔV (电压控制)
        A = np.array([
            [0, 1, 0],
            [a, 0, b],
            [0, 0, -p.R / p.L]
        ])
        B = np.array([
            [0],
            [0],
            [1.0 / p.L]
        ])
        C = np.array([[1, 0, 0]])  # 输出位置
        return A, B, C


# ==============================================================================
# 控制器
# ==============================================================================
class CascadePIDController:
    """串级PID控制器 (位置环 + 电流环)"""
    def __init__(self, Kp_pos, Ki_pos, Kd_pos, Kp_cur, Ki_cur, dt=0.001):
        self.Kp_pos = Kp_pos
        self.Ki_pos = Ki_pos
        self.Kd_pos = Kd_pos
        self.Kp_cur = Kp_cur
        self.Ki_cur = Ki_cur
        self.dt = dt

        self.int_pos = 0.0
        self.prev_pos_err = 0.0
        self.int_cur = 0.0
        self.prev_cur_err = 0.0

    def reset(self):
        self.int_pos = 0.0
        self.prev_pos_err = 0.0
        self.int_cur = 0.0
        self.prev_cur_err = 0.0

    def compute(self, setpoint, h, I):
        """计算电压输出"""
        dt = self.dt

        # 位置环
        pos_err = setpoint - h
        self.int_pos += pos_err * dt
        d_pos = (pos_err - self.prev_pos_err) / dt
        self.prev_pos_err = pos_err

        current_ref = self.Kp_pos * pos_err + self.Ki_pos * self.int_pos + self.Kd_pos * d_pos
        current_ref = np.clip(current_ref, 0, 5)  # 电流限幅

        # 电流环
        cur_err = current_ref - I
        self.int_cur += cur_err * dt
        d_cur = (cur_err - self.prev_cur_err) / dt
        self.prev_cur_err = cur_err

        voltage = self.Kp_cur * cur_err + self.Ki_cur * self.int_cur
        return np.clip(voltage, -5, 5)


class FeedforwardPIDController:
    """前馈+PID控制器"""
    def __init__(self, Kp, Ki, Kd, Kff, dt=0.001):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.Kff = Kff
        self.dt = dt
        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, setpoint, h, I, h_dot=0.0):
        error = setpoint - h
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error

        # PID输出
        pid_out = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        # 前馈: 基于平衡电流的前馈
        p = MagneticLevitationParams
        I_ff = np.sqrt(p.m * p.g * setpoint**2 / p.k) if setpoint > 0 else p.I0
        ff_out = self.Kff * (I_ff - p.I0)

        return np.clip(pid_out + ff_out, -5, 5)


class SimplePIDController:
    """简单PID控制器"""
    def __init__(self, Kp, Ki, Kd, dt=0.001):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, setpoint, h, I=0.0):
        error = setpoint - h
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        return np.clip(output, -5, 5)


# ==============================================================================
# 性能指标
# ==============================================================================
class PerformanceMetrics:
    @staticmethod
    def rise_time(t, y, setpoint):
        y10 = 0.1 * setpoint
        y90 = 0.9 * setpoint
        if setpoint < 0:
            y10, y90 = y90, y10
        t10 = t[np.argmax(y >= y10)] if setpoint > 0 else t[np.argmax(y <= y10)]
        t90 = t[np.argmax(y >= y90)] if setpoint > 0 else t[np.argmax(y <= y90)]
        return abs(t90 - t10)

    @staticmethod
    def overshoot(y, setpoint):
        if setpoint == 0:
            return np.max(np.abs(y))
        return max((np.max(y) - setpoint) / setpoint * 100, 0)

    @staticmethod
    def settling_time(t, y, setpoint, tolerance=0.02):
        band = tolerance * abs(setpoint) if setpoint != 0 else tolerance * np.max(np.abs(y))
        out_of_band = np.where(np.abs(y - setpoint) > band)[0]
        if len(out_of_band) == 0:
            return 0.0
        return t[out_of_band[-1]]

    @staticmethod
    def iae(t, e):
        return _trapz(np.abs(e), t)

    @staticmethod
    def itae(t, e):
        return _trapz(t * np.abs(e), t)

    @staticmethod
    def compute_all(t, y, setpoint):
        e = y - setpoint
        return {
            'rise_time': PerformanceMetrics.rise_time(t, y, setpoint),
            'overshoot': PerformanceMetrics.overshoot(y, setpoint),
            'settling_time': PerformanceMetrics.settling_time(t, y, setpoint),
            'iae': PerformanceMetrics.iae(t, e),
            'itae': PerformanceMetrics.itae(t, e),
        }


# ==============================================================================
# 仿真引擎
# ==============================================================================
class MagneticLevitationSimulation:
    """磁悬浮仿真引擎"""
    def __init__(self, params=None):
        self.params = params or MagneticLevitationParams()
        self.dt = 0.001
        self.T = 2.0
        self.setpoint = 0.025  # 目标悬浮高度 (比平衡点高5mm)

    def nonlinear_model(self, x, t, V, params=None):
        """非线性动力学模型: x = [h, h', I]"""
        p = params or self.params
        h, h_dot, I = x

        # 保证物理约束
        h = max(h, 0.001)  # 避免除零

        # 力学方程
        F_mag = p.k * I**2 / h**2
        ddh = p.g - F_mag / p.m

        # 电路方程: V = R*I + L*dI/dt
        dI = (V - p.R * I) / p.L

        return [h_dot, ddh, dI]

    def simulate(self, controller, controller_type='pid',
                 disturbance_type='none', disturbance_time=1.0,
                 disturbance_magnitude=0.0, param_perturbation=0.0,
                 track_trajectory=False):
        """闭环仿真"""
        t = np.arange(0, self.T, self.dt)
        N = len(t)

        # 参数摄动
        p = MagneticLevitationParams()
        if param_perturbation > 0:
            p.m *= (1 + param_perturbation * np.random.uniform(-1, 1))
            p.k *= (1 + param_perturbation * np.random.uniform(-1, 1))
            p.h0 *= (1 + param_perturbation * np.random.uniform(-1, 1))

        # 初始状态 (从平衡位置开始)
        x = np.array([p.h0, 0.0, p.I0])

        h_hist = np.zeros(N)
        I_hist = np.zeros(N)
        V_hist = np.zeros(N)
        sp_hist = np.zeros(N)

        if hasattr(controller, 'reset'):
            controller.reset()

        for i in range(N):
            t_val = t[i]

            # 轨迹跟踪: 正弦波
            if track_trajectory:
                sp = p.h0 + 0.003 * np.sin(2 * np.pi * 0.5 * t_val)
            else:
                sp = self.setpoint
            sp_hist[i] = sp

            # 控制器
            if controller_type == 'cascade':
                V = controller.compute(sp, x[0], x[2])
            elif controller_type == 'ff_pid':
                V = controller.compute(sp, x[0], x[2], x[1])
            else:
                V = controller.compute(sp, x[0])

            # 扰动
            dist = 0.0
            if disturbance_type == 'step' and t_val >= disturbance_time:
                dist = disturbance_magnitude
            elif disturbance_type == 'sine' and t_val >= disturbance_time:
                dist = disturbance_magnitude * np.sin(2 * np.pi * 3.0 * t_val)

            V_hist[i] = V
            h_hist[i] = x[0]
            I_hist[i] = x[2]

            # 积分 (4阶Runge-Kutta)
            def deriv(x_val, V_val):
                return self.nonlinear_model(x_val, t_val, V_val, p)

            k1 = deriv(x, V + dist)
            k2 = deriv(x + 0.5 * self.dt * k1, V + dist)
            k3 = deriv(x + 0.5 * self.dt * k2, V + dist)
            k4 = deriv(x + self.dt * k3, V + dist)

            x += (self.dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)

        return t, h_hist, I_hist, V_hist, sp_hist


# ==============================================================================
# 参数优化
# ==============================================================================
def optimize_pid_params(sim):
    """优化简单PID参数"""
    print("  优化简单PID参数...")

    def cost_function(params):
        Kp, Ki, Kd = params
        if Kp <= 0 or Ki < 0 or Kd < 0:
            return 1e6
        ctrl = SimplePIDController(Kp, Ki, Kd, dt=sim.dt)
        t, h, I, V, sp = sim.simulate(ctrl, 'pid')
        metrics = PerformanceMetrics.compute_all(t, h, sim.setpoint)
        cost = (metrics['overshoot'] * 2.0 +
                metrics['rise_time'] * 20.0 +
                metrics['settling_time'] * 10.0 +
                metrics['iae'] * 0.1)
        return cost

    bounds = [(10, 1000), (1, 500), (0.01, 50)]
    result = differential_evolution(cost_function, bounds, seed=42,
                                     maxiter=40, tol=1e-5, popsize=10)
    return result.x


def optimize_cascade_pid_params(sim):
    """优化串级PID参数"""
    print("  优化串级PID参数...")

    def cost_function(params):
        Kp_pos, Ki_pos, Kd_pos, Kp_cur, Ki_cur = params
        if any(p <= 0 for p in params[:3]) or params[3] <= 0:
            return 1e6
        ctrl = CascadePIDController(Kp_pos, Ki_pos, Kd_pos, Kp_cur, Ki_cur, dt=sim.dt)
        t, h, I, V, sp = sim.simulate(ctrl, 'cascade')
        metrics = PerformanceMetrics.compute_all(t, h, sim.setpoint)
        cost = (metrics['overshoot'] * 2.0 +
                metrics['rise_time'] * 20.0 +
                metrics['settling_time'] * 10.0 +
                metrics['iae'] * 0.1)
        return cost

    bounds = [(10, 1000), (1, 500), (0.01, 50),
              (1, 100), (0.1, 50)]
    result = differential_evolution(cost_function, bounds, seed=42,
                                     maxiter=40, tol=1e-5, popsize=10)
    return result.x


# ==============================================================================
# 主仿真与绘图
# ==============================================================================
def run_magnetic_levitation_simulation():
    """运行磁悬浮控制系统仿真"""
    print("=" * 70)
    print("磁悬浮系统仿真 - PID控制稳定性验证")
    print("=" * 70)

    sim = MagneticLevitationSimulation()

    # === 参数优化 ===
    print("\n[1/4] 参数优化搜索中...")

    opt_pid_params = optimize_pid_params(sim)
    print(f"  PID最优参数: Kp={opt_pid_params[0]:.2f}, Ki={opt_pid_params[1]:.2f}, Kd={opt_pid_params[2]:.4f}")

    opt_cascade_params = optimize_cascade_pid_params(sim)
    print(f"  串级PID最优参数: Kp_pos={opt_cascade_params[0]:.2f}, Ki_pos={opt_cascade_params[1]:.2f}, "
          f"Kd_pos={opt_cascade_params[2]:.4f}, Kp_cur={opt_cascade_params[3]:.2f}, Ki_cur={opt_cascade_params[4]:.2f}")

    # === 创建控制器 ===
    pid = SimplePIDController(opt_pid_params[0], opt_pid_params[1], opt_pid_params[2], dt=sim.dt)
    cascade = CascadePIDController(opt_cascade_params[0], opt_cascade_params[1],
                                    opt_cascade_params[2], opt_cascade_params[3],
                                    opt_cascade_params[4], dt=sim.dt)
    ff_pid = FeedforwardPIDController(opt_pid_params[0], opt_pid_params[1],
                                       opt_pid_params[2], Kff=1.0, dt=sim.dt)

    controllers = {
        'PID': (pid, 'pid'),
        'CascadePID': (cascade, 'cascade'),
        'FF+PID': (ff_pid, 'ff_pid'),
    }

    # === 阶跃响应 ===
    print("\n[2/4] 阶跃响应对比...")
    results_step = {}
    for name, (ctrl, ctype) in controllers.items():
        t, h, I, V, sp = sim.simulate(ctrl, ctype)
        metrics = PerformanceMetrics.compute_all(t, h, sim.setpoint)
        results_step[name] = {'t': t, 'h': h, 'I': I, 'V': V, 'sp': sp, 'metrics': metrics}
        print(f"  {name}: 上升时间={metrics['rise_time']:.4f}s, "
              f"超调={metrics['overshoot']:.2f}%, "
              f"调节时间={metrics['settling_time']:.4f}s")

    # === 轨迹跟踪 ===
    print("\n[3/4] 正弦轨迹跟踪...")
    results_track = {}
    for name, (ctrl, ctype) in controllers.items():
        t, h, I, V, sp = sim.simulate(ctrl, ctype, track_trajectory=True)
        metrics = PerformanceMetrics.compute_all(t, h, sim.setpoint)
        results_track[name] = {'t': t, 'h': h, 'sp': sp, 'metrics': metrics}

    # === 绘图 ===
    print("\n[4/4] 生成图表...")

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('磁悬浮系统仿真 - PID控制稳定性验证', fontsize=14, fontweight='bold')

    # 阶跃响应
    ax = axes[0, 0]
    for name in ['PID', 'CascadePID', 'FF+PID']:
        ax.plot(results_step[name]['t'], results_step[name]['h'] * 1000,
                label=f'{name}', linewidth=1.5)
    ax.axhline(y=sim.setpoint * 1000, color='k', linestyle='--', alpha=0.5, label='设定值')
    ax.set_xlabel('时间 [s]')
    ax.set_ylabel('悬浮高度 [mm]')
    ax.set_title('阶跃响应 - 位置')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 电流
    ax = axes[0, 1]
    for name in ['PID', 'CascadePID', 'FF+PID']:
        ax.plot(results_step[name]['t'], results_step[name]['I'],
                label=f'{name}', linewidth=1.5)
    ax.set_xlabel('时间 [s]')
    ax.set_ylabel('电流 [A]')
    ax.set_title('线圈电流')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 轨迹跟踪
    ax = axes[1, 0]
    for name in ['PID', 'CascadePID', 'FF+PID']:
        ax.plot(results_track[name]['t'], results_track[name]['h'] * 1000,
                label=f'{name}', linewidth=1.2)
    ax.plot(results_track[name]['t'], results_track[name]['sp'] * 1000,
            'k--', alpha=0.5, label='设定轨迹')
    ax.set_xlabel('时间 [s]')
    ax.set_ylabel('悬浮高度 [mm]')
    ax.set_title('正弦轨迹跟踪')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 性能指标表
    ax = axes[1, 1]
    ax.axis('off')
    headers = ['指标', 'PID', 'CascadePID', 'FF+PID']
    table_data = []
    key_labels = ['上升时间[s]', '超调量[%]', '调节时间[s]', 'IAE', 'ITAE']
    keys = ['rise_time', 'overshoot', 'settling_time', 'iae', 'itae']

    for label, key in zip(key_labels, keys):
        row = [label]
        for name in ['PID', 'CascadePID', 'FF+PID']:
            val = results_step[name]['metrics'][key]
            if key in ['rise_time', 'settling_time']:
                row.append(f'{val:.4f}')
            elif key in ['iae', 'itae']:
                row.append(f'{val:.6f}')
            else:
                row.append(f'{val:.2f}')
        table_data.append(row)

    table = ax.table(cellText=table_data, colLabels=headers,
                     cellLoc='center', loc='center')
    table.auto_set_font_size(False)
    table.set_fontsize(10)
    table.scale(1.2, 1.5)
    ax.set_title('性能指标对比', pad=20)

    plt.tight_layout()
    plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'magnetic_levitation_simulation.png'),
                dpi=150, bbox_inches='tight')
    print("  图表已保存: magnetic_levitation_simulation.png")

    # === 汇总 ===
    optimal_params = {
        'PID': {
            'Kp': opt_pid_params[0],
            'Ki': opt_pid_params[1],
            'Kd': opt_pid_params[2],
            'metrics': results_step['PID']['metrics']
        },
        'CascadePID': {
            'Kp_pos': opt_cascade_params[0],
            'Ki_pos': opt_cascade_params[1],
            'Kd_pos': opt_cascade_params[2],
            'Kp_cur': opt_cascade_params[3],
            'Ki_cur': opt_cascade_params[4],
            'metrics': results_step['CascadePID']['metrics']
        },
        'FF+PID': {
            'Kp': opt_pid_params[0],
            'Ki': opt_pid_params[1],
            'Kd': opt_pid_params[2],
            'Kff': 1.0,
            'metrics': results_step['FF+PID']['metrics']
        }
    }

    print("\n" + "=" * 70)
    print("磁悬浮仿真完成! 最优参数汇总:")
    print("=" * 70)
    for name, params in optimal_params.items():
        print(f"\n{name}:")
        for k, v in params.items():
            if k != 'metrics':
                print(f"  {k} = {v:.6f}" if isinstance(v, float) else f"  {k} = {v}")
        m = params['metrics']
        print(f"  性能: 上升时间={m['rise_time']:.4f}s, 超调={m['overshoot']:.2f}%, "
              f"调节时间={m['settling_time']:.4f}s")

    return optimal_params


if __name__ == '__main__':
    optimal_params = run_magnetic_levitation_simulation()
    plt.close('all')
