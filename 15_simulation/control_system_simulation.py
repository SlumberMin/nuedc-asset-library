#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
控制系统仿真 - 开环/闭环/串级/前馈对比
==========================================
功能：
  - 一阶/二阶/高阶被控对象建模
  - PID控制器设计与调参
  - 开环 vs 闭环响应对比
  - 串级控制仿真
  - 前馈+反馈复合控制
  - 抗扰性能分析
  - 鲁棒性分析

适用场景：控制系统设计、PID调参、控制策略选择
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
from dataclasses import dataclass, field
from typing import List, Tuple, Callable
import warnings
warnings.filterwarnings('ignore')

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
plt.rcParams['axes.unicode_minus'] = False


# ======================== 被控对象模型 ========================

class PlantModel:
    """被控对象基类"""
    pass


@dataclass
class SecondOrderPlant(PlantModel):
    """二阶系统: K / (s^2 + 2*zeta*wn*s + wn^2)"""
    K: float = 1.0       # 增益
    wn: float = 10.0     # 自然频率 (rad/s)
    zeta: float = 0.3    # 阻尼比

    def transfer_function(self):
        num = [self.K * self.wn**2]
        den = [1, 2*self.zeta*self.wn, self.wn**2]
        return signal.TransferFunction(num, den)


@dataclass
class FirstOrderPlant(PlantModel):
    """一阶系统: K / (tau*s + 1)"""
    K: float = 1.0       # 增益
    tau: float = 0.1     # 时间常数 (s)

    def transfer_function(self):
        return signal.TransferFunction([self.K], [self.tau, 1])


@dataclass
class IntegratingPlant(PlantModel):
    """积分系统: K / s"""
    K: float = 1.0

    def transfer_function(self):
        return signal.TransferFunction([self.K], [1, 0])


@dataclass
class DelayPlant(PlantModel):
    """带延迟的一阶系统"""
    K: float = 1.0
    tau: float = 0.1
    delay: float = 0.05  # 延迟时间 (s)

    def pade_approx(self, order: int = 2):
        """Pade近似处理延迟"""
        num_d, den_d = signal.pade(self.delay, order)
        tf_delay = signal.TransferFunction(num_d, den_d)
        tf_plant = signal.TransferFunction([self.K], [self.tau, 1])
        # 级联
        num = np.convolve(tf_plant.num, tf_delay.num)
        den = np.convolve(tf_plant.den, tf_delay.den)
        return signal.TransferFunction(num, den)


# ======================== PID控制器 ========================

class PIDController:
    """PID控制器（含抗饱和）"""

    def __init__(self, Kp: float = 1.0, Ki: float = 0.0, Kd: float = 0.0,
                 output_min: float = -100, output_max: float = 100,
                 d_filter_tau: float = 0.01):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.output_min = output_min
        self.output_max = output_max
        self.d_filter_tau = d_filter_tau

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_d = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_d = 0.0

    def compute(self, error: float, dt: float) -> float:
        """计算PID输出"""
        # P
        P = self.Kp * error
        # I (带抗饱和)
        self.integral += error * dt
        I = self.Ki * self.integral
        # D (带滤波)
        if dt > 0:
            d_raw = (error - self.prev_error) / dt
            alpha = dt / (self.d_filter_tau + dt)
            d_filtered = alpha * d_raw + (1-alpha) * self.prev_d
            self.prev_d = d_filtered
        else:
            d_filtered = 0
        D = self.Kd * d_filtered

        self.prev_error = error

        # 输出限幅 + 抗饱和
        output = P + I + D
        if output > self.output_max:
            output = self.output_max
            self.integral -= error * dt  # 回退积分
        elif output < self.output_min:
            output = self.output_min
            self.integral -= error * dt

        return output

    def transfer_function(self, dt: float = 0.001):
        """离散PID的连续近似传递函数"""
        # C(s) = Kp + Ki/s + Kd*s/(tau_d*s+1)
        # 简化为: C(s) = (Kd*s^2 + Kp*s + Ki) / (s*(tau_d*s+1))
        num = [self.Kd, self.Kp, self.Ki]
        den = [self.d_filter_tau, 1, 0]
        return signal.TransferFunction(num, den)


# ======================== 控制系统仿真器 ========================

class ControlSystemSimulator:
    """控制系统仿真引擎（时域积分法）"""

    def __init__(self, plant_func: Callable, dt: float = 1e-4):
        self.plant_func = plant_func  # plant_func(u, state, dt) -> (y, new_state)
        self.dt = dt

    def simulate(self, t_end: float, setpoint_func: Callable,
                 controller: PIDController = None,
                 disturbance_func: Callable = None,
                 feedforward_func: Callable = None) -> dict:
        """
        仿真闭环控制系统
        setpoint_func(t) -> reference
        disturbance_func(t) -> disturbance added to plant input
        feedforward_func(t) -> feedforward signal
        """
        dt = self.dt
        n_steps = int(t_end / dt)
        t = np.arange(n_steps) * dt

        y = np.zeros(n_steps)
        u = np.zeros(n_steps)
        ref = np.zeros(n_steps)
        error = np.zeros(n_steps)
        dist = np.zeros(n_steps)
        ff = np.zeros(n_steps)

        plant_state = 0.0  # 一阶系统的状态

        for k in range(n_steps - 1):
            ref[k] = setpoint_func(t[k])
            dist[k] = disturbance_func(t[k]) if disturbance_func else 0

            # 测量输出
            if k > 0:
                y[k], plant_state = self.plant_func(u[k-1] + dist[k], plant_state, dt)

            # 误差
            error[k] = ref[k] - y[k]

            # 控制
            if controller is not None:
                u_ctrl = controller.compute(error[k], dt)
            else:
                u_ctrl = ref[k]  # 开环

            # 前馈
            u_ff = 0
            if feedforward_func is not None:
                u_ff = feedforward_func(t[k])
                ff[k] = u_ff

            u[k] = u_ctrl + u_ff

        # 最后一步
        ref[-1] = setpoint_func(t[-1])
        y[-1], _ = self.plant_func(u[-2] + dist[-2], plant_state, dt)
        error[-1] = ref[-1] - y[-1]

        return {
            't': t, 'y': y, 'u': u, 'ref': ref, 'error': error,
            'disturbance': dist, 'feedforward': ff
        }


# ======================== 性能指标 ========================

class PerformanceMetrics:
    """控制系统性能指标计算"""

    @staticmethod
    def step_response_metrics(t: np.ndarray, y: np.ndarray, ref: float = 1.0) -> dict:
        """阶跃响应性能指标"""
        y_norm = y / ref if ref != 0 else y

        # 上升时间 (10%~90%)
        try:
            t_10 = t[np.where(y_norm >= 0.1)[0][0]]
            t_90 = t[np.where(y_norm >= 0.9)[0][0]]
            rise_time = t_90 - t_10
        except IndexError:
            rise_time = np.nan

        # 超调量
        y_max = np.max(y_norm)
        overshoot = max(0, (y_max - 1.0) * 100)

        # 调节时间 (2%带)
        settled_idx = len(t) - 1
        for i in range(len(t)-1, 0, -1):
            if abs(y_norm[i] - 1.0) > 0.02:
                settled_idx = i
                break
        settling_time = t[min(settled_idx+1, len(t)-1)]

        # 稳态误差
        ss_error = abs(1.0 - y_norm[-1])

        # IAE
        iae = np.trapz(np.abs(1.0 - y_norm), t)

        return {
            'rise_time': rise_time,
            'overshoot': overshoot,
            'settling_time': settling_time,
            'ss_error': ss_error,
            'iae': iae
        }


# ======================== Ziegler-Nichols整定 ========================

class ZNTuner:
    """Ziegler-Nichols PID整定"""

    @staticmethod
    def from_step_response(K: float, tau: float, L: float, method: str = 'ziegler') -> dict:
        """
        基于阶跃响应的整定
        K: 稳态增益, tau: 时间常数, L: 延迟
        """
        if method == 'ziegler':
            Kp = 1.2 * tau / (K * L)
            Ki = Kp / (2 * L)
            Kd = Kp * 0.5 * L
        elif method == 'cohen_coon':
            Kp = (1.35/K) * (tau/L + 0.185)
            Ki = Kp / (L * (2.5 - 2*L/(tau+L)) / (1 - 0.39*L/(tau+L)))
            Kd = Kp * L * (0.37 - 0.37*L/(tau+L)) / (1 - 0.82*L/(tau+L))
        elif method == 'lambda':
            lambda_t = 2 * tau  # 闭环时间常数
            Kp = tau / (K * (lambda_t + L))
            Ki = Kp / tau
            Kd = 0

        return {'Kp': Kp, 'Ki': Ki, 'Kd': Kd}


# ======================== 可视化 ========================

def run_control_simulations():
    """运行所有控制策略仿真对比"""
    print("=" * 60)
    print("控制系统仿真 - 多策略对比")
    print("=" * 60)

    # 被控对象: 二阶系统 K*wn^2 / (s^2 + 2*zeta*wn*s + wn^2)
    # 转换为时域积分形式
    K, wn, zeta = 2.0, 15.0, 0.3
    dt = 1e-4

    # 状态空间积分器 (位置, 速度)
    def plant_ss(u, state, dt, K=K, wn=wn, zeta=zeta):
        """二阶系统状态空间积分"""
        if isinstance(state, (int, float)):
            state = np.array([state, 0.0])
        x1, x2 = state
        dx1 = x2
        dx2 = -wn**2 * x1 - 2*zeta*wn*x2 + K*wn**2 * u
        x1_new = x1 + dx1 * dt
        x2_new = x2 + dx2 * dt
        return x1_new, np.array([x1_new, x2_new])

    sim = ControlSystemSimulator(plant_ss, dt)
    t_end = 1.5

    # 设定值函数
    setpoint = lambda t: 1.0 if t > 0.05 else 0.0
    # 扰动
    disturbance = lambda t: 0.3 if 0.8 <= t <= 1.0 else 0.0

    # ---- 1. 开环 ----
    print("\n[1] 开环响应...")
    def open_loop_plant(u, state, dt):
        # 直接把设定值当输入
        return plant_ss(u, state, dt)
    sim_ol = ControlSystemSimulator(open_loop_plant, dt)
    result_ol = sim_ol.simulate(t_end, setpoint)

    # ---- 2. P控制 ----
    print("[2] P控制...")
    pid_p = PIDController(Kp=5.0, Ki=0, Kd=0)
    result_p = sim.simulate(t_end, setpoint, controller=pid_p)

    # ---- 3. PI控制 ----
    print("[3] PI控制...")
    pid_pi = PIDController(Kp=3.0, Ki=20.0, Kd=0)
    result_pi = sim.simulate(t_end, setpoint, controller=pid_pi)

    # ---- 4. PID控制 ----
    print("[4] PID控制...")
    pid_pid = PIDController(Kp=4.0, Ki=25.0, Kd=0.15)
    result_pid = sim.simulate(t_end, setpoint, controller=pid_pid)

    # ---- 5. 串级控制 ----
    print("[5] 串级控制...")
    # 内环: 速度环 (P控制)
    # 外环: 位置环 (PI控制)
    def cascade_plant(u_outer, state_outer, dt):
        """外环: 简单积分"""
        x = state_outer + u_outer * dt * 0.5
        return x, x

    inner_pid = PIDController(Kp=10.0, Ki=50.0, Kd=0.02)
    outer_pid = PIDController(Kp=2.0, Ki=8.0, Kd=0)

    # 手动串级仿真
    n_steps = int(t_end / dt)
    t_arr = np.arange(n_steps) * dt
    y_cascade = np.zeros(n_steps)
    u_cascade = np.zeros(n_steps)
    inner_state = np.array([0.0, 0.0])
    outer_state = 0.0

    for k in range(n_steps - 1):
        ref = setpoint(t_arr[k])
        # 外环
        outer_err = ref - y_cascade[k]
        inner_ref = outer_pid.compute(outer_err, dt)
        # 内环
        inner_err = inner_ref - inner_state[1]  # 速度反馈
        u_cascade[k] = inner_pid.compute(inner_err, dt)
        # 被控对象
        y_cascade[k+1], inner_state = plant_ss(u_cascade[k], inner_state, dt)

    # ---- 6. 前馈+反馈 ----
    print("[6] 前馈+反馈控制...")
    # 前馈: 基于模型的逆
    def feedforward(t):
        if t > 0.05:
            return 1.0 / K  # 稳态前馈
        return 0.0

    pid_ff = PIDController(Kp=2.0, Ki=15.0, Kd=0.1)
    result_ff = sim.simulate(t_end, setpoint, controller=pid_ff,
                              feedforward_func=feedforward)

    # ---- 7. 抗扰测试 ----
    print("[7] 抗扰性能测试...")
    pid_robust = PIDController(Kp=4.0, Ki=30.0, Kd=0.1)
    result_dist = sim.simulate(t_end, setpoint, controller=pid_robust,
                                disturbance_func=disturbance)

    # 性能指标
    pm = PerformanceMetrics()
    metrics_p = pm.step_response_metrics(result_p['t'], result_p['y'])
    metrics_pi = pm.step_response_metrics(result_pi['t'], result_pi['y'])
    metrics_pid = pm.step_response_metrics(result_pid['t'], result_pid['y'])
    metrics_ff = pm.step_response_metrics(result_ff['t'], result_ff['y'])

    print(f"\n{'策略':<12} {'上升时间':<10} {'超调%':<8} {'调节时间':<10} {'稳态误差':<10} {'IAE':<8}")
    print("-" * 60)
    for name, m in [('P', metrics_p), ('PI', metrics_pi),
                     ('PID', metrics_pid), ('FF+FB', metrics_ff)]:
        print(f"{name:<12} {m['rise_time']:<10.4f} {m['overshoot']:<8.1f} "
              f"{m['settling_time']:<10.4f} {m['ss_error']:<10.4f} {m['iae']:<8.3f}")

    # ======================== 绘图 ========================
    fig, axes = plt.subplots(2, 3, figsize=(18, 11))
    fig.suptitle('控制系统仿真 - 多策略对比', fontsize=16, fontweight='bold')

    # (1) 阶跃响应对比
    ax = axes[0, 0]
    step = 20  # 降采样显示
    ax.plot(result_ol['t'][::step], result_ol['y'][::step], 'k:', label='开环', alpha=0.7)
    ax.plot(result_p['t'][::step], result_p['y'][::step], 'r-', label='P', linewidth=1.2)
    ax.plot(result_pi['t'][::step], result_pi['y'][::step], 'b-', label='PI', linewidth=1.2)
    ax.plot(result_pid['t'][::step], result_pid['y'][::step], 'g-', label='PID', linewidth=1.5)
    ax.plot(result_ff['t'][::step], result_ff['y'][::step], 'm-', label='FF+FB', linewidth=1.5)
    ax.plot(result_ol['t'][::step], [setpoint(ti) for ti in result_ol['t'][::step]],
            'k--', alpha=0.3, label='设定值')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('输出')
    ax.set_title('阶跃响应对比')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # (2) 误差对比
    ax = axes[0, 1]
    ax.plot(result_p['t'][::step], result_p['error'][::step], 'r-', label='P')
    ax.plot(result_pi['t'][::step], result_pi['error'][::step], 'b-', label='PI')
    ax.plot(result_pid['t'][::step], result_pid['error'][::step], 'g-', label='PID')
    ax.plot(result_ff['t'][::step], result_ff['error'][::step], 'm-', label='FF+FB')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('误差')
    ax.set_title('跟踪误差对比')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # (3) 控制量
    ax = axes[0, 2]
    ax.plot(result_p['t'][::step], result_p['u'][::step], 'r-', label='P', alpha=0.7)
    ax.plot(result_pi['t'][::step], result_pi['u'][::step], 'b-', label='PI', alpha=0.7)
    ax.plot(result_pid['t'][::step], result_pid['u'][::step], 'g-', label='PID')
    ax.plot(result_ff['t'][::step], result_ff['u'][::step], 'm-', label='FF+FB')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('控制量')
    ax.set_title('控制输出')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # (4) 串级控制
    ax = axes[1, 0]
    ax.plot(t_arr[::step], y_cascade[::step], 'g-', linewidth=1.5, label='串级输出')
    ax.plot(t_arr[::step], [setpoint(ti) for ti in t_arr[::step]], 'k--', alpha=0.3)
    ax.plot(result_pid['t'][::step], result_pid['y'][::step], 'b:', label='单环PID', alpha=0.7)
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('输出')
    ax.set_title('串级控制 vs 单环PID')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # (5) 抗扰性能
    ax = axes[1, 1]
    ax.plot(result_dist['t'][::step], result_dist['y'][::step], 'r-', linewidth=1.5, label='输出')
    ax.plot(result_dist['t'][::step], [setpoint(ti) for ti in result_dist['t'][::step]],
            'k--', alpha=0.3, label='设定值')
    ax2 = ax.twinx()
    ax2.fill_between(result_dist['t'][::step], 0, result_dist['disturbance'][::step],
                     alpha=0.2, color='orange', label='扰动')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('输出')
    ax2.set_ylabel('扰动')
    ax.set_title('抗扰性能测试')
    ax.legend(loc='upper left')
    ax2.legend(loc='upper right')
    ax.grid(True, alpha=0.3)

    # (6) 性能指标雷达图 (柱状图替代)
    ax = axes[1, 2]
    metrics_list = [metrics_p, metrics_pi, metrics_pid, metrics_ff]
    names = ['P', 'PI', 'PID', 'FF+FB']
    x_pos = np.arange(len(names))
    w = 0.2
    overshoots = [m['overshoot'] for m in metrics_list]
    settling = [m['settling_time']*1000 for m in metrics_list]
    ss_err = [m['ss_error']*100 for m in metrics_list]
    iae_vals = [m['iae'] for m in metrics_list]

    ax.bar(x_pos - 1.5*w, overshoots, w, label='超调(%)', color='salmon')
    ax.bar(x_pos - 0.5*w, settling, w, label='调节时间(ms)', color='skyblue')
    ax.bar(x_pos + 0.5*w, ss_err, w, label='稳态误差(×10⁻²)', color='lightgreen')
    ax.bar(x_pos + 1.5*w, iae_vals, w, label='IAE', color='gold')
    ax.set_xticks(x_pos)
    ax.set_xticklabels(names)
    ax.set_title('性能指标对比')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3, axis='y')

    plt.tight_layout()
    save_path = './nuedc-asset-library/15_simulation/control_system_result.png'
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    print(f"\n图表已保存: {save_path}")
    plt.show()
    print("\n仿真完成！")


if __name__ == '__main__':
    run_control_simulations()
