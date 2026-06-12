#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
直流电机控制系统仿真 - PID/LADRC/ADRC对比
============================================
数学模型：
  电气方程: V = Ra*Ia + La*dIa/dt + Ke*w
  机械方程: J*dw/dt = Kt*Ia - B*w - TL
  传递函数: G(s) = Kt / [(Js+B)(Las+Ra) + Kt*Ke]

控制策略:
  1. 传统PID控制器
  2. LADRC (线性自抗扰控制)
  3. ADRC (非线性自抗扰控制)

仿真内容:
  - 阶跃响应对比
  - 参数扫描与最优参数搜索
  - 性能指标计算(上升时间、超调量、调节时间、IAE、ITAE)
  - 扰动注入(阶跃扰动、正弦扰动、随机噪声)
  - 鲁棒性测试(参数摄动±20%)
"""

import os
import numpy as np

# numpy兼容：np.trapz在1.x废弃，2.x移除，统一用np.trapezoid
if hasattr(np, 'trapezoid'):
    _trapz = np.trapezoid
else:
    _trapz = np.trapz

from scipy import signal
from scipy.integrate import odeint
from scipy.optimize import differential_evolution
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import warnings
warnings.filterwarnings('ignore')

matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False

# ==============================================================================
# 直流电机模型参数
# ==============================================================================
class DCMotorParams:
    """直流电机物理参数"""
    Ra = 2.0       # 电枢电阻 [Ohm]
    La = 0.5e-3    # 电枢电感 [H]
    Kt = 0.01      # 转矩常数 [N.m/A]
    Ke = 0.01      # 反电动势常数 [V/(rad/s)]
    J = 1e-5       # 转动惯量 [kg.m^2]
    B = 0.001      # 粘性摩擦系数 [N.m.s/rad]

    @staticmethod
    def transfer_function():
        """连续传递函数 G(s) = Kt / [(Js+B)(Las+Ra) + Kt*Ke]"""
        p = DCMotorParams
        num = [p.Kt]
        den = np.convolve([p.J, p.B], [p.La, p.Ra])
        # (Js+B)(Las+Ra) + Kt*Ke = J*La*s^2 + (J*Ra + B*La)*s + B*Ra + Kt*Ke
        den = [p.J * p.La,
               p.J * p.Ra + p.B * p.La,
               p.B * p.Ra + p.Kt * p.Ke]
        return signal.TransferFunction(num, den)

    @staticmethod
    def state_space():
        """状态空间模型: x = [Ia, w]"""
        p = DCMotorParams
        # dIa/dt = (-Ra/La)*Ia - (Ke/La)*w + (1/La)*V
        # dw/dt  = (Kt/J)*Ia - (B/J)*w - (1/J)*TL
        A = np.array([[-p.Ra/p.La, -p.Ke/p.La],
                       [p.Kt/p.J,  -p.B/p.J]])
        B = np.array([[1.0/p.La], [0.0]])
        C = np.array([[0.0, 1.0]])
        D = np.array([[0.0]])
        return A, B, C, D


# ==============================================================================
# PID 控制器
# ==============================================================================
class PIDController:
    """带抗饱和的标准PID控制器"""
    def __init__(self, Kp, Ki, Kd, output_limits=(-10, 10), dt=0.001):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.output_limits = output_limits
        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, setpoint, measurement):
        error = setpoint - measurement
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error

        # 抗饱和
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        # 输出限幅与积分抗饱和
        if output > self.output_limits[1]:
            output = self.output_limits[1]
            self.integral -= error * self.dt  # 回退积分
        elif output < self.output_limits[0]:
            output = self.output_limits[0]
            self.integral -= error * self.dt

        return output


# ==============================================================================
# LADRC 控制器 (线性自抗扰控制)
# ==============================================================================
class LADRCController:
    """
    线性自抗扰控制器 (Linear ADRC)
    核心思想: 将内部不确定性和外部扰动统一为"总扰动"，用ESO估计并补偿
    """
    def __init__(self, b0, wc, wo, output_limits=(-10, 10), dt=0.001):
        """
        b0: 控制增益估计值
        wc: 控制器带宽
        wo: 观测器带宽
        """
        self.b0 = b0
        self.wc = wc
        self.wo = wo
        self.dt = dt
        self.output_limits = output_limits

        # ESO状态 [x1_hat, x2_hat, x3_hat] = [输出估计, 输出导数估计, 扰动估计]
        self.z = np.zeros(3)

        # LESO增益 (带宽参数化)
        beta1 = 3 * wo
        beta2 = 3 * wo**2
        beta3 = wo**3
        self.beta = np.array([beta1, beta2, beta3])

        # 控制器增益 (带宽参数化)
        self.kp = wc**2
        self.kd = 2 * wc

    def reset(self):
        self.z = np.zeros(3)

    def compute(self, setpoint, measurement):
        # 1. ESO更新 (4阶Runge-Kutta)
        def eso_dynamics(z):
            dz = np.zeros(3)
            e = measurement - z[0]
            dz[0] = z[1] + self.beta[0] * e
            dz[1] = z[2] + self.beta[1] * e + self.b0 * self._last_u
            dz[2] = self.beta[2] * e
            return dz

        self._last_u = getattr(self, '_last_u', 0.0)
        k1 = eso_dynamics(self.z)
        k2 = eso_dynamics(self.z + 0.5 * self.dt * k1)
        k3 = eso_dynamics(self.z + 0.5 * self.dt * k2)
        k4 = eso_dynamics(self.z + self.dt * k3)
        self.z += (self.dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)

        # 2. 控制律: u = (v0 - z3) / b0
        # PD控制: v0 = kp*(r - z1) - kd*z2
        v0 = self.kp * (setpoint - self.z[0]) - self.kd * self.z[1]
        u = (v0 - self.z[2]) / self.b0

        # 输出限幅
        u = np.clip(u, self.output_limits[0], self.output_limits[1])
        self._last_u = u
        return u


# ==============================================================================
# ADRC 控制器 (非线性自抗扰控制)
# ==============================================================================
class ADRCController:
    """
    非线性自抗扰控制器
    核心组件: 跟踪微分器(TD) + 非线性扩张状态观测器(NESO) + 非线性状态误差反馈(NLSEF)
    """
    def __init__(self, b0, r0, h0, beta1, beta2, beta3,
                 alpha1, alpha2, delta, output_limits=(-10, 10), dt=0.001):
        """
        b0: 控制增益
        r0: 跟踪微分器速度因子
        h0: 跟踪微分器滤波因子
        beta1, beta2, beta3: NESO增益
        alpha1, alpha2: NLSEF非线性因子
        delta: NLSEF线性区间宽度
        """
        self.b0 = b0
        self.r0 = r0
        self.h0 = h0
        self.beta1 = beta1
        self.beta2 = beta2
        self.beta3 = beta3
        self.alpha1 = alpha1
        self.alpha2 = alpha2
        self.delta = delta
        self.dt = dt
        self.output_limits = output_limits

        # TD状态
        self.v1 = 0.0  # 跟踪信号
        self.v2 = 0.0  # 跟踪信号导数

        # NESO状态
        self.z = np.zeros(3)

        self._last_u = 0.0

    def reset(self):
        self.v1 = 0.0
        self.v2 = 0.0
        self.z = np.zeros(3)

    @staticmethod
    def _fhan(x1, x2, r, h):
        """最速综合函数 fhan"""
        d = r * h * h
        a0 = h * x2
        y = x1 + a0
        a1 = np.sqrt(d * (d + 8 * np.abs(y)))
        a2 = a0 + np.sign(y) * (a1 - d) / 2
        sy = (np.sign(y + d) - np.sign(y - d)) / 2
        a = (a0 + y - a2) * sy + a2
        sa = (np.sign(a + d) - np.sign(a - d)) / 2
        return -r * (a / d - np.sign(a)) * sa - r * np.sign(a)

    @staticmethod
    def _fal(x, alpha, delta):
        """非线性函数 fal"""
        if np.abs(x) <= delta:
            return x / (delta ** (1 - alpha))
        else:
            return np.abs(x) ** alpha * np.sign(x)

    def compute(self, setpoint, measurement):
        dt = self.dt

        # 1. 跟踪微分器 (TD)
        fhan_val = self._fhan(self.v1 - setpoint, self.v2, self.r0, self.h0)
        self.v1 += dt * self.v2
        self.v2 += dt * fhan_val

        # 2. 非线性扩张状态观测器 (NESO)
        e1 = measurement - self.z[0]
        self.z[0] += dt * (self.z[1] + self.beta1 * self._fal(e1, 0.5, 0.01))
        self.z[1] += dt * (self.z[2] + self.beta2 * self._fal(e1, 0.25, 0.01) + self.b0 * self._last_u)
        self.z[2] += dt * (self.beta3 * self._fal(e1, 0.125, 0.01))

        # 3. 非线性状态误差反馈 (NLSEF)
        e01 = self.v1 - self.z[0]
        e02 = self.v2 - self.z[1]

        u0 = self._fal(e01, self.alpha1, self.delta) + self._fal(e02, self.alpha2, self.delta)
        u = (u0 - self.z[2]) / self.b0

        # 输出限幅
        u = np.clip(u, self.output_limits[0], self.output_limits[1])
        self._last_u = u
        return u


# ==============================================================================
# 仿真引擎
# ==============================================================================
class DCMotorSimulation:
    """直流电机控制系统仿真引擎"""
    def __init__(self, params=None):
        self.params = params or DCMotorParams()
        self.dt = 0.001  # 1kHz采样
        self.T = 2.0     # 仿真时长
        self.setpoint = 100.0  # 目标转速 (rad/s)
        self.tl = 0.0    # 负载转矩
        self.disturbance_type = 'none'
        self.disturbance_time = 0.5
        self.disturbance_freq = 1.0

    def simulate_open_loop(self, V):
        """开环仿真"""
        p = self.params
        t = np.arange(0, self.T, self.dt)

        def model(x, t_val, u):
            Ia, w = x
            # 扰动计算
            disturbance = self._get_disturbance(t_val)
            dIa = (u - p.Ra * Ia - p.Ke * w) / p.La
            dw = (p.Kt * Ia - p.B * w - disturbance) / p.J
            return [dIa, dw]

        x0 = [0.0, 0.0]
        sol = odeint(model, x0, t, args=(V,))
        return t, sol[:, 1]  # 返回转速

    def _get_disturbance(self, t):
        """根据扰动类型返回扰动值"""
        if self.disturbance_type == 'step':
            return self.tl if t >= self.disturbance_time else 0.0
        elif self.disturbance_type == 'sine':
            return self.tl * np.sin(2 * np.pi * self.disturbance_freq * t) if t >= self.disturbance_time else 0.0
        elif self.disturbance_type == 'noise':
            return self.tl * np.random.randn() if t >= self.disturbance_time else 0.0
        return 0.0

    def simulate_closed_loop(self, controller, disturbance_type='none',
                             disturbance_time=0.5, disturbance_magnitude=0.0,
                             disturbance_freq=1.0, param_perturbation=0.0):
        """
        闭环仿真
        disturbance_type: 'none', 'step', 'sine', 'noise'
        param_perturbation: 参数摄动比例 (如0.2表示±20%)
        """
        p = self.params
        t = np.arange(0, self.T, self.dt)
        N = len(t)

        # 参数摄动
        Ra = p.Ra * (1 + param_perturbation * np.random.uniform(-1, 1))
        La = p.La * (1 + param_perturbation * np.random.uniform(-1, 1))
        Kt = p.Kt * (1 + param_perturbation * np.random.uniform(-1, 1))
        Ke = p.Ke * (1 + param_perturbation * np.random.uniform(-1, 1))
        J = p.J * (1 + param_perturbation * np.random.uniform(-1, 1))
        B = p.B * (1 + param_perturbation * np.random.uniform(-1, 1))

        # 存储结果
        Ia_hist = np.zeros(N)
        w_hist = np.zeros(N)
        u_hist = np.zeros(N)
        e_hist = np.zeros(N)

        Ia = 0.0
        w = 0.0

        self.disturbance_type = disturbance_type
        self.disturbance_time = disturbance_time
        self.tl = disturbance_magnitude
        self.disturbance_freq = disturbance_freq

        controller.reset()

        for i in range(N):
            t_val = t[i]
            # 控制器输出
            u = controller.compute(self.setpoint, w)
            u = np.clip(u, -10, 10)  # 电压限幅

            # 扰动
            dist = self._get_disturbance(t_val)

            # 电机模型 (欧拉法)
            dIa = (u - Ra * Ia - Ke * w) / La
            dw_val = (Kt * Ia - B * w - dist) / J

            Ia += dIa * self.dt
            w += dw_val * self.dt

            Ia_hist[i] = Ia
            w_hist[i] = w
            u_hist[i] = u
            e_hist[i] = self.setpoint - w

        return t, w_hist, u_hist, e_hist


# ==============================================================================
# 性能指标计算
# ==============================================================================
class PerformanceMetrics:
    """控制系统性能指标"""

    @staticmethod
    def rise_time(t, y, setpoint):
        """上升时间: 从10%到90%设定值"""
        y10 = 0.1 * setpoint
        y90 = 0.9 * setpoint
        t10 = t[np.argmax(y >= y10)]
        t90 = t[np.argmax(y >= y90)]
        return t90 - t10

    @staticmethod
    def overshoot(y, setpoint):
        """超调量 (%)"""
        max_val = np.max(y)
        return max((max_val - setpoint) / setpoint * 100, 0)

    @staticmethod
    def settling_time(t, y, setpoint, tolerance=0.02):
        """调节时间 (±tolerance*setpoint)"""
        band_upper = setpoint * (1 + tolerance)
        band_lower = setpoint * (1 - tolerance)
        # 找到最后一次超出容差带的时间
        out_of_band = np.where((y > band_upper) | (y < band_lower))[0]
        if len(out_of_band) == 0:
            return 0.0
        return t[out_of_band[-1]]

    @staticmethod
    def iae(t, e):
        """积分绝对误差"""
        return _trapz(np.abs(e), t)

    @staticmethod
    def itae(t, e):
        """积分时间绝对误差"""
        return _trapz(t * np.abs(e), t)

    @staticmethod
    def compute_all(t, y, setpoint):
        """计算所有性能指标"""
        e = setpoint - y
        return {
            'rise_time': PerformanceMetrics.rise_time(t, y, setpoint),
            'overshoot': PerformanceMetrics.overshoot(y, setpoint),
            'settling_time': PerformanceMetrics.settling_time(t, y, setpoint),
            'iae': PerformanceMetrics.iae(t, e),
            'itae': PerformanceMetrics.itae(t, e),
            'steady_state_error': np.abs(setpoint - y[-1]),
        }


# ==============================================================================
# 参数优化
# ==============================================================================
class ParameterOptimizer:
    """参数优化器"""

    def __init__(self, sim_engine):
        self.sim = sim_engine

    def optimize_pid(self):
        """搜索最优PID参数 (差分进化算法)"""
        print("  优化PID参数...")

        def cost_function(params):
            Kp, Ki, Kd = params
            if Kp < 0 or Ki < 0 or Kd < 0:
                return 1e6
            controller = PIDController(Kp, Ki, Kd, dt=self.sim.dt)
            t, w, u, e = self.sim.simulate_closed_loop(controller)
            metrics = PerformanceMetrics.compute_all(t, w, self.sim.setpoint)
            # 综合代价函数
            cost = (metrics['overshoot'] * 2.0 +
                    metrics['rise_time'] * 10.0 +
                    metrics['settling_time'] * 5.0 +
                    metrics['itae'] * 0.01)
            return cost

        bounds = [(0.1, 20.0), (0.01, 100.0), (0.001, 1.0)]
        result = differential_evolution(cost_function, bounds, seed=42,
                                         maxiter=50, tol=1e-6, popsize=10)
        return result.x

    def optimize_ladrc(self, plant_b0):
        """搜索最优LADRC参数"""
        print("  优化LADRC参数...")

        def cost_function(params):
            wc, wo = params
            if wc <= 0 or wo <= 0:
                return 1e6
            controller = LADRCController(plant_b0, wc, wo, dt=self.sim.dt)
            t, w, u, e = self.sim.simulate_closed_loop(controller)
            metrics = PerformanceMetrics.compute_all(t, w, self.sim.setpoint)
            cost = (metrics['overshoot'] * 2.0 +
                    metrics['rise_time'] * 10.0 +
                    metrics['settling_time'] * 5.0 +
                    metrics['itae'] * 0.01)
            return cost

        bounds = [(10, 500), (100, 2000)]
        result = differential_evolution(cost_function, bounds, seed=42,
                                         maxiter=50, tol=1e-6, popsize=10)
        return result.x

    def optimize_adrc(self, plant_b0):
        """搜索最优ADRC参数"""
        print("  优化ADRC参数...")

        def cost_function(params):
            r0, h0, beta1, beta2, beta3, alpha1, alpha2, delta = params
            if r0 <= 0 or h0 <= 0:
                return 1e6
            try:
                controller = ADRCController(
                    b0=plant_b0, r0=r0, h0=h0,
                    beta1=beta1, beta2=beta2, beta3=beta3,
                    alpha1=alpha1, alpha2=alpha2, delta=delta,
                    dt=self.sim.dt
                )
                t, w, u, e = self.sim.simulate_closed_loop(controller)
                metrics = PerformanceMetrics.compute_all(t, w, self.sim.setpoint)
                cost = (metrics['overshoot'] * 2.0 +
                        metrics['rise_time'] * 10.0 +
                        metrics['settling_time'] * 5.0 +
                        metrics['itae'] * 0.01)
                return cost
            except Exception:
                return 1e6

        bounds = [(50, 500), (0.001, 0.1),
                  (50, 500), (50, 500), (50, 500),
                  (0.5, 1.5), (0.25, 1.0), (0.001, 0.1)]
        result = differential_evolution(cost_function, bounds, seed=42,
                                         maxiter=30, tol=1e-4, popsize=8)
        return result.x


# ==============================================================================
# 鲁棒性测试
# ==============================================================================
class RobustnessTest:
    """鲁棒性测试"""

    def __init__(self, sim_engine, controller, controller_name):
        self.sim = sim_engine
        self.controller = controller
        self.controller_name = controller_name

    def parameter_perturbation_test(self, perturbation_levels=[0.0, 0.1, 0.2]):
        """参数摄动鲁棒性测试"""
        results = {}
        for pert in perturbation_levels:
            metrics_list = []
            n_trials = 20
            for _ in range(n_trials):
                t, w, u, e = self.sim.simulate_closed_loop(
                    self.controller, param_perturbation=pert
                )
                metrics = PerformanceMetrics.compute_all(t, w, self.sim.setpoint)
                metrics_list.append(metrics)

            # 平均指标
            avg_metrics = {}
            for key in metrics_list[0]:
                avg_metrics[key] = np.mean([m[key] for m in metrics_list])
                avg_metrics[key + '_std'] = np.std([m[key] for m in metrics_list])
            results[f'perturbation_{int(pert*100)}%'] = avg_metrics

        return results


# ==============================================================================
# 主仿真与绘图
# ==============================================================================
def run_dc_motor_simulation():
    """运行完整的直流电机控制系统仿真"""
    print("=" * 70)
    print("直流电机控制系统仿真 - PID / LADRC / ADRC 对比")
    print("=" * 70)

    # 创建仿真引擎
    sim = DCMotorSimulation()
    optimizer = ParameterOptimizer(sim)
    p = DCMotorParams()

    # === 第1部分: 参数优化 ===
    print("\n[1/5] 参数优化搜索中...")

    # PID参数优化
    opt_pid_params = optimizer.optimize_pid()
    print(f"  PID最优参数: Kp={opt_pid_params[0]:.4f}, Ki={opt_pid_params[1]:.4f}, Kd={opt_pid_params[2]:.4f}")

    # LADRC参数优化
    plant_b0 = p.Kt / (p.J * p.La)  # 控制增益
    opt_ladrc_params = optimizer.optimize_ladrc(plant_b0)
    print(f"  LADRC最优参数: wc={opt_ladrc_params[0]:.2f}, wo={opt_ladrc_params[1]:.2f}")

    # ADRC参数优化
    opt_adrc_params = optimizer.optimize_adrc(plant_b0)
    print(f"  ADRC最优参数: r0={opt_adrc_params[0]:.2f}, h0={opt_adrc_params[1]:.4f}")

    # === 第2部分: 控制器创建 ===
    pid = PIDController(opt_pid_params[0], opt_pid_params[1], opt_pid_params[2], dt=sim.dt)
    ladrc = LADRCController(plant_b0, opt_ladrc_params[0], opt_ladrc_params[1], dt=sim.dt)
    adrc = ADRCController(
        b0=plant_b0,
        r0=opt_adrc_params[0], h0=opt_adrc_params[1],
        beta1=opt_adrc_params[2], beta2=opt_adrc_params[3],
        beta3=opt_adrc_params[4],
        alpha1=opt_adrc_params[5], alpha2=opt_adrc_params[6],
        delta=opt_adrc_params[7],
        dt=sim.dt
    )

    controllers = {'PID': pid, 'LADRC': ladrc, 'ADRC': adrc}

    # === 第3部分: 阶跃响应对比 ===
    print("\n[2/5] 阶跃响应对比...")
    results_step = {}
    for name, ctrl in controllers.items():
        t, w, u, e = sim.simulate_closed_loop(ctrl)
        metrics = PerformanceMetrics.compute_all(t, w, sim.setpoint)
        results_step[name] = {'t': t, 'w': w, 'u': u, 'e': e, 'metrics': metrics}
        print(f"  {name}: 上升时间={metrics['rise_time']:.4f}s, "
              f"超调={metrics['overshoot']:.2f}%, "
              f"调节时间={metrics['settling_time']:.4f}s, "
              f"IAE={metrics['iae']:.4f}, ITAE={metrics['itae']:.4f}")

    # === 第4部分: 扰动测试 ===
    print("\n[3/5] 扰动注入测试...")
    disturbance_tests = {
        'step': {'type': 'step', 'mag': 0.005, 'freq': 1.0},
        'sine': {'type': 'sine', 'mag': 0.005, 'freq': 5.0},
        'noise': {'type': 'noise', 'mag': 0.002, 'freq': 1.0},
    }

    results_disturbance = {}
    for dist_name, dist_params in disturbance_tests.items():
        results_disturbance[dist_name] = {}
        for name, ctrl in controllers.items():
            t, w, u, e = sim.simulate_closed_loop(
                ctrl,
                disturbance_type=dist_params['type'],
                disturbance_time=0.5,
                disturbance_magnitude=dist_params['mag'],
                disturbance_freq=dist_params['freq']
            )
            metrics = PerformanceMetrics.compute_all(t, w, sim.setpoint)
            results_disturbance[dist_name][name] = {'t': t, 'w': w, 'metrics': metrics}

    # === 第5部分: 鲁棒性测试 ===
    print("\n[4/5] 鲁棒性测试 (参数摄动±20%)...")
    robustness_results = {}
    for name, ctrl in controllers.items():
        robust_test = RobustnessTest(sim, ctrl, name)
        robustness_results[name] = robust_test.parameter_perturbation_test([0.0, 0.1, 0.2])
        print(f"  {name} 鲁棒性测试完成")

    # === 绘图 ===
    print("\n[5/5] 生成图表...")

    # 图1: 阶跃响应对比
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('直流电机控制系统仿真 - PID/LADRC/ADRC对比', fontsize=14, fontweight='bold')

    # 1.1 转速响应
    ax = axes[0, 0]
    for name in ['PID', 'LADRC', 'ADRC']:
        ax.plot(results_step[name]['t'], results_step[name]['w'],
                label=f'{name}', linewidth=1.5)
    ax.axhline(y=sim.setpoint, color='k', linestyle='--', alpha=0.5, label='设定值')
    ax.set_xlabel('时间 [s]')
    ax.set_ylabel('转速 [rad/s]')
    ax.set_title('阶跃响应 - 转速')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 1.2 控制量
    ax = axes[0, 1]
    for name in ['PID', 'LADRC', 'ADRC']:
        ax.plot(results_step[name]['t'], results_step[name]['u'],
                label=f'{name}', linewidth=1.5)
    ax.set_xlabel('时间 [s]')
    ax.set_ylabel('控制电压 [V]')
    ax.set_title('控制量输出')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 1.3 误差
    ax = axes[1, 0]
    for name in ['PID', 'LADRC', 'ADRC']:
        ax.plot(results_step[name]['t'], results_step[name]['e'],
                label=f'{name}', linewidth=1.5)
    ax.set_xlabel('时间 [s]')
    ax.set_ylabel('误差 [rad/s]')
    ax.set_title('跟踪误差')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 1.4 性能指标对比表
    ax = axes[1, 1]
    ax.axis('off')
    table_data = []
    headers = ['指标', 'PID', 'LADRC', 'ADRC']
    for key in ['rise_time', 'overshoot', 'settling_time', 'iae', 'itae', 'steady_state_error']:
        row = [key]
        for name in ['PID', 'LADRC', 'ADRC']:
            val = results_step[name]['metrics'][key]
            if key in ['rise_time', 'settling_time']:
                row.append(f'{val:.4f}')
            elif key in ['iae', 'itae']:
                row.append(f'{val:.4f}')
            else:
                row.append(f'{val:.2f}')
        table_data.append(row)

    key_labels = ['上升时间[s]', '超调量[%]', '调节时间[s]', 'IAE', 'ITAE', '稳态误差']
    for i, label in enumerate(key_labels):
        table_data[i][0] = label

    table = ax.table(cellText=table_data, colLabels=headers,
                     cellLoc='center', loc='center')
    table.auto_set_font_size(False)
    table.set_fontsize(10)
    table.scale(1.2, 1.5)
    ax.set_title('性能指标对比', pad=20)

    plt.tight_layout()
    plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'dc_motor_step_response.png'),
                dpi=150, bbox_inches='tight')
    print("  图1已保存: dc_motor_step_response.png")

    # 图2: 扰动响应对比
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    fig.suptitle('扰动注入下的响应对比', fontsize=14, fontweight='bold')

    for idx, (dist_name, dist_data) in enumerate(results_disturbance.items()):
        ax = axes[idx]
        for name in ['PID', 'LADRC', 'ADRC']:
            ax.plot(dist_data[name]['t'], dist_data[name]['w'],
                    label=f'{name}', linewidth=1.2)
        ax.axhline(y=sim.setpoint, color='k', linestyle='--', alpha=0.5)
        ax.set_xlabel('时间 [s]')
        ax.set_ylabel('转速 [rad/s]')
        ax.set_title(f'{dist_name}扰动')
        ax.legend()
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'dc_motor_disturbance.png'),
                dpi=150, bbox_inches='tight')
    print("  图2已保存: dc_motor_disturbance.png")

    # 图3: 鲁棒性测试
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    fig.suptitle('鲁棒性测试 - 参数摄动对性能的影响', fontsize=14, fontweight='bold')

    perturbation_labels = ['0%', '10%', '20%']
    metrics_to_plot = ['overshoot', 'settling_time', 'iae']

    for idx, metric in enumerate(metrics_to_plot):
        ax = axes[idx]
        for name in ['PID', 'LADRC', 'ADRC']:
            values = []
            for pert_label in ['perturbation_0%', 'perturbation_10%', 'perturbation_20%']:
                values.append(robustness_results[name][pert_label][metric])
            ax.plot(perturbation_labels, values, 'o-', label=name, linewidth=1.5)
        ax.set_xlabel('参数摄动')
        ax.set_ylabel(metric)
        ax.set_title(f'{metric} 随参数摄动的变化')
        ax.legend()
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'dc_motor_robustness.png'),
                dpi=150, bbox_inches='tight')
    print("  图3已保存: dc_motor_robustness.png")

    # === 汇总最优参数 ===
    optimal_params = {
        'PID': {
            'Kp': opt_pid_params[0],
            'Ki': opt_pid_params[1],
            'Kd': opt_pid_params[2],
            'metrics': results_step['PID']['metrics']
        },
        'LADRC': {
            'wc': opt_ladrc_params[0],
            'wo': opt_ladrc_params[1],
            'b0': plant_b0,
            'metrics': results_step['LADRC']['metrics']
        },
        'ADRC': {
            'r0': opt_adrc_params[0],
            'h0': opt_adrc_params[1],
            'b0': plant_b0,
            'metrics': results_step['ADRC']['metrics']
        }
    }

    print("\n" + "=" * 70)
    print("仿真完成! 最优参数汇总:")
    print("=" * 70)
    for name, params in optimal_params.items():
        print(f"\n{name}:")
        for k, v in params.items():
            if k != 'metrics':
                print(f"  {k} = {v:.6f}")
        m = params['metrics']
        print(f"  性能: 上升时间={m['rise_time']:.4f}s, 超调={m['overshoot']:.2f}%, "
              f"调节时间={m['settling_time']:.4f}s")

    return optimal_params


if __name__ == '__main__':
    optimal_params = run_dc_motor_simulation()
    plt.close('all')
