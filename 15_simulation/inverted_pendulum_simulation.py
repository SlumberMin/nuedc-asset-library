#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
倒立摆/平衡系统仿真 - LQR/MPC/SMC对比
========================================
数学模型:
  θ'' = (g/l)*sin(θ) - (d/(m*l²))*θ' + (1/(m*l))*F*cos(θ)
  线性化: θ'' ≈ (g/l)*θ - (d/(m*l²))*θ' + (1/(m*l))*u

控制策略:
  1. LQR (线性二次型调节器)
  2. MPC (模型预测控制) 简化实现
  3. SMC (滑模控制)

仿真内容:
  - 非线性模型仿真
  - 三种控制策略对比
  - 参数扫描与最优参数搜索
  - 性能指标计算
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
from scipy.linalg import solve_continuous_are
from scipy.optimize import minimize, differential_evolution
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib
import warnings
warnings.filterwarnings('ignore')

matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False

# ==============================================================================
# 倒立摆模型参数
# ==============================================================================
class InvertedPendulumParams:
    """倒立摆物理参数"""
    m = 0.1       # 摆杆质量 [kg]
    l = 0.3       # 摆杆半长 [m]
    g = 9.81      # 重力加速度 [m/s^2]
    d = 0.01      # 阻尼系数 [N.m.s/rad]
    M = 0.5       # 小车质量 [kg] (如果考虑小车)

    @classmethod
    def state_space(cls):
        """线性化状态空间: x = [θ, θ', x, x'] (考虑小车)"""
        # 简化模型: x = [θ, θ']
        A = np.array([[0, 1],
                       [cls.g / cls.l, -cls.d / (cls.m * cls.l**2)]])
        B = np.array([[0],
                       [np.cos(0) / (cls.m * cls.l)]])  # 在平衡点θ=0
        return A, B

    @classmethod
    def linearized_ss_full(cls):
        """完整4阶线性化模型: x = [θ, θ', x, x']"""
        p = cls
        # 倒立摆线性化 (θ≈0, 小车位移x)
        M_total = p.M + p.m
        A = np.array([
            [0, 1, 0, 0],
            [p.g * M_total / (p.M * p.l),  -p.d / (p.m * p.l**2), 0, 0],
            [0, 0, 0, 1],
            [-p.g * p.m / p.M, 0, 0, 0]
        ])
        B = np.array([
            [0],
            [1.0 / (p.M * p.l)],
            [0],
            [1.0 / M_total]
        ])
        C = np.array([[1, 0, 0, 0]])  # 输出角度
        D = np.array([[0]])
        return A, B, C, D


# ==============================================================================
# LQR 控制器
# ==============================================================================
class LQRController:
    """线性二次型调节器"""
    def __init__(self, A, B, Q, R):
        """
        A, B: 系统矩阵
        Q: 状态权重矩阵
        R: 控制权重
        """
        self.A = A
        self.B = B
        self.Q = Q
        self.R = R

        # 求解连续代数Riccati方程: A'P + PA - PBR^-1B'P + Q = 0
        try:
            P = solve_continuous_are(A, B, Q, R)
            self.K = np.linalg.inv(R) @ B.T @ P
            self.valid = True
        except:
            self.K = np.zeros((1, A.shape[0]))
            self.valid = False

    def compute(self, x):
        """计算控制量 u = -K*x"""
        if self.valid:
            return -self.K @ x
        return 0.0


# ==============================================================================
# MPC 控制器 (简化实现)
# ==============================================================================
class MPCController:
    """简化MPC控制器 (基于离散化LQR的滚动优化)"""
    def __init__(self, A, B, Q, R, N=10, dt=0.01):
        """
        N: 预测时域
        dt: 离散化步长
        """
        self.A = A
        self.B = B
        self.Q = Q
        self.R = R
        self.N = N
        self.dt = dt

        # 离散化
        self.Ad = np.eye(A.shape[0]) + A * dt
        self.Bd = B * dt

    def compute(self, x, u_min=-10, u_max=10):
        """基于序列二次规划的简化MPC"""
        n = self.A.shape[0]
        m = self.B.shape[1]

        # 初始化控制序列
        u_seq = np.zeros((self.N, m))

        # 简化: 使用梯度下降优化控制序列
        for _ in range(5):  # 少量迭代
            cost = 0
            grad = np.zeros_like(u_seq)

            x_pred = x.copy()
            for k in range(self.N):
                # 状态代价
                cost += x_pred @ self.Q @ x_pred
                # 控制代价
                if k == 0:
                    cost += u_seq[k] @ self.R @ u_seq[k]
                else:
                    cost += u_seq[k] @ self.R @ u_seq[k] + \
                            (u_seq[k] - u_seq[k-1]) @ (0.1 * self.R) @ (u_seq[k] - u_seq[k-1])

                # 梯度 (简化)
                grad[k] = 2 * self.R @ u_seq[k]

                # 预测下一步
                x_pred = self.Ad @ x_pred + self.Bd @ u_seq[k]

            # 梯度下降更新
            u_seq -= 0.01 * grad
            # 控制约束
            u_seq = np.clip(u_seq, u_min, u_max)

        return u_seq[0]


# ==============================================================================
# SMC 控制器 (滑模控制)
# ==============================================================================
class SMCController:
    """滑模控制器"""
    def __init__(self, A, B, c=None, eta=5.0, phi=0.1):
        """
        c: 滑模面参数 s = c*e + de/dt
        eta: 切换增益
        phi: 边界层宽度
        """
        self.A = A
        self.B = B
        self.c = c if c is not None else 10.0
        self.eta = eta
        self.phi = phi

    def compute(self, x):
        """计算控制量"""
        # 滑模面: s = c*θ + θ'
        theta = x[0]
        dtheta = x[1]

        s = self.c * theta + dtheta

        # 等效控制
        g = 9.81
        l = 0.3
        m = 0.1
        d = 0.01

        # 等效控制使 s' = 0
        u_eq = -l * (self.c * dtheta + (g/l) * np.sin(theta) - (d/(m*l**2)) * dtheta)

        # 切换控制
        if np.abs(s) > self.phi:
            u_sw = -self.eta * np.sign(s)
        else:
            u_sw = -self.eta * s / self.phi  # 边界层内线性化

        return u_eq + u_sw


# ==============================================================================
# 性能指标
# ==============================================================================
class PerformanceMetrics:
    @staticmethod
    def rise_time(t, y, setpoint=0.0, tol=0.1):
        """上升时间: 从初始值到90%设定值"""
        y0 = y[0]
        target = setpoint + 0.9 * (setpoint - y0)
        if setpoint > y0:
            idx = np.where(y >= target)[0]
        else:
            idx = np.where(y <= target)[0]
        if len(idx) == 0:
            return t[-1]
        return t[idx[0]]

    @staticmethod
    def max_overshoot(y, setpoint=0.0):
        """最大超调量"""
        return np.max(np.abs(y - setpoint))

    @staticmethod
    def settling_time(t, y, setpoint=0.0, tolerance=0.02):
        """调节时间"""
        threshold = tolerance * np.max(np.abs(y - setpoint)) if np.max(np.abs(y - setpoint)) > 0 else tolerance
        out_of_band = np.where(np.abs(y - setpoint) > threshold)[0]
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
    def compute_all(t, y, setpoint=0.0):
        e = y - setpoint
        return {
            'rise_time': PerformanceMetrics.rise_time(t, y, setpoint),
            'max_overshoot': PerformanceMetrics.max_overshoot(y, setpoint),
            'settling_time': PerformanceMetrics.settling_time(t, y, setpoint),
            'iae': PerformanceMetrics.iae(t, e),
            'itae': PerformanceMetrics.itae(t, e),
        }


# ==============================================================================
# 仿真引擎
# ==============================================================================
class InvertedPendulumSimulation:
    """倒立摆仿真引擎"""
    def __init__(self, params=None):
        self.params = params or InvertedPendulumParams()
        self.dt = 0.01
        self.T = 5.0
        self.theta0 = 0.2  # 初始偏角 (约11.5度)

    def nonlinear_model(self, x, t, u, params=None):
        """非线性动力学模型"""
        p = params or self.params
        theta, dtheta = x
        u = np.clip(u, -10, 10)

        ddtheta = (p.g / p.l) * np.sin(theta) - \
                  (p.d / (p.m * p.l**2)) * dtheta + \
                  (1.0 / (p.m * p.l)) * u * np.cos(theta)

        return [dtheta, ddtheta]

    def simulate(self, controller, controller_type='lqr',
                 disturbance_type='none', disturbance_time=2.0,
                 disturbance_magnitude=0.0, param_perturbation=0.0):
        """闭环仿真"""
        t = np.arange(0, self.T, self.dt)
        N = len(t)

        # 参数摄动
        p = InvertedPendulumParams()
        if param_perturbation > 0:
            p.g *= (1 + param_perturbation * np.random.uniform(-1, 1))
            p.m *= (1 + param_perturbation * np.random.uniform(-1, 1))
            p.l *= (1 + param_perturbation * np.random.uniform(-1, 1))
            p.d *= (1 + param_perturbation * np.random.uniform(-1, 1))

        x = np.array([self.theta0, 0.0])
        x_hist = np.zeros((N, 2))
        u_hist = np.zeros(N)

        for i in range(N):
            t_val = t[i]
            x_hist[i] = x

            # 计算控制量
            if controller_type == 'lqr':
                u = controller.compute(x)[0] if hasattr(controller, 'compute') else 0
            elif controller_type == 'mpc':
                u = controller.compute(x)
                if hasattr(u, '__len__'):
                    u = u[0]
            elif controller_type == 'smc':
                u = controller.compute(x)

            u = np.clip(u, -10, 10)
            u_hist[i] = u

            # 扰动
            dist = 0.0
            if disturbance_type == 'step' and t_val >= disturbance_time:
                dist = disturbance_magnitude
            elif disturbance_type == 'sine' and t_val >= disturbance_time:
                dist = disturbance_magnitude * np.sin(2 * np.pi * 2.0 * t_val)

            # 非线性模型积分
            dx = self.nonlinear_model(x, t_val, u + dist, p)
            x[0] += dx[0] * self.dt
            x[1] += dx[1] * self.dt

        return t, x_hist[:, 0], u_hist  # 返回角度和控制量


# ==============================================================================
# 参数优化
# ==============================================================================
def optimize_lqr_params(sim):
    """优化LQR的Q和R矩阵"""
    print("  优化LQR参数...")

    def cost_function(params):
        q1, q2, r = params
        if q1 <= 0 or q2 <= 0 or r <= 0:
            return 1e6
        Q = np.diag([q1, q2])
        R_val = np.array([[r]])
        A, B = InvertedPendulumParams.state_space()
        try:
            lqr = LQRController(A, B, Q, R_val)
            if not lqr.valid:
                return 1e6
            t, theta, u = sim.simulate(lqr, 'lqr')
            metrics = PerformanceMetrics.compute_all(t, theta, 0.0)
            cost = (metrics['max_overshoot'] * 10.0 +
                    metrics['settling_time'] * 5.0 +
                    metrics['iae'] * 1.0)
            return cost
        except:
            return 1e6

    bounds = [(1, 500), (1, 500), (0.01, 100)]
    result = differential_evolution(cost_function, bounds, seed=42,
                                     maxiter=30, tol=1e-4, popsize=8)
    return result.x


def optimize_smc_params(sim):
    """优化SMC参数"""
    print("  优化SMC参数...")

    def cost_function(params):
        c, eta, phi = params
        if c <= 0 or eta <= 0 or phi <= 0:
            return 1e6
        A, B = InvertedPendulumParams.state_space()
        smc = SMCController(A, B, c=c, eta=eta, phi=phi)
        t, theta, u = sim.simulate(smc, 'smc')
        metrics = PerformanceMetrics.compute_all(t, theta, 0.0)
        # 加入控制量抖振惩罚
        chattering = np.mean(np.abs(np.diff(u)))
        cost = (metrics['max_overshoot'] * 10.0 +
                metrics['settling_time'] * 5.0 +
                metrics['iae'] * 1.0 +
                chattering * 2.0)
        return cost

    bounds = [(1, 50), (1, 20), (0.01, 1.0)]
    result = differential_evolution(cost_function, bounds, seed=42,
                                     maxiter=30, tol=1e-4, popsize=8)
    return result.x


# ==============================================================================
# 主仿真与绘图
# ==============================================================================
def run_inverted_pendulum_simulation():
    """运行倒立摆控制系统仿真"""
    print("=" * 70)
    print("倒立摆/平衡系统仿真 - LQR / MPC / SMC 对比")
    print("=" * 70)

    sim = InvertedPendulumSimulation()
    A, B = InvertedPendulumParams.state_space()

    # === 参数优化 ===
    print("\n[1/4] 参数优化搜索中...")

    # LQR优化
    opt_lqr_params = optimize_lqr_params(sim)
    print(f"  LQR最优参数: Q=diag([{opt_lqr_params[0]:.2f}, {opt_lqr_params[1]:.2f}]), "
          f"R={opt_lqr_params[2]:.4f}")

    # SMC优化
    opt_smc_params = optimize_smc_params(sim)
    print(f"  SMC最优参数: c={opt_smc_params[0]:.2f}, eta={opt_smc_params[1]:.2f}, "
          f"phi={opt_smc_params[2]:.4f}")

    # === 创建控制器 ===
    Q_lqr = np.diag([opt_lqr_params[0], opt_lqr_params[1]])
    R_lqr = np.array([[opt_lqr_params[2]]])
    lqr = LQRController(A, B, Q_lqr, R_lqr)

    mpc = MPCController(A, B, Q_lqr, R_lqr, N=15, dt=sim.dt)

    smc = SMCController(A, B, c=opt_smc_params[0],
                        eta=opt_smc_params[1], phi=opt_smc_params[2])

    controllers = {'LQR': (lqr, 'lqr'), 'MPC': (mpc, 'mpc'), 'SMC': (smc, 'smc')}

    # === 阶跃响应对比 ===
    print("\n[2/4] 阶跃响应对比...")
    results_step = {}
    for name, (ctrl, ctype) in controllers.items():
        t, theta, u = sim.simulate(ctrl, ctype)
        metrics = PerformanceMetrics.compute_all(t, theta, 0.0)
        results_step[name] = {'t': t, 'theta': theta, 'u': u, 'metrics': metrics}
        print(f"  {name}: 最大偏角={metrics['max_overshoot']:.4f}rad, "
              f"调节时间={metrics['settling_time']:.4f}s, "
              f"IAE={metrics['iae']:.4f}")

    # === 扰动测试 ===
    print("\n[3/4] 扰动注入测试...")
    disturbance_tests = {
        'step': {'type': 'step', 'mag': 0.5, 'time': 2.0},
        'sine': {'type': 'sine', 'mag': 0.3, 'time': 2.0},
    }

    results_dist = {}
    for dist_name, dist_p in disturbance_tests.items():
        results_dist[dist_name] = {}
        for name, (ctrl, ctype) in controllers.items():
            t, theta, u = sim.simulate(ctrl, ctype,
                                        disturbance_type=dist_p['type'],
                                        disturbance_time=dist_p['time'],
                                        disturbance_magnitude=dist_p['mag'])
            metrics = PerformanceMetrics.compute_all(t, theta, 0.0)
            results_dist[dist_name][name] = {'t': t, 'theta': theta, 'metrics': metrics}

    # === 绘图 ===
    print("\n[4/4] 生成图表...")

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('倒立摆控制系统仿真 - LQR/MPC/SMC对比', fontsize=14, fontweight='bold')

    # 角度响应
    ax = axes[0, 0]
    for name in ['LQR', 'MPC', 'SMC']:
        ax.plot(results_step[name]['t'], np.degrees(results_step[name]['theta']),
                label=f'{name}', linewidth=1.5)
    ax.set_xlabel('时间 [s]')
    ax.set_ylabel('角度 [deg]')
    ax.set_title('自由响应 (初始偏角11.5度)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 控制量
    ax = axes[0, 1]
    for name in ['LQR', 'MPC', 'SMC']:
        ax.plot(results_step[name]['t'], results_step[name]['u'],
                label=f'{name}', linewidth=1.5)
    ax.set_xlabel('时间 [s]')
    ax.set_ylabel('控制力 [N]')
    ax.set_title('控制量输出')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 扰动响应
    ax = axes[1, 0]
    for name in ['LQR', 'MPC', 'SMC']:
        ax.plot(results_dist['step'][name]['t'],
                np.degrees(results_dist['step'][name]['theta']),
                label=f'{name}', linewidth=1.2)
    ax.axvline(x=2.0, color='r', linestyle='--', alpha=0.5, label='扰动时刻')
    ax.set_xlabel('时间 [s]')
    ax.set_ylabel('角度 [deg]')
    ax.set_title('阶跃扰动响应')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 性能指标表
    ax = axes[1, 1]
    ax.axis('off')
    headers = ['指标', 'LQR', 'MPC', 'SMC']
    table_data = []
    key_labels = ['最大偏角[deg]', '调节时间[s]', 'IAE', 'ITAE']
    keys = ['max_overshoot', 'settling_time', 'iae', 'itae']

    for label, key in zip(key_labels, keys):
        row = [label]
        for name in ['LQR', 'MPC', 'SMC']:
            val = results_step[name]['metrics'][key]
            if key == 'max_overshoot':
                row.append(f'{np.degrees(val):.2f}')
            else:
                row.append(f'{val:.4f}')
        table_data.append(row)

    table = ax.table(cellText=table_data, colLabels=headers,
                     cellLoc='center', loc='center')
    table.auto_set_font_size(False)
    table.set_fontsize(10)
    table.scale(1.2, 1.5)
    ax.set_title('性能指标对比', pad=20)

    plt.tight_layout()
    plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'inverted_pendulum_comparison.png'),
                dpi=150, bbox_inches='tight')
    print("  图表已保存: inverted_pendulum_comparison.png")

    # === 汇总 ===
    optimal_params = {
        'LQR': {
            'Q': np.diag([opt_lqr_params[0], opt_lqr_params[1]]).tolist(),
            'R': opt_lqr_params[2],
            'metrics': results_step['LQR']['metrics']
        },
        'MPC': {
            'N': 15,
            'Q': np.diag([opt_lqr_params[0], opt_lqr_params[1]]).tolist(),
            'R': opt_lqr_params[2],
            'metrics': results_step['MPC']['metrics']
        },
        'SMC': {
            'c': opt_smc_params[0],
            'eta': opt_smc_params[1],
            'phi': opt_smc_params[2],
            'metrics': results_step['SMC']['metrics']
        }
    }

    print("\n" + "=" * 70)
    print("倒立摆仿真完成! 最优参数汇总:")
    print("=" * 70)
    for name, params in optimal_params.items():
        print(f"\n{name}:")
        for k, v in params.items():
            if k != 'metrics':
                print(f"  {k} = {v}")
        m = params['metrics']
        print(f"  性能: 最大偏角={np.degrees(m['max_overshoot']):.2f}deg, "
              f"调节时间={m['settling_time']:.4f}s, IAE={m['iae']:.4f}")

    return optimal_params


if __name__ == '__main__':
    optimal_params = run_inverted_pendulum_simulation()
    plt.close('all')
