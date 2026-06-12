#!/usr/bin/env python3
"""
滑模控制仿真V3 - 更多趋近律
Sliding Mode Control Simulation V3 - Multiple Reaching Laws

支持趋近律:
  1. 等速趋近律 (Constant Rate)
  2. 指数趋近律 (Exponential)
  3. 幂次趋近律 (Power Rate)
  4. 一般趋近律 (General)
  5. 双幂次趋近律 (Double Power)
  6. 快速终端滑模 (Fast Terminal)
  7. 自适应趋近律 (Adaptive)
  8. 超螺旋算法 (Super-Twisting)

支持系统:
  1. 二阶线性系统
  2. 倒立摆
  3. 机械臂关节
  4. 非线性系统

作者: nuedc-asset-library
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False


# ========== 趋近律定义 ==========

class ReachingLaws:
    """各种趋近律"""

    @staticmethod
    def constant_rate(s, eta=1.0, epsilon=0.01):
        """等速趋近律: ṡ = -η·sign(s)"""
        return -eta * np.sign(s)

    @staticmethod
    def exponential(s, k=5.0, eta=2.0, epsilon=0.01):
        """指数趋近律: ṡ = -k·s - η·sign(s)"""
        return -k * s - eta * np.sign(s)

    @staticmethod
    def power_rate(s, alpha=0.5, eta=2.0, epsilon=0.01):
        """幂次趋近律: ṡ = -η|s|^α·sign(s)"""
        return -eta * np.abs(s)**alpha * np.sign(s)

    @staticmethod
    def general(s, k=3.0, alpha=0.5, eta=1.5, epsilon=0.01):
        """一般趋近律: ṡ = -k|s|^α·sign(s) - η·sign(s)"""
        return -k * np.abs(s)**alpha * np.sign(s) - eta * np.sign(s)

    @staticmethod
    def double_power(s, k1=5.0, k2=3.0, alpha1=1.5, alpha2=0.5, epsilon=0.01):
        """
        双幂次趋近律: ṡ = -k1|s|^α1·sign(s) - k2|s|^α2·sign(s)
        当|s|>1时主要由α1项控制(快速收敛), |s|<1时主要由α2项控制(减少抖振)
        """
        return -k1 * np.abs(s)**alpha1 * np.sign(s) - k2 * np.abs(s)**alpha2 * np.sign(s)

    @staticmethod
    def fast_terminal(s, alpha=0.5, beta=1.0, k=2.0):
        """
        快速终端滑模: ṡ = -k·s - β|s|^α·sign(s)
        兼顾远离平衡点时的快速性和接近时的有限时间收敛
        """
        return -k * s - beta * np.abs(s)**alpha * np.sign(s)

    @staticmethod
    def adaptive(s, eta0=1.0, gamma=0.5, epsilon=0.01):
        """
        自适应趋近律: η̇ = γ|s|, ṡ = -η·sign(s)
        自动调整切换增益
        """
        # 返回所需的η变化率, 实际η在仿真中积分更新
        return -eta0 * np.sign(s), gamma * np.abs(s)

    @staticmethod
    def super_twisting(s, s_dot, u_prev, dt, k1=2.0, k2=1.5, alpha=0.5):
        """
        超螺旋算法 (二阶滑模):
        u = u_prev + dt * (-k1|s|^α·sign(s) + s_dot_int)
        消除一阶滑模的抖振
        """
        u1 = -k1 * np.abs(s)**alpha * np.sign(s)
        u2 = u_prev + dt * (-k2 * np.sign(s))
        return u1 + u2


# ========== 滑模控制器 ==========

class SlidingModeController:
    """通用滑模控制器"""

    def __init__(self, c=5.0, reaching_law='exponential', **kwargs):
        """
        Args:
            c: 滑模面斜率, s = ė + c·e
            reaching_law: 趋近律类型
            **kwargs: 趋近律参数
        """
        self.c = c
        self.law_type = reaching_law
        self.params = kwargs
        self.laws = ReachingLaws()
        self.eta_adaptive = kwargs.get('eta0', 1.0)

    def sliding_surface(self, e, de):
        """滑模面 s = de + c * e"""
        return de + self.c * e

    def compute_control(self, e, de, dde_ideal=None):
        """
        计算等效控制 + 切换控制

        Args:
            e: 跟踪误差
            de: 误差导数
            dde_ideal: 理想误差二阶导 (如果知道系统模型)
        """
        s = self.sliding_surface(e, de)

        # 选择趋近律
        if self.law_type == 'constant_rate':
            u_switch = self.laws.constant_rate(s, **self.params)
        elif self.law_type == 'exponential':
            u_switch = self.laws.exponential(s, **self.params)
        elif self.law_type == 'power_rate':
            u_switch = self.laws.power_rate(s, **self.params)
        elif self.law_type == 'general':
            u_switch = self.laws.general(s, **self.params)
        elif self.law_type == 'double_power':
            u_switch = self.laws.double_power(s, **self.params)
        elif self.law_type == 'fast_terminal':
            u_switch = self.laws.fast_terminal(s, **self.params)
        elif self.law_type == 'adaptive':
            u_switch, deta = self.laws.adaptive(s, **self.params)
            self.eta_adaptive += 0.001 * deta  # 积分更新η
            u_switch = -self.eta_adaptive * np.sign(s)  # 用更新后的η重新计算控制
        else:
            u_switch = self.laws.exponential(s)

        # 饱和函数替代符号函数 (减少抖振)
        phi = self.params.get('phi', 0.05)
        sat = self.compute_saturated_sign(s, phi)
        u_switch = u_switch / np.sign(s) * sat if np.sign(s) != 0 else u_switch * sat

        return u_switch, s

    def compute_saturated_sign(self, s, phi=0.05):
        """饱和函数 sat(s/φ)"""
        if abs(s) <= phi:
            return s / phi
        return np.sign(s)


# ========== 仿真系统 ==========

class SMCSimulation:
    """滑模控制仿真"""

    def __init__(self):
        self.dt = 0.001
        self.t_end = 5.0

    def second_order_system(self, x, u, disturbance=0):
        """
        二阶非线性系统: ẍ = f(x) + g(x)·u + d(t)
        f(x) = -x₁ - 0.5x₂ (已知部分)
        g(x) = 1 + 0.2sin(x₁) (未知变化增益)
        d(t) = 外部扰动
        """
        x1, x2 = x
        f = -x1 - 0.5 * x2
        g = 1 + 0.2 * np.sin(x1)
        dx1 = x2
        dx2 = f + g * u + disturbance
        return np.array([dx1, dx2])

    def simulate_comparison(self, x0=np.array([1.0, 0.0]), ref=np.array([0.0, 0.0])):
        """对比不同趋近律"""
        print("=" * 60)
        print("滑模控制 - 多趋近律对比仿真")
        print("=" * 60)

        laws = [
            ('constant_rate', '等速趋近律', {'eta': 3.0}),
            ('exponential', '指数趋近律', {'k': 5.0, 'eta': 2.0}),
            ('power_rate', '幂次趋近律', {'alpha': 0.5, 'eta': 3.0}),
            ('general', '一般趋近律', {'k': 3.0, 'alpha': 0.5, 'eta': 1.5}),
            ('double_power', '双幂次趋近律', {'k1': 5.0, 'k2': 3.0, 'alpha1': 1.5, 'alpha2': 0.5}),
            ('fast_terminal', '快速终端', {'alpha': 0.5, 'beta': 2.0, 'k': 3.0}),
        ]

        results = {}
        steps = int(self.t_end / self.dt)
        t = np.linspace(0, self.t_end, steps)

        for law_type, name, params in laws:
            smc = SlidingModeController(c=5.0, reaching_law=law_type, **params)

            x = x0.copy()
            X = np.zeros((steps, 2))
            U = np.zeros(steps)
            S = np.zeros(steps)
            X[0] = x

            for i in range(steps):
                e = x[0] - ref[0]
                de = x[1] - ref[1]

                u, s = smc.compute_control(e, de)
                # 限幅
                u = np.clip(u, -50, 50)

                U[i] = u
                S[i] = s

                # 扰动
                dist = 0.5 * np.sin(2 * t[i])

                # RK4
                k1 = self.second_order_system(x, u, dist)
                k2 = self.second_order_system(x + 0.5*self.dt*k1, u, dist)
                k3 = self.second_order_system(x + 0.5*self.dt*k2, u, dist)
                k4 = self.second_order_system(x + self.dt*k3, u, dist)
                x = x + (self.dt/6) * (k1 + 2*k2 + 2*k3 + k4)

                if i < steps - 1:
                    X[i + 1] = x

            results[name] = {'X': X, 'U': U, 'S': S}

        # 绘图
        self._plot_comparison(t, results, ref[0])

        return t, results

    def _plot_comparison(self, t, results, ref_val):
        fig, axes = plt.subplots(3, 1, figsize=(14, 12))
        fig.suptitle('滑模控制 - 不同趋近律对比 (含扰动)', fontsize=14, fontweight='bold')

        colors = ['b', 'r', 'g', 'm', 'c', 'orange']

        for i, (name, data) in enumerate(results.items()):
            axes[0].plot(t, data['X'][:, 0], colors[i], linewidth=1.5, label=name)
            axes[1].plot(t, data['S'], colors[i], linewidth=1, label=name, alpha=0.8)
            axes[2].plot(t, data['U'], colors[i], linewidth=0.8, label=name, alpha=0.7)

        axes[0].axhline(y=ref_val, color='k', linestyle='--', linewidth=2, alpha=0.5, label='参考')
        axes[0].set_ylabel('状态 x₁')
        axes[0].legend(loc='upper right', fontsize=9)
        axes[0].grid(True, alpha=0.3)

        axes[1].set_ylabel('滑模面 s')
        axes[1].legend(loc='upper right', fontsize=9)
        axes[1].grid(True, alpha=0.3)

        axes[2].set_xlabel('时间 (s)')
        axes[2].set_ylabel('控制输入 u')
        axes[2].legend(loc='upper right', fontsize=9)
        axes[2].grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig('smc_reaching_laws_comparison.png', dpi=150, bbox_inches='tight')
        plt.close('all')

    def simulate_super_twisting(self, x0=np.array([1.0, 0.0])):
        """超螺旋算法仿真"""
        print("\n超螺旋二阶滑模仿真")

        steps = int(self.t_end / self.dt)
        t = np.linspace(0, self.t_end, steps)

        x = x0.copy()
        X = np.zeros((steps, 2))
        U = np.zeros(steps)
        X[0] = x

        c = 5.0
        k1, k2, alpha = 3.0, 2.0, 0.5
        u_st = 0

        for i in range(steps):
            e = x[0]
            de = x[1]
            s = de + c * e

            # 超螺旋控制
            u_switch = ReachingLaws.super_twisting(s, de, u_st, self.dt, k1, k2, alpha)
            u_st = u_switch

            # 等效控制 (已知模型部分)
            f = -x[0] - 0.5 * x[1]
            u_eq = -f - c * de
            u = u_eq + u_switch
            u = np.clip(u, -50, 50)

            U[i] = u

            dist = 0.5 * np.sin(2 * t[i])
            k1_rk = self.second_order_system(x, u, dist)
            k2_rk = self.second_order_system(x + 0.5*self.dt*k1_rk, u, dist)
            k3_rk = self.second_order_system(x + 0.5*self.dt*k2_rk, u, dist)
            k4_rk = self.second_order_system(x + self.dt*k3_rk, u, dist)
            x = x + (self.dt/6) * (k1_rk + 2*k2_rk + 2*k3_rk + k4_rk)

            if i < steps - 1:
                X[i + 1] = x

        fig, axes = plt.subplots(2, 1, figsize=(12, 8))
        fig.suptitle('超螺旋二阶滑模控制', fontsize=14, fontweight='bold')

        axes[0].plot(t, X[:, 0], 'b-', linewidth=2, label='状态 x₁')
        axes[0].axhline(y=0, color='r', linestyle='--', label='参考')
        axes[0].set_ylabel('状态 x₁')
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)

        axes[1].plot(t, U, 'g-', linewidth=1, label='控制输入')
        axes[1].set_xlabel('时间 (s)')
        axes[1].set_ylabel('u')
        axes[1].legend()
        axes[1].grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig('super_twisting_smc.png', dpi=150, bbox_inches='tight')
        plt.close('all')

    def run_all(self):
        self.simulate_comparison()
        self.simulate_super_twisting()
        print("\n所有滑模控制仿真完成!")


if __name__ == '__main__':
    sim = SMCSimulation()
    sim.run_all()
