#!/usr/bin/env python3
"""
反馈线性化仿真 - 非线性系统控制
Feedback Linearization Simulation for Nonlinear Systems

支持系统:
  1. 单摆系统 (Pendulum)
  2. 倒立摆 (Inverted Pendulum on Cart)
  3. Van der Pol 振荡器
  4. 双积分器 + 非线性项

作者: nuedc-asset-library
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False


class FeedbackLinearizationSimulator:
    """反馈线性化仿真器"""

    def __init__(self):
        self.dt = 0.001
        self.t_end = 10.0

    # ========== 非线性系统定义 ==========

    @staticmethod
    def pendulum_dynamics(state, u, m=1.0, l=1.0, g=9.81, b=0.1):
        """
        单摆: ml²θ̈ + bθ̇ + mgl sin(θ) = u
        状态: [θ, θ̇]
        """
        theta, omega = state
        dtheta = omega
        domega = (u - b * omega - m * g * l * np.sin(theta)) / (m * l**2)
        return np.array([dtheta, domega])

    @staticmethod
    def inverted_pendulum_dynamics(state, u, M=1.0, m=0.1, l=0.5, g=9.81):
        """
        倒立摆(简化):
        (M+m)ẍ - mlθ̈cos(θ) + mlθ̇²sin(θ) = u
        ml²θ̈ - mlẍcos(θ) - mglsin(θ) = 0
        状态: [x, ẋ, θ, θ̇]
        """
        x, dx, theta, omega = state
        ct, st = np.cos(theta), np.sin(theta)
        det = (M + m) * m * l**2 - (m * l * ct)**2

        ddx = (m * l**2 * (m * l * omega**2 * st + u) - m * l * ct * (-m * g * l * st)) / det
        domega = ((M + m) * (-m * g * l * st) - m * l * ct * (m * l * omega**2 * st + u)) / (-det)

        return np.array([dx, ddx, omega, domega])

    @staticmethod
    def vanderpol_dynamics(state, u, mu=1.0):
        """
        Van der Pol: ẍ - μ(1-x²)ẋ + x = u
        状态: [x, ẋ]
        """
        x, dx = state
        ddx = mu * (1 - x**2) * dx - x + u
        return np.array([dx, ddx])

    @staticmethod
    def nonlinear_double_integrator(state, u, alpha=0.5):
        """
        非线性双积分器: ẍ + αx³ = u
        状态: [x, ẋ]
        """
        x, dx = state
        ddx = u - alpha * x**3
        return np.array([dx, ddx])

    # ========== 反馈线性化控制器 ==========

    def pendulum_feedback_linearization(self, state, state_d, m=1.0, l=1.0, g=9.81, b=0.1):
        """
        单摆反馈线性化
        设计: v = θ̈_d + k1(θ_d - θ) + k2(θ̇_d - θ̇)
        实际: u = ml²v + bθ̇ + mgl sin(θ)
        """
        theta, omega = state
        theta_d, omega_d = state_d

        k1, k2 = 100, 20  # 期望闭环极点
        v = 0 + k1 * (theta_d - theta) + k2 * (omega_d - omega)
        u = m * l**2 * v + b * omega + m * g * l * np.sin(theta)
        return u

    def vanderpol_feedback_linearization(self, state, state_d, mu=1.0):
        """
        Van der Pol 反馈线性化
        v = ẍ_d + k1(x_d - x) + k2(ẋ_d - ẋ)
        u = v - μ(1-x²)ẋ - x  →  实际 u = v - μ(1-x²)ẋ + x (抵消非线性)
        """
        x, dx = state
        x_d, dx_d = state_d

        k1, k2 = 64, 16
        v = k1 * (x_d - x) + k2 * (dx_d - dx)
        u = v - mu * (1 - x**2) * dx + x
        return u

    def ndi_feedback_linearization(self, state, state_d, alpha=0.5):
        """
        非线性双积分器反馈线性化
        u = v + αx³  (抵消非线性)
        """
        x, dx = state
        x_d, dx_d = state_d

        k1, k2 = 100, 20
        v = k1 * (x_d - x) + k2 * (dx_d - dx)
        u = v + alpha * x**3
        return u

    # ========== 仿真引擎 ==========

    def simulate(self, dynamics, controller, x0, ref_func, t_end=None):
        """通用仿真"""
        t_end = t_end or self.t_end
        steps = int(t_end / self.dt)
        t = np.linspace(0, t_end, steps)

        states = np.zeros((steps, len(x0)))
        controls = np.zeros(steps)
        references = np.zeros((steps, len(x0)))

        states[0] = x0

        for i in range(steps):
            ref = ref_func(t[i])
            references[i] = ref
            u = controller(states[i], ref)
            controls[i] = u

            if i < steps - 1:
                # RK4积分
                k1 = dynamics(states[i], u)
                k2 = dynamics(states[i] + 0.5 * self.dt * k1, u)
                k3 = dynamics(states[i] + 0.5 * self.dt * k2, u)
                k4 = dynamics(states[i] + self.dt * k3, u)
                states[i + 1] = states[i] + (self.dt / 6) * (k1 + 2*k2 + 2*k3 + k4)

        return t, states, controls, references

    # ========== 仿真案例 ==========

    def run_pendulum(self):
        """单摆反馈线性化"""
        print("=" * 60)
        print("单摆系统反馈线性化仿真")
        print("=" * 60)

        x0 = np.array([0.5, 0.0])  # 初始角度0.5rad
        ref_func = lambda t: np.array([np.pi / 4, 0.0])  # 目标 π/4

        t, states, controls, refs = self.simulate(
            lambda s, u: self.pendulum_dynamics(s, u),
            lambda s, r: self.pendulum_feedback_linearization(s, r),
            x0, ref_func
        )
        self._plot_single("单摆反馈线性化", t, states, controls, refs,
                          ['角度 θ (rad)', '角速度 θ̇ (rad/s)'])
        return t, states, controls

    def run_vanderpol(self):
        """Van der Pol 反馈线性化"""
        print("Van der Pol 振荡器反馈线性化仿真")

        x0 = np.array([2.0, 0.0])
        ref_func = lambda t: np.array([1.0, 0.0])

        t, states, controls, refs = self.simulate(
            lambda s, u: self.vanderpol_dynamics(s, u),
            lambda s, r: self.vanderpol_feedback_linearization(s, r),
            x0, ref_func, t_end=8.0
        )
        self._plot_single("Van der Pol 反馈线性化", t, states, controls, refs,
                          ['位置 x', '速度 ẋ'])
        return t, states, controls

    def run_ndi(self):
        """非线性双积分器反馈线性化"""
        print("非线性双积分器反馈线性化仿真")

        x0 = np.array([0.0, 0.0])
        ref_func = lambda t: np.array([np.sin(2 * np.pi * 0.5 * t), 0.0])  # 正弦跟踪

        t, states, controls, refs = self.simulate(
            lambda s, u: self.nonlinear_double_integrator(s, u),
            lambda s, r: self.ndi_feedback_linearization(s, r),
            x0, ref_func
        )
        self._plot_tracking("非线性双积分器 - 正弦跟踪", t, states, controls, refs,
                            ['位置 x', '速度 ẋ'])
        return t, states, controls

    def run_comparison(self):
        """线性化 vs 未线性化对比"""
        print("反馈线性化 vs 简单PD控制对比")

        x0 = np.array([1.0, 0.0])
        ref_func = lambda t: np.array([0.0, 0.0])

        # 反馈线性化
        t1, s1, c1, _ = self.simulate(
            lambda s, u: self.nonlinear_double_integrator(s, u),
            lambda s, r: self.ndi_feedback_linearization(s, r),
            x0, ref_func
        )

        # 纯PD控制 (不补偿非线性)
        def pd_controller(state, ref):
            x, dx = state
            x_d, dx_d = ref
            k1, k2 = 100, 20
            return k1 * (x_d - x) + k2 * (dx_d - dx)

        t2, s2, c2, _ = self.simulate(
            lambda s, u: self.nonlinear_double_integrator(s, u),
            pd_controller,
            x0, ref_func
        )

        fig, axes = plt.subplots(2, 1, figsize=(12, 8))
        fig.suptitle('反馈线性化 vs 纯PD控制 (αx³非线性)', fontsize=14, fontweight='bold')

        axes[0].plot(t1, s1[:, 0], 'b-', linewidth=2, label='反馈线性化')
        axes[0].plot(t2, s2[:, 0], 'r--', linewidth=2, label='纯PD控制')
        axes[0].axhline(y=0, color='k', linestyle=':', alpha=0.3)
        axes[0].set_ylabel('位置 x')
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)

        axes[1].plot(t1, c1, 'b-', linewidth=1.5, label='反馈线性化 u')
        axes[1].plot(t2, c2, 'r--', linewidth=1.5, label='纯PD u')
        axes[1].set_xlabel('时间 (s)')
        axes[1].set_ylabel('控制输入 u')
        axes[1].legend()
        axes[1].grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig('feedback_linearization_comparison.png', dpi=150, bbox_inches='tight')
        plt.close('all')

    # ========== 绘图 ==========

    def _plot_single(self, title, t, states, controls, refs, labels):
        fig = plt.figure(figsize=(14, 10))
        gs = GridSpec(3, 2, figure=fig)
        fig.suptitle(title, fontsize=14, fontweight='bold')

        for i, label in enumerate(labels):
            ax = fig.add_subplot(gs[i, 0])
            ax.plot(t, states[:, i], 'b-', linewidth=1.5, label='实际')
            ax.plot(t, refs[:, i], 'r--', linewidth=1.5, label='参考')
            ax.set_ylabel(label)
            ax.legend()
            ax.grid(True, alpha=0.3)

        ax = fig.add_subplot(gs[:, 1])
        ax.plot(t, controls, 'g-', linewidth=1)
        ax.set_xlabel('时间 (s)')
        ax.set_ylabel('控制输入 u')
        ax.set_title('控制信号')
        ax.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig(f'{title}.png', dpi=150, bbox_inches='tight')
        plt.close('all')

    def _plot_tracking(self, title, t, states, controls, refs, labels):
        fig, axes = plt.subplots(2, 1, figsize=(12, 8))
        fig.suptitle(title, fontsize=14, fontweight='bold')

        axes[0].plot(t, states[:, 0], 'b-', linewidth=2, label='实际位置')
        axes[0].plot(t, refs[:, 0], 'r--', linewidth=2, label='参考')
        axes[0].set_ylabel(labels[0])
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)

        axes[1].plot(t, controls, 'g-', linewidth=1)
        axes[1].set_xlabel('时间 (s)')
        axes[1].set_ylabel('控制输入 u')
        axes[1].grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig(f'{title}.png', dpi=150, bbox_inches='tight')
        plt.close('all')

    def run_all(self):
        """运行所有仿真"""
        self.run_pendulum()
        self.run_vanderpol()
        self.run_ndi()
        self.run_comparison()
        print("\n所有反馈线性化仿真完成!")


if __name__ == '__main__':
    sim = FeedbackLinearizationSimulator()
    sim.run_all()
