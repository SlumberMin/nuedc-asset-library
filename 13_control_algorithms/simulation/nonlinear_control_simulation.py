#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
非线性控制仿真 (Nonlinear Control Simulation)

本脚本仿真以下非线性控制算法:
1. 反馈线性化 (Feedback Linearization)
2. 反步法 (Backstepping)
3. 滑模控制 (Sliding Mode Control)
4. Lyapunov直接法控制器设计

对比各算法在非线性系统上的控制效果。

Author: nuedc-asset-library
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import rcParams

rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
rcParams['axes.unicode_minus'] = False


class InvertedPendulum:
    """倒立摆系统
    状态: x = [θ, θ_dot]
    动力学: ml²θ'' = mgl*sin(θ) - b*θ' + u
    即: θ'' = (g/l)*sin(θ) - (b/(ml²))*θ' + u/(ml²)
    """

    def __init__(self, m=1.0, l=1.0, g=9.81, b=0.1):
        self.m = m
        self.l = l
        self.g = g
        self.b = b
        self.x = np.array([0.3, 0.0])  # 初始偏角0.3rad

    def dynamics(self, x, u):
        """状态导数"""
        theta, theta_dot = x
        dtheta = theta_dot
        dtheta_dot = (self.g / self.l) * np.sin(theta) \
                    - (self.b / (self.m * self.l**2)) * theta_dot \
                    + u / (self.m * self.l**2)
        return np.array([dtheta, dtheta_dot])

    def update(self, u, dt):
        """RK4积分"""
        k1 = self.dynamics(self.x, u)
        k2 = self.dynamics(self.x + 0.5 * dt * k1, u)
        k3 = self.dynamics(self.x + 0.5 * dt * k2, u)
        k4 = self.dynamics(self.x + dt * k3, u)
        self.x = self.x + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
        return self.x.copy()

    def reset(self, x0=None):
        self.x = x0 if x0 is not None else np.array([0.3, 0.0])


class DuffingOscillator:
    """Duffing振子
    ẍ + δẋ - αx + βx³ = u
    典型参数产生混沌行为,用于测试非线性控制
    """

    def __init__(self, delta=0.3, alpha=1.0, beta=1.0):
        self.delta = delta
        self.alpha = alpha
        self.beta = beta
        self.x = np.array([0.5, 0.0])

    def dynamics(self, x, u):
        x1, x2 = x
        dx1 = x2
        dx2 = -self.delta * x2 + self.alpha * x1 - self.beta * x1**3 + u
        return np.array([dx1, dx2])

    def update(self, u, dt):
        k1 = self.dynamics(self.x, u)
        k2 = self.dynamics(self.x + 0.5 * dt * k1, u)
        k3 = self.dynamics(self.x + 0.5 * dt * k2, u)
        k4 = self.dynamics(self.x + dt * k3, u)
        self.x = self.x + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
        return self.x.copy()

    def reset(self, x0=None):
        self.x = x0 if x0 is not None else np.array([0.5, 0.0])


class FeedbackLinearizationController:
    """反馈线性化控制器 (倒立摆)

    将倒立摆线性化为二阶积分器:
    y = θ, y'' = f(x) + g(x)*u
    f(x) = (g/l)*sin(θ) - (b/(ml²))*θ'
    g(x) = 1/(ml²)
    令 v = -k1*(θ - θ_ref) - k2*θ'
    u = ml² * (v - (g/l)*sin(θ) + (b/(ml²))*θ')
    """

    def __init__(self, m=1.0, l=1.0, g=9.81, b=0.1):
        self.m = m
        self.l = l
        self.g = g
        self.b = b

        # 闭环极点配置 (期望快速衰减)
        wn = 5.0    # 自然频率
        zeta = 0.9  # 阻尼比
        self.k1 = wn**2          # 位置增益
        self.k2 = 2 * zeta * wn  # 速度增益

        self.u_max = 100.0

    def update(self, x, r):
        """
        x: [θ, θ_dot]
        r: 期望角度
        """
        theta, theta_dot = x

        # 误差
        e = theta - r
        e_dot = theta_dot

        # 线性控制 (虚拟控制量)
        v = -self.k1 * e - self.k2 * e_dot

        # 反馈线性化: u = ml² * (v - f(x)) / g(x)
        # 这里 g(x) = 1/(ml²), 所以 u = ml²*v - ml²*f(x)
        ml2 = self.m * self.l**2
        f_x = (self.g / self.l) * np.sin(theta) \
            - (self.b / ml2) * theta_dot

        u = ml2 * (v - f_x)
        u = np.clip(u, -self.u_max, self.u_max)

        return u


class BacksteppingController:
    """反步法控制器 (Backstepping)

    适用于严格反馈形式:
    ẋ₁ = f₁(x₁) + g₁(x₁)x₂
    ẋ₂ = f₂(x₁,x₂) + g₂(x₁,x₂)u

    设计步骤:
    Step 1: 定义 z₁ = x₁ - x₁d, 选择虚拟控制 α₁
    Step 2: 定义 z₂ = x₂ - α₁, 设计真实控制 u
    """

    def __init__(self, m=1.0, l=1.0, g=9.81, b=0.1):
        self.m = m
        self.l = l
        self.g = g
        self.b = b

        # 反步增益
        self.c1 = 3.0  # 第一步Lyapunov增益
        self.c2 = 5.0  # 第二步Lyapunov增益

        self.u_max = 100.0

    def update(self, x, r):
        """
        x: [θ, θ_dot]
        r: 期望角度
        """
        theta, theta_dot = x
        ml2 = self.m * self.l**2

        # Step 1: z₁ = θ - r, 虚拟控制 α₁
        z1 = theta - r
        # α₁ = -c₁*z₁ (虚拟控制律, 即期望的 θ_dot)
        alpha1 = -self.c1 * z1

        # Step 2: z₂ = θ_dot - α₁
        z2 = theta_dot - alpha1

        # 系统函数
        f1 = 0.0  # ẋ₁ = x₂ (即 f₁ = 0, g₁ = 1)
        f2 = (self.g / self.l) * np.sin(theta) - (self.b / ml2) * theta_dot
        g2 = 1.0 / ml2

        # d(alpha1)/dt ≈ -c₁ * z1_dot = -c₁ * theta_dot (简化)
        dalpha1_dt = -self.c1 * theta_dot

        # 控制律 (从Lyapunov稳定性分析推导)
        # V = 0.5*z₁² + 0.5*z₂²
        # dV/dt = z₁*z₁_dot + z₂*z₂_dot
        #       = z₁*(z₂ + alpha1) + z₂*(f₂ + g₂*u - dalpha1/dt)
        #       = z₁*z₂ - c₁*z₁² + z₂*(f₂ + g₂*u - dalpha1/dt + z₁)
        # 令 dV/dt < 0: f₂ + g₂*u - dalpha1/dt + z₁ = -c₂*z₂
        # => u = (-c₂*z₂ - z₁ - f₂ + dalpha1/dt) / g2

        u = (-self.c2 * z2 - z1 - f2 + dalpha1_dt) / g2
        u = np.clip(u, -self.u_max, self.u_max)

        return u


class SlidingModeController:
    """滑模控制器 (Sliding Mode Control)

    设计滑模面: s = c*e + e_dot, 其中 e = x₁ - x₁d
    控制律: u = u_eq + u_sw
    u_eq: 等效控制 (保持在滑模面上)
    u_sw: 切换控制 (到达滑模面)

    使用饱和函数替代符号函数以减轻抖振
    """

    def __init__(self, m=1.0, l=1.0, g=9.81, b=0.1):
        self.m = m
        self.l = l
        self.g = g
        self.b = b

        # 滑模参数
        self.c = 5.0          # 滑模面斜率
        self.eta = 3.0        # 切换增益
        self.phi = 0.1        # 边界层厚度 (饱和函数)
        self.lambda_smc = 2.0 # 指数趋近律

        self.u_max = 100.0

    def sat(self, s):
        """饱和函数(替代符号函数)"""
        if abs(s) <= self.phi:
            return s / self.phi
        return np.sign(s)

    def update(self, x, r, r_dot=0.0):
        """
        x: [θ, θ_dot]
        r: 期望角度
        r_dot: 期望角速度
        """
        theta, theta_dot = x
        ml2 = self.m * self.l**2

        # 误差
        e = theta - r
        e_dot = theta_dot - r_dot

        # 滑模面
        s = self.c * e + e_dot

        # 系统函数
        f_x = (self.g / self.l) * np.sin(theta) - (self.b / ml2) * theta_dot
        g_x = 1.0 / ml2

        # 等效控制 (令 ds/dt = 0 不考虑不确定性)
        # s_dot = c*e_dot + e_ddot = c*e_dot + f_x + g_x*u - r_ddot
        # 令 s_dot = 0: u_eq = (-c*e_dot - f_x + r_ddot) / g_x
        u_eq = (-self.c * e_dot - f_x) / g_x

        # 切换控制 (趋近律: s_dot = -eta*sat(s) - lambda*s)
        u_sw = (-self.eta * self.sat(s) - self.lambda_smc * s) / g_x

        u = u_eq + u_sw
        u = np.clip(u, -self.u_max, self.u_max)

        return u, s


class LyapunovDirectController:
    """Lyapunov直接法控制器

    不经过线性化,直接基于Lyapunov稳定性理论设计控制律。
    选择Lyapunov函数 V(x),保证 dV/dt < 0。

    对于倒立摆: V = 0.5*ml²*θ'² + mgl*(1-cos(θ)) + 0.5*k*θ²
    dV/dt = θ'*(ml²*θ'' - mgl*sin(θ)) + k*θ*θ'
          = θ'*(u - b*θ' + k*θ) < 0
    选择: u = -(k₁+k)*θ - (k₂+b)*θ'
    """

    def __init__(self, m=1.0, l=1.0, g=9.81, b=0.1):
        self.m = m
        self.l = l
        self.g = g
        self.b = b

        # Lyapunov增益
        self.k_pos = 10.0   # 位置增益
        self.k_vel = 5.0    # 速度增益

        self.u_max = 100.0

    def update(self, x, r):
        theta, theta_dot = x
        ml2 = self.m * self.l**2

        e = theta - r
        e_dot = theta_dot

        # Lyapunov控制律
        # u = -(k_pos)*e - (k_vel + b/(ml²))*θ_dot + mgl/l*sin(θ)
        # 补偿非线性项 + 注入阻尼
        u = -self.k_pos * e * ml2 \
            - (self.k_vel + self.b / ml2) * theta_dot * ml2 \
            + self.m * self.g * self.l * np.sin(theta)

        u = np.clip(u, -self.u_max, self.u_max)
        return u


def run_simulation():
    """运行非线性控制仿真"""
    print("=" * 60)
    print("非线性控制仿真 (Nonlinear Control Simulation)")
    print("=" * 60)

    dt = 0.001
    T = 10.0
    N = int(T / dt)
    t = np.linspace(0, T, N)

    # 期望轨迹 (正弦参考)
    r = 0.5 * np.sin(2 * np.pi * 0.5 * t)

    # ====== 1. 反馈线性化 ======
    print("\n[1] 仿真 反馈线性化 (Feedback Linearization)...")
    plant1 = InvertedPendulum(m=1.0, l=1.0, g=9.81, b=0.1)
    plant1.reset(np.array([0.5, 0.0]))
    ctrl1 = FeedbackLinearizationController(m=1.0, l=1.0, g=9.81, b=0.1)

    y_fl = np.zeros(N)
    u_fl = np.zeros(N)
    for i in range(N):
        y_fl[i] = plant1.x[0]
        u_fl[i] = ctrl1.update(plant1.x, r[i])
        plant1.update(u_fl[i], dt)

    fl_error = np.mean(np.abs(y_fl - r))
    print(f"  反馈线性化 平均跟踪误差: {fl_error:.6f}")

    # ====== 2. 反步法 ======
    print("\n[2] 仿真 反步法 (Backstepping)...")
    plant2 = InvertedPendulum(m=1.0, l=1.0, g=9.81, b=0.1)
    plant2.reset(np.array([0.5, 0.0]))
    ctrl2 = BacksteppingController(m=1.0, l=1.0, g=9.81, b=0.1)

    y_bs = np.zeros(N)
    u_bs = np.zeros(N)
    for i in range(N):
        y_bs[i] = plant2.x[0]
        u_bs[i] = ctrl2.update(plant2.x, r[i])
        plant2.update(u_bs[i], dt)

    bs_error = np.mean(np.abs(y_bs - r))
    print(f"  反步法 平均跟踪误差: {bs_error:.6f}")

    # ====== 3. 滑模控制 ======
    print("\n[3] 仿真 滑模控制 (Sliding Mode Control)...")
    plant3 = InvertedPendulum(m=1.0, l=1.0, g=9.81, b=0.1)
    plant3.reset(np.array([0.5, 0.0]))
    ctrl3 = SlidingModeController(m=1.0, l=1.0, g=9.81, b=0.1)

    y_smc = np.zeros(N)
    u_smc = np.zeros(N)
    s_smc = np.zeros(N)
    for i in range(N):
        y_smc[i] = plant3.x[0]
        u_val, s_val = ctrl3.update(plant3.x, r[i])
        u_smc[i] = u_val
        s_smc[i] = s_val
        plant3.update(u_val, dt)

    smc_error = np.mean(np.abs(y_smc - r))
    print(f"  滑模控制 平均跟踪误差: {smc_error:.6f}")

    # ====== 4. Lyapunov直接法 ======
    print("\n[4] 仿真 Lyapunov直接法...")
    plant4 = InvertedPendulum(m=1.0, l=1.0, g=9.81, b=0.1)
    plant4.reset(np.array([0.5, 0.0]))
    ctrl4 = LyapunovDirectController(m=1.0, l=1.0, g=9.81, b=0.1)

    y_lf = np.zeros(N)
    u_lf = np.zeros(N)
    for i in range(N):
        y_lf[i] = plant4.x[0]
        u_lf[i] = ctrl4.update(plant4.x, r[i])
        plant4.update(u_lf[i], dt)

    lf_error = np.mean(np.abs(y_lf - r))
    print(f"  Lyapunov直接法 平均跟踪误差: {lf_error:.6f}")

    # ====== 5. Duffing振子控制 (反馈线性化) ======
    print("\n[5] 仿真 Duffing振子 + 反馈线性化...")
    plant_duff = DuffingOscillator(delta=0.3, alpha=1.0, beta=1.0)
    plant_duff.reset(np.array([1.0, 0.0]))

    r_duff = np.zeros(N)  # 目标: 原点

    y_duff = np.zeros(N)
    u_duff = np.zeros(N)

    # 简单反馈线性化用于Duffing
    k1_duff, k2_duff = 4.0, 4.0
    for i in range(N):
        y_duff[i] = plant_duff.x[0]
        x1, x2 = plant_duff.x

        # 反馈线性化: u = v - f(x), v = -k1*x1 - k2*x2
        f_x = -0.3 * x2 + 1.0 * x1 - 1.0 * x1**3
        v = -k1_duff * x1 - k2_duff * x2
        u_val = v - f_x
        u_val = np.clip(u_val, -50.0, 50.0)
        u_duff[i] = u_val

        plant_duff.update(u_val, dt)

    duff_error = np.mean(np.abs(y_duff))
    print(f"  Duffing振子 最终偏差: {y_duff[-1]:.6f}")

    # ====== 绘图 ======
    print("\n生成仿真图表...")

    fig, axes = plt.subplots(3, 2, figsize=(16, 14))

    # 跟踪效果对比
    ax = axes[0, 0]
    ax.plot(t, r, 'k--', alpha=0.5, label='参考 r(t)')
    ax.plot(t, y_fl, 'b-', linewidth=1.0, label=f'反馈线性化 (err={fl_error:.4f})')
    ax.plot(t, y_bs, 'r-', linewidth=1.0, label=f'反步法 (err={bs_error:.4f})')
    ax.set_title('反馈线性化 vs 反步法 跟踪效果')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('角度 (rad)')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[0, 1]
    ax.plot(t, r, 'k--', alpha=0.5, label='参考 r(t)')
    ax.plot(t, y_smc, 'g-', linewidth=1.0, label=f'滑模控制 (err={smc_error:.4f})')
    ax.plot(t, y_lf, 'm-', linewidth=1.0, label=f'Lyapunov法 (err={lf_error:.4f})')
    ax.set_title('滑模控制 vs Lyapunov直接法 跟踪效果')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('角度 (rad)')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # 控制量对比
    ax = axes[1, 0]
    ax.plot(t, u_fl, 'b-', alpha=0.7, label='反馈线性化')
    ax.plot(t, u_bs, 'r-', alpha=0.7, label='反步法')
    ax.set_title('控制量对比 (反馈线性化 vs 反步法)')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('控制力矩 u')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[1, 1]
    ax.plot(t, u_smc, 'g-', alpha=0.7, label='滑模控制')
    ax.plot(t, u_lf, 'm-', alpha=0.7, label='Lyapunov法')
    ax.set_title('控制量对比 (滑模 vs Lyapunov)')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('控制力矩 u')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # 滑模面
    ax = axes[2, 0]
    ax.plot(t, s_smc, 'g-', linewidth=0.8)
    ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax.set_title('滑模面 s(t)')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('s')
    ax.grid(True, alpha=0.3)

    # Duffing振子
    ax = axes[2, 1]
    ax.plot(t, y_duff, 'b-', linewidth=1.0, label='Duffing振子 x₁')
    ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax.set_title('Duffing振子 反馈线性化控制')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('状态 x₁')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    plt.suptitle('非线性控制算法仿真对比', fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig('nonlinear_control_simulation.png', dpi=150, bbox_inches='tight')
    print("图表已保存: nonlinear_control_simulation.png")
    plt.close('all')


if __name__ == '__main__':
    run_simulation()
