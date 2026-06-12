"""
非线性摆仿真 - 大角度摆动
============================
完整非线性摆方程: (不进行小角度近似)
  mL²θ̈ + mgL·sin(θ) + b·θ̈ = τ

无量纲形式:
  θ̈ + (b/mL)·θ̇ + (g/L)·sin(θ) = τ/(mL²)

特性:
1. 小角度: sin(θ) ≈ θ, 线性化为经典简谐运动
2. 大角度: 非线性效应显著, 周期随振幅增大
3. 高能量: 可以翻转 (θ > π)
4. 临界能量: 异宿轨道, 混沌行为

对比:
- 线性化模型 vs 完整非线性模型
- 不同初始角度下的行为差异
- 能量分析与相图
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import rcParams
rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
rcParams['axes.unicode_minus'] = False


def pendulum_dynamics(state, t, m=1.0, L=1.0, g=9.81, b=0.1, tau=0.0):
    """非线性摆动力学"""
    theta, omega = state
    dtheta = omega
    domega = (tau - b * omega - m * g * L * np.sin(theta)) / (m * L**2)
    return np.array([dtheta, domega])


def linear_pendulum_dynamics(state, t, m=1.0, L=1.0, g=9.81, b=0.1, tau=0.0):
    """线性化摆动力学 (小角度近似)"""
    theta, omega = state
    dtheta = omega
    domega = (tau - b * omega - m * g * L * theta) / (m * L**2)
    return np.array([dtheta, domega])


def simulate_pendulum(dynamics_func, theta0, omega0, t_array, m=1.0, L=1.0, g=9.81, b=0.1, tau=0.0):
    """RK4 积分仿真"""
    n = len(t_array)
    states = np.zeros((n, 2))
    states[0] = [theta0, omega0]
    dt = t_array[1] - t_array[0]

    for i in range(n - 1):
        def dyn(s):
            return dynamics_func(s, t_array[i], m, L, g, b, tau)

        k1 = dyn(states[i])
        k2 = dyn(states[i] + 0.5 * dt * k1)
        k3 = dyn(states[i] + 0.5 * dt * k2)
        k4 = dyn(states[i] + dt * k3)
        states[i+1] = states[i] + (dt / 6) * (k1 + 2*k2 + 2*k3 + k4)

    return states


def nonlinear_pendulum_simulation():
    """非线性摆仿真主函数"""
    dt = 0.001
    t_end = 20.0
    t = np.arange(0, t_end, dt)

    # 物理参数
    m, L, g, b = 1.0, 1.0, 9.81, 0.05

    # 不同初始角度
    initial_angles = [0.1, 0.5, 1.0, 2.0, 2.8, np.pi - 0.1]
    labels = ['θ₀=5.7°', 'θ₀=28.6°', 'θ₀=57.3°', 'θ₀=114.6°', 'θ₀=160.4°', 'θ₀=174.3°']
    colors = plt.cm.viridis(np.linspace(0.1, 0.9, len(initial_angles)))

    # ========== 仿真1: 不同初始角度 ==========
    results = {}
    for theta0, label, color in zip(initial_angles, labels, colors):
        states_nl = simulate_pendulum(pendulum_dynamics, theta0, 0, t, m, L, g, b)
        results[label] = states_nl

    # ========== 仿真2: 线性 vs 非线性对比 ==========
    theta_compare = 1.5  # 85.9°
    states_nl = simulate_pendulum(pendulum_dynamics, theta_compare, 0, t, m, L, g, b)
    states_lin = simulate_pendulum(linear_pendulum_dynamics, theta_compare, 0, t, m, L, g, b)

    # ========== 仿真3: 能量分析 ==========
    def total_energy(theta, omega):
        """计算总能量: KE + PE"""
        KE = 0.5 * m * L**2 * omega**2
        PE = m * g * L * (1 - np.cos(theta))
        return KE, PE, KE + PE

    # ========== 绘图 ==========
    fig = plt.figure(figsize=(16, 14))
    fig.suptitle('非线性摆仿真 - 大角度摆动分析', fontsize=14, fontweight='bold')

    # 子图1: 不同初始角度的角度响应
    ax1 = fig.add_subplot(3, 2, 1)
    for label, color in zip(labels, colors):
        ax1.plot(t, np.degrees(results[label][:, 0]), color=color, linewidth=0.8, label=label)
    ax1.set_ylabel('角度 (°)')
    ax1.set_title('不同初始角度的摆动响应')
    ax1.legend(fontsize=7, loc='upper right')
    ax1.grid(True, alpha=0.3)
    ax1.set_xlim([0, t_end])

    # 子图2: 线性 vs 非线性对比
    ax2 = fig.add_subplot(3, 2, 2)
    ax2.plot(t, np.degrees(states_nl[:, 0]), 'b-', linewidth=1.0, label='非线性模型')
    ax2.plot(t, np.degrees(states_lin[:, 0]), 'r--', linewidth=1.0, label='线性化模型')
    ax2.set_ylabel('角度 (°)')
    ax2.set_title(f'线性 vs 非线性对比 (θ₀={np.degrees(theta_compare):.1f}°)')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    ax2.set_xlim([0, t_end])

    # 子图3: 周期分析 (大角度效应)
    ax3 = fig.add_subplot(3, 2, 3)
    thetas_scan = np.linspace(0.01, np.pi - 0.01, 200)
    # 数值计算周期
    T_analytical = []
    T_linear = 2 * np.pi * np.sqrt(L / g)  # 线性化周期
    for th0 in thetas_scan:
        # 椭圆积分近似
        k = np.sin(th0 / 2)
        T = T_linear * (1 + k**2/4 + 9*k**4/64 + 25*k**6/256)
        T_analytical.append(T)

    ax3.plot(np.degrees(thetas_scan), T_analytical, 'b-', linewidth=2, label='非线性周期(近似)')
    ax3.axhline(y=T_linear, color='r', linestyle='--', linewidth=1.5, label=f'线性周期 T₀={T_linear:.3f}s')
    ax3.set_xlabel('初始角度 (°)')
    ax3.set_ylabel('周期 T (s)')
    ax3.set_title('摆动周期 vs 初始角度')
    ax3.legend()
    ax3.grid(True, alpha=0.3)

    # 子图4: 相图 (Phase Portrait)
    ax4 = fig.add_subplot(3, 2, 4)
    # 绘制多条相轨线
    for theta0, label, color in zip(initial_angles, labels, colors):
        s = results[label]
        ax4.plot(s[:, 0], s[:, 1], color=color, linewidth=0.5, alpha=0.8, label=label)

    # 标注平衡点
    ax4.plot(0, 0, 'ko', markersize=8, label='稳定平衡点')
    ax4.plot(np.pi, 0, 'rx', markersize=10, markeredgewidth=2, label='不稳定平衡点')
    ax4.plot(-np.pi, 0, 'rx', markersize=10, markeredgewidth=2)

    ax4.set_xlabel('角度 θ (rad)')
    ax4.set_ylabel('角速度 ω (rad/s)')
    ax4.set_title('相图 (Phase Portrait)')
    ax4.legend(fontsize=6, loc='upper right')
    ax4.grid(True, alpha=0.3)
    ax4.set_xlim([-4, 4])

    # 子图5: 能量分析
    ax5 = fig.add_subplot(3, 2, 5)
    theta0_energy = 2.5  # 143°
    s_energy = simulate_pendulum(pendulum_dynamics, theta0_energy, 0, t, m, L, g, b)
    KE, PE, TE = total_energy(s_energy[:, 0], s_energy[:, 1])
    ax5.plot(t, KE, 'b-', linewidth=1.0, label='动能')
    ax5.plot(t, PE, 'r-', linewidth=1.0, label='势能')
    ax5.plot(t, TE, 'g--', linewidth=1.5, label='总能量')
    ax5.set_xlabel('时间 (s)')
    ax5.set_ylabel('能量 (J)')
    ax5.set_title(f'能量分析 (θ₀={np.degrees(theta0_energy):.0f}°, 含阻尼)')
    ax5.legend()
    ax5.grid(True, alpha=0.3)
    ax5.set_xlim([0, t_end])

    # 子图6: 角速度响应
    ax6 = fig.add_subplot(3, 2, 6)
    for label, color in zip(labels, colors):
        ax6.plot(t, results[label][:, 1], color=color, linewidth=0.5, alpha=0.7, label=label)
    ax6.set_xlabel('时间 (s)')
    ax6.set_ylabel('角速度 ω (rad/s)')
    ax6.set_title('角速度响应')
    ax6.legend(fontsize=7)
    ax6.grid(True, alpha=0.3)
    ax6.set_xlim([0, t_end])

    plt.tight_layout()
    plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'nonlinear_pendulum_result.png'),
                dpi=150, bbox_inches='tight')
    plt.close('all')

    # 打印分析结果
    print("=" * 60)
    print("非线性摆仿真结果分析")
    print("=" * 60)
    print(f"物理参数: m={m}kg, L={L}m, g={g}m/s², b={b}")
    print(f"线性化周期: T₀ = {T_linear:.4f} s")
    print()
    print("不同初始角度的周期估算:")
    for theta0_deg in [5, 15, 30, 45, 60, 90, 120, 150, 170]:
        th0 = np.radians(theta0_deg)
        k = np.sin(th0 / 2)
        T = T_linear * (1 + k**2/4 + 9*k**4/64 + 25*k**6/256)
        ratio = T / T_linear
        print(f"  θ₀ = {theta0_deg:3d}°  →  T = {T:.4f}s  (T/T₀ = {ratio:.4f})")

    # 线性 vs 非线性误差
    error = np.abs(states_nl[:, 0] - states_lin[:, 0])
    print(f"\n线性化误差 (θ₀={np.degrees(theta_compare):.1f}°):")
    print(f"  最大误差: {np.degrees(np.max(error)):.2f}°")
    print(f"  10秒后均值误差: {np.degrees(np.mean(error[int(10/dt):])):.2f}°")

    return results


if __name__ == '__main__':
    nonlinear_pendulum_simulation()
