"""
反步控制(Backstepping Control)仿真 - 非线性系统
================================================
系统: 三阶严格反馈非线性系统
dx1 = x2 + x1^2
dx2 = x3 + sin(x1)
dx3 = u + x1*x3
目标: 使 x1 → x_d (参考信号)

反步控制设计:
  步骤1: z1 = x1 - x_d, 设计虚拟控制 α1
  步骤2: z2 = x2 - α1, 设计虚拟控制 α2
  步骤3: z3 = x3 - α2, 设计实际控制 u
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import rcParams
rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
rcParams['axes.unicode_minus'] = False


def backstepping_control():
    """反步控制仿真主函数"""
    # 仿真参数
    dt = 0.001
    t_end = 10.0
    t = np.arange(0, t_end, dt)
    n = len(t)

    # 控制增益
    c1, c2, c3 = 5.0, 5.0, 5.0

    # 状态初始化
    x1 = np.zeros(n)
    x2 = np.zeros(n)
    x3 = np.zeros(n)
    u = np.zeros(n)

    x1[0] = 0.5
    x2[0] = 0.0
    x3[0] = 0.0

    # 参考信号: 正弦跟踪
    x_d = np.sin(2 * np.pi * 0.5 * t)
    dx_d = 2 * np.pi * 0.5 * np.cos(2 * np.pi * 0.5 * t)

    # Lyapunov 函数记录
    V = np.zeros(n)

    for i in range(n - 1):
        # 步骤1: z1 = x1 - x_d
        z1 = x1[i] - x_d[i]

        # 虚拟控制 α1: 使 z1 收敛
        # V1 = 0.5*z1^2, dV1/dt = z1*(x2 + x1^2 - dx_d) = z1*(z2 + α1 + x1^2 - dx_d)
        # 选择 α1 = -c1*z1 - x1^2 + dx_d
        alpha1 = -c1 * z1 - x1[i]**2 + dx_d[i]

        # 步骤2: z2 = x2 - α1
        dalpha1_dt = (-c1 * (x2[i] + x1[i]**2 - dx_d[i]) - 2*x1[i]*(x2[i] + x1[i]**2) +
                      2*np.pi*0.5*2*np.pi*0.5*(-np.sin(2*np.pi*0.5*t[i])))
        z2 = x2[i] - alpha1

        # 虚拟控制 α2
        # dV2/dt = z1*(-c1*z1 + z2) + z2*(x3 + sin(x1) - dα1/dt)
        # 选择 α2 = -z1 - c2*z2 - sin(x1[i]) + dalpha1_dt
        alpha2 = -z1 - c2 * z2 - np.sin(x1[i]) + dalpha1_dt

        # 步骤3: z3 = x3 - α2
        dalpha2_dt = 0  # 简化，实际需要更复杂的计算
        z3 = x3[i] - alpha2

        # 实际控制 u
        # dV3/dt = z2*(-c2*z2 + z3) + z3*(u + x1*x3 - dα2/dt)
        # 选择 u = -z2 - c3*z3 - x1[i]*x3[i]
        u[i] = -z2 - c3 * z3 - x1[i] * x3[i]

        # 状态更新 (Runge-Kutta 4)
        def dynamics(xi):
            dx1 = xi[1] + xi[0]**2
            dx2 = xi[2] + np.sin(xi[0])
            dx3 = u[i] + xi[0] * xi[2]
            return np.array([dx1, dx2, dx3])

        state = np.array([x1[i], x2[i], x3[i]])
        k1 = dynamics(state)
        k2 = dynamics(state + 0.5*dt*k1)
        k3 = dynamics(state + 0.5*dt*k2)
        k4 = dynamics(state + dt*k3)
        state_new = state + (dt/6)*(k1 + 2*k2 + 2*k3 + k4)

        x1[i+1] = state_new[0]
        x2[i+1] = state_new[1]
        x3[i+1] = state_new[2]

        # Lyapunov 函数
        V[i] = 0.5 * (z1**2 + z2**2 + z3**2)

    # 最后一步控制
    z1 = x1[-1] - x_d[-1]
    alpha1 = -c1 * z1 - x1[-1]**2 + dx_d[-1]
    z2 = x2[-1] - alpha1
    alpha2 = -z1 - c2 * z2 - np.sin(x1[-1])
    z3 = x3[-1] - alpha2
    u[-1] = -z2 - c3 * z3 - x1[-1] * x3[-1]
    V[-1] = 0.5 * (z1**2 + z2**2 + z3**2)

    # 绘图
    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    fig.suptitle('反步控制(Backstepping Control) - 非线性系统仿真', fontsize=14, fontweight='bold')

    # 状态 x1 跟踪
    axes[0, 0].plot(t, x_d, 'r--', linewidth=1.5, label='参考信号 $x_d$')
    axes[0, 0].plot(t, x1, 'b-', linewidth=1.0, label='状态 $x_1$')
    axes[0, 0].set_ylabel('幅值')
    axes[0, 0].set_title('参考信号跟踪')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)

    # 跟踪误差
    axes[0, 1].plot(t, x1 - x_d, 'r-', linewidth=1.0)
    axes[0, 1].set_ylabel('误差')
    axes[0, 1].set_title('跟踪误差 e = x₁ - x_d')
    axes[0, 1].grid(True, alpha=0.3)
    axes[0, 1].axhline(y=0, color='k', linestyle='--', linewidth=0.5)

    # 状态轨迹
    axes[1, 0].plot(t, x1, label='$x_1$')
    axes[1, 0].plot(t, x2, label='$x_2$')
    axes[1, 0].plot(t, x3, label='$x_3$')
    axes[1, 0].set_ylabel('状态值')
    axes[1, 0].set_title('系统状态轨迹')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)

    # 控制输入
    axes[1, 1].plot(t, u, 'g-', linewidth=1.0)
    axes[1, 1].set_ylabel('控制量 u')
    axes[1, 1].set_title('控制输入')
    axes[1, 1].grid(True, alpha=0.3)

    # Lyapunov 函数
    axes[2, 0].plot(t, V, 'm-', linewidth=1.0)
    axes[2, 0].set_xlabel('时间 (s)')
    axes[2, 0].set_ylabel('V(t)')
    axes[2, 0].set_title('Lyapunov 函数 V = ½(z₁² + z₂² + z₃²)')
    axes[2, 0].grid(True, alpha=0.3)

    # 相平面
    axes[2, 1].plot(x1, x2, 'b-', linewidth=0.5, alpha=0.7)
    axes[2, 1].plot(x1[0], x2[0], 'ro', markersize=8, label='起点')
    axes[2, 1].plot(x1[-1], x2[-1], 'g*', markersize=10, label='终点')
    axes[2, 1].set_xlabel('$x_1$')
    axes[2, 1].set_ylabel('$x_2$')
    axes[2, 1].set_title('相平面轨迹 ($x_1$ vs $x_2$)')
    axes[2, 1].legend()
    axes[2, 1].grid(True, alpha=0.3)

    for ax in axes.flat:
        ax.set_xlim([0, t_end])

    plt.tight_layout()
    plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'backstepping_result.png'),
                dpi=150, bbox_inches='tight')
    plt.close('all')

    # 性能指标
    e = x1 - x_d
    idx_2s = int(2.0 / dt)
    steady_error = np.mean(np.abs(e[idx_2s:]))
    max_error = np.max(np.abs(e[int(0.5/dt):]))
    control_energy = np.sum(u**2) * dt

    print("=" * 60)
    print("反步控制仿真结果")
    print("=" * 60)
    print(f"控制增益: c1={c1}, c2={c2}, c3={c3}")
    print(f"稳态误差 (2s后均值): {steady_error:.6f}")
    print(f"最大跟踪误差: {max_error:.6f}")
    print(f"控制能量: {control_energy:.4f}")
    print(f"最终Lyapunov函数值: {V[-1]:.8f}")

    return t, x1, x_d, u, V


if __name__ == '__main__':
    backstepping_control()
