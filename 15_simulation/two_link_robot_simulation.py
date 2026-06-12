"""
二连杆机械臂仿真 - PD + 重力补偿控制
=======================================
动力学方程:
  M(q)q̈ + C(q,q̇)q̇ + G(q) = τ

其中:
  M(q) - 惯性矩阵 (2x2, 正定对称)
  C(q,q̇) - 科里奥利和离心力矩阵
  G(q) - 重力项
  τ - 关节力矩

PD + 重力补偿:
  τ = Kp(qd - q) - Kv·q̇ + G(q)

特性:
  - 全局渐近稳定 (当 G(q) 精确已知)
  - 实现简单, 无需加速度反馈
  - 适合定点控制和缓慢轨迹跟踪
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import rcParams
rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
rcParams['axes.unicode_minus'] = False


def robot_dynamics(q, dq, tau, m1=1.0, m2=1.0, l1=1.0, l2=1.0,
                   lc1=0.5, lc2=0.5, I1=0.083, I2=0.083, g=9.81):
    """
    二连杆机械臂动力学
    返回: ddq (关节加速度)
    """
    q1, q2 = q
    dq1, dq2 = dq

    # 惯性矩阵 M(q)
    M11 = m1*lc1**2 + m2*(l1**2 + lc2**2 + 2*l1*lc2*np.cos(q2)) + I1 + I2
    M12 = m2*(lc2**2 + l1*lc2*np.cos(q2)) + I2
    M21 = M12
    M22 = m2*lc2**2 + I2
    M = np.array([[M11, M12], [M21, M22]])

    # 科里奥利和离心力矩阵 C(q, dq)
    h = m2 * l1 * lc2 * np.sin(q2)
    C11 = -h * dq2
    C12 = -h * (dq1 + dq2)
    C21 = h * dq1
    C22 = 0
    C = np.array([[C11, C12], [C21, C22]])

    # 重力项 G(q)
    G1 = (m1*lc1 + m2*l1)*g*np.cos(q1) + m2*lc2*g*np.cos(q1 + q2)
    G2 = m2*lc2*g*np.cos(q1 + q2)
    G_vec = np.array([G1, G2])

    # 计算加速度: M*q_dd = tau - C*dq - G
    ddq = np.linalg.solve(M, tau - C @ dq - G_vec)

    return ddq, M, C, G_vec


def gravity_compensation(q, m1=1.0, m2=1.0, l1=1.0, l2=1.0,
                         lc1=0.5, lc2=0.5, g=9.81):
    """计算重力补偿项"""
    q1, q2 = q
    G1 = (m1*lc1 + m2*l1)*g*np.cos(q1) + m2*lc2*g*np.cos(q1 + q2)
    G2 = m2*lc2*g*np.cos(q1 + q2)
    return np.array([G1, G2])


def two_link_robot_simulation():
    """二连杆机械臂 PD+重力补偿 控制仿真"""
    dt = 0.001
    t_end = 8.0
    t = np.arange(0, t_end, dt)
    n = len(t)

    # 物理参数
    params = dict(m1=1.0, m2=1.0, l1=1.0, l2=1.0,
                  lc1=0.5, lc2=0.5, I1=0.083, I2=0.083, g=9.81)

    # PD 增益
    Kp = np.diag([100.0, 100.0])
    Kv = np.diag([20.0, 20.0])

    # 仿真场景
    scenarios = {
        '定点控制': {
            'qd': np.array([np.pi/4, np.pi/3]),
            'dqd': np.array([0.0, 0.0]),
            'q0': np.array([0.0, 0.0]),
            'dq0': np.array([0.0, 0.0]),
        },
        '正弦轨迹跟踪': {
            'qd_func': lambda t: np.array([np.pi/4 * np.sin(0.5*t),
                                            np.pi/6 * np.cos(0.5*t)]),
            'dqd_func': lambda t: np.array([np.pi/4 * 0.5 * np.cos(0.5*t),
                                             -np.pi/6 * 0.5 * np.sin(0.5*t)]),
            'q0': np.array([0.0, 0.0]),
            'dq0': np.array([0.0, 0.0]),
        }
    }

    # ========== 场景1: 定点控制 ==========
    q = np.zeros((n, 2))
    dq = np.zeros((n, 2))
    tau_arr = np.zeros((n, 2))
    q[0] = scenarios['定点控制']['q0']
    dq[0] = scenarios['定点控制']['dq0']
    qd = scenarios['定点控制']['qd']
    dqd = scenarios['定点控制']['dqd']

    for i in range(n - 1):
        e = qd - q[i]
        de = dqd - dq[i]

        # PD + 重力补偿
        G = gravity_compensation(q[i], **{k: params[k] for k in ['m1','m2','l1','l2','lc1','lc2','g']})
        tau = Kp @ e - Kv @ de + G
        tau_arr[i] = tau

        # RK4 积分
        def dynamics(state):
            q_s, dq_s = state[:2], state[2:]
            ddq_s, _, _, _ = robot_dynamics(q_s, dq_s, tau, **params)
            return np.concatenate([dq_s, ddq_s])

        state = np.concatenate([q[i], dq[i]])
        k1 = dynamics(state)
        k2 = dynamics(state + 0.5*dt*k1)
        k3 = dynamics(state + 0.5*dt*k2)
        k4 = dynamics(state + dt*k3)
        state_new = state + (dt/6)*(k1 + 2*k2 + 2*k3 + k4)
        q[i+1] = state_new[:2]
        dq[i+1] = state_new[2:]

    q_fixed = q.copy()
    dq_fixed = dq.copy()
    tau_fixed = tau_arr.copy()

    # ========== 场景2: 正弦轨迹跟踪 ==========
    q = np.zeros((n, 2))
    dq = np.zeros((n, 2))
    tau_arr = np.zeros((n, 2))
    qd_arr = np.zeros((n, 2))
    q[0] = scenarios['正弦轨迹跟踪']['q0']
    dq[0] = scenarios['正弦轨迹跟踪']['dq0']

    for i in range(n - 1):
        qd = scenarios['正弦轨迹跟踪']['qd_func'](t[i])
        dqd = scenarios['正弦轨迹跟踪']['dqd_func'](t[i])
        qd_arr[i] = qd

        e = qd - q[i]
        de = dqd - dq[i]

        G = gravity_compensation(q[i], **{k: params[k] for k in ['m1','m2','l1','l2','lc1','lc2','g']})
        tau = Kp @ e - Kv @ de + G
        tau_arr[i] = tau

        def dynamics(state):
            q_s, dq_s = state[:2], state[2:]
            ddq_s, _, _, _ = robot_dynamics(q_s, dq_s, tau, **params)
            return np.concatenate([dq_s, ddq_s])

        state = np.concatenate([q[i], dq[i]])
        k1 = dynamics(state)
        k2 = dynamics(state + 0.5*dt*k1)
        k3 = dynamics(state + 0.5*dt*k2)
        k4 = dynamics(state + dt*k3)
        state_new = state + (dt/6)*(k1 + 2*k2 + 2*k3 + k4)
        q[i+1] = state_new[:2]
        dq[i+1] = state_new[2:]

    qd_arr[-1] = scenarios['正弦轨迹跟踪']['qd_func'](t[-1])
    q_track = q.copy()
    tau_track = tau_arr.copy()

    # ========== 绘图 ==========
    fig = plt.figure(figsize=(16, 14))
    fig.suptitle('二连杆机械臂 - PD + 重力补偿控制仿真', fontsize=14, fontweight='bold')

    # 定点控制 - 角度响应
    ax1 = fig.add_subplot(3, 2, 1)
    ax1.plot(t, np.degrees(q_fixed[:, 0]), 'b-', label='关节1实际')
    ax1.plot(t, np.degrees(q_fixed[:, 1]), 'r-', label='关节2实际')
    ax1.axhline(y=np.degrees(np.pi/4), color='b', linestyle='--', alpha=0.5, label='关节1目标')
    ax1.axhline(y=np.degrees(np.pi/3), color='r', linestyle='--', alpha=0.5, label='关节2目标')
    ax1.set_ylabel('角度 (°)')
    ax1.set_title('定点控制 - 关节角度响应')
    ax1.legend(fontsize=8)
    ax1.grid(True, alpha=0.3)

    # 定点控制 - 误差
    ax2 = fig.add_subplot(3, 2, 2)
    e1 = np.degrees(np.pi/4 - q_fixed[:, 0])
    e2 = np.degrees(np.pi/3 - q_fixed[:, 1])
    ax2.plot(t, e1, 'b-', label='关节1误差')
    ax2.plot(t, e2, 'r-', label='关节2误差')
    ax2.set_ylabel('误差 (°)')
    ax2.set_title('定点控制 - 跟踪误差')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    ax2.axhline(y=0, color='k', linestyle='--', linewidth=0.5)

    # 轨迹跟踪 - 角度
    ax3 = fig.add_subplot(3, 2, 3)
    ax3.plot(t, np.degrees(qd_arr[:, 0]), 'b--', linewidth=1.5, label='关节1期望')
    ax3.plot(t, np.degrees(q_track[:, 0]), 'b-', linewidth=0.8, label='关节1实际')
    ax3.plot(t, np.degrees(qd_arr[:, 1]), 'r--', linewidth=1.5, label='关节2期望')
    ax3.plot(t, np.degrees(q_track[:, 1]), 'r-', linewidth=0.8, label='关节2实际')
    ax3.set_ylabel('角度 (°)')
    ax3.set_title('正弦轨迹跟踪')
    ax3.legend(fontsize=8)
    ax3.grid(True, alpha=0.3)

    # 轨迹跟踪 - 误差
    ax4 = fig.add_subplot(3, 2, 4)
    e_track1 = np.degrees(qd_arr[:, 0] - q_track[:, 0])
    e_track2 = np.degrees(qd_arr[:, 1] - q_track[:, 1])
    ax4.plot(t, e_track1, 'b-', label='关节1误差')
    ax4.plot(t, e_track2, 'r-', label='关节2误差')
    ax4.set_ylabel('误差 (°)')
    ax4.set_title('轨迹跟踪误差')
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    ax4.axhline(y=0, color='k', linestyle='--', linewidth=0.5)

    # 控制力矩
    ax5 = fig.add_subplot(3, 2, 5)
    ax5.plot(t, tau_fixed[:, 0], 'b-', linewidth=0.8, label='关节1力矩')
    ax5.plot(t, tau_fixed[:, 1], 'r-', linewidth=0.8, label='关节2力矩')
    ax5.set_xlabel('时间 (s)')
    ax5.set_ylabel('力矩 (N·m)')
    ax5.set_title('定点控制 - 关节力矩')
    ax5.legend()
    ax5.grid(True, alpha=0.3)

    # 工作空间轨迹 (末端执行器)
    ax6 = fig.add_subplot(3, 2, 6)
    l1, l2 = params['l1'], params['l2']
    # 定点控制轨迹
    x_end_fixed = l1*np.cos(q_fixed[:, 0]) + l2*np.cos(q_fixed[:, 0] + q_fixed[:, 1])
    y_end_fixed = l1*np.sin(q_fixed[:, 0]) + l2*np.sin(q_fixed[:, 0] + q_fixed[:, 1])
    # 跟踪轨迹
    x_end_track = l1*np.cos(q_track[:, 0]) + l2*np.cos(q_track[:, 0] + q_track[:, 1])
    y_end_track = l1*np.sin(q_track[:, 0]) + l2*np.sin(q_track[:, 0] + q_track[:, 1])

    ax6.plot(x_end_fixed, y_end_fixed, 'b-', linewidth=0.5, alpha=0.5, label='定点控制')
    ax6.plot(x_end_track, y_end_track, 'r-', linewidth=0.5, alpha=0.5, label='轨迹跟踪')
    ax6.plot(x_end_fixed[0], y_end_fixed[0], 'go', markersize=8, label='起点')
    ax6.plot(x_end_fixed[-1], y_end_fixed[-1], 'b*', markersize=10)
    ax6.plot(x_end_track[-1], y_end_track[-1], 'r*', markersize=10)
    ax6.set_xlabel('X (m)')
    ax6.set_ylabel('Y (m)')
    ax6.set_title('工作空间末端轨迹')
    ax6.legend(fontsize=8)
    ax6.grid(True, alpha=0.3)
    ax6.set_aspect('equal')

    plt.tight_layout()
    plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'two_link_robot_result.png'),
                dpi=150, bbox_inches='tight')
    plt.close('all')

    # 性能指标
    print("=" * 60)
    print("二连杆机械臂 PD+重力补偿 仿真结果")
    print("=" * 60)
    print(f"PD增益: Kp = diag({Kp[0,0]}, {Kp[1,1]}), Kv = diag({Kv[0,0]}, {Kv[1,1]})")
    print()
    print("--- 定点控制 ---")
    steady_e1 = np.mean(np.abs(np.pi/4 - q_fixed[int(3/dt):, 0]))
    steady_e2 = np.mean(np.abs(np.pi/3 - q_fixed[int(3/dt):, 1]))
    print(f"  关节1稳态误差: {np.degrees(steady_e1):.6f}°")
    print(f"  关节2稳态误差: {np.degrees(steady_e2):.6f}°")
    print()
    print("--- 正弦轨迹跟踪 ---")
    se1 = np.mean(np.abs(qd_arr[int(2/dt):, 0] - q_track[int(2/dt):, 0]))
    se2 = np.mean(np.abs(qd_arr[int(2/dt):, 1] - q_track[int(2/dt):, 1]))
    print(f"  关节1跟踪误差均值: {np.degrees(se1):.6f}°")
    print(f"  关节2跟踪误差均值: {np.degrees(se2):.6f}°")
    me1 = np.max(np.abs(qd_arr[int(1/dt):, 0] - q_track[int(1/dt):, 0]))
    me2 = np.max(np.abs(qd_arr[int(1/dt):, 1] - q_track[int(1/dt):, 1]))
    print(f"  关节1最大误差: {np.degrees(me1):.6f}°")
    print(f"  关节2最大误差: {np.degrees(me2):.6f}°")

    return q_fixed, q_track


if __name__ == '__main__':
    two_link_robot_simulation()
