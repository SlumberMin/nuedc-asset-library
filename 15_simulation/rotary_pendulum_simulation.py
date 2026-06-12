#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
旋转倒立摆仿真 - 能量摆起 + LQR平衡控制
============================================
适用于电赛旋转倒立摆/自平衡类题目
包含两个阶段: 1.能量摆起(摇起到倒立点附近) 2.LQR平衡控制
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.signal import cont2discrete
from scipy.linalg import solve_continuous_are
import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False


class RotaryPendulum:
    """旋转倒立摆动力学模型
    状态: [theta, alpha, dtheta, dalpha]
    theta: 旋臂角度, alpha: 摆杆角度(0=倒立)
    """
    def __init__(self, dt=0.002):
        # 物理参数 (可调)
        self.Mp = 0.1       # 摆杆质量 (kg)
        self.Lp = 0.3       # 摆杆长度 (m)
        self.Lr = 0.2       # 旋臂长度 (m)
        self.Jr = 1e-4      # 旋臂转动惯量
        self.Jp = 3e-3      # 摆杆转动惯量
        self.Bp = 0.001     # 摆杆摩擦
        self.Br = 0.005     # 旋臂摩擦
        self.g = 9.81
        self.dt = dt

        # 状态初始化 (摆杆自然下垂)
        self.state = np.array([0.0, np.pi, 0.0, 0.0])  # [theta, alpha, dtheta, dalpha]

    def dynamics(self, state, tau):
        """连续时间动力学 (非线性)"""
        theta, alpha, dtheta, dalpha = state
        s_alpha = np.sin(alpha)
        c_alpha = np.cos(alpha)

        # 简化的旋转倒立摆方程
        # 旋臂: (Jr + Mp*Lr²)d²theta/dt² = tau - Br*dtheta - Mp*Lr*Lp/2*cos(alpha)*d²alpha/dt²
        #        + Mp*Lr*Lp/2*sin(alpha)*dalpha²
        # 摆杆: Jp*d²alpha/dt² = Mp*g*Lp/2*sin(alpha) - Bp*dalpha - Mp*Lr*Lp/2*cos(alpha)*d²theta/dt²

        # 惯性矩阵
        M11 = self.Jr + self.Mp * self.Lr**2
        M12 = self.Mp * self.Lr * self.Lp / 2 * c_alpha
        M22 = self.Jp
        M21 = M12

        # Coriolis力
        C1 = self.Mp * self.Lr * self.Lp / 2 * s_alpha * dalpha**2
        C2 = 0

        # 重力
        G2 = self.Mp * self.g * self.Lp / 2 * s_alpha

        # 摩擦
        F1 = self.Br * dtheta
        F2 = self.Bp * dalpha

        # 求解加速度 [M11 M12; M21 M22] * [ddtheta; ddalpha] = [RHS1; RHS2]
        RHS1 = tau - F1 + C1
        RHS2 = G2 - F2 + C2

        det = M11 * M22 - M12 * M21
        if abs(det) < 1e-10:
            det = 1e-10

        ddtheta = (M22 * RHS1 - M12 * RHS2) / det
        ddalpha = (M11 * RHS2 - M21 * RHS1) / det

        return np.array([dtheta, dalpha, ddtheta, ddalpha])

    def step(self, tau):
        """四阶龙格-库塔积分"""
        s = self.state
        k1 = self.dynamics(s, tau)
        k2 = self.dynamics(s + 0.5 * self.dt * k1, tau)
        k3 = self.dynamics(s + 0.5 * self.dt * k2, tau)
        k4 = self.dynamics(s + self.dt * k3, tau)
        self.state = s + self.dt / 6 * (k1 + 2*k2 + 2*k3 + k4)

        # 角度归一化
        self.state[0] = (self.state[0] + np.pi) % (2 * np.pi) - np.pi
        self.state[1] = (self.state[1] + np.pi) % (2 * np.pi) - np.pi

        return self.state.copy()


def lqr(A, B, Q, R):
    """求解连续时间LQR"""
    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.inv(R) @ B.T @ P
    return K, P


def linearize_pendulum(params):
    """在倒立平衡点线性化 (alpha=0附近)
    得到 x_dot = Ax + Bu 形式
    """
    Mp, Lp, Lr, Jr, Jp, Bp, Br, g = params

    # 线性化后的A, B矩阵 (简化推导)
    # 状态: [theta, alpha, dtheta, dalpha]
    M11 = Jr + Mp * Lr**2
    M22 = Jp
    M12 = Mp * Lr * Lp / 2

    det_M = M11 * M22 - M12**2

    A = np.zeros((4, 4))
    A[0, 2] = 1.0
    A[1, 3] = 1.0
    # ddtheta 线性化
    A[2, 2] = -Br * M22 / det_M
    A[2, 3] = M12 * Bp / det_M
    # ddalpha 线性化 (含重力项)
    A[3, 1] = Mp * g * Lp / 2 * M11 / det_M  # 不稳定极点来源
    A[3, 2] = M12 * Br / det_M
    A[3, 3] = -Bp * M11 / det_M

    B = np.zeros((4, 1))
    B[2, 0] = M22 / det_M
    B[3, 0] = -M12 / det_M

    return A, B


def energy_shaping_control(pendulum, E_target):
    """能量摆起控制器
    通过控制旋臂使摆杆获得足够能量到达倒立位置
    """
    alpha = pendulum.state[1]
    dalpha = pendulum.state[3]
    theta = pendulum.state[0]

    # 摆杆能量 (相对于倒立点)
    # E = 0.5 * Jp * dalpha^2 + Mp*g*Lp/2 * (1 - cos(alpha))
    E = 0.5 * pendulum.Jp * dalpha**2 + pendulum.Mp * pendulum.g * pendulum.Lp / 2 * (1 - np.cos(alpha))

    # 能量误差
    dE = E - E_target

    # Lyapunov能量控制律
    # tau = -k * dE * dalpha * cos(alpha)  (使能量趋近目标)
    k_energy = 0.5
    tau = -k_energy * dE * dalpha * np.cos(alpha)

    # 限制力矩
    tau = np.clip(tau, -2.0, 2.0)

    return tau, E


class LQRController:
    """LQR平衡控制器"""
    def __init__(self, pendulum):
        params = (pendulum.Mp, pendulum.Lp, pendulum.Lr,
                  pendulum.Jr, pendulum.Jp, pendulum.Bp,
                  pendulum.Br, pendulum.g)
        A, B = linearize_pendulum(params)

        # LQR权重矩阵
        Q = np.diag([10.0, 100.0, 1.0, 10.0])  # 状态惩罚
        R = np.array([[1.0]])                     # 控制惩罚

        self.K, self.P = lqr(A, B, Q, R)
        self.A = A
        self.B = B

    def compute(self, state):
        """LQR控制: u = -K*x"""
        # 摆杆角度偏移 (以倒立点为0)
        x = state.copy()
        x[1] = state[1]  # alpha已经是相对于倒立点的偏移

        tau = -self.K @ x
        return np.clip(tau[0], -5.0, 5.0)


def simulate(duration=15.0, dt=0.002):
    """运行仿真: 先能量摆起，再LQR平衡"""
    steps = int(duration / dt)
    pendulum = RotaryPendulum(dt=dt)

    # 初始: 摆杆自然下垂 (alpha = pi)
    pendulum.state = np.array([0.0, np.pi, 0.0, 0.0])

    lqr_ctrl = LQRController(pendulum)

    # 目标能量 (倒立点势能)
    E_target = pendulum.Mp * pendulum.g * pendulum.Lp

    # 结果
    t = np.arange(steps) * dt
    theta_log = np.zeros(steps)
    alpha_log = np.zeros(steps)
    torque_log = np.zeros(steps)
    energy_log = np.zeros(steps)
    phase_log = np.zeros(steps)  # 0=摆起, 1=LQR

    lqr_switched = False
    switch_time = 0

    for i in range(steps):
        alpha = pendulum.state[1]

        # 切换条件: 摆杆接近倒立点 (|alpha| < 0.3 rad ≈ 17°) 且角速度不大
        if not lqr_switched and abs(alpha) < 0.3 and abs(pendulum.state[3]) < 2.0:
            lqr_switched = True
            switch_time = t[i]
            print(f"  在 t={switch_time:.2f}s 切换到LQR平衡控制")

        if lqr_switched:
            tau = lqr_ctrl.compute(pendulum.state)
            phase_log[i] = 1
        else:
            tau, E = energy_shaping_control(pendulum, E_target)
            energy_log[i] = E

        state = pendulum.step(tau)
        theta_log[i] = state[0]
        alpha_log[i] = state[1]
        torque_log[i] = tau

    return {
        't': t, 'theta': theta_log, 'alpha': alpha_log,
        'torque': torque_log, 'energy': energy_log,
        'phase': phase_log, 'switch_time': switch_time,
        'E_target': E_target
    }


def plot_results(r):
    """绘制结果"""
    fig, axes = plt.subplots(4, 1, figsize=(12, 12), sharex=True)

    # 旋臂角度
    axes[0].plot(r['t'], np.degrees(r['theta']), 'b-', linewidth=1)
    axes[0].set_ylabel('旋臂角度 (°)')
    axes[0].set_title('旋转倒立摆仿真 - 能量摆起 + LQR平衡')
    axes[0].grid(True, alpha=0.3)

    # 摆杆角度
    axes[1].plot(r['t'], np.degrees(r['alpha']), 'r-', linewidth=1)
    axes[1].axhline(y=0, color='k', linestyle='--', alpha=0.3)
    axes[1].axhline(y=180, color='k', linestyle='--', alpha=0.3)
    axes[1].axhline(y=-180, color='k', linestyle='--', alpha=0.3)
    if r['switch_time'] > 0:
        axes[1].axvline(x=r['switch_time'], color='g', linestyle=':', linewidth=2,
                        label=f'LQR切换点 t={r["switch_time"]:.1f}s')
    axes[1].set_ylabel('摆杆角度 (°)')
    axes[1].legend(loc='best')
    axes[1].grid(True, alpha=0.3)

    # 能量
    axes[2].plot(r['t'], r['energy'], 'g-', linewidth=1)
    axes[2].axhline(y=r['E_target'], color='r', linestyle='--', label='目标能量')
    axes[2].set_ylabel('能量 (J)')
    axes[2].set_title('摆杆能量变化')
    axes[2].legend(loc='best')
    axes[2].grid(True, alpha=0.3)

    # 控制力矩
    axes[3].plot(r['t'], r['torque'], 'purple', linewidth=0.5, alpha=0.7)
    axes[3].set_xlabel('时间 (s)')
    axes[3].set_ylabel('控制力矩 (N·m)')
    axes[3].set_title('控制输入')
    axes[3].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('rotary_pendulum_result.png', dpi=150, bbox_inches='tight')
    plt.close('all')

    # 统计
    lqr_mask = r['phase'] > 0
    if np.any(lqr_mask):
        alpha_lqr = r['alpha'][lqr_mask]
        print(f"  LQR阶段摆杆角度标准差: {np.std(np.degrees(alpha_lqr)):.2f}°")
        print(f"  最终摆杆角度: {np.degrees(alpha_lqr[-1]):.2f}°")


if __name__ == '__main__':
    print("=" * 60)
    print("  旋转倒立摆仿真 (能量摆起 + LQR平衡)")
    print("=" * 60)
    results = simulate()
    plot_results(results)
