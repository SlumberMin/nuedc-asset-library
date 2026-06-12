#!/usr/bin/env python3
"""
H∞鲁棒控制仿真
================
对比H∞控制与LQR控制在存在模型不确定性和干扰时的鲁棒性能。

被控对象：二阶系统（如电机、弹簧-质量-阻尼器）
- 标称模型：G(s) = 1 / (ms² + bs + k)
- 不确定性：参数 ±30% 摄动
- 外部干扰：阶跃干扰 + 正弦干扰

运行: python h_infinity_simulation.py
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import rcParams
from scipy import linalg

rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
rcParams['axes.unicode_minus'] = False


# ============================================================
# 系统模型
# ============================================================

class SecondOrderPlant:
    """二阶系统：m*x'' + b*x' + k*x = u"""
    def __init__(self, m=1.0, b=0.5, k=2.0, dt=0.001):
        self.m, self.b, self.k = m, b, k
        self.dt = dt
        self.x1 = 0.0  # 位置
        self.x2 = 0.0  # 速度

    def update(self, u, disturbance=0.0):
        # x1' = x2
        # x2' = (u - b*x2 - k*x1 + d) / m
        dx1 = self.x2 * self.dt
        dx2 = ((u + disturbance - self.b * self.x2 - self.k * self.x1) / self.m) * self.dt
        self.x1 += dx1
        self.x2 += dx2
        return self.x1, self.x2

    def get_state(self):
        return np.array([self.x1, self.x2])

    def reset(self):
        self.x1 = 0.0
        self.x2 = 0.0


# ============================================================
# 控制器实现
# ============================================================

class HInfController:
    """H∞控制器（离散化状态反馈）"""
    def __init__(self, A, B, Q, R, gamma=1.0, B2=None, dt=0.001):
        self.n = A.shape[0]
        self.dt = dt
        self.K = self._solve_hinf(A, B, Q, R, gamma, B2)
        self.x_ref = np.zeros(self.n)

    def _solve_hinf(self, A, B, Q, R, gamma, B2):
        """求解H∞控制增益（连续域Riccati方程迭代法）"""
        n = A.shape[0]
        m = B.shape[1]
        if B2 is None:
            B2 = np.eye(n)  # 默认干扰从所有状态输入

        P = Q.copy()
        Rinv = np.linalg.inv(R)
        gamma2_inv = 1.0 / (gamma ** 2)

        for _ in range(500):
            P_old = P.copy()
            # ARE: A^T P + PA + Q + (1/γ²)P B2 B2^T P - P B R^{-1} B^T P = 0
            Riccati = (A.T @ P + P @ A + Q
                       + gamma2_inv * P @ B2 @ B2.T @ P
                       - P @ B @ Rinv @ B.T @ P)
            P -= 0.005 * Riccati
            if np.linalg.norm(P - P_old) < 1e-8:
                break

        K = Rinv @ B.T @ P
        return K

    def compute(self, x_meas):
        e = self.x_ref - x_meas
        u = -self.K @ e
        return float(u)


class LQRController:
    """LQR控制器（对比用）"""
    def __init__(self, A, B, Q, R, dt=0.001):
        self.dt = dt
        self.n = A.shape[0]
        self.K = self._solve_lqr(A, B, Q, R)
        self.x_ref = np.zeros(self.n)

    def _solve_lqr(self, A, B, Q, R):
        """求解连续代数Riccati方程"""
        P = linalg.solve_continuous_are(A, B, Q, R)
        Rinv = np.linalg.inv(R)
        K = Rinv @ B.T @ P
        return K

    def compute(self, x_meas):
        e = self.x_ref - x_meas
        u = -self.K @ e
        return float(u)


# ============================================================
# 仿真函数
# ============================================================

def simulate(controller, plant, target_func, duration, dt,
             disturbance_func=None):
    steps = int(duration / dt)
    t = np.arange(steps) * dt
    x1_out = np.zeros(steps)
    x2_out = np.zeros(steps)
    u_out = np.zeros(steps)

    for i in range(steps):
        r = target_func(t[i])
        controller.x_ref[0] = r
        controller.x_ref[1] = 0.0

        state = plant.get_state()
        u = controller.compute(state)

        d = disturbance_func(t[i]) if disturbance_func else 0.0
        x1, x2 = plant.update(u, d)

        x1_out[i] = x1
        x2_out[i] = x2
        u_out[i] = u

    return t, x1_out, x2_out, u_out


# ============================================================
# 主程序
# ============================================================

def main():
    dt = 0.001
    duration = 5.0

    # 标称参数
    m_nom, b_nom, k_nom = 1.0, 0.5, 2.0

    # 状态空间：x' = Ax + Bu
    A_nom = np.array([[0, 1],
                       [-k_nom/m_nom, -b_nom/m_nom]])
    B_nom = np.array([[0], [1/m_nom]])

    # 加权矩阵
    Q = np.diag([10.0, 1.0])
    R = np.array([[0.1]])

    # H∞控制器（γ=1.0，B2=I表示干扰从所有通道进入）
    B2 = np.eye(2)
    hinf_ctrl = HInfController(A_nom, B_nom, Q, R, gamma=1.0, B2=B2, dt=dt)

    # LQR控制器
    lqr_ctrl = LQRController(A_nom, B_nom, Q, R, dt=dt)

    # 目标信号
    def target_step(t):
        return 1.0 if t >= 0.5 else 0.0

    def target_sine(t):
        return np.sin(2 * np.pi * 0.5 * t) if t >= 0.5 else 0.0

    # 干扰信号
    def dist_step(t):
        return 0.5 if t >= 2.0 else 0.0

    def dist_sine(t):
        return 0.3 * np.sin(2 * np.pi * 2.0 * t) if t >= 1.0 else 0.0

    colors = {'H∞控制': '#2196F3', 'LQR控制': '#FF5722'}
    fig, axes = plt.subplots(3, 2, figsize=(14, 14))
    fig.suptitle('H∞鲁棒控制仿真对比', fontsize=16, fontweight='bold')

    # ========== 场景1: 标称模型 + 阶跃干扰 ==========
    ax = axes[0, 0]
    plant = SecondOrderPlant(m_nom, b_nom, k_nom, dt)
    hinf_ctrl.x_ref = np.zeros(2)
    t, x1, _, u = simulate(hinf_ctrl, SecondOrderPlant(m_nom, b_nom, k_nom, dt),
                            target_step, duration, dt, dist_step)
    ax.plot(t, x1, color='#2196F3', label='H∞', linewidth=1.5)
    plant = SecondOrderPlant(m_nom, b_nom, k_nom, dt)
    lqr_ctrl.x_ref = np.zeros(2)
    t, x1, _, u = simulate(lqr_ctrl, SecondOrderPlant(m_nom, b_nom, k_nom, dt),
                            target_step, duration, dt, dist_step)
    ax.plot(t, x1, color='#FF5722', label='LQR', linewidth=1.5)
    target = np.array([target_step(ti) for ti in t])
    ax.plot(t, target, 'k--', alpha=0.4, label='目标')
    ax.axvline(x=2.0, color='gray', linestyle=':', alpha=0.5, label='干扰开始')
    ax.set_title('标称模型 + 阶跃干扰')
    ax.set_ylabel('输出')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # ========== 场景2: 标称模型 + 正弦干扰 ==========
    ax = axes[0, 1]
    plant = SecondOrderPlant(m_nom, b_nom, k_nom, dt)
    hinf_ctrl.x_ref = np.zeros(2)
    t, x1, _, u = simulate(hinf_ctrl, SecondOrderPlant(m_nom, b_nom, k_nom, dt),
                            target_step, duration, dt, dist_sine)
    ax.plot(t, x1, color='#2196F3', label='H∞', linewidth=1.5)
    lqr_ctrl.x_ref = np.zeros(2)
    t, x1, _, u = simulate(lqr_ctrl, SecondOrderPlant(m_nom, b_nom, k_nom, dt),
                            target_step, duration, dt, dist_sine)
    ax.plot(t, x1, color='#FF5722', label='LQR', linewidth=1.5)
    target = np.array([target_step(ti) for ti in t])
    ax.plot(t, target, 'k--', alpha=0.4, label='目标')
    ax.set_title('标称模型 + 正弦干扰')
    ax.set_ylabel('输出')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # ========== 场景3: 参数摄动+30% ==========
    ax = axes[1, 0]
    m_pert, b_pert, k_pert = m_nom * 1.3, b_nom * 0.7, k_nom * 1.3
    hinf_ctrl.x_ref = np.zeros(2)
    t, x1, _, _ = simulate(hinf_ctrl, SecondOrderPlant(m_pert, b_pert, k_pert, dt),
                            target_step, duration, dt, dist_step)
    ax.plot(t, x1, color='#2196F3', label='H∞', linewidth=1.5)
    lqr_ctrl.x_ref = np.zeros(2)
    t, x1, _, _ = simulate(lqr_ctrl, SecondOrderPlant(m_pert, b_pert, k_pert, dt),
                            target_step, duration, dt, dist_step)
    ax.plot(t, x1, color='#FF5722', label='LQR', linewidth=1.5)
    target = np.array([target_step(ti) for ti in t])
    ax.plot(t, target, 'k--', alpha=0.4, label='目标')
    ax.set_title('参数摄动 +30% + 阶跃干扰')
    ax.set_ylabel('输出')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # ========== 场景4: 参数摄动-30% ==========
    ax = axes[1, 1]
    m_pert2, b_pert2, k_pert2 = m_nom * 0.7, b_nom * 1.3, k_nom * 0.7
    hinf_ctrl.x_ref = np.zeros(2)
    t, x1, _, _ = simulate(hinf_ctrl, SecondOrderPlant(m_pert2, b_pert2, k_pert2, dt),
                            target_step, duration, dt, dist_step)
    ax.plot(t, x1, color='#2196F3', label='H∞', linewidth=1.5)
    lqr_ctrl.x_ref = np.zeros(2)
    t, x1, _, _ = simulate(lqr_ctrl, SecondOrderPlant(m_pert2, b_pert2, k_pert2, dt),
                            target_step, duration, dt, dist_step)
    ax.plot(t, x1, color='#FF5722', label='LQR', linewidth=1.5)
    target = np.array([target_step(ti) for ti in t])
    ax.plot(t, target, 'k--', alpha=0.4, label='目标')
    ax.set_title('参数摄动 -30% + 阶跃干扰')
    ax.set_ylabel('输出')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # ========== 场景5: 正弦跟踪 ==========
    ax = axes[2, 0]
    hinf_ctrl.x_ref = np.zeros(2)
    t, x1, _, _ = simulate(hinf_ctrl, SecondOrderPlant(m_nom, b_nom, k_nom, dt),
                            target_sine, duration, dt)
    ax.plot(t, x1, color='#2196F3', label='H∞', linewidth=1.5)
    lqr_ctrl.x_ref = np.zeros(2)
    t, x1, _, _ = simulate(lqr_ctrl, SecondOrderPlant(m_nom, b_nom, k_nom, dt),
                            target_sine, duration, dt)
    ax.plot(t, x1, color='#FF5722', label='LQR', linewidth=1.5)
    target = np.array([target_sine(ti) for ti in t])
    ax.plot(t, target, 'k--', alpha=0.4, label='目标')
    ax.set_title('正弦信号跟踪')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('输出')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # ========== 场景6: 控制量对比 ==========
    ax = axes[2, 1]
    hinf_ctrl.x_ref = np.zeros(2)
    t, _, _, u_hinf = simulate(hinf_ctrl, SecondOrderPlant(m_nom, b_nom, k_nom, dt),
                                target_step, duration, dt, dist_step)
    ax.plot(t, u_hinf, color='#2196F3', label='H∞', linewidth=1)
    lqr_ctrl.x_ref = np.zeros(2)
    t, _, _, u_lqr = simulate(lqr_ctrl, SecondOrderPlant(m_nom, b_nom, k_nom, dt),
                                target_step, duration, dt, dist_step)
    ax.plot(t, u_lqr, color='#FF5722', label='LQR', linewidth=1)
    ax.set_title('控制量对比')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('控制量 u')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('h_infinity_simulation.png', dpi=150, bbox_inches='tight')
    print("图片已保存: h_infinity_simulation.png")

    # 打印增益矩阵
    print("\n" + "=" * 50)
    print("控制器增益矩阵 K")
    print("=" * 50)
    print(f"H∞ K = {hinf_ctrl.K}")
    print(f"LQR K = {lqr_ctrl.K}")
    print(f"H∞ γ = {1.0}")

    plt.close('all')


if __name__ == '__main__':
    main()
