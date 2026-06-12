#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
二连杆机械臂自适应控制仿真
==========================
对比: 计算力矩法(已知模型) vs 自适应控制(参数不确定)
动力学: M(q)q̈ + C(q,q̇)q̇ + G(q) = τ
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
plt.rcParams['axes.unicode_minus'] = False


# ---- 物理参数 (真值) ----
PARAMS_TRUE = dict(m1=1.0, m2=1.0, l1=1.0, l2=1.0, lc1=0.5, lc2=0.5, I1=0.083, I2=0.083, g=9.81)
# 用于自适应的初始猜测 (偏离真值)
PARAMS_INIT = dict(m1=0.5, m2=0.5, l1=1.0, l2=1.0, lc1=0.5, lc2=0.5, I1=0.04, I2=0.04, g=9.81)


def dynamics_matrices(q, p):
    """计算 M(q), C(q,q̇)q̇, G(q)"""
    q1, q2 = q
    m1, m2, l1, l2 = p['m1'], p['m2'], p['l1'], p['l2']
    lc1, lc2 = p['lc1'], p['lc2']
    I1, I2, g = p['I1'], p['I2'], p['g']

    c2 = np.cos(q2)
    s2 = np.sin(q2)

    M11 = m1*lc1**2 + I1 + m2*(l1**2 + lc2**2 + 2*l1*lc2*c2) + I2
    M12 = m2*(lc2**2 + l1*lc2*c2) + I2
    M22 = m2*lc2**2 + I2

    M = np.array([[M11, M12], [M12, M22]])

    h = -m2 * l1 * lc2 * s2
    C = np.array([[h, h], [-h, 0]])  # simplified

    G1 = (m1*lc1 + m2*l1)*g*np.cos(q1) + m2*lc2*g*np.cos(q1+q2)
    G2 = m2*lc2*g*np.cos(q1+q2)
    G = np.array([G1, G2])

    return M, C, G


def desired_trajectory(t):
    """期望关节轨迹"""
    qd1 = np.sin(t)
    qd2 = np.cos(t)
    dqd1 = np.cos(t)
    dqd2 = -np.sin(t)
    ddqd1 = -np.sin(t)
    ddqd2 = -np.cos(t)
    return np.array([qd1, qd2]), np.array([dqd1, dqd2]), np.array([ddqd1, ddqd2])


def compute_torque_control(q, dq, qd, dqd, ddqd, p, Kp, Kd):
    """计算力矩法 (需要精确模型)"""
    M, C, G = dynamics_matrices(q, p)
    e = qd - q
    de = dqd - dq
    v = ddqd + Kd @ de + Kp @ e
    tau = M @ v + C @ dq + G
    return tau, e


def adaptive_control(q, dq, qd, dqd, ddqd, p_est, theta_hat, Kp, Kd, Gamma, dt):
    """自适应控制律"""
    M_est, C_est, G_est = dynamics_matrices(q, p_est)
    e = qd - q
    de = dqd - dq
    r = de + Kp @ e  # 滑模面-like

    v = ddqd + Kp @ de
    # 自适应更新 (简化: 跟踪误差驱动)
    theta_hat += -Gamma @ r * dt
    theta_hat = np.clip(theta_hat, -10, 10)

    tau = M_est @ v + C_est @ dq + G_est + Kd @ r + theta_hat
    return tau, e, theta_hat


def simulate(controller_type, T=10.0, dt=0.001):
    """仿真主循环"""
    t_arr = np.arange(0, T, dt)
    Kp = np.diag([100.0, 100.0])
    Kd = np.diag([50.0, 50.0])

    q = np.array([0.0, 0.0])
    dq = np.array([0.0, 0.0])

    if controller_type == 'adaptive':
        p = PARAMS_INIT.copy()
        theta_hat = np.zeros(2)
        Gamma = np.diag([1.0, 1.0])
    else:
        p = PARAMS_TRUE.copy()

    history = {'t': [], 'q': [], 'e': []}

    for i, t in enumerate(t_arr):
        qd, dqd, ddqd = desired_trajectory(t)

        if controller_type == 'ctc':
            tau, e = compute_torque_control(q, dq, qd, dqd, ddqd, p, Kp, Kd)
        else:
            tau, e, theta_hat = adaptive_control(q, dq, qd, dqd, ddqd, p, theta_hat, Kp, Kd, Gamma, dt)

        # 动力学积分 (真值参数)
        M_true, C_true, G_true = dynamics_matrices(q, PARAMS_TRUE)
        ddq = np.linalg.solve(M_true, tau - C_true @ dq - G_true)
        dq += ddq * dt
        q += dq * dt

        history['t'].append(t)
        history['q'].append(q.copy())
        history['e'].append(e.copy())

    for k in history:
        history[k] = np.array(history[k])
    return history


def main():
    print("仿真计算力矩法...")
    h_ctc = simulate('ctc')
    print("仿真自适应控制...")
    h_adp = simulate('adaptive')

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    for i, joint in enumerate(['关节1', '关节2']):
        axes[0, i].plot(h_ctc['t'], h_ctc['q'][:, i], label='CTC(精确模型)')
        axes[0, i].plot(h_adp['t'], h_adp['q'][:, i], '--', label='自适应')
        qd = np.sin(h_ctc['t']) if i == 0 else np.cos(h_ctc['t'])
        axes[0, i].plot(h_ctc['t'], qd, 'k:', label='期望')
        axes[0, i].set_title(f'{joint} 跟踪')
        axes[0, i].legend()
        axes[0, i].grid(True, alpha=0.3)

        axes[1, i].plot(h_ctc['t'], h_ctc['e'][:, i], label='CTC误差')
        axes[1, i].plot(h_adp['t'], h_adp['e'][:, i], '--', label='自适应误差')
        axes[1, i].set_title(f'{joint} 跟踪误差')
        axes[1, i].set_xlabel('时间 (s)')
        axes[1, i].legend()
        axes[1, i].grid(True, alpha=0.3)

    plt.suptitle('二连杆机械臂自适应控制仿真', fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig('two_link_adaptive_result.png', dpi=150)
    plt.close('all')

    print(f"\nCTC    关节1 RMSE: {np.sqrt(np.mean(h_ctc['e'][:,0]**2)):.6f}")
    print(f"CTC    关节2 RMSE: {np.sqrt(np.mean(h_ctc['e'][:,1]**2)):.6f}")
    print(f"自适应 关节1 RMSE: {np.sqrt(np.mean(h_adp['e'][:,0]**2)):.6f}")
    print(f"自适应 关节2 RMSE: {np.sqrt(np.mean(h_adp['e'][:,1]**2)):.6f}")


if __name__ == '__main__':
    main()
