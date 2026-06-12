#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ILC 迭代学习控制仿真
====================
仿真内容：迭代学习控制用于重复任务，展示精度随迭代次数提升
被控对象：机械臂单关节（二阶系统）
任务：周期性轨迹跟踪
"""

import os
import numpy as np
import matplotlib


def main():
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import matplotlib
    matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
    matplotlib.rcParams['axes.unicode_minus'] = False

    dt = 0.005
    T_cycle = 4.0  # 一个周期的时间
    N_cycle = int(T_cycle / dt)
    t_cycle = np.arange(N_cycle) * dt
    n_iterations = 15  # 迭代次数

    # ============ 被控对象：机械臂关节 ============
    def arm_dynamics(state, u, dt):
        """机械臂单关节模型: J*theta'' + b*theta' + mgl*sin(theta) = u"""
        theta, theta_dot = state
        J = 0.1     # 转动惯量
        b = 0.05    # 粘性摩擦
        mgl = 0.5   # 重力矩

        theta_ddot = (u - b * theta_dot - mgl * np.sin(theta)) / J
        theta_dot_new = theta_dot + theta_ddot * dt
        theta_new = theta + theta_dot_new * dt

        return np.array([theta_new, theta_dot_new])

    # ============ 参考轨迹 ============
    def reference_trajectory(t):
        """正弦轨迹: theta_d = A*sin(omega*t)"""
        A = 1.0  # 振幅 (rad)
        omega = 2 * np.pi / T_cycle
        return A * np.sin(omega * t)

    ref_traj = np.array([reference_trajectory(t_cycle[i]) for i in range(N_cycle)])

    # ============ PID控制器（用于前馈补偿） ============
    class PIDController:
        def __init__(self, Kp, Ki, Kd):
            self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
            self.integral = 0.0
            self.prev_error = 0.0

        def compute(self, ref, y, dt):
            error = ref - y
            self.integral += error * dt
            derivative = (error - self.prev_error) / dt
            self.prev_error = error
            return self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        def reset(self):
            self.integral = 0.0
            self.prev_error = 0.0

    # ============ ILC控制器 ============
    class ILCController:
        """
        迭代学习控制器
        学习律: u_{k+1}(t) = Q * [u_k(t) + L * e_k(t+1)]
        Q: 低通滤波器（鲁棒性）
        L: 学习增益
        """
        def __init__(self, N, learning_gain=0.8, Q_filter=None):
            self.N = N
            self.L = learning_gain
            self.Q_filter = Q_filter
            self.u_prev = np.zeros(N)  # 上一次迭代的控制序列

        def learn(self, error_trajectory):
            """
            根据误差轨迹更新控制序列
            error_trajectory: 本次迭代的误差序列
            """
            # 学习律
            u_new = self.u_prev + self.L * error_trajectory

            # Q滤波（低通，去除高频）
            if self.Q_filter is not None:
                # 简单移动平均滤波
                kernel = np.ones(self.Q_filter) / self.Q_filter
                u_new = np.convolve(u_new, kernel, mode='same')

            self.u_prev = u_new.copy()
            return u_new

        def reset(self):
            self.u_prev = np.zeros(self.N)

    # ============ 迭代学习仿真 ============
    print("=" * 60)
    print("ILC 迭代学习控制仿真")
    print("=" * 60)

    ilc = ILCController(N_cycle, learning_gain=0.5, Q_filter=5)
    pid_base = PIDController(Kp=20, Ki=5, Kd=2)

    # 记录每次迭代的误差
    iae_history = []
    ise_history = []
    max_error_history = []
    all_y_trajs = []  # 保存部分轨迹用于绘图
    all_u_trajs = []

    save_iterations = [0, 2, 4, 7, 10, 14]  # 保存这些迭代的轨迹

    for k in range(n_iterations):
        state = np.array([0.0, 0.0])  # 每次迭代从相同初始状态开始
        pid_base.reset()

        y_traj = np.zeros(N_cycle)
        u_traj = np.zeros(N_cycle)
        e_traj = np.zeros(N_cycle)

        for i in range(N_cycle):
            ref = ref_traj[i]
            y = state[0]

            # PID反馈 + ILC前馈
            u_pid = pid_base.compute(ref, y, dt)
            u_ilc = ilc.u_prev[i]
            u = u_pid + u_ilc

            # 限幅
            u = np.clip(u, -100, 100)

            # 状态更新
            state = arm_dynamics(state, u, dt)

            y_traj[i] = state[0]
            u_traj[i] = u
            e_traj[i] = ref - y

        # ILC学习
        ilc.learn(e_traj)

        # 计算指标
        iae = np.sum(np.abs(e_traj)) * dt
        ise = np.sum(e_traj**2) * dt
        max_err = np.max(np.abs(e_traj))

        iae_history.append(iae)
        ise_history.append(ise)
        max_error_history.append(max_err)

        if k in save_iterations:
            all_y_trajs.append((k, y_traj.copy()))

        if k % 3 == 0:
            print(f"迭代 {k:2d}: IAE={iae:.4f}, ISE={ise:.6f}, 最大误差={max_err:.4f} rad")

    print(f"\n最终迭代: IAE={iae_history[-1]:.4f}, 最大误差={max_error_history[-1]:.4f} rad")
    print(f"精度提升: {iae_history[0]/iae_history[-1]:.1f}倍")

    # ============ 绘图 ============
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    fig.suptitle('ILC迭代学习控制仿真', fontsize=16, fontweight='bold')

    # 不同迭代次数的轨迹跟踪
    ax = axes[0, 0]
    ax.plot(t_cycle, ref_traj, 'k--', linewidth=2, label='参考轨迹')
    colors = plt.cm.viridis(np.linspace(0, 1, len(all_y_trajs)))
    for idx, (iter_num, y_traj) in enumerate(all_y_trajs):
        ax.plot(t_cycle, y_traj, '-', color=colors[idx], linewidth=1.5, 
                label=f'迭代 {iter_num+1}')
    ax.set_title('不同迭代次数的跟踪轨迹')
    ax.set_xlabel('时间 (s)'); ax.set_ylabel('角度 (rad)')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    # IAE学习曲线
    ax = axes[0, 1]
    iterations = np.arange(1, n_iterations + 1)
    ax.plot(iterations, iae_history, 'ro-', linewidth=2, markersize=6, label='IAE')
    ax.set_title('IAE学习曲线（收敛性）')
    ax.set_xlabel('迭代次数'); ax.set_ylabel('IAE')
    ax.legend(); ax.grid(True, alpha=0.3)
    ax.set_yscale('log')

    # ISE学习曲线
    ax = axes[1, 0]
    ax.plot(iterations, ise_history, 'bs-', linewidth=2, markersize=6, label='ISE')
    ax.set_title('ISE学习曲线')
    ax.set_xlabel('迭代次数'); ax.set_ylabel('ISE')
    ax.legend(); ax.grid(True, alpha=0.3)
    ax.set_yscale('log')

    # 最大误差
    ax = axes[1, 1]
    ax.plot(iterations, np.degrees(max_error_history), 'g^-', linewidth=2, markersize=6, label='最大跟踪误差')
    ax.set_title('最大跟踪误差收敛')
    ax.set_xlabel('迭代次数'); ax.set_ylabel('最大误差 (°)')
    ax.legend(); ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'ilc_repetitive_task_result.png'), dpi=150, bbox_inches='tight')
    print("\n图表已保存: ilc_repetitive_task_result.png")
    plt.close('all')
    print("ILC迭代学习仿真完成!")



if __name__ == '__main__':
    main()
