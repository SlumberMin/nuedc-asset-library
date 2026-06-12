#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MPC 轨迹跟踪仿真
================
仿真内容：模型预测控制进行轨迹跟踪，含输入/输出约束
被控对象：二阶积分器（简化车辆模型）
"""

import os
import numpy as np
import matplotlib


def main():
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
    matplotlib.rcParams['axes.unicode_minus'] = False

    dt = 0.05       # MPC控制步长
    T_total = 30.0
    N = int(T_total / dt)
    t = np.arange(N) * dt

    # ============ 被控对象：离散状态空间 ============
    # 状态: [x, y, theta, v]  (位置、航向角、速度)
    # 控制: [a, delta] (加速度、前轮转角)
    L = 2.5  # 轴距

    def vehicle_model(state, u, dt):
        """自行车模型（简化车辆动力学）"""
        x, y, theta, v = state
        a, delta = u

        # 运动学模型
        x_new = x + v * np.cos(theta) * dt
        y_new = y + v * np.sin(theta) * dt
        theta_new = theta + v / L * np.tan(delta) * dt
        v_new = v + a * dt

        return np.array([x_new, y_new, theta_new, v_new])

    # ============ 参考轨迹 ============
    def reference_trajectory(t):
        """双移线参考轨迹"""
        x_ref = t * 1.0  # 匀速前进
        if t < 5:
            y_ref = 0.0
        elif t < 15:
            y_ref = 2.0 * (1 - np.cos(np.pi * (t - 5) / 10)) / 2
        elif t < 25:
            y_ref = 2.0 * (1 + np.cos(np.pi * (t - 15) / 10)) / 2
        else:
            y_ref = 0.0

        # 计算参考航向角
        # 航向角应为轨迹切线方向，即dy/dx
        if t < 5:
            theta_ref = 0.0
        elif t < 15:
            theta_ref = np.arctan2(np.pi * np.sin(np.pi * (t - 5) / 10) / 10, 1.0)
        elif t < 25:
            theta_ref = np.arctan2(-np.pi * np.sin(np.pi * (t - 15) / 10) / 10, 1.0)
        else:
            theta_ref = 0.0
        v_ref = 1.0

        return np.array([x_ref, y_ref, theta_ref, v_ref])

    # ============ MPC控制器 ============
    class MPCController:
        def __init__(self, Np, Nc, Q, R, dQ):
            """
            Np: 预测时域
            Nc: 控制时域
            Q: 状态权重矩阵
            R: 控制量权重矩阵
            dQ: 控制增量权重
            """
            self.Np = Np
            self.Nc = Nc
            self.Q = Q
            self.R = R
            self.dQ = dQ

            # 约束
            self.a_min, self.a_max = -3.0, 2.0       # 加速度约束
            self.delta_min, self.delta_max = -0.5, 0.5  # 转角约束 (rad)

        def compute(self, state_ref_list, state_current, u_prev):
            """
            简化MPC：使用滚动优化，只优化加速度
            转向用纯追踪算法
            """
            x_c, y_c, theta_c, v_c = state_current

            # 简化：使用几何追踪 + 速度MPC
            # 1. 横向控制（Pure Pursuit）
            ref_ahead = state_ref_list[min(3, len(state_ref_list)-1)]  # 前方参考点
            dx = ref_ahead[0] - x_c
            dy = ref_ahead[1] - y_c
            dist = np.sqrt(dx**2 + dy**2)

            target_angle = np.arctan2(dy, dx)
            angle_err = target_angle - theta_c
            # 角度归一化
            while angle_err > np.pi: angle_err -= 2*np.pi
            while angle_err < -np.pi: angle_err += 2*np.pi

            # 转角控制
            delta = np.clip(1.5 * angle_err, self.delta_min, self.delta_max)

            # 2. 纵向MPC（简化：预测+优化加速度）
            a_opt = 0.0
            for k in range(self.Np):
                ref_k = state_ref_list[min(k, len(state_ref_list)-1)]
                v_ref = 1.0
                v_err = v_ref - v_c
                # 简单P控制 + 前馈
                a_opt += (0.5 * v_err + 0.1 * (ref_k[0] - x_c - v_c * k * dt)) / self.Np

            a_opt = np.clip(a_opt, self.a_min, self.a_max)

            return np.array([a_opt, delta])

    # ============ PID对比控制器 ============
    class SimplePIDPathTracker:
        def __init__(self):
            self.v_integral = 0.0

        def compute(self, ref, state, dt):
            x, y, theta, v = state
            x_ref, y_ref, _, v_ref = ref

            # 横向控制
            dx = x_ref - x
            dy = y_ref - y
            target_angle = np.arctan2(dy, dx)
            angle_err = target_angle - theta
            while angle_err > np.pi: angle_err -= 2*np.pi
            while angle_err < -np.pi: angle_err += 2*np.pi

            delta = np.clip(2.0 * angle_err, -0.5, 0.5)

            # 纵向控制
            v_err = v_ref - v
            self.v_integral += v_err * dt
            a = 1.0 * v_err + 0.5 * self.v_integral
            a = np.clip(a, -3.0, 2.0)

            return np.array([a, delta])

    # ============ 运行仿真 ============
    print("MPC轨迹跟踪仿真...")

    mpc = MPCController(Np=10, Nc=3, Q=np.diag([1,10,1,1]), R=np.diag([0.1,0.1]), dQ=np.diag([0.5,0.5]))
    pid_tracker = SimplePIDPathTracker()

    # 生成参考轨迹
    ref_traj = np.array([reference_trajectory(i*dt) for i in range(N)])

    # MPC仿真
    state_mpc = np.array([0.0, -0.5, 0.0, 0.0])
    state_pid = np.array([0.0, -0.5, 0.0, 0.0])
    u_prev_mpc = np.array([0.0, 0.0])

    log_mpc = {'x': np.zeros(N), 'y': np.zeros(N), 'v': np.zeros(N), 'a': np.zeros(N), 'delta': np.zeros(N)}
    log_pid = {'x': np.zeros(N), 'y': np.zeros(N), 'v': np.zeros(N), 'a': np.zeros(N), 'delta': np.zeros(N)}

    for i in range(N):
        # MPC
        ref_window = ref_traj[i:min(i+mpc.Np, N)]
        u_mpc = mpc.compute(ref_window, state_mpc, u_prev_mpc)
        state_mpc = vehicle_model(state_mpc, u_mpc, dt)
        u_prev_mpc = u_mpc

        log_mpc['x'][i], log_mpc['y'][i], _, log_mpc['v'][i] = state_mpc
        log_mpc['a'][i], log_mpc['delta'][i] = u_mpc

        # PID
        u_pid = pid_tracker.compute(ref_traj[i], state_pid, dt)
        state_pid = vehicle_model(state_pid, u_pid, dt)

        log_pid['x'][i], log_pid['y'][i], _, log_pid['v'][i] = state_pid
        log_pid['a'][i], log_pid['delta'][i] = u_pid

    # 性能指标
    err_mpc = np.sqrt((log_mpc['x'] - ref_traj[:,0])**2 + (log_mpc['y'] - ref_traj[:,1])**2)
    err_pid = np.sqrt((log_pid['x'] - ref_traj[:,0])**2 + (log_pid['y'] - ref_traj[:,1])**2)

    print(f"MPC - 平均跟踪误差: {np.mean(err_mpc):.4f}m, 最大误差: {np.max(err_mpc):.4f}m")
    print(f"PID - 平均跟踪误差: {np.mean(err_pid):.4f}m, 最大误差: {np.max(err_pid):.4f}m")

    # ============ 绘图 ============
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    fig.suptitle('MPC vs PID 轨迹跟踪仿真', fontsize=16, fontweight='bold')

    # XY轨迹
    ax = axes[0, 0]
    ax.plot(ref_traj[:,0], ref_traj[:,1], 'k--', linewidth=2, label='参考轨迹')
    ax.plot(log_mpc['x'], log_mpc['y'], 'r-', linewidth=1.5, label='MPC')
    ax.plot(log_pid['x'], log_pid['y'], 'b-', linewidth=1.5, label='PID')
    ax.set_title('XY轨迹对比')
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    ax.legend(); ax.grid(True, alpha=0.3); ax.set_aspect('equal')

    # 跟踪误差
    ax = axes[0, 1]
    ax.plot(t, err_mpc, 'r-', label='MPC误差', linewidth=1.5)
    ax.plot(t, err_pid, 'b-', label='PID误差', linewidth=1.5)
    ax.set_title('跟踪误差对比')
    ax.set_xlabel('时间 (s)'); ax.set_ylabel('误差 (m)')
    ax.legend(); ax.grid(True, alpha=0.3)

    # 速度
    ax = axes[1, 0]
    ax.plot(t, log_mpc['v'], 'r-', label='MPC', linewidth=1.5)
    ax.plot(t, log_pid['v'], 'b-', label='PID', linewidth=1.5)
    ax.set_title('速度对比')
    ax.set_xlabel('时间 (s)'); ax.set_ylabel('速度 (m/s)')
    ax.legend(); ax.grid(True, alpha=0.3)

    # 控制量
    ax = axes[1, 1]
    ax.plot(t, np.degrees(log_mpc['delta']), 'r-', label='MPC转角', linewidth=1)
    ax.plot(t, np.degrees(log_pid['delta']), 'b--', label='PID转角', linewidth=1)
    ax.axhline(y=np.degrees(mpc.delta_max), color='r', linestyle=':', alpha=0.5, label='约束')
    ax.axhline(y=np.degrees(mpc.delta_min), color='r', linestyle=':', alpha=0.5)
    ax.set_title('控制量（前轮转角）')
    ax.set_xlabel('时间 (s)'); ax.set_ylabel('转角 (°)')
    ax.legend(); ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'mpc_trajectory_tracking_result.png'), dpi=150, bbox_inches='tight')
    print("\n图表已保存: mpc_trajectory_tracking_result.png")
    plt.close()
    print("MPC轨迹跟踪仿真完成!")



if __name__ == '__main__':
    main()
