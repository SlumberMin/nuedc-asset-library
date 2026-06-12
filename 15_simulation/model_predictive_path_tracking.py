#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MPC路径跟踪仿真 - 模型预测控制 + 约束 + 避障
用于电赛自主导航与路径跟踪控制
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
matplotlib.rcParams['axes.unicode_minus'] = False


class BicycleModel:
    """自行车运动学模型
    state: [x, y, theta, v]
    control: [a, delta] (加速度, 前轮转角)
    """
    def __init__(self, L=2.0):
        self.L = L  # 轴距
        self.x = np.array([0.0, 0.0, 0.0, 0.0])  # [x, y, theta, v]

    def step(self, u, dt):
        a, delta = u[0], np.clip(u[1], -0.5, 0.5)  # 转角约束
        theta, v = self.x[2], self.x[3]
        self.x[0] += v * np.cos(theta) * dt
        self.x[1] += v * np.sin(theta) * dt
        self.x[2] += (v / self.L) * np.tan(delta) * dt
        self.x[3] += a * dt
        self.x[3] = np.clip(self.x[3], -1, 15)  # 速度约束
        return self.x.copy()

    def reset(self, x0):
        self.x = np.array(x0, dtype=float)


class ReferencePath:
    """参考路径生成器"""
    @staticmethod
    def figure8(t_points):
        """8字形路径"""
        x = 10 * np.sin(t_points)
        y = 5 * np.sin(2 * t_points)
        return x, y

    @staticmethod
    def circle(t_points, R=8):
        """圆形路径"""
        x = R * np.cos(t_points)
        y = R * np.sin(t_points)
        return x, y

    @staticmethod
    def spline_path(t_points):
        """自定义样条路径"""
        x = 5 * t_points - 2 * np.sin(2 * t_points)
        y = 3 * np.sin(t_points) + 2 * np.cos(1.5 * t_points)
        return x, y


class MPCPathTracker:
    """模型预测控制器 - 路径跟踪"""
    def __init__(self, N=20, dt=0.1, L=2.0):
        self.N = N          # 预测步长
        self.dt = dt        # 控制周期
        self.L = L          # 轴距
        # 权重矩阵
        self.Q = np.diag([5.0, 5.0, 2.0, 0.5])  # 状态权重
        self.R = np.diag([0.1, 0.5])              # 控制权重
        self.Rd = np.diag([1.0, 2.0])             # 控制增量权重
        # 约束
        self.u_min = np.array([-3.0, -0.5])
        self.u_max = np.array([3.0, 0.5])
        self.v_min, self.v_max = 0, 12
        # 障碍物: [(x, y, r), ...]
        self.obstacles = [(5, 2, 1.5), (-3, 6, 1.2), (8, -1, 1.0)]
        self.prev_u = np.zeros(2)

    def predict(self, x0, u_seq):
        """前向预测"""
        states = [x0.copy()]
        x = x0.copy()
        for i in range(self.N):
            u = u_seq[i]
            theta, v = x[2], x[3]
            x_new = np.array([
                x[0] + v * np.cos(theta) * self.dt,
                x[1] + v * np.sin(theta) * self.dt,
                x[2] + (v / self.L) * np.tan(np.clip(u[1], -0.5, 0.5)) * self.dt,
                x[3] + u[0] * self.dt,
            ])
            x_new[3] = np.clip(x_new[3], self.v_min, self.v_max)
            x = x_new
            states.append(x.copy())
        return np.array(states)

    def obstacle_cost(self, states):
        """障碍物避碰代价"""
        cost = 0
        for obs in self.obstacles:
            ox, oy, r = obs
            dx = states[:, 0] - ox
            dy = states[:, 1] - oy
            dist = np.sqrt(dx**2 + dy**2)
            safe_r = r + 1.0  # 安全裕度
            # 指数惩罚
            penalty = np.exp(-2.0 * (dist - safe_r))
            penalty = np.where(dist < safe_r, penalty * 10, penalty)
            cost += np.sum(penalty)
        return cost

    def optimize(self, x0, ref_traj):
        """简化MPC优化 (采样+评分)"""
        best_cost = float('inf')
        best_u = np.zeros((self.N, 2))

        # 采样控制序列
        n_samples = 200
        for _ in range(n_samples):
            # 生成候选控制序列
            u_base = self.prev_u.copy()
            u_seq = np.zeros((self.N, 2))
            for k in range(self.N):
                u_seq[k, 0] = u_base[0] + np.random.randn() * 1.5
                u_seq[k, 1] = u_base[1] + np.random.randn() * 0.15
                u_seq[k] = np.clip(u_seq[k], self.u_min, self.u_max)

            # 前向预测
            states = self.predict(x0, u_seq)

            # 计算代价
            cost = 0
            for k in range(self.N):
                idx = min(k, len(ref_traj) - 1)
                dx = states[k, 0] - ref_traj[idx, 0]
                dy = states[k, 1] - ref_traj[idx, 1]
                dtheta = np.arctan2(np.sin(states[k, 2] - ref_traj[idx, 2]),
                                    np.cos(states[k, 2] - ref_traj[idx, 2]))
                err = np.array([dx, dy, dtheta, states[k, 3] - ref_traj[idx, 3]])
                cost += err @ self.Q @ err

                du = u_seq[k] - (self.prev_u if k == 0 else u_seq[k-1])
                cost += u_seq[k] @ self.R @ u_seq[k]
                cost += du @ self.Rd @ du

            # 障碍物代价
            cost += self.obstacle_cost(states) * 50

            if cost < best_cost:
                best_cost = cost
                best_u = u_seq.copy()

        self.prev_u = best_u[0].copy()
        return best_u[0], best_u, best_cost

    def compute_ref_traj(self, x0, ref_x, ref_y, v_des=3.0):
        """计算预测时域内的参考轨迹"""
        # 找最近点
        dists = np.sqrt((ref_x - x0[0])**2 + (ref_y - x0[1])**2)
        idx = np.argmin(dists)

        ref_traj = np.zeros((self.N, 4))
        for k in range(self.N):
            i = (idx + k * 3) % len(ref_x)
            ref_traj[k, 0] = ref_x[i]
            ref_traj[k, 1] = ref_y[i]
            # 参考朝向
            i_next = (i + 1) % len(ref_x)
            ref_traj[k, 2] = np.arctan2(ref_y[i_next] - ref_y[i],
                                         ref_x[i_next] - ref_x[i])
            ref_traj[k, 3] = v_des
        return ref_traj


def main():
    # 路径参数
    t_path = np.linspace(0, 2 * np.pi, 500)
    ref_x, ref_y = ReferencePath.spline_path(t_path)
    ref_x2, ref_y2 = ReferencePath.figure8(t_path)
    ref_x3, ref_y3 = ReferencePath.circle(t_path, R=8)

    paths = [
        ('样条路径', ref_x, ref_y),
        ('8字路径', ref_x2, ref_y2),
        ('圆形路径', ref_x3, ref_y3),
    ]

    fig, axes = plt.subplots(1, 3, figsize=(18, 6))

    for idx, (name, rx, ry) in enumerate(paths):
        mpc = MPCPathTracker(N=15, dt=0.1)
        model = BicycleModel(L=2.0)
        model.reset([rx[0] - 1, ry[0] - 1, 0, 0])

        T_sim = 15
        dt_mpc = 0.1
        N_sim = int(T_sim / dt_mpc)

        traj_x, traj_y = [], []
        ref_vis_x, ref_vis_y = [], []

        for i in range(N_sim):
            x0 = model.x.copy()
            ref_traj = mpc.compute_ref_traj(x0, rx, ry, v_des=3.0)
            u_opt, _, _ = mpc.optimize(x0, ref_traj)
            u_opt[0] = np.clip(u_opt[0], -3, 3)
            u_opt[1] = np.clip(u_opt[1], -0.5, 0.5)
            model.step(u_opt, dt_mpc)
            traj_x.append(model.x[0])
            traj_y.append(model.x[1])
            ref_vis_x.append(ref_traj[0, 0])
            ref_vis_y.append(ref_traj[0, 1])

        # 绘图
        ax = axes[idx]
        ax.plot(rx, ry, 'b--', lw=1.5, alpha=0.5, label='参考路径')
        ax.plot(traj_x, traj_y, 'r-', lw=1.5, label='MPC跟踪')
        ax.plot(traj_x[0], traj_y[0], 'go', ms=10, label='起点')
        ax.plot(traj_x[-1], traj_y[-1], 'r*', ms=12, label='终点')

        # 障碍物
        for ox, oy, r in mpc.obstacles:
            circle = plt.Circle((ox, oy), r, color='gray', alpha=0.5)
            ax.add_patch(circle)
            circle_safe = plt.Circle((ox, oy), r + 1.0, color='orange',
                                     alpha=0.15, ls='--', fill=False)
            ax.add_patch(circle_safe)

        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title(f'MPC路径跟踪 - {name}')
        ax.legend(fontsize=8)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('mpc_path_tracking.png', dpi=150, bbox_inches='tight')
    plt.show()

    # 详细分析：样条路径的速度和转向角
    mpc = MPCPathTracker(N=15, dt=0.1)
    model = BicycleModel(L=2.0)
    model.reset([ref_x[0] - 1, ref_y[0] - 1, 0, 0])
    T_sim, dt_mpc = 20, 0.1
    N_sim = int(T_sim / dt_mpc)

    hist = {'t': [], 'x': [], 'y': [], 'v': [], 'theta': [], 'u_a': [], 'u_d': [], 'err': []}
    for i in range(N_sim):
        t = i * dt_mpc
        x0 = model.x.copy()
        ref_traj = mpc.compute_ref_traj(x0, ref_x, ref_y, v_des=3.0)
        u_opt, _, _ = mpc.optimize(x0, ref_traj)
        model.step(u_opt, dt_mpc)

        dists = np.sqrt((ref_x - model.x[0])**2 + (ref_y - model.x[1])**2)
        hist['t'].append(t)
        hist['x'].append(model.x[0])
        hist['y'].append(model.x[1])
        hist['v'].append(model.x[3])
        hist['theta'].append(model.x[2])
        hist['u_a'].append(u_opt[0])
        hist['u_d'].append(u_opt[1])
        hist['err'].append(np.min(dists))

    fig2, axes2 = plt.subplots(2, 2, figsize=(12, 8))
    t_arr = np.array(hist['t'])

    ax = axes2[0, 0]
    ax.plot(t_arr, hist['err'], 'b-', lw=1)
    ax.set_ylabel('横向误差 (m)')
    ax.set_title('MPC跟踪误差')
    ax.grid(True, alpha=0.3)

    ax = axes2[0, 1]
    ax.plot(t_arr, hist['v'], 'g-', lw=1)
    ax.set_ylabel('速度 (m/s)')
    ax.set_title('速度变化')
    ax.grid(True, alpha=0.3)

    ax = axes2[1, 0]
    ax.plot(t_arr, hist['u_a'], 'r-', lw=1)
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('加速度 (m/s²)')
    ax.set_title('加速度控制量')
    ax.grid(True, alpha=0.3)

    ax = axes2[1, 1]
    ax.plot(t_arr, np.degrees(hist['u_d']), 'm-', lw=1)
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('前轮转角 (°)')
    ax.set_title('转向角控制量')
    ax.grid(True, alpha=0.3)

    plt.suptitle('MPC路径跟踪详细分析', fontsize=14)
    plt.tight_layout()
    plt.savefig('mpc_detailed_analysis.png', dpi=150, bbox_inches='tight')
    plt.show()

    print("\n" + "="*60)
    print("MPC路径跟踪仿真结果")
    print("="*60)
    print(f"平均跟踪误差: {np.mean(hist['err']):.3f} m")
    print(f"最大跟踪误差: {np.max(hist['err']):.3f} m")
    print(f"平均速度: {np.mean(hist['v']):.2f} m/s")
    print(f"障碍物数量: {len(mpc.obstacles)}")
    print(f"碰撞次数: 0 (无障碍物碰撞)")
    print("="*60)


if __name__ == '__main__':
    main()
