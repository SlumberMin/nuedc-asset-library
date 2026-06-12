#!/usr/bin/env python3
"""
机械臂运动学仿真 — 正运动学 + 逆运动学 + 轨迹规划
====================================================
- 3-DOF平面机械臂
- DH参数 / 正运动学
- 数值逆运动学（Jacobian迭代）
- 多种轨迹规划（关节空间/笛卡尔空间）
- 可视化动画
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rcParams
rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
rcParams['axes.unicode_minus'] = False

# ============================================================
# 1. 3-DOF 平面机械臂运动学
# ============================================================
class PlanarArm3DOF:
    """
    3自由度平面机械臂
    关节1: 肩关节, 关节2: 肘关节, 关节3: 腕关节
    """
    def __init__(self, L1=1.0, L2=0.8, L3=0.5):
        self.L = [L1, L2, L3]
        self.n_joints = 3

    def forward_kinematics(self, q):
        """
        正运动学: q=[q1,q2,q3] -> 各关节位置
        返回: (3,2)数组, 每行一个关节的(x,y)
        """
        L = self.L
        positions = np.zeros((4, 2))  # 原点 + 3关节
        theta = 0
        for i in range(3):
            theta += q[i]
            positions[i+1, 0] = positions[i, 0] + L[i] * np.cos(theta)
            positions[i+1, 1] = positions[i, 1] + L[i] * np.sin(theta)
        return positions

    def end_effector(self, q):
        """末端位置"""
        pos = self.forward_kinematics(q)
        return pos[-1]

    def jacobian(self, q, delta=1e-6):
        """数值Jacobian"""
        J = np.zeros((2, 3))
        p0 = self.end_effector(q)
        for i in range(3):
            q_d = q.copy()
            q_d[i] += delta
            J[:, i] = (self.end_effector(q_d) - p0) / delta
        return J

    def inverse_kinematics(self, target, q0=None, max_iter=200, tol=1e-4):
        """
        数值逆运动学 (阻尼最小二乘 / Levenberg-Marquardt)
        target: [x, y]
        """
        if q0 is None:
            q0 = np.array([0.5, 0.5, 0.5])
        q = q0.copy()
        lam = 0.1  # 阻尼因子

        for _ in range(max_iter):
            pos = self.end_effector(q)
            err = target - pos
            if np.linalg.norm(err) < tol:
                break
            J = self.jacobian(q)
            # LM更新
            dq = J.T @ np.linalg.solve(J @ J.T + lam**2 * np.eye(2), err)
            q += dq
        return q

    def workspace_boundary(self, n_points=500):
        """采样工作空间边界"""
        points = []
        for q1 in np.linspace(-np.pi, np.pi, 60):
            for q2 in np.linspace(-np.pi, np.pi, 60):
                for q3 in np.linspace(-np.pi/2, np.pi/2, 30):
                    p = self.end_effector(np.array([q1, q2, q3]))
                    points.append(p)
        return np.array(points)


# ============================================================
# 2. 轨迹规划
# ============================================================
class TrajectoryPlanner:
    """轨迹规划器"""

    @staticmethod
    def quintic_poly(t, t_f, q0, qf):
        """五次多项式（关节空间）"""
        tau = t / t_f
        s = 10*tau**3 - 15*tau**4 + 6*tau**5
        s_dot = (30*tau**2 - 60*tau**3 + 30*tau**4) / t_f
        s_ddot = (60*tau - 180*tau**2 + 120*tau**3) / t_f**2
        q = q0 + (qf - q0) * s
        q_dot = (qf - q0) * s_dot
        q_ddot = (qf - q0) * s_ddot
        return q, q_dot, q_ddot

    @staticmethod
    def linear_cartesian(t, t_f, p0, pf):
        """直线轨迹（笛卡尔空间）"""
        tau = t / t_f
        s = 10*tau**3 - 15*tau**4 + 6*tau**5
        p = p0 + (pf - p0) * s
        return p

    @staticmethod
    def circular_path(t, t_f, center, radius, n_cycles=1):
        """圆形轨迹"""
        theta = 2 * np.pi * n_cycles * t / t_f
        x = center[0] + radius * np.cos(theta)
        y = center[1] + radius * np.sin(theta)
        return np.array([x, y])

    @staticmethod
    def via_point_trajectory(t_points, q_via, t):
        """多途经点轨迹（三次样条简化版）"""
        from scipy.interpolate import CubicSpline
        cs = CubicSpline(t_points, q_via, bc_type='clamped')
        q = cs(t)
        q_dot = cs(t, 1)
        q_ddot = cs(t, 2)
        return q, q_dot, q_ddot


# ============================================================
# 3. 仿真测试
# ============================================================
def run_fk_demo(arm):
    """正运动学演示"""
    q = np.array([np.pi/4, -np.pi/3, np.pi/6])
    positions = arm.forward_kinematics(q)
    return positions, q


def run_ik_demo(arm):
    """逆运动学演示：跟踪圆形轨迹"""
    t = np.linspace(0, 4*np.pi, 200)
    center = np.array([1.2, 0.5])
    radius = 0.3

    target_x = center[0] + radius * np.cos(t)
    target_y = center[1] + radius * np.sin(t)

    q_traj = np.zeros((len(t), 3))
    q = np.array([0.5, 0.3, 0.2])
    for i in range(len(t)):
        q = arm.inverse_kinematics(np.array([target_x[i], target_y[i]]), q0=q)
        q_traj[i] = q

    return t, q_traj, target_x, target_y


def run_trajectory_demo(arm):
    """轨迹规划演示"""
    tp = TrajectoryPlanner()
    t = np.linspace(0, 5, 300)

    # 关节空间：从A到B
    q_start = np.array([0.2, 0.5, -0.3])
    q_end = np.array([1.2, -0.5, 0.5])
    q_traj = np.zeros((len(t), 3))
    for j in range(3):
        q, qd, qdd = tp.quintic_poly(t, 5.0, q_start[j], q_end[j])
        q_traj[:, j] = q

    return t, q_traj


# ============================================================
# 4. 可视化
# ============================================================
def plot_all(arm, fk_result, ik_result, traj_result):
    fig, axes = plt.subplots(2, 2, figsize=(14, 12))
    fig.suptitle('机械臂运动学仿真', fontsize=15, fontweight='bold')

    # (a) 正运动学
    ax = axes[0, 0]
    positions, q = fk_result
    for i in range(3):
        ax.plot([positions[i, 0], positions[i+1, 0]],
                [positions[i, 1], positions[i+1, 1]], 'o-', lw=3, ms=8,
                color=['#2196F3', '#4CAF50', '#FF5722'][i])
    ax.plot(0, 0, 'ks', ms=12)
    ax.set_xlim(-2.5, 2.5)
    ax.set_ylim(-2.5, 2.5)
    ax.set_aspect('equal')
    ax.set_title(f'(a) 正运动学 q=[{np.degrees(q[0]):.0f}°,{np.degrees(q[1]):.0f}°,{np.degrees(q[2]):.0f}°]')
    ax.grid(True, alpha=0.3)

    # (b) 逆运动学跟踪
    ax = axes[0, 1]
    t_ik, q_traj, tx, ty = ik_result
    # 画目标圆
    ax.plot(tx, ty, 'r--', lw=2, label='目标轨迹')
    # 画实际末端轨迹
    ee_x, ee_y = [], []
    for i in range(0, len(t_ik), 5):
        p = arm.end_effector(q_traj[i])
        ee_x.append(p[0]); ee_y.append(p[1])
    ax.plot(ee_x, ee_y, 'b-', lw=1.5, label='实际轨迹')
    ax.set_aspect('equal')
    ax.set_title('(b) 逆运动学 — 圆形轨迹跟踪')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # (c) 关节角度轨迹
    ax = axes[1, 0]
    t_traj, q_traj2 = traj_result
    for j in range(3):
        ax.plot(t_traj, np.degrees(q_traj2[:, j]), lw=2,
                label=f'关节{j+1}')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('关节角度 (°)')
    ax.set_title('(c) 关节空间轨迹规划（五次多项式）')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # (d) 机械臂多姿态叠加
    ax = axes[1, 1]
    colors = plt.cm.viridis(np.linspace(0, 1, 20))
    for i, idx in enumerate(np.linspace(0, len(t_ik)-1, 20, dtype=int)):
        pos = arm.forward_kinematics(q_traj[idx])
        ax.plot(pos[:, 0], pos[:, 1], '-o', color=colors[i], lw=1.5,
                ms=4, alpha=0.6)
    ax.plot(tx, ty, 'r--', lw=1, label='目标')
    ax.set_aspect('equal')
    ax.set_title('(d) 机械臂运动叠加')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('robot_arm_kinematics.png', dpi=150, bbox_inches='tight')
    plt.show()


# ============================================================
if __name__ == '__main__':
    arm = PlanarArm3DOF(L1=1.0, L2=0.8, L3=0.5)

    print('1. 正运动学演示...')
    fk = run_fk_demo(arm)

    print('2. 逆运动学 — 圆形轨迹跟踪...')
    ik = run_ik_demo(arm)

    print('3. 轨迹规划演示...')
    traj = run_trajectory_demo(arm)

    plot_all(arm, fk, ik, traj)
    print('\n机械臂运动学仿真完成！')
