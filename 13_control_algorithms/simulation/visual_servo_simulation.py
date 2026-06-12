"""
视觉伺服仿真(IBVS / PBVS)
============================
仿真相机跟踪目标特征点或位姿的收敛过程。
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib

matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial Unicode MS']
matplotlib.rcParams['axes.unicode_minus'] = False


def interaction_matrix_point(u, v, Z, fx, fy, cx, cy):
    """计算单个特征点的交互矩阵 (2x6)"""
    ui, vi = u - cx, v - cy
    L = np.array([
        [-fx/Z,  0,     ui/Z,   ui*vi/fx,   -(fx + ui**2/fx),  vi],
        [0,     -fy/Z,  vi/Z,   (fy + vi**2/fy), -ui*vi/fy,    -ui]
    ])
    return L


class IBVSSimulator:
    """基于图像的视觉伺服(IBVS)仿真"""

    def __init__(self, n_features=4, fx=500, fy=500, cx=320, cy=240,
                 lambda_gain=0.5, depth_est=1.0):
        self.fx, self.fy = fx, fy
        self.cx, self.cy = cx, cy
        self.lambda_gain = lambda_gain
        self.Z = depth_est
        self.n = n_features

        # 期望特征点(正方形排列)
        s = 40.0
        self.desired = np.array([
            [cx - s, cy - s],
            [cx + s, cy - s],
            [cx + s, cy + s],
            [cx - s, cy + s]
        ], dtype=float)

        # 当前特征点(带偏移)
        offset = 80.0
        self.current = np.array([
            [cx - s + offset, cy - s - offset],
            [cx + s + offset + 20, cy - s - offset + 10],
            [cx + s + offset + 30, cy + s - offset + 40],
            [cx - s + offset - 10, cy + s - offset + 20]
        ], dtype=float)

        self.error_hist = []
        self.feature_trails = [[] for _ in range(n_features)]

    def step(self):
        """单步IBVS控制"""
        error = self.current - self.desired
        self.error_hist.append(np.linalg.norm(error))

        # 记录轨迹
        for i in range(self.n):
            self.feature_trails[i].append(self.current[i].copy())

        # 组装交互矩阵
        L_stack = []
        for i in range(self.n):
            L_stack.append(interaction_matrix_point(
                self.current[i, 0], self.current[i, 1],
                self.Z, self.fx, self.fy, self.cx, self.cy))
        L = np.vstack(L_stack)  # (2n x 6)
        e = error.flatten()      # (2n,)

        # 伪逆求速度
        L_pinv = np.linalg.pinv(L)
        v_cam = -self.lambda_gain * L_pinv @ e  # (6,)

        # 模拟运动: 特征点在图像上的变化 (s_dot = L * v)
        s_dot = L @ v_cam
        self.current += s_dot.reshape(self.n, 2) * 0.1  # dt=0.1

        return v_cam

    def run(self, max_iter=100, threshold=1.0):
        for i in range(max_iter):
            v = self.step()
            if self.error_hist[-1] < threshold:
                print(f'IBVS 收敛于第 {i} 次迭代, 误差={self.error_hist[-1]:.4f}')
                return
        print(f'IBVS 未收敛, 最终误差={self.error_hist[-1]:.4f}')


class PBVSSimulator:
    """基于位置的视觉伺服(PBVS)仿真"""

    def __init__(self, lambda_pos=1.0, lambda_rot=0.8):
        self.lambda_pos = lambda_pos
        self.lambda_rot = lambda_rot

        # 期望位姿 [x, y, z, roll, pitch, yaw]
        self.desired_pose = np.array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
        # 当前位姿(偏移)
        self.current_pose = np.array([0.5, -0.3, 1.8, 0.2, -0.15, 0.3])

        self.error_pos_hist = []
        self.error_rot_hist = []
        self.pose_hist = []

    def step(self, dt=0.05):
        error = self.current_pose - self.desired_pose
        # 角度归一化
        error[3:] = np.arctan2(np.sin(error[3:]), np.cos(error[3:]))

        self.error_pos_hist.append(np.linalg.norm(error[:3]))
        self.error_rot_hist.append(np.linalg.norm(error[3:]))
        self.pose_hist.append(self.current_pose.copy())

        v = np.zeros(6)
        v[:3] = -self.lambda_pos * error[:3]
        v[3:] = -self.lambda_rot * error[3:]
        v = np.clip(v, -2.0, 2.0)

        self.current_pose += v * dt
        return v

    def run(self, max_iter=300, threshold=0.01):
        for i in range(max_iter):
            self.step()
            total_err = self.error_pos_hist[-1] + self.error_rot_hist[-1]
            if total_err < threshold:
                print(f'PBVS 收敛于第 {i} 次迭代')
                return
        print(f'PBVS 未收敛, 最终位置误差={self.error_pos_hist[-1]:.4f}')


def run_ibvs_simulation():
    print('=== IBVS 视觉伺服仿真 ===')
    sim = IBVSSimulator(n_features=4, lambda_gain=0.5, depth_est=1.0)
    sim.run(max_iter=150)

    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    fig.suptitle('IBVS 基于图像的视觉伺服仿真', fontsize=14, fontweight='bold')

    # 特征点运动轨迹
    ax = axes[0]
    colors = ['r', 'g', 'b', 'orange']
    for i in range(sim.n):
        trail = np.array(sim.feature_trails[i])
        ax.plot(trail[:, 0], trail[:, 1], f'{colors[i]}-', linewidth=1.5)
        ax.plot(trail[0, 0], trail[0, 1], f'{colors[i]}o', markersize=8)
        ax.plot(trail[-1, 0], trail[-1, 1], f'{colors[i]}s', markersize=8)
    ax.plot(sim.desired[:, 0], sim.desired[:, 1], 'k*', markersize=15, label='期望')
    ax.set_xlabel('u (pixels)')
    ax.set_ylabel('v (pixels)')
    ax.set_title('特征点运动轨迹(图像坐标)')
    ax.legend(['特征点轨迹', '', '', '', '期望位置'])
    ax.invert_yaxis()
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')

    # 收敛曲线
    ax = axes[1]
    ax.plot(sim.error_hist, 'b-', linewidth=1.5)
    ax.set_xlabel('迭代次数')
    ax.set_ylabel('误差范数')
    ax.set_title(f'收敛曲线 (最终: {sim.error_hist[-1]:.2f} pixels)')
    ax.grid(True, alpha=0.3)

    # 各特征点误差
    ax = axes[2]
    for i in range(sim.n):
        trail = np.array(sim.feature_trails[i])
        errs = np.linalg.norm(trail - sim.desired[i], axis=1)
        ax.plot(errs, label=f'特征点{i+1}', linewidth=1.2)
    ax.set_xlabel('迭代次数')
    ax.set_ylabel('特征点误差 (pixels)')
    ax.set_title('各特征点跟踪误差')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('ibvs_simulation.png', dpi=150, bbox_inches='tight')
    plt.close('all')


def run_pbvs_simulation():
    print('\n=== PBVS 视觉伺服仿真 ===')
    sim = PBVSSimulator(lambda_pos=1.0, lambda_rot=0.8)
    sim.run(max_iter=300)

    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    fig.suptitle('PBVS 基于位置的视觉伺服仿真', fontsize=14, fontweight='bold')

    # 位姿收敛
    ax = axes[0]
    poses = np.array(sim.pose_hist)
    labels = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
    for i, lbl in enumerate(labels):
        ax.plot(poses[:, i], label=lbl, linewidth=1.2)
    ax.axhline(y=0, color='gray', linestyle='--', alpha=0.3)
    ax.set_xlabel('迭代次数')
    ax.set_ylabel('位姿值')
    ax.set_title('位姿分量收敛')
    ax.legend(ncol=3)
    ax.grid(True, alpha=0.3)

    # 误差曲线
    ax = axes[1]
    ax.plot(sim.error_pos_hist, 'b-', label='位置误差', linewidth=1.5)
    ax.plot(sim.error_rot_hist, 'r-', label='旋转误差', linewidth=1.5)
    ax.set_xlabel('迭代次数')
    ax.set_ylabel('误差')
    ax.set_title('位置与旋转误差收敛')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 3D轨迹
    ax = axes[2]
    if len(sim.pose_hist) > 0:
        poses = np.array(sim.pose_hist)
        ax.plot(poses[:, 0], poses[:, 1], 'b-', linewidth=2, label='运动轨迹')
        ax.plot(poses[0, 0], poses[0, 1], 'go', markersize=12, label='起点')
        ax.plot(poses[-1, 0], poses[-1, 1], 'r*', markersize=14, label='终点')
        ax.plot(sim.desired_pose[0], sim.desired_pose[1], 'kx', markersize=14, markeredgewidth=3, label='期望')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('XY平面运动轨迹')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')

    plt.tight_layout()
    plt.savefig('pbvs_simulation.png', dpi=150, bbox_inches='tight')
    plt.close('all')


if __name__ == '__main__':
    run_ibvs_simulation()
    run_pbvs_simulation()
