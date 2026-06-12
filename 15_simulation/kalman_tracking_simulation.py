#!/usr/bin/env python3
"""
卡尔曼跟踪仿真 - 多目标跟踪
============================
使用卡尔曼滤波器进行多目标跟踪，包含：
- 标准卡尔曼滤波 (KF) 状态估计
- 多目标关联 (最近邻/匈牙利算法)
- 航迹管理 (起始/维持/终止)

运行: python kalman_tracking_simulation.py
依赖: numpy, matplotlib
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from collections import defaultdict

# ========== 卡尔曼滤波器 ==========
class KalmanFilter:
    """2D匀速运动模型的卡尔曼滤波器"""

    def __init__(self, dt=0.1, process_noise=1.0, measurement_noise=5.0):
        self.dt = dt

        # 状态向量: [x, y, vx, vy]
        self.x = np.zeros(4)

        # 状态转移矩阵
        self.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1,  0],
            [0, 0, 0,  1]
        ])

        # 观测矩阵 (只观测位置)
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])

        # 过程噪声
        q = process_noise
        self.Q = np.array([
            [dt**4/4, 0,       dt**3/2, 0      ],
            [0,       dt**4/4, 0,       dt**3/2],
            [dt**3/2, 0,       dt**2,   0      ],
            [0,       dt**3/2, 0,       dt**2  ]
        ]) * q

        # 观测噪声
        self.R = np.eye(2) * measurement_noise

        # 协方差矩阵
        self.P = np.eye(4) * 100

        self.initialized = False

    def initialize(self, measurement):
        """用第一次观测初始化"""
        self.x[:2] = measurement
        self.x[2:] = 0
        self.P = np.eye(4) * 100
        self.initialized = True

    def predict(self):
        """预测步"""
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        return self.x[:2]

    def update(self, z):
        """更新步"""
        if not self.initialized:
            self.initialize(z)
            return self.x[:2]

        # 卡尔曼增益
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        # 残差
        y = z - self.H @ self.x
        # 角度归一化(如果需要)

        # 更新
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P

        return self.x[:2]

    def get_state(self):
        return self.x.copy()

    def get_position(self):
        return self.x[:2].copy()

    def get_velocity(self):
        return self.x[2:].copy()

    def innovation_covariance(self):
        return self.H @ self.P @ self.H.T + self.R


# ========== 目标模型 ==========
class Target:
    """运动目标"""

    def __init__(self, id, x, y, vx, vy, motion_type='linear'):
        self.id = id
        self.x, self.y = x, y
        self.vx, self.vy = vx, vy
        self.motion_type = motion_type
        self.trajectory = [(x, y)]
        self.time = 0.0
        self.active = True

    def step(self, dt=0.1):
        """更新目标位置"""
        self.time += dt

        if self.motion_type == 'linear':
            self.x += self.vx * dt
            self.y += self.vy * dt
        elif self.motion_type == 'turning':
            # 转弯运动
            omega = 0.3  # 角速度
            vx_new = self.vx * np.cos(omega * dt) - self.vy * np.sin(omega * dt)
            vy_new = self.vx * np.sin(omega * dt) + self.vy * np.cos(omega * dt)
            self.vx, self.vy = vx_new, vy_new
            self.x += self.vx * dt
            self.y += self.vy * dt
        elif self.motion_type == 'sinusoidal':
            self.x += self.vx * dt
            self.y += self.vy * dt + 2.0 * np.sin(0.5 * self.time) * dt
        elif self.motion_type == 'accelerating':
            self.vx += 0.5 * dt
            self.x += self.vx * dt
            self.y += self.vy * dt

        # 范围约束
        self.x = np.clip(self.x, -50, 50)
        self.y = np.clip(self.y, -50, 50)

        self.trajectory.append((self.x, self.y))
        return self.x, self.y


# ========== 多目标跟踪器 ==========
class MultiTargetTracker:
    """多目标跟踪管理器"""

    def __init__(self, gate_threshold=15.0, max_missed=5, min_hits=3):
        self.trackers = {}          # id -> KalmanFilter
        self.track_ids = {}         # tracker_id -> target_id (假设)
        self.missed_count = {}      # 连续漏检次数
        self.hit_count = {}         # 连续检测次数
        self.track_history = {}     # 跟踪历史
        self.gate_threshold = gate_threshold
        self.max_missed = max_missed
        self.min_hits = min_hits
        self.next_id = 0
        self.confirmed_tracks = set()

    def _assign_id(self):
        tid = self.next_id
        self.next_id += 1
        return tid

    def _mahalanobis_distance(self, kf, measurement):
        """计算马氏距离 (用于门限关联)"""
        predicted = kf.H @ kf.predict()  # 使用预测位置
        S = kf.innovation_covariance()
        residual = measurement - predicted
        d = np.sqrt(residual @ np.linalg.inv(S) @ residual)
        return d

    def _simple_associate(self, detections):
        """最近邻关联"""
        if not self.trackers:
            # 所有检测初始化新航迹
            associations = {}
            for i, det in enumerate(detections):
                tid = self._assign_id()
                kf = KalmanFilter()
                kf.initialize(det)
                self.trackers[tid] = kf
                self.missed_count[tid] = 0
                self.hit_count[tid] = 1
                self.track_history[tid] = [det.copy()]
                associations[i] = tid
            return associations

        # 计算距离矩阵
        track_ids = list(self.trackers.keys())
        cost_matrix = np.full((len(detections), len(track_ids)), 1e6)

        for i, det in enumerate(detections):
            for j, tid in enumerate(track_ids):
                kf = self.trackers[tid]
                kf.predict()  # 预测
                pred_pos = kf.get_position()
                dist = np.linalg.norm(det - pred_pos)
                if dist < self.gate_threshold:
                    cost_matrix[i, j] = dist

        # 最近邻匹配
        associations = {}
        matched_tracks = set()
        matched_dets = set()

        while True:
            if cost_matrix.size == 0 or np.min(cost_matrix) >= self.gate_threshold:
                break
            min_idx = np.unravel_index(np.argmin(cost_matrix), cost_matrix.shape)
            i, j = min_idx
            associations[i] = track_ids[j]
            matched_tracks.add(track_ids[j])
            matched_dets.add(i)
            cost_matrix[i, :] = 1e6
            cost_matrix[:, j] = 1e6

        # 未匹配的检测 -> 新航迹
        for i in range(len(detections)):
            if i not in matched_dets:
                tid = self._assign_id()
                kf = KalmanFilter()
                kf.initialize(detections[i])
                self.trackers[tid] = kf
                self.missed_count[tid] = 0
                self.hit_count[tid] = 1
                self.track_history[tid] = [detections[i].copy()]
                associations[i] = tid

        # 未匹配的航迹 -> 漏检
        for tid in track_ids:
            if tid not in matched_tracks:
                self.missed_count[tid] += 1

        return associations

    def update(self, detections):
        """更新跟踪器"""
        if len(detections) == 0:
            for tid in list(self.trackers.keys()):
                self.missed_count[tid] += 1
            self._cleanup()
            return {}

        associations = self._simple_associate(detections)

        # 更新关联的滤波器
        for det_idx, tid in associations.items():
            if det_idx < len(detections):
                kf = self.trackers[tid]
                kf.update(detections[det_idx])
                self.missed_count[tid] = 0
                self.hit_count[tid] += 1
                self.track_history[tid].append(kf.get_position().copy())

                if self.hit_count[tid] >= self.min_hits:
                    self.confirmed_tracks.add(tid)

        self._cleanup()
        return associations

    def _cleanup(self):
        """移除丢失的航迹"""
        to_remove = []
        for tid in self.trackers:
            if self.missed_count[tid] >= self.max_missed:
                to_remove.append(tid)
        for tid in to_remove:
            del self.trackers[tid]
            del self.missed_count[tid]
            del self.hit_count[tid]
            if tid in self.confirmed_tracks:
                self.confirmed_tracks.discard(tid)

    def get_confirmed_tracks(self):
        """获取已确认航迹"""
        result = {}
        for tid in self.confirmed_tracks:
            if tid in self.trackers:
                result[tid] = {
                    'position': self.trackers[tid].get_position(),
                    'velocity': self.trackers[tid].get_velocity(),
                    'history': self.track_history.get(tid, [])
                }
        return result


# ========== 仿真主循环 ==========
def run_simulation(num_targets=5, duration=30.0, dt=0.1, detection_prob=0.9, false_alarm_rate=2):
    """运行多目标跟踪仿真"""
    np.random.seed(42)

    # 创建目标
    motion_types = ['linear', 'turning', 'sinusoidal', 'accelerating', 'linear']
    targets = []
    for i in range(num_targets):
        angle = 2 * np.pi * i / num_targets
        r = 15 + np.random.uniform(-5, 5)
        x, y = r * np.cos(angle), r * np.sin(angle)
        speed = 3 + np.random.uniform(-1, 1)
        vx = speed * np.cos(angle + np.pi / 2)
        vy = speed * np.sin(angle + np.pi / 2)
        targets.append(Target(i, x, y, vx, vy, motion_types[i % len(motion_types)]))

    tracker = MultiTargetTracker(gate_threshold=12.0, max_missed=5, min_hits=3)

    steps = int(duration / dt)
    all_detections = []
    all_tracks = []

    for step in range(steps):
        # 目标运动
        true_positions = []
        for tgt in targets:
            pos = tgt.step(dt)
            true_positions.append(np.array(pos))

        # 生成观测 (带漏检和虚警)
        detections = []
        det_truth = []  # 每个检测对应的真实目标

        for i, pos in enumerate(true_positions):
            if np.random.random() < detection_prob:
                noise = np.random.randn(2) * 2.0
                detections.append(pos + noise)
                det_truth.append(i)

        # 虚警
        for _ in range(np.random.poisson(false_alarm_rate)):
            fa = np.random.uniform(-40, 40, 2)
            detections.append(fa)
            det_truth.append(-1)

        det_array = np.array(detections) if detections else np.empty((0, 2))

        # 更新跟踪器
        tracker.update(det_array)

        all_detections.append(det_array)
        all_tracks.append(tracker.get_confirmed_tracks())

    return targets, tracker, all_detections, all_tracks, steps, dt


# ========== 可视化 ==========
def plot_results(targets, tracker, all_detections, all_tracks, steps, dt):
    """绘制跟踪结果"""
    fig, axes = plt.subplots(2, 2, figsize=(16, 14))
    fig.suptitle('卡尔曼滤波多目标跟踪仿真', fontsize=16, fontweight='bold')

    colors = plt.cm.tab10(np.linspace(0, 1, 10))

    # 1. 轨迹对比
    ax = axes[0, 0]
    # 真实轨迹
    for tgt in targets:
        traj = np.array(tgt.trajectory)
        ax.plot(traj[:, 0], traj[:, 1], '--', color=colors[tgt.id], linewidth=2, alpha=0.6,
                label=f'目标{tgt.id}(真实)')

    # 估计轨迹
    for tid, track_info in tracker.get_confirmed_tracks().items():
        if len(track_info['history']) > 2:
            hist = np.array(track_info['history'])
            ax.plot(hist[:, 0], hist[:, 1], '-', color=colors[tid % 10], linewidth=1.5,
                    label=f'航迹{tid}(估计)')

    # 最后一帧的检测
    if len(all_detections[-1]) > 0:
        ax.scatter(all_detections[-1][:, 0], all_detections[-1][:, 1],
                   c='red', marker='x', s=50, zorder=5, label='当前检测')

    ax.set_title('目标轨迹 (真实 vs 估计)')
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    ax.legend(fontsize=7, ncol=2)
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')

    # 2. 跟踪误差随时间
    ax = axes[0, 1]
    time_axis = np.arange(steps) * dt
    for tgt in targets:
        errors = []
        for step_idx in range(steps):
            if step_idx < len(tgt.trajectory):
                true_pos = np.array(tgt.trajectory[step_idx])
                # 找最近的已确认航迹
                min_err = np.inf
                for tid, track_info in tracker.get_confirmed_tracks().items():
                    if step_idx < len(track_info['history']):
                        est_pos = np.array(track_info['history'][step_idx])
                        err = np.linalg.norm(est_pos - true_pos)
                        min_err = min(min_err, err)
                errors.append(min_err if min_err < np.inf else np.nan)
            else:
                errors.append(np.nan)
        ax.plot(time_axis, errors, color=colors[tgt.id], linewidth=1.5, label=f'目标{tgt.id}')

    ax.set_title('跟踪误差')
    ax.set_xlabel('时间 (s)'); ax.set_ylabel('位置误差 (m)')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    # 3. 航迹数量
    ax = axes[1, 0]
    track_counts = [len(t) for t in all_tracks]
    ax.plot(time_axis, track_counts, 'b-', linewidth=2)
    ax.axhline(y=len(targets), color='r', linestyle='--', label=f'真实目标数={len(targets)}')
    ax.set_title('活跃航迹数量')
    ax.set_xlabel('时间 (s)'); ax.set_ylabel('航迹数')
    ax.legend(); ax.grid(True, alpha=0.3)

    # 4. 速度估计
    ax = axes[1, 1]
    for tgt in targets:
        true_speed = np.sqrt(tgt.vx**2 + tgt.vy**2)
        ax.axhline(y=true_speed, color=colors[tgt.id], linestyle='--', alpha=0.5)

    for tid, track_info in tracker.get_confirmed_tracks().items():
        vel = track_info['velocity']
        speed = np.linalg.norm(vel)
        ax.bar(tid, speed, color=colors[tid % 10], alpha=0.7, label=f'航迹{tid}')

    ax.set_title('速度估计')
    ax.set_xlabel('航迹ID'); ax.set_ylabel('速度 (m/s)')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3, axis='y')

    plt.tight_layout()
    plt.savefig('kalman_tracking_results.png', dpi=150, bbox_inches='tight')
    plt.close('all')

    print("结果已保存: kalman_tracking_results.png")


# ========== 主程序 ==========
if __name__ == '__main__':
    print("=" * 60)
    print("卡尔曼跟踪仿真 - 多目标跟踪")
    print("=" * 60)

    print("仿真参数: 5个目标, 30秒, 检测概率90%")
    targets, tracker, detections, tracks, steps, dt = run_simulation(
        num_targets=5, duration=30.0, dt=0.1, detection_prob=0.9, false_alarm_rate=2
    )

    # 统计
    confirmed = tracker.get_confirmed_tracks()
    print(f"\n跟踪统计:")
    print(f"  真实目标数: {len(targets)}")
    print(f"  已确认航迹数: {len(confirmed)}")

    for tid, info in confirmed.items():
        hist = np.array(info['history'])
        print(f"  航迹{tid}: 长度={len(hist)}, 最终位置=({info['position'][0]:.1f}, {info['position'][1]:.1f}), "
              f"速度=({info['velocity'][0]:.1f}, {info['velocity'][1]:.1f})")

    plot_results(targets, tracker, detections, tracks, steps, dt)
    print("\n仿真完成！")
