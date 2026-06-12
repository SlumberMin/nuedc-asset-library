#!/usr/bin/env python3
"""
避障仿真 — 超声波传感器 + 状态机控制
========================================
模拟机器人在有障碍物的环境中，使用超声波测距，
通过有限状态机实现避障行为。
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

# ============ 障碍物定义 ============
obstacles = [
    {'cx': 3.0, 'cy': 1.5, 'r': 0.5},
    {'cx': 5.0, 'cy': -1.0, 'r': 0.6},
    {'cx': 7.0, 'cy': 2.0, 'r': 0.4},
    {'cx': 4.0, 'cy': 3.0, 'r': 0.5},
    {'cx': 9.0, 'cy': 0.5, 'r': 0.7},
    {'cx': 6.0, 'cy': -2.0, 'r': 0.5},
    {'cx': 8.0, 'cy': -1.5, 'r': 0.4},
    {'cx': 2.0, 'cy': -2.0, 'r': 0.3},
]

# 目标点
goal = np.array([10.0, 0.0])

# ============ 机器人模型 ============
class AvoidanceRobot:
    def __init__(self, x, y, heading):
        self.x = x
        self.y = y
        self.heading = heading
        self.v = 0.0            # 当前速度 (m/s)
        self.omega = 0.0        # 角速度 (rad/s)
        self.max_v = 1.0        # 最大速度
        self.max_omega = 2.5    # 最大角速度
        self.sensor_range = 2.5 # 超声波最大量程
        self.trajectory = []
        self.state_log = []

    def get_pos(self):
        return np.array([self.x, self.y])

    def update(self, v_cmd, omega_cmd, dt):
        self.v = np.clip(v_cmd, -self.max_v, self.max_v)
        self.omega = np.clip(omega_cmd, -self.max_omega, self.max_omega)
        self.heading += self.omega * dt
        self.x += self.v * np.cos(self.heading) * dt
        self.y += self.v * np.sin(self.heading) * dt
        self.trajectory.append((self.x, self.y))

# ============ 超声波传感器模型 ============
def ultrasonic_sensors(robot, obstacles, n_beams=7, max_range=2.5):
    """
    模拟多路超声波传感器
    n_beams: 传感器数量（扇形分布）
    返回每个传感器的测量距离
    """
    # 传感器角度（相对于车头方向，扇形覆盖）
    angles = np.linspace(-np.pi/3, np.pi/3, n_beams) + robot.heading
    distances = []

    for angle in angles:
        ray_dir = np.array([np.cos(angle), np.sin(angle)])
        min_dist = max_range

        # 射线与圆形障碍物相交检测
        for obs in obstacles:
            oc = np.array([obs['cx'], obs['cy']]) - robot.get_pos()
            # 射线参数方程: P = robot + t * ray_dir
            # |t*ray_dir - oc|^2 = r^2
            a = np.dot(ray_dir, ray_dir)
            b = -2 * np.dot(oc, ray_dir)
            c = np.dot(oc, oc) - obs['r']**2
            discriminant = b**2 - 4*a*c
            if discriminant >= 0:
                t1 = (-b - np.sqrt(discriminant)) / (2*a)
                if 0 < t1 < min_dist:
                    min_dist = t1
        distances.append(min_dist)

    return np.array(distances)

# ============ 有限状态机 ============
class ObstacleFSM:
    """
    状态机：FORWARD → TURNING → AVOIDING → FORWARD
    """
    def __init__(self):
        self.state = 'FORWARD'
        self.turn_direction = 0  # -1=左, 1=右
        self.avoid_timer = 0

    def update(self, distances, goal_angle, dt):
        front = distances[len(distances)//2]  # 前方传感器
        left = distances[:len(distances)//3].mean()   # 左侧平均
        right = distances[len(distances)//3*2:].mean()  # 右侧平均

        if self.state == 'FORWARD':
            if front < 0.8:
                self.state = 'TURNING'
                # 选择空间更大的方向
                self.turn_direction = 1 if left > right else -1
            elif front < 1.5 and abs(goal_angle) > 0.3:
                self.state = 'TURNING'
                self.turn_direction = 1 if goal_angle > 0 else -1

        elif self.state == 'TURNING':
            if front > 1.5:
                self.avoid_timer = 0
                self.state = 'AVOIDING'
            elif front < 0.4:
                # 太近，加大转弯
                pass

        elif self.state == 'AVOIDING':
            self.avoid_timer += dt
            if front > 2.0 and self.avoid_timer > 0.5:
                self.state = 'FORWARD'

        # 返回控制指令
        if self.state == 'FORWARD':
            return 0.8, goal_angle * 0.5  # v, omega
        elif self.state == 'TURNING':
            return 0.2, self.turn_direction * 2.0
        elif self.state == 'AVOIDING':
            return 0.6, self.turn_direction * 1.0
        return 0, 0

# ============ 主仿真 ============
dt = 0.05
T = 30.0
N = int(T / dt)

robot = AvoidanceRobot(0, 0, 0)
fsm = ObstacleFSM()

# 记录
time_log = []
distances_log = []
state_log = []
front_dist_log = []
goal_dist_log = []

for i in range(N):
    t = i * dt

    # 超声波测距
    distances = ultrasonic_sensors(robot, obstacles)

    # 计算目标方向
    to_goal = goal - robot.get_pos()
    goal_angle = np.arctan2(to_goal[1], to_goal[0]) - robot.heading
    goal_angle = np.arctan2(np.sin(goal_angle), np.cos(goal_angle))  # 归一化

    # 状态机决策
    v_cmd, omega_cmd = fsm.update(distances, goal_angle, dt)

    # 更新机器人
    robot.update(v_cmd, omega_cmd, dt)

    # 记录
    time_log.append(t)
    distances_log.append(distances)
    state_log.append(fsm.state)
    front_dist_log.append(distances[len(distances)//2])
    goal_dist_log.append(np.linalg.norm(to_goal))

    # 到达目标
    if np.linalg.norm(goal - robot.get_pos()) < 0.5:
        break

# ============ 绘图 ============
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

fig, axes = plt.subplots(2, 2, figsize=(14, 10))
fig.suptitle('避障仿真 — 超声波传感器 + 状态机', fontsize=16, fontweight='bold')

traj = np.array(robot.trajectory)

# 图1：环境和轨迹
ax = axes[0, 0]
for obs in obstacles:
    circle = plt.Circle((obs['cx'], obs['cy']), obs['r'], color='red', alpha=0.5)
    ax.add_patch(circle)
ax.plot(traj[:, 0], traj[:, 1], 'b-', linewidth=1.5, label='轨迹')
ax.plot(traj[0, 0], traj[0, 1], 'go', markersize=10, label='起点')
ax.plot(goal[0], goal[1], 'r*', markersize=15, label='目标')
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_title('避障轨迹')
ax.legend()
ax.set_aspect('equal')
ax.grid(True, alpha=0.3)
ax.set_xlim(-1, 12)
ax.set_ylim(-4, 5)

# 图2：状态分布
t_arr = np.array(time_log)
state_colors = {'FORWARD': 'green', 'TURNING': 'orange', 'AVOIDING': 'red'}
for idx, (ti, state) in enumerate(zip(t_arr, state_log)):
    ax = axes[0, 1]
    ax.bar(ti, 1, width=dt, color=state_colors[state], alpha=0.7)

fwd_patch = mpatches.Patch(color='green', alpha=0.7, label='前进')
turn_patch = mpatches.Patch(color='orange', alpha=0.7, label='转向')
avoid_patch = mpatches.Patch(color='red', alpha=0.7, label='避障')
axes[0, 1].legend(handles=[fwd_patch, turn_patch, avoid_patch])
axes[0, 1].set_xlabel('时间 (s)')
axes[0, 1].set_title('状态机状态')
axes[0, 1].set_yticks([])

# 图3：前方距离
axes[1, 0].plot(t_arr, front_dist_log, 'b-', linewidth=1)
axes[1, 0].axhline(y=0.8, color='r', linestyle='--', alpha=0.5, label='避障阈值')
axes[1, 0].set_xlabel('时间 (s)')
axes[1, 0].set_ylabel('距离 (m)')
axes[1, 0].set_title('前方超声波测距')
axes[1, 0].legend()
axes[1, 0].grid(True, alpha=0.3)

# 图4：目标距离
axes[1, 1].plot(t_arr, goal_dist_log, 'g-', linewidth=1.5)
axes[1, 1].set_xlabel('时间 (s)')
axes[1, 1].set_ylabel('距离 (m)')
axes[1, 1].set_title('到目标距离')
axes[1, 1].grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('./nuedc-asset-library/15_simulation/obstacle_avoidance_result.png', dpi=150, bbox_inches='tight')
print("✅ 避障仿真完成，图表已保存")

# 统计
state_counts = {s: state_log.count(s) for s in ['FORWARD', 'TURNING', 'AVOIDING']}
total = len(state_log)
print(f"  总仿真时间: {t_arr[-1]:.1f} s")
print(f"  到目标距离: {goal_dist_log[-1]:.2f} m")
print(f"  状态分布: 前进{state_counts['FORWARD']/total*100:.0f}% "
      f"转向{state_counts['TURNING']/total*100:.0f}% "
      f"避障{state_counts['AVOIDING']/total*100:.0f}%")
print(f"  行驶距离: {np.sum(np.linalg.norm(np.diff(traj, axis=0), axis=1)):.2f} m")
