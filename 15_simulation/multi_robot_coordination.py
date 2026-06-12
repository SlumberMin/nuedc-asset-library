#!/usr/bin/env python3
"""
多机器人协调仿真 — 编队控制 + 任务分配
==========================================
模拟多台机器人协作完成搜索覆盖任务。
使用人工势场法实现避碰，虚拟结构法实现编队。
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import matplotlib.patches as mpatches

# ============ 环境定义 ============
ARENA_SIZE = 10.0  # 场地边长 (m)
N_ROBOTS = 5       # 机器人数量

# 目标点序列（巡逻路径）
waypoints = [
    np.array([2.0, 2.0]),
    np.array([8.0, 2.0]),
    np.array([8.0, 8.0]),
    np.array([2.0, 8.0]),
    np.array([5.0, 5.0]),
]

# ============ 机器人模型 ============
class Robot:
    def __init__(self, robot_id, x, y, color):
        self.id = robot_id
        self.x = x
        self.y = y
        self.vx = 0.0
        self.vy = 0.0
        self.color = color
        self.max_speed = 1.5
        self.comm_range = 3.0       # 通信范围
        self.safe_dist = 0.5        # 安全距离
        self.trajectory = []
        self.current_waypoint = 0
        self.task_complete = False

    def get_pos(self):
        return np.array([self.x, self.y])

    def get_vel(self):
        return np.array([self.vx, self.vy])

    def update(self, vx_cmd, vy_cmd, dt):
        # 限速
        speed = np.sqrt(vx_cmd**2 + vy_cmd**2)
        if speed > self.max_speed:
            vx_cmd = vx_cmd / speed * self.max_speed
            vy_cmd = vy_cmd / speed * self.max_speed

        self.vx = vx_cmd
        self.vy = vy_cmd
        self.x += self.vx * dt
        self.y += self.vy * dt

        # 场地边界约束
        margin = 0.3
        self.x = np.clip(self.x, margin, ARENA_SIZE - margin)
        self.y = np.clip(self.y, margin, ARENA_SIZE - margin)

        self.trajectory.append((self.x, self.y))

# ============ 人工势场法避碰 ============
def repulsive_force(robot_pos, other_pos, safe_dist=0.5, k_rep=2.0):
    """计算排斥力"""
    diff = robot_pos - other_pos
    dist = np.linalg.norm(diff)
    if dist < 0.01:
        dist = 0.01
    if dist < safe_dist * 3:
        # 排斥力与距离平方成反比
        force_mag = k_rep * (1/dist - 1/(safe_dist*3)) / (dist**2)
        return diff / dist * force_mag
    return np.zeros(2)

# ============ 编队控制（虚拟结构法） ============
def formation_offset(robot_id, n_robots, pattern='circle'):
    """计算编队偏移量"""
    if pattern == 'circle':
        # 圆形编队
        r = 1.0
        angle = 2 * np.pi * robot_id / n_robots
        return np.array([r * np.cos(angle), r * np.sin(angle)])
    elif pattern == 'v_shape':
        # V字编队
        if robot_id == 0:
            return np.array([0, 0])
        side = 1 if robot_id % 2 == 1 else -1
        rank = (robot_id + 1) // 2
        return np.array([-rank * 0.6, side * rank * 0.6])
    elif pattern == 'line':
        # 横排
        offset = (robot_id - (n_robots-1)/2) * 0.8
        return np.array([0, offset])
    return np.zeros(2)

# ============ 任务分配（最近邻法） ============
def assign_tasks(robots, waypoints):
    """为每个机器人分配最近的未完成目标"""
    available = list(range(len(waypoints)))
    assignments = {}

    for robot in robots:
        if available:
            dists = [np.linalg.norm(robot.get_pos() - waypoints[w]) for w in available]
            best = available[np.argmin(dists)]
            assignments[robot.id] = best
            available.remove(best)

    return assignments

# ============ 主仿真 ============
np.random.seed(42)
dt = 0.05
T = 30.0
N_steps = int(T / dt)

# 创建机器人（不同初始位置）
colors = ['#FF4444', '#44FF44', '#4444FF', '#FFAA00', '#FF44FF']
robots = []
start_positions = [
    (1.0, 1.0), (3.0, 1.0), (2.0, 3.0), (1.0, 5.0), (4.0, 2.0)
]
for i in range(N_ROBOTS):
    r = Robot(i, start_positions[i][0], start_positions[i][1], colors[i])
    robots.append(r)

# 记录
time_log = []
formation_error_log = []
collision_count = 0
coverage_map = np.zeros((20, 20))  # 覆盖网格
communication_log = []

for step in range(N_steps):
    t = step * dt

    # 编队目标 = 当前航点 + 编队偏移
    for robot in robots:
        wp = waypoints[robot.current_waypoint]
        f_offset = formation_offset(robot.id, N_ROBOTS, pattern='v_shape')
        target = wp + f_offset

        # 趋向目标的速度
        to_target = target - robot.get_pos()
        dist_to_target = np.linalg.norm(to_target)
        if dist_to_target > 0.1:
            v_attr = to_target / dist_to_target * min(1.0, dist_to_target)
        else:
            v_attr = np.zeros(2)

        # 到达航点检测
        if np.linalg.norm(wp - robot.get_pos()) < 1.0:
            robot.current_waypoint = (robot.current_waypoint + 1) % len(waypoints)

        # 避碰排斥力
        v_rep = np.zeros(2)
        neighbors = 0
        for other in robots:
            if other.id != robot.id:
                dist = np.linalg.norm(robot.get_pos() - other.get_pos())
                if dist < robot.comm_range:
                    neighbors += 1
                    v_rep += repulsive_force(robot.get_pos(), other.get_pos(), robot.safe_dist)
                    if dist < robot.safe_dist:
                        collision_count += 1

        # 总速度 = 吸引 + 排斥
        v_total = v_attr + v_rep * 0.5
        robot.update(v_total[0], v_total[1], dt)

        # 更新覆盖地图
        gx = int(robot.x / ARENA_SIZE * 20)
        gy = int(robot.y / ARENA_SIZE * 20)
        gx = np.clip(gx, 0, 19)
        gy = np.clip(gy, 0, 19)
        coverage_map[gx, gy] = 1

    # 记录编队误差（与理想V形的距离）
    formation_error = 0
    center = np.mean([r.get_pos() for r in robots], axis=0)
    for robot in robots:
        ideal = center + formation_offset(robot.id, N_ROBOTS, 'v_shape')
        formation_error += np.linalg.norm(robot.get_pos() - ideal)
    formation_error /= N_ROBOTS

    time_log.append(t)
    formation_error_log.append(formation_error)
    communication_log.append(sum(1 for r in robots for o in robots
                                  if r.id != o.id and
                                  np.linalg.norm(r.get_pos()-o.get_pos()) < r.comm_range) / 2)

# ============ 绘图 ============
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

fig, axes = plt.subplots(2, 2, figsize=(14, 10))
fig.suptitle('多机器人协调仿真 — 编队控制 + 避碰', fontsize=16, fontweight='bold')

# 图1：轨迹和编队
ax = axes[0, 0]
# 画航点
for i, wp in enumerate(waypoints):
    ax.plot(wp[0], wp[1], 'k*', markersize=12, zorder=5)
    ax.annotate(f'WP{i+1}', wp + 0.2, fontsize=8)

# 画机器人轨迹
for robot in robots:
    traj = np.array(robot.trajectory)
    ax.plot(traj[:, 0], traj[:, 1], '-', color=robot.color, alpha=0.5, linewidth=0.8)
    # 最终位置
    ax.plot(traj[-1, 0], traj[-1, 1], 'o', color=robot.color, markersize=10,
            label=f'Robot {robot.id}')

ax.set_xlim(0, ARENA_SIZE)
ax.set_ylim(0, ARENA_SIZE)
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_title('机器人轨迹与编队')
ax.legend(loc='upper right', fontsize=8)
ax.set_aspect('equal')
ax.grid(True, alpha=0.3)

# 图2：编队误差
t_arr = np.array(time_log)
axes[0, 1].plot(t_arr, formation_error_log, 'b-', linewidth=1)
axes[0, 1].set_xlabel('时间 (s)')
axes[0, 1].set_ylabel('编队误差 (m)')
axes[0, 1].set_title('编队保持误差')
axes[0, 1].grid(True, alpha=0.3)

# 图3：覆盖地图
im = axes[1, 0].imshow(coverage_map.T, origin='lower', cmap='YlOrRd',
                         extent=[0, ARENA_SIZE, 0, ARENA_SIZE])
axes[1, 0].set_xlabel('X (m)')
axes[1, 0].set_ylabel('Y (m)')
axes[1, 0].set_title(f'覆盖地图 (覆盖率: {coverage_map.sum()/400*100:.1f}%)')
plt.colorbar(im, ax=axes[1, 0])

# 图4：通信连接数
axes[1, 1].plot(t_arr, communication_log, 'g-', linewidth=1.5)
axes[1, 1].set_xlabel('时间 (s)')
axes[1, 1].set_ylabel('活跃通信链路数')
axes[1, 1].set_title('机器人间通信')
axes[1, 1].grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('./nuedc-asset-library/15_simulation/multi_robot_result.png', dpi=150, bbox_inches='tight')
print("✅ 多机器人协调仿真完成，图表已保存")

# 统计
total_travel = sum(np.sum(np.linalg.norm(np.diff(np.array(r.trajectory), axis=0), axis=1)) for r in robots)
print(f"  机器人数量: {N_ROBOTS}")
print(f"  总行驶距离: {total_travel:.1f} m")
print(f"  覆盖率: {coverage_map.sum()/400*100:.1f}%")
print(f"  平均编队误差: {np.mean(formation_error_log):.3f} m")
print(f"  碰撞检测次数: {collision_count // 2} (双向计数)")
