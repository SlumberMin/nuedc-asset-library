#!/usr/bin/env python3
"""
循迹仿真 — 灰度传感器 + PID控制
==================================
模拟AGV小车沿黑线路径行驶，使用5路灰度传感器检测位置偏差，
通过PID控制器调节左右轮速差实现循迹。
"""

import numpy as np
import matplotlib.pyplot as plt

# ============ 仿真参数 ============
dt = 0.01           # 时间步长 (s)
T = 40.0             # 仿真时长 (s)
N = int(T / dt)

# ============ 赛道定义（参数化曲线） ============
def track_points_fn(n=2000):
    """
    生成赛道中心线坐标点列表
    赛道：直线 → 弯道 → 直线 → S弯 → 直线
    """
    pts = []
    # 段1：水平直线 (0~4m)
    for x in np.linspace(0, 4, 200):
        pts.append([x, 0])
    # 段2：圆弧弯道 (半径2m, 90°)
    cx, cy = 4, 2
    for a in np.linspace(-np.pi/2, 0, 300):
        pts.append([cx + 2*np.cos(a), cy + 2*np.sin(a)])
    # 段3：垂直直线
    for y in np.linspace(2, 6, 200):
        pts.append([6, y])
    # 段4：S弯（正弦波）
    for y in np.linspace(6, 10, 400):
        x = 6 + 1.0 * np.sin((y - 6) / 4 * 2 * np.pi)
        pts.append([x, y])
    # 段5：直线
    for y in np.linspace(10, 14, 200):
        pts.append([6, y])
    return np.array(pts)

track = track_points_fn()

# ============ PID控制器 ============
class PIDController:
    """PID控制器"""
    def __init__(self, kp, ki, kd, out_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_limit = out_limit
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error, dt):
        self.integral += error * dt
        self.integral = np.clip(self.integral, -3, 3)  # 积分限幅
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        if self.out_limit:
            output = np.clip(output, -self.out_limit, self.out_limit)
        return output

# ============ 寻找最近赛道点 ============
def find_nearest(track, pos):
    """返回最近赛道点的索引和横向偏差"""
    dists = np.linalg.norm(track - pos, axis=1)
    idx = np.argmin(dists)

    # 计算赛道切线方向
    if idx < len(track) - 1:
        tangent = track[idx+1] - track[idx]
    else:
        tangent = track[idx] - track[idx-1]
    tangent = tangent / (np.linalg.norm(tangent) + 1e-8)

    # 横向偏差（带符号）
    to_center = track[idx] - pos
    # 法线方向（左手定则）
    normal = np.array([-tangent[1], tangent[0]])
    lateral_error = np.dot(to_center, normal)

    return idx, lateral_error, tangent

# ============ 灰度传感器模型 ============
def grayscale_reading(lateral_error, n_sensors=5, spacing=0.03):
    """
    模拟5路灰度传感器输出
    lateral_error: 横向偏差 (m)
    返回: [0~1]数组, 0=黑线 1=白
    """
    offsets = np.linspace(-spacing*(n_sensors-1)/2, spacing*(n_sensors-1)/2, n_sensors)
    values = []
    for off in offsets:
        dist = abs(lateral_error + off)
        if dist < 0.01:       # 黑线宽2cm
            values.append(0.0)
        elif dist < 0.02:
            values.append(0.5)
        else:
            values.append(1.0)
    return np.array(values)

# ============ 机器人动力学 ============
class LineFollower:
    """循迹机器人"""
    def __init__(self, x, y, heading):
        self.x = x
        self.y = y
        self.heading = heading
        self.v = 0.5            # 基础速度 (m/s)
        self.max_omega = 3.0    # 最大角速度 (rad/s)
        self.trajectory = []
        self.speed = 0.5

    def update(self, omega, dt):
        omega = np.clip(omega, -self.max_omega, self.max_omega)
        self.heading += omega * dt
        self.x += self.speed * np.cos(self.heading) * dt
        self.y += self.speed * np.sin(self.heading) * dt
        self.trajectory.append((self.x, self.y))

    def get_pos(self):
        return np.array([self.x, self.y])

# ============ 主仿真循环 ============
robot = LineFollower(track[0, 0], track[0, 1], 0.0)
pid = PIDController(kp=8.0, ki=0.05, kd=2.0, out_limit=3.0)

# 记录数据
time_log = []
lateral_error_log = []
sensor_log = []
steering_log = []
robot_traj = []

for i in range(N):
    t = i * dt

    # 寻找最近赛道点
    idx, lat_error, tangent = find_nearest(track, robot.get_pos())

    # 灰度传感器读数
    sensors = grayscale_reading(lat_error)

    # PID控制转向
    omega = pid.update(lat_error, dt)

    # 更新机器人
    robot.update(omega, dt)

    # 记录
    time_log.append(t)
    lateral_error_log.append(lat_error * 100)  # 转为cm
    sensor_log.append(sensors)
    steering_log.append(omega)
    robot_traj.append(robot.get_pos())

    # 接近赛道终点
    if idx > len(track) - 10:
        break

robot_traj = np.array(robot_traj)
sensor_log = np.array(sensor_log)

# ============ 绘图 ============
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

fig, axes = plt.subplots(2, 2, figsize=(14, 10))
fig.suptitle('循迹仿真 — 灰度传感器 + PID控制', fontsize=16, fontweight='bold')

# 图1：赛道和轨迹
axes[0, 0].plot(track[:, 0], track[:, 1], 'k-', linewidth=8, alpha=0.3, label='黑线赛道')
axes[0, 0].plot(robot_traj[:, 0], robot_traj[:, 1], 'b-', linewidth=1.5, label='机器人轨迹')
axes[0, 0].plot(robot_traj[0, 0], robot_traj[0, 1], 'go', markersize=10, label='起点')
axes[0, 0].plot(robot_traj[-1, 0], robot_traj[-1, 1], 'r*', markersize=12, label='终点')
axes[0, 0].set_xlabel('X (m)')
axes[0, 0].set_ylabel('Y (m)')
axes[0, 0].set_title('赛道轨迹')
axes[0, 0].legend(loc='upper left')
axes[0, 0].set_aspect('equal')
axes[0, 0].grid(True, alpha=0.3)

# 图2：横向偏差
t_arr = np.array(time_log[:len(lateral_error_log)])
axes[0, 1].plot(t_arr, lateral_error_log, 'b-', linewidth=1)
axes[0, 1].axhline(y=0, color='r', linestyle='--', alpha=0.5)
axes[0, 1].set_xlabel('时间 (s)')
axes[0, 1].set_ylabel('横向偏差 (cm)')
axes[0, 1].set_title('循迹偏差')
axes[0, 1].grid(True, alpha=0.3)

# 图3：传感器值
for s_idx in range(5):
    axes[1, 0].plot(t_arr[:len(sensor_log)], sensor_log[:, s_idx], linewidth=1, label=f'S{s_idx+1}')
axes[1, 0].set_xlabel('时间 (s)')
axes[1, 0].set_ylabel('传感器值')
axes[1, 0].set_title('灰度传感器输出')
axes[1, 0].legend()
axes[1, 0].grid(True, alpha=0.3)

# 图4：转向角速度
axes[1, 1].plot(t_arr[:len(steering_log)], np.degrees(steering_log[:len(t_arr)]), 'r-', linewidth=1)
axes[1, 1].set_xlabel('时间 (s)')
axes[1, 1].set_ylabel('转向角速度 (°/s)')
axes[1, 1].set_title('PID转向控制')
axes[1, 1].grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('./nuedc-asset-library/15_simulation/line_following_result.png', dpi=150, bbox_inches='tight')
print("✅ 循迹仿真完成，图表已保存")

# 统计
lat_arr = np.array(lateral_error_log)
print(f"  平均横向偏差: {np.mean(np.abs(lat_arr)):.2f} cm")
print(f"  最大横向偏差: {np.max(np.abs(lat_arr)):.2f} cm")
print(f"  行驶时间: {t_arr[-1]:.1f} s")
print(f"  行驶距离: {np.sum(np.linalg.norm(np.diff(robot_traj, axis=0), axis=1)):.2f} m")
