#!/usr/bin/env python3
"""
循迹PID仿真 - 不同赛道 + 不同PID参数
==============================================
功能：
  1. 生成不同赛道（直线、弯道、S弯、急转弯）
  2. 传感器阵列模拟（5路红外/电感）
  3. PID 控制转向
  4. 对比不同 PID 参数效果
使用：
  python line_following_pid_simulation.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch
import matplotlib.gridspec as gridspec

# ========== 赛道生成 ==========
def generate_track(track_type, n_points=2000):
    """生成赛道中心线坐标"""
    t = np.linspace(0, 10, n_points)

    if track_type == 'straight':
        x = t * 100
        y = np.zeros_like(t)
    elif track_type == 'curve':
        x = t * 100
        y = 50 * np.sin(0.3 * t)
    elif track_type == 's_curve':
        x = t * 100
        y = 80 * np.sin(0.4 * t) * np.sin(0.15 * t)
    elif track_type == 'sharp':
        x = t * 100
        y = 60 * np.sign(np.sin(0.5 * t)) * np.abs(np.sin(0.5 * t))**0.5
    else:  # complex
        x = t * 100
        y = 40 * np.sin(0.6*t) + 30 * np.cos(0.25*t) + 20 * np.sin(1.2*t)

    # 计算航向角
    dx = np.gradient(x)
    dy = np.gradient(y)
    heading = np.arctan2(dy, dx)

    return x, y, heading

# ========== 车辆模型 ==========
class LineFollower:
    def __init__(self, sensor_num=5, sensor_spacing=15):
        self.sensor_num = sensor_num
        self.sensor_spacing = sensor_spacing  # mm
        self.x = 0
        self.y = 0
        self.heading = 0
        self.speed = 200  # mm/s
        self.max_steer = 0.4  # rad

    def get_sensor_readings(self, track_x, track_y, track_heading):
        """模拟传感器读数，返回加权位置偏差"""
        readings = []
        for i in range(self.sensor_num):
            offset = (i - self.sensor_num // 2) * self.sensor_spacing
            sx = self.x + offset * np.cos(self.heading + np.pi/2)
            sy = self.y + offset * np.sin(self.heading + np.pi/2)

            # 找最近赛道点
            dists = np.sqrt((track_x - sx)**2 + (track_y - sy)**2)
            min_dist = np.min(dists)

            # 线检测范围
            if min_dist < 10:
                readings.append(1.0 - min_dist / 10)
            else:
                readings.append(0.0)

        # 加权位置偏差 (-1 到 1)
        weights = np.array([-2, -1, 0, 1, 2])
        if np.sum(readings) > 0.01:
            error = np.dot(readings, weights) / np.sum(readings)
        else:
            error = 0  # 丢线
        return error, readings

    def update(self, steer, dt):
        self.heading += steer * dt * 5
        self.heading = np.clip(self.heading, -np.pi/3, np.pi/3)
        self.x += self.speed * dt * np.cos(self.heading)
        self.y += self.speed * dt * np.sin(self.heading)

# ========== PID 控制器 ==========
class PID:
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.integral = 0
        self.prev_error = 0

    def update(self, error, dt):
        self.integral += error * dt
        self.integral = np.clip(self.integral, -2, 2)
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

# ========== 仿真函数 ==========
def simulate(track_type, pid_params, dt=0.01):
    tx, ty, th = generate_track(track_type)
    car = LineFollower()
    car.x, car.y, car.heading = tx[0], ty[0], th[0]
    pid = PID(**pid_params)

    hist = {'x': [], 'y': [], 'error': [], 'steer': []}

    for i in range(800):
        t = i * dt
        error, _ = car.get_sensor_readings(tx, ty, th)
        steer = pid.update(error, dt)
        steer = np.clip(steer, -car.max_steer, car.max_steer)
        car.update(steer, dt)

        # 计算到赛道中心距离
        dists = np.sqrt((tx - car.x)**2 + (ty - car.y)**2)
        lateral_err = np.min(dists) * np.sign(error) if abs(error) > 0.01 else np.min(dists)

        hist['x'].append(car.x)
        hist['y'].append(car.y)
        hist['error'].append(lateral_err)
        hist['steer'].append(steer)

    return tx, ty, hist

# ========== 主仿真 ==========
print("开始循迹PID仿真...")

tracks = ['straight', 'curve', 's_curve', 'sharp', 'complex']
track_names = ['直线', '弯道', 'S弯', '急转弯', '复杂赛道']

pid_configs = {
    '保守 (Kp=1.0, Kd=0.5)': {'kp': 1.0, 'ki': 0.0, 'kd': 0.5},
    '标准 (Kp=2.0, Kd=1.0)': {'kp': 2.0, 'ki': 0.05, 'kd': 1.0},
    '激进 (Kp=4.0, Kd=2.0)': {'kp': 4.0, 'ki': 0.1, 'kd': 2.0},
}

# 图1: 不同赛道对比
fig1, axes1 = plt.subplots(2, 3, figsize=(16, 10))
fig1.suptitle('循迹PID仿真 - 不同赛道', fontsize=14, fontweight='bold')

for idx, (track, name) in enumerate(zip(tracks, track_names)):
    ax = axes1[idx // 3, idx % 3]
    tx, ty, hist = simulate(track, {'kp': 2.0, 'ki': 0.05, 'kd': 1.0})
    ax.plot(tx, ty, 'k-', linewidth=3, alpha=0.3, label='赛道')
    ax.plot(hist['x'], hist['y'], 'r-', linewidth=1, label='车辆轨迹')
    ax.set_title(f'{name}')
    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')
    print(f"  [{name}] 最大横向误差: {np.max(np.abs(hist['error'])):.1f} mm, 平均: {np.mean(np.abs(hist['error'])):.1f} mm")

# 图2: 不同PID参数对比 (S弯赛道)
fig2, axes2 = plt.subplots(1, 3, figsize=(16, 5))
fig2.suptitle('S弯赛道 - 不同PID参数对比', fontsize=14, fontweight='bold')

tx, ty, _ = generate_track('s_curve')
for idx, (name, params) in enumerate(pid_configs.items()):
    ax = axes2[idx]
    _, _, hist = simulate('s_curve', params)
    ax.plot(tx, ty, 'k-', linewidth=3, alpha=0.3, label='赛道')
    ax.plot(hist['x'], hist['y'], 'r-', linewidth=1, label='轨迹')
    ax.set_title(name)
    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')

# 图3: 误差对比
fig3, ax3 = plt.subplots(figsize=(12, 5))
fig3.suptitle('S弯赛道 - 不同PID参数误差对比', fontsize=14, fontweight='bold')
for name, params in pid_configs.items():
    _, _, hist = simulate('s_curve', params)
    ax3.plot(np.array(range(len(hist['error']))) * 0.01, np.abs(hist['error']),
             linewidth=0.8, alpha=0.7, label=name)
ax3.set_xlabel('时间 (s)')
ax3.set_ylabel('横向误差 (mm)')
ax3.legend()
ax3.grid(True, alpha=0.3)

for fig in [fig1, fig2, fig3]:
    fig.tight_layout()

fig1.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'line_follow_tracks.png'), dpi=150)
fig2.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'line_follow_pid_compare.png'), dpi=150)
fig3.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'line_follow_error_compare.png'), dpi=150)
print("图表已保存")
plt.close('all')

if __name__ == '__main__':
    simulate()
