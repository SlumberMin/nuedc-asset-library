#!/usr/bin/env python3
"""
路径跟踪仿真 - Pure Pursuit + Stanley 控制器
==============================================
对比两种经典路径跟踪算法的性能。
- Pure Pursuit: 基于几何的前瞻点跟踪
- Stanley: 基于横向误差和航向误差的跟踪

运行: python path_tracking_simulation.py
依赖: numpy, matplotlib
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import matplotlib.patches as mpatches

# ========== 路径生成 ==========
def generate_figure8(length=200):
    """生成8字形路径"""
    t = np.linspace(0, 2 * np.pi, length)
    x = 10 * np.sin(t)
    y = 5 * np.sin(2 * t)
    return x, y

def generate_circle(radius=10, length=200):
    """生成圆形路径"""
    t = np.linspace(0, 2 * np.pi, length, endpoint=False)
    x = radius * np.cos(t)
    y = radius * np.sin(t)
    return x, y

def generate_sine_wave(length=300):
    """生成正弦路径"""
    x = np.linspace(0, 40, length)
    y = 5 * np.sin(0.5 * x)
    return x, y

def compute_path_headings(x, y):
    """计算路径各点的航向角"""
    dx = np.gradient(x)
    dy = np.gradient(y)
    headings = np.arctan2(dy, dx)
    return headings


# ========== 车辆模型 ==========
class BicycleModel:
    """简化的自行车动力学模型"""

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=2.0, L=2.5):
        self.x = x          # 后轴中心x
        self.y = y          # 后轴中心y
        self.yaw = yaw      # 航向角
        self.v = v          # 速度 (m/s)
        self.L = L          # 轴距 (m)
        self.delta = 0.0    # 前轮转角

    def update(self, delta, dt=0.05):
        """更新车辆状态"""
        self.delta = np.clip(delta, -np.radians(30), np.radians(30))
        self.x += self.v * np.cos(self.yaw) * dt
        self.y += self.v * np.sin(self.yaw) * dt
        self.yaw += self.v / self.L * np.tan(self.delta) * dt
        self.yaw = self._normalize_angle(self.yaw)

    @staticmethod
    def _normalize_angle(angle):
        while angle > np.pi: angle -= 2 * np.pi
        while angle < -np.pi: angle += 2 * np.pi
        return angle


# ========== Pure Pursuit 控制器 ==========
class PurePursuitController:
    """Pure Pursuit 前瞻点跟踪控制器"""

    def __init__(self, lookahead_dist=3.0, L=2.5):
        self.ld = lookahead_dist
        self.L = L

    def find_target(self, x, y, path_x, path_y):
        """找到前瞻点"""
        dists = np.sqrt((path_x - x)**2 + (path_y - y)**2)
        # 找到距离前瞻距离最近的点
        min_idx = np.argmin(np.abs(dists - self.ld))
        return path_x[min_idx], path_y[min_idx], min_idx

    def compute_steering(self, vehicle, path_x, path_y):
        """计算转向角"""
        tx, ty, _ = self.find_target(vehicle.x, vehicle.y, path_x, path_y)

        # 车辆坐标系下前瞻点位置
        dx = tx - vehicle.x
        dy = ty - vehicle.y
        local_x = dx * np.cos(vehicle.yaw) + dy * np.sin(vehicle.yaw)
        local_y = -dx * np.sin(vehicle.yaw) + dy * np.cos(vehicle.yaw)

        # 曲率 -> 转向角
        if local_x < 0.1:
            return 0.0
        curvature = 2.0 * local_y / (self.ld ** 2)
        delta = np.arctan(curvature * self.L)
        return delta


# ========== Stanley 控制器 ==========
class StanleyController:
    """Stanley 横向跟踪控制器"""

    def __init__(self, k_e=2.5, L=2.5):
        self.k_e = k_e   # 横向误差增益
        self.L = L

    def compute_steering(self, vehicle, path_x, path_y, path_headings):
        """计算转向角"""
        # 找最近路径点
        dists = np.sqrt((path_x - vehicle.x)**2 + (path_y - vehicle.y)**2)
        min_idx = np.argmin(dists)

        # 横向误差 (带符号)
        dx = vehicle.x - path_x[min_idx]
        dy = vehicle.y - path_y[min_idx]
        path_yaw = path_headings[min_idx]
        cross_track_error = -dx * np.sin(path_yaw) + dy * np.cos(path_yaw)

        # 航向误差
        heading_error = BicycleModel._normalize_angle(path_yaw - vehicle.yaw)

        # Stanley 控制律
        if vehicle.v < 0.1:
            delta = heading_error
        else:
            delta = heading_error + np.arctan2(self.k_e * cross_track_error, vehicle.v)

        return delta, cross_track_error


# ========== 仿真主循环 ==========
def run_tracking(controller_type, path_func, duration=30.0, dt=0.05):
    """运行路径跟踪仿真"""
    path_x, path_y = path_func()
    path_headings = compute_path_headings(path_x, path_y)

    # 初始化车辆（在路径起点附近）
    start_idx = 0
    vehicle = BicycleModel(
        x=path_x[start_idx] - 1.0,
        y=path_y[start_idx] - 1.0,
        yaw=path_headings[start_idx],
        v=2.5
    )

    if controller_type == 'pure_pursuit':
        ctrl = PurePursuitController(lookahead_dist=3.0)
    else:
        ctrl = StanleyController(k_e=2.5)

    steps = int(duration / dt)
    results = {
        'time': [], 'x': [], 'y': [], 'yaw': [],
        'cte': [], 'target_x': [], 'target_y': []
    }

    for i in range(steps):
        t = i * dt

        if controller_type == 'pure_pursuit':
            delta = ctrl.compute_steering(vehicle, path_x, path_y)
            # 计算横向误差
            dists = np.sqrt((path_x - vehicle.x)**2 + (path_y - vehicle.y)**2)
            min_idx = np.argmin(dists)
            dx = vehicle.x - path_x[min_idx]
            dy = vehicle.y - path_y[min_idx]
            cte = -dx * np.sin(path_headings[min_idx]) + dy * np.cos(path_headings[min_idx])
            tx, ty, _ = ctrl.find_target(vehicle.x, vehicle.y, path_x, path_y)
        else:
            delta, cte = ctrl.compute_steering(vehicle, path_x, path_y, path_headings)
            dists = np.sqrt((path_x - vehicle.x)**2 + (path_y - vehicle.y)**2)
            min_idx = np.argmin(dists)
            tx, ty = path_x[min_idx], path_y[min_idx]

        vehicle.update(delta, dt)

        results['time'].append(t)
        results['x'].append(vehicle.x)
        results['y'].append(vehicle.y)
        results['yaw'].append(vehicle.yaw)
        results['cte'].append(cte)
        results['target_x'].append(tx)
        results['target_y'].append(ty)

    return results, path_x, path_y


# ========== 可视化 ==========
def plot_comparison(path_name, path_func):
    """对比 Pure Pursuit 和 Stanley"""
    print(f"  仿真路径: {path_name}")

    res_pp, px, py = run_tracking('pure_pursuit', path_func)
    res_st, _, _ = run_tracking('stanley', path_func)

    fig, axes = plt.subplots(2, 2, figsize=(15, 12))
    fig.suptitle(f'路径跟踪仿真对比 - {path_name}', fontsize=16, fontweight='bold')

    # 路径跟踪轨迹
    ax = axes[0, 0]
    ax.plot(px, py, 'k--', linewidth=2, label='参考路径')
    ax.plot(res_pp['x'], res_pp['y'], 'b-', linewidth=1.5, label='Pure Pursuit', alpha=0.8)
    ax.plot(res_st['x'], res_st['y'], 'r-', linewidth=1.5, label='Stanley', alpha=0.8)
    ax.set_title('路径跟踪轨迹')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')

    # 横向误差
    ax = axes[0, 1]
    ax.plot(res_pp['time'], np.array(res_pp['cte']) * 100, 'b-', label='Pure Pursuit', linewidth=1.5)
    ax.plot(res_st['time'], np.array(res_st['cte']) * 100, 'r-', label='Stanley', linewidth=1.5)
    ax.set_title('横向跟踪误差')
    ax.set_ylabel('横向误差 (cm)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 航向角
    ax = axes[1, 0]
    ax.plot(res_pp['time'], np.degrees(res_pp['yaw']), 'b-', label='Pure Pursuit', linewidth=1.5)
    ax.plot(res_st['time'], np.degrees(res_st['yaw']), 'r-', label='Stanley', linewidth=1.5)
    ax.set_title('航向角')
    ax.set_ylabel('航向角 (°)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 误差统计
    ax = axes[1, 1]
    pp_cte = np.abs(np.array(res_pp['cte'])) * 100
    st_cte = np.abs(np.array(res_st['cte'])) * 100
    labels = ['Pure Pursuit', 'Stanley']
    means = [np.mean(pp_cte), np.mean(st_cte)]
    maxs = [np.max(pp_cte), np.max(st_cte)]
    rms = [np.sqrt(np.mean(pp_cte**2)), np.sqrt(np.mean(st_cte**2))]

    x_pos = np.arange(len(labels))
    width = 0.25
    ax.bar(x_pos - width, means, width, label='平均误差', color='#2196F3')
    ax.bar(x_pos, maxs, width, label='最大误差', color='#F44336')
    ax.bar(x_pos + width, rms, width, label='RMS误差', color='#4CAF50')
    ax.set_title('误差统计对比')
    ax.set_ylabel('误差 (cm)')
    ax.set_xticks(x_pos)
    ax.set_xticklabels(labels)
    ax.legend()
    ax.grid(True, alpha=0.3, axis='y')

    for ax in axes.flat:
        ax.set_xlabel('时间 (s)' if 'time' in ax.get_xlabel().lower() or ax.get_xlabel() == '' else ax.get_xlabel())

    plt.tight_layout()
    filename = f'path_tracking_{path_name.replace(" ", "_")}.png'
    plt.savefig(filename, dpi=150, bbox_inches='tight')
    plt.close('all')

    # 打印指标
    print(f"    Pure Pursuit - 平均误差: {np.mean(pp_cte):.1f}cm, 最大: {np.max(pp_cte):.1f}cm, RMS: {np.sqrt(np.mean(pp_cte**2)):.1f}cm")
    print(f"    Stanley      - 平均误差: {np.mean(st_cte):.1f}cm, 最大: {np.max(st_cte):.1f}cm, RMS: {np.sqrt(np.mean(st_cte**2)):.1f}cm")


# ========== 主程序 ==========
if __name__ == '__main__':
    print("=" * 60)
    print("路径跟踪仿真 - Pure Pursuit + Stanley 控制器")
    print("=" * 60)

    scenarios = {
        '圆形路径': generate_circle,
        'S形路径': generate_sine_wave,
        '8字路径': generate_figure8,
    }

    for name, func in scenarios.items():
        plot_comparison(name, func)

    print("\n仿真完成！所有结果已保存为PNG文件。")
