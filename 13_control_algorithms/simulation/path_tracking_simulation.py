"""
Pure Pursuit 路径跟踪仿真
==========================
模拟车辆跟踪参考路径，可视化前视圆、横向误差和转向角。
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import matplotlib

# 中文字体
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial Unicode MS']
matplotlib.rcParams['axes.unicode_minus'] = False


class BicycleModel:
    """简化的自行车运动模型"""

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, wheelbase=0.5):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.L = wheelbase
        # 历史轨迹
        self.trail_x = [x]
        self.trail_y = [y]

    def update(self, steering, target_speed, dt=0.05):
        """状态更新(自行车模型)"""
        # 简单速度控制
        accel = np.clip((target_speed - self.v) * 2.0, -2.0, 2.0)
        self.v += accel * dt
        self.v = np.clip(self.v, -1.0, 3.0)

        # 运动学
        self.x += self.v * np.cos(self.yaw) * dt
        self.y += self.v * np.sin(self.yaw) * dt
        self.yaw += (self.v / self.L) * np.tan(steering) * dt
        self.yaw = np.arctan2(np.sin(self.yaw), np.cos(self.yaw))

        self.trail_x.append(self.x)
        self.trail_y.append(self.y)


class PurePursuitController:
    """Pure Pursuit 路径跟踪控制器"""

    def __init__(self, path_x, path_y, wheelbase=0.5,
                 lookahead_base=0.5, lookahead_speed_coeff=0.3,
                 target_speed=1.0, max_steer=np.radians(30)):
        self.path_x = np.array(path_x)
        self.path_y = np.array(path_y)
        self.wheelbase = wheelbase
        self.lookahead_base = lookahead_base
        self.lookahead_speed_coeff = lookahead_speed_coeff
        self.target_speed = target_speed
        self.max_steer = max_steer
        self.last_idx = 0

    def find_nearest(self, x, y):
        """找最近路径点"""
        dx = self.path_x - x
        dy = self.path_y - y
        dists = dx**2 + dy**2
        # 从上次索引开始搜索加速
        n = len(self.path_x)
        search_start = max(0, self.last_idx - 10)
        search_end = min(n, self.last_idx + 50)
        local_dists = dists[search_start:search_end]
        if len(local_dists) == 0:
            return 0
        best = search_start + np.argmin(local_dists)
        self.last_idx = best
        return best

    def compute(self, x, y, yaw, v):
        """计算控制量"""
        # 前视距离
        Ld = self.lookahead_base + self.lookahead_speed_coeff * abs(v)
        Ld = np.clip(Ld, 0.3, 8.0)

        # 找目标点(距离 >= Ld)
        nearest = self.find_nearest(x, y)
        target_idx = nearest
        for i in range(nearest, len(self.path_x)):
            d = np.hypot(self.path_x[i] - x, self.path_y[i] - y)
            if d >= Ld:
                target_idx = i
                break
        else:
            target_idx = len(self.path_x) - 1

        tx = self.path_x[target_idx]
        ty = self.path_y[target_idx]

        # 转换到车辆坐标系
        dx = tx - x
        dy = ty - y
        local_x = np.cos(yaw) * dx + np.sin(yaw) * dy
        local_y = -np.sin(yaw) * dx + np.cos(yaw) * dy

        # 曲率和转向角
        curvature = 2.0 * local_y / (Ld**2) if Ld > 1e-3 else 0.0
        steering = np.arctan(self.wheelbase * curvature)
        steering = np.clip(steering, -self.max_steer, self.max_steer)

        # 弯道减速
        speed_factor = 1.0 - 0.5 * np.clip(abs(curvature) * 5.0, 0, 0.8)
        target_speed = self.target_speed * speed_factor

        return steering, target_speed, curvature, Ld, target_idx, (tx, ty)


def generate_figure8_path(n_points=200):
    """生成8字形参考路径"""
    t = np.linspace(0, 2 * np.pi, n_points)
    x = 3.0 * np.sin(t)
    y = 1.5 * np.sin(2 * t)
    return x, y


def generate_smooth_path(n_points=300):
    """生成平滑S形路径"""
    t = np.linspace(0, 4 * np.pi, n_points)
    x = t / (4 * np.pi) * 10.0
    y = 2.0 * np.sin(0.5 * t) + 0.5 * np.sin(1.5 * t)
    return x, y


def run_simulation(path_type='figure8', total_time=30.0, dt=0.05):
    """运行仿真"""
    # 生成路径
    if path_type == 'figure8':
        path_x, path_y = generate_figure8_path(300)
    else:
        path_x, path_y = generate_smooth_path(300)

    # 初始化
    model = BicycleModel(x=path_x[0] - 0.5, y=path_y[0] - 0.5,
                          yaw=0.0, v=0.0, wheelbase=0.5)
    controller = PurePursuitController(path_x, path_y,
                                        wheelbase=0.5,
                                        lookahead_base=0.6,
                                        target_speed=1.5)

    steps = int(total_time / dt)
    steer_hist = []
    speed_hist = []
    cte_hist = []
    err_hist = []

    # 仿真主循环
    for _ in range(steps):
        steer, tgt_speed, curv, Ld, tidx, tpt = controller.compute(
            model.x, model.y, model.yaw, model.v)
        model.update(steer, tgt_speed, dt)

        steer_hist.append(np.degrees(steer))
        speed_hist.append(model.v)
        # 横向误差
        dx = model.path_x - model.x
        dy = model.path_y - model.y
        cte_hist.append(np.min(np.hypot(dx, dy)))
        err_hist.append(np.hypot(model.x - path_x[tidx], model.y - path_y[tidx]))

    # 可视化
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Pure Pursuit 路径跟踪仿真', fontsize=16, fontweight='bold')

    # 1. 路径与轨迹
    ax = axes[0, 0]
    ax.plot(path_x, path_y, 'b--', linewidth=2, label='参考路径')
    ax.plot(model.trail_x, model.trail_y, 'r-', linewidth=1.5, alpha=0.8, label='跟踪轨迹')
    ax.plot(model.trail_x[0], model.trail_y[0], 'go', markersize=10, label='起点')
    ax.plot(model.trail_x[-1], model.trail_y[-1], 'r*', markersize=12, label='终点')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('路径跟踪轨迹')
    ax.legend()
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)

    # 2. 横向误差
    ax = axes[0, 1]
    time_arr = np.arange(len(cte_hist)) * dt
    ax.plot(time_arr, cte_hist, 'g-', linewidth=1.2)
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('横向误差 (m)')
    ax.set_title(f'横向跟踪误差 (均值: {np.mean(cte_hist):.3f}m)')
    ax.grid(True, alpha=0.3)

    # 3. 转向角
    ax = axes[1, 0]
    ax.plot(time_arr, steer_hist, 'b-', linewidth=1.0)
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('转向角 (°)')
    ax.set_title('转向角变化')
    ax.grid(True, alpha=0.3)

    # 4. 速度
    ax = axes[1, 1]
    ax.plot(time_arr, speed_hist, 'r-', linewidth=1.2)
    ax.axhline(y=controller.target_speed, color='gray', linestyle='--', alpha=0.5, label='目标速度')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('速度 (m/s)')
    ax.set_title('纵向速度')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('pure_pursuit_simulation.png', dpi=150, bbox_inches='tight')
    plt.close('all')
    print(f'仿真完成! 横向误差: 均值={np.mean(cte_hist):.4f}m, 最大={np.max(cte_hist):.4f}m')


if __name__ == '__main__':
    run_simulation(path_type='figure8', total_time=25.0)
