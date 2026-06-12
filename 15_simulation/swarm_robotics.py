#!/usr/bin/env python3
"""
群机器人仿真 — 编队 + 避碰 + 覆盖 + 通信
==========================================
- 多种编队模式（线形/三角/圆形/自定义）
- 基于人工势场的避碰
- Voronoi覆盖控制
- 通信拓扑与一致性
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rcParams
from scipy.spatial import Voronoi, voronoi_plot_2d
rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
rcParams['axes.unicode_minus'] = False

# ============================================================
# 1. 机器人个体
# ============================================================
class Robot:
    _id_counter = 0

    def __init__(self, x, y, v_max=1.0):
        self.id = Robot._id_counter
        Robot._id_counter += 1
        self.pos = np.array([x, y], dtype=float)
        self.vel = np.array([0.0, 0.0])
        self.v_max = v_max
        self.neighbors = []  # 通信邻居
        self.comm_range = 3.0
        self.safe_dist = 0.3

    def step(self, dt=0.05):
        speed = np.linalg.norm(self.vel)
        if speed > self.v_max:
            self.vel = self.vel / speed * self.v_max
        self.pos += self.vel * dt


# ============================================================
# 2. 编队控制
# ============================================================
class FormationController:
    """基于一致性的编队控制"""

    FORMATIONS = {
        'line': lambda n: [(i - (n-1)/2, 0) for i in range(n)],
        'triangle': lambda n: [(i*0.8 - (n//2)*0.8, -0.7*(i%2)) for i in range(n)],
        'circle': lambda n: [(np.cos(2*np.pi*i/n), np.sin(2*np.pi*i/n)) for i in range(n)],
        'v_shape': lambda n: [((i//2)*(-1)**i*0.8, -(i//2)*0.6) for i in range(n)],
    }

    def __init__(self, robots, formation='circle'):
        self.robots = robots
        self.n = len(robots)
        self.set_formation(formation)

    def set_formation(self, name):
        self.formation_name = name
        gen = self.FORMATIONS.get(name, self.FORMATIONS['circle'])
        offsets = gen(self.n)
        self.desired_offsets = [np.array(o) for o in offsets]

    def compute_control(self, leader_pos=np.array([0, 0])):
        """编队控制律: 趋向编队位置 + 避碰 + 一致性"""
        for i, robot in enumerate(self.robots):
            target = leader_pos + self.desired_offsets[i % len(self.desired_offsets)]
            # 1. 编队保持力
            f_formation = 2.0 * (target - robot.pos)
            # 2. 避碰力
            f_avoid = np.zeros(2)
            for j, other in enumerate(self.robots):
                if i == j:
                    continue
                diff = robot.pos - other.pos
                dist = np.linalg.norm(diff)
                if dist < robot.safe_dist * 3 and dist > 0.01:
                    f_avoid += 5.0 * diff / dist**2 * np.exp(-dist / robot.safe_dist)
            # 3. 一致性（速度同步）
            f_consensus = np.zeros(2)
            count = 0
            for j, other in enumerate(self.robots):
                if i == j:
                    continue
                diff = other.pos - robot.pos
                dist = np.linalg.norm(diff)
                if dist < robot.comm_range:
                    f_consensus += 0.5 * (other.vel - robot.vel)
                    count += 1
            if count > 0:
                f_consensus /= count

            robot.vel = f_formation + f_avoid + f_consensus


# ============================================================
# 3. Voronoi覆盖控制
# ============================================================
class CoverageController:
    """基于Voronoi的覆盖控制"""

    def __init__(self, robots, area=(-5, 5, -5, 5)):
        self.robots = robots
        self.area = area  # (xmin, xmax, ymin, ymax)

    def compute_voronoi_centroids(self, density_fn=None, n_samples=200):
        """计算每个机器人的Voronoi质心"""
        positions = np.array([r.pos for r in self.robots])
        xmin, xmax, ymin, ymax = self.area

        # 加采样边界点防止Voronoi无限区域
        boundary = []
        for x in np.linspace(xmin, xmax, 20):
            boundary.extend([(x, ymin), (x, ymax)])
        for y in np.linspace(ymin, ymax, 20):
            boundary.extend([(xmin, y), (xmax, y)])
        boundary = np.array(boundary)

        all_points = np.vstack([positions, boundary])
        vor = Voronoi(all_points)

        centroids = []
        for i in range(len(self.robots)):
            # 找到机器人i的Voronoi区域
            region_idx = vor.point_region[i]
            region = vor.regions[region_idx]
            if -1 in region or len(region) == 0:
                centroids.append(positions[i])
                continue

            verts = np.array([vor.vertices[v] for v in region])
            # 蒙特卡洛积分求质心
            inside_x = np.random.uniform(verts[:, 0].min(), verts[:, 0].max(), n_samples)
            inside_y = np.random.uniform(verts[:, 1].min(), verts[:, 1].max(), n_samples)

            # 简单点在多边形内检测
            from matplotlib.path import Path
            path = Path(verts)
            mask = path.contains_points(np.column_stack([inside_x, inside_y]))

            if mask.sum() > 0:
                if density_fn is not None:
                    weights = density_fn(inside_x[mask], inside_y[mask])
                    cx = np.average(inside_x[mask], weights=weights)
                    cy = np.average(inside_y[mask], weights=weights)
                else:
                    cx = inside_x[mask].mean()
                    cy = inside_y[mask].mean()
                centroids.append(np.array([cx, cy]))
            else:
                centroids.append(positions[i])

        return centroids

    def compute_control(self):
        """覆盖控制律: 趋向Voronoi质心"""
        centroids = self.compute_voronoi_centroids()
        for robot, centroid in zip(self.robots, centroids):
            robot.vel = 1.5 * (centroid - robot.pos)


# ============================================================
# 4. 通信管理
# ============================================================
class CommNetwork:
    """通信拓扑管理"""

    def __init__(self, robots):
        self.robots = robots
        self.adj_matrix = np.zeros((len(robots), len(robots)))
        self.msg_buffer = {r.id: [] for r in robots}

    def update_topology(self):
        """基于距离更新通信拓扑"""
        n = len(self.robots)
        self.adj_matrix = np.zeros((n, n))
        for i in range(n):
            for j in range(i+1, n):
                dist = np.linalg.norm(self.robots[i].pos - self.robots[j].pos)
                if dist < self.robots[i].comm_range:
                    self.adj_matrix[i, j] = 1
                    self.adj_matrix[j, i] = 1

    def broadcast(self, robot_id, msg):
        """广播消息给邻居"""
        for j, other in enumerate(self.robots):
            if self.adj_matrix[robot_id, j]:
                self.msg_buffer[other.id].append(msg)

    def get_neighbors(self, robot_id):
        """获取邻居列表"""
        return [j for j in range(len(self.robots)) if self.adj_matrix[robot_id, j]]

    def connectivity(self):
        """检查图连通性（BFS）"""
        n = len(self.robots)
        visited = set()
        queue = [0]
        visited.add(0)
        while queue:
            node = queue.pop(0)
            for j in range(n):
                if self.adj_matrix[node, j] and j not in visited:
                    visited.add(j)
                    queue.append(j)
        return len(visited) == n


# ============================================================
# 5. 仿真场景
# ============================================================
def run_formation_scenario(n_robots=8, T=15):
    """编队仿真"""
    Robot._id_counter = 0
    robots = [Robot(np.random.uniform(-3, 3), np.random.uniform(-3, 3)) for _ in range(n_robots)]
    ctrl = FormationController(robots, 'circle')
    comm = CommNetwork(robots)

    dt = 0.05
    steps = int(T / dt)
    history = np.zeros((steps, n_robots, 2))

    # 领航者轨迹：8字形
    for t in range(steps):
        time = t * dt
        leader_pos = np.array([2*np.cos(0.3*time), 1.5*np.sin(0.6*time)])

        # 编队模式切换
        if abs(time - 5.0) < dt/2:
            ctrl.set_formation('v_shape')
        if abs(time - 10.0) < dt/2:
            ctrl.set_formation('triangle')

        ctrl.compute_control(leader_pos)
        comm.update_topology()

        for i, r in enumerate(robots):
            r.step(dt)
            history[t, i] = r.pos.copy()

    return history, robots


def run_coverage_scenario(n_robots=10, T=30):
    """覆盖仿真"""
    Robot._id_counter = 100
    robots = [Robot(np.random.uniform(-3, 3), np.random.uniform(-3, 3)) for _ in range(n_robots)]
    ctrl = CoverageController(robots, area=(-5, 5, -5, 5))

    # 密度函数：中心区域密度高
    def density(x, y):
        return np.exp(-0.3*(x**2 + y**2)) + 0.1

    dt = 0.05
    steps = int(T / dt)
    history = np.zeros((steps, n_robots, 2))

    for t in range(steps):
        ctrl.density_fn = density
        ctrl.compute_control()
        for i, r in enumerate(robots):
            r.step(dt)
            history[t, i] = r.pos.copy()

    return history, robots


# ============================================================
# 6. 可视化
# ============================================================
def plot_results(form_hist, cover_hist):
    fig, axes = plt.subplots(2, 2, figsize=(14, 12))
    fig.suptitle('群机器人仿真 — 编队 + 覆盖 + 通信', fontsize=15, fontweight='bold')

    # (a) 编队轨迹
    ax = axes[0, 0]
    n_robots = form_hist.shape[1]
    for i in range(n_robots):
        ax.plot(form_hist[:, i, 0], form_hist[:, i, 1], lw=0.5, alpha=0.6)
        # 标记初始/最终位置
        ax.plot(form_hist[0, i, 0], form_hist[0, i, 1], 'o', ms=5, color='red')
        ax.plot(form_hist[-1, i, 0], form_hist[-1, i, 1], 's', ms=5, color='blue')
    # 画最终编队连线
    for i in range(n_robots):
        for j in range(i+1, n_robots):
            dist = np.linalg.norm(form_hist[-1, i] - form_hist[-1, j])
            if dist < 3.0:
                ax.plot([form_hist[-1, i, 0], form_hist[-1, j, 0]],
                        [form_hist[-1, i, 1], form_hist[-1, j, 1]], 'b-', lw=0.5, alpha=0.3)
    ax.set_aspect('equal')
    ax.set_title('(a) 编队控制轨迹')
    ax.grid(True, alpha=0.3)

    # (b) 编队距离误差
    ax = axes[0, 1]
    n_robots = form_hist.shape[1]
    # 计算每步的编队误差
    from itertools import combinations
    pair_dists = []
    for i, j in combinations(range(n_robots), 2):
        d = np.linalg.norm(form_hist[:, i] - form_hist[:, j], axis=1)
        pair_dists.append(d)
    pair_dists = np.array(pair_dists)
    ax.plot(pair_dists.mean(axis=0), lw=1, label='平均距离')
    ax.plot(pair_dists.std(axis=0), lw=1, label='距离标准差')
    ax.set_xlabel('时间步')
    ax.set_ylabel('距离')
    ax.set_title('(b) 编队一致性指标')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # (c) 覆盖最终分布
    ax = axes[1, 0]
    n_cover = cover_hist.shape[1]
    for i in range(n_cover):
        ax.plot(cover_hist[:, i, 0], cover_hist[:, i, 1], lw=0.3, alpha=0.4)
        ax.plot(cover_hist[-1, i, 0], cover_hist[-1, i, 1], 'ro', ms=6)
    # 画Voronoi
    final_pos = cover_hist[-1]
    if len(final_pos) >= 4:
        try:
            vor = Voronoi(final_pos)
            voronoi_plot_2d(vor, ax=ax, show_vertices=False, line_colors='green',
                            line_width=0.8, alpha=0.5)
        except Exception:
            pass
    ax.set_xlim(-5, 5); ax.set_ylim(-5, 5)
    ax.set_aspect('equal')
    ax.set_title('(c) Voronoi覆盖控制')
    ax.grid(True, alpha=0.3)

    # (d) 通信拓扑
    ax = axes[1, 1]
    Robot._id_counter = 200
    robots_demo = [Robot(final_pos[i, 0], final_pos[i, 1]) for i in range(n_cover)]
    comm = CommNetwork(robots_demo)
    comm.update_topology()
    for i in range(n_cover):
        for j in range(i+1, n_cover):
            if comm.adj_matrix[i, j]:
                ax.plot([final_pos[i, 0], final_pos[j, 0]],
                        [final_pos[i, 1], final_pos[j, 1]], 'g-', lw=1, alpha=0.5)
    ax.plot(final_pos[:, 0], final_pos[:, 1], 'ro', ms=8)
    for i in range(n_cover):
        ax.annotate(str(i), final_pos[i], fontsize=8, ha='center', va='bottom')
    ax.set_xlim(-5, 5); ax.set_ylim(-5, 5)
    ax.set_aspect('equal')
    ax.set_title(f'(d) 通信拓扑 (连通={comm.connectivity()})')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('swarm_robotics.png', dpi=150, bbox_inches='tight')
    plt.show()


# ============================================================
if __name__ == '__main__':
    print('1. 编队控制仿真...')
    form_hist, _ = run_formation_scenario(n_robots=8, T=15)

    print('2. 覆盖控制仿真...')
    cover_hist, _ = run_coverage_scenario(n_robots=10, T=30)

    plot_results(form_hist, cover_hist)
    print('\n群机器人仿真完成！')
