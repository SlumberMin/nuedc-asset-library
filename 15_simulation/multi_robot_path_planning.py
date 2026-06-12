#!/usr/bin/env python3
"""
多机器人路径规划仿真
- A* 算法
- RRT* 算法
- PRM (概率路线图)
- 冲突解决 (CBS / 优先级规划)
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from typing import List, Tuple, Optional, Dict, Set
from dataclasses import dataclass, field
from heapq import heappush, heappop
import random

# ============================================================
# 1. 地图与环境
# ============================================================

@dataclass
class GridMap:
    width: int
    height: int
    obstacles: Set[Tuple[int, int]] = field(default_factory=set)

    def is_valid(self, x: int, y: int) -> bool:
        return 0 <= x < self.width and 0 <= y < self.height and (x, y) not in self.obstacles

    def neighbors_8(self, x: int, y: int) -> List[Tuple[int, int]]:
        dirs = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]
        return [(x+dx, y+dy) for dx, dy in dirs if self.is_valid(x+dx, y+dy)]

    def neighbors_4(self, x: int, y: int) -> List[Tuple[int, int]]:
        dirs = [(-1,0),(1,0),(0,-1),(0,1)]
        return [(x+dx, y+dy) for dx, dy in dirs if self.is_valid(x+dx, y+dy)]

def generate_random_map(width: int, height: int, obstacle_ratio: float = 0.2) -> GridMap:
    gm = GridMap(width, height)
    for _ in range(int(width * height * obstacle_ratio)):
        x, y = random.randint(0, width-1), random.randint(0, height-1)
        gm.obstacles.add((x, y))
    return gm


# ============================================================
# 2. A* 算法
# ============================================================

def heuristic(a: Tuple[int,int], b: Tuple[int,int]) -> float:
    """对角线距离"""
    dx, dy = abs(a[0]-b[0]), abs(a[1]-b[1])
    return max(dx, dy) + (np.sqrt(2)-1)*min(dx, dy)

def astar(grid: GridMap, start: Tuple[int,int], goal: Tuple[int,int]) -> Optional[List[Tuple[int,int]]]:
    """A* 寻路"""
    open_set = [(0, start)]
    came_from = {}
    g_score = {start: 0}
    visited = set()

    while open_set:
        _, current = heappop(open_set)
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return path[::-1]

        if current in visited:
            continue
        visited.add(current)

        for nb in grid.neighbors_8(*current):
            dx, dy = abs(nb[0]-current[0]), abs(nb[1]-current[1])
            cost = np.sqrt(2) if dx+dy == 2 else 1.0
            tentative = g_score[current] + cost
            if tentative < g_score.get(nb, float('inf')):
                came_from[nb] = current
                g_score[nb] = tentative
                heappush(open_set, (tentative + heuristic(nb, goal), nb))

    return None  # 无路径


# ============================================================
# 3. RRT* 算法
# ============================================================

@dataclass
class RRTNode:
    x: float
    y: float
    parent: Optional[int] = None
    cost: float = 0.0

def rrt_star(grid: GridMap, start: Tuple[int,int], goal: Tuple[int,int],
             max_iter: int = 3000, step_size: float = 2.0, radius: float = 3.0) -> Optional[List[Tuple[float,float]]]:
    """RRT* 寻路"""
    nodes = [RRTNode(start[0]+0.5, start[1]+0.5)]
    goal_node = RRTNode(goal[0]+0.5, goal[1]+0.5)

    for _ in range(max_iter):
        # 随机采样 (10% 采样目标)
        if random.random() < 0.1:
            rx, ry = goal_node.x, goal_node.y
        else:
            rx = random.uniform(0, grid.width)
            ry = random.uniform(0, grid.height)

        # 最近节点
        dists = [(n.x-rx)**2 + (n.y-ry)**2 for n in nodes]
        nearest_idx = int(np.argmin(dists))
        nearest = nodes[nearest_idx]

        # 扩展
        dx, dy = rx - nearest.x, ry - nearest.y
        d = np.sqrt(dx*dx + dy*dy)
        if d < 1e-6:
            continue
        nx = nearest.x + step_size * dx / d
        ny = nearest.y + step_size * dy / d

        # 碰撞检测 (简化: 网格检查)
        gx, gy = int(nx), int(ny)
        if not grid.is_valid(gx, gy):
            continue

        new_cost = nearest.cost + step_size

        # 选择最优父节点 (rewire)
        best_parent = nearest_idx
        best_cost = new_cost
        for i, nd in enumerate(nodes):
            dd = np.sqrt((nd.x-nx)**2 + (nd.y-ny)**2)
            if dd < radius and nd.cost + dd < best_cost:
                best_parent = i
                best_cost = nd.cost + dd

        new_node = RRTNode(nx, ny, best_parent, best_cost)
        nodes.append(new_node)
        new_idx = len(nodes) - 1

        # Rewire existing nodes
        for i, nd in enumerate(nodes):
            dd = np.sqrt((nd.x-nx)**2 + (nd.y-ny)**2)
            if dd < radius and best_cost + dd < nd.cost:
                nd.parent = new_idx
                nd.cost = best_cost + dd

        # 检查是否到达目标
        if np.sqrt((nx-goal_node.x)**2 + (ny-goal_node.y)**2) < step_size:
            path = [(nx, ny)]
            idx = new_idx
            while idx is not None:
                path.append((nodes[idx].x, nodes[idx].y))
                idx = nodes[idx].parent
            return path[::-1]

    return None


# ============================================================
# 4. PRM (概率路线图)
# ============================================================

def prm(grid: GridMap, n_samples: int = 200, k_neighbors: int = 8) -> Dict:
    """构建 PRM 图"""
    # 随机采样自由空间节点
    nodes = []
    for _ in range(n_samples):
        x = random.randint(0, grid.width-1)
        y = random.randint(0, grid.height-1)
        if grid.is_valid(x, y):
            nodes.append((x+0.5, y+0.5))

    # 连接邻近节点
    edges = {}
    for i, n1 in enumerate(nodes):
        dists = [(np.sqrt((n1[0]-n2[0])**2 + (n1[1]-n2[1])**2), j)
                 for j, n2 in enumerate(nodes) if j != i]
        dists.sort()
        for d, j in dists[:k_neighbors]:
            n2 = nodes[j]
            # 简化碰撞检测: 检查端点
            if grid.is_valid(int(n1[0]), int(n1[1])) and grid.is_valid(int(n2[0]), int(n2[1])):
                edges.setdefault(i, []).append((j, d))
                edges.setdefault(j, []).append((i, d))

    return {'nodes': nodes, 'edges': edges}

def prm_search(prm_graph: Dict, start: Tuple[int,int], goal: Tuple[int,int]) -> Optional[List[Tuple[float,float]]]:
    """在 PRM 上 A* 搜索"""
    nodes = prm_graph['nodes']
    edges = prm_graph['edges']

    # 找最近节点
    s = min(range(len(nodes)), key=lambda i: (nodes[i][0]-start[0]-0.5)**2 + (nodes[i][1]-start[1]-0.5)**2)
    g = min(range(len(nodes)), key=lambda i: (nodes[i][0]-goal[0]-0.5)**2 + (nodes[i][1]-goal[1]-0.5)**2)

    # A*
    open_set = [(0, s)]
    came_from = {}
    cost = {s: 0}

    while open_set:
        _, current = heappop(open_set)
        if current == g:
            path = []
            while current in came_from:
                path.append(nodes[current])
                current = came_from[current]
            path.append(nodes[s])
            return path[::-1]

        for nb, w in edges.get(current, []):
            nc = cost[current] + w
            if nc < cost.get(nb, float('inf')):
                came_from[nb] = current
                cost[nb] = nc
                heappush(open_set, (nc + heuristic(nodes[nb], nodes[g]), nb))

    return None


# ============================================================
# 5. 多机器人冲突解决 (CBS)
# ============================================================

@dataclass
class Conflict:
    robot_a: int
    robot_b: int
    location: Tuple[int, int]
    time: int

@dataclass
class CBSNode:
    cost: float
    paths: Dict[int, List[Tuple[int,int]]]
    constraints: List = field(default_factory=list)

def detect_conflict(paths: Dict[int, List[Tuple[int,int]]]) -> Optional[Conflict]:
    """检测顶点冲突和边冲突"""
    max_len = max(len(p) for p in paths.values())
    for t in range(max_len):
        positions = {}
        for rid, path in paths.items():
            pos = path[min(t, len(path)-1)]
            if pos in positions:
                return Conflict(positions[pos], rid, pos, t)
            positions[pos] = rid
        # 边冲突
        if t > 0:
            edges = {}
            for rid, path in paths.items():
                t0 = min(t-1, len(path)-1)
                t1 = min(t, len(path)-1)
                edge = (path[t0], path[t1])
                rev = (path[t1], path[t0])
                if rev in edges:
                    return Conflict(edges[rev], rid, path[t0], t)
                edges[edge] = rid
    return None

def cbs(grid: GridMap, starts: List[Tuple[int,int]], goals: List[Tuple[int,int]]) -> Dict[int, List[Tuple[int,int]]]:
    """
    Conflict-Based Search
    简化版: 使用优先级规划
    """
    n = len(starts)
    paths = {}

    # 阶段1: 独立规划
    for i in range(n):
        p = astar(grid, starts[i], goals[i])
        if p is None:
            raise ValueError(f"Robot {i}: 无可行路径")
        paths[i] = p

    # 阶段2: 优先级冲突解决
    for i in range(n):
        for j in range(i+1, n):
            conflict = detect_conflict({i: paths[i], j: paths[j]})
            if conflict:
                # 为 robot j 添加时间扩展障碍
                obs_backup = grid.obstacles.copy()
                for t, pos in enumerate(paths[i]):
                    grid.obstacles.add(pos)
                new_path = astar(grid, starts[j], goals[j])
                if new_path:
                    paths[j] = new_path
                grid.obstacles = obs_backup

    return paths


# ============================================================
# 6. 可视化
# ============================================================

def visualize(grid: GridMap, paths: Dict[int, List[Tuple[int,int]]],
              title: str = "多机器人路径规划"):
    """可视化地图和路径"""
    fig, ax = plt.subplots(figsize=(10, 10))

    # 绘制障碍物
    for ox, oy in grid.obstacles:
        ax.add_patch(plt.Rectangle((ox, oy), 1, 1, color='black'))

    colors = ['red', 'blue', 'green', 'orange', 'purple', 'cyan']
    for rid, path in paths.items():
        xs = [p[0]+0.5 for p in path]
        ys = [p[1]+0.5 for p in path]
        c = colors[rid % len(colors)]
        ax.plot(xs, ys, '-o', color=c, markersize=3, linewidth=2, label=f'Robot {rid}')
        ax.plot(xs[0], ys[0], 's', color=c, markersize=10)
        ax.plot(xs[-1], ys[-1], '*', color=c, markersize=15)

    ax.set_xlim(0, grid.width)
    ax.set_ylim(0, grid.height)
    ax.set_aspect('equal')
    ax.set_title(title)
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('./nuedc-asset-library/15_simulation/multi_robot_path.png', dpi=150)
    plt.show()


# ============================================================
# 7. 主程序
# ============================================================

def demo_astar():
    print("\n--- A* 算法演示 ---")
    random.seed(42)
    grid = generate_random_map(30, 30, 0.2)
    # 清除起点终点附近障碍
    for x in range(3):
        for y in range(3):
            grid.obstacles.discard((x, y))
            grid.obstacles.discard((29-x, 29-y))

    path = astar(grid, (0, 0), (29, 29))
    if path:
        print(f"  路径长度: {len(path)} 步")
        visualize(grid, {0: path}, "A* 路径规划")
    else:
        print("  ❌ 无可行路径")

def demo_rrt_star():
    print("\n--- RRT* 算法演示 ---")
    random.seed(42)
    grid = generate_random_map(30, 30, 0.15)
    for x in range(3):
        for y in range(3):
            grid.obstacles.discard((x, y))
            grid.obstacles.discard((29-x, 29-y))

    path = rrt_star(grid, (0, 0), (29, 29))
    if path:
        print(f"  路径节点: {len(path)}")
        # 转换为网格路径用于可视化
        grid_path = [(int(p[0]), int(p[1])) for p in path]
        visualize(grid, {0: grid_path}, "RRT* 路径规划")
    else:
        print("  ❌ RRT* 未找到路径")

def demo_prm():
    print("\n--- PRM 概率路线图演示 ---")
    random.seed(42)
    grid = generate_random_map(30, 30, 0.15)
    for x in range(3):
        for y in range(3):
            grid.obstacles.discard((x, y))
            grid.obstacles.discard((29-x, 29-y))

    prm_graph = prm(grid, n_samples=300)
    path = prm_search(prm_graph, (0, 0), (29, 29))
    if path:
        print(f"  PRM 节点数: {len(prm_graph['nodes'])}")
        grid_path = [(int(p[0]), int(p[1])) for p in path]
        visualize(grid, {0: grid_path}, "PRM 路径规划")
    else:
        print("  ❌ PRM 未找到路径")

def demo_multi_robot():
    print("\n--- 多机器人冲突解决演示 ---")
    random.seed(42)
    grid = generate_random_map(25, 25, 0.15)
    # 清除起点终点
    for s, g in [((0,0),(24,24)), ((24,0),(0,24)), ((12,0),(12,24))]:
        for p in [s, g]:
            for dx in range(-1, 2):
                for dy in range(-1, 2):
                    grid.obstacles.discard((p[0]+dx, p[1]+dy))

    starts = [(0,0), (24,0), (12,0)]
    goals  = [(24,24), (0,24), (12,24)]

    paths = cbs(grid, starts, goals)
    conflict = detect_conflict(paths)
    if conflict:
        print(f"  ⚠️ 残留冲突: Robot {conflict.robot_a} vs {conflict.robot_b} @ t={conflict.time}")
    else:
        print("  ✅ 无冲突")
    for rid, path in paths.items():
        print(f"  Robot {rid}: {len(path)} 步")
    visualize(grid, paths, "多机器人路径规划 (冲突解决)")


if __name__ == '__main__':
    print("=" * 60)
    print("  多机器人路径规划仿真")
    print("=" * 60)
    demo_astar()
    demo_rrt_star()
    demo_prm()
    demo_multi_robot()
    print("\n✅ 所有仿真完成")
