#!/usr/bin/env python3
"""
路径规划器 - Orange Pi 5
A* / Dijkstra / RRT 算法实现
"""

import numpy as np
import heapq
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple, Set, Dict
from enum import Enum, auto
import math


class PlannerType(Enum):
    ASTAR = auto()
    DIJKSTRA = auto()
    RRT = auto()
    RRT_STAR = auto()


@dataclass
class GridMap:
    """栅格地图"""
    width: int
    height: int
    resolution: float = 0.05    # 每格代表的米数
    origin: Tuple[float, float] = (0.0, 0.0)
    data: Optional[np.ndarray] = None  # 0=free, 1=occupied, 255=unknown

    def __post_init__(self):
        if self.data is None:
            self.data = np.zeros((self.height, self.width), dtype=np.uint8)

    def set_obstacle(self, x: float, y: float, radius: float = 0.1):
        """设置障碍物"""
        gx, gy = self.world_to_grid(x, y)
        cells = int(radius / self.resolution)
        for dx in range(-cells, cells+1):
            for dy in range(-cells, cells+1):
                if dx**2 + dy**2 <= cells**2:
                    nx, ny = gx + dx, gy + dy
                    if 0 <= nx < self.width and 0 <= ny < self.height:
                        self.data[ny, nx] = 1

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        gx = int((x - self.origin[0]) / self.resolution)
        gy = int((y - self.origin[1]) / self.resolution)
        return (np.clip(gx, 0, self.width-1), np.clip(gy, 0, self.height-1))

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        x = gx * self.resolution + self.origin[0]
        y = gy * self.resolution + self.origin[1]
        return (x, y)

    def is_free(self, gx: int, gy: int) -> bool:
        if 0 <= gx < self.width and 0 <= gy < self.height:
            return self.data[gy, gx] == 0
        return False


@dataclass(order=True)
class Node:
    cost: float
    x: int = field(compare=False)
    y: int = field(compare=False)
    parent: Optional['Node'] = field(default=None, compare=False)


def heuristic(a: Tuple[int, int], b: Tuple[int, int]) -> float:
    """欧几里得距离启发式"""
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)


def astar(grid: GridMap, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
    """A*算法"""
    open_set = []
    heapq.heappush(open_set, Node(0, start[0], start[1]))
    came_from: Dict[Tuple[int,int], Tuple[int,int]] = {}
    g_score = {start: 0}

    neighbors = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]

    while open_set:
        current = heapq.heappop(open_set)
        cx, cy = current.x, current.y

        if (cx, cy) == goal:
            path = []
            pos = goal
            while pos in came_from:
                path.append(pos)
                pos = came_from[pos]
            path.append(start)
            return path[::-1]

        for dx, dy in neighbors:
            nx, ny = cx + dx, cy + dy
            if not grid.is_free(nx, ny):
                continue
            cost = math.sqrt(dx*dx + dy*dy)
            tentative = g_score[(cx, cy)] + cost
            if (nx, ny) not in g_score or tentative < g_score[(nx, ny)]:
                g_score[(nx, ny)] = tentative
                f = tentative + heuristic((nx, ny), goal)
                heapq.heappush(open_set, Node(f, nx, ny))
                came_from[(nx, ny)] = (cx, cy)

    return None


def dijkstra(grid: GridMap, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
    """Dijkstra算法"""
    open_set = []
    heapq.heappush(open_set, Node(0, start[0], start[1]))
    came_from: Dict[Tuple[int,int], Tuple[int,int]] = {}
    g_score = {start: 0}

    neighbors = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]

    while open_set:
        current = heapq.heappop(open_set)
        cx, cy = current.x, current.y

        if (cx, cy) == goal:
            path = []
            pos = goal
            while pos in came_from:
                path.append(pos)
                pos = came_from[pos]
            path.append(start)
            return path[::-1]

        for dx, dy in neighbors:
            nx, ny = cx + dx, cy + dy
            if not grid.is_free(nx, ny):
                continue
            cost = math.sqrt(dx*dx + dy*dy)
            tentative = g_score[(cx, cy)] + cost
            if (nx, ny) not in g_score or tentative < g_score[(nx, ny)]:
                g_score[(nx, ny)] = tentative
                heapq.heappush(open_set, Node(tentative, nx, ny))
                came_from[(nx, ny)] = (cx, cy)

    return None


@dataclass
class RRTNode:
    x: float
    y: float
    parent: Optional[int] = None


def rrt(grid: GridMap, start: Tuple[float, float], goal: Tuple[float, float],
        max_iter: int = 5000, step_size: float = 0.1, goal_threshold: float = 0.1,
        goal_bias: float = 0.1) -> Optional[List[Tuple[float, float]]]:
    """RRT算法"""
    nodes = [RRTNode(start[0], start[1])]

    for _ in range(max_iter):
        # 采样 (带目标偏向)
        if np.random.random() < goal_bias:
            rand_x, rand_y = goal
        else:
            rand_x = np.random.uniform(0, grid.width * grid.resolution)
            rand_y = np.random.uniform(0, grid.height * grid.resolution)

        # 找最近节点
        nearest_idx = min(range(len(nodes)),
                         key=lambda i: (nodes[i].x-rand_x)**2 + (nodes[i].y-rand_y)**2)
        nearest = nodes[nearest_idx]

        # 扩展
        dx = rand_x - nearest.x
        dy = rand_y - nearest.y
        dist = math.sqrt(dx*dx + dy*dy)
        if dist < 1e-6:
            continue
        new_x = nearest.x + step_size * dx / dist
        new_y = nearest.y + step_size * dy / dist

        # 碰撞检测
        gx, gy = grid.world_to_grid(new_x, new_y)
        if not grid.is_free(gx, gy):
            continue

        new_node = RRTNode(new_x, new_y, nearest_idx)
        nodes.append(new_node)

        # 到达目标?
        if math.sqrt((new_x-goal[0])**2 + (new_y-goal[1])**2) < goal_threshold:
            path = []
            idx = len(nodes) - 1
            while idx is not None:
                path.append((nodes[idx].x, nodes[idx].y))
                idx = nodes[idx].parent
            return path[::-1]

    return None


def rrt_star(grid: GridMap, start: Tuple[float, float], goal: Tuple[float, float],
             max_iter: int = 5000, step_size: float = 0.1, goal_threshold: float = 0.1,
             search_radius: float = 0.3) -> Optional[List[Tuple[float, float]]]:
    """RRT*算法 (带路径优化)"""
    nodes = [RRTNode(start[0], start[1])]
    costs = [0.0]

    for _ in range(max_iter):
        rand_x = np.random.uniform(0, grid.width * grid.resolution)
        rand_y = np.random.uniform(0, grid.height * grid.resolution)
        if np.random.random() < 0.1:
            rand_x, rand_y = goal

        nearest_idx = min(range(len(nodes)),
                         key=lambda i: (nodes[i].x-rand_x)**2 + (nodes[i].y-rand_y)**2)
        nearest = nodes[nearest_idx]

        dx = rand_x - nearest.x
        dy = rand_y - nearest.y
        dist = math.sqrt(dx*dx + dy*dy)
        if dist < 1e-6:
            continue
        new_x = nearest.x + step_size * dx / dist
        new_y = nearest.y + step_size * dy / dist

        gx, gy = grid.world_to_grid(new_x, new_y)
        if not grid.is_free(gx, gy):
            continue

        # 找搜索半径内最优父节点
        best_parent = nearest_idx
        best_cost = costs[nearest_idx] + dist
        for i, node in enumerate(nodes):
            d = math.sqrt((node.x-new_x)**2 + (node.y-new_y)**2)
            if d < search_radius and costs[i] + d < best_cost:
                best_cost = costs[i] + d
                best_parent = i

        new_node = RRTNode(new_x, new_y, best_parent)
        nodes.append(new_node)
        costs.append(best_cost)

        if math.sqrt((new_x-goal[0])**2 + (new_y-goal[1])**2) < goal_threshold:
            path = []
            idx = len(nodes) - 1
            while idx is not None:
                path.append((nodes[idx].x, nodes[idx].y))
                idx = nodes[idx].parent
            return path[::-1]

    return None


class PathPlanner:
    """
    路径规划器
    - 支持A*、Dijkstra、RRT、RRT*
    - 路径平滑
    """

    def __init__(self, grid: GridMap, planner_type: PlannerType = PlannerType.ASTAR):
        self.grid = grid
        self.planner_type = planner_type

    def plan(self, start: Tuple[float, float], goal: Tuple[float, float]) -> Optional[List[Tuple[float, float]]]:
        """规划路径，返回世界坐标路径点列表"""
        t0 = time.time()

        if self.planner_type in (PlannerType.ASTAR, PlannerType.DIKSTRA):
            sg = self.grid.world_to_grid(*start)
            gg = self.grid.world_to_grid(*goal)
            if self.planner_type == PlannerType.ASTAR:
                grid_path = astar(self.grid, sg, gg)
            else:
                grid_path = dijkstra(self.grid, sg, gg)
            if grid_path is None:
                return None
            path = [self.grid.grid_to_world(gx, gy) for gx, gy in grid_path]
        elif self.planner_type == PlannerType.RRT:
            path = rrt(self.grid, start, goal)
        elif self.planner_type == PlannerType.RRT_STAR:
            path = rrt_star(self.grid, start, goal)
        else:
            return None

        elapsed = time.time() - t0
        if path:
            path = self._smooth_path(path)
        return path

    def plan_with_timing(self, start, goal):
        """规划路径并返回 (路径, 耗时秒数)。"""
        t0 = time.time()
        path = self.plan(start, goal)
        elapsed = time.time() - t0
        return path, elapsed

    def _smooth_path(self, path: List[Tuple[float, float]], iterations: int = 50,
                     weight: float = 0.5, tolerance: float = 1e-4) -> List[Tuple[float, float]]:
        """路径平滑 (梯度下降)"""
        if len(path) <= 2:
            return path

        smooth = [list(p) for p in path]
        for _ in range(iterations):
            max_change = 0
            for i in range(1, len(smooth) - 1):
                for j in range(2):  # x, y
                    old = smooth[i][j]
                    smooth[i][j] += weight * (path[i][j] - smooth[i][j])
                    smooth[i][j] += weight * (smooth[i-1][j] + smooth[i+1][j] - 2*smooth[i][j])
                    max_change = max(max_change, abs(smooth[i][j] - old))
            if max_change < tolerance:
                break

        return [(p[0], p[1]) for p in smooth]

    def path_length(self, path: List[Tuple[float, float]]) -> float:
        """计算路径长度"""
        length = 0.0
        for i in range(1, len(path)):
            dx = path[i][0] - path[i-1][0]
            dy = path[i][1] - path[i-1][1]
            length += math.sqrt(dx*dx + dy*dy)
        return length


if __name__ == '__main__':
    # 示例：创建地图并规划
    grid = GridMap(width=100, height=100, resolution=0.05)

    # 添加障碍物
    grid.set_obstacle(1.5, 1.0, 0.3)
    grid.set_obstacle(2.0, 2.0, 0.4)
    grid.set_obstacle(1.0, 2.5, 0.2)

    planner = PathPlanner(grid, PlannerType.ASTAR)
    path = planner.plan((0.5, 0.5), (3.0, 3.5))

    if path:
        print(f"Path found: {len(path)} waypoints, length: {planner.path_length(path):.2f}m")
        for p in path[:5]:
            print(f"  ({p[0]:.2f}, {p[1]:.2f})")
    else:
        print("No path found")
