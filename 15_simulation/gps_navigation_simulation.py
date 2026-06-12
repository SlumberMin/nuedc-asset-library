#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GPS导航仿真 - 经纬度→XY坐标转换 + A*路径规划
nuedc-asset-library V3
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import heapq
import math

# ============================================================
# 1. 经纬度 ↔ XY 坐标转换（墨卡托投影近似）
# ============================================================

EARTH_RADIUS = 6371000  # 地球半径(m)

def ll2xy(lat, lon, lat0, lon0):
    """经纬度转局部XY坐标(m)，lat0/lon0为原点"""
    x = EARTH_RADIUS * np.radians(lon - lon0) * np.cos(np.radians(lat0))
    y = EARTH_RADIUS * np.radians(lat - lat0)
    return x, y

def xy2ll(x, y, lat0, lon0):
    """局部XY坐标(m)转经纬度"""
    lat = lat0 + np.degrees(y / EARTH_RADIUS)
    lon = lon0 + np.degrees(x / (EARTH_RADIUS * np.cos(np.radians(lat0))))
    return lat, lon

# ============================================================
# 2. A* 路径规划
# ============================================================

class GridMap:
    def __init__(self, width_m, height_m, resolution=1.0):
        self.res = resolution
        self.w = int(width_m / resolution)
        self.h = int(height_m / resolution)
        self.grid = np.zeros((self.h, self.w), dtype=np.uint8)  # 0=free, 1=obstacle
        self.origin_x = 0.0
        self.origin_y = 0.0

    def add_obstacle(self, cx, cy, radius):
        """在(cx,cy)处添加圆形障碍"""
        for iy in range(self.h):
            for ix in range(self.w):
                px = ix * self.res + self.origin_x
                py = iy * self.res + self.origin_y
                if (px - cx)**2 + (py - cy)**2 <= radius**2:
                    self.grid[iy, ix] = 1

    def add_rect_obstacle(self, x1, y1, x2, y2):
        """矩形障碍"""
        ix1 = max(0, int((x1 - self.origin_x) / self.res))
        iy1 = max(0, int((y1 - self.origin_y) / self.res))
        ix2 = min(self.w, int((x2 - self.origin_x) / self.res))
        iy2 = min(self.h, int((y2 - self.origin_y) / self.res))
        self.grid[iy1:iy2, ix1:ix2] = 1

    def is_free(self, ix, iy):
        if 0 <= ix < self.w and 0 <= iy < self.h:
            return self.grid[iy, ix] == 0
        return False

    def to_world(self, ix, iy):
        return ix * self.res + self.origin_x, iy * self.res + self.origin_y

    def to_grid(self, x, y):
        return int((x - self.origin_x) / self.res), int((y - self.origin_y) / self.res)


def astar(grid_map, start_xy, goal_xy):
    """A*路径规划，返回世界坐标路径列表"""
    sx, sy = grid_map.to_grid(*start_xy)
    gx, gy = grid_map.to_grid(*goal_xy)

    open_set = []
    heapq.heappush(open_set, (0, (sx, sy)))
    came_from = {}
    g_score = {(sx, sy): 0}

    dirs = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]
    costs = [1,1,1,1,1.414,1.414,1.414,1.414]

    while open_set:
        _, current = heapq.heappop(open_set)
        if current == (gx, gy):
            path = []
            while current in came_from:
                path.append(grid_map.to_world(*current))
                current = came_from[current]
            path.append(grid_map.to_world(sx, sy))
            return path[::-1]

        cx, cy = current
        for (dx, dy), cost in zip(dirs, costs):
            nx, ny = cx + dx, cy + dy
            if not grid_map.is_free(nx, ny):
                continue
            ng = g_score[current] + cost
            if ng < g_score.get((nx, ny), float('inf')):
                g_score[(nx, ny)] = ng
                h = math.sqrt((nx - gx)**2 + (ny - gy)**2)
                came_from[(nx, ny)] = current
                heapq.heappush(open_set, (ng + h, (nx, ny)))

    return []  # 无路径

# ============================================================
# 3. GPS误差模拟
# ============================================================

def gps_noise(lat, lon, noise_m=3.0):
    """模拟GPS定位误差"""
    dlat = np.random.normal(0, noise_m / EARTH_RADIUS * 180 / np.pi)
    dlon = np.random.normal(0, noise_m / (EARTH_RADIUS * np.cos(np.radians(lat))) * 180 / np.pi)
    return lat + dlat, lon + dlon

# ============================================================
# 4. 仿真主程序
# ============================================================

def run_gps_simulation():
    np.random.seed(42)

    # 参考点（某大学操场附近）
    lat0, lon0 = 31.2304, 121.4737  # 上海

    # 创建500m×500m地图
    gmap = GridMap(500, 500, resolution=2.0)

    # 添加障碍物
    obstacles = [
        (120, 120, 30), (250, 200, 40), (350, 100, 25),
        (180, 350, 35), (400, 300, 30), (100, 280, 20),
    ]
    for ox, oy, r in obstacles:
        gmap.add_obstacle(ox, oy, r)

    # 起点终点
    start_xy = (20, 20)
    goal_xy = (480, 480)

    # A*路径规划
    path = astar(gmap, start_xy, goal_xy)
    if not path:
        print("未找到路径！")
        return

    # 沿路径移动并加GPS噪声
    noisy_path = []
    for px, py in path:
        lat, lon = xy2ll(px, py, lat0, lon0)
        nlat, nlon = gps_noise(lat, lon, noise_m=5.0)
        nx, ny = ll2xy(nlat, nlon, lat0, lon0)
        noisy_path.append((nx, ny))

    # 可视化
    fig, axes = plt.subplots(1, 2, figsize=(16, 7))

    # 左图：地图+路径
    ax = axes[0]
    ax.imshow(gmap.grid, cmap='gray_r', origin='lower',
              extent=[0, 500, 0, 500], alpha=0.3)
    for ox, oy, r in obstacles:
        ax.add_patch(Circle((ox, oy), r, color='red', alpha=0.3, label='障碍'))
    path_x, path_y = zip(*path)
    ax.plot(path_x, path_y, 'b-', linewidth=2, label='规划路径')
    npx, npy = zip(*noisy_path)
    ax.plot(npx, npy, 'r.', markersize=2, alpha=0.5, label='GPS观测')
    ax.plot(*start_xy, 'go', markersize=10, label='起点')
    ax.plot(*goal_xy, 'r*', markersize=15, label='终点')
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    ax.set_title('GPS导航 - A*路径规划 + GPS噪声')
    ax.legend(fontsize=8); ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)

    # 右图：误差分析
    ax2 = axes[1]
    errors = [math.sqrt((px-nx)**2 + (py-ny)**2) for (px,py),(nx,ny) in zip(path, noisy_path)]
    ax2.plot(errors, 'r-', alpha=0.7)
    ax2.axhline(y=np.mean(errors), color='b', linestyle='--', label=f'均值={np.mean(errors):.1f}m')
    ax2.fill_between(range(len(errors)), 0, errors, alpha=0.2, color='red')
    ax2.set_xlabel('路径点序号'); ax2.set_ylabel('定位误差 (m)')
    ax2.set_title('GPS定位误差分布')
    ax2.legend(); ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('gps_navigation_simulation.png', dpi=150)
    plt.show()

    print("=" * 50)
    print("GPS导航仿真结果")
    print("=" * 50)
    print(f"地图大小: 500×500m, 分辨率: 2.0m")
    print(f"路径点数: {len(path)}")
    print(f"路径总长: {sum(math.sqrt((path[i][0]-path[i-1][0])**2 + (path[i][1]-path[i-1][1])**2) for i in range(1, len(path))):.1f}m")
    print(f"GPS误差  均值: {np.mean(errors):.2f}m, 最大: {np.max(errors):.2f}m")

if __name__ == '__main__':
    run_gps_simulation()
