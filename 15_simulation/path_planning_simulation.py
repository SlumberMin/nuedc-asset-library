#!/usr/bin/env python3
"""
路径规划仿真 - A* / RRT / Dijkstra
=====================================
支持算法:
  1. A* (网格地图)
  2. Dijkstra (网格地图)
  3. RRT (连续空间)

交互:
  - 左键设置起点/终点
  - 右键拖拽画障碍物
  - 选择算法后点击运行

适用: 电赛智能车路径规划
"""

import math
import heapq
import random
import tkinter as tk
from collections import defaultdict

# ============================================================
#  地图
# ============================================================

class GridMap:
    """网格地图"""
    def __init__(self, cols=50, rows=35, cell_size=16):
        self.cols = cols
        self.rows = rows
        self.cell_size = cell_size
        self.obstacles = set()
        self.start = (2, 2)
        self.goal = (cols-3, rows-3)

    def is_valid(self, x, y):
        return 0 <= x < self.cols and 0 <= y < self.rows

    def is_free(self, x, y):
        return self.is_valid(x, y) and (x, y) not in self.obstacles

    def neighbors(self, x, y, allow_diag=True):
        dirs = [(0,1),(0,-1),(1,0),(-1,0)]
        if allow_diag:
            dirs += [(1,1),(1,-1),(-1,1),(-1,-1)]
        result = []
        for dx, dy in dirs:
            nx, ny = x+dx, y+dy
            if self.is_free(nx, ny):
                # 对角线检查是否穿越障碍
                if abs(dx)+abs(dy) == 2:
                    if not (self.is_free(x+dx, y) and self.is_free(x, y+dy)):
                        continue
                cost = math.sqrt(dx*dx+dy*dy)
                result.append((nx, ny, cost))
        return result

    def random_free(self):
        while True:
            x = random.randint(0, self.cols-1)
            y = random.randint(0, self.rows-1)
            if self.is_free(x, y):
                return (x, y)

    def generate_obstacles(self, density=0.2):
        """随机生成障碍"""
        self.obstacles.clear()
        for x in range(self.cols):
            for y in range(self.rows):
                if random.random() < density:
                    if (x, y) != self.start and (x, y) != self.goal:
                        self.obstacles.add((x, y))

    def add_wall(self, x1, y1, x2, y2):
        """添加墙壁"""
        dx = 0 if x1==x2 else (1 if x2>x1 else -1)
        dy = 0 if y1==y2 else (1 if y2>y1 else -1)
        x, y = x1, y1
        while True:
            if self.is_valid(x, y) and (x,y) != self.start and (x,y) != self.goal:
                self.obstacles.add((x, y))
            if x == x2 and y == y2:
                break
            x += dx
            y += dy


# ============================================================
#  A* 算法
# ============================================================

def astar(grid, start, goal, callback=None):
    """A*搜索, 返回路径列表. callback(node)用于可视化"""
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = defaultdict(lambda: float('inf'))
    g_score[start] = 0
    explored = []

    while open_set:
        _, current = heapq.heappop(open_set)
        explored.append(current)

        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return path[::-1], explored

        for nx, ny, cost in grid.neighbors(*current):
            tentative = g_score[current] + cost
            if tentative < g_score[(nx, ny)]:
                came_from[(nx, ny)] = current
                g_score[(nx, ny)] = tentative
                h = math.sqrt((nx-goal[0])**2 + (ny-goal[1])**2)
                heapq.heappush(open_set, (tentative + h, (nx, ny)))

    return None, explored


# ============================================================
#  Dijkstra 算法
# ============================================================

def dijkstra(grid, start, goal, callback=None):
    """Dijkstra搜索"""
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    dist = defaultdict(lambda: float('inf'))
    dist[start] = 0
    explored = []

    while open_set:
        d, current = heapq.heappop(open_set)
        if d > dist[current]:
            continue
        explored.append(current)

        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return path[::-1], explored

        for nx, ny, cost in grid.neighbors(*current):
            new_dist = dist[current] + cost
            if new_dist < dist[(nx, ny)]:
                dist[(nx, ny)] = new_dist
                came_from[(nx, ny)] = current
                heapq.heappush(open_set, (new_dist, (nx, ny)))

    return None, explored


# ============================================================
#  RRT 算法
# ============================================================

def rrt(grid, start, goal, max_iter=3000, step_size=3, goal_bias=0.1):
    """RRT (快速随机树)"""
    # 转换为连续坐标
    tree = {start: None}
    nodes = [start]

    for _ in range(max_iter):
        # 目标偏向采样
        if random.random() < goal_bias:
            target = goal
        else:
            target = (random.randint(0, grid.cols-1), random.randint(0, grid.rows-1))

        # 找最近节点
        nearest = min(nodes, key=lambda n: math.sqrt((n[0]-target[0])**2 + (n[1]-target[1])**2))

        # 向目标扩展
        dx = target[0] - nearest[0]
        dy = target[1] - nearest[1]
        dist = math.sqrt(dx*dx + dy*dy)
        if dist < 0.01:
            continue
        nx = nearest[0] + step_size * dx / dist
        ny = nearest[1] + step_size * dy / dist

        # 取整并检查
        gx, gy = int(round(nx)), int(round(ny))
        if not grid.is_free(gx, gy):
            continue

        # 碰撞检查(线段)
        if not _line_free(grid, nearest, (gx, gy)):
            continue

        new_node = (gx, gy)
        tree[new_node] = nearest
        nodes.append(new_node)

        # 到达目标
        if abs(gx - goal[0]) <= 1 and abs(gy - goal[1]) <= 1:
            tree[goal] = new_node
            path = [goal]
            current = goal
            while tree[current] is not None:
                current = tree[current]
                path.append(current)
            return path[::-1], nodes

    return None, nodes


def _line_free(grid, p1, p2):
    """检查两点间线段是否无碰撞"""
    x1, y1 = p1
    x2, y2 = p2
    steps = int(max(abs(x2-x1), abs(y2-y1))) + 1
    for i in range(steps+1):
        t = i / max(steps, 1)
        x = int(round(x1 + (x2-x1)*t))
        y = int(round(y1 + (y2-y1)*t))
        if not grid.is_free(x, y):
            return False
    return True


# ============================================================
#  GUI
# ============================================================

class PathPlanningApp:
    def __init__(self, root):
        self.root = root
        self.root.title("路径规划仿真 - A*/Dijkstra/RRT")
        self.root.geometry("1000x700")
        self.root.configure(bg="#1e1e2e")

        self.grid = GridMap(cols=60, rows=40, cell_size=14)
        self.grid.generate_obstacles(0.2)

        self.path = None
        self.explored = []
        self.algorithm = "astar"
        self.click_mode = "start"  # start / goal
        self.dragging = False

        self._setup_ui()
        self._draw()

    def _setup_ui(self):
        ctrl = tk.Frame(self.root, bg="#2d2d44", pady=5)
        ctrl.pack(fill=tk.X)
        s = {"bg": "#3d3d5c", "fg": "#e0e0e0", "font": ("Consolas", 10), "relief": tk.FLAT}

        tk.Label(ctrl, text="算法:", bg="#2d2d44", fg="#aaa").pack(side=tk.LEFT, padx=5)
        self.alg_var = tk.StringVar(value="astar")
        for name, label in [("astar","A*"), ("dijkstra","Dijkstra"), ("rrt","RRT")]:
            tk.Radiobutton(ctrl, text=label, variable=self.alg_var, value=name,
                          bg="#2d2d44", fg="#aaa", selectcolor="#3d3d5c",
                          activebackground="#2d2d44").pack(side=tk.LEFT, padx=5)

        tk.Button(ctrl, text="▶ 运行", command=self.run_algo, **s).pack(side=tk.LEFT, padx=10)
        tk.Button(ctrl, text="随机地图", command=self.random_map, **s).pack(side=tk.LEFT, padx=5)
        tk.Button(ctrl, text="清空障碍", command=self.clear_obstacles, **s).pack(side=tk.LEFT, padx=5)
        tk.Button(ctrl, text="清空路径", command=self.clear_path, **s).pack(side=tk.LEFT, padx=5)

        self.lbl_info = tk.Label(ctrl, text="", bg="#2d2d44", fg="#4fc3f7", font=("Consolas", 10))
        self.lbl_info.pack(side=tk.RIGHT, padx=10)

        self.canvas = tk.Canvas(self.root, bg="#0d1117", highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        self.canvas.bind("<Button-1>", self._on_left_click)
        self.canvas.bind("<B3-Motion>", self._on_right_drag)
        self.canvas.bind("<Button-3>", self._on_right_click)
        self.canvas.bind("<ButtonRelease-3>", lambda e: setattr(self, 'dragging', False))

        tip = tk.Label(self.root, text="左键:设起点(再左键设终点) | 右键拖拽:画障碍 | 算法选择后点运行",
                       bg="#2d2d44", fg="#888", font=("Consolas", 9))
        tip.pack(fill=tk.X)

    def _screen_to_grid(self, sx, sy):
        g = self.grid
        cx = sx // g.cell_size
        cy = sy // g.cell_size
        return int(cx), int(cy)

    def _on_left_click(self, event):
        gx, gy = self._screen_to_grid(event.x, event.y)
        if self.grid.is_valid(gx, gy):
            if self.click_mode == "start":
                self.grid.start = (gx, gy)
                self.click_mode = "goal"
            else:
                self.grid.goal = (gx, gy)
                self.click_mode = "start"
            self.clear_path()
            self._draw()

    def _on_right_click(self, event):
        self.dragging = True
        self._toggle_obstacle(event)

    def _on_right_drag(self, event):
        self._toggle_obstacle(event)

    def _toggle_obstacle(self, event):
        gx, gy = self._screen_to_grid(event.x, event.y)
        if self.grid.is_valid(gx, gy) and (gx,gy) != self.grid.start and (gx,gy) != self.grid.goal:
            if (gx, gy) in self.grid.obstacles:
                self.grid.obstacles.discard((gx, gy))
            else:
                self.grid.obstacles.add((gx, gy))
            self._draw()

    def run_algo(self):
        alg = self.alg_var.get()
        start_time = __import__('time').time()
        if alg == "astar":
            self.path, self.explored = astar(self.grid, self.grid.start, self.grid.goal)
        elif alg == "dijkstra":
            self.path, self.explored = dijkstra(self.grid, self.grid.start, self.grid.goal)
        else:
            self.path, self.explored = rrt(self.grid, self.grid.start, self.grid.goal)
        elapsed = __import__('time').time() - start_time

        path_len = 0
        if self.path:
            for i in range(len(self.path)-1):
                dx = self.path[i+1][0]-self.path[i][0]
                dy = self.path[i+1][1]-self.path[i][1]
                path_len += math.sqrt(dx*dx+dy*dy)

        self.lbl_info.config(text=f"{alg.upper()} | 探索:{len(self.explored)}节点 | "
                                   f"路径长:{path_len:.1f} | 耗时:{elapsed*1000:.1f}ms")
        self._draw()

    def random_map(self):
        self.grid.generate_obstacles(0.2)
        self.clear_path()
        self._draw()

    def clear_obstacles(self):
        self.grid.obstacles.clear()
        self.clear_path()
        self._draw()

    def clear_path(self):
        self.path = None
        self.explored = []
        self._draw()

    def _draw(self):
        c = self.canvas
        c.delete("all")
        g = self.grid
        cs = g.cell_size

        # 障碍
        for (x, y) in g.obstacles:
            c.create_rectangle(x*cs, y*cs, (x+1)*cs, (y+1)*cs, fill="#3d3d5c", outline="#2d2d44")

        # 探索节点
        for (x, y) in self.explored:
            c.create_rectangle(x*cs+2, y*cs+2, (x+1)*cs-2, (y+1)*cs-2,
                             fill="#1a3a2a", outline="")

        # 路径
        if self.path and len(self.path) > 1:
            pts = []
            for (x, y) in self.path:
                pts.extend([(x+0.5)*cs, (y+0.5)*cs])
            c.create_line(pts, fill="#ffb74d", width=3, smooth=True)
            # 路径点
            for (x, y) in self.path:
                cx_s = (x+0.5)*cs
                cy_s = (y+0.5)*cs
                c.create_oval(cx_s-2, cy_s-2, cx_s+2, cy_s+2, fill="#ffb74d", outline="")

        # 起点/终点
        sx, sy = g.start
        gx, gy = g.goal
        c.create_rectangle(sx*cs, sy*cs, (sx+1)*cs, (sy+1)*cs, fill="#4caf50", outline="white")
        c.create_text((sx+0.5)*cs, (sy+0.5)*cs, text="S", fill="white", font=("Consolas", 8, "bold"))
        c.create_rectangle(gx*cs, gy*cs, (gx+1)*cs, (gy+1)*cs, fill="#f44336", outline="white")
        c.create_text((gx+0.5)*cs, (gy+0.5)*cs, text="G", fill="white", font=("Consolas", 8, "bold"))

        # 网格线(轻)
        for x in range(g.cols+1):
            c.create_line(x*cs, 0, x*cs, g.rows*cs, fill="#1a2332", width=1)
        for y in range(g.rows+1):
            c.create_line(0, y*cs, g.cols*cs, y*cs, fill="#1a2332", width=1)


# ============================================================
#  主入口
# ============================================================

if __name__ == "__main__":
    root = tk.Tk()
    app = PathPlanningApp(root)
    root.mainloop()
