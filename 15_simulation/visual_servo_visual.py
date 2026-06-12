#!/usr/bin/env python3
"""
视觉伺服可视化仿真 - 带图形界面
=================================
基于图像的视觉伺服(IBVS)仿真，支持:
- 相机模型仿真
- 特征点跟踪与误差计算
- 雅可比矩阵迭代控制
- 实时可视化界面(Tkinter)

适用: 电赛智能车/机械臂视觉伺服控制
"""

import math
import time
import tkinter as tk
from tkinter import ttk
import random

# ============================================================
#  数学工具
# ============================================================

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def mat_mul(A, B):
    """矩阵乘法 A(m×n) * B(n×p) -> C(m×p)"""
    m, n, p = len(A), len(A[0]), len(B[0])
    C = [[0.0]*p for _ in range(m)]
    for i in range(m):
        for j in range(p):
            s = 0.0
            for k in range(n):
                s += A[i][k] * B[k][j]
            C[i][j] = s
    return C

def mat_transpose(A):
    m, n = len(A), len(A[0])
    return [[A[j][i] for j in range(m)] for i in range(n)]

def mat_add(A, B):
    return [[A[i][j]+B[i][j] for j in range(len(A[0]))] for i in range(len(A))]

def mat_scale(A, s):
    return [[A[i][j]*s for j in range(len(A[0]))] for i in range(len(A))]

def mat_pseudo_inv(A, damping=0.01):
    """阻尼最小二乘伪逆: (A^T A + λI)^{-1} A^T"""
    At = mat_transpose(A)
    AtA = mat_mul(At, A)
    n = len(AtA)
    for i in range(n):
        AtA[i][i] += damping
    # 简化: 对角近似求逆
    inv = [[0.0]*n for _ in range(n)]
    for i in range(n):
        inv[i][i] = 1.0 / AtA[i][i] if abs(AtA[i][i]) > 1e-9 else 0.0
    return mat_mul(inv, At)


# ============================================================
#  相机模型
# ============================================================

class PinholeCamera:
    """针孔相机模型"""
    def __init__(self, fx=500, fy=500, cx=320, cy=240, img_w=640, img_h=480):
        self.fx, self.fy = fx, fy
        self.cx, self.cy = cx, cy
        self.img_w, self.img_h = img_w, img_h
        # 相机位姿 (简单2D: x, y, theta)
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        self.v = 0.0       # 线速度
        self.omega = 0.0   # 角速度

    def project(self, world_pts):
        """将世界坐标点投影到图像平面(简化2D投影)"""
        img_pts = []
        for (wx, wy) in world_pts:
            # 相机坐标系变换
            dx = wx - self.x
            dy = wy - self.y
            cx = math.cos(-self.theta) * dx - math.sin(-self.theta) * dy
            cy = math.sin(-self.theta) * dx + math.cos(-self.theta) * dy
            if cx < 0.1:
                cx = 0.1
            # 投影
            u = self.fx * cy / cx + self.cx
            v_img = self.fy * (-0.5) / cx + self.cy  # 简化z=1
            # 用更直观的方式: 直接用距离投影
            u = self.cx + self.fx * math.atan2(dy, dx + 0.01) * 0.5
            v_img = self.cy - 100.0 / (cx + 0.1) + 50
            img_pts.append((u, v_img))
        return img_pts

    def update(self, dt):
        """更新相机位姿"""
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        self.theta += self.omega * dt

    def get_interaction_matrix(self, img_pts, depth=1.0):
        """计算图像雅可比矩阵(交互矩阵) Li for each point"""
        L_list = []
        for (u, v) in img_pts:
            nu = (u - self.cx) / self.fx
            nv = (v - self.cy) / self.fy
            Li = [
                [-1.0/depth, 0, nu/depth, nu*nv, -(1+nu*nu), nv],
                [0, -1.0/depth, nv/depth, 1+nv*nv, -nu*nv, -nu],
            ]
            L_list.append(Li)
        return L_list


# ============================================================
#  IBVS 控制器
# ============================================================

class IBVSController:
    """基于图像的视觉伺服控制器"""
    def __init__(self, camera, desired_pts, gain=0.5, damping=0.01):
        self.camera = camera
        self.desired_pts = desired_pts  # 期望图像特征点
        self.gain = gain
        self.damping = damping
        self.errors = []

    def compute_control(self, current_pts):
        """计算控制量: v_cam = -λ * L⁺ * e"""
        n = len(current_pts)
        if n == 0:
            return 0.0, 0.0

        # 误差 e = s - s*
        errors = []
        for i in range(n):
            ex = current_pts[i][0] - self.desired_pts[i][0]
            ey = current_pts[i][1] - self.desired_pts[i][1]
            errors.append([ex, ey])
        self.errors = errors

        # 展平误差
        e = []
        for er in errors:
            e.extend(er)
        e_mat = [[ev] for ev in e]

        # 交互矩阵 (仅使用 vx, vtheta 对应的列)
        L_full = self.camera.get_interaction_matrix(current_pts, depth=1.5)
        # 提取2列: vx(对应第1列), omega(对应第6列简化)
        L = []
        for Li in L_full:
            L.append([Li[0][0], Li[0][5]])
            L.append([Li[1][0], Li[1][5]])

        # 伪逆
        L_pinv = mat_pseudo_inv(L, self.damping)

        # 控制量
        control = mat_mul(L_pinv, e_mat)
        vx = clamp(-self.gain * control[0][0], -50, 50)
        omega = clamp(-self.gain * control[1][0], -0.5, 0.5)
        return vx, omega


# ============================================================
#  特征点目标
# ============================================================

class FeatureTarget:
    """被跟踪的特征目标(世界坐标)"""
    def __init__(self, points):
        self.points = points  # [(x, y), ...]

    def transform(self, dx=0, dy=0, dtheta=0):
        """目标运动"""
        new_pts = []
        for (x, y) in self.points:
            nx = math.cos(dtheta)*(x) - math.sin(dtheta)*(y) + dx
            ny = math.sin(dtheta)*(x) + math.cos(dtheta)*(y) + dy
            new_pts.append((nx, ny))
        self.points = new_pts


# ============================================================
#  GUI 应用
# ============================================================

class VisualServoApp:
    """视觉伺服可视化仿真 GUI"""

    def __init__(self, root):
        self.root = root
        self.root.title("视觉伺服(IBVS)可视化仿真")
        self.root.geometry("1100x700")
        self.root.configure(bg="#1e1e2e")

        self.running = False
        self.step_count = 0
        self.dt = 0.05
        self.log_lines = []

        self._setup_ui()
        self._init_simulation()

    def _setup_ui(self):
        # 顶部控制栏
        ctrl = tk.Frame(self.root, bg="#2d2d44", pady=5)
        ctrl.pack(fill=tk.X)

        style = {"bg": "#3d3d5c", "fg": "#e0e0e0", "font": ("Consolas", 10), "relief": tk.FLAT}

        self.btn_start = tk.Button(ctrl, text="▶ 启动", command=self.toggle, width=10, **style)
        self.btn_start.pack(side=tk.LEFT, padx=5)

        self.btn_reset = tk.Button(ctrl, text="↺ 重置", command=self.reset, width=10, **style)
        self.btn_reset.pack(side=tk.LEFT, padx=5)

        tk.Label(ctrl, text="增益λ:", bg="#2d2d44", fg="#aaa").pack(side=tk.LEFT, padx=(20,2))
        self.gain_var = tk.DoubleVar(value=0.5)
        tk.Scale(ctrl, from_=0.05, to=2.0, resolution=0.05, orient=tk.HORIZONTAL,
                 variable=self.gain_var, bg="#2d2d44", fg="#aaa", highlightthickness=0,
                 length=120).pack(side=tk.LEFT)

        self.lbl_status = tk.Label(ctrl, text="就绪", bg="#2d2d44", fg="#4fc3f7",
                                   font=("Consolas", 10))
        self.lbl_status.pack(side=tk.RIGHT, padx=10)

        # 主区域: 左=世界视图, 右=图像视图
        main = tk.Frame(self.root, bg="#1e1e2e")
        main.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # 左: 世界坐标视图
        left = tk.LabelFrame(main, text="世界视图 (俯视)", bg="#1e1e2e", fg="#aaa",
                              font=("Consolas", 10))
        left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0,3))
        self.world_canvas = tk.Canvas(left, bg="#0d1117", highlightthickness=0)
        self.world_canvas.pack(fill=tk.BOTH, expand=True)

        # 右: 图像平面视图
        right = tk.LabelFrame(main, text="图像视图 (相机)", bg="#1e1e2e", fg="#aaa",
                               font=("Consolas", 10))
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(3,0))
        self.img_canvas = tk.Canvas(right, bg="#0d1117", highlightthickness=0)
        self.img_canvas.pack(fill=tk.BOTH, expand=True)

        # 底部信息栏
        bottom = tk.Frame(self.root, bg="#2d2d44", pady=5)
        bottom.pack(fill=tk.X)

        self.lbl_info = tk.Label(bottom, text="", bg="#2d2d44", fg="#b0b0b0",
                                  font=("Consolas", 9), anchor=tk.W)
        self.lbl_info.pack(fill=tk.X, padx=10)

        # 右下: 误差曲线
        self.err_canvas = tk.Canvas(bottom, bg="#0d1117", height=80, highlightthickness=0)
        self.err_canvas.pack(fill=tk.X, padx=10, pady=(3,0))
        self.err_history = []

    def _init_simulation(self):
        # 相机初始位置
        self.camera = PinholeCamera()
        self.camera.x, self.camera.y, self.camera.theta = -200, 50, 0.0

        # 目标特征点 (矩形4点)
        self.target = FeatureTarget([
            (200, 100), (280, 100), (280, 170), (200, 170)
        ])

        # 期望图像点 (画面中心正方形)
        cx, cy = 320, 240
        self.desired_img_pts = [
            (cx-40, cy-40), (cx+40, cy-40), (cx+40, cy+40), (cx-40, cy+40)
        ]

        # IBVS控制器
        self.controller = IBVSController(self.camera, self.desired_img_pts, gain=0.5)
        self.err_history = []
        self.step_count = 0
        self._draw()

    def toggle(self):
        self.running = not self.running
        self.btn_start.config(text="⏸ 暂停" if self.running else "▶ 启动")
        self.lbl_status.config(text="运行中" if self.running else "已暂停")
        if self.running:
            self._tick()

    def reset(self):
        self.running = False
        self.btn_start.config(text="▶ 启动")
        self.lbl_status.config(text="已重置")
        self._init_simulation()

    def _tick(self):
        if not self.running:
            return
        self._step()
        self._draw()
        self.step_count += 1
        self.root.after(int(self.dt * 1000), self._tick)

    def _step(self):
        # 投影当前特征点
        current_img_pts = self.camera.project(self.target.points)

        # IBVS控制
        vx, omega = self.controller.compute_control(current_img_pts)
        self.camera.v = vx
        self.camera.omega = omega
        self.controller.gain = self.gain_var.get()

        # 更新相机位姿
        self.camera.update(self.dt)

        # 目标缓慢移动(模拟动态目标)
        if self.step_count % 200 == 0 and self.step_count > 0:
            self.target.transform(dx=random.uniform(-5, 5), dy=random.uniform(-5, 5))

        # 记录总误差
        total_err = 0
        for er in self.controller.errors:
            total_err += math.sqrt(er[0]**2 + er[1]**2)
        self.err_history.append(total_err)
        if len(self.err_history) > 300:
            self.err_history.pop(0)

    def _draw(self):
        self._draw_world()
        self._draw_image()
        self._draw_error()
        # 更新信息
        err_sum = self.err_history[-1] if self.err_history else 0
        self.lbl_info.config(
            text=f"步数: {self.step_count}  |  相机: ({self.camera.x:.1f}, {self.camera.y:.1f}, θ={math.degrees(self.camera.theta):.1f}°)  |  "
                 f"v={self.camera.v:.2f}  ω={self.camera.omega:.3f}  |  误差: {err_sum:.2f}"
        )

    def _draw_world(self):
        c = self.world_canvas
        c.delete("all")
        w = c.winfo_width() or 500
        h = c.winfo_height() or 400
        ox, oy = w//2, h//2
        scale = 0.8

        # 网格
        for gx in range(-400, 401, 80):
            c.create_line(ox+gx*scale, 0, ox+gx*scale, h, fill="#1a2332", width=1)
        for gy in range(-300, 301, 80):
            c.create_line(0, oy-gy*scale, w, oy-gy*scale, fill="#1a2332", width=1)

        # 坐标轴
        c.create_line(ox, 0, ox, h, fill="#334455", width=1)
        c.create_line(0, oy, w, oy, fill="#334455", width=1)

        # 目标特征点
        pts = self.target.points
        for i, (px, py) in enumerate(pts):
            sx, sy = ox + px*scale, oy - py*scale
            c.create_oval(sx-6, sy-6, sx+6, sy+6, fill="#ff6b6b", outline="#ff8888")
            c.create_text(sx+10, sy-10, text=f"P{i}", fill="#ff8888", font=("Consolas", 8))
        # 连线
        for i in range(len(pts)):
            j = (i+1) % len(pts)
            x1, y1 = ox+pts[i][0]*scale, oy-pts[i][1]*scale
            x2, y2 = ox+pts[j][0]*scale, oy-pts[j][1]*scale
            c.create_line(x1, y1, x2, y2, fill="#ff6b6b", width=2, dash=(4,2))

        # 相机
        cx_s = ox + self.camera.x * scale
        cy_s = oy - self.camera.y * scale
        theta = self.camera.theta
        # 相机三角形
        size = 20
        pts_cam = [
            (cx_s + size*math.cos(theta), cy_s - size*math.sin(theta)),
            (cx_s + size*0.6*math.cos(theta+2.5), cy_s - size*0.6*math.sin(theta+2.5)),
            (cx_s + size*0.6*math.cos(theta-2.5), cy_s - size*0.6*math.sin(theta-2.5)),
        ]
        c.create_polygon(pts_cam, fill="#4fc3f7", outline="#81d4fa", width=2)
        # 视野线
        fov = 0.6
        for angle in [theta-fov, theta+fov]:
            ex = cx_s + 150*math.cos(angle)
            ey = cy_s - 150*math.sin(angle)
            c.create_line(cx_s, cy_s, ex, ey, fill="#4fc3f7", width=1, dash=(3,3))

    def _draw_image(self):
        c = self.img_canvas
        c.delete("all")
        w = c.winfo_width() or 500
        h = c.winfo_height() or 400

        # 图像坐标系原点在左上
        # 缩放因子
        sx = w / 640
        sy = h / 480

        # 期望特征点(绿色)
        for i, (u, v) in enumerate(self.desired_img_pts):
            x, y = u*sx, v*sy
            c.create_rectangle(x-8, y-8, x+8, y+8, outline="#4caf50", width=2)
            c.create_text(x, y-15, text=f"D{i}", fill="#4caf50", font=("Consolas", 8))
        for i in range(4):
            j = (i+1) % 4
            c.create_line(self.desired_img_pts[i][0]*sx, self.desired_img_pts[i][1]*sy,
                          self.desired_img_pts[j][0]*sx, self.desired_img_pts[j][1]*sy,
                          fill="#4caf50", width=1, dash=(4,2))

        # 当前特征点(红色)
        current = self.camera.project(self.target.points)
        for i, (u, v) in enumerate(current):
            x, y = u*sx, v*sy
            c.create_oval(x-6, y-6, x+6, y+6, fill="#ff6b6b", outline="#ff8888")
            c.create_text(x, y+15, text=f"C{i}", fill="#ff8888", font=("Consolas", 8))
        for i in range(len(current)):
            j = (i+1) % len(current)
            c.create_line(current[i][0]*sx, current[i][1]*sy,
                          current[j][0]*sx, current[j][1]*sy,
                          fill="#ff6b6b", width=2)
            # 误差矢量
            dx = self.desired_img_pts[i][0]*sx - current[i][0]*sx
            dy = self.desired_img_pts[i][1]*sy - current[i][1]*sy
            c.create_line(current[i][0]*sx, current[i][1]*sy,
                          current[i][0]*sx+dx, current[i][1]*sy+dy,
                          fill="#ffb74d", width=2, arrow=tk.LAST)

        c.create_text(w//2, 15, text="● 期望  ● 当前  → 误差", fill="#888",
                       font=("Consolas", 9))

    def _draw_error(self):
        c = self.err_canvas
        c.delete("all")
        w = c.winfo_width() or 400
        h = c.winfo_height() or 80
        if not self.err_history:
            return

        max_e = max(max(self.err_history), 1)
        n = len(self.err_history)
        step = max(1, w // 300)
        points = []
        for i in range(0, n, step):
            x = i * w / 300
            y = h - (self.err_history[i] / max_e) * (h - 10) - 5
            points.append((x, y))

        for i in range(len(points)-1):
            c.create_line(points[i][0], points[i][1],
                          points[i+1][0], points[i+1][1],
                          fill="#ffb74d", width=2)

        c.create_text(w-5, 5, text=f"误差: {self.err_history[-1]:.1f}  max:{max_e:.1f}",
                       anchor=tk.NE, fill="#888", font=("Consolas", 8))


# ============================================================
#  主入口
# ============================================================

if __name__ == "__main__":
    root = tk.Tk()
    app = VisualServoApp(root)
    root.mainloop()
