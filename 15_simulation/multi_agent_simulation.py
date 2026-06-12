#!/usr/bin/env python3
"""
多智能体仿真 - 双车跟随 / 编队控制
=====================================
仿真模式:
  1. 跟随模式: 后车保持与前车固定距离
  2. 编队模式: 多车保持V字/一字编队
  3. 切换领航者

适用: 电赛多车协作、编队控制场景
"""

import math
import random
import time
import tkinter as tk

# ============================================================
#  车辆模型
# ============================================================

class Vehicle:
    """差速驱动车辆模型"""
    _id_counter = 0

    def __init__(self, x, y, theta=0, color="#4fc3f7", name=None):
        Vehicle._id_counter += 1
        self.id = Vehicle._id_counter
        self.name = name or f"车{self.id}"
        self.x, self.y, self.theta = float(x), float(y), float(theta)
        self.v = 0.0
        self.omega = 0.0
        self.color = color
        self.trail = []
        self.max_v = 80.0
        self.max_omega = 1.5

        # 控制参数
        self.target_v = 0.0
        self.target_omega = 0.0

    def set_control(self, v, omega):
        self.target_v = max(-self.max_v, min(self.max_v, v))
        self.target_omega = max(-self.max_omega, min(self.max_omega, omega))

    def update(self, dt):
        # 平滑过渡
        self.v += (self.target_v - self.v) * 0.3
        self.omega += (self.target_omega - self.omega) * 0.3
        # 运动学更新
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        self.theta += self.omega * dt
        # 记录轨迹
        self.trail.append((self.x, self.y))
        if len(self.trail) > 500:
            self.trail.pop(0)

    def distance_to(self, other):
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)

    def angle_to(self, other):
        return math.atan2(other.y - self.y, other.x - self.x)


# ============================================================
#  控制策略
# ============================================================

def follow_controller(follower, leader, desired_dist=60, kp_v=1.5, kp_w=2.0):
    """跟随控制器: follower保持与leader距离为desired_dist"""
    dist = follower.distance_to(leader)
    angle_to_leader = follower.angle_to_leader(leader)

    # 距离误差
    dist_err = dist - desired_dist

    # 航向误差
    heading_err = angle_to_leader - follower.theta
    while heading_err > math.pi:
        heading_err -= 2 * math.pi
    while heading_err < -math.pi:
        heading_err += 2 * math.pi

    # 前向速度: 正比于距离误差
    v = kp_v * dist_err
    # 转向速度: 正比于航向误差
    omega = kp_w * heading_err

    return v, omega


def formation_controller(vehicle, vehicles, formation_offsets, leader_idx=0):
    """编队控制器: 基于leader的相对偏移"""
    leader = vehicles[leader_idx]
    idx = vehicles.index(vehicle)
    if idx == leader_idx:
        # 领航者直线前进
        return 40, 0

    dx, dy = formation_offsets[idx]
    # 目标位置: leader坐标系下偏移
    target_x = leader.x + dx * math.cos(leader.theta) - dy * math.sin(leader.theta)
    target_y = leader.y + dx * math.sin(leader.theta) + dy * math.cos(leader.theta)

    # 到目标的距离和角度
    dist = math.sqrt((vehicle.x - target_x)**2 + (vehicle.y - target_y)**2)
    angle = math.atan2(target_y - vehicle.y, target_x - vehicle.x)
    heading_err = angle - vehicle.theta
    while heading_err > math.pi:
        heading_err -= 2 * math.pi
    while heading_err < -math.pi:
        heading_err += 2 * math.pi

    v = min(1.5 * dist, 80)
    omega = 2.5 * heading_err
    return v, omega


# 给Vehicle添加angle_to方法
def _angle_to(self, other):
    return math.atan2(other.y - self.y, other.x - self.x)
Vehicle.angle_to_leader = _angle_to


# ============================================================
#  GUI
# ============================================================

class MultiAgentApp:
    def __init__(self, root):
        self.root = root
        self.root.title("多智能体仿真 - 跟随/编队")
        self.root.geometry("1000x700")
        self.root.configure(bg="#1e1e2e")

        self.running = False
        self.dt = 0.05
        self.mode = "follow"  # follow / formation
        self.step_count = 0
        self.leader_idx = 0

        self._setup_ui()
        self._init_vehicles()

    def _setup_ui(self):
        # 控制栏
        ctrl = tk.Frame(self.root, bg="#2d2d44", pady=5)
        ctrl.pack(fill=tk.X)

        style = {"bg": "#3d3d5c", "fg": "#e0e0e0", "font": ("Consolas", 10), "relief": tk.FLAT}

        self.btn_run = tk.Button(ctrl, text="▶ 启动", command=self.toggle, width=10, **style)
        self.btn_run.pack(side=tk.LEFT, padx=5)

        self.btn_reset = tk.Button(ctrl, text="↺ 重置", command=self.reset, width=10, **style)
        self.btn_reset.pack(side=tk.LEFT, padx=5)

        # 模式选择
        self.mode_var = tk.StringVar(value="follow")
        tk.Radiobutton(ctrl, text="跟随模式", variable=self.mode_var, value="follow",
                       bg="#2d2d44", fg="#aaa", selectcolor="#3d3d5c",
                       activebackground="#2d2d44").pack(side=tk.LEFT, padx=10)
        tk.Radiobutton(ctrl, text="编队模式", variable=self.mode_var, value="formation",
                       bg="#2d2d44", fg="#aaa", selectcolor="#3d3d5c",
                       activebackground="#2d2d44").pack(side=tk.LEFT, padx=5)

        self.btn_leader = tk.Button(ctrl, text="切换领航者", command=self.switch_leader, **style)
        self.btn_leader.pack(side=tk.LEFT, padx=10)

        self.lbl_status = tk.Label(ctrl, text="就绪", bg="#2d2d44", fg="#4fc3f7",
                                   font=("Consolas", 10))
        self.lbl_status.pack(side=tk.RIGHT, padx=10)

        # 画布
        self.canvas = tk.Canvas(self.root, bg="#0d1117", highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # 底部信息
        self.lbl_info = tk.Label(self.root, text="", bg="#2d2d44", fg="#b0b0b0",
                                  font=("Consolas", 9), anchor=tk.W, pady=5)
        self.lbl_info.pack(fill=tk.X)

        # 鼠标点击设置领航者路径点
        self.canvas.bind("<Button-1>", self._on_click)
        self.waypoints = []

    def _init_vehicles(self):
        Vehicle._id_counter = 0
        colors = ["#4fc3f7", "#ff6b6b", "#66bb6a", "#ffa726", "#ab47bc"]
        self.vehicles = []
        positions = [(-100, 0), (-180, -60), (-180, 60), (-260, -30), (-260, 30)]
        for i, (x, y) in enumerate(positions):
            self.vehicles.append(Vehicle(x, y, 0, colors[i % len(colors)]))

        self.leader_idx = 0
        # V字编队偏移
        self.v_formation = [
            (0, 0),       # leader
            (-70, -50),   # 左后
            (-70, 50),    # 右后
            (-140, -90),  # 左后后
            (-140, 90),   # 右后后
        ]
        # 一字编队
        self.line_formation = [
            (0, 0), (-70, 0), (-140, 0), (-210, 0), (-280, 0)
        ]
        self.formation_offsets = self.v_formation

        self.waypoints = [(200, 0), (200, 200), (-200, 200), (-200, 0)]
        self.wp_idx = 0
        self.step_count = 0

    def _on_click(self, event):
        w = self.canvas.winfo_width()
        h = self.canvas.winfo_height()
        x = event.x - w//2
        y = -(event.y - h//2)
        self.waypoints.append((x, y))

    def toggle(self):
        self.running = not self.running
        self.btn_run.config(text="⏸ 暂停" if self.running else "▶ 启动")
        if self.running:
            self._tick()

    def reset(self):
        self.running = False
        self.btn_run.config(text="▶ 启动")
        self._init_vehicles()
        self._draw()

    def switch_leader(self):
        self.leader_idx = (self.leader_idx + 1) % len(self.vehicles)
        self.lbl_status.config(text=f"领航者: {self.vehicles[self.leader_idx].name}")

    def _tick(self):
        if not self.running:
            return
        self._step()
        self._draw()
        self.root.after(int(self.dt * 1000), self._tick)

    def _step(self):
        self.mode = self.mode_var.get()
        leader = self.vehicles[self.leader_idx]

        # 领航者航点跟踪
        if self.waypoints:
            tx, ty = self.waypoints[self.wp_idx]
            dist = math.sqrt((leader.x - tx)**2 + (leader.y - ty)**2)
            angle = math.atan2(ty - leader.y, tx - leader.x)
            heading_err = angle - leader.theta
            while heading_err > math.pi:
                heading_err -= 2*math.pi
            while heading_err < -math.pi:
                heading_err += 2*math.pi

            if dist < 20:
                self.wp_idx = (self.wp_idx + 1) % len(self.waypoints)
            leader.set_control(min(50, 1.0*dist), 2.0*heading_err)
        else:
            # 鼠标路径漫游
            leader.set_control(40, 0.2 * math.sin(self.step_count * 0.02))

        leader.update(self.dt)

        # 其它车辆
        for i, v in enumerate(self.vehicles):
            if i == self.leader_idx:
                continue
            if self.mode == "follow":
                # 跟随前车(编号-1)
                target = self.vehicles[i-1] if i > 0 else leader
                cv, cw = follow_controller(v, target, desired_dist=60)
            else:
                cv, cw = formation_controller(v, self.vehicles, self.formation_offsets, self.leader_idx)
            v.set_control(cv, cw)
            v.update(self.dt)

        self.step_count += 1

    def _draw(self):
        c = self.canvas
        c.delete("all")
        w = c.winfo_width() or 900
        h = c.winfo_height() or 600
        ox, oy = w//2, h//2
        scale = 0.6

        # 网格
        for gx in range(-500, 501, 100):
            c.create_line(ox+gx*scale, 0, ox+gx*scale, h, fill="#1a2332")
        for gy in range(-400, 401, 100):
            c.create_line(0, oy-gy*scale, w, oy-gy*scale, fill="#1a2332")

        # 航点
        for i, (wx, wy) in enumerate(self.waypoints):
            sx, sy = ox+wx*scale, oy-wy*scale
            c.create_oval(sx-4, sy-4, sx+4, sy+4, outline="#666", width=1)
            if i == self.wp_idx:
                c.create_oval(sx-8, sy-8, sx+8, sy+8, outline="#ffb74d", width=2)

        # 车辆
        for i, v in enumerate(self.vehicles):
            # 轨迹
            if len(v.trail) > 1:
                trail_pts = []
                for (tx, ty) in v.trail[-200:]:
                    trail_pts.extend([ox+tx*scale, oy-ty*scale])
                if len(trail_pts) >= 4:
                    c.create_line(trail_pts, fill=v.color, width=1, stipple="gray25")

            # 车体
            sx, sy = ox+v.x*scale, oy-v.y*scale
            size = 15
            pts = [
                (sx + size*math.cos(v.theta), sy - size*math.sin(v.theta)),
                (sx + size*0.5*math.cos(v.theta+2.3), sy - size*0.5*math.sin(v.theta+2.3)),
                (sx - size*0.3*math.cos(v.theta), sy + size*0.3*math.sin(v.theta)),
                (sx + size*0.5*math.cos(v.theta-2.3), sy - size*0.5*math.sin(v.theta-2.3)),
            ]
            c.create_polygon(pts, fill=v.color, outline="white", width=1)
            c.create_text(sx, sy-20, text=v.name, fill=v.color, font=("Consolas", 9))

            # 领航者标记
            if i == self.leader_idx:
                c.create_text(sx, sy-32, text="★", fill="#ffb74d", font=("Consolas", 12))

        # 信息
        info_parts = []
        for v in self.vehicles:
            info_parts.append(f"{v.name}:({v.x:.0f},{v.y:.0f}) v={v.v:.1f}")
        self.lbl_info.config(text=" | ".join(info_parts) + f" | 步数:{self.step_count}")


# ============================================================
#  主入口
# ============================================================

if __name__ == "__main__":
    root = tk.Tk()
    app = MultiAgentApp(root)
    root.mainloop()
