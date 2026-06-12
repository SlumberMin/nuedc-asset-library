#!/usr/bin/env python3
"""
倒立摆LQR仿真 — 摆起 + 平衡 + 扰动抑制
==========================================
- 能量摆起控制（Energy-based swing-up）
- LQR平衡控制
- 扰动抑制
- 动画演示
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rcParams
from matplotlib.patches import Rectangle, FancyArrowPatch
rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
rcParams['axes.unicode_minus'] = False

# ============================================================
# 1. 倒立摆物理模型（非线性）
# ============================================================
class InvertedPendulum:
    """
    小车-倒立摆系统
    状态: [x, x_dot, theta, theta_dot]
    x: 小车位移, theta: 摆杆角度(0=竖直向上)
    """
    def __init__(self):
        self.M = 1.0    # 小车质量 (kg)
        self.m = 0.1    # 摆杆质量 (kg)
        self.l = 0.5    # 摆杆半长 (m)
        self.g = 9.81   # 重力加速度
        self.b = 0.1    # 摩擦系数
        self.dt = 0.01
        self.state = np.array([0.0, 0.0, np.pi, 0.0])  # 初始倒挂

    def reset(self, theta0=np.pi):
        self.state = np.array([0.0, 0.0, theta0, 0.0])

    def nonlinear_dynamics(self, state, u):
        """非线性动力学"""
        x, x_dot, theta, theta_dot = state
        sin_t = np.sin(theta)
        cos_t = np.cos(theta)
        total_mass = self.M + self.m
        pole_half_len = self.l

        # 摆杆转动惯量 (关于质心)
        J = (1/3) * self.m * (2*pole_half_len)**2  # ~ 0.0333

        # 加速度
        temp = (u + self.m * pole_half_len * theta_dot**2 * sin_t - self.b * x_dot)
        denom = total_mass - self.m * cos_t**2

        theta_acc = (self.g * sin_t * total_mass - cos_t * temp) / \
                    (pole_half_len * (4/3 * total_mass - self.m * cos_t**2))
        x_acc = (temp - self.m * pole_half_len * theta_acc * cos_t) / total_mass

        return np.array([x_dot, x_acc, theta_dot, theta_acc])

    def step(self, u, dt=None):
        """RK4积分"""
        dt = dt or self.dt
        s = self.state
        k1 = self.nonlinear_dynamics(s, u)
        k2 = self.nonlinear_dynamics(s + 0.5*dt*k1, u)
        k3 = self.nonlinear_dynamics(s + 0.5*dt*k2, u)
        k4 = self.nonlinear_dynamics(s + dt*k3, u)
        self.state = s + (dt/6)*(k1 + 2*k2 + 2*k3 + k4)
        # 限制小车位置
        self.state[0] = np.clip(self.state[0], -3, 3)
        return self.state.copy()

    def linearize(self):
        """在竖直向上平衡点线性化 -> A, B 矩阵"""
        M, m, l, g, b = self.M, self.m, self.l, self.g, self.b
        total = M + m
        J = (1/3) * m * (2*l)**2
        denom = l * (4/3 * total - m)

        A = np.array([
            [0, 1, 0, 0],
            [0, -b/total, -m*g*l/total, 0],   # 近似（小角度）
            [0, 0, 0, 1],
            [0, b*l*M/(total*denom), g*total/denom, 0]
        ])

        B = np.array([
            [0],
            [1/total],
            [0],
            [-l/(denom)]
        ])

        # 修正A[1,2]和A[3,2]
        A[1, 2] = -m * g / total  # 小角度近似
        A[3, 2] = g * total / denom

        return A, B


# ============================================================
# 2. LQR控制器
# ============================================================
class LQRController:
    """LQR最优控制器"""
    def __init__(self, A, B, Q, R):
        self.K = self._solve_lqr(A, B, Q, R)

    def _solve_lqr(self, A, B, Q, R, max_iter=200, tol=1e-10):
        """迭代法求解代数Riccati方程"""
        P = Q.copy()
        for _ in range(max_iter):
            K = np.linalg.inv(R + B.T @ P @ B) @ B.T @ P @ A
            P_new = Q + A.T @ P @ A - A.T @ P @ B @ K
            if np.max(np.abs(P_new - P)) < tol:
                break
            P = P_new
        return K

    def compute(self, state):
        u = -self.K @ state
        return np.clip(u[0], -30, 30)


# ============================================================
# 3. 能量摆起控制器
# ============================================================
class SwingUpController:
    """基于能量的摆起控制"""
    def __init__(self, plant):
        self.plant = plant
        self.m = plant.m
        self.l = plant.l
        self.g = plant.g

    def compute(self, state):
        _, _, theta, theta_dot = state
        # 摆杆能量 (以铰链为参考)
        E = 0.5 * self.m * (2*self.l)**2 / 3 * theta_dot**2 - \
            self.m * self.g * self.l * np.cos(theta)
        E_target = self.m * self.g * self.l  # 竖直向上的能量
        dE = E - E_target
        # 能量反馈
        u = -10.0 * dE * np.sign(theta_dot * np.cos(theta))
        return np.clip(u, -20, 20)


# ============================================================
# 4. 混合控制器：摆起 + LQR切换
# ============================================================
class HybridController:
    def __init__(self, plant):
        A, B = plant.linearize()
        Q = np.diag([10, 1, 100, 10])  # 状态权重
        R = np.array([[0.1]])           # 控制权重
        self.lqr = LQRController(A, B, Q, R)
        self.swing_up = SwingUpController(plant)
        self.switch_angle = 0.3  # 切换阈值(rad)

    def compute(self, state):
        theta = state[2]
        # 归一化角度到[-pi, pi]
        theta_norm = (theta + np.pi) % (2*np.pi) - np.pi
        if abs(theta_norm) < self.switch_angle:
            return self.lqr.compute(state), 'LQR'
        else:
            return self.swing_up.compute(state), '摆起'


# ============================================================
# 5. 仿真
# ============================================================
def run_simulation():
    dt = 0.01
    T_total = 15.0
    n = int(T_total / dt)
    t = np.linspace(0, T_total, n)

    plant = InvertedPendulum()
    plant.reset(theta0=np.pi + 0.1)  # 接近倒挂
    ctrl = HybridController(plant)

    states = np.zeros((n, 4))
    controls = np.zeros(n)
    modes = []

    for i in range(n):
        s = plant.state
        states[i] = s
        u, mode = ctrl.compute(s)
        controls[i] = u
        modes.append(mode)

        # 在t=10s加入脉冲扰动
        if abs(t[i] - 10.0) < dt/2:
            plant.state[3] += 3.0  # 角速度扰动

        plant.step(u, dt)

    return t, states, controls, modes


# ============================================================
# 6. 可视化
# ============================================================
def plot_results(t, states, controls, modes):
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('倒立摆LQR仿真 — 摆起 + 平衡 + 扰动抑制', fontsize=15, fontweight='bold')

    # 角度（转为度）
    theta_deg = np.degrees((states[:, 2] + np.pi) % (2*np.pi) - np.pi)

    # (a) 角度
    ax = axes[0, 0]
    ax.plot(t, theta_deg, 'b-', lw=1)
    ax.axhline(0, color='g', ls='--', lw=2, label='平衡点')
    ax.axvline(10, color='r', ls=':', label='扰动时刻')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('摆杆角度 (°)')
    ax.set_title('(a) 摆杆角度响应')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # (b) 小车位置
    ax = axes[0, 1]
    ax.plot(t, states[:, 0], 'r-', lw=1)
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('小车位移 (m)')
    ax.set_title('(b) 小车位置')
    ax.grid(True, alpha=0.3)

    # (c) 控制量
    ax = axes[1, 0]
    ax.plot(t, controls, 'purple', lw=0.8)
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('控制力 (N)')
    ax.set_title('(c) 控制输入')
    ax.grid(True, alpha=0.3)

    # (d) 相平面
    ax = axes[1, 1]
    theta_dot = states[:, 3]
    # 摆起阶段用红色，LQR阶段用蓝色
    for i in range(0, len(t), 5):
        c = 'red' if modes[i] == '摆起' else 'blue'
        ax.plot(theta_deg[i], theta_dot[i], '.', color=c, markersize=2)
    ax.plot(theta_deg[0], theta_dot[0], 'go', ms=10, label='起点')
    ax.plot(theta_deg[-1], theta_dot[-1], 'r*', ms=15, label='终点')
    ax.axvline(0, color='gray', ls='--', lw=0.5)
    ax.set_xlabel('角度 (°)')
    ax.set_ylabel('角速度 (rad/s)')
    ax.set_title('(d) 相平面 (红=摆起, 蓝=LQR)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('inverted_pendulum_lqr.png', dpi=150, bbox_inches='tight')
    plt.show()


# ============================================================
# 7. 简易动画（可选）
# ============================================================
def animate(t, states, step=10):
    """简易帧动画"""
    from matplotlib.animation import FuncAnimation

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.set_xlim(-3, 3)
    ax.set_ylim(-1, 1.5)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)

    cart_w, cart_h = 0.4, 0.2
    cart = Rectangle((0, 0), cart_w, cart_h, fc='#2196F3', ec='black', lw=2)
    rod, = ax.plot([], [], 'r-', lw=3)
    trail, = ax.plot([], [], 'g-', lw=0.5, alpha=0.5)
    ax.add_patch(cart)

    trail_x, trail_y = [], []

    def init():
        cart.set_xy((-cart_w/2, 0))
        rod.set_data([], [])
        return cart, rod, trail

    def update(frame):
        i = frame * step
        if i >= len(t):
            i = len(t) - 1
        x, _, theta, _ = states[i]
        # 归一化
        theta = (theta + np.pi) % (2*np.pi) - np.pi

        cart.set_xy((x - cart_w/2, 0))
        px = x + 1.0 * np.sin(theta)
        py = cart_h + 1.0 * np.cos(theta)
        rod.set_data([x, px], [cart_h, py])

        trail_x.append(px)
        trail_y.append(py)
        trail.set_data(trail_x, trail_y)

        ax.set_title(f'倒立摆 t={t[i]:.2f}s', fontsize=13)
        return cart, rod, trail

    n_frames = len(t) // step
    ani = FuncAnimation(fig, update, frames=n_frames, init_func=init,
                        interval=20, blit=False)
    plt.show()
    return ani


# ============================================================
if __name__ == '__main__':
    print('正在运行倒立摆LQR仿真...')
    t, states, controls, modes = run_simulation()
    plot_results(t, states, controls, modes)

    # 打印最终状态
    theta_final = np.degrees((states[-1, 2] + np.pi) % (2*np.pi) - np.pi)
    print(f'\n最终摆杆角度: {theta_final:.2f}°')
    print(f'最终小车位置: {states[-1, 0]:.3f} m')
    print(f'摆起→LQR切换次数: {modes.count("LQR")} / {len(modes)} 步处于LQR模式')
    print('\n倒立摆仿真完成！')

    # 如需动画取消下行注释
    # animate(t, states, step=5)
