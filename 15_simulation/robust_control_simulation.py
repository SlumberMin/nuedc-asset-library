#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
鲁棒控制仿真 - H∞ + 滑模控制 + PID对比
用于电赛抗干扰控制系统设计
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
matplotlib.rcParams['axes.unicode_minus'] = False


class UncertainPlant:
    """带不确定性和扰动的二阶系统
    x'' + (a+Δa)x' + (b+Δb)x = u + d(t)
    """
    def __init__(self, a=2.0, b=5.0, K=1.0):
        self.a_nom, self.b_nom, self.K = a, b, K
        self.x = np.zeros(2)
        self.t = 0

    def get_params(self):
        # 参数不确定性: ±30%
        da = 0.3 * self.a_nom * np.sin(0.1 * self.t)
        db = 0.3 * self.b_nom * np.cos(0.07 * self.t)
        return self.a_nom + da, self.b_nom + db

    def disturbance(self, t):
        # 复合扰动: 阶跃+正弦+脉冲
        d = 0.5 * np.sin(2.0 * t)
        if 8 < t < 10:
            d += 3.0  # 阶跃扰动
        if abs(t - 15) < 0.1:
            d += 5.0  # 脉冲扰动
        return d

    def step(self, u, dt):
        a, b = self.get_params()
        d = self.disturbance(self.t)
        dx0 = self.x[1]
        dx1 = -a * self.x[1] - b * self.x[0] + self.K * u + d
        self.x[0] += dx0 * dt
        self.x[1] += dx1 * dt
        self.t += dt
        return self.x[0]

    def reset(self):
        self.x = np.zeros(2)
        self.t = 0


class StandardPID:
    """标准PID (基线)"""
    def __init__(self, Kp=5, Ki=2, Kd=1):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.integral = 0
        self.prev_error = 0

    def compute(self, error, dt):
        self.integral = np.clip(self.integral + error * dt, -10, 10)
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative

    def reset(self):
        self.integral = 0
        self.prev_error = 0


class HInfinityController:
    """简化的H∞鲁棒控制器
    基于状态反馈 + 扰动观测器 (DOB) 结构"""
    def __init__(self):
        # 状态反馈增益 (通过H∞设计得到)
        self.K1 = 8.0   # 位置反馈
        self.K2 = 4.0   # 速度反馈
        # 扰动观测器参数
        self.d_hat = 0.0
        self.L = 5.0     # 观测器增益
        self.x_hat = np.zeros(2)

    def dob_estimate(self, y, u, dt):
        """扰动观测器: 估计总扰动"""
        a, b = 2.0, 5.0  # 标称模型
        # 观测器动力学
        x_hat_dot = np.array([
            self.x_hat[1],
            -a * self.x_hat[1] - b * self.x_hat[0] + u + self.d_hat
        ]) + self.L * np.array([y - self.x_hat[0], 0])
        self.x_hat += x_hat_dot * dt

        # 扰动估计
        y_ddot_est = -a * self.x_hat[1] - b * self.x_hat[0] + u + self.d_hat
        residual = (y - self.x_hat[0]) * self.L
        self.d_hat += residual * dt * 0.5
        self.d_hat = np.clip(self.d_hat, -10, 10)
        return self.d_hat

    def compute(self, r, y, dy, u_prev, dt):
        # 扰动估计
        d_hat = self.dob_estimate(y, u_prev, dt)
        # 状态反馈 + 扰动补偿
        e = r - y
        de = 0 - dy  # 期望速度为0
        u = self.K1 * e + self.K2 * de - d_hat
        return np.clip(u, -20, 20)

    def reset(self):
        self.d_hat = 0.0
        self.x_hat = np.zeros(2)


class SlidingModeController:
    """滑模控制器 (趋近律: 指数趋近+等速趋近)"""
    def __init__(self, c=5.0, eta=3.0, eps=0.5, k=10.0):
        self.c = c           # 滑模面斜率
        self.eta = eta       # 等速趋近项增益
        self.eps = eps       # 切换增益
        self.k = k           # 指数趋近增益
        self.integral_e = 0

    def compute(self, r, y, dy, dt):
        e = r - y
        de = 0 - dy
        self.integral_e += e * dt
        # 滑模面: s = de + c*e + λ*∫e
        lam = 0.5
        s = de + self.c * e + lam * self.integral_e
        # 控制律: 等效控制 + 切换控制
        u_eq = self.c * de + lam * e  # 等效部分
        u_sw = self.eta * np.tanh(s / self.eps) + self.k * s  # 切换部分(连续近似)
        u = u_eq + u_sw
        return np.clip(u, -20, 20), s

    def reset(self):
        self.integral_e = 0


class RobustPID:
    """鲁棒PID (带积分抗饱和+扰动前馈)"""
    def __init__(self, Kp=6, Ki=2, Kd=2):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.integral = 0
        self.prev_error = 0
        self.prev_u = 0

    def compute(self, error, dt):
        self.integral = np.clip(self.integral + error * dt, -8, 8)
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error
        u = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        # 积分抗饱和
        u_sat = np.clip(u, -20, 20)
        if abs(u_sat - u) > 0.01:
            self.integral -= error * dt * 0.5
        self.prev_u = u_sat
        return u_sat

    def reset(self):
        self.integral = 0
        self.prev_error = 0


def simulate(controller, plant, ref_func, T=25, dt=0.005, ctrl_type='pid'):
    N = int(T / dt)
    t_arr = np.linspace(0, T, N)
    y_arr, u_arr, r_arr = np.zeros(N), np.zeros(N), np.zeros(N)
    s_arr = np.zeros(N)  # 滑模面(仅SMC用)

    plant.reset()
    if hasattr(controller, 'reset'):
        controller.reset()

    y_prev = 0
    for i in range(N):
        t = i * dt
        r = ref_func(t)
        y = plant.x[0]
        dy = plant.x[1]
        r_arr[i] = r

        if ctrl_type == 'pid' or ctrl_type == 'robust_pid':
            error = r - y
            u = controller.compute(error, dt)
        elif ctrl_type == 'hinfinity':
            u_prev = u_arr[max(0, i-1)]
            u = controller.compute(r, y, dy, u_prev, dt)
        elif ctrl_type == 'smc':
            u, s = controller.compute(r, y, dy, dt)
            s_arr[i] = s

        u = np.clip(u, -20, 20)
        plant.step(u, dt)
        y_arr[i] = plant.x[0]
        u_arr[i] = u
        y_prev = y

    return t_arr, y_arr, u_arr, r_arr, s_arr


def main():
    T, dt = 25, 0.005

    def ref_func(t):
        return 1.0

    controllers = {
        '标准PID': ('pid', StandardPID(Kp=5, Ki=2, Kd=1)),
        '鲁棒PID': ('robust_pid', RobustPID(Kp=6, Ki=2, Kd=2)),
        'H∞+DOB': ('hinfinity', HInfinityController()),
        '滑模控制': ('smc', SlidingModeController(c=5, eta=3, eps=0.3, k=10)),
    }

    results = {}
    for name, (ctype, ctrl) in controllers.items():
        plant = UncertainPlant(a=2.0, b=5.0, K=1.0)
        t, y, u, r, s = simulate(ctrl, plant, ref_func, T, dt, ctype)
        results[name] = {'t': t, 'y': y, 'u': u, 'r': r, 's': s}

    # 绘图
    fig, axes = plt.subplots(3, 1, figsize=(14, 11))
    colors = ['#2196F3', '#4CAF50', '#FF9800', '#E91E63']

    # 输出响应
    ax = axes[0]
    ax.plot(results['标准PID']['t'], results['标准PID']['r'], 'k--', lw=1.5, label='参考')
    for (name, data), c in zip(results.items(), colors):
        ax.plot(data['t'], data['y'], color=c, lw=1.0, label=name)
    # 标注扰动区间
    ax.axvspan(8, 10, alpha=0.1, color='red', label='阶跃扰动')
    ax.axvline(15, color='red', ls=':', alpha=0.5, label='脉冲扰动')
    ax.set_ylabel('输出 y(t)')
    ax.set_title('鲁棒控制方法对比 - 含参数不确定性和外部扰动')
    ax.legend(loc='upper right', fontsize=8, ncol=2)
    ax.grid(True, alpha=0.3)

    # 误差
    ax = axes[1]
    for (name, data), c in zip(results.items(), colors):
        ax.plot(data['t'], data['r'] - data['y'], color=c, lw=0.8, label=name)
    ax.axvspan(8, 10, alpha=0.1, color='red')
    ax.set_ylabel('跟踪误差 e(t)')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)

    # 控制量
    ax = axes[2]
    for (name, data), c in zip(results.items(), colors):
        ax.plot(data['t'], data['u'], color=c, lw=0.8, label=name)
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('控制量 u(t)')
    ax.set_title('控制量对比')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('robust_control_comparison.png', dpi=150, bbox_inches='tight')
    plt.show()

    # 滑模面相图
    fig2, ax2 = plt.subplots(figsize=(8, 6))
    d = results['滑模控制']
    ax2.plot(d['t'], d['s'], '#E91E63', lw=1.0)
    ax2.axhline(0, color='k', ls='--', lw=0.8)
    ax2.set_xlabel('时间 (s)')
    ax2.set_ylabel('滑模面 s(t)')
    ax2.set_title('滑模面收敛过程')
    ax2.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('sliding_surface.png', dpi=150, bbox_inches='tight')
    plt.show()

    # 性能指标
    print("\n" + "="*75)
    print("鲁棒控制方法性能指标对比 (含扰动)")
    print("="*75)
    print(f"{'方法':<12} {'ISE':>10} {'IAE':>10} {'扰动恢复时间':>14} {'控制能量':>10}")
    print("-"*75)
    for name, data in results.items():
        e = data['r'] - data['y']
        ise = np.sum(e**2) * dt
        iae = np.sum(np.abs(e)) * dt
        # 扰动恢复时间(扰动后|e|<0.05的时刻)
        mask = data['t'] > 10
        recovery = 0
        if np.any(mask):
            e_after = np.abs(e[mask])
            below = np.where(e_after < 0.05)[0]
            recovery = data['t'][mask][below[0]] - 10 if len(below) > 0 else float('inf')
        ctrl_energy = np.sum(data['u']**2) * dt
        print(f"{name:<12} {ise:>10.3f} {iae:>10.3f} {recovery:>13.2f}s {ctrl_energy:>10.1f}")
    print("="*75)


if __name__ == '__main__':
    main()
