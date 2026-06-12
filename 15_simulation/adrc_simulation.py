"""
ADRC自抗扰控制仿真 - 对比PID控制
适用于电赛电机控制、平衡小车等场景

依赖: pip install numpy matplotlib
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# ============ 被控对象: 二阶系统 ============
# 电机模型: J*ddtheta + b*dtheta = K*u + d(t)
J = 0.01    # 转动惯量
b = 0.1     # 阻尼系数
K = 1.0     # 电机增益

def plant(y, dy, u, d=0):
    """二阶被控对象: J*y'' + b*y' = K*u + d"""
    ddy = (K * u + d - b * dy) / J
    return ddy

# ============ PID控制器 ============
class PID:
    def __init__(self, kp, ki, kd, dt):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dt = dt
        self.integral = 0
        self.prev_error = 0

    def update(self, ref, y):
        error = ref - y
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

# ============ ADRC控制器 ============
class TD:
    """跟踪微分器"""
    def __init__(self, r, h, dt):
        self.r = r
        self.h = h
        self.dt = dt
        self.x1 = 0
        self.x2 = 0

    def update(self, v):
        d = self.r * self.h
        d0 = d * self.h
        y = self.x1 - v + self.h * self.x2
        a0 = np.sqrt(d * d + 8 * self.r * abs(y))
        if y > 0:
            a = self.x2 + (a0 - d) / 2
        else:
            a = self.x2 - (a0 - d) / 2
        if abs(a) <= d:
            fs = -self.r * a / d
        else:
            fs = -self.r * np.sign(a)
        self.x1 += self.dt * self.x2
        self.x2 += self.dt * fs
        return self.x1, self.x2

class ESO:
    """扩张状态观测器 (二阶对象)"""
    def __init__(self, beta1, beta2, beta3, b0, dt):
        self.beta1, self.beta2, self.beta3 = beta1, beta2, beta3
        self.b0 = b0
        self.dt = dt
        self.z1 = 0
        self.z2 = 0
        self.z3 = 0

    def update(self, y, u):
        e = self.z1 - y
        self.z1 += self.dt * (self.z2 - self.beta1 * e)
        self.z2 += self.dt * (self.z3 - self.beta2 * e + self.b0 * u)
        self.z3 += self.dt * (-self.beta3 * e)
        return self.z1, self.z2, self.z3

class ADRC:
    """二阶ADRC控制器"""
    def __init__(self, dt):
        self.dt = dt
        # TD参数
        self.td = TD(r=100, h=dt*10, dt=dt)
        # ESO参数
        self.eso = ESO(beta1=100, beta2=300, beta3=1000, b0=K/J, dt=dt)
        # 控制增益
        self.kp = 10
        self.kd = 5
        self.b0 = K / J

    def update(self, ref, y):
        v1, v2 = self.td.update(ref)
        z1, z2, z3 = self.eso.update(y, 0)  # u由上一步决定,此处简化
        # 非线性误差反馈
        e1 = v1 - z1
        e2 = v2 - z2
        u0 = self.kp * e1 + self.kd * e2
        # 扰动补偿
        u = (u0 - z3) / self.b0
        return u

    def update_full(self, ref, y, u_prev):
        """完整ADRC更新(使用上一步控制量进行ESO估计)"""
        v1, v2 = self.td.update(ref)
        z1, z2, z3 = self.eso.update(y, u_prev)
        e1 = v1 - z1
        e2 = v2 - z2
        u0 = self.kp * e1 + self.kd * e2
        u = (u0 - z3) / self.b0
        return np.clip(u, -50, 50)

# ============ 仿真主循环 ============
def run_simulation(controller_type='adrc', disturbance_type='step'):
    dt = 0.001
    t_end = 2.0
    steps = int(t_end / dt)
    t = np.linspace(0, t_end, steps)

    # 参考信号
    ref = np.ones(steps) * 1.0
    ref[int(0.5/dt):int(1.0/dt)] = 2.0
    ref[int(1.5/dt):] = 0.5

    # 扰动
    d = np.zeros(steps)
    if disturbance_type == 'step':
        d[int(1.0/dt):int(1.2/dt)] = 5.0  # 阶跃扰动
    elif disturbance_type == 'sin':
        d = 3.0 * np.sin(2 * np.pi * 5 * t)  # 正弦扰动

    if controller_type == 'pid':
        ctrl = PID(kp=15, ki=5, kd=0.5, dt=dt)
    else:
        ctrl = ADRC(dt)

    y, dy = 0, 0
    ys, us = [], []
    u_out = 0

    for i in range(steps):
        if controller_type == 'adrc':
            u_out = ctrl.update_full(ref[i], y, u_out)
        else:
            u_out = ctrl.update(ref[i], y)
        u_out = np.clip(u_out, -50, 50)

        ddy = plant(y, dy, u_out, d[i])
        dy += ddy * dt
        y += dy * dt
        ys.append(y)
        us.append(u_out)

    return t, np.array(ys), np.array(us), ref, d

# ============ 运行对比仿真 ============
if __name__ == '__main__':
    t, y_pid, u_pid, ref, d = run_simulation('pid')
    _, y_adrc, u_adrc, _, _ = run_simulation('adrc')

    fig, axes = plt.subplots(3, 1, figsize=(12, 10))

    axes[0].plot(t, ref, 'k--', label='参考', linewidth=1.5)
    axes[0].plot(t, y_pid, 'r-', label='PID', alpha=0.8)
    axes[0].plot(t, y_adrc, 'b-', label='ADRC', alpha=0.8)
    axes[0].set_ylabel('输出')
    axes[0].legend()
    axes[0].set_title('ADRC vs PID 响应对比')
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t, u_pid, 'r-', label='PID控制量', alpha=0.8)
    axes[1].plot(t, u_adrc, 'b-', label='ADRC控制量', alpha=0.8)
    axes[1].set_ylabel('控制量')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(t, d, 'g-', label='扰动')
    axes[2].set_xlabel('时间 (s)')
    axes[2].set_ylabel('扰动')
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('adrc_vs_pid.png', dpi=150)
    plt.close('all')

    # 性能指标
    for name, y in [('PID', y_pid), ('ADRC', y_adrc)]:
        error = ref - y
        rmse = np.sqrt(np.mean(error**2))
        iae = np.sum(np.abs(error)) * (t[1]-t[0])
        print(f'{name}: RMSE={rmse:.4f}, IAE={iae:.4f}')
