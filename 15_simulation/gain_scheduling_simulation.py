"""
增益调度PID仿真 (Gain Scheduling PID Simulation)
===================================================
变参数系统：系统参数随工作点变化，使用增益调度策略保持控制性能。
场景：直流电机在不同转速下惯性变化，温度控制在不同温度下热容变化。

Author: nuedc-asset-library
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
matplotlib.rcParams['axes.unicode_minus'] = False


class VariablePlant:
    """变参数被控对象（参数随状态/时间变化）"""

    def __init__(self, plant_type='motor'):
        self.plant_type = plant_type
        self.y = 0.0
        self.dy = 0.0

    def get_params(self, y):
        """根据当前输出获取系统参数"""
        if self.plant_type == 'motor':
            # 直流电机：惯性随转速增大，阻尼随转速增大
            J = 0.01 + 0.005 * abs(y)   # 转动惯量
            b = 0.1 + 0.05 * abs(y)      # 阻尼系数
            K = 1.0 - 0.2 * abs(y)       # 增益(饱和)
            tau = J / max(b, 0.01)
            K_dc = K / max(b, 0.01)
            return K_dc, tau
        elif self.plant_type == 'thermal':
            # 热系统：热容随温度变化
            C = 10.0 + 0.5 * abs(y)      # 热容
            R = 2.0 + 0.1 * y             # 热阻
            return 1.0, C * R
        else:
            return 1.0, 1.0

    def step(self, u, dt):
        """执行一步仿真（二阶系统）"""
        K, tau = self.get_params(self.y)
        # 二阶模型: tau * dy'' + dy' = K * u
        # 离散化
        ddy = (K * u - self.dy) / max(tau, 0.1)
        self.dy += ddy * dt
        self.y += self.dy * dt
        return self.y


class PIDController:
    """标准PID控制器"""

    def __init__(self, Kp, Ki, Kd, dt, output_limit=(-100, 100)):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.output_limit = output_limit
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error):
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error

        u = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        u = np.clip(u, self.output_limit[0], self.output_limit[1])
        return u

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


class GainScheduler:
    """增益调度器：根据工作点选择PID参数"""

    def __init__(self, schedule_table):
        """
        schedule_table: list of (threshold, Kp, Ki, Kd)
        按阈值从小到大排列
        """
        self.schedule_table = schedule_table

    def get_gains(self, y):
        """根据当前输出插值获取PID参数"""
        # 边界处理
        if y <= self.schedule_table[0][0]:
            return self.schedule_table[0][1], self.schedule_table[0][2], self.schedule_table[0][3]
        if y >= self.schedule_table[-1][0]:
            return self.schedule_table[-1][1], self.schedule_table[-1][2], self.schedule_table[-1][3]

        # 线性插值
        for i in range(len(self.schedule_table) - 1):
            t0, kp0, ki0, kd0 = self.schedule_table[i]
            t1, kp1, ki1, kd1 = self.schedule_table[i + 1]
            if t0 <= y <= t1:
                alpha = (y - t0) / (t1 - t0)
                kp = kp0 + alpha * (kp1 - kp0)
                ki = ki0 + alpha * (ki1 - ki0)
                kd = kd0 + alpha * (kd1 - kd0)
                return kp, ki, kd

        return self.schedule_table[0][1], self.schedule_table[0][2], self.schedule_table[0][3]


def simulate(dt=0.01, T=20.0, plant_type='motor'):
    """运行仿真"""
    N = int(T / dt)
    t = np.linspace(0, T, N)

    # 参考信号：阶跃变化
    ref = np.zeros(N)
    ref[int(2/dt):int(8/dt)] = 2.0
    ref[int(8/dt):int(14/dt)] = 5.0
    ref[int(14/dt):] = 1.0

    # --- 1. 固定PID ---
    plant_fixed = VariablePlant(plant_type)
    pid_fixed = PIDController(Kp=3.0, Ki=1.0, Kd=0.5, dt=dt)
    y_fixed = np.zeros(N)

    for i in range(N):
        error = ref[i] - y_fixed[i]
        u = pid_fixed.compute(error)
        y_fixed[i] = plant_fixed.step(u, dt)

    # --- 2. 增益调度PID ---
    plant_gs = VariablePlant(plant_type)
    schedule = [
        (0.0, 2.0, 0.8, 0.3),   # 低速区：保守增益
        (2.0, 3.5, 1.2, 0.6),   # 中速区：中等增益
        (4.0, 5.0, 2.0, 0.8),   # 高速区：积极增益
        (7.0, 6.0, 2.5, 1.0),   # 超高速：高增益
    ]
    scheduler = GainScheduler(schedule)
    pid_gs = PIDController(Kp=3.0, Ki=1.0, Kd=0.5, dt=dt)
    y_gs = np.zeros(N)
    kp_hist, ki_hist, kd_hist = np.zeros(N), np.zeros(N), np.zeros(N)

    for i in range(N):
        kp, ki, kd = scheduler.get_gains(y_gs[i])
        pid_gs.Kp, pid_gs.Ki, pid_gs.Kd = kp, ki, kd
        kp_hist[i], ki_hist[i], kd_hist[i] = kp, ki, kd
        error = ref[i] - y_gs[i]
        u = pid_gs.compute(error)
        y_gs[i] = plant_gs.step(u, dt)

    # --- 绘图 ---
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))

    axes[0].plot(t, ref, 'k--', lw=2, label='参考信号')
    axes[0].plot(t, y_fixed, 'b-', lw=1.2, label='固定PID')
    axes[0].plot(t, y_gs, 'r-', lw=1.2, label='增益调度PID')
    axes[0].set_ylabel('输出')
    axes[0].set_title(f'增益调度PID仿真 - {plant_type}系统')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t, ref - y_fixed, 'b-', lw=1, label='固定PID误差')
    axes[1].plot(t, ref - y_gs, 'r-', lw=1, label='增益调度PID误差')
    axes[1].set_ylabel('跟踪误差')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(t, kp_hist, 'r-', lw=1.5, label='Kp')
    axes[2].plot(t, ki_hist, 'g-', lw=1.5, label='Ki')
    axes[2].plot(t, kd_hist, 'b-', lw=1.5, label='Kd')
    axes[2].set_xlabel('时间 (s)')
    axes[2].set_ylabel('增益值')
    axes[2].set_title('增益调度参数变化')
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('gain_scheduling_result.png', dpi=150, bbox_inches='tight')
    plt.close('all')

    # 性能对比
    err_fix = np.abs(ref - y_fixed)
    err_gs = np.abs(ref - y_gs)
    print(f"{'指标':<20} {'固定PID':>12} {'增益调度PID':>12}")
    print(f"{'最大误差':<20} {err_fix.max():>12.4f} {err_gs.max():>12.4f}")
    print(f"{'平均误差':<20} {err_fix.mean():>12.4f} {err_gs.mean():>12.4f}")
    print(f"{'IAE':<20} {np.sum(err_fix)*dt:>12.4f} {np.sum(err_gs)*dt:>12.4f}")


if __name__ == '__main__':
    simulate(plant_type='motor')
