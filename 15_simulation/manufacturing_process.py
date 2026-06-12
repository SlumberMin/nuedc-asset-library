"""
制造工艺仿真 - PID温控/流量控制/液位控制
============================================
仿真典型制造工艺中的三大经典控制回路：
1. PID温度控制（加热炉/反应釜）
2. 流量控制（管道阀门）
3. 液位控制（储罐系统）

每个子系统包含物理模型、PID控制器、扰动注入和性能评估。
"""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False

# ─────────────────────────────────────────────
# PID 控制器
# ─────────────────────────────────────────────
class PIDController:
    """通用PID控制器（带抗积分饱和）"""

    def __init__(self, Kp, Ki, Kd, dt, out_min=-100, out_max=100):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.dt = dt
        self.out_min, self.out_max = out_min, out_max
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, setpoint, measurement):
        error = setpoint - measurement
        self.integral += error * self.dt
        # 抗积分饱和：Clamp积分项
        self.integral = np.clip(self.integral, self.out_min / max(self.Ki, 1e-6),
                                self.out_max / max(self.Ki, 1e-6))
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        return np.clip(output, self.out_min, self.out_max)

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


# ─────────────────────────────────────────────
# 1. 温度控制仿真（一阶+纯滞后系统）
# ─────────────────────────────────────────────
class ThermalProcess:
    """
    加热炉温度模型: 一阶惯性 + 纯滞后
    G(s) = K / (τs + 1) * e^(-θs)
    离散化: T(k+1) = T(k) + dt/τ * [K*u(k-θ/dt) - (T(k) - T_env)]
    """

    def __init__(self, K=2.0, tau=120.0, theta=10.0, T_env=25.0, dt=0.5):
        self.K = K          # 增益 (°C/%功率)
        self.tau = tau       # 时间常数 (s)
        self.theta = theta   # 纯滞后 (s)
        self.T_env = T_env   # 环境温度
        self.dt = dt
        self.T = T_env       # 当前温度
        self.delay_buffer = [0.0] * int(theta / dt)  # 滞后缓冲

    def step(self, power_pct):
        """功率百分比 -> 下一时刻温度"""
        delayed_power = self.delay_buffer.pop(0)
        self.delay_buffer.append(power_pct)
        dT = self.dt / self.tau * (self.K * delayed_power - (self.T - self.T_env))
        self.T += dT
        return self.T


# ─────────────────────────────────────────────
# 2. 流量控制仿真（非线性阀门 + 管道）
# ─────────────────────────────────────────────
class FlowProcess:
    """
    流量模型: Q = Cv * sqrt(ΔP) * f(u)
    阀门特性: 等百分比特性 f(u) = R^(u-1), R=30
    管道惯性: 一阶滤波
    """

    def __init__(self, Cv=10.0, delta_P=2.0, R_valve=30.0, tau=2.0, dt=0.1):
        self.Cv = Cv
        self.delta_P = delta_P
        self.R = R_valve
        self.tau = tau
        self.dt = dt
        self.Q = 0.0  # 当前流量 L/min

    def step(self, valve_pct):
        """阀门开度百分比(0-100) -> 流量 L/min"""
        u = np.clip(valve_pct, 0, 100) / 100.0
        f_u = self.R ** (u - 1)  # 等百分比特性
        Q_target = self.Cv * np.sqrt(self.delta_P) * f_u
        # 一阶滤波模拟管道动态
        self.Q += self.dt / self.tau * (Q_target - self.Q)
        return self.Q


# ─────────────────────────────────────────────
# 3. 液位控制仿真（非线性储罐）
# ─────────────────────────────────────────────
class LevelProcess:
    """
    液位模型: 基于质量守恒
    A * dh/dt = Q_in - Q_out
    Q_out = Cd * a * sqrt(2*g*h)  (托里拆利定律)
    """

    def __init__(self, A=1.0, Cd=0.6, a_out=0.005, g=9.81, dt=0.1):
        self.A = A            # 储罐截面积 m^2
        self.Cd = Cd          # 流出系数
        self.a_out = a_out    # 出口面积 m^2
        self.g = g
        self.dt = dt
        self.h = 1.0          # 初始液位 m
        self.h_max = 3.0      # 最大液位

    def step(self, Q_in):
        """入口流量(m^3/s) -> 液位(m)"""
        Q_out = self.Cd * self.a_out * np.sqrt(2 * self.g * max(self.h, 0))
        dh = self.dt / self.A * (Q_in - Q_out)
        self.h = np.clip(self.h + dh, 0, self.h_max)
        return self.h


# ─────────────────────────────────────────────
# 仿真主循环
# ─────────────────────────────────────────────
def run_temperature_control(duration=600, dt=0.5):
    """PID温度控制仿真"""
    process = ThermalProcess(K=2.0, tau=120.0, theta=10.0, T_env=25.0, dt=dt)
    pid = PIDController(Kp=8.0, Ki=0.05, Kd=15.0, dt=dt, out_min=0, out_max=100)

    steps = int(duration / dt)
    t = np.arange(steps) * dt
    temp_log, setpoint_log, power_log = [], [], []

    setpoint = 25.0
    for i in range(steps):
        # 阶跃设定值变化
        if i == int(100 / dt):
            setpoint = 150.0
        elif i == int(350 / dt):
            setpoint = 200.0
        # 扰动：环境温度变化
        T_env_now = 25.0 + 5.0 * np.sin(2 * np.pi * t[i] / 600)
        process.T_env = T_env_now

        power = pid.compute(setpoint, process.T)
        temp = process.step(power)
        temp_log.append(temp)
        setpoint_log.append(setpoint)
        power_log.append(power)

    # 性能指标
    idx1 = int(100 / dt)
    idx2 = int(350 / dt)
    overshoot1 = (max(temp_log[idx1:idx2]) - 150) / 150 * 100
    settle_idx = idx1
    for j in range(idx1, idx2):
        if abs(temp_log[j] - 150) < 1.5 and j > idx1 + 50:
            settle_idx = j
            break
    settle_time1 = (settle_idx - idx1) * dt

    print(f"[温度控制] 阶跃1超调: {overshoot1:.1f}%, 调节时间: {settle_time1:.1f}s")

    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    fig.suptitle('PID温度控制仿真', fontsize=14)
    axes[0].plot(t, temp_log, 'r-', label='温度')
    axes[0].plot(t, setpoint_log, 'k--', label='设定值')
    axes[0].set_ylabel('温度 (°C)')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    axes[1].plot(t, power_log, 'b-', label='加热功率')
    axes[1].set_xlabel('时间 (s)')
    axes[1].set_ylabel('功率 (%)')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('./nuedc-asset-library/15_simulation/temperature_control.png', dpi=150)
    plt.close()
    print("[温度控制] 图表已保存")


def run_flow_control(duration=100, dt=0.1):
    """PID流量控制仿真"""
    process = FlowProcess(Cv=10.0, delta_P=2.0, R_valve=30.0, tau=2.0, dt=dt)
    pid = PIDController(Kp=3.0, Ki=0.8, Kd=0.5, dt=dt, out_min=0, out_max=100)

    steps = int(duration / dt)
    t = np.arange(steps) * dt
    flow_log, setpoint_log, valve_log = [], [], []

    setpoint = 20.0
    for i in range(steps):
        if i == int(20 / dt):
            setpoint = 35.0
        elif i == int(60 / dt):
            setpoint = 15.0
        # 扰动：压力波动
        process.delta_P = 2.0 + 0.3 * np.sin(2 * np.pi * t[i] / 30)

        valve = pid.compute(setpoint, process.Q)
        flow = process.step(valve)
        flow_log.append(flow)
        setpoint_log.append(setpoint)
        valve_log.append(valve)

    print(f"[流量控制] 最终流量: {flow_log[-1]:.2f} L/min, 设定值: {setpoint_log[-1]:.1f}")

    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    fig.suptitle('PID流量控制仿真', fontsize=14)
    axes[0].plot(t, flow_log, 'b-', label='流量')
    axes[0].plot(t, setpoint_log, 'k--', label='设定值')
    axes[0].set_ylabel('流量 (L/min)')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    axes[1].plot(t, valve_log, 'g-', label='阀门开度')
    axes[1].set_xlabel('时间 (s)')
    axes[1].set_ylabel('开度 (%)')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('./nuedc-asset-library/15_simulation/flow_control.png', dpi=150)
    plt.close()
    print("[流量控制] 图表已保存")


def run_level_control(duration=200, dt=0.1):
    """PID液位控制仿真"""
    process = LevelProcess(A=1.0, Cd=0.6, a_out=0.005, g=9.81, dt=dt)
    pid = PIDController(Kp=0.02, Ki=0.001, Kd=0.08, dt=dt, out_min=0, out_max=0.05)

    steps = int(duration / dt)
    t = np.arange(steps) * dt
    level_log, setpoint_log, Qin_log = [], [], []

    setpoint = 1.5
    for i in range(steps):
        if i == int(50 / dt):
            setpoint = 2.0
        elif i == int(120 / dt):
            setpoint = 1.0
        # 扰动：出口堵塞
        if 80 / dt <= i < 100 / dt:
            process.a_out = 0.003
        else:
            process.a_out = 0.005

        Q_in = pid.compute(setpoint, process.h)
        level = process.step(Q_in)
        level_log.append(level)
        setpoint_log.append(setpoint)
        Qin_log.append(Q_in)

    print(f"[液位控制] 最终液位: {level_log[-1]:.3f} m, 设定值: {setpoint_log[-1]:.1f} m")

    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    fig.suptitle('PID液位控制仿真', fontsize=14)
    axes[0].plot(t, level_log, 'm-', label='液位')
    axes[0].plot(t, setpoint_log, 'k--', label='设定值')
    axes[0].set_ylabel('液位 (m)')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    axes[1].plot(t, Qin_log, 'c-', label='入口流量')
    axes[1].set_xlabel('时间 (s)')
    axes[1].set_ylabel('流量 (m³/s)')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('./nuedc-asset-library/15_simulation/level_control.png', dpi=150)
    plt.close()
    print("[液位控制] 图表已保存")


if __name__ == '__main__':
    print("=" * 60)
    print("制造工艺仿真 - PID温控/流量控制/液位控制")
    print("=" * 60)
    run_temperature_control()
    run_flow_control()
    run_level_control()
    print("\n✅ 全部制造工艺仿真完成！")
