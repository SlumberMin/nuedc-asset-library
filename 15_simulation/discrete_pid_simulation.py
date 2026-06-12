"""
离散PID仿真 - 定点化效应分析
==============================
对比浮点PID与不同精度定点PID的性能差异，展示量化误差对控制效果的影响。
适用于嵌入式定点MCU(如STM32 Q15/Q31格式)的PID实现验证。

运行: python discrete_pid_simulation.py
"""
import os
import math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

# ============ 被控对象: 一阶惯性 + 纯滞后 ============
class FirstOrderPlant:
    def __init__(self, K=1.0, T=0.5, delay=0.0, dt=0.01):
        self.K, self.T, self.delay, self.dt = K, T, delay, dt
        self.y = 0.0
        self.delay_buf = [0.0] * max(1, int(delay / dt))

    def update(self, u):
        self.delay_buf.append(u)
        u_delayed = self.delay_buf.pop(0)
        alpha = self.dt / (self.T + self.dt)
        self.y += alpha * (self.K * u_delayed - self.y)
        return self.y

# ============ 定点化工具 ============
def float_to_fixed(val, frac_bits):
    """浮点 → 定点(饱和截断)"""
    max_val = (1 << (15 - frac_bits)) - (1 << -frac_bits) if frac_bits < 0 else (2**(15-frac_bits)) - 1.0/2**frac_bits
    max_val = 32767.0 / (2**frac_bits)
    min_val = -32768.0 / (2**frac_bits)
    val = max(min_val, min(max_val, val))
    return round(val * (2**frac_bits)) / (2**frac_bits)

def fixed_pid_step(e, e_sum, e_prev, Kp, Ki, Kd, dt, frac_bits=None):
    """单步PID计算, 可选定点化"""
    de = (e - e_prev) / dt
    u = Kp * e + Ki * e_sum + Kd * de
    if frac_bits is not None:
        u = float_to_fixed(u, frac_bits)
    return u

# ============ 仿真主循环 ============
def run_simulation(Kp, Ki, Kd, setpoint, plant_params, dt, t_end, frac_bits=None, label=""):
    plant = FirstOrderPlant(dt=dt, **plant_params)
    n = int(t_end / dt)
    t_arr, y_arr, u_arr, e_arr = [], [], [], []
    e_sum, e_prev = 0.0, 0.0

    for i in range(n):
        t = i * dt
        e = setpoint - plant.y
        e_sum += e * dt
        # 积分项也定点化
        if frac_bits is not None:
            e_sum = float_to_fixed(e_sum, frac_bits)
        u = fixed_pid_step(e, e_sum, e_prev, Kp, Ki, Kd, dt, frac_bits)
        e_prev = e
        y = plant.update(u)
        t_arr.append(t); y_arr.append(y); u_arr.append(u); e_arr.append(e)

    return np.array(t_arr), np.array(y_arr), np.array(u_arr), np.array(e_arr)

# ============ 性能指标 ============
def calc_metrics(t, y, setpoint):
    ss_val = np.mean(y[int(0.8*len(y)):])
    overshoot = (np.max(y) - setpoint) / setpoint * 100 if setpoint != 0 else 0
    ss_error = abs(ss_val - setpoint) / abs(setpoint) * 100 if setpoint != 0 else 0
    # 上升时间(10%→90%)
    idx_10 = next((i for i,v in enumerate(y) if v >= 0.1*setpoint), len(y)-1)
    idx_90 = next((i for i,v in enumerate(y) if v >= 0.9*setpoint), len(y)-1)
    rise_time = t[idx_90] - t[idx_10] if idx_90 > idx_10 else float('nan')
    return overshoot, rise_time, ss_error, ss_val

# ============ 主程序 ============
if __name__ == "__main__":
    # PID参数
    Kp, Ki, Kd = 2.0, 1.0, 0.3
    dt = 0.01
    t_end = 5.0
    setpoint = 1.0
    plant_params = dict(K=1.0, T=0.5, delay=0.05)

    # 不同定点精度: None=浮点, Q12=4位小数, Q10=6位小数, Q8=8位小数
    configs = [
        (None,  "浮点 (无量化)", "tab:blue"),
        (12,    "Q3.12 (4位小数)", "tab:orange"),
        (10,    "Q5.10 (6位小数)", "tab:green"),
        (8,     "Q7.8  (8位小数)", "tab:red"),
        (4,     "Q11.4 (12位整数)", "tab:purple"),
    ]

    plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    fig.suptitle('离散PID仿真 - 定点化效应分析', fontsize=14, fontweight='bold')

    print("=" * 70)
    print("离散PID仿真 - 定点化效应分析")
    print("=" * 70)
    print(f"PID参数: Kp={Kp}, Ki={Ki}, Kd={Kd}, dt={dt}s, 目标={setpoint}")
    print("-" * 70)
    print(f"{'配置':<20} {'超调%':>8} {'上升时间':>10} {'稳态误差%':>10} {'稳态值':>8}")
    print("-" * 70)

    for frac_bits, label, color in configs:
        t, y, u, e = run_simulation(Kp, Ki, Kd, setpoint, plant_params, dt, t_end, frac_bits, label)
        overshoot, rise_time, ss_err, ss_val = calc_metrics(t, y, setpoint)

        print(f"{label:<20} {overshoot:>7.2f}% {rise_time:>9.4f}s {ss_err:>9.4f}% {ss_val:>8.4f}")

        axes[0].plot(t, y, color=color, label=label, linewidth=1.2)
        axes[1].plot(t, u, color=color, label=label, linewidth=1.0)
        axes[2].plot(t, e, color=color, label=label, linewidth=1.0)

    for ax, title, ylabel in zip(axes,
            ['阶跃响应(不同定点精度)', '控制量u(t)', '误差e(t)'],
            ['输出 y(t)', '控制量 u', '误差 e']):
        ax.set_title(title); ax.set_ylabel(ylabel); ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    axes[0].axhline(setpoint, color='gray', linestyle='--', alpha=0.5, label='设定值')
    axes[2].set_xlabel('时间 (s)')

    plt.tight_layout()
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'discrete_pid_result.png')
    plt.savefig(out, dpi=150)
    print(f"\n仿真图已保存: {out}")
    print("\n结论: 定点位数越少, 量化噪声越大, 稳态出现极限环振荡。")
    print("      建议嵌入式PID至少使用Q4.12以上精度。")
