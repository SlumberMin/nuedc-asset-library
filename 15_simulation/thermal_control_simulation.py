#!/usr/bin/env python3
"""
温度控制仿真 - PID / 模糊PID / Smith预估器 对比
====================================================
- 经典PID温控
- 模糊PID自整定温控
- Smith预估器补偿纯滞后
- 阶跃响应 / 扰动抑制 / 鲁棒性对比
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rcParams
rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
rcParams['axes.unicode_minus'] = False

# ============================================================
# 1. 被控对象：带纯滞后的一阶惯性环节
#    G(s) = K / (Ts+1) * e^(-Ls)
# ============================================================
class ThermalPlant:
    """温度被控对象"""
    def __init__(self, K=1.0, T=10.0, L=2.0, dt=0.1):
        self.K = K       # 增益
        self.T = T       # 时间常数
        self.L = L       # 纯滞后
        self.dt = dt
        self.y = 25.0    # 初始温度(室温)
        self.buffer = [25.0] * int(L / dt)  # 滞后缓冲
        self.d_state = 0.0  # 扰动

    def update(self, u):
        """u: 加热功率输入, 返回当前温度"""
        # 一阶惯性离散化
        alpha = self.dt / (self.T + self.dt)
        self.y += alpha * (self.K * self._delayed_u(u) - self.y) + self.d_state * self.dt
        return self.y

    def _delayed_u(self, u):
        self.buffer.append(u)
        return self.buffer.pop(0)

    def add_disturbance(self, d):
        self.d_state = d

    def reset(self, T_init=25.0):
        self.y = T_init
        self.buffer = [T_init] * int(self.L / self.dt)
        self.d_state = 0.0


# ============================================================
# 2. 经典PID控制器
# ============================================================
class PIDController:
    def __init__(self, Kp, Ki, Kd, dt, out_min=0, out_max=100):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.dt = dt
        self.out_min, self.out_max = out_min, out_max
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, setpoint, measurement):
        error = setpoint - measurement
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error
        u = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        # 抗积分饱和
        if u > self.out_max:
            u = self.out_max
            self.integral -= error * self.dt
        elif u < self.out_min:
            u = self.out_min
            self.integral -= error * self.dt
        return u

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


# ============================================================
# 3. 模糊PID控制器
# ============================================================
class FuzzyPIDController:
    """
    模糊自整定PID：根据e和ec实时调整Kp, Ki, Kd
    模糊规则：7x7规则表（NB/NM/NS/ZO/PS/PM/PB）
    """
    # 模糊隶属度中心
    _NL, _NM, _NS, _ZO, _PS, _PM, _PB = -3, -2, -1, 0, 1, 2, 3

    # ΔKp 规则表（7x7）
    KP_RULES = [
        [ 3,  3,  2,  2,  1,  0,  0],
        [ 3,  3,  2,  1,  1,  0, -1],
        [ 2,  2,  1,  1,  0, -1, -1],
        [ 2,  1,  1,  0, -1, -1, -2],
        [ 1,  1,  0, -1, -1, -2, -2],
        [ 0,  0, -1, -1, -2, -2, -3],
        [ 0, -1, -1, -2, -2, -3, -3],
    ]
    # ΔKi 规则表
    KI_RULES = [
        [-3, -3, -2, -2, -1,  0,  0],
        [-3, -3, -2, -1, -1,  0,  0],
        [-2, -2, -1, -1,  0,  1,  1],
        [-2, -1, -1,  0,  1,  1,  2],
        [-1, -1,  0,  1,  1,  2,  2],
        [ 0,  0,  1,  1,  2,  3,  3],
        [ 0,  0,  1,  2,  2,  3,  3],
    ]
    # ΔKd 规则表
    KD_RULES = [
        [ 1,  1,  0, -1, -1, -2, -3],
        [ 1,  1,  0, -1, -1, -2, -3],
        [ 0,  0, -1, -1, -2, -2, -2],
        [ 0, -1, -1, -2, -2, -2, -2],
        [-1, -1, -2, -2, -2, -2, -1],
        [-2, -2, -2, -2, -1, -1,  0],
        [-3, -3, -2, -1, -1,  0,  0],
    ]

    def __init__(self, Kp0, Ki0, Kd0, dt, e_range=50, ec_range=10,
                 dKp=0.5, dKi=0.1, dKd=0.5, out_min=0, out_max=100):
        self.Kp0, self.Ki0, self.Kd0 = Kp0, Ki0, Kd0
        self.dKp, self.dKi, self.dKd = dKp, dKi, dKd
        self.e_range, self.ec_range = e_range, ec_range
        self.dt = dt
        self.out_min, self.out_max = out_min, out_max
        self.integral = 0.0
        self.prev_error = 0.0

    def _fuzzify(self, value, range_val):
        """将连续值映射到[-3,3]区间"""
        v = np.clip(value / range_val * 3, -3, 3)
        return v

    def _membership(self, v, center):
        """三角隶属度"""
        return max(0, 1 - abs(v - center))

    def _fuzzy_infer(self, e_fuzzy, ec_fuzzy, rules):
        """加权平均去模糊化"""
        numerator, denominator = 0.0, 0.0
        for i in range(7):
            for j in range(7):
                w = self._membership(e_fuzzy, i - 3) * self._membership(ec_fuzzy, j - 3)
                if w > 0:
                    numerator += w * rules[i][j]
                    denominator += w
        return numerator / denominator if denominator > 0 else 0

    def compute(self, setpoint, measurement):
        error = setpoint - measurement
        ec = (error - self.prev_error) / self.dt
        self.prev_error = error

        e_f = self._fuzzify(error, self.e_range)
        ec_f = self._fuzzify(ec, self.ec_range)

        dKp = self._fuzzy_infer(e_f, ec_f, self.KP_RULES) * self.dKp
        dKi = self._fuzzy_infer(e_f, ec_f, self.KI_RULES) * self.dKi
        dKd = self._fuzzy_infer(e_f, ec_f, self.KD_RULES) * self.dKd

        Kp = max(0, self.Kp0 + dKp)
        Ki = max(0, self.Ki0 + dKi)
        Kd = max(0, self.Kd0 + dKd)

        self.integral += error * self.dt
        derivative = ec
        u = Kp * error + Ki * self.integral + Kd * derivative

        if u > self.out_max:
            u = self.out_max
            self.integral -= error * self.dt
        elif u < self.out_min:
            u = self.out_min
            self.integral -= error * self.dt
        return u

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


# ============================================================
# 4. Smith预估器
# ============================================================
class SmithPredictor:
    """
    Smith预估器：用模型预估消除纯滞后对控制的影响
    控制器内部使用无滞后模型的输出
    """
    def __init__(self, Kp, Ki, Kd, dt, K_model, T_model, L_model):
        self.pid = PIDController(Kp, Ki, Kd, dt, out_min=0, out_max=100)
        self.dt = dt
        # 无滞后模型
        self.K_m = K_model
        self.T_m = T_model
        self.y_model = 25.0
        # 有滞后模型
        self.L_m = L_model
        self.buffer = [25.0] * int(L_model / dt)
        self.y_delayed = 25.0

    def compute(self, setpoint, measurement):
        # Smith预估：用measurement - y_delayed + y_model 作为反馈
        # 这样控制器看到的是"无滞后"的响应
        feedback = measurement - self.y_delayed + self.y_model
        u = self.pid.compute(setpoint, feedback)
        # 更新模型
        alpha = self.dt / (self.T_m + self.dt)
        self.y_model += alpha * (self.K_m * u - self.y_model)
        # 更新滞后模型
        self.buffer.append(self.y_model)
        self.y_delayed = self.buffer.pop(0)
        return u

    def reset(self):
        self.pid.reset()
        self.y_model = 25.0
        self.buffer = [25.0] * int(self.L_m / self.dt)
        self.y_delayed = 25.0


# ============================================================
# 5. 仿真主函数
# ============================================================
def run_simulation(dt=0.1, t_end=200):
    t = np.arange(0, t_end, dt)
    n = len(t)
    setpoint = np.ones(n) * 60.0  # 目标温度60°C
    setpoint[:int(10/dt)] = 25.0  # 前10秒保持室温

    results = {}

    # --- 经典PID ---
    plant = ThermalPlant(K=1.0, T=10.0, L=2.0, dt=dt)
    pid = PIDController(Kp=8.0, Ki=0.3, Kd=5.0, dt=dt)
    y_pid = np.zeros(n)
    for i in range(n):
        # 在t=120s加入扰动（开门降温）
        if abs(t[i] - 120) < dt/2:
            plant.add_disturbance(-2.0)
        elif abs(t[i] - 130) < dt/2:
            plant.add_disturbance(0.0)
        u = pid.compute(setpoint[i], plant.y)
        y_pid[i] = plant.update(u)
    results['PID'] = y_pid

    # --- 模糊PID ---
    plant = ThermalPlant(K=1.0, T=10.0, L=2.0, dt=dt)
    fuzzy = FuzzyPIDController(Kp0=8.0, Ki0=0.3, Kd0=5.0, dt=dt,
                                e_range=40, ec_range=5, dKp=3, dKi=0.2, dKd=3)
    y_fuzzy = np.zeros(n)
    for i in range(n):
        if abs(t[i] - 120) < dt/2:
            plant.add_disturbance(-2.0)
        elif abs(t[i] - 130) < dt/2:
            plant.add_disturbance(0.0)
        u = fuzzy.compute(setpoint[i], plant.y)
        y_fuzzy[i] = plant.update(u)
    results['模糊PID'] = y_fuzzy

    # --- Smith预估器 ---
    plant = ThermalPlant(K=1.0, T=10.0, L=2.0, dt=dt)
    smith = SmithPredictor(Kp=8.0, Ki=0.3, Kd=5.0, dt=dt,
                            K_model=1.0, T_model=10.0, L_model=2.0)
    y_smith = np.zeros(n)
    for i in range(n):
        if abs(t[i] - 120) < dt/2:
            plant.add_disturbance(-2.0)
        elif abs(t[i] - 130) < dt/2:
            plant.add_disturbance(0.0)
        u = smith.compute(setpoint[i], plant.y)
        y_smith[i] = plant.update(u)
    results['Smith预估'] = y_smith

    return t, setpoint, results


# ============================================================
# 6. 性能指标计算
# ============================================================
def calc_metrics(t, y, sp):
    """计算超调量、调节时间、IAE"""
    sp_val = sp[-1]
    overshoot = (np.max(y) - sp_val) / (sp_val - 25.0) * 100 if sp_val != 25.0 else 0
    # 调节时间（±2%）
    settling_time = t[-1]
    for i in range(len(t)-1, 0, -1):
        if abs(y[i] - sp_val) > 0.02 * abs(sp_val - 25.0):
            settling_time = t[min(i+1, len(t)-1)]
            break
    iae = np.sum(np.abs(sp - y)) * (t[1] - t[0])
    return overshoot, settling_time, iae


# ============================================================
# 7. 可视化
# ============================================================
def plot_results(t, setpoint, results):
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('温度控制仿真 — PID vs 模糊PID vs Smith预估器', fontsize=15, fontweight='bold')

    # (a) 阶跃响应对比
    ax = axes[0, 0]
    ax.plot(t, setpoint, 'k--', lw=2, label='设定值')
    colors = ['#2196F3', '#4CAF50', '#FF5722']
    for (name, y), c in zip(results.items(), colors):
        ax.plot(t, y, color=c, lw=1.5, label=name)
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('温度 (°C)')
    ax.set_title('(a) 阶跃响应对比')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # (b) 误差曲线
    ax = axes[0, 1]
    for (name, y), c in zip(results.items(), colors):
        ax.plot(t, setpoint - y, color=c, lw=1, label=name)
    ax.axhline(0, color='k', ls='--', lw=0.5)
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('误差 (°C)')
    ax.set_title('(b) 跟踪误差')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # (c) 放大扰动区域
    ax = axes[1, 0]
    mask = (t >= 110) & (t <= 160)
    ax.plot(t[mask], setpoint[mask], 'k--', lw=2, label='设定值')
    for (name, y), c in zip(results.items(), colors):
        ax.plot(t[mask], y[mask], color=c, lw=1.5, label=name)
    ax.axvspan(120, 130, alpha=0.1, color='red', label='扰动区间')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('温度 (°C)')
    ax.set_title('(c) 扰动抑制对比')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # (d) 性能指标柱状图
    ax = axes[1, 1]
    metrics = {}
    for name, y in results.items():
        ovs, st, iae = calc_metrics(t, y, setpoint)
        metrics[name] = {'超调量(%)': ovs, '调节时间(s)': st, 'IAE': iae}

    x_pos = np.arange(len(results))
    width = 0.25
    for i, metric_name in enumerate(['超调量(%)', '调节时间(s)', 'IAE']):
        vals = [metrics[n][metric_name] for n in results]
        ax.bar(x_pos + i * width, vals, width, label=metric_name, alpha=0.8)
    ax.set_xticks(x_pos + width)
    ax.set_xticklabels(list(results.keys()))
    ax.set_title('(d) 性能指标对比')
    ax.legend()
    ax.grid(True, alpha=0.3, axis='y')

    plt.tight_layout()
    plt.savefig('thermal_control_comparison.png', dpi=150, bbox_inches='tight')
    plt.show()

    # 打印指标
    print('\n' + '='*60)
    print('温度控制性能指标对比')
    print('='*60)
    for name, m in metrics.items():
        print(f'\n【{name}】')
        for k, v in m.items():
            print(f'  {k}: {v:.2f}')


# ============================================================
if __name__ == '__main__':
    t, sp, results = run_simulation()
    plot_results(t, sp, results)
    print('\n温度控制仿真完成！')
