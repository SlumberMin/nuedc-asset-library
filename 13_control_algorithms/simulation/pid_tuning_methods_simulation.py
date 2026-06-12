#!/usr/bin/env python3
"""
PID调参方法仿真对比

演示常见PID整定方法的效果:
  1. Ziegler-Nichols 临界比例度法
  2. Ziegler-Nichols 阶跃响应法
  3. Cohen-Coon 法
  4. SIMC (Skogestad IMC) 法
  5. 手动调参（工程经验值）

被控对象: G(s) = K * e^(-L*s) / (tau*s + 1)
  一阶惯性+纯滞后系统
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# ============================================================
# 被控对象: 一阶惯性+纯滞后
# ============================================================
class FOPDT_Plant:
    """First Order Plus Dead Time"""

    def __init__(self, K=1.0, tau=1.0, L=0.2, dt=0.01):
        self.K = K      # 增益
        self.tau = tau   # 时间常数
        self.L = L       # 纯滞后
        self.dt = dt
        self.y = 0.0
        self._delay_steps = int(L / dt)
        self._u_buffer = [0.0] * max(self._delay_steps + 1, 1)

    def update(self, u):
        self._u_buffer.append(u)
        if len(self._u_buffer) > self._delay_steps + 1:
            self._u_buffer.pop(0)
        u_delayed = self._u_buffer[0]
        # 一阶惯性: tau * dy/dt + y = K * u
        self.y += (self.K * u_delayed - self.y) / self.tau * self.dt
        return self.y

    def reset(self):
        self.y = 0.0
        self._u_buffer = [0.0] * max(self._delay_steps + 1, 1)


# ============================================================
# PID控制器(带微分滤波和抗饱和)
# ============================================================
class PID:
    def __init__(self, kp, ki, kd, out_min=-100, out_max=100, dt=0.01):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self.dt = dt
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_d = 0.0
        self.d_filter = 0.1  # 微分滤波

    def update(self, sp, fb):
        err = sp - fb
        self.integral += err * self.dt
        raw_d = (err - self.prev_error) / self.dt
        filt_d = self.d_filter * raw_d + (1 - self.d_filter) * self.prev_d
        self.prev_d = filt_d
        self.prev_error = err
        out = self.kp * (err + self.ki * self.integral + self.kd * filt_d)
        # anti-windup
        saturated = False
        if out > self.out_max:
            out = self.out_max; saturated = True
        elif out < self.out_min:
            out = self.out_min; saturated = True
        if saturated:
            self.integral -= err * self.dt
        return out

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_d = 0.0


# ============================================================
# 整定公式
# ============================================================
def zn_critical_period(Ku, Tu):
    """Ziegler-Nichols 临界比例度法
    Ku: 临界增益, Tu: 临界振荡周期"""
    kp = 0.6 * Ku
    ki = 2.0 * kp / Tu   # Ti = Tu/2
    kd = kp * Tu / 8.0   # Td = Tu/8
    return kp, ki, kd

def zn_step_response(K, tau, L):
    """Ziegler-Nichols 阶跃响应法 (S-shaped曲线法)
    K: 过程增益, tau: 时间常数, L: 纯滞后"""
    a = K * L / tau  # 阶跃响应斜率比
    kp = 1.2 / a
    Ti = 2.0 * L
    Td = 0.5 * L
    ki = kp / Ti
    kd = kp * Td
    return kp, ki, kd

def cohen_coon(K, tau, L):
    """Cohen-Coon 整定法"""
    r = L / tau  # 滞后比
    kp = (1.0 / (K * r)) * (1.0 + r / 3.0)
    Ti = L * (30.0 + 3.0 * r) / (9.0 + 20.0 * r)
    Td = L * 4.0 / (11.0 + 2.0 * r)
    ki = kp / Ti
    kd = kp * Td
    return kp, ki, kd

def simc_tuning(K, tau, L):
    """SIMC (Skogestad IMC) 法
    简单实用，鲁棒性好"""
    tau_c = max(L, 0.1 * tau)  # 期望闭环时间常数(取滞后或tau的10%)
    kp = tau / (K * (tau_c + L))
    Ti = min(tau, 4.0 * (tau_c + L))  # 积分时间
    Td = 0.0  # SIMC通常不用微分
    ki = kp / Ti
    kd = kp * Td
    return kp, ki, kd

def manual_tuning(K, tau, L):
    """手动工程调参经验法
    先P后I再D，逐步调整"""
    # 经验起点: Kp = tau / (K * L)
    kp = 0.8 * tau / (K * max(L, 0.01))
    Ti = 2.0 * tau  # 积分时间取2倍时间常数
    Td = 0.25 * L   # 微分取0.25倍滞后
    ki = kp / Ti
    kd = kp * Td
    return kp, ki, kd


# ============================================================
# 仿真函数
# ============================================================
def simulate_pid(pid, plant, setpoint, dt, steps):
    y = np.zeros(steps)
    u = np.zeros(steps)
    for i in range(steps):
        y[i] = plant.update(u[i])
        u[i] = pid.update(setpoint[i], y[i])
    return y, u


def calc_metrics(y, sp, dt, settle_threshold=0.02):
    """计算性能指标"""
    final_val = sp[-1]
    if final_val == 0:
        return {}, y

    # 超调量
    peak = np.max(y)
    overshoot = max(0, (peak - final_val) / final_val * 100)

    # IAE (绝对误差积分)
    iae = np.sum(np.abs(sp - y)) * dt

    # ITAE (时间加权绝对误差积分)
    t = np.arange(len(y)) * dt
    itae = np.sum(t * np.abs(sp - y)) * dt

    # 调节时间 (进入±2%带的时间)
    settling_time = len(y) * dt
    for i in range(len(y) - 1, 0, -1):
        if abs(y[i] - final_val) > settle_threshold * final_val:
            settling_time = (i + 1) * dt
            break

    # 上升时间 (10%~90%)
    rise_start = None
    rise_end = None
    for i in range(len(y)):
        if y[i] >= 0.1 * final_val and rise_start is None:
            rise_start = i * dt
        if y[i] >= 0.9 * final_val and rise_end is None:
            rise_end = i * dt
            break
    rise_time = (rise_end - rise_start) if (rise_start and rise_end) else float('inf')

    return {
        'overshoot': overshoot,
        'iae': iae,
        'itae': itae,
        'settling_time': settling_time,
        'rise_time': rise_time,
    }, y


# ============================================================
# 主程序
# ============================================================
def main():
    dt = 0.01
    T = 15.0
    steps = int(T / dt)
    t = np.linspace(0, T, steps)

    # 被控对象参数
    K, tau, L = 2.0, 3.0, 0.5

    # 阶跃设定值
    setpoint = np.zeros(steps)
    setpoint[int(1.0 / dt):] = 1.0

    # 整定方法
    methods = {
        'Z-N临界比例度': zn_critical_period(Ku=4.5, Tu=2.8),  # 模拟临界振荡实验
        'Z-N阶跃响应':  zn_step_response(K, tau, L),
        'Cohen-Coon':   cohen_coon(K, tau, L),
        'SIMC':         simc_tuning(K, tau, L),
        '手动调参':      manual_tuning(K, tau, L),
    }

    # 统一的工程经验值(带微分滤波)
    manual_override = (5.0, 0.8, 0.5)  # 手动精调结果

    results = {}
    all_y = {}
    colors = ['b', 'r', 'g', 'm', 'orange']

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    for idx, (name, (kp, ki, kd)) in enumerate(methods.items()):
        plant = FOPDT_Plant(K=K, tau=tau, L=L, dt=dt)
        pid = PID(kp=kp, ki=ki, kd=kd, dt=dt, out_min=-100, out_max=100)
        y, u = simulate_pid(pid, plant, setpoint, dt, steps)
        metrics, y = calc_metrics(y, setpoint, dt)
        results[name] = metrics
        all_y[name] = y

        c = colors[idx % len(colors)]
        axes[0, 0].plot(t, y, color=c, lw=1.3, label=f'{name}')

    # 设定值
    axes[0, 0].plot(t, setpoint, 'k--', lw=2, label='设定值')
    axes[0, 0].set_title('阶跃响应对比')
    axes[0, 0].set_ylabel('输出')
    axes[0, 0].legend(fontsize=8)
    axes[0, 0].grid(True, alpha=0.3)

    # 超调量柱状图
    names = list(results.keys())
    overshoots = [results[n]['overshoot'] for n in names]
    axes[0, 1].bar(range(len(names)), overshoots, color=colors[:len(names)])
    axes[0, 1].set_xticks(range(len(names)))
    axes[0, 1].set_xticklabels(names, rotation=30, ha='right', fontsize=8)
    axes[0, 1].set_ylabel('超调量 (%)')
    axes[0, 1].set_title('超调量对比')
    axes[0, 1].grid(True, alpha=0.3, axis='y')

    # IAE/ITAE
    iaes = [results[n]['iae'] for n in names]
    itaes = [results[n]['itae'] for n in names]
    x_pos = np.arange(len(names))
    w = 0.35
    axes[1, 0].bar(x_pos - w/2, iaes, w, label='IAE', color='steelblue')
    axes[1, 0].bar(x_pos + w/2, itaes, w, label='ITAE', color='coral')
    axes[1, 0].set_xticks(x_pos)
    axes[1, 0].set_xticklabels(names, rotation=30, ha='right', fontsize=8)
    axes[1, 0].set_title('误差积分指标')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3, axis='y')

    # 调节时间
    settlings = [results[n]['settling_time'] for n in names]
    axes[1, 1].bar(range(len(names)), settlings, color=colors[:len(names)])
    axes[1, 1].set_xticks(range(len(names)))
    axes[1, 1].set_xticklabels(names, rotation=30, ha='right', fontsize=8)
    axes[1, 1].set_ylabel('调节时间 (s)')
    axes[1, 1].set_title('调节时间对比 (±2%)')
    axes[1, 1].grid(True, alpha=0.3, axis='y')

    plt.suptitle(f'PID调参方法仿真对比\n被控对象: K={K}, τ={tau}, L={L}', fontsize=13)
    plt.tight_layout()
    plt.savefig('pid_tuning_comparison.png', dpi=150)
    print("仿真完成，结果已保存为 pid_tuning_comparison.png")

    # 打印指标表格
    print(f"\n{'方法':<16} {'超调%':>8} {'IAE':>10} {'ITAE':>10} {'调节时间s':>10} {'上升时间s':>10}")
    print("-" * 70)
    for n in names:
        m = results[n]
        print(f"{n:<16} {m['overshoot']:>7.1f}% {m['iae']:>10.3f} {m['itae']:>10.3f} "
              f"{m['settling_time']:>10.2f} {m['rise_time']:>10.2f}")

    print("\n推荐:")
    print("  - 一般场合: SIMC法(鲁棒性好, 参数保守)")
    print("  - 快速响应: Z-N法或Cohen-Coon(需微调)")
    print("  - 电赛实战: 先用SIMC出初始值,再手动微调")


if __name__ == '__main__':
    main()
