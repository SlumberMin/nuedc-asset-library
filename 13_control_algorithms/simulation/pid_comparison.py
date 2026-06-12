#!/usr/bin/env python3
"""
PID变种对比仿真
===============
对比5种PID控制算法在不同被控对象下的性能：
1. 位置式PID (Position PID)
2. 增量式PID (Incremental PID)
3. 模糊PID (Fuzzy PID)
4. 自适应PID (Adaptive PID)
5. 神经网络PID (Neural PID)

被控对象：一阶惯性 + 纯滞后 + 非线性

运行: python pid_comparison.py
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import rcParams

# 支持中文显示
rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
rcParams['axes.unicode_minus'] = False

# ============================================================
# 被控对象模型
# ============================================================

class Plant:
    """一阶惯性 + 纯滞后 + 可选非线性"""
    def __init__(self, K=1.0, T=1.0, delay=0.0, dt=0.001):
        self.K = K
        self.T = T
        self.dt = dt
        self.delay_steps = int(delay / dt)
        self.delay_buffer = [0.0] * max(self.delay_steps + 1, 1)
        self.state = 0.0

    def update(self, u):
        # 一阶惯性: y' = (K*u - y) / T
        self.state += (self.K * u - self.state) / self.T * self.dt
        # 纯滞后
        self.delay_buffer.append(self.state)
        if len(self.delay_buffer) > self.delay_steps + 1:
            self.delay_buffer.pop(0)
        return self.delay_buffer[0]

    def reset(self):
        self.state = 0.0
        self.delay_buffer = [0.0] * max(self.delay_steps + 1, 1)


class NonlinearPlant:
    """含非线性的一阶系统（死区+饱和）"""
    def __init__(self, K=1.0, T=1.0, delay=0.0, dt=0.001):
        self.K = K
        self.T = T
        self.dt = dt
        self.delay_steps = int(delay / dt)
        self.delay_buffer = [0.0] * max(self.delay_steps + 1, 1)
        self.state = 0.0

    def update(self, u):
        # 死区
        if abs(u) < 0.5:
            u_nl = 0.0
        else:
            u_nl = u - 0.5 * np.sign(u)
        # 饱和
        u_nl = np.clip(u_nl, -10, 10)
        self.state += (self.K * u_nl - self.state) / self.T * self.dt
        self.delay_buffer.append(self.state)
        if len(self.delay_buffer) > self.delay_steps + 1:
            self.delay_buffer.pop(0)
        return self.delay_buffer[0]

    def reset(self):
        self.state = 0.0
        self.delay_buffer = [0.0] * max(self.delay_steps + 1, 1)


# ============================================================
# PID控制器实现
# ============================================================

class PositionPID:
    """位置式PID"""
    def __init__(self, Kp, Ki, Kd, dt, out_min=-10, out_max=10):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.dt = dt
        self.out_min, self.out_max = out_min, out_max
        self.integral = 0.0
        self.error_last = 0.0
        self.name = "位置式PID"

    def compute(self, target, measurement):
        error = target - measurement
        self.integral += error * self.dt
        self.integral = np.clip(self.integral, -100, 100)
        derivative = (error - self.error_last) / self.dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        output = np.clip(output, self.out_min, self.out_max)
        self.error_last = error
        return output

    def reset(self):
        self.integral = 0.0
        self.error_last = 0.0


class IncrementalPID:
    """增量式PID"""
    def __init__(self, Kp, Ki, Kd, dt, out_min=-10, out_max=10):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.dt = dt
        self.out_min, self.out_max = out_min, out_max
        self.error_last = 0.0
        self.error_prev = 0.0
        self.output = 0.0
        self.name = "增量式PID"

    def compute(self, target, measurement):
        error = target - measurement
        delta = self.Kp * (error - self.error_last) \
              + self.Ki * error * self.dt \
              + self.Kd * (error - 2 * self.error_last + self.error_prev) / self.dt
        self.output += delta
        self.output = np.clip(self.output, self.out_min, self.out_max)
        self.error_prev = self.error_last
        self.error_last = error
        return self.output

    def reset(self):
        self.error_last = 0.0
        self.error_prev = 0.0
        self.output = 0.0


class FuzzyPID:
    """模糊PID（简化版：根据误差和误差变化率在线微调Kp/Ki/Kd）"""
    def __init__(self, Kp, Ki, Kd, dt, out_min=-10, out_max=10):
        self.Kp0, self.Ki0, self.Kd0 = Kp, Ki, Kd
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.dt = dt
        self.out_min, self.out_max = out_min, out_max
        self.integral = 0.0
        self.error_last = 0.0
        self.name = "模糊PID"

    def _fuzzy_adjust(self, error, delta_error):
        """简化模糊规则：根据|e|和|de|调整参数"""
        ae, ade = abs(error), abs(delta_error)

        # 误差大 -> 加大Kp，减小Kd
        if ae > 2.0:
            self.Kp = self.Kp0 * 1.5
            self.Ki = self.Ki0 * 0.5
            self.Kd = self.Kd0 * 0.8
        # 误差中等 + 变化快 -> 加大Kd
        elif ae > 0.5 and ade > 0.1:
            self.Kp = self.Kp0 * 1.2
            self.Ki = self.Ki0 * 0.8
            self.Kd = self.Kd0 * 1.5
        # 误差小 -> 加大Ki消除稳态误差
        elif ae < 0.1:
            self.Kp = self.Kp0 * 0.8
            self.Ki = self.Ki0 * 1.5
            self.Kd = self.Kd0 * 0.5
        else:
            self.Kp = self.Kp0
            self.Ki = self.Ki0
            self.Kd = self.Kd0

    def compute(self, target, measurement):
        error = target - measurement
        delta_error = (error - self.error_last) / self.dt
        self._fuzzy_adjust(error, delta_error)

        self.integral += error * self.dt
        self.integral = np.clip(self.integral, -100, 100)
        derivative = delta_error
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        output = np.clip(output, self.out_min, self.out_max)
        self.error_last = error
        return output

    def reset(self):
        self.integral = 0.0
        self.error_last = 0.0
        self.Kp, self.Ki, self.Kd = self.Kp0, self.Ki0, self.Kd0


class AdaptivePID:
    """自适应PID（梯度下降法在线调整参数）"""
    def __init__(self, Kp, Ki, Kd, dt, out_min=-10, out_max=10):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.dt = dt
        self.out_min, self.out_max = out_min, out_max
        self.integral = 0.0
        self.error_last = 0.0
        self.lr_p, self.lr_i, self.lr_d = 0.01, 0.005, 0.002
        self.name = "自适应PID"

    def compute(self, target, measurement):
        error = target - measurement
        self.integral += error * self.dt
        self.integral = np.clip(self.integral, -100, 100)
        derivative = (error - self.error_last) / self.dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        output = np.clip(output, self.out_min, self.out_max)

        # 梯度下降调整参数
        if abs(error) > 0.01:
            self.Kp -= self.lr_p * error * error
            self.Ki -= self.lr_i * error * self.integral
            self.Kd -= self.lr_d * error * derivative
            self.Kp = np.clip(self.Kp, 0.01, 20)
            self.Ki = np.clip(self.Ki, 0.0, 10)
            self.Kd = np.clip(self.Kd, 0.0, 5)

        self.error_last = error
        return output

    def reset(self):
        self.integral = 0.0
        self.error_last = 0.0


class NeuralPID:
    """神经元PID（单神经元自适应权重）"""
    def __init__(self, dt, out_min=-10, out_max=10):
        self.w = np.array([0.5, 0.2, 0.1])  # 归一化权重
        self.w_raw = np.array([5.0, 2.0, 1.0])
        self.lr = np.array([0.3, 0.15, 0.05])
        self.dt = dt
        self.out_min, self.out_max = out_min, out_max
        self.integral = 0.0
        self.error_last = 0.0
        self.name = "神经网络PID"

    def compute(self, target, measurement):
        error = target - measurement
        self.integral += error * self.dt
        self.integral = np.clip(self.integral, -100, 100)
        derivative = (error - self.error_last) / self.dt

        x = np.array([error, self.integral, derivative])
        output = np.dot(self.w, x)
        output = np.clip(output, self.out_min, self.out_max)

        # 权重学习（改进型）
        sign = 1.0 if output * error > 0 else -1.0
        factor = 1.0 + 0.5 * sign
        self.w_raw += self.lr * error * x * factor
        self.w_raw = np.clip(self.w_raw, 0, 100)

        # 归一化
        s = np.sum(np.abs(self.w_raw))
        if s > 1e-6:
            self.w = self.w_raw / s

        self.error_last = error
        return output

    def reset(self):
        self.integral = 0.0
        self.error_last = 0.0
        self.w_raw = np.array([5.0, 2.0, 1.0])
        self.w = self.w_raw / np.sum(np.abs(self.w_raw))


# ============================================================
# 仿真函数
# ============================================================

def simulate(controller, plant, target_func, duration, dt):
    """运行一次仿真"""
    steps = int(duration / dt)
    t = np.arange(steps) * dt
    y_out = np.zeros(steps)
    u_out = np.zeros(steps)

    controller.reset()
    plant.reset()

    for i in range(steps):
        r = target_func(t[i])
        y = y_out[i - 1] if i > 0 else 0.0
        u = controller.compute(r, y)
        y_new = plant.update(u)
        y_out[i] = y_new
        u_out[i] = u

    return t, y_out, u_out


def calc_metrics(t, y, target_func):
    """计算性能指标"""
    target = np.array([target_func(ti) for ti in t])
    error = target - y

    # 上升时间（10%->90%）
    y_final = target[-1] if abs(target[-1]) > 0.01 else 1.0
    try:
        t_10 = t[np.where(np.abs(y) >= 0.1 * abs(y_final))[0][0]]
        t_90 = t[np.where(np.abs(y) >= 0.9 * abs(y_final))[0][0]]
        rise_time = t_90 - t_10
    except IndexError:
        rise_time = float('inf')

    # 超调量
    overshoot = 0.0
    if abs(y_final) > 0.01:
        peak = np.max(np.abs(y))
        overshoot = max(0, (peak - abs(y_final)) / abs(y_final) * 100)

    # 稳态误差（最后10%时间的平均误差）
    n = len(error)
    ss_error = np.mean(np.abs(error[int(0.9*n):]))

    # IAE（误差绝对值积分）
    iae = np.sum(np.abs(error)) * (t[1] - t[0])

    return {
        'rise_time': rise_time,
        'overshoot': overshoot,
        'ss_error': ss_error,
        'iae': iae
    }


# ============================================================
# 主程序
# ============================================================

def main():
    dt = 0.001
    duration = 5.0

    # 被控对象：K=2, T=0.5, 延迟=0.2s
    plant = Plant(K=2.0, T=0.5, delay=0.2, dt=dt)
    nl_plant = NonlinearPlant(K=2.0, T=0.5, delay=0.1, dt=dt)

    # 目标信号：阶跃
    def target_step(t):
        return 1.0 if t >= 0.5 else 0.0

    # 目标信号：正弦跟踪
    def target_sine(t):
        return np.sin(2 * np.pi * 0.5 * t) if t >= 0.5 else 0.0

    # 创建控制器
    controllers = [
        PositionPID(Kp=2.0, Ki=1.0, Kd=0.5, dt=dt),
        IncrementalPID(Kp=2.0, Ki=1.0, Kd=0.5, dt=dt),
        FuzzyPID(Kp=2.0, Ki=1.0, Kd=0.5, dt=dt),
        AdaptivePID(Kp=2.0, Ki=1.0, Kd=0.5, dt=dt),
        NeuralPID(dt=dt),
    ]

    colors = ['#2196F3', '#FF5722', '#4CAF50', '#9C27B0', '#FF9800']
    markers = ['-', '--', '-.', ':', '-']

    # ========== 图1: 阶跃响应对比 ==========
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('PID变种算法对比仿真', fontsize=16, fontweight='bold')

    # 阶跃响应
    ax = axes[0, 0]
    all_metrics = {}
    for ctrl, color, ls in zip(controllers, colors, markers):
        t, y, u = simulate(ctrl, Plant(K=2.0, T=0.5, delay=0.2, dt=dt), target_step, duration, dt)
        ax.plot(t, y, color=color, linestyle=ls, label=ctrl.name, linewidth=1.5)
        all_metrics[ctrl.name] = calc_metrics(t, y, target_step)
    target_line = np.array([target_step(ti) for ti in t])
    ax.plot(t, target_line, 'k--', label='目标', linewidth=1, alpha=0.5)
    ax.set_title('阶跃响应对比 (带纯滞后)')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('输出')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # 正弦跟踪
    ax = axes[0, 1]
    for ctrl, color, ls in zip(controllers, colors, markers):
        ctrl.reset()
        t, y, u = simulate(ctrl, Plant(K=2.0, T=0.5, delay=0.1, dt=dt), target_sine, duration, dt)
        ax.plot(t, y, color=color, linestyle=ls, label=ctrl.name, linewidth=1.5)
    target_line = np.array([target_sine(ti) for ti in t])
    ax.plot(t, target_line, 'k--', label='目标', linewidth=1, alpha=0.5)
    ax.set_title('正弦信号跟踪')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('输出')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # 控制量对比
    ax = axes[1, 0]
    for ctrl, color, ls in zip(controllers, colors, markers):
        ctrl.reset()
        t, y, u = simulate(ctrl, Plant(K=2.0, T=0.5, delay=0.2, dt=dt), target_step, duration, dt)
        ax.plot(t, u, color=color, linestyle=ls, label=ctrl.name, linewidth=1)
    ax.set_title('控制量对比')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('控制量 u')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # 非线性系统响应
    ax = axes[1, 1]
    for ctrl, color, ls in zip(controllers, colors, markers):
        ctrl.reset()
        t, y, u = simulate(ctrl, NonlinearPlant(K=2.0, T=0.5, delay=0.1, dt=dt), target_step, duration, dt)
        ax.plot(t, y, color=color, linestyle=ls, label=ctrl.name, linewidth=1.5)
    target_line = np.array([target_step(ti) for ti in t])
    ax.plot(t, target_line, 'k--', label='目标', linewidth=1, alpha=0.5)
    ax.set_title('非线性系统响应 (死区+饱和)')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('输出')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('pid_comparison.png', dpi=150, bbox_inches='tight')
    print("图片已保存: pid_comparison.png")

    # ========== 性能指标表 ==========
    print("\n" + "=" * 70)
    print("阶跃响应性能指标")
    print("=" * 70)
    print(f"{'算法':<12} {'上升时间(s)':<12} {'超调量(%)':<12} {'稳态误差':<12} {'IAE':<10}")
    print("-" * 70)
    for name, m in all_metrics.items():
        rt = f"{m['rise_time']:.4f}" if m['rise_time'] != float('inf') else "N/A"
        print(f"{name:<12} {rt:<12} {m['overshoot']:<12.2f} {m['ss_error']:<12.4f} {m['iae']:<10.4f}")

    # ========== 图2: 参数演化（自适应/神经PID） ==========
    fig2, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4))

    # 自适应PID参数变化
    ctrl = AdaptivePID(Kp=2.0, Ki=1.0, Kd=0.5, dt=dt)
    p_record = []
    i_record = []
    d_record = []
    plant_sim = Plant(K=2.0, T=0.5, delay=0.2, dt=dt)
    plant_sim.reset()
    for i in range(int(duration / dt)):
        t_val = i * dt
        r = target_step(t_val)
        y = plant_sim.state
        u = ctrl.compute(r, y)
        plant_sim.update(u)
        p_record.append(ctrl.Kp)
        i_record.append(ctrl.Ki)
        d_record.append(ctrl.Kd)

    t_arr = np.arange(len(p_record)) * dt
    ax1.plot(t_arr, p_record, label='Kp', color='#2196F3')
    ax1.plot(t_arr, i_record, label='Ki', color='#FF5722')
    ax1.plot(t_arr, d_record, label='Kd', color='#4CAF50')
    ax1.set_title('自适应PID - 参数演化')
    ax1.set_xlabel('时间 (s)')
    ax1.set_ylabel('参数值')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # 神经PID权重变化
    ctrl2 = NeuralPID(dt=dt)
    w_record = []
    plant_sim2 = Plant(K=2.0, T=0.5, delay=0.2, dt=dt)
    plant_sim2.reset()
    for i in range(int(duration / dt)):
        t_val = i * dt
        r = target_step(t_val)
        y = plant_sim2.state
        u = ctrl2.compute(r, y)
        plant_sim2.update(u)
        w_record.append(ctrl2.w.copy())

    w_arr = np.array(w_record)
    ax2.plot(t_arr, w_arr[:, 0], label='w1(P)', color='#2196F3')
    ax2.plot(t_arr, w_arr[:, 1], label='w2(I)', color='#FF5722')
    ax2.plot(t_arr, w_arr[:, 2], label='w3(D)', color='#4CAF50')
    ax2.set_title('神经网络PID - 权重演化')
    ax2.set_xlabel('时间 (s)')
    ax2.set_ylabel('归一化权重')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('pid_params_evolution.png', dpi=150, bbox_inches='tight')
    print("参数演化图已保存: pid_params_evolution.png")
    plt.close('all')


if __name__ == '__main__':
    main()
