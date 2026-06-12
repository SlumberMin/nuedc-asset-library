#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
所有PID变种对比仿真
====================
在统一被控对象上对比全部PID变种:
1. 经典PID (基准)
2. 增量式PID
3. 抗积分饱和PID
4. 不完全微分PID
5. 微分先行PID
6. 串级PID
7. 模糊PID
8. 自适应PID (增益调度)
9. Smith预估PID

输出: 综合性能指标对比表 + 阶跃响应图 + 控制量图
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from collections import OrderedDict

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
plt.rcParams['axes.unicode_minus'] = False


# ============================================================
# 被控对象
# ============================================================

class Plant2:
    """二阶对象"""
    def __init__(self, a1=1.2, a0=0.8, b=1.0):
        self.a1, self.a0, self.b = a1, a0, b
        self.reset()

    def reset(self):
        self.x1, self.x2 = 0.0, 0.0

    def update(self, u, dt=0.01):
        dx2 = -self.a1 * self.x2 - self.a0 * self.x1 + self.b * u
        self.x2 += dx2 * dt
        self.x1 += self.x2 * dt
        return self.x1


class DelayPlant:
    """带纯滞后的二阶对象 (用于Smith预估)"""
    def __init__(self, a1=1.2, a0=0.8, b=1.0, delay=0.3):
        self.plant = Plant2(a1, a0, b)
        self.delay_steps = int(delay / 0.01)
        self.u_buf = [0.0] * (self.delay_steps + 1)
        self.y = 0.0

    def reset(self):
        self.plant.reset()
        self.u_buf = [0.0] * (self.delay_steps + 1)
        self.y = 0.0

    def update(self, u, dt=0.01):
        self.u_buf.append(u)
        u_delayed = self.u_buf.pop(0)
        self.y = self.plant.update(u_delayed, dt)
        return self.y


# ============================================================
# PID变种实现
# ============================================================

class PIDBase:
    def __init__(self, Kp, Ki, Kd, dt=0.01):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.dt = dt
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_u = 0.0

    def compute(self, error):
        self.integral += error * self.dt
        d_error = (error - self.prev_error) / self.dt
        u = self.Kp * error + self.Ki * self.integral + self.Kd * d_error
        self.prev_error = error
        self.prev_u = u
        return u


class ClassicPID(PIDBase):
    """1. 经典PID"""
    name = '经典PID'


class IncrementalPID(PIDBase):
    """2. 增量式PID"""
    name = '增量式PID'
    def __init__(self, Kp, Ki, Kd, dt=0.01):
        super().__init__(Kp, Ki, Kd, dt)
        self.prev2_error = 0.0

    def reset(self):
        super().reset()
        self.prev2_error = 0.0

    def compute(self, error):
        du = (self.Kp * (error - self.prev_error) +
              self.Ki * error * self.dt +
              self.Kd * (error - 2 * self.prev_error + self.prev2_error) / self.dt)
        u = self.prev_u + du
        self.prev2_error = self.prev_error
        self.prev_error = error
        self.prev_u = u
        return u


class AntiWindupPID(PIDBase):
    """3. 抗积分饱和PID"""
    name = '抗积分饱和PID'
    def __init__(self, Kp, Ki, Kd, dt=0.01, u_max=10, u_min=-10):
        super().__init__(Kp, Ki, Kd, dt)
        self.u_max, self.u_min = u_max, u_min

    def compute(self, error):
        self.integral += error * self.dt
        d_error = (error - self.prev_error) / self.dt
        u_unsat = self.Kp * error + self.Ki * self.integral + self.Kd * d_error
        u = np.clip(u_unsat, self.u_min, self.u_max)
        # 条件积分: 饱和时回退积分
        if u != u_unsat:
            self.integral -= error * self.dt * 0.5
        self.prev_error = error
        self.prev_u = u
        return u


class IncompleteDiffPID(PIDBase):
    """4. 不完全微分PID"""
    name = '不完全微分PID'
    def __init__(self, Kp, Ki, Kd, dt=0.01, alpha=0.1):
        super().__init__(Kp, Ki, Kd, dt)
        self.alpha = alpha
        self.d_prev = 0.0

    def reset(self):
        super().reset()
        self.d_prev = 0.0

    def compute(self, error):
        self.integral += error * self.dt
        d_raw = (error - self.prev_error) / self.dt
        # 一阶低通滤波微分项
        d_filtered = self.alpha * d_raw + (1 - self.alpha) * self.d_prev
        u = self.Kp * error + self.Ki * self.integral + self.Kd * d_filtered
        self.prev_error = error
        self.d_prev = d_filtered
        self.prev_u = u
        return u


class DerivativeFirstPID(PIDBase):
    """5. 微分先行PID"""
    name = '微分先行PID'
    def __init__(self, Kp, Ki, Kd, dt=0.01, beta=0.8):
        super().__init__(Kp, Ki, Kd, dt)
        self.beta = beta
        self.prev_y = 0.0

    def reset(self):
        super().reset()
        self.prev_y = 0.0

    def compute(self, error, y=None, r=None):
        self.integral += error * self.dt
        if y is not None:
            # 微分作用于输出而非误差
            d_term = -self.Kd * (y - self.prev_y) / self.dt
            self.prev_y = y
        else:
            d_term = self.Kd * (error - self.prev_error) / self.dt
        u = self.Kp * error + self.Ki * self.integral + d_term
        self.prev_error = error
        self.prev_u = u
        return u


class CascadePID:
    """6. 串级PID (主环+副环)"""
    name = '串级PID'
    def __init__(self, Kp1, Ki1, Kd1, Kp2, Ki2, Kd2, dt=0.01):
        self.outer = PIDBase(Kp1, Ki1, Kd1, dt)
        self.inner = PIDBase(Kp2, Ki2, Kd2, dt)
        self.dt = dt

    def reset(self):
        self.outer.reset()
        self.inner.reset()

    def compute(self, error, y_inner=0.0):
        # 外环输出作为内环设定值
        inner_ref = self.outer.compute(error)
        inner_error = inner_ref - y_inner
        u = self.inner.compute(inner_error)
        return u


class SimpleFuzzyPID(PIDBase):
    """7. 简化模糊PID"""
    name = '模糊PID'
    def __init__(self, Kp, Ki, Kd, dt=0.01):
        super().__init__(Kp, Ki, Kd, dt)
        self.Kp0, self.Ki0, self.Kd0 = Kp, Ki, Kd

    def compute(self, error):
        d_error = (error - self.prev_error) / self.dt
        ae = abs(error)
        # 简化模糊规则
        self.Kp = self.Kp0 * (1 + 0.8 * ae)
        self.Ki = self.Ki0 / (1 + 2 * ae)
        self.Kd = self.Kd0 * (1 + 0.5 * abs(d_error))
        self.Kp = np.clip(self.Kp, 0.1, 50)
        self.Ki = np.clip(self.Ki, 0.01, 20)
        self.Kd = np.clip(self.Kd, 0.01, 20)

        self.integral += error * self.dt
        self.integral = np.clip(self.integral, -10, 10)
        u = self.Kp * error + self.Ki * self.integral + self.Kd * d_error
        self.prev_error = error
        self.prev_u = u
        return u


class AdaptivePID(PIDBase):
    """8. 自适应PID (增益调度)"""
    name = '自适应PID'
    def __init__(self, Kp, Ki, Kd, dt=0.01):
        super().__init__(Kp, Ki, Kd, dt)
        self.Kp0, self.Ki0, self.Kd0 = Kp, Ki, Kd

    def compute(self, error):
        d_error = (error - self.prev_error) / self.dt
        ae = abs(error)
        if ae > 2:
            self.Kp = self.Kp0 * 2.5
            self.Ki = self.Ki0 * 0.3
            self.Kd = self.Kd0 * 2.0
        elif ae > 0.5:
            self.Kp = self.Kp0 * 1.5
            self.Ki = self.Ki0 * 1.0
            self.Kd = self.Kd0 * 1.2
        else:
            self.Kp = self.Kp0 * 1.0
            self.Ki = self.Ki0 * 1.5
            self.Kd = self.Kd0 * 0.8

        self.integral += error * self.dt
        self.integral = np.clip(self.integral, -10, 10)
        u = self.Kp * error + self.Ki * self.integral + self.Kd * d_error
        self.prev_error = error
        self.prev_u = u
        return u


class SmithPredictorPID:
    """9. Smith预估PID"""
    name = 'Smith预估PID'
    def __init__(self, Kp, Ki, Kd, dt=0.01, delay=0.3):
        self.pid = PIDBase(Kp, Ki, Kd, dt)
        self.dt = dt
        self.delay_steps = int(delay / dt)
        # 内部无滞后模型
        self.model = Plant2()
        self.model_hist = [0.0] * (self.delay_steps + 1)
        self.u_hist = [0.0] * (self.delay_steps + 1)

    def reset(self):
        self.pid.reset()
        self.model.reset()
        self.model_hist = [0.0] * (self.delay_steps + 1)
        self.u_hist = [0.0] * (self.delay_steps + 1)

    def compute(self, error, y_actual=0.0, u_prev=0.0):
        # 无滞后模型输出
        y_model_nodelay = self.model.update(u_prev, self.dt)
        self.model_hist.append(y_model_nodelay)
        y_model_delayed = self.model_hist[-self.delay_steps - 1]

        # Smith补偿: r - (y + y_model_nodelay - y_model_delayed)
        compensated_error = error + (y_model_nodelay - y_model_delayed)
        u = self.pid.compute(compensated_error)
        return u


# ============================================================
# 仿真函数
# ============================================================

def run_sim(controller, plant, setpoint, t, need_y=False):
    plant.reset()
    controller.reset()
    y_hist, u_hist = [], []
    for i in range(len(t)):
        y = plant.x1
        error = setpoint[i] - y

        if isinstance(controller, DerivativeFirstPID):
            u = controller.compute(error, y=y, r=setpoint[i])
        elif isinstance(controller, SmithPredictorPID):
            u_prev = controller.prev_u if hasattr(controller, 'prev_u') else 0
            u = controller.compute(error, y_actual=y, u_prev=u_hist[-1] if u_hist else 0)
        elif isinstance(controller, CascadePID):
            u = controller.compute(error, y_inner=y)
        else:
            u = controller.compute(error)

        plant.update(u)
        y_hist.append(y)
        u_hist.append(u)
    return np.array(y_hist), np.array(u_hist)


def calc_metrics(y, setpoint, t):
    e = setpoint - y
    rise_idx = np.where(y >= 0.9 * setpoint[-1])[0]
    rise_time = t[rise_idx[0]] if len(rise_idx) > 0 else float('inf')
    overshoot = max(0, (np.max(y) - setpoint[-1]) / setpoint[-1] * 100) if setpoint[-1] != 0 else 0
    ss_start = int(0.9 * len(t))
    ss_error = np.mean(np.abs(e[ss_start:]))
    itae = np.sum(t * np.abs(e)) * 0.01
    iae = np.sum(np.abs(e)) * 0.01
    # 控制量变化幅度
    return {'rise_time': rise_time, 'overshoot': overshoot,
            'ss_error': ss_error, 'itae': itae, 'iae': iae}


def main():
    dt = 0.01
    t = np.arange(0, 8, dt)

    # 测试1: 标准阶跃
    setpoint1 = np.ones_like(t) * 3.0

    # 测试2: 方波跟踪
    setpoint2 = np.zeros_like(t)
    for i, ti in enumerate(t):
        setpoint2[i] = 3.0 if (ti % 4) < 2 else 1.0

    plant = Plant2(a1=1.2, a0=0.8, b=1.0)

    # 统一参数 (已调优)
    controllers = OrderedDict([
        ('经典PID', ClassicPID(2.5, 1.2, 0.8, dt)),
        ('增量式PID', IncrementalPID(2.5, 1.2, 0.8, dt)),
        ('抗积分饱和PID', AntiWindupPID(2.5, 1.2, 0.8, dt, u_max=8, u_min=-8)),
        ('不完全微分PID', IncompleteDiffPID(2.5, 1.2, 0.8, dt, alpha=0.15)),
        ('微分先行PID', DerivativeFirstPID(2.5, 1.2, 0.8, dt, beta=0.8)),
        ('串级PID', CascadePID(2.5, 1.2, 0.8, 4.0, 2.0, 0.3, dt)),
        ('模糊PID', SimpleFuzzyPID(2.5, 1.2, 0.8, dt)),
        ('自适应PID', AdaptivePID(2.5, 1.2, 0.8, dt)),
        ('Smith预估PID', SmithPredictorPID(2.5, 1.2, 0.8, dt, delay=0.2)),
    ])

    colors = ['#e41a1c', '#377eb8', '#4daf4a', '#984ea3', '#ff7f00',
              '#a65628', '#f781bf', '#999999', '#66c2a5']

    # ---- 阶跃响应对比 ----
    fig1, axes1 = plt.subplots(1, 2, figsize=(16, 7))
    fig1.suptitle('所有PID变种对比 - 阶跃响应', fontsize=14, fontweight='bold')

    results = {}
    for (name, ctrl), color in zip(controllers.items(), colors):
        y, u = run_sim(ctrl, plant, setpoint1, t)
        results[name] = calc_metrics(y, setpoint1, t)
        axes1[0].plot(t, y, color=color, label=name, linewidth=1.5)
        axes1[1].plot(t, u, color=color, label=name, linewidth=1.0)

    axes1[0].plot(t, setpoint1, 'k--', label='设定值', linewidth=1)
    axes1[0].set_title('系统输出')
    axes1[1].set_title('控制信号')
    for ax in axes1:
        ax.legend(fontsize=7, loc='best')
        ax.grid(True, alpha=0.3)
        ax.set_xlabel('时间 (s)')
    fig1.tight_layout()
    fig1.savefig('pid_comparison_all_step.png', dpi=150, bbox_inches='tight')

    # ---- 方波跟踪对比 ----
    fig2, axes2 = plt.subplots(1, 2, figsize=(16, 7))
    fig2.suptitle('所有PID变种对比 - 方波跟踪', fontsize=14, fontweight='bold')

    for (name, ctrl), color in zip(controllers.items(), colors):
        y, u = run_sim(ctrl, plant, setpoint2, t)
        axes2[0].plot(t, y, color=color, label=name, linewidth=1.5)
        axes2[1].plot(t, u, color=color, label=name, linewidth=1.0)

    axes2[0].plot(t, setpoint2, 'k--', label='设定值', linewidth=1)
    axes2[0].set_title('系统输出')
    axes2[1].set_title('控制信号')
    for ax in axes2:
        ax.legend(fontsize=7, loc='best')
        ax.grid(True, alpha=0.3)
        ax.set_xlabel('时间 (s)')
    fig2.tight_layout()
    fig2.savefig('pid_comparison_all_square.png', dpi=150, bbox_inches='tight')

    # ---- 性能指标对比表 ----
    print("\n" + "=" * 100)
    print("所有PID变种 性能指标对比 (阶跃响应)")
    print("=" * 100)
    print(f"{'方法':<16} {'上升时间(s)':<12} {'超调量(%)':<10} {'稳态误差':<10} {'ITAE':<10} {'IAE':<10}")
    print("-" * 100)
    for name, m in results.items():
        print(f"{name:<16} {m['rise_time']:<12.3f} {m['overshoot']:<10.2f} {m['ss_error']:<10.4f} {m['itae']:<10.2f} {m['iae']:<10.2f}")

    # ---- 图3: 性能指标柱状图 ----
    fig3, axes3 = plt.subplots(2, 2, figsize=(16, 10))
    fig3.suptitle('PID变种性能指标对比', fontsize=14, fontweight='bold')

    names = list(results.keys())
    metrics_keys = ['rise_time', 'overshoot', 'ss_error', 'itae']
    titles = ['上升时间 (s)', '超调量 (%)', '稳态误差', 'ITAE']

    for ax, key, title in zip(axes3.flat, metrics_keys, titles):
        vals = [results[n][key] for n in names]
        bars = ax.bar(range(len(names)), vals, color=colors[:len(names)], alpha=0.8)
        ax.set_xticks(range(len(names)))
        ax.set_xticklabels(names, rotation=45, ha='right', fontsize=8)
        ax.set_title(title)
        ax.grid(True, alpha=0.3, axis='y')
        # 在柱上标注数值
        for bar, v in zip(bars, vals):
            ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height(),
                    f'{v:.3f}', ha='center', va='bottom', fontsize=7)

    fig3.tight_layout()
    fig3.savefig('pid_comparison_all_metrics.png', dpi=150, bbox_inches='tight')

    # ---- 综合评分 ----
    print("\n" + "=" * 60)
    print("综合评分 (越低越好, 加权)")
    print("=" * 60)
    weights = {'rise_time': 0.3, 'overshoot': 0.2, 'ss_error': 0.2, 'itae': 0.3}
    scores = {}
    for name in names:
        score = sum(weights[k] * results[name][k] for k in weights)
        scores[name] = score
    ranked = sorted(scores.items(), key=lambda x: x[1])
    for i, (name, score) in enumerate(ranked, 1):
        print(f"  {i}. {name:<16} 综合评分: {score:.4f}")

    print("\n仿真完成! 图片已保存。")
    plt.close('all')


if __name__ == '__main__':
    main()
