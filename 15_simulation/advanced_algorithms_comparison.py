#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
先进算法综合对比仿真
====================
8种控制算法在同一系统上的全面对比：
1. PID
2. 增量式PID
3. 前馈+PID
4. 模糊PID
5. 基于观测器的PID
6. 自适应PID
7. 内模控制(IMC)
8. 状态反馈+积分器

被控对象：二阶系统 G(s) = 1 / (s^2 + 2s + 1)
"""

import os
import numpy as np
import matplotlib


def main():
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
    matplotlib.rcParams['axes.unicode_minus'] = False

    dt = 0.001
    T_total = 5.0
    N = int(T_total / dt)
    t = np.arange(N) * dt

    # ============ 被控对象 ============
    def plant(x, u, dt):
        """二阶系统: x'' + 2*x' + x = u"""
        pos, vel = x
        acc = (u - 2.0 * vel - pos) / 1.0
        vel_new = vel + acc * dt
        pos_new = pos + vel_new * dt
        return np.array([pos_new, vel_new])

    # ============ 控制器定义 ============

    class PIDController:
        """标准PID"""
        def __init__(self, Kp, Ki, Kd):
            self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
            self.integral = 0.0
            self.prev_error = 0.0

        def compute(self, ref, y, dt):
            error = ref - y
            self.integral += error * dt
            derivative = (error - self.prev_error) / dt
            self.prev_error = error
            return self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        def reset(self):
            self.integral = 0.0
            self.prev_error = 0.0


    class IncrementalPID:
        """增量式PID"""
        def __init__(self, Kp, Ki, Kd):
            self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
            self.prev_error = 0.0
            self.prev_prev_error = 0.0
            self.prev_u = 0.0

        def compute(self, ref, y, dt):
            error = ref - y
            du = self.Kp * (error - self.prev_error) + self.Ki * error * dt + \
                 self.Kd * (error - 2*self.prev_error + self.prev_prev_error) / dt
            self.prev_prev_error = self.prev_error
            self.prev_error = error
            self.prev_u += du
            return self.prev_u

        def reset(self):
            self.prev_error = 0.0
            self.prev_prev_error = 0.0
            self.prev_u = 0.0


    class FeedforwardPID:
        """前馈 + PID（使用参考值的二阶导数做前馈）"""
        def __init__(self, Kp, Ki, Kd):
            self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
            self.integral = 0.0
            self.prev_error = 0.0
            self.prev_ref = 0.0
            self.prev_prev_ref = 0.0

        def compute(self, ref, y, dt):
            error = ref - y
            self.integral += error * dt
            derivative = (error - self.prev_error) / dt
            # 前馈项：利用参考信号的微分信息
            ref_dot = (ref - self.prev_ref) / dt
            ref_ddot = (ref - 2*self.prev_ref + self.prev_prev_ref) / dt**2

            u_pid = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
            u_ff = 0.5 * ref + 0.8 * ref_dot  # 简化前馈

            self.prev_prev_ref = self.prev_ref
            self.prev_ref = ref
            self.prev_error = error
            return u_pid + u_ff

        def reset(self):
            self.integral = 0.0
            self.prev_error = 0.0
            self.prev_ref = 0.0
            self.prev_prev_ref = 0.0


    class SimpleFuzzyPID:
        """简化的模糊PID"""
        def __init__(self, Kp0, Ki0, Kd0):
            self.Kp0, self.Ki0, self.Kd0 = Kp0, Ki0, Kd0
            self.integral = 0.0
            self.prev_error = 0.0

        def compute(self, ref, y, dt):
            error = ref - y
            d_error = (error - self.prev_error) / dt
            abs_e = abs(error)

            if abs_e > 0.5:
                Kp = self.Kp0 * 1.5; Ki = self.Ki0 * 0.5; Kd = self.Kd0 * 1.2
            elif abs_e > 0.1:
                Kp = self.Kp0; Ki = self.Ki0; Kd = self.Kd0
            else:
                Kp = self.Kp0 * 0.8; Ki = self.Ki0 * 1.5; Kd = self.Kd0 * 0.5

            self.integral += error * dt
            self.prev_error = error
            return Kp * error + Ki * self.integral + Kd * d_error

        def reset(self):
            self.integral = 0.0
            self.prev_error = 0.0


    class ObserverPID:
        """带状态观测器的PID（利用速度估计改善微分）"""
        def __init__(self, Kp, Ki, Kd, L1, L2):
            self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
            self.L1, self.L2 = L1, L2
            self.integral = 0.0
            self.prev_error = 0.0
            self.x_hat = np.array([0.0, 0.0])
            self.prev_u = 0.0

        def compute(self, ref, y, dt):
            error = ref - y

            # 状态观测器更新 (Luenberger观测器)
            y_hat = self.x_hat[0]
            e_obs = y - y_hat
            x1_dot = self.x_hat[1] + self.L1 * e_obs
            x2_dot = -self.x_hat[0] - 2*self.x_hat[1] + self.prev_u + self.L2 * e_obs
            self.x_hat[0] += x1_dot * dt
            self.x_hat[1] += x2_dot * dt

            self.integral += error * dt
            derivative = (error - self.prev_error) / dt
            self.prev_error = error

            u = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
            self.prev_u = u
            return u

        def reset(self):
            self.integral = 0.0
            self.prev_error = 0.0
            self.x_hat = np.array([0.0, 0.0])


    class AdaptivePID:
        """自适应PID（梯度法在线调整Kp）"""
        def __init__(self, Kp0, Ki0, Kd0, gamma=0.5):
            self.Kp, self.Ki, self.Kd = Kp0, Ki0, Kd0
            self.gamma = gamma
            self.integral = 0.0
            self.prev_error = 0.0

        def compute(self, ref, y, dt):
            error = ref - y
            d_error = (error - self.prev_error) / dt
            self.integral += error * dt

            # 梯度法自适应：dKp = -gamma * e * de/dKp
            self.Kp += -self.gamma * error * d_error * dt
            self.Kp = max(1.0, min(20.0, self.Kp))

            self.prev_error = error
            return self.Kp * error + self.Ki * self.integral + self.Kd * d_error

        def reset(self):
            self.integral = 0.0
            self.prev_error = 0.0
            self.Kp = 10.0


    class IMCController:
        """内模控制 (IMC) - PID式实现"""
        def __init__(self, tau_c):
            self.tau_c = tau_c
            self.integral = 0.0
            self.prev_error = 0.0
            self.xm = np.array([0.0, 0.0])

        def compute(self, ref, y, dt):
            error = ref - y

            # 更新内部模型
            xm_dot = self.xm[1]
            xm_ddot = -2.0 * self.xm[1] - self.xm[0] + self.prev_u
            self.xm[0] += xm_dot * dt
            self.xm[1] += xm_ddot * dt

            # 扰动估计
            d_hat = y - self.xm[0]

            # IMC-PID等效
            self.integral += error * dt
            derivative = (error - self.prev_error) / dt
            self.prev_error = error

            Kp_imc = 2.0 / self.tau_c
            Ki_imc = 1.0 / self.tau_c
            Kd_imc = self.tau_c

            u = Kp_imc * error + Ki_imc * self.integral + Kd_imc * derivative - d_hat * 0.5
            self.prev_u = u
            return np.clip(u, -50, 50)

        def reset(self):
            self.integral = 0.0
            self.prev_error = 0.0
            self.prev_u = 0.0
            self.xm = np.array([0.0, 0.0])


    class StateFeedbackIntegrator:
        """状态反馈 + 积分器"""
        def __init__(self, K, Ki):
            self.K = np.array(K)
            self.Ki = Ki
            self.integral = 0.0
            self.prev_y = 0.0
            self.y_dot = 0.0

        def compute(self, ref, y, dt):
            error = ref - y
            self.integral += error * dt
            self.y_dot = (y - self.prev_y) / dt
            self.prev_y = y

            # 状态反馈 + 积分
            u = self.K[0] * error + self.K[1] * (-self.y_dot) + self.Ki * self.integral
            return np.clip(u, -50, 50)

        def reset(self):
            self.integral = 0.0
            self.prev_y = 0.0


    # ============ 运行仿真 ============
    def run_sim(controller, ref_func, dist_func, dt, N):
        state = np.array([0.0, 0.0])
        controller.reset()
        y_log = np.zeros(N)
        u_log = np.zeros(N)

        for i in range(N):
            ref = ref_func(i * dt)
            y = state[0]
            u = controller.compute(ref, y, dt)
            d = dist_func(i * dt)
            state = plant(state, u + d, dt)
            y_log[i] = state[0]
            u_log[i] = u

        return y_log, u_log


    def calc_metrics(t, y, ref, t_start=0.3):
        idx = int(t_start / dt)
        err = ref - y[idx:]
        IAE = np.sum(np.abs(err)) * dt
        ISE = np.sum(err**2) * dt
        ITAE = np.sum(t[idx:idx+len(err)] * np.abs(err)) * dt
        overshoot = max(0, (np.max(y) - ref) / ref * 100) if ref > 0 else 0
        settling = t[-1]
        for i in range(len(y)-1, 0, -1):
            if np.abs(y[i] - ref) > 0.02 * abs(ref):
                settling = t[min(i+1, len(t)-1)]
                break
        return {'IAE': IAE, 'ISE': ISE, 'ITAE': ITAE, '超调%': overshoot, '调节时间': settling}

    # ============ 控制器实例 ============
    controllers = {
        'PID':           PIDController(Kp=10, Ki=5, Kd=3),
        '增量式PID':      IncrementalPID(Kp=10, Ki=5, Kd=3),
        '前馈+PID':       FeedforwardPID(Kp=10, Ki=5, Kd=3),
        '模糊PID':        SimpleFuzzyPID(Kp0=10, Ki0=5, Kd0=3),
        '观测器PID':      ObserverPID(Kp=10, Ki=5, Kd=3, L1=20, L2=100),
        '自适应PID':      AdaptivePID(Kp0=10, Ki0=5, Kd0=3, gamma=0.5),
        'IMC':           IMCController(tau_c=0.3),
        '状态反馈+积分':   StateFeedbackIntegrator(K=[8, 3], Ki=5),
    }

    # ============ 实验1: 阶跃响应 ============
    print("=" * 60)
    print("先进算法综合对比仿真")
    print("=" * 60)

    ref_func = lambda t: 1.0 if t >= 0.3 else 0.0
    dist_func = lambda t: 0.0

    results = {}
    metrics_table = {}

    for name, ctrl in controllers.items():
        y, u = run_sim(ctrl, ref_func, dist_func, dt, N)
        results[name] = (y, u)
        metrics_table[name] = calc_metrics(t, y, 1.0)
        print(f"{name:12s} - IAE:{metrics_table[name]['IAE']:.3f} "
              f"超调:{metrics_table[name]['超调%']:.1f}% "
              f"调节时间:{metrics_table[name]['调节时间']:.3f}s")

    # ============ 实验2: 正弦跟踪 ============
    ref_sine = lambda t: np.sin(2 * np.pi * 0.5 * t)
    results_sine = {}
    metrics_sine = {}

    print("\n正弦跟踪:")
    for name, ctrl in controllers.items():
        y, u = run_sim(ctrl, ref_sine, dist_func, dt, N)
        results_sine[name] = (y, u)
        err = np.abs(ref_sine(t[300:]) - y[300:])
        metrics_sine[name] = np.mean(err)
        print(f"{name:12s} - 平均误差: {metrics_sine[name]:.4f}")

    # ============ 绘图 ============
    fig, axes = plt.subplots(2, 2, figsize=(18, 14))
    fig.suptitle('先进控制算法综合对比（8种算法）', fontsize=16, fontweight='bold')

    colors = plt.cm.Set2(np.linspace(0, 1, 8))

    # 阶跃响应
    ax = axes[0, 0]
    ax.axhline(y=1.0, color='k', linestyle=':', alpha=0.5, label='参考值')
    for idx, (name, (y, u)) in enumerate(results.items()):
        ax.plot(t, y, '-', color=colors[idx], linewidth=1.5, label=name)
    ax.set_title('阶跃响应对比')
    ax.set_xlabel('时间 (s)'); ax.set_ylabel('输出')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    # 正弦跟踪
    ax = axes[0, 1]
    ax.plot(t, ref_sine(t), 'k--', linewidth=2, label='参考')
    for idx, (name, (y, u)) in enumerate(results_sine.items()):
        ax.plot(t, y, '-', color=colors[idx], linewidth=1, alpha=0.8, label=name)
    ax.set_title('正弦轨迹跟踪')
    ax.set_xlabel('时间 (s)'); ax.set_ylabel('输出')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    # 性能指标柱状图
    ax = axes[1, 0]
    names = list(metrics_table.keys())
    iae_vals = [metrics_table[n]['IAE'] for n in names]
    x_pos = np.arange(len(names))
    bars = ax.bar(x_pos, iae_vals, color=colors[:len(names)])
    ax.set_title('IAE性能指标对比')
    ax.set_xticks(x_pos)
    ax.set_xticklabels(names, fontsize=8, rotation=30, ha='right')
    ax.set_ylabel('IAE')
    ax.grid(True, alpha=0.3, axis='y')
    for bar in bars:
        ax.text(bar.get_x() + bar.get_width()/2., bar.get_height(),
                f'{bar.get_height():.2f}', ha='center', va='bottom', fontsize=7)

    # 正弦跟踪平均误差
    ax = axes[1, 1]
    sine_vals = [metrics_sine[n] for n in names]
    bars2 = ax.bar(x_pos, sine_vals, color=colors[:len(names)])
    ax.set_title('正弦跟踪平均误差 (rad)')
    ax.set_xticks(x_pos)
    ax.set_xticklabels(names, fontsize=8, rotation=30, ha='right')
    ax.set_ylabel('平均误差')
    ax.grid(True, alpha=0.3, axis='y')
    for bar in bars2:
        ax.text(bar.get_x() + bar.get_width()/2., bar.get_height(),
                f'{bar.get_height():.3f}', ha='center', va='bottom', fontsize=7)

    plt.tight_layout()
    plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'advanced_algorithms_comparison_result.png'), dpi=150, bbox_inches='tight')
    print("\n图表已保存: advanced_algorithms_comparison_result.png")
    plt.close('all')
    print("先进算法综合对比仿真完成!")



if __name__ == '__main__':
    main()
