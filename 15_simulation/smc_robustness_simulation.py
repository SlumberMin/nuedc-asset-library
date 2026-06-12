#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SMC 鲁棒性仿真
===============
仿真内容：滑模控制在参数不确定性+外部扰动下的鲁棒性
被控对象：二阶系统
对比：SMC vs PID 在不同扰动条件下的表现
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
    T_total = 8.0
    N = int(T_total / dt)
    t = np.arange(N) * dt

    # ============ 被控对象：二阶系统 ============
    # x'' + a*x' + b*x = c*u + d(t)
    def plant_step(x, x_dot, u, params, d=0):
        """二阶系统一步更新"""
        a, b, c = params
        x_ddot = -a * x_dot - b * x + c * u + d
        x_dot_new = x_dot + x_ddot * dt
        x_new = x + x_dot_new * dt
        return x_new, x_dot_new

    # 标称参数
    params_nominal = (2.0, 1.0, 1.0)
    # 不确定参数
    params_uncertain = (3.0, 2.0, 1.3)

    # ============ SMC控制器（基于趋近律） ============
    class SMCController:
        """滑模控制器 - 用于二阶系统 x''=f(x)+g(x)*u"""
        def __init__(self, c_s, eta_s, k_s):
            """
            c_s: 滑模面参数 s = e_dot + c_s * e
            eta_s: 趋近律增益 (到达速率)
            k_s: 等效控制增益（名义增益）
            """
            self.c_s = c_s
            self.eta_s = eta_s
            self.k_s = k_s
            self.prev_e = 0.0

        def compute(self, ref, x1, x2, dt):
            """
            ref: 参考输入（假设为常值或缓慢变化）
            x1: 位置, x2: 速度
            """
            e = ref - x1
            e_dot = (e - self.prev_e) / dt
            self.prev_e = e

            # 滑模面
            s = e_dot + self.c_s * e

            # 等效控制 + 切换控制
            # 简化：u = (1/c)*(a_n*x2+b_n*x1) + (1/c)*(-c_s*e_dot+ref_ddot) + 切换项
            # 其中切线项保证趋近滑模面
            u_eq = self.k_s * (e + self.c_s * e_dot)
            u_sw = self.eta_s * np.sign(s)

            u = u_eq + u_sw
            return np.clip(u, -50, 50)

    # ============ PID控制器 ============
    class PIDController:
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

    # ============ 仿真运行 ============
    def run_sim(controller, params_true, ref_func, dist_func, dt, N, is_smc=False):
        x, x_dot = 0.0, 0.0
        y_log = np.zeros(N)
        u_log = np.zeros(N)

        for i in range(N):
            ref = ref_func(i * dt)
            y = x

            if is_smc:
                u = controller.compute(ref, x, x_dot, dt)
            else:
                u = controller.compute(ref, y, dt)

            d = dist_func(i * dt)
            x, x_dot = plant_step(x, x_dot, u, params_true, d)

            y_log[i] = x
            u_log[i] = u

        return y_log, u_log

    # ============ 性能指标 ============
    def calc_metrics(t, y, ref, t_start=0.5):
        idx = int(t_start / dt)
        err = ref - y[idx:]
        IAE = np.sum(np.abs(err)) * dt
        ISE = np.sum(err**2) * dt
        overshoot = max(0, (np.max(y) - ref) / ref * 100) if ref > 0 else 0
        settling = t[-1]
        for i in range(len(y)-1, 0, -1):
            if np.abs(y[i] - ref) > 0.02 * abs(ref):
                settling = t[min(i+1, len(t)-1)]
                break
        return {'IAE': IAE, 'ISE': ISE, '超调%': overshoot, '调节时间': settling}

    # ============ 实验场景 ============
    print("=" * 60)
    print("SMC 鲁棒性仿真")
    print("=" * 60)

    ref = lambda t: 1.0 if t >= 0.5 else 0.0
    dist0 = lambda t: 0.0
    dist_sine = lambda t: 0.3 * np.sin(2*np.pi*2*t) if t >= 2.0 else 0.0

    scenarios = [
        ("场景1: 标称参数", params_nominal, dist0),
        ("场景2: 参数不确定", params_uncertain, dist0),
        ("场景3: 正弦扰动", params_nominal, dist_sine),
        ("场景4: 参数+扰动", params_uncertain, dist_sine),
    ]

    all_results = {}
    for name, params, dist in scenarios:
        smc = SMCController(c_s=5.0, eta_s=8.0, k_s=5.0)
        pid = PIDController(Kp=15, Ki=8, Kd=3)

        y_smc, u_smc = run_sim(smc, params, ref, dist, dt, N, is_smc=True)
        y_pid, u_pid = run_sim(pid, params, ref, dist, dt, N)

        m_smc = calc_metrics(t, y_smc, 1.0)
        m_pid = calc_metrics(t, y_pid, 1.0)

        all_results[name] = (y_smc, u_smc, m_smc, y_pid, u_pid, m_pid)
        print(f"{name}: SMC-IAE={m_smc['IAE']:.3f} 超调={m_smc['超调%']:.1f}%, "
              f"PID-IAE={m_pid['IAE']:.3f} 超调={m_pid['超调%']:.1f}%")

    # ============ 绘图 ============
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    fig.suptitle('SMC vs PID 鲁棒性对比仿真', fontsize=16, fontweight='bold')

    for idx, (name, (y_smc, u_smc, m_smc, y_pid, u_pid, m_pid)) in enumerate(all_results.items()):
        ax = axes[idx // 2, idx % 2]
        ax.plot(t, y_smc, 'r-', label=f'SMC (IAE={m_smc["IAE"]:.2f})', linewidth=1.5)
        ax.plot(t, y_pid, 'b--', label=f'PID (IAE={m_pid["IAE"]:.2f})', linewidth=1.5)
        ax.axhline(y=1.0, color='k', linestyle=':', alpha=0.5, label='参考值')
        if idx >= 2:
            ax.axvline(x=2.0, color='green', linestyle='--', alpha=0.5, label='扰动施加')
        ax.set_title(name)
        ax.set_xlabel('时间 (s)'); ax.set_ylabel('输出')
        ax.legend(fontsize=9); ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'smc_robustness_result.png'), dpi=150, bbox_inches='tight')
    print("\n图表已保存: smc_robustness_result.png")
    print("SMC鲁棒性仿真完成!")



if __name__ == '__main__':
    main()
