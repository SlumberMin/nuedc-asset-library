#!/usr/bin/env python3
"""
ADRC参数研究仿真
================
仿真内容：ESO带宽、控制带宽、b0对ADRC性能的影响
被控对象：二阶系统
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
matplotlib.rcParams['axes.unicode_minus'] = False


class LADRC:
    """线性ADRC控制器"""

    def __init__(self, wc, wo, b0, dt):
        self.wc = wc
        self.wo = wo
        self.b0 = b0 if abs(b0) > 1e-6 else 1e-6
        self.dt = dt
        self.z1 = 0.0
        self.z2 = 0.0
        self.z3 = 0.0
        self.prev_y = 0.0
        self.u_prev = 0.0

    def compute(self, ref, y):
        e_eso = self.prev_y - self.z1
        self.z1 += (self.z2 + 3 * self.wo * e_eso) * self.dt
        self.z2 += (self.z3 + 3 * self.wo**2 * e_eso + self.b0 * self.u_prev) * self.dt
        self.z3 += (self.wo**3 * e_eso) * self.dt
        self.prev_y = y

        e1 = ref - self.z1
        e2 = 0 - self.z2
        u0 = self.wc**2 * e1 + 2 * self.wc * e2
        u = (u0 - self.z3) / self.b0
        self.u_prev = u
        return u

    def reset(self):
        self.z1 = self.z2 = self.z3 = 0.0
        self.prev_y = 0.0
        self.u_prev = 0.0


def plant_step(x, u, m, b, k, dt):
    pos, vel = x
    acc = (u - b * vel - k * pos) / m
    vel_new = vel + acc * dt
    pos_new = pos + vel_new * dt
    return np.array([pos_new, vel_new])


def run_sim(ctrl, m, b, k, ref_func, dist_func, dt, N):
    x = np.array([0.0, 0.0])
    y_log = np.zeros(N)
    for i in range(N):
        ref = ref_func(i * dt)
        y = x[0]
        u = ctrl.compute(ref, y)
        d = dist_func(i * dt)
        x = plant_step(x, u + d, m, b, k, dt)
        y_log[i] = x[0]
    return y_log


def calc_iae(y, ref, dt, t_start=0.0, N_total=None):
    if N_total is None:
        N_total = len(y)
    idx = int(t_start / dt)
    return np.sum(np.abs(ref - y[idx:])) * dt


def main():
    dt = 0.001
    T = 5.0
    N = int(T / dt)
    t = np.arange(N) * dt
    m, b, k = 1.0, 0.5, 1.0
    ref_func = lambda t: 1.0 if t >= 0.5 else 0.0
    dist_func = lambda t: 0.0

    fig, axes = plt.subplots(3, 3, figsize=(18, 14))
    fig.suptitle('ADRC参数研究仿真', fontsize=16, fontweight='bold')

    # ===== 1. ESO带宽 wo 影响 =====
    wo_list = [5, 10, 20, 50, 100]
    colors = plt.cm.viridis(np.linspace(0.2, 0.9, len(wo_list)))
    iaes_wo = []
    ax = axes[0, 0]
    for wo, c in zip(wo_list, colors):
        ctrl = LADRC(wc=10, wo=wo, b0=1.0, dt=dt)
        y = run_sim(ctrl, m, b, k, ref_func, dist_func, dt, N)
        iae = calc_iae(y, 1.0, dt, 0.5)
        iaes_wo.append(iae)
        ax.plot(t, y, color=c, linewidth=1, label=f'wo={wo}')
    ax.axhline(1.0, color='k', linestyle=':', alpha=0.5)
    ax.set_title('ESO带宽(wo)对响应的影响')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('输出')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[0, 1]
    ax.bar(range(len(wo_list)), iaes_wo, color=colors)
    ax.set_xticks(range(len(wo_list)))
    ax.set_xticklabels([str(w) for w in wo_list])
    ax.set_title('不同wo的IAE')
    ax.set_xlabel('wo')
    ax.set_ylabel('IAE')
    ax.grid(True, alpha=0.3, axis='y')

    # wo vs IAE趋势
    ax = axes[0, 2]
    wo_range = np.arange(5, 150, 5)
    iaes_wo_range = []
    for wo in wo_range:
        ctrl = LADRC(wc=10, wo=wo, b0=1.0, dt=dt)
        y = run_sim(ctrl, m, b, k, ref_func, dist_func, dt, N)
        iaes_wo_range.append(calc_iae(y, 1.0, dt, 0.5))
    ax.plot(wo_range, iaes_wo_range, 'b-', linewidth=2)
    ax.set_title('wo vs IAE趋势')
    ax.set_xlabel('ESO带宽 wo')
    ax.set_ylabel('IAE')
    ax.grid(True, alpha=0.3)

    # ===== 2. 控制带宽 wc 影响 =====
    wc_list = [2, 5, 10, 20, 50]
    colors2 = plt.cm.plasma(np.linspace(0.2, 0.9, len(wc_list)))
    iaes_wc = []
    ax = axes[1, 0]
    for wc, c in zip(wc_list, colors2):
        ctrl = LADRC(wc=wc, wo=30, b0=1.0, dt=dt)
        y = run_sim(ctrl, m, b, k, ref_func, dist_func, dt, N)
        iae = calc_iae(y, 1.0, dt, 0.5)
        iaes_wc.append(iae)
        ax.plot(t, y, color=c, linewidth=1, label=f'wc={wc}')
    ax.axhline(1.0, color='k', linestyle=':', alpha=0.5)
    ax.set_title('控制带宽(wc)对响应的影响')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('输出')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[1, 1]
    ax.bar(range(len(wc_list)), iaes_wc, color=colors2)
    ax.set_xticks(range(len(wc_list)))
    ax.set_xticklabels([str(w) for w in wc_list])
    ax.set_title('不同wc的IAE')
    ax.set_xlabel('wc')
    ax.set_ylabel('IAE')
    ax.grid(True, alpha=0.3, axis='y')

    # wc趋势
    ax = axes[1, 2]
    wc_range = np.arange(2, 80, 3)
    iaes_wc_range = []
    for wc in wc_range:
        ctrl = LADRC(wc=wc, wo=30, b0=1.0, dt=dt)
        y = run_sim(ctrl, m, b, k, ref_func, dist_func, dt, N)
        iaes_wc_range.append(calc_iae(y, 1.0, dt, 0.5))
    ax.plot(wc_range, iaes_wc_range, 'r-', linewidth=2)
    ax.set_title('wc vs IAE趋势')
    ax.set_xlabel('控制带宽 wc')
    ax.set_ylabel('IAE')
    ax.grid(True, alpha=0.3)

    # ===== 3. b0 影响 =====
    b0_list = [0.5, 0.8, 1.0, 1.5, 2.0]
    colors3 = plt.cm.coolwarm(np.linspace(0.2, 0.9, len(b0_list)))
    iaes_b0 = []
    ax = axes[2, 0]
    for b0, c in zip(b0_list, colors3):
        ctrl = LADRC(wc=10, wo=30, b0=b0, dt=dt)
        y = run_sim(ctrl, m, b, k, ref_func, dist_func, dt, N)
        iae = calc_iae(y, 1.0, dt, 0.5)
        iaes_b0.append(iae)
        ax.plot(t, y, color=c, linewidth=1, label=f'b0={b0}')
    ax.axhline(1.0, color='k', linestyle=':', alpha=0.5)
    ax.set_title('对象增益估计(b0)对响应的影响')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('输出')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[2, 1]
    ax.bar(range(len(b0_list)), iaes_b0, color=colors3)
    ax.set_xticks(range(len(b0_list)))
    ax.set_xticklabels([str(b) for b in b0_list])
    ax.set_title('不同b0的IAE')
    ax.set_xlabel('b0')
    ax.set_ylabel('IAE')
    ax.grid(True, alpha=0.3, axis='y')

    # 参数敏感度热图
    ax = axes[2, 2]
    wc_arr = [5, 10, 20, 40]
    wo_arr = [10, 20, 50, 100]
    heat = np.zeros((len(wc_arr), len(wo_arr)))
    for i, wc in enumerate(wc_arr):
        for j, wo in enumerate(wo_arr):
            ctrl = LADRC(wc=wc, wo=wo, b0=1.0, dt=dt)
            y = run_sim(ctrl, m, b, k, ref_func, dist_func, dt, N)
            heat[i, j] = calc_iae(y, 1.0, dt, 0.5)
    im = ax.imshow(heat, cmap='YlOrRd', aspect='auto')
    ax.set_xticks(range(len(wo_arr)))
    ax.set_xticklabels([str(w) for w in wo_arr])
    ax.set_yticks(range(len(wc_arr)))
    ax.set_yticklabels([str(w) for w in wc_arr])
    ax.set_xlabel('wo')
    ax.set_ylabel('wc')
    ax.set_title('wc-wo参数空间IAE热图')
    plt.colorbar(im, ax=ax, label='IAE')
    for i in range(len(wc_arr)):
        for j in range(len(wo_arr)):
            ax.text(j, i, f'{heat[i,j]:.2f}', ha='center', va='center', fontsize=8)

    plt.tight_layout()
    out_dir = os.path.dirname(os.path.abspath(__file__))
    plt.savefig(os.path.join(out_dir, 'adrc_parameter_study_result.png'), dpi=150, bbox_inches='tight')
    print("图表已保存: adrc_parameter_study_result.png")
    plt.close('all')

    # 总结
    best_wo = wo_list[np.argmin(iaes_wo)]
    best_wc = wc_list[np.argmin(iaes_wc)]
    best_b0 = b0_list[np.argmin(iaes_b0)]
    print(f"\n仿真完成!")
    print(f"最优wo={best_wo}, 最优wc={best_wc}, 最优b0={best_b0}")


if __name__ == '__main__':
    main()
