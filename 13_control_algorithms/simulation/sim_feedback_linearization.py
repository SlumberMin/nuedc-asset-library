"""
反馈线性化仿真 (Feedback Linearization Simulation)
==============================================
通过非线性坐标变换和状态反馈，将非线性系统精确线性化，
再对线性化后的等效系统设计控制器。

系统（单摆 + 扭矩输入）:
  ẍ + sin(x) = u
  即: ẋ₁ = x₂
      ẋ₂ = -sin(x₁) + u
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import rcParams



def main():
    rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'STHeiti']
    rcParams['axes.unicode_minus'] = False

    # ============================================================
    # 仿真参数
    # ============================================================
    dt = 1e-4
    T_sim = 5.0
    N = int(T_sim / dt)
    t = np.arange(N) * dt

    # 参考轨迹: 平滑阶跃（5次多项式插值）
    x_d = np.zeros(N)
    t_rise = 0.5  # 上升时间
    for k in range(N):
        if t[k] < t_rise:
            tau = t[k] / t_rise
            x_d[k] = 1.0 * (6*tau**5 - 15*tau**4 + 10*tau**3)
        else:
            x_d[k] = 1.0

    # 参考轨迹的导数
    dx_d = np.gradient(x_d, dt)

    # 线性化后的PD增益（等效二阶线性系统）
    omega_n = 15.0   # 自然频率
    zeta = 0.8       # 阻尼比
    Kp_lin = omega_n**2    # 225
    Kd_lin = 2 * zeta * omega_n  # 24

    # ============================================================
    # 反馈线性化控制仿真
    # ============================================================
    def run_feedback_lin():
        """精确反馈线性化: u = sin(x₁) + v, 其中v为线性控制律"""
        x1 = np.zeros(N)
        x2 = np.zeros(N)
        u = np.zeros(N)
        v_arr = np.zeros(N)
        x1[0] = 0.0  # 初始角度
        x2[0] = 0.0  # 初始角速度

        for k in range(N - 1):
            e = x_d[k] - x1[k]
            de = dx_d[k] - x2[k]
            # 线性化后等效控制 v = ẍ_d + Kd*ė + Kp*e
            v = Kp_lin * e + Kd_lin * de
            v_arr[k] = v
            # 非线性补偿: u = sin(x₁) + v
            u[k] = np.sin(x1[k]) + v
            # 系统仿真
            f1 = x2[k]
            f2 = -np.sin(x1[k]) + u[k]
            x1[k+1] = x1[k] + dt * f1
            x2[k+1] = x2[k] + dt * f2
        return x1, x2, u, v_arr

    def run_without_linearization():
        """不使用反馈线性化，直接PD控制（忽略非线性项）"""
        x1 = np.zeros(N)
        x2 = np.zeros(N)
        u = np.zeros(N)
        x1[0] = 0.0
        x2[0] = 0.0

        for k in range(N - 1):
            e = x_d[k] - x1[k]
            de = dx_d[k] - x2[k]
            u[k] = Kp_lin * e + Kd_lin * de
            f1 = x2[k]
            f2 = -np.sin(x1[k]) + u[k]
            x1[k+1] = x1[k] + dt * f1
            x2[k+1] = x2[k] + dt * f2
        return x1, x2, u

    # ============================================================
    # 运行仿真
    # ============================================================
    x1_fl, x2_fl, u_fl, v_fl = run_feedback_lin()
    x1_pd, x2_pd, u_pd = run_without_linearization()

    # ============================================================
    # 性能指标
    # ============================================================
    def calc_metrics(x, x_d_arr, t_start=1.0):
        e = x - x_d_arr
        s = int(t_start / dt)
        e_ss = e[s:]
        rmse = np.sqrt(np.mean(e_ss**2))
        mae = np.mean(np.abs(e_ss))
        max_e = np.max(np.abs(e_ss))
        return rmse, mae, max_e

    rmse_fl, mae_fl, max_fl = calc_metrics(x1_fl, x_d)
    rmse_pd, mae_pd, max_pd = calc_metrics(x1_pd, x_d)

    print("=" * 60)
    print("    反馈线性化 vs 直接PD — 非线性系统精确线性化")
    print("=" * 60)
    print(f"{'指标':<16} {'反馈线性化':>14} {'直接PD':>14}")
    print("-" * 60)
    print(f"{'RMSE (稳态)':<16} {rmse_fl:>14.6f} {rmse_pd:>14.6f}")
    print(f"{'MAE  (稳态)':<16} {mae_fl:>14.6f} {mae_pd:>14.6f}")
    print(f"{'最大误差(稳态)':<14} {max_fl:>14.6f} {max_pd:>14.6f}")
    print("=" * 60)

    # ============================================================
    # 绘图
    # ============================================================
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('反馈线性化仿真 — 非线性系统精确线性化', fontsize=15, fontweight='bold')

    ax = axes[0, 0]
    ax.plot(t, x_d, 'k--', lw=1.2, label='参考轨迹')
    ax.plot(t, x1_fl, 'b-', lw=0.9, label='反馈线性化')
    ax.plot(t, x1_pd, 'r-', lw=0.9, alpha=0.7, label='直接PD(无补偿)')
    ax.set_xlabel('时间 (s)'); ax.set_ylabel('角度 x₁ (rad)')
    ax.set_title('(a) 角度跟踪对比')
    ax.legend(fontsize=9); ax.grid(True, alpha=0.3)

    ax = axes[0, 1]
    ax.plot(t, x1_fl - x_d, 'b-', lw=0.6, label='反馈线性化误差')
    ax.plot(t, x1_pd - x_d, 'r-', lw=0.6, alpha=0.7, label='直接PD误差')
    ax.set_xlabel('时间 (s)'); ax.set_ylabel('误差 (rad)')
    ax.set_title('(b) 跟踪误差')
    ax.legend(fontsize=9); ax.grid(True, alpha=0.3)

    ax = axes[1, 0]
    ax.plot(t, u_fl, 'b-', lw=0.6, label='反馈线性化 u')
    ax.plot(t, u_pd, 'r-', lw=0.6, alpha=0.7, label='直接PD u')
    ax.set_xlabel('时间 (s)'); ax.set_ylabel('控制力矩')
    ax.set_title('(c) 控制量对比')
    ax.legend(fontsize=9); ax.grid(True, alpha=0.3)

    ax = axes[1, 1]
    ax.plot(t, v_fl, 'b-', lw=0.6, label='等效线性控制 v')
    ax.plot(t, np.sin(x1_fl), 'g-', lw=0.6, label='非线性补偿 sin(x₁)')
    ax.plot(t, u_fl, 'r-', lw=0.4, alpha=0.5, label='总控制 u = sin(x₁)+v')
    ax.set_xlabel('时间 (s)'); ax.set_ylabel('幅值')
    ax.set_title('(d) 控制律分解')
    ax.legend(fontsize=9); ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('sim_feedback_linearization.png', dpi=150, bbox_inches='tight')
    plt.close('all')
    print("图表已保存: sim_feedback_linearization.png")



if __name__ == '__main__':
    main()
