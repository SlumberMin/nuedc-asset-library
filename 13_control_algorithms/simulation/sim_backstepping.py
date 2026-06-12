"""
反步法仿真 (Backstepping Control Simulation)
==============================================
针对严格反馈非线性系统，递推设计控制律。
与PID对比展示反步法在非线性系统中的优势。

系统: 严反馈形式
  ẋ₁ = x₁² + x₂
  ẋ₂ = u

目标: 跟踪参考信号 x₁ → x_d
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
    T_sim = 3.0
    N = int(T_sim / dt)
    t = np.arange(N) * dt

    # 参考信号: 平滑正弦
    x_d = np.sin(1.5 * t) + 0.3 * np.sin(3.0 * t)
    dx_d = 1.5 * np.cos(1.5 * t) + 0.9 * np.cos(3.0 * t)

    # 反步法设计参数
    c1 = 5.0   # 第一层虚拟控制增益
    c2 = 8.0   # 第二层控制增益

    # PID参数（对比用）
    Kp = 80.0
    Ki = 200.0
    Kd = 5.0

    # ============================================================
    # 反步法仿真
    # ============================================================
    def run_backstepping():
        x1 = np.zeros(N)
        x2 = np.zeros(N)
        u = np.zeros(N)
        # 初始条件
        x1[0] = 0.5
        x2[0] = 0.0

        for k in range(N - 1):
            e1 = x1[k] - x_d[k]
            # 虚拟控制律 α₁
            alpha1 = -c1 * e1 + dx_d[k] - x1[k]**2
            # 第二层误差
            e2 = x2[k] - alpha1
            # 反步控制律
            # V̇ = -c1*e1² + e1*e2 + e2*(u - ∂α₁/∂x₁ * ẋ₁ - ∂α₁/∂x_d * ẋ_d)
            # ∂α₁/∂x₁ = -c1 - 2*x1,  这里简化处理
            dalpha1_dx1 = -c1 - 2 * x1[k]
            dalpha1_dxd = 1.0  # ∂α₁/∂ẋ_d 部分已包含
            # u = -e1 - c2*e2 + dalpha1_dx1*(x1²+x2) + dalpha1_dxd*ẍ_d（简化）
            # 简化控制律:
            u[k] = -e1 - c2 * e2 + dalpha1_dx1 * (x1[k]**2 + x2[k])
            # 系统仿真 (Runge-Kutta 4 简化为前向欧拉)
            f1 = x1[k]**2 + x2[k]
            f2 = u[k]
            x1[k+1] = x1[k] + dt * f1
            x2[k+1] = x2[k] + dt * f2
        return x1, x2, u

    def run_pid():
        x1 = np.zeros(N)
        x2 = np.zeros(N)
        u = np.zeros(N)
        x1[0] = 0.5
        x2[0] = 0.0
        e_int = 0.0
        e_prev = 0.0

        for k in range(N - 1):
            e = x_d[k] - x1[k]
            e_int += e * dt
            e_der = (e - e_prev) / dt
            u[k] = Kp * e + Ki * e_int + Kd * e_der
            e_prev = e
            f1 = x1[k]**2 + x2[k]
            f2 = u[k]
            x1[k+1] = x1[k] + dt * f1
            x2[k+1] = x2[k] + dt * f2
        return x1, x2, u

    # ============================================================
    # 运行仿真
    # ============================================================
    x1_bs, x2_bs, u_bs = run_backstepping()
    x1_pid, x2_pid, u_pid = run_pid()

    # ============================================================
    # 性能指标
    # ============================================================
    def calc_metrics(x1, x_d_arr):
        e = x1 - x_d_arr
        steady = int(0.3 * N)
        e_ss = e[steady:]
        rmse = np.sqrt(np.mean(e_ss**2))
        mae = np.mean(np.abs(e_ss))
        max_err = np.max(np.abs(e_ss))
        return rmse, mae, max_err

    rmse_bs, mae_bs, max_bs = calc_metrics(x1_bs, x_d)
    rmse_pid, mae_pid, max_pid = calc_metrics(x1_pid, x_d)

    print("=" * 55)
    print("       反步法 vs PID — 非线性系统跟踪性能")
    print("=" * 55)
    print(f"{'指标':<16} {'反步法':>12} {'PID':>12}")
    print("-" * 55)
    print(f"{'RMSE (稳态)':<16} {rmse_bs:>12.6f} {rmse_pid:>12.6f}")
    print(f"{'MAE  (稳态)':<16} {mae_bs:>12.6f} {mae_pid:>12.6f}")
    print(f"{'最大误差(稳态)':<14} {max_bs:>12.6f} {max_pid:>12.6f}")
    print("=" * 55)

    # ============================================================
    # 绘图
    # ============================================================
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('反步法控制仿真 — 非线性系统跟踪', fontsize=15, fontweight='bold')

    ax = axes[0, 0]
    ax.plot(t, x_d, 'k--', lw=1.0, label='参考信号')
    ax.plot(t, x1_bs, 'b-', lw=0.8, label='反步法')
    ax.plot(t, x1_pid, 'r-', lw=0.8, alpha=0.7, label='PID')
    ax.set_xlabel('时间 (s)'); ax.set_ylabel('x₁')
    ax.set_title('(a) 状态跟踪对比')
    ax.legend(fontsize=9); ax.grid(True, alpha=0.3)

    ax = axes[0, 1]
    e_bs = x1_bs - x_d
    e_pid = x1_pid - x_d
    ax.plot(t, e_bs, 'b-', lw=0.6, label='反步法误差')
    ax.plot(t, e_pid, 'r-', lw=0.6, alpha=0.7, label='PID误差')
    ax.set_xlabel('时间 (s)'); ax.set_ylabel('误差')
    ax.set_title('(b) 跟踪误差')
    ax.legend(fontsize=9); ax.grid(True, alpha=0.3)

    ax = axes[1, 0]
    ax.plot(t, u_bs, 'b-', lw=0.6, label='反步法 u')
    ax.plot(t, u_pid, 'r-', lw=0.6, alpha=0.7, label='PID u')
    ax.set_xlabel('时间 (s)'); ax.set_ylabel('控制量')
    ax.set_title('(c) 控制量对比')
    ax.legend(fontsize=9); ax.grid(True, alpha=0.3)

    ax = axes[1, 1]
    ax.plot(x1_bs, x2_bs, 'b-', lw=0.6, label='反步法相轨迹')
    ax.plot(x1_pid, x2_pid, 'r-', lw=0.6, alpha=0.7, label='PID相轨迹')
    ax.set_xlabel('x₁'); ax.set_ylabel('x₂')
    ax.set_title('(d) 相平面轨迹')
    ax.legend(fontsize=9); ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('sim_backstepping.png', dpi=150, bbox_inches='tight')
    plt.close('all')
    print("图表已保存: sim_backstepping.png")



if __name__ == '__main__':
    main()
