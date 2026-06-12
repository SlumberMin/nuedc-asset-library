"""
重复控制仿真 (Repetitive Control Simulation)
==============================================
基于内模原理，对周期信号实现零稳态误差跟踪。
适用场景：逆变器输出、有源滤波器等周期信号跟踪场合。

特性：
- 周期内模 1/(1 - e^{-sT}) 离散化
- 收敛曲线展示
- 与普通PID对比
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import rcParams

# ============================================================
# 中文显示设置
# ============================================================


def main():
    rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'STHeiti']
    rcParams['axes.unicode_minus'] = False

    # ============================================================
    # 系统参数
    # ============================================================
    dt = 1e-4          # 仿真步长 (s)
    T_sim = 0.5        # 仿真总时长 (s)
    T_period = 0.02    # 参考信号周期 (50Hz)
    N = int(T_sim / dt)
    N_period = int(T_period / dt)  # 一个周期采样点数

    # 被控对象: G(s) = 1 / (s^2 + 2s + 1)，二阶系统
    # 离散化（前向欧拉）
    a1, a2 = 2.0, 1.0  # 系统参数

    # 重复控制器参数
    kr = 0.15          # 重复控制增益
    Q = 0.95           # Q滤波器（低通，增强鲁棒性）

    # PID参数（对比用）
    Kp, Ki, Kd = 50.0, 800.0, 0.5

    # ============================================================
    # 参考信号：50Hz 正弦
    # ============================================================
    t = np.arange(N) * dt
    ref = np.sin(2 * np.pi / T_period * t)

    # ============================================================
    # 重复控制仿真
    # ============================================================
    def run_repetitive_control():
        """重复控制：误差经一个周期延迟后累加修正"""
        x1 = np.zeros(N)  # 状态1
        x2 = np.zeros(N)  # 状态2
        y = np.zeros(N)   # 输出
        u = np.zeros(N)   # 控制量
        e = np.zeros(N)   # 误差
        # 重复控制缓冲区（延迟一个周期）
        buffer = np.zeros(N_period)

        for k in range(N - 1):
            e[k] = ref[k] - y[k]
            # 重复控制器: u_r = kr * [Q * buffer[last] + e[k]]
            idx_delay = k % N_period
            u_rep = kr * (Q * buffer[idx_delay] + e[k])
            buffer[idx_delay] = Q * buffer[idx_delay] + e[k]
            # 加入前馈补偿
            u[k] = u_rep + 0.5 * ref[k]
            # 被控对象离散仿真（前向欧拉）
            dx1 = x2[k]
            dx2 = -a2 * x1[k] - a1 * x2[k] + u[k]
            x1[k+1] = x1[k] + dt * dx1
            x2[k+1] = x2[k] + dt * dx2
            y[k+1] = x1[k+1]
        e[-1] = ref[-1] - y[-1]
        return y, u, e

    def run_pid_control():
        """普通PID控制（对比）"""
        x1 = np.zeros(N)
        x2 = np.zeros(N)
        y = np.zeros(N)
        u = np.zeros(N)
        e = np.zeros(N)
        e_int = 0.0
        e_prev = 0.0

        for k in range(N - 1):
            e[k] = ref[k] - y[k]
            e_int += e[k] * dt
            e_der = (e[k] - e_prev) / dt
            u[k] = Kp * e[k] + Ki * e_int + Kd * e_der
            e_prev = e[k]
            dx1 = x2[k]
            dx2 = -a2 * x1[k] - a1 * x2[k] + u[k]
            x1[k+1] = x1[k] + dt * dx1
            x2[k+1] = x2[k] + dt * dx2
            y[k+1] = x1[k+1]
        e[-1] = ref[-1] - y[-1]
        return y, u, e

    # ============================================================
    # 运行仿真
    # ============================================================
    y_rep, u_rep, e_rep = run_repetitive_control()
    y_pid, u_pid, e_pid = run_pid_control()

    # ============================================================
    # 性能指标计算
    # ============================================================
    def calc_metrics(e, t_arr):
        """计算跟踪性能指标"""
        steady_start = int(0.6 * len(e))  # 后40%视为稳态
        e_ss = e[steady_start:]
        rmse = np.sqrt(np.mean(e_ss**2))
        mae = np.mean(np.abs(e_ss))
        max_err = np.max(np.abs(e_ss))
        return rmse, mae, max_err

    rmse_rep, mae_rep, max_rep = calc_metrics(e_rep, t)
    rmse_pid, mae_pid, max_pid = calc_metrics(e_pid, t)

    print("=" * 55)
    print("        重复控制 vs PID 控制 — 性能对比")
    print("=" * 55)
    print(f"{'指标':<16} {'重复控制':>12} {'PID':>12}")
    print("-" * 55)
    print(f"{'RMSE (稳态)':<16} {rmse_rep:>12.6f} {rmse_pid:>12.6f}")
    print(f"{'MAE  (稳态)':<16} {mae_rep:>12.6f} {mae_pid:>12.6f}")
    print(f"{'最大误差(稳态)':<14} {max_rep:>12.6f} {max_pid:>12.6f}")
    print("=" * 55)

    # ============================================================
    # 绘图
    # ============================================================
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('重复控制仿真 — 周期信号跟踪', fontsize=15, fontweight='bold')

    # (a) 跟踪对比
    ax = axes[0, 0]
    ax.plot(t, ref, 'k--', linewidth=1.0, label='参考信号')
    ax.plot(t, y_rep, 'b-', linewidth=0.8, label='重复控制')
    ax.plot(t, y_pid, 'r-', linewidth=0.8, alpha=0.7, label='PID')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('幅值')
    ax.set_title('(a) 信号跟踪对比')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, alpha=0.3)

    # (b) 跟踪误差
    ax = axes[0, 1]
    ax.plot(t, e_rep, 'b-', linewidth=0.6, label='重复控制误差')
    ax.plot(t, e_pid, 'r-', linewidth=0.6, alpha=0.7, label='PID误差')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('误差')
    ax.set_title('(b) 跟踪误差')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, alpha=0.3)

    # (c) 收敛曲线（误差包络/滑动RMS）
    ax = axes[1, 0]
    window = N_period  # 一个周期窗口
    rms_rep = np.array([np.sqrt(np.mean(e_rep[max(0,i-window):i+1]**2)) for i in range(N)])
    rms_pid = np.array([np.sqrt(np.mean(e_pid[max(0,i-window):i+1]**2)) for i in range(N)])
    ax.plot(t, rms_rep, 'b-', linewidth=1.0, label='重复控制 RMS误差')
    ax.plot(t, rms_pid, 'r-', linewidth=1.0, alpha=0.7, label='PID RMS误差')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('滑动RMS误差')
    ax.set_title('(c) 误差收敛曲线')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, alpha=0.3)

    # (d) 控制量
    ax = axes[1, 1]
    ax.plot(t, u_rep, 'b-', linewidth=0.6, label='重复控制 u')
    ax.plot(t, u_pid, 'r-', linewidth=0.6, alpha=0.7, label='PID u')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('控制量')
    ax.set_title('(d) 控制量对比')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('sim_repetitive_control.png', dpi=150, bbox_inches='tight')
    plt.close('all')
    print("图表已保存: sim_repetitive_control.png")



if __name__ == '__main__':
    main()
