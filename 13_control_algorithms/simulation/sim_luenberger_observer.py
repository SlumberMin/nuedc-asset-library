"""
Luenberger观测器仿真 (Luenberger Observer Simulation)
==============================================
通过输出反馈重构系统全部状态，并与卡尔曼滤波器对比。

系统（二阶线性）:
  ẋ = Ax + Bu
  y = Cx + v   (v为测量噪声)

仅测量位置x₁，需估计速度x₂。
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
    dt = 1e-3
    T_sim = 5.0
    N = int(T_sim / dt)
    t = np.arange(N) * dt

    # ============================================================
    # 系统模型: 二阶质量-弹簧-阻尼
    #   ẋ₁ = x₂
    #   ẋ₂ = -k/m * x₁ - b/m * x₂ + 1/m * u
    # ============================================================
    m, k, b = 1.0, 4.0, 0.5  # 质量、弹簧刚度、阻尼系数
    A = np.array([[0, 1],
                  [-k/m, -b/m]])
    B = np.array([[0],
                  [1/m]])
    C = np.array([[1, 0]])  # 仅测量位置
    n = 2

    # 离散化 (ZOH)
    Ad = np.eye(n) + A * dt
    Bd = B * dt
    Cd = C

    # 输入信号: 正弦力
    u_input = 2.0 * np.sin(2 * np.pi * 0.5 * t)

    # 测量噪声
    noise_std = 0.05
    np.random.seed(42)
    v_noise = noise_std * np.random.randn(N)

    # ============================================================
    # Luenberger 观测器
    # ============================================================
    # 观测器增益: 期望极点 p = [-10, -12]
    p_desired = np.array([-10.0, -12.0])
    # 利用极点配置: det(zI - (Ad - L*C)) = 0
    # 连续域: acker(A', C', p_desired)' → L
    # 手动计算: L = [l1; l2],  特征方程 s²+(0.5+l1)s+(4+l2)=0
    # (s+10)(s+12) = s²+22s+120 → l1=21.5, l2=116
    L = np.array([[21.5],
                  [116.0]])
    Ld = L * dt  # 离散近似

    def run_luenberger():
        """Luenberger全阶观测器"""
        x = np.zeros((n, N))      # 真实状态
        x_hat = np.zeros((n, N))  # 观测状态
        y = np.zeros(N)

        x[0, 0] = 0.5
        x[1, 0] = 0.0
        x_hat[:, 0] = np.array([0.0, 0.0])  # 观测器初始猜测

        for k in range(N - 1):
            # 真实系统
            x[:, k+1] = Ad @ x[:, k] + (Bd * u_input[k]).flatten()
            y[k] = (Cd @ x[:, k])[0] + v_noise[k]

            # Luenberger 观测器: x̂(k+1) = Ad*x̂(k) + Bd*u(k) + Ld*(y(k) - C*x̂(k))
            y_hat = (Cd @ x_hat[:, k])[0]
            x_hat[:, k+1] = Ad @ x_hat[:, k] + (Bd * u_input[k]).flatten() + (Ld * (y[k] - y_hat)).flatten()

        y[-1] = (Cd @ x[:, -1])[0] + v_noise[-1]
        return x, x_hat, y

    # ============================================================
    # 卡尔曼滤波器 (对比)
    # ============================================================
    def run_kalman():
        """离散卡尔曼滤波器"""
        # 过程噪声和测量噪声协方差
        Q_k = np.diag([1e-4, 1e-4])  # 过程噪声
        R_k = np.array([[noise_std**2]])  # 测量噪声

        x = np.zeros((n, N))
        x_kf = np.zeros((n, N))
        y = np.zeros(N)

        x[0, 0] = 0.5
        x[1, 0] = 0.0
        x_kf[:, 0] = np.array([0.0, 0.0])
        P = np.eye(n) * 0.1  # 初始协方差

        for k in range(N - 1):
            x[:, k+1] = Ad @ x[:, k] + (Bd * u_input[k]).flatten()
            y[k] = (Cd @ x[:, k])[0] + v_noise[k]

            # 预测
            x_pred = Ad @ x_kf[:, k] + (Bd * u_input[k]).flatten()
            P_pred = Ad @ P @ Ad.T + Q_k

            # 更新
            S = Cd @ P_pred @ Cd.T + R_k
            K = P_pred @ Cd.T @ np.linalg.inv(S)
            innovation = y[k] - (Cd @ x_pred)[0]
            x_kf[:, k+1] = x_pred + (K * innovation).flatten()
            P = (np.eye(n) - K @ Cd) @ P_pred

        y[-1] = (Cd @ x[:, -1])[0] + v_noise[-1]
        return x, x_kf, y

    # ============================================================
    # 运行仿真
    # ============================================================
    x_luen, x_hat_luen, y_luen = run_luenberger()
    x_kal, x_hat_kal, y_kal = run_kalman()

    # ============================================================
    # 性能指标
    # ============================================================
    def state_error(x_true, x_est, t_start=0.5):
        s = int(t_start / dt)
        e1 = x_true[0, s:] - x_est[0, s:]
        e2 = x_true[1, s:] - x_est[1, s:]
        rmse1 = np.sqrt(np.mean(e1**2))
        rmse2 = np.sqrt(np.mean(e2**2))
        return rmse1, rmse2

    rmse1_lb, rmse2_lb = state_error(x_luen, x_hat_luen)
    rmse1_kf, rmse2_kf = state_error(x_kal, x_hat_kal)

    print("=" * 60)
    print("    Luenberger观测器 vs 卡尔曼滤波 — 状态估计")
    print("=" * 60)
    print(f"{'指标':<20} {'Luenberger':>12} {'Kalman':>12}")
    print("-" * 60)
    print(f"{'x₁估计RMSE(稳态)':<18} {rmse1_lb:>12.6f} {rmse1_kf:>12.6f}")
    print(f"{'x₂估计RMSE(稳态)':<18} {rmse2_lb:>12.6f} {rmse2_kf:>12.6f}")
    print("=" * 60)

    # ============================================================
    # 绘图
    # ============================================================
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Luenberger观测器 vs 卡尔曼滤波 — 状态估计', fontsize=15, fontweight='bold')

    ax = axes[0, 0]
    ax.plot(t, x_luen[0], 'k-', lw=1.0, label='真实 x₁')
    ax.plot(t, y_luen, 'gray', lw=0.3, alpha=0.5, label='含噪测量 y')
    ax.plot(t, x_hat_luen[0], 'b--', lw=0.9, label='Luenberger x̂₁')
    ax.plot(t, x_hat_kal[0], 'r--', lw=0.9, label='Kalman x̂₁')
    ax.set_xlabel('时间 (s)'); ax.set_ylabel('位置 x₁')
    ax.set_title('(a) 位置状态估计')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    ax = axes[0, 1]
    ax.plot(t, x_luen[1], 'k-', lw=1.0, label='真实 x₂')
    ax.plot(t, x_hat_luen[1], 'b--', lw=0.9, label='Luenberger x̂₂')
    ax.plot(t, x_hat_kal[1], 'r--', lw=0.9, label='Kalman x̂₂')
    ax.set_xlabel('时间 (s)'); ax.set_ylabel('速度 x₂')
    ax.set_title('(b) 速度状态估计（未直接测量）')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    ax = axes[1, 0]
    ax.plot(t, x_luen[0] - x_hat_luen[0], 'b-', lw=0.5, label='Luenberger x₁误差')
    ax.plot(t, x_luen[0] - x_hat_kal[0], 'r-', lw=0.5, alpha=0.7, label='Kalman x₁误差')
    ax.set_xlabel('时间 (s)'); ax.set_ylabel('估计误差')
    ax.set_title('(c) 位置估计误差')
    ax.legend(fontsize=9); ax.grid(True, alpha=0.3)

    ax = axes[1, 1]
    ax.plot(t, x_luen[1] - x_hat_luen[1], 'b-', lw=0.5, label='Luenberger x₂误差')
    ax.plot(t, x_luen[1] - x_hat_kal[1], 'r-', lw=0.5, alpha=0.7, label='Kalman x₂误差')
    ax.set_xlabel('时间 (s)'); ax.set_ylabel('估计误差')
    ax.set_title('(d) 速度估计误差')
    ax.legend(fontsize=9); ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('sim_luenberger_observer.png', dpi=150, bbox_inches='tight')
    plt.close('all')
    print("图表已保存: sim_luenberger_observer.png")



if __name__ == '__main__':
    main()
