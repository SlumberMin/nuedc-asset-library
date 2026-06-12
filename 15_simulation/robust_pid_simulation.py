"""
鲁棒PID仿真 (Robust PID Simulation)
=====================================
参数不确定性下的PID控制性能：
- 标称模型 vs 多组随机扰动模型
- 鲁棒PID设计：确保在参数变化范围内系统稳定
- 蒙特卡洛分析

Author: nuedc-asset-library
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
matplotlib.rcParams['axes.unicode_minus'] = False


class UncertainPlant:
    """含不确定性的被控对象: G(s) = K*e^(-theta*s) / (tau*s + 1)"""

    def __init__(self, K_nom, tau_nom, theta_nom, uncertainty=0.3, seed=None):
        if seed is not None:
            np.random.seed(seed)
        self.K = K_nom * (1 + uncertainty * (2 * np.random.rand() - 1))
        self.tau = tau_nom * (1 + uncertainty * (2 * np.random.rand() - 1))
        self.theta = theta_nom * (1 + uncertainty * (2 * np.random.rand() - 1))
        self.y = 0.0
        self.K_nom = K_nom
        self.tau_nom = tau_nom
        self.theta_nom = theta_nom

    def step(self, u, dt, u_history):
        delay_steps = max(1, int(self.theta / dt))
        idx = max(0, len(u_history) - delay_steps)
        u_delayed = u_history[idx] if idx < len(u_history) else 0
        self.y += dt / self.tau * (self.K * u_delayed - self.y)
        return self.y

    def reset(self):
        self.y = 0.0

    def __repr__(self):
        return f"K={self.K:.3f}, τ={self.tau:.3f}, θ={self.theta:.3f}"


def simulate_closed_loop(pid_gains, plant, ref, dt, T):
    """闭环仿真"""
    N = int(T / dt)
    Kp, Ki, Kd = pid_gains
    y = np.zeros(N)
    u = np.zeros(N)
    e_int = 0.0
    e_prev = 0.0

    for i in range(N):
        error = ref[i] - y[i]
        e_int += error * dt
        de = (error - e_prev) / dt if i > 0 else 0
        e_prev = error

        u[i] = Kp * error + Ki * e_int + Kd * de
        u[i] = np.clip(u[i], -100, 100)

        y[i] = plant.step(u[i], dt, u[:i+1].tolist())

    return y, u


def compute_iae(y, ref, dt):
    return np.sum(np.abs(ref - y)) * dt


def monte_carlo_analysis(pid_gains, K_nom, tau_nom, theta_nom, uncertainty, n_trials, dt, T, ref):
    """蒙特卡洛分析"""
    N = int(T / dt)
    y_all = np.zeros((n_trials, N))
    iae_all = np.zeros(n_trials)

    for trial in range(n_trials):
        plant = UncertainPlant(K_nom, tau_nom, theta_nom, uncertainty, seed=trial)
        y, _ = simulate_closed_loop(pid_gains, plant, ref, dt, T)
        y_all[trial] = y
        iae_all[trial] = compute_iae(y, ref, dt)

    return y_all, iae_all


if __name__ == '__main__':
    dt = 0.01
    T = 20.0
    N = int(T / dt)
    t = np.linspace(0, T, N)
    ref = np.ones(N)

    K_nom, tau_nom, theta_nom = 1.0, 2.0, 0.5
    uncertainty = 0.3
    n_trials = 50

    # --- 标称PID（Z-N法） ---
    Kp_nom = 1.2 * tau_nom / (K_nom * theta_nom)
    Ki_nom = Kp_nom / (2.0 * theta_nom)
    Kd_nom = Kp_nom * 0.5 * theta_nom
    pid_nom = (Kp_nom, Ki_nom, Kd_nom)

    # --- 鲁棒PID（保守整定） ---
    Kp_rob = 0.7 * Kp_nom
    Ki_rob = 0.5 * Ki_nom
    Kd_rob = 0.8 * Kd_nom
    pid_rob = (Kp_rob, Ki_rob, Kd_rob)

    print(f"标称PID: Kp={Kp_nom:.3f}, Ki={Ki_nom:.3f}, Kd={Kd_nom:.3f}")
    print(f"鲁棒PID: Kp={Kp_rob:.3f}, Ki={Ki_rob:.3f}, Kd={Kd_rob:.3f}")

    # 蒙特卡洛分析
    y_nom, iae_nom = monte_carlo_analysis(pid_nom, K_nom, tau_nom, theta_nom, uncertainty, n_trials, dt, T, ref)
    y_rob, iae_rob = monte_carlo_analysis(pid_rob, K_nom, tau_nom, theta_nom, uncertainty, n_trials, dt, T, ref)

    # --- 绘图 ---
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    # 标称PID响应带
    axes[0, 0].fill_between(t, y_nom.min(axis=0), y_nom.max(axis=0), alpha=0.3, color='blue', label='不确定范围')
    axes[0, 0].plot(t, y_nom.mean(axis=0), 'b-', lw=1.5, label='均值响应')
    axes[0, 0].plot(t, ref, 'k--', lw=1.5, label='参考')
    axes[0, 0].set_title(f'标称PID (蒙特卡洛 n={n_trials})')
    axes[0, 0].set_ylabel('输出')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)

    # 鲁棒PID响应带
    axes[0, 1].fill_between(t, y_rob.min(axis=0), y_rob.max(axis=0), alpha=0.3, color='red', label='不确定范围')
    axes[0, 1].plot(t, y_rob.mean(axis=0), 'r-', lw=1.5, label='均值响应')
    axes[0, 1].plot(t, ref, 'k--', lw=1.5, label='参考')
    axes[0, 1].set_title(f'鲁棒PID (蒙特卡洛 n={n_trials})')
    axes[0, 1].set_ylabel('输出')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)

    # IAE分布
    axes[1, 0].hist(iae_nom, bins=20, alpha=0.6, color='blue', label=f'标称PID (μ={iae_nom.mean():.2f})')
    axes[1, 0].hist(iae_rob, bins=20, alpha=0.6, color='red', label=f'鲁棒PID (μ={iae_rob.mean():.2f})')
    axes[1, 0].set_xlabel('IAE')
    axes[1, 0].set_ylabel('频次')
    axes[1, 0].set_title('IAE分布对比')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)

    # 性能对比
    metrics = ['IAE均值', 'IAE标准差', 'IAE最大值', '超调量%']
    overshoot_nom = np.max((y_nom.max(axis=1) - 1.0) * 100)
    overshoot_rob = np.max((y_rob.max(axis=1) - 1.0) * 100)
    vals_nom = [iae_nom.mean(), iae_nom.std(), iae_nom.max(), overshoot_nom]
    vals_rob = [iae_rob.mean(), iae_rob.std(), iae_rob.max(), overshoot_rob]

    x = np.arange(len(metrics))
    w = 0.35
    axes[1, 1].bar(x - w/2, vals_nom, w, color='blue', alpha=0.7, label='标称PID')
    axes[1, 1].bar(x + w/2, vals_rob, w, color='red', alpha=0.7, label='鲁棒PID')
    axes[1, 1].set_xticks(x)
    axes[1, 1].set_xticklabels(metrics)
    axes[1, 1].set_title('鲁棒性指标对比')
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3, axis='y')

    plt.tight_layout()
    plt.savefig('robust_pid_result.png', dpi=150, bbox_inches='tight')
    plt.close('all')

    print(f"\n{'指标':<16} {'标称PID':>12} {'鲁棒PID':>12}")
    print('-' * 42)
    for m, v1, v2 in zip(metrics, vals_nom, vals_rob):
        print(f"{m:<16} {v1:>12.4f} {v2:>12.4f}")
