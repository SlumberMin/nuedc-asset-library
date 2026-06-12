"""
倒立摆仿真 - LQR最优控制
用法: python pendulum_simulation.py
"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.linalg import solve_continuous_are

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
plt.rcParams['axes.unicode_minus'] = False


class InvertedPendulum:
    """倒立摆模型 (小车-摆杆)
    状态: [x, dx, theta, dtheta]
    """
    def __init__(self, M=0.5, m=0.2, l=0.3, g=9.81, b=0.1):
        self.M = M      # 小车质量 (kg)
        self.m = m      # 摆杆质量 (kg)
        self.l = l      # 摆杆半长 (m)
        self.g = g
        self.b = b      # 小车摩擦系数
        self.state = np.array([0.0, 0.0, 0.0, 0.0])

    def dynamics(self, state, F):
        """返回状态导数 [dx, ddx, dtheta, ddtheta]"""
        x, dx, theta, dtheta = state
        M, m, l, g, b = self.M, self.m, self.l, self.g, self.b
        sin_t = np.sin(theta)
        cos_t = np.cos(theta)
        denom = M + m - m * cos_t**2
        ddx = (F - b * dx + m * l * dtheta**2 * sin_t - m * g * sin_t * cos_t) / denom
        ddtheta = (-cos_t * (F - b * dx + m * l * dtheta**2 * sin_t) +
                   (M + m) * g * sin_t) / (l * denom)
        return np.array([dx, ddx, dtheta, ddtheta])

    def update(self, F, dt):
        """RK4积分"""
        k1 = self.dynamics(self.state, F)
        k2 = self.dynamics(self.state + dt/2 * k1, F)
        k3 = self.dynamics(self.state + dt/2 * k2, F)
        k4 = self.dynamics(self.state + dt * k3, F)
        self.state += dt/6 * (k1 + 2*k2 + 2*k3 + k4)
        return self.state.copy()

    def reset(self, theta0=0.1):
        self.state = np.array([0.0, 0.0, theta0, 0.0])


def compute_lqr_gain(M=0.5, m=0.2, l=0.3, g=9.81, b=0.1):
    """线性化 + LQR增益计算"""
    # 线性化 (theta≈0): A, B矩阵
    denom = M * l
    A = np.array([
        [0, 1, 0, 0],
        [0, -b/denom * l, -m*g*l/denom * l, 0],
        [0, 0, 0, 1],
        [0, b/denom, (M+m)*g/denom, 0]
    ])
    # 重新计算线性化
    p = M + m
    A = np.array([
        [0, 1, 0, 0],
        [0, -b/p, -m*g/p, 0],
        [0, 0, 0, 1],
        [0, -b/(p*l), (M+m)*g/(p*l), 0]
    ])
    B = np.array([[0], [1/p], [0], [-1/(p*l)]])

    # LQR权重
    Q = np.diag([10, 1, 100, 1])  # 强调角度稳定
    R = np.array([[0.1]])

    # 解Riccati方程
    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.inv(R) @ B.T @ P
    return K, A, B


def simulate_lqr(theta0=0.2, duration=10.0, dt=0.01, noise_std=0.0):
    """LQR控制仿真"""
    pend = InvertedPendulum()
    pend.reset(theta0)
    K, A, B = compute_lqr_gain()

    # 平衡点
    x_eq = np.array([0.0, 0.0, 0.0, 0.0])

    log = {'t': [], 'x': [], 'theta': [], 'F': [], 'dx': []}
    for i in range(int(duration / dt)):
        t = i * dt
        state = pend.state.copy()
        if noise_std > 0:
            state += np.random.normal(0, noise_std, 4)
        F = -(K @ (state - x_eq))[0]
        F = max(-50, min(50, F))  # 力限幅
        pend.update(F, dt)
        log['t'].append(t)
        log['x'].append(pend.state[0])
        log['theta'].append(np.degrees(pend.state[2]))
        log['F'].append(F)
        log['dx'].append(pend.state[1])

    return {k: np.array(v) for k, v in log.items()}, K


def simulate_pole_placement(theta0=0.2, duration=10.0, dt=0.01):
    """极点配置法对比"""
    pend = InvertedPendulum()
    pend.reset(theta0)
    K, A, B = compute_lqr_gain()
    # 使用不同增益模拟极点配置效果
    K_pp = K * 1.5  # 模拟更激进的控制

    log = {'t': [], 'x': [], 'theta': [], 'F': []}
    for i in range(int(duration / dt)):
        t = i * dt
        F = -(K_pp @ pend.state)[0]
        F = max(-50, min(50, F))
        pend.update(F, dt)
        log['t'].append(t)
        log['x'].append(pend.state[0])
        log['theta'].append(np.degrees(pend.state[2]))
        log['F'].append(F)
    return {k: np.array(v) for k, v in log.items()}


def main():
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    # 不同初始角度
    for theta0 in [0.1, 0.2, 0.3, 0.5]:
        log, _ = simulate_lqr(theta0)
        axes[0, 0].plot(log['t'], log['theta'], label=f'θ₀={np.degrees(theta0):.1f}°')
    axes[0, 0].axhline(0, color='k', linestyle='--')
    axes[0, 0].set_title('不同初始角度的LQR控制')
    axes[0, 0].set_ylabel('摆角 (°)')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)

    # 带噪声
    log_clean, _ = simulate_lqr(0.2, noise_std=0)
    log_noisy, _ = simulate_lqr(0.2, noise_std=0.02)
    axes[0, 1].plot(log_clean['t'], log_clean['theta'], label='无噪声')
    axes[0, 1].plot(log_noisy['t'], log_noisy['theta'], label='有噪声', alpha=0.7)
    axes[0, 1].set_title('噪声鲁棒性')
    axes[0, 1].set_ylabel('摆角 (°)')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)

    # 小车位移
    log, _ = simulate_lqr(0.3)
    axes[1, 0].plot(log['t'], log['x'] * 100)
    axes[1, 0].set_title('小车位移')
    axes[1, 0].set_ylabel('位移 (cm)')
    axes[1, 0].set_xlabel('时间 (s)')
    axes[1, 0].grid(True, alpha=0.3)

    # 控制力
    axes[1, 1].plot(log['t'], log['F'])
    axes[1, 1].set_title('控制力')
    axes[1, 1].set_ylabel('力 (N)')
    axes[1, 1].set_xlabel('时间 (s)')
    axes[1, 1].grid(True, alpha=0.3)

    K, A, B = compute_lqr_gain()
    plt.suptitle(f'倒立摆LQR仿真  K=[{", ".join(f"{k:.2f}" for k in K[0])}]',
                 fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig('pendulum_result.png', dpi=150, bbox_inches='tight')
    plt.close('all')

    print("LQR增益矩阵 K:", K)
    print("仿真结果已保存: pendulum_result.png")


if __name__ == '__main__':
    main()
