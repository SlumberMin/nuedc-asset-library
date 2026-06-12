"""
卡尔曼滤波仿真 - 一维/二维
适用于电赛传感器数据融合、信号去噪等场景

依赖: pip install numpy matplotlib
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# ============ 一维卡尔曼滤波 ============
class KalmanFilter1D:
    """
    一维卡尔曼滤波器
    状态模型: x(k) = x(k-1) + w, w ~ N(0, Q)
    观测模型: z(k) = x(k) + v, v ~ N(0, R)
    """
    def __init__(self, x0=0, P0=1, Q=0.01, R=0.1):
        self.x = x0    # 状态估计
        self.P = P0     # 估计协方差
        self.Q = Q      # 过程噪声
        self.R = R      # 观测噪声

    def predict(self):
        self.P += self.Q
        return self.x

    def update(self, z):
        K = self.P / (self.P + self.R)  # 卡尔曼增益
        self.x = self.x + K * (z - self.x)
        self.P = (1 - K) * self.P
        return self.x

# ============ 二维卡尔曼滤波(位置+速度) ============
class KalmanFilter2D:
    """
    二维卡尔曼滤波器: 跟踪位置和速度
    状态: [position, velocity]
    观测: [position]
    """
    def __init__(self, dt=1.0, q=0.1, r=1.0):
        self.dt = dt
        # 状态转移矩阵
        self.F = np.array([[1, dt],
                           [0, 1]])
        # 观测矩阵
        self.H = np.array([[1, 0]])
        # 过程噪声协方差
        self.Q = q * np.array([[dt**4/4, dt**3/2],
                                [dt**3/2, dt**2]])
        # 观测噪声协方差
        self.R = np.array([[r]])
        # 初始状态
        self.x = np.array([[0], [0]])
        self.P = np.eye(2) * 100

    def predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        return self.x.flatten()

    def update(self, z):
        z = np.array([[z]])
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(2) - K @ self.H) @ self.P
        return self.x.flatten()

# ============ 一维仿真: 传感器去噪 ============
def simulate_1d():
    np.random.seed(42)
    dt = 0.01
    steps = 1000
    t = np.arange(steps) * dt

    # 真实信号: 正弦 + 趋势
    true_signal = 2.0 * np.sin(2 * np.pi * 0.5 * t) + 0.01 * t
    # 带噪声的观测
    noise_std = 0.5
    measurements = true_signal + np.random.randn(steps) * noise_std

    # 不同参数的卡尔曼滤波
    configs = [
        ('低Q高R(信任模型)', 0.001, 1.0),
        ('平衡参数', 0.01, 0.25),
        ('高Q低R(信任观测)', 0.1, 0.01),
    ]

    fig, axes = plt.subplots(3, 1, figsize=(12, 10))

    for idx, (label, Q, R) in enumerate(configs):
        kf = KalmanFilter1D(x0=0, P0=1, Q=Q, R=R)
        filtered = []
        for z in measurements:
            kf.predict()
            filtered.append(kf.update(z))
        filtered = np.array(filtered)

        axes[idx].plot(t, true_signal, 'g-', label='真实信号', linewidth=1.5)
        axes[idx].plot(t, measurements, 'r.', label='观测值', markersize=1, alpha=0.3)
        axes[idx].plot(t, filtered, 'b-', label=f'卡尔曼滤波({label})', linewidth=1.2)
        axes[idx].set_ylabel('幅值')
        axes[idx].legend(fontsize=8)
        axes[idx].grid(True, alpha=0.3)
        axes[idx].set_title(f'一维卡尔曼滤波 - {label} (Q={Q}, R={R})')

        err = np.sqrt(np.mean((filtered - true_signal)**2))
        print(f'{label}: RMSE={err:.4f}')

    axes[-1].set_xlabel('时间 (s)')
    plt.tight_layout()
    plt.savefig('kalman_1d.png', dpi=150)
    plt.close('all')

# ============ 二维仿真: 位置+速度跟踪 ============
def simulate_2d():
    np.random.seed(42)
    dt = 0.1
    steps = 200
    t = np.arange(steps) * dt

    # 真实轨迹: 匀加速运动
    true_pos = 0.5 * 2.0 * t**2  # x = 0.5*a*t^2, a=2
    true_vel = 2.0 * t

    # 观测: 位置, 带噪声
    pos_noise_std = 5.0
    pos_measurements = true_pos + np.random.randn(steps) * pos_noise_std

    # 卡尔曼滤波
    kf = KalmanFilter2D(dt=dt, q=0.5, r=pos_noise_std**2)
    est_pos, est_vel = [], []
    for z in pos_measurements:
        kf.predict()
        state = kf.update(z)
        est_pos.append(state[0])
        est_vel.append(state[1])

    est_pos = np.array(est_pos)
    est_vel = np.array(est_vel)

    fig, axes = plt.subplots(2, 1, figsize=(12, 8))

    axes[0].plot(t, true_pos, 'g-', label='真实位置', linewidth=1.5)
    axes[0].plot(t, pos_measurements, 'r.', label='位置观测', markersize=2, alpha=0.4)
    axes[0].plot(t, est_pos, 'b-', label='卡尔曼估计位置', linewidth=1.2)
    axes[0].set_ylabel('位置 (m)')
    axes[0].set_title('二维卡尔曼滤波 - 位置和速度估计')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t, true_vel, 'g-', label='真实速度', linewidth=1.5)
    axes[1].plot(t, est_vel, 'b-', label='卡尔曼估计速度', linewidth=1.2)
    axes[1].set_xlabel('时间 (s)')
    axes[1].set_ylabel('速度 (m/s)')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('kalman_2d.png', dpi=150)
    plt.close('all')

    pos_rmse = np.sqrt(np.mean((est_pos - true_pos)**2))
    vel_rmse = np.sqrt(np.mean((est_vel - true_vel)**2))
    print(f'位置RMSE: {pos_rmse:.4f}m, 观测RMSE: {np.sqrt(np.mean((pos_measurements - true_pos)**2)):.4f}m')
    print(f'速度RMSE: {vel_rmse:.4f}m/s')

if __name__ == '__main__':
    print("=== 一维卡尔曼滤波仿真 ===")
    simulate_1d()
    print("\n=== 二维卡尔曼滤波仿真 ===")
    simulate_2d()
