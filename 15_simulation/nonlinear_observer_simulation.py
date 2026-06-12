"""
非线性观测器仿真 - EKF / UKF / 粒子滤波对比
"""
import numpy as np
import matplotlib.pyplot as plt

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
plt.rcParams['axes.unicode_minus'] = False

np.random.seed(42)

# ========== 非线性系统模型 ==========
def f_nonlinear(x, dt):
    """非线性状态转移: 匀速转弯模型"""
    # x = [px, py, v, theta, omega]
    px, py, v, theta, omega = x
    if abs(omega) < 1e-6:
        px_new = px + v * np.cos(theta) * dt
        py_new = py + v * np.sin(theta) * dt
    else:
        px_new = px + v/omega * (np.sin(theta + omega*dt) - np.sin(theta))
        py_new = py - v/omega * (np.cos(theta + omega*dt) - np.cos(theta))
    return np.array([px_new, py_new, v, theta + omega*dt, omega])

def h_nonlinear(x):
    """非线性观测: 距离+角度"""
    px, py, v, theta, omega = x
    r = np.sqrt(px**2 + py**2)
    angle = np.arctan2(py, px)
    return np.array([r, angle])

def jacobian_F(x, dt):
    """状态转移雅可比矩阵 (EKF用)"""
    px, py, v, theta, omega = x
    F = np.eye(5)
    if abs(omega) < 1e-6:
        F[0, 2] = np.cos(theta) * dt
        F[0, 3] = -v * np.sin(theta) * dt
        F[1, 2] = np.sin(theta) * dt
        F[1, 3] = v * np.cos(theta) * dt
        F[3, 4] = dt
    else:
        F[0, 2] = (np.sin(theta + omega*dt) - np.sin(theta)) / omega
        F[0, 3] = v/omega * (np.cos(theta + omega*dt) - np.cos(theta))
        F[0, 4] = -v/omega**2 * (np.sin(theta + omega*dt) - np.sin(theta)) + v*dt/omega * np.cos(theta + omega*dt)
        F[1, 2] = -(np.cos(theta + omega*dt) - np.cos(theta)) / omega
        F[1, 3] = v/omega * (np.sin(theta + omega*dt) - np.sin(theta))
        F[1, 4] = v/omega**2 * (np.cos(theta + omega*dt) - np.cos(theta)) + v*dt/omega * np.sin(theta + omega*dt)
        F[3, 4] = dt
    return F

def jacobian_H(x):
    """观测雅可比矩阵 (EKF用)"""
    px, py = x[0], x[1]
    r = np.sqrt(px**2 + py**2) + 1e-10
    H = np.zeros((2, 5))
    H[0, 0] = px / r
    H[0, 1] = py / r
    H[1, 0] = -py / (r**2)
    H[1, 1] = px / (r**2)
    return H

# ========== EKF ==========
class ExtendedKalmanFilter:
    def __init__(self, x0, P0, Q, R, dt):
        self.x = x0.copy()
        self.P = P0.copy()
        self.Q = Q
        self.R = R
        self.dt = dt

    def predict(self):
        F = jacobian_F(self.x, self.dt)
        self.x = f_nonlinear(self.x, self.dt)
        self.P = F @ self.P @ F.T + self.Q

    def update(self, z):
        H = jacobian_H(self.x)
        y = z - h_nonlinear(self.x)
        # 角度归一化
        y[1] = np.arctan2(np.sin(y[1]), np.cos(y[1]))
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(5) - K @ H) @ self.P

# ========== UKF ==========
class UnscentedKalmanFilter:
    def __init__(self, x0, P0, Q, R, dt, alpha=1e-3, beta=2, kappa=0):
        self.x = x0.copy()
        self.P = P0.copy()
        self.Q = Q
        self.R = R
        self.dt = dt
        self.n = len(x0)
        self.alpha = alpha
        self.beta = beta
        self.kappa = kappa
        self.lam = alpha**2 * (self.n + kappa) - self.n
        self.gamma = np.sqrt(self.n + self.lam)
        self.Wm = np.zeros(2*self.n + 1)
        self.Wc = np.zeros(2*self.n + 1)
        self.Wm[0] = self.lam / (self.n + self.lam)
        self.Wc[0] = self.lam / (self.n + self.lam) + (1 - alpha**2 + beta)
        for i in range(1, 2*self.n + 1):
            self.Wm[i] = 1 / (2 * (self.n + self.lam))
            self.Wc[i] = 1 / (2 * (self.n + self.lam))

    def _sigma_points(self):
        pts = np.zeros((2*self.n+1, self.n))
        pts[0] = self.x
        try:
            sqrt_P = np.linalg.cholesky((self.n + self.lam) * self.P)
        except np.linalg.LinAlgError:
            sqrt_P = np.linalg.cholesky((self.n + self.lam) * self.P + 1e-6*np.eye(self.n))
        for i in range(self.n):
            pts[i+1] = self.x + sqrt_P[i]
            pts[self.n+i+1] = self.x - sqrt_P[i]
        return pts

    def predict(self):
        pts = self._sigma_points()
        pts_pred = np.array([f_nonlinear(p, self.dt) for p in pts])
        self.x = np.sum(self.Wm[:, None] * pts_pred, axis=0)
        self.P = self.Q.copy()
        for i in range(2*self.n+1):
            d = pts_pred[i] - self.x
            self.P += self.Wc[i] * np.outer(d, d)

    def update(self, z):
        pts = self._sigma_points()
        pts_pred = np.array([f_nonlinear(p, self.dt) for p in pts])
        y_pts = np.array([h_nonlinear(p) for p in pts_pred])
        y_mean = np.sum(self.Wm[:, None] * y_pts, axis=0)
        Pyy = self.R.copy()
        Pxy = np.zeros((self.n, 2))
        for i in range(2*self.n+1):
            dy = y_pts[i] - y_mean
            dy[1] = np.arctan2(np.sin(dy[1]), np.cos(dy[1]))
            dx = pts_pred[i] - self.x
            Pyy += self.Wc[i] * np.outer(dy, dy)
            Pxy += self.Wc[i] * np.outer(dx, dy)
        K = Pxy @ np.linalg.inv(Pyy)
        innov = z - y_mean
        innov[1] = np.arctan2(np.sin(innov[1]), np.cos(innov[1]))
        self.x = self.x + K @ innov
        self.P = self.P - K @ Pyy @ K.T

# ========== 粒子滤波 ==========
class ParticleFilter:
    def __init__(self, n_particles, x0, Q, R, dt):
        self.n = n_particles
        self.dt = dt
        self.Q = Q
        self.R = R
        # 初始化粒子
        self.particles = x0 + np.random.randn(n_particles, 5) * 0.5
        self.weights = np.ones(n_particles) / n_particles

    def predict(self):
        for i in range(self.n):
            noise = np.random.multivariate_normal(np.zeros(5), self.Q)
            self.particles[i] = f_nonlinear(self.particles[i], self.dt) + noise

    def update(self, z):
        for i in range(self.n):
            y_pred = h_nonlinear(self.particles[i])
            innov = z - y_pred
            innov[1] = np.arctan2(np.sin(innov[1]), np.cos(innov[1]))
            self.weights[i] *= np.exp(-0.5 * innov @ np.linalg.inv(self.R) @ innov)
        self.weights += 1e-300
        self.weights /= np.sum(self.weights)

    def resample(self):
        """系统重采样"""
        if 1.0 / np.sum(self.weights**2) < self.n / 2:
            positions = (np.arange(self.n) + np.random.random()) / self.n
            cumsum = np.cumsum(self.weights)
            indices = np.searchsorted(cumsum, positions)
            self.particles = self.particles[indices]
            self.weights = np.ones(self.n) / self.n

    @property
    def estimate(self):
        return np.average(self.particles, weights=self.weights, axis=0)

# ========== 仿真主程序 ==========
def run_simulation():
    print("=" * 60)
    print("  非线性观测器仿真 - EKF/UKF/粒子滤波对比")
    print("=" * 60)

    # 系统参数
    dt = 0.1
    T = 50
    N = int(T / dt)

    # 真实初始状态 [px, py, v, theta, omega]
    x_true = np.array([0.0, 0.0, 5.0, np.pi/4, 0.1])

    # 噪声
    Q_true = np.diag([0.1, 0.1, 0.05, 0.01, 0.01])
    R_true = np.diag([1.0, 0.01])  # 距离和角度测量噪声

    # 生成真实轨迹
    x_history_true = [x_true.copy()]
    z_history = []

    for i in range(N):
        x_true = f_nonlinear(x_true, dt) + np.random.multivariate_normal(np.zeros(5), Q_true)
        z = h_nonlinear(x_true) + np.random.multivariate_normal(np.zeros(2), R_true)
        x_history_true.append(x_true.copy())
        z_history.append(z)

    x_true = np.array(x_history_true)
    z_history = np.array(z_history)

    # 初始化观测器
    x0_est = np.array([1.0, 1.0, 4.0, np.pi/3, 0.05])
    P0 = np.diag([10, 10, 1, 0.5, 0.1])
    Q_est = Q_true * 2
    R_est = R_true * 1.5

    ekf = ExtendedKalmanFilter(x0_est, P0, Q_est, R_est, dt)
    ukf = UnscentedKalmanFilter(x0_est, P0, Q_est, R_est, dt)
    pf = ParticleFilter(500, x0_est, Q_est, R_est, dt)

    ekf_history = [x0_est.copy()]
    ukf_history = [x0_est.copy()]
    pf_history = [x0_est.copy()]

    for i in range(N):
        ekf.predict(); ekf.update(z_history[i])
        ukf.predict(); ukf.update(z_history[i])
        pf.predict(); pf.update(z_history[i]); pf.resample()

        ekf_history.append(ekf.x.copy())
        ukf_history.append(ukf.x.copy())
        pf_history.append(pf.estimate.copy())

    ekf_hist = np.array(ekf_history)
    ukf_hist = np.array(ukf_history)
    pf_hist = np.array(pf_history)
    t = np.arange(N+1) * dt

    # 绘图
    fig, axes = plt.subplots(3, 2, figsize=(16, 14))
    fig.suptitle('非线性观测器对比: EKF vs UKF vs 粒子滤波', fontsize=14, fontweight='bold')

    labels = ['px', 'py', 'v', 'θ', 'ω']
    true_idx = [0, 1, 2, 3, 4]

    for idx in range(5):
        ax = axes.flat[idx]
        ax.plot(t, x_true[:, idx], 'k-', linewidth=2, label='真实值')
        ax.plot(t, ekf_hist[:, idx], 'b--', linewidth=1.2, label='EKF')
        ax.plot(t, ukf_hist[:, idx], 'r-.', linewidth=1.2, label='UKF')
        ax.plot(t, pf_hist[:, idx], 'g:', linewidth=1.2, label='PF')
        ax.set_title(f'状态 {labels[idx]}', fontsize=12)
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    # 轨迹对比
    ax = axes.flat[5]
    ax.plot(x_true[:, 0], x_true[:, 1], 'k-', linewidth=2, label='真实轨迹')
    ax.plot(ekf_hist[:, 0], ekf_hist[:, 1], 'b--', linewidth=1.2, label='EKF')
    ax.plot(ukf_hist[:, 0], ukf_hist[:, 1], 'r-.', linewidth=1.2, label='UKF')
    ax.plot(pf_hist[:, 0], pf_hist[:, 1], 'g:', linewidth=1.2, label='PF')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('轨迹估计对比', fontsize=12)
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')

    plt.tight_layout()
    plt.savefig('nonlinear_observer_comparison_result.png', dpi=150)
    plt.close()

    # RMSE对比
    fig, ax = plt.subplots(figsize=(12, 6))
    rmse_ekf = np.sqrt(np.mean((ekf_hist - x_true)**2, axis=0))
    rmse_ukf = np.sqrt(np.mean((ukf_hist - x_true)**2, axis=0))
    rmse_pf = np.sqrt(np.mean((pf_hist - x_true)**2, axis=0))

    x_pos = np.arange(5)
    width = 0.25
    ax.bar(x_pos - width, rmse_ekf, width, label='EKF', color='royalblue')
    ax.bar(x_pos, rmse_ukf, width, label='UKF', color='tomato')
    ax.bar(x_pos + width, rmse_pf, width, label='PF', color='forestgreen')
    ax.set_xticks(x_pos)
    ax.set_xticklabels(labels)
    ax.set_ylabel('RMSE')
    ax.set_title('各状态RMSE对比', fontsize=14, fontweight='bold')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('nonlinear_observer_rmse_result.png', dpi=150)
    plt.close()

    print(f"  EKF RMSE: {np.mean(rmse_ekf):.4f}")
    print(f"  UKF RMSE: {np.mean(rmse_ukf):.4f}")
    print(f"  PF  RMSE: {np.mean(rmse_pf):.4f}")
    print("✅ 非线性观测器对比完成")


if __name__ == '__main__':
    run_simulation()
