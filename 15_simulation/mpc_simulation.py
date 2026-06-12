"""
MPC模型预测控制仿真
适用于电赛小车轨迹跟踪、运动控制等场景

依赖: pip install numpy matplotlib scipy
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.optimize import minimize

# ============ 被控对象: 简单车辆模型 ============
class BicycleModel:
    """简化的自行车运动模型"""
    def __init__(self, dt=0.05):
        self.dt = dt
        self.x = 0
        self.y = 0
        self.theta = 0  # 航向角
        self.v = 0      # 速度

    def step(self, a, delta, v_max=3.0):
        """a: 加速度, delta: 转向角"""
        self.v += a * self.dt
        self.v = np.clip(self.v, 0, v_max)
        L = 0.3  # 轴距
        self.theta += (self.v / L) * np.tan(delta) * self.dt
        self.x += self.v * np.cos(self.theta) * self.dt
        self.y += self.v * np.sin(self.theta) * self.dt
        return self.x, self.y, self.theta, self.v

    def state(self):
        return np.array([self.x, self.y, self.theta, self.v])

# ============ MPC控制器 ============
class MPC:
    """模型预测控制器"""
    def __init__(self, N=20, dt=0.05):
        self.N = N          # 预测步长
        self.dt = dt
        self.nu = 2         # 控制量维度 [a, delta]
        self.L = 0.3        # 轴距
        # 权重矩阵
        self.Q = np.diag([10, 10, 5, 1])   # 状态权重 [x, y, theta, v]
        self.R = np.diag([0.1, 0.5])        # 控制权重
        self.Rd = np.diag([0.5, 1.0])       # 控制增量权重
        # 控制约束
        self.u_min = np.array([-2.0, -0.5])  # [a_min, delta_min]
        self.u_max = np.array([2.0, 0.5])    # [a_max, delta_max]

    def predict(self, x0, u_seq):
        """根据初始状态和控制序列预测未来状态"""
        states = [x0.copy()]
        x = x0.copy()
        for i in range(self.N):
            a = u_seq[i * self.nu]
            delta = u_seq[i * self.nu + 1]
            x_new = x.copy()
            x_new[3] += a * self.dt
            x_new[3] = np.clip(x_new[3], 0, 3.0)
            x_new[2] += (x_new[3] / self.L) * np.tan(delta) * self.dt
            x_new[0] += x_new[3] * np.cos(x_new[2]) * self.dt
            x_new[1] += x_new[3] * np.sin(x_new[2]) * self.dt
            x = x_new
            states.append(x.copy())
        return states

    def cost_function(self, u_flat, x0, ref_traj):
        """代价函数"""
        states = self.predict(x0, u_flat)
        u_seq = u_flat.reshape(self.N, self.nu)
        cost = 0
        for k in range(self.N):
            x_err = states[k] - ref_traj[k]
            cost += x_err @ self.Q @ x_err
            cost += u_seq[k] @ self.R @ u_seq[k]
            if k > 0:
                du = u_seq[k] - u_seq[k-1]
                cost += du @ self.Rd @ du
        # 终端代价
        x_err = states[-1] - ref_traj[-1]
        cost += 5 * x_err @ self.Q @ x_err
        return cost

    def solve(self, x0, ref_traj):
        """求解MPC优化问题"""
        u0 = np.zeros(self.N * self.nu)
        bounds = []
        for _ in range(self.N):
            bounds.append((self.u_min[0], self.u_max[0]))
            bounds.append((self.u_min[1], self.u_max[1]))

        result = minimize(self.cost_function, u0, args=(x0, ref_traj),
                         method='SLSQP', bounds=bounds,
                         options={'maxiter': 50, 'ftol': 1e-4})
        return result.x.reshape(self.N, self.nu)

# ============ 生成参考轨迹 ============
def generate_ref_trajectory(t_total, dt, pattern='circle'):
    N = int(t_total / dt)
    ref = np.zeros((N, 4))
    if pattern == 'circle':
        R = 2.0
        for i in range(N):
            t = i * dt
            angle = 0.5 * t
            ref[i] = [R * np.cos(angle), R * np.sin(angle), angle + np.pi/2, 1.0]
    elif pattern == 'figure8':
        R = 1.5
        omega = 0.4
        for i in range(N):
            t = i * dt
            w = omega * t
            dx = R * omega * np.cos(w)
            dy = R * omega * np.cos(2 * w)
            theta = np.arctan2(dy, dx)
            ref[i] = [R * np.sin(w), R * np.sin(2*w)/2, theta, 1.0]
    return ref

# ============ 仿真主循环 ============
def run_mpc_simulation(pattern='circle', t_total=15.0):
    dt = 0.05
    mpc = MPC(N=20, dt=dt)
    plant = BicycleModel(dt=dt)
    plant.v = 1.0

    ref_traj = generate_ref_trajectory(t_total, dt, pattern)
    steps = int(t_total / dt)

    history_x, history_y = [], []
    ref_x, ref_y = [], []

    for i in range(steps - mpc.N):
        x0 = plant.state()
        ref_window = ref_traj[i:i+mpc.N]
        u_opt = mpc.solve(x0, ref_window)
        a_cmd, delta_cmd = u_opt[0]
        plant.step(a_cmd, delta_cmd)

        history_x.append(plant.x)
        history_y.append(plant.y)
        ref_x.append(ref_traj[i, 0])
        ref_y.append(ref_traj[i, 1])

    return history_x, history_y, ref_x, ref_y, ref_traj

# ============ 可视化 ============
if __name__ == '__main__':
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    for idx, pattern in enumerate(['circle', 'figure8']):
        hx, hy, rx, ry, _ = run_mpc_simulation(pattern, t_total=15.0)
        ax = axes[idx]
        ax.plot(rx, ry, 'r--', label='参考轨迹', linewidth=1.5)
        ax.plot(hx, hy, 'b-', label='MPC跟踪', linewidth=1.2)
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title(f'MPC轨迹跟踪 - {"圆形" if pattern=="circle" else "8字形"}')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')

        # 计算跟踪误差
        n = min(len(hx), len(rx))
        err = np.sqrt((np.array(hx[:n]) - np.array(rx[:n]))**2 +
                      (np.array(hy[:n]) - np.array(ry[:n]))**2)
        print(f'{pattern}: 平均跟踪误差={np.mean(err):.4f}m, 最大误差={np.max(err):.4f}m')

    plt.tight_layout()
    plt.savefig('mpc_tracking.png', dpi=150)
    plt.close('all')
