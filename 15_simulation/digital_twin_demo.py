#!/usr/bin/env python3
"""
数字孪生Demo — 物理模型 + 数据驱动 + 实时同步
================================================
- 电机系统作为被控对象
- 物理模型（白盒）vs 数据驱动模型（黑盒）
- 参数漂移与在线辨识
- 实时同步与可视化
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rcParams
from collections import deque
rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
rcParams['axes.unicode_minus'] = False

# ============================================================
# 1. 真实系统（仿真用，模拟真实电机）
# ============================================================
class RealMotor:
    """真实电机系统（带非线性和参数漂移）"""
    def __init__(self, J=0.01, b=0.1, Kt=0.1, R=1.0, L=0.01):
        self.J = J       # 转动惯量
        self.b = b       # 粘性摩擦
        self.Kt = Kt     # 力矩常数
        self.R = R       # 电阻
        self.L = L       # 电感
        self.omega = 0.0 # 角速度
        self.current = 0.0
        self.dt = 0.001
        self.t = 0.0
        # 参数漂移
        self.drift_rate = 0.001

    def step(self, voltage):
        """真实电机一步响应"""
        self.t += self.dt
        # 参数随时间漂移
        J = self.J * (1 + self.drift_rate * self.t)
        b = self.b * (1 + 0.5 * self.drift_rate * self.t)

        # 电枢电流
        di = (voltage - self.R * self.current - self.Kt * self.omega) / self.L
        self.current += di * self.dt

        # 机械动力学（带非线性摩擦）
        friction = b * self.omega + 0.01 * np.sign(self.omega) + \
                   0.005 * self.omega**3  # 非线性摩擦
        d_omega = (self.Kt * self.current - friction) / J
        self.omega += d_omega * self.dt
        self.omega = max(0, self.omega)  # 单向旋转

        # 噪声
        omega_noisy = self.omega + np.random.normal(0, 0.05)

        return self.omega, omega_noisy, self.current

    def reset(self):
        self.omega = 0.0
        self.current = 0.0
        self.t = 0.0


# ============================================================
# 2. 物理孪生模型（白盒）
# ============================================================
class PhysicsModel:
    """基于物理方程的数字孪生"""
    def __init__(self, J=0.01, b=0.1, Kt=0.1, R=1.0, L=0.01):
        self.J = J
        self.b = b
        self.Kt = Kt
        self.R = R
        self.L = L
        self.omega = 0.0
        self.current = 0.0
        self.dt = 0.001

    def step(self, voltage):
        di = (voltage - self.R * self.current - self.Kt * self.omega) / self.L
        self.current += di * self.dt
        friction = self.b * self.omega  # 线性摩擦近似
        d_omega = (self.Kt * self.current - friction) / self.J
        self.omega += d_omega * self.dt
        self.omega = max(0, self.omega)
        return self.omega

    def update_params(self, J=None, b=None, Kt=None):
        if J is not None: self.J = J
        if b is not None: self.b = b
        if Kt is not None: self.Kt = Kt

    def reset(self):
        self.omega = 0.0
        self.current = 0.0


# ============================================================
# 3. 数据驱动孪生模型（黑盒 - 简化神经网络）
# ============================================================
class DataDrivenModel:
    """
    基于在线学习的数据驱动模型
    使用简化的线性回归 + 特征工程
    """
    def __init__(self, n_features=6, lr=0.001):
        self.weights = np.random.randn(n_features) * 0.01
        self.lr = lr
        self.omega = 0.0
        self.history = deque(maxlen=500)
        self.loss_history = []

    def _features(self, voltage, omega_prev):
        """特征提取: [omega, omega^2, omega^3, voltage, voltage*omega, 1]"""
        return np.array([
            omega_prev,
            omega_prev**2,
            omega_prev**3,
            voltage,
            voltage * omega_prev,
            1.0
        ])

    def predict(self, voltage, omega_prev):
        x = self._features(voltage, omega_prev)
        return max(0, self.weights @ x)

    def train_step(self, voltage, omega_prev, omega_true):
        """在线学习一步"""
        x = self._features(voltage, omega_prev)
        pred = self.weights @ x
        err = pred - omega_true
        # SGD更新
        self.weights -= self.lr * err * x
        self.loss_history.append(err**2)
        return pred

    def step(self, voltage):
        omega_new = self.predict(voltage, self.omega)
        self.omega = omega_new
        return omega_new

    def reset(self):
        self.omega = 0.0


# ============================================================
# 4. 参数辨识器（在线最小二乘）
# ============================================================
class OnlineParamIdentifier:
    """递推最小二乘参数辨识"""
    def __init__(self, n_params=3):
        self.n = n_params
        self.theta = np.array([0.01, 0.1, 0.1])  # [J, b, Kt]初始猜测
        self.P = np.eye(n_params) * 100  # 协方差
        self.R_forget = 0.998  # 遗忘因子

    def update(self, omega, d_omega, current, voltage):
        """递推更新"""
        # 简化：d_omega = (Kt*current - b*omega) / J
        # -> J*d_omega = Kt*current - b*omega
        # -> [d_omega, -omega, current] @ [J, b, Kt] = 0 (近似)
        # 用实际数据构建回归
        phi = np.array([d_omega, -omega, current])
        y = 0  # 近似残差

        # RLS
        K = self.P @ phi / (self.R_forget + phi @ self.P @ phi)
        self.theta = self.theta + K * (y - phi @ self.theta)
        self.P = (self.P - np.outer(K, phi @ self.P)) / self.R_forget

        # 约束参数物理意义
        self.theta[0] = np.clip(self.theta[0], 0.001, 1.0)  # J > 0
        self.theta[1] = np.clip(self.theta[1], 0.001, 1.0)  # b > 0
        self.theta[2] = np.clip(self.theta[2], 0.001, 1.0)  # Kt > 0

        return self.theta.copy()


# ============================================================
# 5. 数字孪生系统
# ============================================================
class DigitalTwinSystem:
    """数字孪生完整系统"""
    def __init__(self):
        self.real = RealMotor()
        self.physics = PhysicsModel()
        self.data_model = DataDrivenModel()
        self.identifier = OnlineParamIdentifier()
        self.dt = 0.001

    def run(self, T=10.0):
        n = int(T / self.dt)
        # 输入信号：阶跃+正弦+随机
        t = np.arange(n) * self.dt
        voltage = np.zeros(n)
        voltage[int(0.5/self.dt):int(3/self.dt)] = 12.0  # 阶跃
        voltage[int(3/self.dt):int(6/self.dt)] = 6.0 + 3.0*np.sin(2*np.pi*2*t[int(3/self.dt):int(6/self.dt)])
        voltage[int(6/self.dt):] = 8.0 + np.random.normal(0, 0.5, n - int(6/self.dt))

        # 记录
        rec = {
            't': t, 'voltage': voltage,
            'real_omega': np.zeros(n), 'measured_omega': np.zeros(n),
            'physics_omega': np.zeros(n), 'data_omega': np.zeros(n),
            'physics_error': np.zeros(n), 'data_error': np.zeros(n),
            'J_est': np.zeros(n), 'b_est': np.zeros(n), 'Kt_est': np.zeros(n),
        }

        prev_omega = 0.0
        for i in range(n):
            v = voltage[i]
            # 真实系统
            omega_true, omega_noisy, current = self.real.step(v)
            rec['real_omega'][i] = omega_true
            rec['measured_omega'][i] = omega_noisy

            # 物理模型
            omega_phys = self.physics.step(v)
            rec['physics_omega'][i] = omega_phys

            # 数据驱动模型
            self.data_model.train_step(v, prev_omega, omega_noisy)
            omega_data = self.data_model.step(v)
            rec['data_omega'][i] = omega_data

            # 参数辨识（每10步）
            if i > 0 and i % 10 == 0:
                d_omega = (omega_true - prev_omega) / (self.dt * 10)
                params = self.identifier.update(omega_true, d_omega, current, v)
                rec['J_est'][i] = params[0]
                rec['b_est'][i] = params[1]
                rec['Kt_est'][i] = params[2]

                # 用辨识参数更新物理模型（每100步）
                if i % 100 == 0:
                    self.physics.update_params(J=params[0], b=params[1], Kt=params[2])
            else:
                if i > 0:
                    rec['J_est'][i] = rec['J_est'][i-1]
                    rec['b_est'][i] = rec['b_est'][i-1]
                    rec['Kt_est'][i] = rec['Kt_est'][i-1]

            # 误差
            rec['physics_error'][i] = omega_true - omega_phys
            rec['data_error'][i] = omega_true - omega_data

            prev_omega = omega_true

        return rec


# ============================================================
# 6. 可视化
# ============================================================
def plot_results(rec):
    fig, axes = plt.subplots(3, 2, figsize=(15, 14))
    fig.suptitle('数字孪生Demo — 物理模型 + 数据驱动 + 实时同步', fontsize=15, fontweight='bold')

    t = rec['t']
    # 降采样显示
    ds = max(1, len(t) // 5000)

    # (a) 输入电压
    ax = axes[0, 0]
    ax.plot(t[::ds], rec['voltage'][::ds], 'k-', lw=0.8)
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('电压 (V)')
    ax.set_title('(a) 输入电压')
    ax.grid(True, alpha=0.3)

    # (b) 转速对比
    ax = axes[0, 1]
    ax.plot(t[::ds], rec['real_omega'][::ds], 'k-', lw=1.5, label='真实', alpha=0.8)
    ax.plot(t[::ds], rec['physics_omega'][::ds], 'b--', lw=1, label='物理模型')
    ax.plot(t[::ds], rec['data_omega'][::ds], 'r:', lw=1, label='数据驱动')
    ax.plot(t[::ds], rec['measured_omega'][::ds], color='gray', lw=0.3, alpha=0.3, label='测量值')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('角速度 (rad/s)')
    ax.set_title('(b) 转速对比')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # (c) 误差对比
    ax = axes[1, 0]
    ax.plot(t[::ds], rec['physics_error'][::ds], 'b-', lw=0.8, label='物理模型误差', alpha=0.7)
    ax.plot(t[::ds], rec['data_error'][::ds], 'r-', lw=0.8, label='数据驱动误差', alpha=0.7)
    ax.axhline(0, color='k', ls='--', lw=0.5)
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('误差 (rad/s)')
    ax.set_title('(c) 模型误差')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # (d) 误差统计
    ax = axes[1, 1]
    phys_rmse = np.sqrt(np.mean(rec['physics_error']**2))
    data_rmse = np.sqrt(np.mean(rec['data_error']**2))
    phys_mae = np.mean(np.abs(rec['physics_error']))
    data_mae = np.mean(np.abs(rec['data_error']))

    metrics = ['RMSE', 'MAE']
    phys_vals = [phys_rmse, phys_mae]
    data_vals = [data_rmse, data_mae]
    x = np.arange(len(metrics))
    ax.bar(x - 0.15, phys_vals, 0.3, label='物理模型', color='#2196F3', alpha=0.8)
    ax.bar(x + 0.15, data_vals, 0.3, label='数据驱动', color='#FF5722', alpha=0.8)
    ax.set_xticks(x)
    ax.set_xticklabels(metrics)
    ax.set_title('(d) 误差统计')
    ax.legend()
    ax.grid(True, alpha=0.3, axis='y')

    # (e) 参数辨识
    ax = axes[2, 0]
    ax.plot(t, rec['J_est'], lw=1, label='Ĵ (辨识值)')
    ax.axhline(0.01, color='b', ls='--', lw=1, label='J真值=0.01')
    ax.plot(t, rec['b_est'], lw=1, label='b̂ (辨识值)')
    ax.axhline(0.1, color='r', ls='--', lw=1, label='b真值=0.1')
    ax.plot(t, rec['Kt_est'], lw=1, label='K̂t (辨识值)')
    ax.axhline(0.1, color='g', ls='--', lw=1, label='Kt真值=0.1')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('参数值')
    ax.set_title('(e) 在线参数辨识（RLS）')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # (f) 同步误差时序
    ax = axes[2, 1]
    # 滑动窗口RMSE
    window = 1000
    phys_rmse_slide = np.array([np.sqrt(np.mean(rec['physics_error'][max(0,i-window):i]**2))
                                 for i in range(1, len(t), ds)])
    data_rmse_slide = np.array([np.sqrt(np.mean(rec['data_error'][max(0,i-window):i]**2))
                                 for i in range(1, len(t), ds)])
    ax.plot(t[1::ds][:len(phys_rmse_slide)], phys_rmse_slide, 'b-', lw=1, label='物理模型')
    ax.plot(t[1::ds][:len(data_rmse_slide)], data_rmse_slide, 'r-', lw=1, label='数据驱动')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('滑动RMSE')
    ax.set_title('(f) 同步精度（滑动窗口RMSE）')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('digital_twin_demo.png', dpi=150, bbox_inches='tight')
    plt.show()

    # 打印摘要
    print('\n' + '='*50)
    print('数字孪生性能摘要')
    print('='*50)
    print(f'物理模型  RMSE={phys_rmse:.4f}  MAE={phys_mae:.4f}')
    print(f'数据驱动  RMSE={data_rmse:.4f}  MAE={data_mae:.4f}')
    print(f'参数辨识最终值: J={rec["J_est"][-1]:.4f} (真值0.01), '
          f'b={rec["b_est"][-1]:.4f} (真值0.1), '
          f'Kt={rec["Kt_est"][-1]:.4f} (真值0.1)')


# ============================================================
if __name__ == '__main__':
    print('运行数字孪生仿真...')
    twin = DigitalTwinSystem()
    rec = twin.run(T=10.0)
    plot_results(rec)
    print('\n数字孪生Demo完成！')
