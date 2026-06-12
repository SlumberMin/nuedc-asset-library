#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
神经网络控制仿真 - BP/RBF/自适应神经网络控制器
用于电赛智能控制与非线性系统辨识
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
matplotlib.rcParams['axes.unicode_minus'] = False


# ======================== BP神经网络 ========================
class BPNeuralNetwork:
    """三层BP神经网络 (输入-隐层-输出)"""
    def __init__(self, n_input=3, n_hidden=10, n_output=1, lr=0.001):
        self.lr = lr
        # Xavier初始化
        self.W1 = np.random.randn(n_input, n_hidden) * np.sqrt(2.0 / n_input)
        self.b1 = np.zeros(n_hidden)
        self.W2 = np.random.randn(n_hidden, n_output) * np.sqrt(2.0 / n_hidden)
        self.b2 = np.zeros(n_output)
        # 动量
        self.dW1 = np.zeros_like(self.W1)
        self.dW2 = np.zeros_like(self.W2)
        self.momentum = 0.9

    def sigmoid(self, x):
        return 1.0 / (1.0 + np.exp(-np.clip(x, -500, 500)))

    def sigmoid_deriv(self, s):
        return s * (1 - s)

    def forward(self, x):
        x = np.atleast_2d(x)
        self.z1 = x @ self.W1 + self.b1
        self.a1 = self.sigmoid(self.z1)
        self.z2 = self.a1 @ self.W2 + self.b2
        return self.z2.flatten()

    def train(self, x, y_target):
        x = np.atleast_2d(x)
        y_target = np.atleast_1d(y_target)
        # 前向
        output = self.forward(x)
        error = y_target - output
        # 反向传播
        d_z2 = error.reshape(-1, 1)
        d_W2 = self.a1.T @ d_z2
        d_b2 = np.sum(d_z2, axis=0)
        d_a1 = d_z2 @ self.W2.T
        d_z1 = d_a1 * self.sigmoid_deriv(self.a1)
        d_W1 = x.T @ d_z1
        d_b1 = np.sum(d_z1, axis=0)
        # 更新(带动量)
        self.dW2 = self.momentum * self.dW2 + self.lr * d_W2
        self.W2 += self.dW2
        self.b2 += self.lr * d_b2
        self.dW1 = self.momentum * self.dW1 + self.lr * d_W1
        self.W1 += self.dW1
        self.b1 += self.lr * d_b1
        return np.sum(error**2)


class BPController:
    """BP神经网络前馈控制器"""
    def __init__(self):
        self.nn = BPNeuralNetwork(n_input=3, n_hidden=12, n_output=1, lr=0.002)
        self.prev_error = 0
        self.prev_prev_error = 0
        self.integral = 0
        self.Kp = 3.0
        self.Ki = 0.5

    def compute(self, error, dt):
        self.integral += error * dt
        nn_input = np.array([error, self.prev_error, self.integral])
        u_nn = self.nn.forward(nn_input)[0]
        # PID补偿 + NN前馈
        u_pid = self.Kp * error + self.Ki * self.integral
        u = u_pid + 2.0 * u_nn

        # 在线学习(用误差的负梯度方向)
        y_target = np.array([error * 0.5])  # 学习目标
        self.nn.train(nn_input, y_target)

        self.prev_prev_error = self.prev_error
        self.prev_error = error
        return np.clip(u, -15, 15)


# ======================== RBF神经网络 ========================
class RBFNetwork:
    """径向基函数神经网络"""
    def __init__(self, n_centers=15, n_input=2, lr=0.01):
        self.n_centers = n_centers
        self.lr = lr
        # 中心点(均匀分布在输入空间)
        self.centers = np.random.randn(n_centers, n_input) * 2
        self.sigmas = np.ones(n_centers) * 1.5  # 宽度
        self.weights = np.random.randn(n_centers) * 0.1
        self.dw = np.zeros(n_centers)

    def rbf(self, x, c, sigma):
        dist = np.sum((x - c)**2)
        return np.exp(-dist / (2 * sigma**2))

    def forward(self, x):
        x = np.atleast_1d(x)
        self.phi = np.array([self.rbf(x, c, s) for c, s in zip(self.centers, self.sigmas)])
        return np.dot(self.weights, self.phi)

    def train(self, x, y_target):
        output = self.forward(x)
        error = y_target - output
        # LMS更新权重
        self.dw = 0.9 * self.dw + self.lr * error * self.phi
        self.weights += self.dw
        # 自适应调整中心(竞争学习)
        dists = np.array([np.sum((x - c)**2) for c in self.centers])
        winner = np.argmin(dists)
        self.centers[winner] += 0.01 * (x - self.centers[winner])
        return error**2


class RBFController:
    """RBF神经网络自适应控制器"""
    def __init__(self):
        self.rbf = RBFNetwork(n_centers=20, n_input=2, lr=0.02)
        self.prev_error = 0
        self.Kp = 2.0

    def compute(self, error, dt):
        de = (error - self.prev_error) / dt if dt > 0 else 0
        nn_input = np.array([error, de])
        u_rbf = self.rbf.forward(nn_input)
        u_pid = self.Kp * error
        u = u_pid + u_rbf
        # 在线学习
        self.rbf.train(nn_input, error * 0.3)
        self.prev_error = error
        return np.clip(u, -15, 15)


# ======================== 自适应神经网络控制器 ========================
class AdaptiveNNController:
    """自适应神经网络控制器 (带Lyapunov稳定性保证)"""
    def __init__(self, n_hidden=15):
        self.n_input = 3  # [e, de, integral_e]
        self.n_hidden = n_hidden
        # 网络参数
        self.W = np.random.randn(n_hidden) * 0.1
        self.V = np.random.randn(self.n_input, n_hidden) * 0.5
        # 自适应律参数
        self.gamma_w = 0.5
        self.gamma_v = 0.1
        self.k = 3.0  # 反馈增益
        self.prev_error = 0
        self.integral = 0

    def gaussian(self, x):
        return np.exp(-np.sum(x**2) / 2)

    def forward(self, x):
        self.h = np.array([self.gaussian(x - self.V[:, j])
                          for j in range(self.n_hidden)])
        return np.dot(self.W, self.h)

    def compute(self, error, dt):
        de = (error - self.prev_error) / dt if dt > 0 else 0
        self.integral += error * dt
        x = np.array([error, de, self.integral])

        # 神经网络输出
        u_nn = self.forward(x)

        # 滑模面
        s = de + self.k * error

        # 自适应律 (基于Lyapunov)
        self.W += self.gamma_w * s * self.h * dt
        for j in range(self.n_hidden):
            diff = x - self.V[:, j]
            dh_dv = self.h[j] * diff
            self.V[:, j] += self.gamma_v * s * self.W[j] * dh_dv * dt

        # 鲁棒项
        rho = 0.5
        u_robust = -rho * np.sign(s)

        u = self.k * error + u_nn + u_robust
        self.prev_error = error
        return np.clip(u, -15, 15)


# ======================== 对比: PID ========================
class StandardPID:
    def __init__(self, Kp=5, Ki=1, Kd=2):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.integral = 0
        self.prev_error = 0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative


# ======================== 非线性被控对象 ========================
class ComplexPlant:
    """复杂非线性被控对象"""
    def __init__(self):
        self.x = np.zeros(2)
        self.t = 0

    def step(self, u, dt):
        # 非线性动力学 + 参数不确定性
        a = 2.0 + 0.3 * np.sin(0.5 * self.t)
        b = 5.0 + 0.5 * np.cos(0.3 * self.t)
        # 死区
        u_nl = u if abs(u) > 0.3 else 0
        # 饱和
        u_nl = np.clip(u_nl, -10, 10)

        dx0 = self.x[1]
        dx1 = -a * self.x[1] - b * self.x[0] - 0.05 * self.x[0]**3 + u_nl
        dx1 += 0.3 * np.sin(2 * self.t)  # 外扰

        self.x[0] += dx0 * dt
        self.x[1] += dx1 * dt
        self.t += dt
        return self.x[0]

    def reset(self):
        self.x = np.zeros(2)
        self.t = 0


# ======================== 仿真 ========================
def simulate(controller, ref_func, T=20, dt=0.01):
    N = int(T / dt)
    t_arr = np.linspace(0, T, N)
    y_arr, u_arr = np.zeros(N), np.zeros(N)
    plant = ComplexPlant()

    for i in range(N):
        t = i * dt
        r = ref_func(t)
        error = r - plant.x[0]
        u = controller.compute(error, dt)
        plant.step(u, dt)
        y_arr[i] = plant.x[0]
        u_arr[i] = u

    return t_arr, y_arr, u_arr


def main():
    T, dt = 20, 0.01

    def ref_func(t):
        # 复合参考: 方波 + 正弦
        if t < 5:
            return 1.0
        elif t < 10:
            return -0.5
        else:
            return 0.5 + 0.5 * np.sin(2 * np.pi * (t - 10) / 5)

    controllers = {
        'PID': StandardPID(Kp=5, Ki=1, Kd=2),
        'BP神经网络': BPController(),
        'RBF神经网络': RBFController(),
        '自适应NN(Lyapunov)': AdaptiveNNController(n_hidden=15),
    }

    results = {}
    for name, ctrl in controllers.items():
        t, y, u = simulate(ctrl, ref_func, T, dt)
        results[name] = {'t': t, 'y': y, 'u': u}

    ref_arr = np.array([ref_func(t) for t in results['PID']['t']])

    # 绘图
    colors = ['#2196F3', '#4CAF50', '#FF9800', '#E91E63']
    fig, axes = plt.subplots(3, 1, figsize=(14, 11))

    ax = axes[0]
    ax.plot(results['PID']['t'], ref_arr, 'k--', lw=1.5, label='参考')
    for (name, data), c in zip(results.items(), colors):
        ax.plot(data['t'], data['y'], color=c, lw=1.0, label=name)
    ax.set_ylabel('输出')
    ax.set_title('神经网络控制器 vs PID - 非线性系统控制')
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    ax = axes[1]
    for (name, data), c in zip(results.items(), colors):
        ax.plot(data['t'], ref_arr - data['y'], color=c, lw=0.8, label=name)
    ax.set_ylabel('误差')
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    ax = axes[2]
    for (name, data), c in zip(results.items(), colors):
        ax.plot(data['t'], data['u'], color=c, lw=0.8, label=name)
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('控制量')
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('nn_control_comparison.png', dpi=150, bbox_inches='tight')
    plt.show()

    # RBF网络结构可视化
    rbf_ctrl = RBFController()
    fig2, axes2 = plt.subplots(1, 2, figsize=(12, 5))

    # 中心分布
    ax = axes2[0]
    ax.scatter(rbf_ctrl.rbf.centers[:, 0], rbf_ctrl.rbf.centers[:, 1],
               c=rbf_ctrl.rbf.weights, cmap='RdBu', s=100, edgecolors='k')
    ax.set_xlabel('输入1 (误差)')
    ax.set_ylabel('输入2 (误差变化率)')
    ax.set_title('RBF网络中心分布 (颜色=权重)')
    ax.grid(True, alpha=0.3)

    # 响应面
    ax = axes2[1]
    e_range = np.linspace(-3, 3, 50)
    de_range = np.linspace(-3, 3, 50)
    E, DE = np.meshgrid(e_range, de_range)
    U = np.zeros_like(E)
    for i in range(50):
        for j in range(50):
            U[i, j] = rbf_ctrl.rbf.forward(np.array([E[i, j], DE[i, j]]))
    cs = ax.contourf(E, DE, U, levels=20, cmap='viridis')
    ax.set_xlabel('误差')
    ax.set_ylabel('误差变化率')
    ax.set_title('RBF控制响应面')
    plt.colorbar(cs, ax=ax)

    plt.tight_layout()
    plt.savefig('rbf_network_structure.png', dpi=150, bbox_inches='tight')
    plt.show()

    # 性能指标
    print("\n" + "="*65)
    print("神经网络控制器性能对比")
    print("="*65)
    print(f"{'方法':<20} {'ISE':>10} {'IAE':>10} {'超调%':>10} {'控制能量':>10}")
    print("-"*65)
    for name, data in results.items():
        e = ref_arr - data['y']
        ise = np.sum(e**2) * dt
        iae = np.sum(np.abs(e)) * dt
        overshoot = max(0, (np.max(data['y']) - 1.0) / 1.0 * 100)
        ctrl_energy = np.sum(data['u']**2) * dt
        print(f"{name:<20} {ise:>10.3f} {iae:>10.3f} {overshoot:>9.1f}% {ctrl_energy:>10.1f}")
    print("="*65)


if __name__ == '__main__':
    main()
