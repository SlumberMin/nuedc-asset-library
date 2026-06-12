#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
神经网络PID仿真 — 学习过程可视化
==================================
使用单隐层神经网络在线调整PID参数 Kp, Ki, Kd
网络结构: 3-5-3 (输入: e, Σe, Δe → 输出: ΔKp, ΔKi, ΔKd)
激活函数: tanh, 输出层线性
学习算法: 梯度下降 (BP)
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib


def main():
    matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
    matplotlib.rcParams['axes.unicode_minus'] = False

    np.random.seed(42)

    # ── 仿真参数 ──
    Ts = 0.01
    T_sim = 15.0
    N = int(T_sim / Ts)
    t = np.arange(N) * Ts

    # ── 参考信号: 方波 ──
    r = np.zeros(N)
    for i in range(N):
        period = 4.0  # 4秒周期
        r[i] = 1.0 if (t[i] % period) < period/2 else 0.0

    # ── 被控对象: 一阶惯性 + 纯滞后 ──
    # G(s) = 2 * e^{-0.5s} / (2s + 1)
    class Plant:
        def __init__(self, Ts):
            self.Ts = Ts
            self.y = 0.0
            self.tau = 2.0   # 时间常数
            self.K = 2.0     # 增益
            self.delay_steps = int(0.5 / Ts)  # 纯滞后
            self.u_buffer = [0.0] * self.delay_steps

        def step(self, u):
            self.u_buffer.append(u)
            u_delayed = self.u_buffer.pop(0)
            # 一阶离散化: y(k+1) = a*y(k) + b*u(k)
            a = np.exp(-self.Ts / self.tau)
            b = self.K * (1 - a)
            self.y = a * self.y + b * u_delayed
            return self.y

    plant = Plant(Ts)

    # ── 神经网络PID控制器 ──
    class NeuralPID:
        def __init__(self, lr=0.01):
            self.lr = lr
            # 网络权重: 输入层(3) -> 隐层(5) -> 输出层(3)
            self.W1 = np.random.randn(3, 5) * 0.5  # 输入→隐层
            self.b1 = np.zeros(5)
            self.W2 = np.random.randn(5, 3) * 0.1  # 隐层→输出
            self.b2 = np.array([0.5, 0.3, 0.1])    # 初始偏置: 基础Kp,Ki,Kd

            # PID参数基值
            self.Kp0, self.Ki0, self.Kd0 = 1.0, 0.5, 0.2

            # 历史记录
            self.e_prev = 0.0
            self.e_sum = 0.0
            self.Kp_hist, self.Ki_hist, self.Kd_hist = [], [], []
            self.loss_hist = []
            self.W1_hist = []  # 记录权重变化
            self.W2_hist = []

        def _tanh(self, x):
            return np.tanh(x)

        def _dtanh(self, x):
            return 1 - np.tanh(x)**2

        def forward(self, x_in):
            """前向传播: 输入=[e, Σe, Δe]"""
            self.h_input = self.W1.T @ x_in + self.b1
            self.h_act = self._tanh(self.h_input)
            self.o_input = self.W2.T @ self.h_act + self.b2
            self.o_act = self._tanh(self.o_input)  # 输出范围(-1,1)
            return self.o_act

        def update(self, e):
            # 计算PID输入
            self.e_sum += e * Ts
            self.e_sum = np.clip(self.e_sum, -5, 5)
            de = (e - self.e_prev) / Ts
            self.e_prev = e

            # 归一化输入
            x_in = np.array([e, self.e_sum, de]) * 0.5
            x_in = np.clip(x_in, -2, 2)

            # 前向传播
            delta = self.forward(x_in)

            # 计算PID参数
            Kp = max(self.Kp0 + delta[0] * 0.5, 0.01)
            Ki = max(self.Ki0 + delta[1] * 0.3, 0.01)
            Kd = max(self.Kd0 + delta[2] * 0.2, 0.0)

            # 记录
            self.Kp_hist.append(Kp)
            self.Ki_hist.append(Ki)
            self.Kd_hist.append(Kd)
            self.W1_hist.append(self.W1.copy())
            self.W2_hist.append(self.W2.copy())

            return Kp, Ki, Kd

        def backward(self, e, de, u, Kp, Ki, Kd, y_prev):
            """反向传播更新权重 (基于误差平方最小化)"""
            # 简化的梯度: dJ/dw ∝ e * sign(dy/du) * du/dw
            # sign(dy/du) > 0 for stable plant
            sign_dydu = 1.0

            # 输出层梯度
            dJ_du = -e * sign_dydu
            du_dKp = self.e_prev + self.e_sum * Ts + de * Ts  # 近似
            du_dKi = self.e_sum
            du_dKd = de

            delta_out = np.array([
                dJ_du * du_dKp,
                dJ_du * du_dKi,
                dJ_du * du_dKd
            ])

            # 反向传播
            d_act = self._dtanh(self.o_input)
            delta_out_raw = delta_out * d_act

            # 更新 W2
            dW2 = np.outer(self.h_act, delta_out_raw) * self.lr
            self.W2 -= np.clip(dW2, -0.1, 0.1)
            self.b2 -= self.lr * delta_out_raw * 0.1
            self.b2 = np.clip(self.b2, -2, 2)

            # 更新 W1
            delta_hidden = (self.W2 @ delta_out_raw) * self._dtanh(self.h_input)
            x_in = np.array([e, self.e_sum, de]) * 0.5
            dW1 = np.outer(x_in, delta_hidden) * self.lr * 0.1
            self.W1 -= np.clip(dW1, -0.05, 0.05)

            loss = e**2
            self.loss_hist.append(loss)
            return loss


    # ── 仿真 ──
    nn_pid = NeuralPID(lr=0.005)
    y_hist = np.zeros(N)
    u_hist = np.zeros(N)
    losses = []

    for i in range(N):
        y = plant.y
        e = r[i] - y

        Kp, Ki, Kd = nn_pid.update(e)

        # PID计算
        u = Kp * e + Ki * nn_pid.e_sum + Kd * nn_pid.e_prev
        u = np.clip(u, -5, 5)

        # 更新被控对象
        y_new = plant.step(u)

        # 反向传播
        de = (e - nn_pid.e_prev) / Ts if i > 0 else 0
        loss = nn_pid.backward(e, de, u, Kp, Ki, Kd, y)

        y_hist[i] = y_new
        u_hist[i] = u

    # ── 绘图 ──
    fig, axes = plt.subplots(3, 2, figsize=(14, 12))
    fig.suptitle('神经网络PID仿真 — 学习过程可视化', fontsize=14, fontweight='bold')

    # (0,0) 跟踪效果
    axes[0, 0].plot(t, r, 'k--', lw=1.2, label='设定值')
    axes[0, 0].plot(t, y_hist, 'b-', lw=1.2, label='NN-PID输出')
    axes[0, 0].set_ylabel('输出 y(t)')
    axes[0, 0].set_title('方波跟踪效果')
    axes[0, 0].legend(fontsize=9)
    axes[0, 0].grid(True, alpha=0.3)

    # (0,1) 控制量
    axes[0, 1].plot(t, u_hist, 'r-', lw=1)
    axes[0, 1].set_ylabel('控制量 u(t)')
    axes[0, 1].set_title('控制信号')
    axes[0, 1].grid(True, alpha=0.3)

    # (1,0) Kp, Ki, Kd学习过程
    axes[1, 0].plot(t, nn_pid.Kp_hist, 'r-', lw=1, label='Kp')
    axes[1, 0].plot(t, nn_pid.Ki_hist, 'g-', lw=1, label='Ki')
    axes[1, 0].plot(t, nn_pid.Kd_hist, 'b-', lw=1, label='Kd')
    axes[1, 0].set_ylabel('PID参数值')
    axes[1, 0].set_title('Kp / Ki / Kd 学习演化过程')
    axes[1, 0].legend(fontsize=9)
    axes[1, 0].grid(True, alpha=0.3)

    # (1,1) 损失函数
    if nn_pid.loss_hist:
        # 滑动平均平滑
        window = 50
        loss_smooth = np.convolve(nn_pid.loss_hist, np.ones(window)/window, mode='valid')
        axes[1, 1].plot(t[:len(loss_smooth)], loss_smooth, 'm-', lw=1.2)
    axes[1, 1].set_ylabel('损失 e²(t)')
    axes[1, 1].set_title('学习损失函数 (滑动平均)')
    axes[1, 1].set_xlabel('时间 (s)')
    axes[1, 1].grid(True, alpha=0.3)

    # (2,0) W1权重演化 (选取几个代表性权重)
    W1_arr = np.array(nn_pid.W1_hist)  # shape: (N, 3, 5)
    for idx in [(0,0), (1,2), (2,4)]:
        axes[2, 0].plot(t[:len(W1_arr)], W1_arr[:, idx[0], idx[1]],
                        lw=0.8, label=f'W1[{idx[0]},{idx[1]}]')
    axes[2, 0].set_xlabel('时间 (s)')
    axes[2, 0].set_ylabel('权重值')
    axes[2, 0].set_title('隐层权重 W1 演化 (部分)')
    axes[2, 0].legend(fontsize=8)
    axes[2, 0].grid(True, alpha=0.3)

    # (2,1) W2权重演化
    W2_arr = np.array(nn_pid.W2_hist)
    for idx in [(0,0), (2,1), (4,2)]:
        axes[2, 1].plot(t[:len(W2_arr)], W2_arr[:, idx[0], idx[1]],
                        lw=0.8, label=f'W2[{idx[0]},{idx[1]}]')
    axes[2, 1].set_xlabel('时间 (s)')
    axes[2, 1].set_ylabel('权重值')
    axes[2, 1].set_title('输出层权重 W2 演化 (部分)')
    axes[2, 1].legend(fontsize=8)
    axes[2, 1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'neural_pid_result.png'), dpi=150, bbox_inches='tight')
    plt.close('all')

    iae = np.sum(np.abs(r - y_hist)) * Ts
    print(f"✅ 神经网络PID仿真完成")
    print(f"   最终Kp={nn_pid.Kp_hist[-1]:.3f}, Ki={nn_pid.Ki_hist[-1]:.3f}, Kd={nn_pid.Kd_hist[-1]:.3f}")
    print(f"   IAE = {iae:.4f}")



if __name__ == '__main__':
    main()
