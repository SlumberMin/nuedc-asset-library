#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
神经网络PID学习仿真
===================
仿真内容：基于BP神经网络的PID参数自学习
展示学习曲线和收敛性
"""

import os
import numpy as np
import matplotlib


def main():
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
    matplotlib.rcParams['axes.unicode_minus'] = False

    dt = 0.01
    T_total = 20.0
    N = int(T_total / dt)
    t = np.arange(N) * dt

    # ============ 被控对象 ============
    def plant(x, u, dt):
        """非线性二阶系统: y'' + a*y' + b*y^2 = c*u"""
        pos, vel = x
        acc = (-0.5 * vel - 0.3 * pos**2 + u) / 1.0
        vel_new = vel + acc * dt
        pos_new = pos + vel_new * dt
        return np.array([pos_new, vel_new])

    # ============ 神经网络PID控制器 ============
    class NeuralPID:
        """
        基于BP神经网络的PID参数自整定

        网络结构: 3输入 -> 5隐层 -> 3输出(Kp, Ki, Kd)
        输入: [error, error_dot, error_sum]
        输出: [Kp, Ki, Kd]
        """
        def __init__(self, lr=0.01):
            self.lr = lr  # 学习率

            # 网络权重初始化
            np.random.seed(42)
            self.W1 = np.random.randn(3, 5) * 0.5   # 输入层->隐层
            self.b1 = np.zeros(5)
            self.W2 = np.random.randn(5, 3) * 0.5   # 隐层->输出层
            self.b2 = np.zeros(3)

            # PID状态
            self.integral = 0.0
            self.prev_error = 0.0
            self.prev_u = 0.0

            # 记录
            self.Kp_history = []
            self.Ki_history = []
            self.Kd_history = []
            self.loss_history = []

        def _sigmoid(self, x):
            return 1.0 / (1.0 + np.exp(-np.clip(x, -500, 500)))

        def _tanh(self, x):
            return np.tanh(x)

        def forward(self, x_in):
            """前向传播"""
            # 隐层
            self.h_input = self.W1.T @ x_in + self.b1
            self.h_output = self._sigmoid(self.h_input)
            # 输出层
            self.o_input = self.W2.T @ self.h_output + self.b2
            # 输出Kp, Ki, Kd (用sigmoid限制在[0,1]，再缩放)
            params = self._sigmoid(self.o_input)
            return params

        def backward(self, x_in, params, error, d_error, u, y, ref):
            """
            反向传播更新权重
            使用梯度下降最小化 J = 0.5 * error^2
            """
            # 近似梯度: dy/du ≈ sign(y(k)-y(k-1)) / (u(k)-u(k-1)+eps)
            eps = 1e-6
            dy_du_approx = np.sign(error) / (abs(self.prev_u) + eps)

            # 损失对输出的梯度
            dJ_de = error  # d(0.5*e^2)/de = e

            # 对Kp, Ki, Kd的梯度（通过PID公式链式法则）
            dJ_dKp = dJ_de * dy_du_approx * (-error)  # du/dKp = -error
            dJ_dKi = dJ_de * dy_du_approx * (-self.integral)
            dJ_dKd = dJ_de * dy_du_approx * (-d_error)

            dJ_dout = np.array([dJ_dKp, dJ_dKi, dJ_dKd])

            # sigmoid输出层梯度
            sigmoid_deriv = params * (1 - params)
            delta2 = dJ_dout * sigmoid_deriv

            # 更新W2, b2
            self.W2 -= self.lr * np.outer(self.h_output, delta2)
            self.b2 -= self.lr * delta2

            # 隐层梯度
            delta1 = (self.W2 @ delta2) * self.h_output * (1 - self.h_output)

            # 更新W1, b1
            self.W1 -= self.lr * np.outer(x_in, delta1)
            self.b1 -= self.lr * delta1

        def compute(self, ref, y, dt):
            error = ref - y
            d_error = (error - self.prev_error) / dt
            self.integral += error * dt

            # 网络输入
            x_in = np.array([
                np.clip(error, -5, 5),
                np.clip(d_error, -5, 5),
                np.clip(self.integral, -10, 10)
            ])

            # 前向传播得到PID参数
            params = self.forward(x_in)
            Kp = params[0] * 20.0    # 缩放到[0, 20]
            Ki = params[1] * 5.0     # 缩放到[0, 5]
            Kd = params[2] * 5.0     # 缩放到[0, 5]

            # PID计算
            u = Kp * error + Ki * self.integral + Kd * d_error
            u = np.clip(u, -50, 50)

            # 反向传播更新
            self.backward(x_in, params, error, d_error, u, y, ref)

            # 记录
            self.Kp_history.append(Kp)
            self.Ki_history.append(Ki)
            self.Kd_history.append(Kd)
            self.loss_history.append(error**2)

            self.prev_error = error
            self.prev_u = u

            return u

    # ============ 标准PID ============
    class StandardPID:
        def __init__(self, Kp, Ki, Kd):
            self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
            self.integral = 0.0
            self.prev_error = 0.0

        def compute(self, ref, y, dt):
            error = ref - y
            self.integral += error * dt
            derivative = (error - self.prev_error) / dt
            self.prev_error = error
            return self.Kp * error + self.Ki * self.integral + self.Kd * derivative

    # ============ 运行仿真 ============
    print("神经网络PID学习仿真...")

    nn_pid = NeuralPID(lr=0.005)
    std_pid = StandardPID(Kp=5.0, Ki=1.0, Kd=2.0)

    # 参考信号
    def ref_func(t):
        if t < 5:
            return 1.0
        elif t < 10:
            return 0.5 * np.sin(2 * np.pi * 0.5 * t) + 1.0
        elif t < 15:
            return 0.5
        else:
            return 1.5 + 0.3 * np.sin(2 * np.pi * t)

    x_nn = np.array([0.0, 0.0])
    x_std = np.array([0.0, 0.0])
    y_nn = np.zeros(N)
    y_std = np.zeros(N)
    ref_log = np.zeros(N)

    for i in range(N):
        ref = ref_func(i * dt)
        ref_log[i] = ref

        u_nn = nn_pid.compute(ref, x_nn[0], dt)
        x_nn = plant(x_nn, u_nn, dt)
        y_nn[i] = x_nn[0]

        u_std = std_pid.compute(ref, x_std[0], dt)
        x_std = plant(x_std, u_std, dt)
        y_std[i] = x_std[0]

    Kp_arr = np.array(nn_pid.Kp_history)
    Ki_arr = np.array(nn_pid.Ki_history)
    Kd_arr = np.array(nn_pid.Kd_history)
    loss_arr = np.array(nn_pid.loss_history)

    # 性能指标
    iae_nn = np.sum(np.abs(ref_log - y_nn)) * dt
    iae_std = np.sum(np.abs(ref_log - y_std)) * dt
    print(f"神经网络PID - IAE: {iae_nn:.3f}")
    print(f"标准PID     - IAE: {iae_std:.3f}")

    # 学习曲线（滑动平均窗口）
    window = 50
    loss_smooth = np.convolve(loss_arr, np.ones(window)/window, mode='valid')
    print(f"初始阶段平均损失: {np.mean(loss_arr[:100]):.4f}")
    print(f"最终阶段平均损失: {np.mean(loss_arr[-100:]):.4f}")

    # ============ 绘图 ============
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    fig.suptitle('神经网络PID学习仿真', fontsize=16, fontweight='bold')

    # 跟踪响应
    ax = axes[0, 0]
    ax.plot(t, ref_log, 'k--', linewidth=2, label='参考值')
    ax.plot(t, y_nn, 'r-', linewidth=1.5, label='神经网络PID')
    ax.plot(t, y_std, 'b-', linewidth=1, alpha=0.7, label='标准PID')
    ax.set_title('跟踪响应对比')
    ax.set_xlabel('时间 (s)'); ax.set_ylabel('输出')
    ax.legend(); ax.grid(True, alpha=0.3)

    # 学习曲线
    ax = axes[0, 1]
    ax.plot(t, loss_arr, 'gray', linewidth=0.5, alpha=0.3, label='瞬时损失')
    ax.plot(t[window-1:], loss_smooth, 'r-', linewidth=2, label=f'滑动平均({window}步)')
    ax.set_title('学习曲线（损失函数收敛）')
    ax.set_xlabel('时间 (s)'); ax.set_ylabel('损失 J = e²')
    ax.legend(); ax.grid(True, alpha=0.3)
    ax.set_yscale('log')

    # PID参数演化
    ax = axes[1, 0]
    ax.plot(t, Kp_arr, 'r-', linewidth=1.5, label='Kp')
    ax.plot(t, Ki_arr, 'g-', linewidth=1.5, label='Ki')
    ax.plot(t, Kd_arr, 'b-', linewidth=1.5, label='Kd')
    ax.axhline(y=5.0, color='r', linestyle=':', alpha=0.3)
    ax.axhline(y=1.0, color='g', linestyle=':', alpha=0.3)
    ax.axhline(y=2.0, color='b', linestyle=':', alpha=0.3)
    ax.set_title('PID参数学习过程')
    ax.set_xlabel('时间 (s)'); ax.set_ylabel('参数值')
    ax.legend(); ax.grid(True, alpha=0.3)

    # 跟踪误差对比
    ax = axes[1, 1]
    ax.plot(t, np.abs(ref_log - y_nn), 'r-', linewidth=1, label='神经网络PID')
    ax.plot(t, np.abs(ref_log - y_std), 'b-', linewidth=1, alpha=0.7, label='标准PID')
    ax.set_title('跟踪误差')
    ax.set_xlabel('时间 (s)'); ax.set_ylabel('|误差|')
    ax.legend(); ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'neural_pid_learning_result.png'), dpi=150, bbox_inches='tight')
    print("\n图表已保存: neural_pid_learning_result.png")
    print("神经网络PID学习仿真完成!")



if __name__ == '__main__':
    main()
