#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
扩展卡尔曼滤波器 (EKF) 仿真演示

演示内容：
1. EKF 估计非线性系统状态
2. 传感器融合示例
3. 不同噪声水平下的滤波效果
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial Unicode MS']
plt.rcParams['axes.unicode_minus'] = False


class EKF:
    """通用 EKF 实现"""

    def __init__(self, n, p, f_func, h_func, F_jac, H_jac):
        self.n = n  # 状态维度
        self.p = p  # 量测维度
        self.f = f_func
        self.h = h_func
        self.F = F_jac
        self.H = H_jac

        self.x = np.zeros(n)
        self.P = np.eye(n) * 10.0
        self.Q = np.eye(n) * 0.01
        self.R = np.eye(p) * 1.0

    def predict(self, u=None):
        F_mat = self.F(self.x, u)
        self.x = self.f(self.x, u)
        self.P = F_mat @ self.P @ F_mat.T + self.Q

    def update(self, z):
        H_mat = self.H(self.x)
        z_pred = self.h(self.x)
        S = H_mat @ self.P @ H_mat.T + self.R
        K = self.P @ H_mat.T @ np.linalg.inv(S)
        self.x = self.x + K @ (z - z_pred)
        I = np.eye(self.n)
        self.P = (I - K @ H_mat) @ self.P

    def step(self, z, u=None):
        self.predict(u)
        self.update(z)
        return self.x.copy()


def demo1_nonlinear_system():
    """演示1：非线性系统状态估计（带噪声的角度传感器）"""
    dt = 0.01
    T = 10.0
    n_steps = int(T / dt)

    # 真实系统：简单谐振 + 非线性
    # 状态：[角度, 角速度]
    # x' = [x[1], -sin(x[0]) - 0.1*x[1] + u]
    def f_true(x, u):
        return np.array([
            x[0] + x[1] * dt,
            x[1] + (-np.sin(x[0]) - 0.1 * x[1] + u) * dt
        ])

    # EKF 用的模型（略有误差）
    def f_model(x, u):
        return np.array([
            x[0] + x[1] * dt,
            x[1] + (-np.sin(x[0]) - 0.08 * x[1]) * dt  # 阻尼系数有误差
        ])

    # 量测函数：只观测角度
    def h_func(x):
        return np.array([x[0]])

    # 雅可比矩阵
    def F_jac(x, u):
        return np.array([
            [1.0, dt],
            [-np.cos(x[0]) * dt, 1 - 0.08 * dt]
        ])

    def H_jac(x):
        return np.array([[1.0, 0.0]])

    # 初始化 EKF
    ekf = EKF(2, 1, f_model, h_func, F_jac, H_jac)
    ekf.x = np.array([0.5, 0.0])  # 初始估计（有偏）
    ekf.Q = np.diag([0.001, 0.01])
    ekf.R = np.array([[0.1]])  # 角度量测噪声

    # 仿真
    x_true = np.array([0.0, 1.0])  # 真实初始状态
    np.random.seed(42)

    t_arr = np.arange(n_steps) * dt
    x_true_arr = np.zeros((n_steps, 2))
    x_est_arr = np.zeros((n_steps, 2))
    z_arr = np.zeros(n_steps)

    for i in range(n_steps):
        u = 0.5 * np.sin(2 * np.pi * 0.5 * t_arr[i])  # 正弦激励

        # 真实状态更新
        x_true = f_true(x_true, u) + np.random.multivariate_normal(
            np.zeros(2), np.diag([0.0001, 0.001]))

        # 量测（带噪声）
        z = x_true[0] + np.random.normal(0, np.sqrt(0.1))

        # EKF 估计
        x_est = ekf.step(np.array([z]), u)

        x_true_arr[i] = x_true
        x_est_arr[i] = x_est
        z_arr[i] = z

    fig, axes = plt.subplots(2, 1, figsize=(10, 7))

    axes[0].plot(t_arr, x_true_arr[:, 0], 'b-', label='真实角度', linewidth=1.5)
    axes[0].plot(t_arr, x_est_arr[:, 0], 'r--', label='EKF估计', linewidth=1.5)
    axes[0].scatter(t_arr[::20], z_arr[::20], s=5, c='gray', alpha=0.3, label='量测（噪声）')
    axes[0].set_ylabel('角度 (rad)')
    axes[0].set_title('EKF 非线性系统状态估计 - 角度')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t_arr, x_true_arr[:, 1], 'b-', label='真实角速度', linewidth=1.5)
    axes[1].plot(t_arr, x_est_arr[:, 1], 'r--', label='EKF估计', linewidth=1.5)
    axes[1].set_xlabel('时间 (s)')
    axes[1].set_ylabel('角速度 (rad/s)')
    axes[1].set_title('EKF 非线性系统状态估计 - 角速度')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('ekf_nonlinear.png', dpi=150, bbox_inches='tight')

    # 计算误差
    err_angle = np.sqrt(np.mean((x_true_arr[:, 0] - x_est_arr[:, 0])**2))
    err_vel = np.sqrt(np.mean((x_true_arr[:, 1] - x_est_arr[:, 1])**2))
    print(f"角度 RMSE: {err_angle:.4f} rad")
    print(f"角速度 RMSE: {err_vel:.4f} rad/s")


def demo2_sensor_fusion():
    """演示2：传感器融合（双传感器量测同一状态）"""
    dt = 0.01
    T = 10.0
    n_steps = int(T / dt)

    # 状态：[位置, 速度]
    # 匀速运动模型
    def f_model(x, u=None):
        return np.array([x[0] + x[1] * dt, x[1]])

    # 两个传感器：[位置传感器1, 位置传感器2（有偏）]
    def h_func(x):
        return np.array([x[0], x[0] + 0.5])  # 传感器2有0.5的固定偏差

    def F_jac(x, u=None):
        return np.array([[1, dt], [0, 1]])

    def H_jac(x):
        return np.array([[1, 0], [1, 0]])

    ekf = EKF(2, 2, f_model, h_func, F_jac, H_jac)
    ekf.x = np.array([0.0, 1.0])
    ekf.Q = np.diag([0.01, 0.001])
    ekf.R = np.diag([0.1, 0.5])  # 传感器2噪声更大

    x_true = np.array([0.0, 1.0])
    np.random.seed(42)

    t_arr = np.arange(n_steps) * dt
    x_true_arr = np.zeros((n_steps, 2))
    x_est_arr = np.zeros((n_steps, 2))

    for i in range(n_steps):
        x_true = f_model(x_true) + np.random.multivariate_normal(
            np.zeros(2), np.diag([0.001, 0.0001]))
        z1 = x_true[0] + np.random.normal(0, np.sqrt(0.1))
        z2 = x_true[0] + 0.5 + np.random.normal(0, np.sqrt(0.5))
        x_est = ekf.step(np.array([z1, z2]))
        x_true_arr[i] = x_true
        x_est_arr[i] = x_est

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(t_arr, x_true_arr[:, 0], 'b-', label='真实位置', linewidth=2)
    ax.plot(t_arr, x_est_arr[:, 0], 'r--', label='EKF融合估计', linewidth=1.5)
    ax.set_xlabel('时间 (s)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('ekf_sensor_fusion.png', dpi=150, bbox_inches='tight')


if __name__ == '__main__':
    print("=== EKF 仿真演示 ===")
    print("演示1: 非线性系统状态估计...")
    demo1_nonlinear_system()
    print("演示2: 传感器融合...")
    demo2_sensor_fusion()
    print("仿真完成！")
