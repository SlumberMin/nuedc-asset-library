#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IMU姿态估计仿真 - 欧拉角 + 四元数 + 旋转矩阵
nuedc-asset-library V3
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# ============================================================
# 1. 旋转矩阵工具
# ============================================================

def euler_to_rotmat(roll, pitch, yaw):
    """欧拉角→旋转矩阵 (ZYX顺序, rad)"""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rx = np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
    Ry = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
    Rz = np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])
    return Rz @ Ry @ Rx

def rotmat_to_euler(R):
    """旋转矩阵→欧拉角"""
    pitch = np.arcsin(-np.clip(R[2,0], -1, 1))
    if np.abs(np.cos(pitch)) > 1e-6:
        roll = np.arctan2(R[2,1], R[2,2])
        yaw = np.arctan2(R[1,0], R[0,0])
    else:
        roll = np.arctan2(-R[0,1], R[1,1])
        yaw = 0
    return roll, pitch, yaw

# ============================================================
# 2. 四元数工具
# ============================================================

def euler_to_quat(roll, pitch, yaw):
    """欧拉角→四元数 [w, x, y, z]"""
    cr, sr = np.cos(roll/2), np.sin(roll/2)
    cp, sp = np.cos(pitch/2), np.sin(pitch/2)
    cy, sy = np.cos(yaw/2), np.sin(yaw/2)
    w = cr*cp*cy + sr*sp*sy
    x = sr*cp*cy - cr*sp*sy
    y = cr*sp*cy + sr*cp*sy
    z = cr*cp*sy - sr*sp*cy
    return np.array([w, x, y, z])

def quat_to_euler(q):
    """四元数→欧拉角"""
    w, x, y, z = q
    roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch = np.arcsin(np.clip(2*(w*y - z*x), -1, 1))
    yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return roll, pitch, yaw

def quat_multiply(q1, q2):
    """四元数乘法"""
    w1,x1,y1,z1 = q1
    w2,x2,y2,z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])

def quat_normalize(q):
    return q / np.linalg.norm(q)

def quat_slerp(q0, q1, t):
    """球面线性插值"""
    dot = np.dot(q0, q1)
    if dot < 0:
        q1 = -q1
        dot = -dot
    dot = np.clip(dot, -1, 1)
    if dot > 0.9995:
        return quat_normalize(q0 + t*(q1 - q0))
    theta = np.arccos(dot)
    return (np.sin((1-t)*theta)*q0 + np.sin(t*theta)*q1) / np.sin(theta)

# ============================================================
# 3. IMU传感器模型
# ============================================================

class IMUModel:
    def __init__(self, gyro_noise=0.01, accel_noise=0.1, gyro_bias_drift=0.001):
        self.gyro_noise = gyro_noise
        self.accel_noise = accel_noise
        self.gyro_bias = np.zeros(3)
        self.bias_drift = gyro_bias_drift
        self.gravity = np.array([0, 0, 9.81])

    def update(self, true_rpy, dt):
        """模拟IMU读数"""
        # 陀螺仪：角速度 + 偏置漂移 + 噪声
        self.gyro_bias += np.random.normal(0, self.bias_drift * dt, 3)
        gyro = np.array([0, 0, 0]) + self.gyro_bias + np.random.normal(0, self.gyro_noise, 3)

        # 加速度计：重力投影 + 噪声
        R = euler_to_rotmat(*true_rpy)
        accel = R.T @ self.gravity + np.random.normal(0, self.accel_noise, 3)

        return gyro, accel

# ============================================================
# 4. 姿态估计器（互补滤波 + 扩展卡尔曼滤波）
# ============================================================

class ComplementaryFilter:
    def __init__(self, alpha=0.98):
        self.alpha = alpha
        self.rpy = np.zeros(3)

    def update(self, gyro, accel, dt):
        # 陀螺仪积分
        gyro_rpy = self.rpy + gyro * dt
        # 加速度计估计
        accel_roll = np.arctan2(accel[1], accel[2])
        accel_pitch = np.arctan2(-accel[0], np.sqrt(accel[1]**2 + accel[2]**2))
        # 互补融合
        self.rpy[0] = self.alpha * gyro_rpy[0] + (1 - self.alpha) * accel_roll
        self.rpy[1] = self.alpha * gyro_rpy[1] + (1 - self.alpha) * accel_pitch
        self.rpy[2] = gyro_rpy[2]  # yaw只能靠陀螺仪
        return self.rpy.copy()

class EKFOrientation:
    """简化EKF姿态估计"""
    def __init__(self):
        self.x = np.array([0,0,0,1,0,0,0], dtype=float)  # [gyro_bias(3), quat(4)]
        self.x[3] = 1.0
        self.P = np.eye(7) * 0.1
        self.Q = np.eye(7) * 0.001
        self.R = np.eye(3) * 0.5

    def predict(self, gyro, dt):
        omega = gyro - self.x[:3]
        q = self.x[3:]
        # 四元数微分
        Omega = np.array([
            [0, -omega[0], -omega[1], -omega[2]],
            [omega[0], 0, omega[2], -omega[1]],
            [omega[1], -omega[2], 0, omega[0]],
            [omega[2], omega[1], -omega[0], 0]
        ]) * 0.5 * dt
        q_new = (np.eye(4) + Omega) @ q
        q_new = q_new / np.linalg.norm(q_new)
        self.x[3:] = q_new
        self.P = self.P + self.Q * dt

    def update_accel(self, accel):
        """加速度计修正roll/pitch"""
        q = self.x[3:]
        w,x,y,z = q
        # 预期重力方向
        g_pred = np.array([
            2*(x*z - w*y),
            2*(w*x + y*z),
            w*w - x*x - y*y + z*z
        ]) * 9.81
        accel_norm = accel / np.linalg.norm(accel) * 9.81

        z_meas = accel_norm - g_pred
        # 简化H矩阵
        H = np.zeros((3, 7))
        H[:, 3:] = np.eye(3) * 0.01  # 简化雅可比
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x += K @ z_meas
        self.x[3:] /= np.linalg.norm(self.x[3:])
        self.P = (np.eye(7) - K @ H) @ self.P

    def get_rpy(self):
        return quat_to_euler(self.x[3:])

# ============================================================
# 5. 仿真主程序
# ============================================================

def run_imu_simulation():
    np.random.seed(42)

    dt = 0.01  # 100Hz
    t_total = 10.0
    n = int(t_total / dt)
    t = np.arange(n) * dt

    # 真实姿态：正弦摆动
    true_roll  = 0.3 * np.sin(2 * np.pi * 0.5 * t)
    true_pitch = 0.2 * np.sin(2 * np.pi * 0.3 * t)
    true_yaw   = 0.1 * np.sin(2 * np.pi * 0.1 * t) + 0.05 * t

    imu = IMUModel(gyro_noise=0.02, accel_noise=0.15, gyro_bias_drift=0.002)
    comp = ComplementaryFilter(alpha=0.96)
    ekf = EKFOrientation()

    comp_rpy = np.zeros((n, 3))
    ekf_rpy = np.zeros((n, 3))

    for i in range(n):
        true_rpy = [true_roll[i], true_pitch[i], true_yaw[i]]
        gyro, accel = imu.update(true_rpy, dt)
        comp_rpy[i] = comp.update(gyro, accel, dt)
        ekf.predict(gyro, dt)
        ekf.update_accel(accel)
        ekf_rpy[i] = ekf.get_rpy()

    # 可视化
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    labels = ['Roll (°)', 'Pitch (°)', 'Yaw (°)']
    true_data = [true_roll, true_pitch, true_yaw]

    for ax, label, true_d in zip(axes, labels, true_data):
        idx = labels.index(label)
        ax.plot(t, np.degrees(true_d), 'k-', linewidth=1.5, label='真实')
        ax.plot(t, np.degrees(comp_rpy[:, idx]), 'b-', alpha=0.7, label='互补滤波')
        ax.plot(t, np.degrees(ekf_rpy[:, idx]), 'r-', alpha=0.7, label='EKF')
        ax.set_ylabel(label)
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel('时间 (s)')
    fig.suptitle('IMU姿态估计仿真 (互补滤波 vs EKF)', fontsize=14)

    # 误差对比
    fig2, ax2 = plt.subplots(1, 1, figsize=(10, 5))
    comp_err = np.sqrt(np.mean((true_roll - comp_rpy[:,0])**2 + (true_pitch - comp_rpy[:,1])**2))
    ekf_err = np.sqrt(np.mean((true_roll - ekf_rpy[:,0])**2 + (true_pitch - ekf_rpy[:,1])**2))
    ax2.bar(['互补滤波', 'EKF'], [np.degrees(comp_err), np.degrees(ekf_err)],
            color=['blue', 'red'], alpha=0.7)
    ax2.set_ylabel('RMSE (°)')
    ax2.set_title('姿态估计精度对比')
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('imu_orientation_simulation.png', dpi=150)
    plt.show()

    print("=" * 50)
    print("IMU姿态估计仿真结果")
    print("=" * 50)
    print(f"采样率: 100Hz, 仿真时长: {t_total}s")
    print(f"互补滤波 RMSE: {np.degrees(comp_err):.3f}°")
    print(f"EKF      RMSE: {np.degrees(ekf_err):.3f}°")

if __name__ == '__main__':
    run_imu_simulation()
