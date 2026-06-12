#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
多传感器卡尔曼融合仿真
融合GPS+IMU+编码器，对比不同传感器组合效果
nuedc-asset-library V3
"""

import numpy as np
import matplotlib.pyplot as plt

# ============================================================
# 1. 系统模型：2D运动 (x, y, vx, vy, yaw)
# ============================================================

class VehicleModel:
    def __init__(self, x=0, y=0, yaw=0):
        self.state = np.array([x, y, 0.0, 0.0, yaw])  # x,y,vx,vy,yaw
        self.trajectory = [self.state.copy()]

    def step(self, speed, yaw_rate, dt):
        """运动更新"""
        self.state[4] += yaw_rate * dt
        self.state[2] = speed * np.cos(self.state[4])
        self.state[3] = speed * np.sin(self.state[4])
        self.state[0] += self.state[2] * dt
        self.state[1] += self.state[3] * dt
        self.trajectory.append(self.state.copy())

    def get_trajectory(self):
        return np.array(self.trajectory)

# ============================================================
# 2. 传感器模型
# ============================================================

class GPSSensor:
    def __init__(self, noise=2.0, rate=1.0):
        self.noise = noise
        self.rate = rate  # Hz
        self.last_t = 0

    def read(self, state, t):
        if t - self.last_t < 1.0 / self.rate:
            return None
        self.last_t = t
        return state[:2] + np.random.normal(0, self.noise, 2)

class IMUSensor:
    def __init__(self, accel_noise=0.5, gyro_noise=0.05, rate=50.0):
        self.accel_noise = accel_noise
        self.gyro_noise = gyro_noise
        self.rate = rate
        self.last_t = 0

    def read(self, state, dt_sim, t):
        if t - self.last_t < 1.0 / self.rate:
            return None
        self.last_t = t
        ax = np.random.normal(0, self.accel_noise)
        ay = np.random.normal(0, self.accel_noise)
        yaw_rate = np.random.normal(0, self.gyro_noise)
        return np.array([ax, ay, yaw_rate])

class EncoderSensor:
    """编码器：测量速度"""
    def __init__(self, noise=0.3, rate=20.0):
        self.noise = noise
        self.rate = rate
        self.last_t = 0

    def read(self, state, t):
        if t - self.last_t < 1.0 / self.rate:
            return None
        self.last_t = t
        speed = np.sqrt(state[2]**2 + state[3]**2)
        return speed + np.random.normal(0, self.noise)

# ============================================================
# 3. 扩展卡尔曼滤波器
# ============================================================

class EKF:
    def __init__(self, x0=None, P0=None):
        self.x = x0 if x0 is not None else np.zeros(5)
        self.P = P0 if P0 is not None else np.eye(5) * 10

    def predict(self, dt):
        """预测步（简化匀速模型）"""
        F = np.eye(5)
        F[0, 2] = dt
        F[1, 3] = dt
        Q = np.diag([0.1, 0.1, 0.5, 0.5, 0.01]) * dt
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def update_gps(self, z):
        """GPS更新：观测x,y"""
        H = np.zeros((2, 5))
        H[0, 0] = 1; H[1, 1] = 1
        R = np.eye(2) * 4.0  # GPS噪声
        self._update(z, H, R)

    def update_encoder(self, z):
        """编码器更新：观测速度"""
        H = np.zeros((1, 5))
        speed = np.sqrt(self.x[2]**2 + self.x[3]**2) + 1e-6
        H[0, 2] = self.x[2] / speed
        H[0, 3] = self.x[3] / speed
        R = np.eye(1) * 0.5
        self._update(np.array([z]), H, R)

    def update_imu(self, z):
        """IMU更新：观测yaw_rate"""
        H = np.zeros((1, 5))
        H[0, 4] = 1
        R = np.eye(1) * 0.1
        self._update(np.array([z[2]]), H, R)

    def _update(self, z, H, R):
        y = z - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(5) - K @ H) @ self.P

# ============================================================
# 4. 仿真
# ============================================================

def run_sensor_fusion():
    np.random.seed(42)
    dt = 0.02
    t_total = 30.0
    steps = int(t_total / dt)

    # 真实轨迹：圆形运动
    vehicle = VehicleModel(0, 0, 0)
    for i in range(steps):
        t = i * dt
        speed = 2.0 + 0.5 * np.sin(0.2 * t)
        yaw_rate = 0.3 * np.cos(0.1 * t)
        vehicle.step(speed, yaw_rate, dt)

    true_traj = vehicle.get_trajectory()

    # 传感器
    gps = GPSSensor(noise=3.0, rate=1.0)
    imu = IMUSensor(accel_noise=0.5, gyro_noise=0.05, rate=50.0)
    enc = EncoderSensor(noise=0.5, rate=20.0)

    # 三种融合方案对比
    ekf_all = EKF()     # GPS+IMU+编码器
    ekf_gps_imu = EKF() # GPS+IMU
    ekf_gps_only = EKF()# 仅GPS

    est_all, est_gps_imu, est_gps_only = [], [], []

    for i in range(steps):
        t = i * dt
        # 预测
        ekf_all.predict(dt)
        ekf_gps_imu.predict(dt)
        ekf_gps_only.predict(dt)

        # GPS
        gps_data = gps.read(true_traj[i], t)
        if gps_data is not None:
            ekf_all.update_gps(gps_data)
            ekf_gps_imu.update_gps(gps_data)
            ekf_gps_only.update_gps(gps_data)

        # IMU
        imu_data = imu.read(true_traj[i], dt, t)
        if imu_data is not None:
            ekf_all.update_imu(imu_data)
            ekf_gps_imu.update_imu(imu_data)

        # 编码器
        enc_data = enc.read(true_traj[i], t)
        if enc_data is not None:
            ekf_all.update_encoder(enc_data)

        est_all.append(ekf_all.x[:2].copy())
        est_gps_imu.append(ekf_gps_imu.x[:2].copy())
        est_gps_only.append(ekf_gps_only.x[:2].copy())

    est_all = np.array(est_all)
    est_gps_imu = np.array(est_gps_imu)
    est_gps_only = np.array(est_gps_only)

    # 误差计算
    err_all = np.sqrt(np.sum((true_traj[:, :2] - est_all)**2, axis=1))
    err_gps_imu = np.sqrt(np.sum((true_traj[:, :2] - est_gps_imu)**2, axis=1))
    err_gps_only = np.sqrt(np.sum((true_traj[:, :2] - est_gps_only)**2, axis=1))

    t_arr = np.arange(steps) * dt

    # 可视化
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))

    # 轨迹对比
    ax = axes[0, 0]
    ax.plot(true_traj[:,0], true_traj[:,1], 'k-', linewidth=2, label='真实')
    ax.plot(est_all[:,0], est_all[:,1], 'g-', alpha=0.8, label='GPS+IMU+编码器')
    ax.plot(est_gps_imu[:,0], est_gps_imu[:,1], 'b-', alpha=0.8, label='GPS+IMU')
    ax.plot(est_gps_only[:,0], est_gps_only[:,1], 'r-', alpha=0.8, label='仅GPS')
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    ax.set_title('轨迹对比'); ax.legend(); ax.grid(True, alpha=0.3); ax.set_aspect('equal')

    # 误差时域
    ax = axes[0, 1]
    ax.plot(t_arr, err_all, 'g-', label='GPS+IMU+编码器')
    ax.plot(t_arr, err_gps_imu, 'b-', label='GPS+IMU')
    ax.plot(t_arr, err_gps_only, 'r-', label='仅GPS')
    ax.set_xlabel('时间 (s)'); ax.set_ylabel('定位误差 (m)')
    ax.set_title('误差时域对比'); ax.legend(); ax.grid(True, alpha=0.3)

    # 误差CDF
    ax = axes[1, 0]
    for err, label, color in [(err_all,'GPS+IMU+编码器','g'),
                               (err_gps_imu,'GPS+IMU','b'),
                               (err_gps_only,'仅GPS','r')]:
        sorted_err = np.sort(err)
        cdf = np.arange(1, len(sorted_err)+1) / len(sorted_err)
        ax.plot(sorted_err, cdf, color=color, label=label)
    ax.set_xlabel('误差 (m)'); ax.set_ylabel('CDF')
    ax.set_title('误差累积分布'); ax.legend(); ax.grid(True, alpha=0.3)

    # 统计柱状图
    ax = axes[1, 1]
    labels = ['GPS+IMU+编码', 'GPS+IMU', '仅GPS']
    means = [np.mean(err_all), np.mean(err_gps_imu), np.mean(err_gps_only)]
    stds = [np.std(err_all), np.std(err_gps_imu), np.std(err_gps_only)]
    bars = ax.bar(labels, means, yerr=stds, color=['g','b','r'], alpha=0.7, capsize=5)
    ax.set_ylabel('平均误差 (m)'); ax.set_title('定位精度对比')
    ax.grid(True, alpha=0.3, axis='y')

    plt.tight_layout()
    plt.savefig('multi_sensor_kalman.png', dpi=150)
    plt.show()

    print("=" * 50)
    print("多传感器卡尔曼融合仿真结果")
    print("=" * 50)
    for name, err in zip(labels, [err_all, err_gps_imu, err_gps_only]):
        print(f"{name:16s} | RMSE: {np.mean(err):.3f}m | 最大: {np.max(err):.3f}m | CEP95: {np.percentile(err, 95):.3f}m")

if __name__ == '__main__':
    run_sensor_fusion()
