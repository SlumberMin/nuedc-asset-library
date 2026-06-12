"""
传感器融合仿真 - 编码器 + IMU(陀螺仪/加速度计)
适用于电赛平衡小车、AGV导航等场景

依赖: pip install numpy matplotlib
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# ============ 互补滤波器 ============
class ComplementaryFilter:
    """互补滤波: 融合陀螺仪(高频)和加速度计(低频)"""
    def __init__(self, alpha=0.98, dt=0.01):
        self.alpha = alpha
        self.dt = dt
        self.angle = 0

    def update(self, gyro_rate, accel_angle):
        self.angle = self.alpha * (self.angle + gyro_rate * self.dt) + \
                     (1 - self.alpha) * accel_angle
        return self.angle

# ============ 扩展卡尔曼滤波器(简化) ============
class EKFFusion:
    """
    扩展卡尔曼滤波器: 融合编码器和IMU
    状态: [位置, 速度, 偏航角, 陀螺仪偏置]
    """
    def __init__(self, dt=0.01):
        self.dt = dt
        self.x = np.zeros(4)  # [pos, vel, yaw, gyro_bias]
        self.P = np.eye(4) * 10
        # 过程噪声
        self.Q = np.diag([0.01, 0.1, 0.01, 0.001])
        # 编码器观测噪声
        self.R_enc = np.array([[0.1]])
        # IMU角度观测噪声
        self.R_imu = np.array([[0.5]])

    def predict(self, gyro_rate, accel=0):
        """预测: 使用陀螺仪积分"""
        dt = self.dt
        F = np.eye(4)
        F[0, 1] = dt  # pos += vel * dt
        F[2, 3] = 0   # yaw通过观测更新

        # 状态预测
        self.x[0] += self.x[1] * dt
        self.x[2] += (gyro_rate - self.x[3]) * dt  # 积分陀螺仪(减去偏置)
        self.x[2] = self._normalize_angle(self.x[2])

        self.P = F @ self.P @ F.T + self.Q
        return self.x.copy()

    def update_encoder(self, encoder_pos):
        """编码器位置更新"""
        H = np.array([[1, 0, 0, 0]])
        z = np.array([encoder_pos])
        y = z - H @ self.x
        S = H @ self.P @ H.T + self.R_enc
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + (K @ y).flatten()
        self.P = (np.eye(4) - K @ H) @ self.P

    def update_imu_angle(self, accel_angle):
        """IMU加速度计角度更新"""
        H = np.array([[0, 0, 1, 0]])
        z = np.array([accel_angle])
        y = z - H @ self.x
        # 处理角度环绕
        y[0] = self._normalize_angle(y[0])
        S = H @ self.P @ H.T + self.R_imu
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + (K @ y).flatten()
        self.x[2] = self._normalize_angle(self.x[2])
        self.P = (np.eye(4) - K @ H) @ self.P

    def _normalize_angle(self, a):
        while a > np.pi: a -= 2*np.pi
        while a < -np.pi: a += 2*np.pi
        return a

# ============ 传感器模拟 ============
class SensorSimulator:
    """模拟编码器、陀螺仪、加速度计"""
    def __init__(self, dt=0.01):
        self.dt = dt
        # 传感器噪声参数
        self.enc_noise_std = 0.05      # 编码器位置噪声 (m)
        self.gyro_noise_std = 0.02     # 陀螺仪噪声 (rad/s)
        self.gyro_bias = 0.01          # 陀螺仪常值偏置 (rad/s)
        self.accel_noise_std = 0.3     # 加速度计噪声 (rad)
        self.accel_lpf = 0             # 加速度计低通滤波

    def generate_trajectory(self, steps):
        """生成真实轨迹"""
        t = np.arange(steps) * self.dt
        # 圆弧运动
        true_pos = 2.0 * np.sin(0.5 * t)
        true_vel = 2.0 * 0.5 * np.cos(0.5 * t)
        true_yaw = 0.5 * t
        true_yaw = np.arctan2(np.sin(true_yaw), np.cos(true_yaw))
        return t, true_pos, true_vel, true_yaw

    def read_encoder(self, true_pos):
        return true_pos + np.random.randn() * self.enc_noise_std

    def read_gyro(self, true_yaw_rate):
        return true_yaw_rate + self.gyro_bias + np.random.randn() * self.gyro_noise_std

    def read_accel(self, true_yaw):
        # 加速度计得到的角度(低频准确, 高频噪声大)
        return true_yaw + np.random.randn() * self.accel_noise_std

# ============ 仿真主程序 ============
def run_fusion_simulation():
    dt = 0.01
    steps = 1000
    sim = SensorSimulator(dt=dt)
    t, true_pos, true_vel, true_yaw = sim.generate_trajectory(steps)

    # 三种方案
    comp_filter = ComplementaryFilter(alpha=0.98, dt=dt)
    ekf = EKFFusion(dt=dt)

    # 存储结果
    comp_angles = []
    ekf_positions = []
    ekf_velocities = []
    ekf_angles = []
    enc_positions = []
    imu_angles = []

    for i in range(steps):
        # 读取传感器
        enc_pos = sim.read_encoder(true_pos[i])
        gyro_rate = sim.read_gyro(true_vel[i] * 0.3)  # 简化: 角速度与速度相关
        accel_angle = sim.read_accel(true_yaw[i])

        enc_positions.append(enc_pos)
        imu_angles.append(accel_angle)

        # 互补滤波(仅角度)
        comp_angle = comp_filter.update(gyro_rate, accel_angle)
        comp_angles.append(comp_angle)

        # EKF融合
        ekf.predict(gyro_rate)
        ekf.update_encoder(enc_pos)
        ekf.update_imu_angle(accel_angle)
        ekf_positions.append(ekf.x[0])
        ekf_velocities.append(ekf.x[1])
        ekf_angles.append(ekf.x[2])

    # 绘图
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))

    # 位置融合
    axes[0].plot(t, true_pos, 'g-', label='真实位置', linewidth=1.5)
    axes[0].plot(t, enc_positions, 'r.', label='编码器(含噪声)', markersize=1, alpha=0.3)
    axes[0].plot(t, ekf_positions, 'b-', label='EKF融合位置', linewidth=1.2)
    axes[0].set_ylabel('位置 (m)')
    axes[0].set_title('传感器融合: 编码器 + IMU')
    axes[0].legend(fontsize=9)
    axes[0].grid(True, alpha=0.3)

    # 速度估计
    axes[1].plot(t, true_vel, 'g-', label='真实速度', linewidth=1.5)
    axes[1].plot(t, ekf_velocities, 'b-', label='EKF估计速度', linewidth=1.2)
    axes[1].set_ylabel('速度 (m/s)')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    # 角度融合
    axes[2].plot(t, true_yaw, 'g-', label='真实航向角', linewidth=1.5)
    axes[2].plot(t, imu_angles, 'r.', label='加速度计(含噪声)', markersize=1, alpha=0.3)
    axes[2].plot(t, comp_angles, 'm-', label='互补滤波', linewidth=1)
    axes[2].plot(t, ekf_angles, 'b-', label='EKF融合', linewidth=1.2)
    axes[2].set_xlabel('时间 (s)')
    axes[2].set_ylabel('航向角 (rad)')
    axes[2].legend(fontsize=9)
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('sensor_fusion.png', dpi=150)
    plt.close('all')

    # 性能指标
    pos_enc_rmse = np.sqrt(np.mean((np.array(enc_positions) - true_pos)**2))
    pos_ekf_rmse = np.sqrt(np.mean((np.array(ekf_positions) - true_pos)**2))
    vel_ekf_rmse = np.sqrt(np.mean((np.array(ekf_velocities) - true_vel)**2))
    yaw_imu_rmse = np.sqrt(np.mean((np.array(imu_angles) - true_yaw)**2))
    yaw_comp_rmse = np.sqrt(np.mean((np.array(comp_angles) - true_yaw)**2))
    yaw_ekf_rmse = np.sqrt(np.mean((np.array(ekf_angles) - true_yaw)**2))

    print("=== 传感器融合性能对比 ===")
    print(f"位置 - 编码器RMSE: {pos_enc_rmse:.4f}m, EKF融合RMSE: {pos_ekf_rmse:.4f}m, "
          f"改善 {(1-pos_ekf_rmse/pos_enc_rmse)*100:.1f}%")
    print(f"速度 - EKF估计RMSE: {vel_ekf_rmse:.4f}m/s")
    print(f"角度 - 加速度计RMSE: {yaw_imu_rmse:.4f}rad, "
          f"互补滤波RMSE: {yaw_comp_rmse:.4f}rad, "
          f"EKF融合RMSE: {yaw_ekf_rmse:.4f}rad")

if __name__ == '__main__':
    run_fusion_simulation()
