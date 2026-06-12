# -*- coding: utf-8 -*-
"""
IMU姿态估计仿真 — 互补滤波 + 扩展卡尔曼 + Mahony + 对比分析

仿真内容：
  1. 互补滤波（一阶）：简单高效
  2. 扩展卡尔曼滤波（EKF）：最优估计
  3. Mahony互补滤波：四元数实现，适合嵌入式
  4. 三者对比分析

依赖：numpy, matplotlib
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# ============================================================
# 全局设置
# ============================================================


def main():
    plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False

    np.random.seed(0)

    # ============================================================
    # 1. 仿真参数
    # ============================================================
    dt = 0.01              # 采样周期 10ms
    T_total = 20.0         # 总仿真时间
    N = int(T_total / dt)
    t = np.arange(N) * dt

    # 传感器噪声参数
    gyro_noise = 0.01      # 陀螺仪噪声 (rad/s)
    gyro_bias_true = np.array([0.02, -0.01, 0.015])  # 真实零偏
    accel_noise = 0.5      # 加速度计噪声 (m/s²)

    # ============================================================
    # 2. 生成真实姿态轨迹
    # ============================================================
    # 模拟飞行器翻转运动
    roll_true = np.zeros(N)
    pitch_true = np.zeros(N)
    yaw_true = np.zeros(N)

    for i in range(N):
        if t[i] < 5:
            # 缓慢滚转
            roll_true[i] = 30 * np.sin(2 * np.pi * 0.2 * t[i]) * np.pi / 180
            pitch_true[i] = 10 * np.sin(2 * np.pi * 0.1 * t[i]) * np.pi / 180
        elif t[i] < 10:
            # 快速机动
            roll_true[i] = 60 * np.sin(2 * np.pi * 0.5 * t[i]) * np.pi / 180
            pitch_true[i] = 45 * np.sin(2 * np.pi * 0.3 * t[i]) * np.pi / 180
        elif t[i] < 15:
            # 大角度翻转
            roll_true[i] = 90 * np.sin(2 * np.pi * 0.2 * t[i]) * np.pi / 180
            pitch_true[i] = 70 * np.sin(2 * np.pi * 0.15 * t[i]) * np.pi / 180
        else:
            # 恢复平稳
            roll_true[i] = 20 * np.sin(2 * np.pi * 0.1 * t[i]) * np.pi / 180
            pitch_true[i] = 15 * np.sin(2 * np.pi * 0.08 * t[i]) * np.pi / 180
        yaw_true[i] = 45 * np.sin(2 * np.pi * 0.05 * t[i]) * np.pi / 180

    # 真实角速度（数值微分）
    omega_true = np.zeros((N, 3))
    omega_true[1:, 0] = np.diff(roll_true) / dt
    omega_true[1:, 1] = np.diff(pitch_true) / dt
    omega_true[1:, 2] = np.diff(yaw_true) / dt

    # 真实重力分量
    g = 9.81
    accel_true = np.zeros((N, 3))
    for i in range(N):
        cr, sr = np.cos(roll_true[i]), np.sin(roll_true[i])
        cp, sp = np.cos(pitch_true[i]), np.sin(pitch_true[i])
        accel_true[i, 0] = -g * sp
        accel_true[i, 1] = g * sr * cp
        accel_true[i, 2] = g * cr * cp

    # ============================================================
    # 3. 生成传感器数据
    # ============================================================
    # 陀螺仪 = 真实角速度 + 零偏 + 噪声
    gyro_data = omega_true + gyro_bias_true + np.random.randn(N, 3) * gyro_noise

    # 加速度计 = 真实加速度 + 噪声
    accel_data = accel_true + np.random.randn(N, 3) * accel_noise

    # ============================================================
    # 4. 互补滤波
    # ============================================================
    print("互补滤波...")
    alpha = 0.98  # 陀螺仪权重
    roll_comp = np.zeros(N)
    pitch_comp = np.zeros(N)
    yaw_comp = np.zeros(N)

    for i in range(1, N):
        # 陀螺仪积分
        roll_gyro = roll_comp[i-1] + gyro_data[i, 0] * dt
        pitch_gyro = pitch_comp[i-1] + gyro_data[i, 1] * dt
        yaw_gyro = yaw_comp[i-1] + gyro_data[i, 2] * dt

        # 加速度计解算
        ax, ay, az = accel_data[i]
        roll_acc = np.arctan2(ay, az)
        pitch_acc = np.arctan2(-ax, np.sqrt(ay**2 + az**2))

        # 互补融合
        roll_comp[i] = alpha * roll_gyro + (1 - alpha) * roll_acc
        pitch_comp[i] = alpha * pitch_gyro + (1 - alpha) * pitch_acc
        yaw_comp[i] = yaw_gyro  # 偏航角无参考，纯积分

    # ============================================================
    # 5. 扩展卡尔曼滤波 (EKF)
    # ============================================================
    print("EKF滤波...")
    # 状态：[roll, pitch, yaw, bias_x, bias_y, bias_z]
    x_ekf = np.zeros(6)
    P_ekf = np.eye(6) * 0.1
    Q_ekf = np.diag([0.001, 0.001, 0.001, 0.0001, 0.0001, 0.0001])
    R_ekf = np.diag([1.0, 1.0, 1.0])  # 加速度计测量噪声

    roll_ekf = np.zeros(N)
    pitch_ekf = np.zeros(N)
    yaw_ekf = np.zeros(N)

    for i in range(N):
        # 预测：角速度积分
        gyro_corrected = gyro_data[i] - x_ekf[3:6]
        x_ekf[0] += gyro_corrected[0] * dt
        x_ekf[1] += gyro_corrected[1] * dt
        x_ekf[2] += gyro_corrected[2] * dt

        # 雅可比矩阵
        F = np.eye(6)
        F[0, 3] = -dt
        F[1, 4] = -dt
        F[2, 5] = -dt

        P_ekf = F @ P_ekf @ F.T + Q_ekf

        # 加速度计更新（仅roll和pitch）
        ax, ay, az = accel_data[i]
        roll_meas = np.arctan2(ay, az)
        pitch_meas = np.arctan2(-ax, np.sqrt(ay**2 + az**2))
        z = np.array([roll_meas, pitch_meas, x_ekf[2]])  # yaw无观测

        H = np.eye(3, 6)
        H[2, :] = 0
        H[2, 2] = 1

        y = z - H @ x_ekf
        S = H @ P_ekf @ H.T + R_ekf
        K = P_ekf @ H.T @ np.linalg.inv(S)
        x_ekf = x_ekf + K @ y
        P_ekf = (np.eye(6) - K @ H) @ P_ekf

        roll_ekf[i] = x_ekf[0]
        pitch_ekf[i] = x_ekf[1]
        yaw_ekf[i] = x_ekf[2]

    # ============================================================
    # 6. Mahony互补滤波（四元数）
    # ============================================================
    print("Mahony滤波...")
    Kp_mahony = 2.0
    Ki_mahony = 0.01
    q = np.array([1.0, 0.0, 0.0, 0.0])  # 四元数
    e_int = np.zeros(3)  # 积分误差

    roll_mahony = np.zeros(N)
    pitch_mahony = np.zeros(N)
    yaw_mahony = np.zeros(N)

    def quat_to_euler(q):
        """四元数转欧拉角"""
        w, x, y, z = q
        roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        pitch = np.arcsin(np.clip(2*(w*y - z*x), -1, 1))
        yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(z*z + y*y))
        return roll, pitch, yaw

    def quat_mult(q1, q2):
        """四元数乘法"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ])

    for i in range(N):
        ax, ay, az = accel_data[i]
        a_norm = np.sqrt(ax**2 + ay**2 + az**2)
        if a_norm > 0:
            ax, ay, az = ax/a_norm, ay/a_norm, az/a_norm

        # 重力方向（从四元数计算）
        w, qx, qy, qz = q
        vx = 2*(qx*qz - w*qy)
        vy = 2*(w*qx + qy*qz)
        vz = 1 - 2*(qx**2 + qy**2)

        # 误差（叉积）
        e = np.array([ay*vz - az*vy, az*vx - ax*vz, ax*vy - ay*vx])
        e_int += e * Ki_mahony * dt

        # 角速度修正
        gyro_m = gyro_data[i] + Kp_mahony * e + e_int

        # 四元数更新
        omega_q = np.array([0, gyro_m[0], gyro_m[1], gyro_m[2]])
        q_dot = 0.5 * quat_mult(q, omega_q)
        q = q + q_dot * dt
        q = q / np.linalg.norm(q)  # 归一化

        roll_mahony[i], pitch_mahony[i], yaw_mahony[i] = quat_to_euler(q)

    # ============================================================
    # 7. 误差分析
    # ============================================================
    rmse_comp = np.sqrt(np.mean((roll_comp - roll_true)**2 + (pitch_comp - pitch_true)**2))
    rmse_ekf = np.sqrt(np.mean((roll_ekf - roll_true)**2 + (pitch_ekf - pitch_true)**2))
    rmse_mahony = np.sqrt(np.mean((roll_mahony - roll_true)**2 + (pitch_mahony - pitch_true)**2))

    print(f"互补滤波 RMSE: {np.degrees(rmse_comp):.3f}°")
    print(f"EKF RMSE: {np.degrees(rmse_ekf):.3f}°")
    print(f"Mahony RMSE: {np.degrees(rmse_mahony):.3f}°")

    # ============================================================
    # 8. 绘图
    # ============================================================
    fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True)

    # Roll对比
    ax1 = axes[0]
    ax1.plot(t, np.degrees(roll_true), 'k-', linewidth=2, label='真实值')
    ax1.plot(t, np.degrees(roll_comp), 'b-', alpha=0.7, label='互补滤波')
    ax1.plot(t, np.degrees(roll_ekf), 'r-', alpha=0.7, label='EKF')
    ax1.plot(t, np.degrees(roll_mahony), 'g-', alpha=0.7, label='Mahony')
    ax1.set_ylabel('Roll (°)')
    ax1.set_title('IMU姿态估计 — Roll角对比')
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)

    # Pitch对比
    ax2 = axes[1]
    ax2.plot(t, np.degrees(pitch_true), 'k-', linewidth=2, label='真实值')
    ax2.plot(t, np.degrees(pitch_comp), 'b-', alpha=0.7, label='互补滤波')
    ax2.plot(t, np.degrees(pitch_ekf), 'r-', alpha=0.7, label='EKF')
    ax2.plot(t, np.degrees(pitch_mahony), 'g-', alpha=0.7, label='Mahony')
    ax2.set_ylabel('Pitch (°)')
    ax2.set_title('IMU姿态估计 — Pitch角对比')
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3)

    # 误差对比
    ax3 = axes[2]
    ax3.plot(t, np.degrees(roll_comp - roll_true), 'b-', alpha=0.5, label='互补滤波')
    ax3.plot(t, np.degrees(roll_ekf - roll_true), 'r-', alpha=0.5, label='EKF')
    ax3.plot(t, np.degrees(roll_mahony - roll_true), 'g-', alpha=0.5, label='Mahony')
    ax3.set_ylabel('Roll误差 (°)')
    ax3.set_title('Roll角估计误差')
    ax3.legend(loc='upper right')
    ax3.grid(True, alpha=0.3)

    # EKF零偏估计
    ax4 = axes[3]
    ax4.plot(t, gyro_bias_true[0] * np.ones(N), 'k--', linewidth=2, label='真实零偏X')
    ax4.plot(t, gyro_bias_true[1] * np.ones(N), 'k--', linewidth=2, label='真实零偏Y')
    ax4.plot(t, gyro_bias_true[2] * np.ones(N), 'k--', linewidth=2, label='真实零偏Z')
    ax4.plot(t, roll_ekf * 0, 'r-', alpha=0)  # placeholder
    # EKF估计的零偏在状态中
    bias_est = np.zeros((N, 3))
    x_tmp = np.zeros(6)
    P_tmp = np.eye(6) * 0.1
    for i in range(N):
        gc = gyro_data[i] - x_tmp[3:6]
        x_tmp[0] += gc[0]*dt; x_tmp[1] += gc[1]*dt; x_tmp[2] += gc[2]*dt
        F = np.eye(6); F[0,3]=-dt; F[1,4]=-dt; F[2,5]=-dt
        P_tmp = F@P_tmp@F.T + Q_ekf
        ax_, ay_, az_ = accel_data[i]
        rm = np.arctan2(ay_, az_); pm = np.arctan2(-ax_, np.sqrt(ay_**2+az_**2))
        z = np.array([rm, pm, x_tmp[2]])
        H = np.eye(3,6); H[2,:]=0; H[2,2]=1
        y = z - H@x_tmp; S = H@P_tmp@H.T + R_ekf; K = P_tmp@H.T@np.linalg.inv(S)
        x_tmp = x_tmp + K@y; P_tmp = (np.eye(6)-K@H)@P_tmp
        bias_est[i] = x_tmp[3:6]

    ax4.plot(t, bias_est[:, 0], 'r-', label='估计零偏X')
    ax4.plot(t, bias_est[:, 1], 'b-', label='估计零偏Y')
    ax4.plot(t, bias_est[:, 2], 'g-', label='估计零偏Z')
    ax4.set_xlabel('时间 (s)')
    ax4.set_ylabel('零偏 (rad/s)')
    ax4.set_title('EKF陀螺仪零偏估计')
    ax4.legend(loc='upper right')
    ax4.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'imu_attitude_estimation.png'), dpi=150, bbox_inches='tight')
    print("图表已保存: imu_attitude_estimation.png")
    plt.close('all')

    print("\n=== IMU姿态估计仿真完成 ===")



if __name__ == '__main__':
    main()
