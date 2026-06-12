"""
IMU传感器融合仿真
方法：互补滤波 + 卡尔曼滤波 + Mahony互补滤波器
应用：加速度计+陀螺仪融合估计姿态角（Roll, Pitch）
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt



def main():
    plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
    plt.rcParams['axes.unicode_minus'] = False

    # ============================================================
    # 1. 生成真实运动轨迹和IMU仿真数据
    # ============================================================
    dt = 0.01       # 100Hz采样
    t_end = 20.0
    t = np.arange(0, t_end, dt)
    N = len(t)

    # 真实姿态角（弧度）：正弦摆动
    roll_true = 0.3 * np.sin(2 * np.pi * 0.2 * t)         # ±17°
    pitch_true = 0.2 * np.sin(2 * np.pi * 0.15 * t + 0.5) # ±11.5°

    # 真实角速度（rad/s）= 姿态角的导数
    roll_rate_true = 0.3 * 2 * np.pi * 0.2 * np.cos(2 * np.pi * 0.2 * t)
    pitch_rate_true = 0.2 * 2 * np.pi * 0.15 * np.cos(2 * np.pi * 0.15 * t + 0.5)

    # 陀螺仪数据（带漂移和噪声）
    gyro_bias_roll = 0.02     # rad/s 常值漂移
    gyro_bias_pitch = -0.015
    gyro_noise_std = 0.01     # rad/s

    np.random.seed(42)
    gyro_roll = roll_rate_true + gyro_bias_roll + np.random.normal(0, gyro_noise_std, N)
    gyro_pitch = pitch_rate_true + gyro_bias_pitch + np.random.normal(0, gyro_noise_std, N)

    # 加速度计数据（估计roll和pitch，带噪声）
    # acc_roll = atan2(ay, az), acc_pitch = atan2(-ax, sqrt(ay^2+az^2))
    acc_noise_std = 0.05  # rad
    acc_roll = roll_true + np.random.normal(0, acc_noise_std, N)
    acc_pitch = pitch_true + np.random.normal(0, acc_noise_std, N)

    # ============================================================
    # 2. 互补滤波器
    # ============================================================
    def complementary_filter(acc_roll, acc_pitch, gyro_roll, gyro_pitch, dt, alpha=0.98):
        """
        互补滤波：高通陀螺仪 + 低通加速度计
        alpha: 陀螺仪权重（越大越信任陀螺仪）
        """
        N = len(acc_roll)
        roll = np.zeros(N)
        pitch = np.zeros(N)

        for i in range(1, N):
            # 陀螺仪积分
            roll_gyro = roll[i-1] + gyro_roll[i] * dt
            pitch_gyro = pitch[i-1] + gyro_pitch[i] * dt

            # 互补融合
            roll[i] = alpha * roll_gyro + (1 - alpha) * acc_roll[i]
            pitch[i] = alpha * pitch_gyro + (1 - alpha) * acc_pitch[i]

        return roll, pitch

    roll_cf, pitch_cf = complementary_filter(acc_roll, acc_pitch, gyro_roll, gyro_pitch, dt)

    # ============================================================
    # 3. 卡尔曼滤波器（简化版，角度+偏置状态）
    # ============================================================
    def kalman_filter_imu(acc_angle, gyro_rate, dt, Q_angle=0.001, Q_bias=0.003, R_measure=0.03):
        """
        一维卡尔曼滤波器
        状态: [angle, bias]
        测量: acc_angle（加速度计角度）
        输入: gyro_rate（陀螺仪角速度）
        """
        N = len(acc_angle)
        angle = np.zeros(N)
        bias = np.zeros(N)
        P = np.array([[1, 0], [0, 1]])  # 协方差矩阵

        Q = np.array([[Q_angle, 0], [0, Q_bias]])
        R = R_measure

        for i in range(1, N):
            # 预测
            rate = gyro_rate[i] - bias[i-1]
            angle_pred = angle[i-1] + rate * dt
            bias_pred = bias[i-1]

            P_pred = np.array([
                [P[0,0] + dt*(dt*P[1,1] - P[0,1] - P[1,0]) + Q_angle, P[0,1] - dt*P[1,1]],
                [P[1,0] - dt*P[1,1], P[1,1] + Q_bias]
            ])

            # 更新
            S = P_pred[0, 0] + R
            K = np.array([P_pred[0, 0] / S, P_pred[1, 0] / S])  # 卡尔曼增益

            y = acc_angle[i] - angle_pred  # 残差
            angle[i] = angle_pred + K[0] * y
            bias[i] = bias_pred + K[1] * y

            P = np.array([
                [(1 - K[0]) * P_pred[0, 0], (1 - K[0]) * P_pred[0, 1]],
                [-K[1] * P_pred[0, 0] + P_pred[1, 0], -K[1] * P_pred[0, 1] + P_pred[1, 1]]
            ])

        return angle, bias

    roll_kf, roll_bias_kf = kalman_filter_imu(acc_roll, gyro_roll, dt)
    pitch_kf, pitch_bias_kf = kalman_filter_imu(acc_pitch, gyro_pitch, dt)

    # ============================================================
    # 4. Mahony互补滤波器（AHRS）
    # ============================================================
    def mahony_filter(acc_roll, acc_pitch, gyro_roll, gyro_pitch, dt, Kp=0.5, Ki=0.002):
        """
        Mahony互补滤波器（简化版）
        使用PI控制器估计并补偿陀螺仪偏置，同时融合加速度计
        核心思想：低频信任加速度计，高频信任陀螺仪，通过PI自适应调整偏置
        """
        N = len(acc_roll)
        roll = np.zeros(N)
        pitch = np.zeros(N)
        bias_roll = 0.0
        bias_pitch = 0.0
        integral_err_roll = 0.0
        integral_err_pitch = 0.0

        for i in range(1, N):
            # 加速度计与当前估计的角度误差（用于修正陀螺仪偏置）
            err_roll = acc_roll[i] - roll[i-1]
            err_pitch = acc_pitch[i] - pitch[i-1]

            # PI控制器估计偏置（缓慢修正漂移）
            integral_err_roll += err_roll * dt
            integral_err_pitch += err_pitch * dt
            integral_err_roll = np.clip(integral_err_roll, -2.0, 2.0)
            integral_err_pitch = np.clip(integral_err_pitch, -2.0, 2.0)

            bias_roll = Ki * integral_err_roll
            bias_pitch = Ki * integral_err_pitch

            # 修正角速度 = 原始角速度 - 偏置估计 + 比例修正
            corrected_rate_roll = gyro_roll[i] - bias_roll + Kp * err_roll
            corrected_rate_pitch = gyro_pitch[i] - bias_pitch + Kp * err_pitch

            # 积分得到角度
            roll[i] = roll[i-1] + corrected_rate_roll * dt
            pitch[i] = pitch[i-1] + corrected_rate_pitch * dt

        return roll, pitch

    roll_mah, pitch_mah = mahony_filter(acc_roll, acc_pitch, gyro_roll, gyro_pitch, dt)

    # ============================================================
    # 5. 计算误差
    # ============================================================
    def rmse(est, true):
        return np.sqrt(np.mean((est - true)**2))

    print("=" * 60)
    print("IMU传感器融合算法对比")
    print("=" * 60)
    print(f"{'方法':<12} {'Roll RMSE(°)':<15} {'Pitch RMSE(°)':<15}")
    print("-" * 42)
    for name, r, p in [('纯陀螺仪积分', None, None),
                         ('互补滤波', roll_cf, pitch_cf),
                         ('卡尔曼滤波', roll_kf, pitch_kf),
                         ('Mahony', roll_mah, pitch_mah)]:
        if r is None:
            # 纯陀螺仪积分
            r_int = np.cumsum(gyro_roll) * dt
            p_int = np.cumsum(gyro_pitch) * dt
            print(f"{name:<12} {np.degrees(rmse(r_int, roll_true)):<15.2f} {np.degrees(rmse(p_int, pitch_true)):<15.2f}")
        else:
            print(f"{name:<12} {np.degrees(rmse(r, roll_true)):<15.2f} {np.degrees(rmse(p, pitch_true)):<15.2f}")

    # ============================================================
    # 6. 绘图
    # ============================================================
    fig, axes = plt.subplots(3, 2, figsize=(16, 14))

    # 纯陀螺仪积分对比
    roll_gyro_int = np.cumsum(gyro_roll) * dt
    pitch_gyro_int = np.cumsum(gyro_pitch) * dt

    # Roll对比
    ax = axes[0, 0]
    ax.plot(t, np.degrees(roll_true), 'k-', linewidth=2, label='真实值')
    ax.plot(t, np.degrees(acc_roll), 'gray', alpha=0.4, label='加速度计')
    ax.plot(t, np.degrees(roll_gyro_int), 'm:', linewidth=1, label='陀螺积分(漂移)')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('Roll (°)')
    ax.set_title('Roll角：原始传感器数据')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # Pitch对比
    ax = axes[0, 1]
    ax.plot(t, np.degrees(pitch_true), 'k-', linewidth=2, label='真实值')
    ax.plot(t, np.degrees(acc_pitch), 'gray', alpha=0.4, label='加速度计')
    ax.plot(t, np.degrees(pitch_gyro_int), 'm:', linewidth=1, label='陀螺积分(漂移)')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('Pitch (°)')
    ax.set_title('Pitch角：原始传感器数据')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # Roll融合对比
    ax = axes[1, 0]
    ax.plot(t, np.degrees(roll_true), 'k-', linewidth=2, label='真实值')
    ax.plot(t, np.degrees(roll_cf), 'b-', linewidth=1, alpha=0.8, label='互补滤波')
    ax.plot(t, np.degrees(roll_kf), 'r-', linewidth=1, alpha=0.8, label='卡尔曼滤波')
    ax.plot(t, np.degrees(roll_mah), 'g-', linewidth=1, alpha=0.8, label='Mahony')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('Roll (°)')
    ax.set_title('Roll角：三种融合算法对比')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # Pitch融合对比
    ax = axes[1, 1]
    ax.plot(t, np.degrees(pitch_true), 'k-', linewidth=2, label='真实值')
    ax.plot(t, np.degrees(pitch_cf), 'b-', linewidth=1, alpha=0.8, label='互补滤波')
    ax.plot(t, np.degrees(pitch_kf), 'r-', linewidth=1, alpha=0.8, label='卡尔曼滤波')
    ax.plot(t, np.degrees(pitch_mah), 'g-', linewidth=1, alpha=0.8, label='Mahony')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('Pitch (°)')
    ax.set_title('Pitch角：三种融合算法对比')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # Roll误差对比
    ax = axes[2, 0]
    ax.plot(t, np.degrees(roll_cf - roll_true), 'b-', linewidth=0.8, label='互补滤波')
    ax.plot(t, np.degrees(roll_kf - roll_true), 'r-', linewidth=0.8, label='卡尔曼')
    ax.plot(t, np.degrees(roll_mah - roll_true), 'g-', linewidth=0.8, label='Mahony')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('误差 (°)')
    ax.set_title('Roll角估计误差')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # Pitch误差对比
    ax = axes[2, 1]
    ax.plot(t, np.degrees(pitch_cf - pitch_true), 'b-', linewidth=0.8, label='互补滤波')
    ax.plot(t, np.degrees(pitch_kf - pitch_true), 'r-', linewidth=0.8, label='卡尔曼')
    ax.plot(t, np.degrees(pitch_mah - pitch_true), 'g-', linewidth=0.8, label='Mahony')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('误差 (°)')
    ax.set_title('Pitch角估计误差')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    plt.suptitle('IMU传感器融合仿真（互补滤波 + 卡尔曼 + Mahony）', fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sensor_fusion_imu.png'), dpi=150, bbox_inches='tight')
    print("\n图表已保存: sensor_fusion_imu.png")
    plt.close('all')



if __name__ == '__main__':
    main()
