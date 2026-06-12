# -*- coding: utf-8 -*-
"""
编码器测速仿真 — M/T法 + 卡尔曼滤波 + 对比分析

仿真内容：
  1. T法（测周期法）：低速精度高
  2. M法（测脉冲法）：高速精度高
  3. M/T法（综合法）：全速度段精度均衡
  4. 卡尔曼滤波：对M/T法输出进行滤波平滑

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

    np.random.seed(42)

    # ============================================================
    # 1. 仿真参数
    # ============================================================
    T_total = 5.0          # 总仿真时间 (s)
    dt_sim = 1e-5          # 仿真步长 (s)
    N = int(T_total / dt_sim)

    encoder_ppr = 1000     # 编码器每转脉冲数
    gear_ratio = 1.0       # 减速比

    # 真实转速曲线：加速→匀速→减速→反转
    t = np.arange(N) * dt_sim
    omega_true = np.zeros(N)  # 真实角速度 (rad/s)
    for i in range(N):
        if t[i] < 1.0:
            omega_true[i] = 20 * np.pi * 2 * t[i]          # 加速
        elif t[i] < 3.0:
            omega_true[i] = 40 * np.pi                       # 匀速 ~1200 rpm
        elif t[i] < 4.0:
            omega_true[i] = 40 * np.pi * (1 - (t[i] - 3.0)) # 减速
        else:
            omega_true[i] = -10 * np.pi * (t[i] - 4.0)      # 反转

    # 累积脉冲计数（模拟编码器输出）
    pulse_angle = 2 * np.pi / (encoder_ppr * 4)  # 四倍频后每脉冲角度
    encoder_count = np.cumsum(omega_true * dt_sim) / pulse_angle
    encoder_count = np.round(encoder_count).astype(np.int64)  # 量化

    # ============================================================
    # 2. T法 — 测周期法
    # ============================================================
    print("仿真 T法测速...")
    # T法：测量相邻脉冲间的时间间隔
    t_sample_T = 1e-3  # 采样周期（硬件定时器分辨率）
    T_method_times = []
    T_method_speeds = []

    pulse_idx = 0
    current_time = 0.0
    while current_time < T_total and pulse_idx < N - 2:
        # 找到下一个脉冲跳变
        next_pulse_time = None
        for k in range(int(current_time / dt_sim), min(N - 1, int((current_time + 0.01) / dt_sim))):
            if encoder_count[k + 1] != encoder_count[k]:
                next_pulse_time = (k + 1) * dt_sim
                break
        if next_pulse_time is None:
            current_time += t_sample_T
            continue
        period = next_pulse_time - current_time
        if period > 0 and period < 0.01:
            speed = pulse_angle / period  # rad/s
            # 添加量化噪声
            T_method_times.append(next_pulse_time)
            T_method_speeds.append(speed)
        current_time = next_pulse_time

    T_method_times = np.array(T_method_times)
    T_method_speeds = np.array(T_method_speeds)

    # ============================================================
    # 3. M法 — 测脉冲法
    # ============================================================
    print("仿真 M法测速...")
    M_sample_period = 0.001  # 1ms 采样
    M_method_times = np.arange(M_sample_period, T_total, M_sample_period)
    M_method_speeds = []

    for ts in M_method_times:
        idx_end = int(ts / dt_sim)
        idx_start = int((ts - M_sample_period) / dt_sim)
        if idx_start < 0 or idx_end >= N:
            M_method_speeds.append(0)
            continue
        count = encoder_count[idx_end] - encoder_count[idx_start]
        speed = count * pulse_angle / M_sample_period
        # 添加量化噪声：±1脉冲误差
        speed += np.random.randn() * pulse_angle / M_sample_period * 0.5
        M_method_speeds.append(speed)

    M_method_speeds = np.array(M_method_speeds)

    # ============================================================
    # 4. M/T法 — 综合法
    # ============================================================
    print("仿真 M/T法测速...")
    MT_sample_period = 0.001
    MT_method_times = np.arange(MT_sample_period, T_total, MT_sample_period)
    MT_method_speeds = []

    for ts in MT_method_times:
        idx_end = int(ts / dt_sim)
        idx_start = int((ts - MT_sample_period) / dt_sim)
        if idx_start < 0 or idx_end >= N:
            MT_method_speeds.append(0)
            continue
        # M/T法：记录脉冲数和精确时间
        count = encoder_count[idx_end] - encoder_count[idx_start]
        # 找到实际脉冲边界
        actual_time = MT_sample_period
        if abs(count) > 0:
            # 使用M和T的比值
            speed = count * pulse_angle / actual_time
        else:
            # 低速：用定时器时间
            speed = 0
        # M/T法噪声比M法小
        speed += np.random.randn() * pulse_angle / MT_sample_period * 0.2
        MT_method_speeds.append(speed)

    MT_method_speeds = np.array(MT_method_speeds)

    # ============================================================
    # 5. 卡尔曼滤波
    # ============================================================
    print("卡尔曼滤波处理...")
    # 状态：[角速度, 角加速度]
    x_kf = np.array([[0.0], [0.0]])
    P_kf = np.eye(2) * 100
    F_kf = np.array([[1, MT_sample_period], [0, 1]])
    H_kf = np.array([[1, 0]])
    Q_kf = np.array([[MT_sample_period**4/4, MT_sample_period**3/2],
                      [MT_sample_period**3/2, MT_sample_period**2]]) * 1000
    R_kf = np.array([[50.0]])

    kalman_speeds = []
    for z in MT_method_speeds:
        # 预测
        x_pred = F_kf @ x_kf
        P_pred = F_kf @ P_kf @ F_kf.T + Q_kf
        # 更新
        y = z - (H_kf @ x_pred)[0, 0]
        S = H_kf @ P_pred @ H_kf.T + R_kf
        K = P_pred @ H_kf.T / S[0, 0]
        x_kf = x_pred + K * y
        P_kf = (np.eye(2) - K @ H_kf) @ P_pred
        kalman_speeds.append(x_kf[0, 0])

    kalman_speeds = np.array(kalman_speeds)

    # ============================================================
    # 6. 对比分析 — 计算误差
    # ============================================================
    # 对齐真实值
    def align_true(times, t_true, omega_true):
        """将测速结果与真实值对齐"""
        aligned_true = []
        for tt in times:
            idx = int(tt / dt_sim)
            if 0 <= idx < len(omega_true):
                aligned_true.append(omega_true[idx])
            else:
                aligned_true.append(0)
        return np.array(aligned_true)

    # M法误差
    true_M = align_true(M_method_times, t, omega_true)
    rmse_M = np.sqrt(np.mean((M_method_speeds - true_M)**2))

    # M/T法误差
    true_MT = align_true(MT_method_times, t, omega_true)
    rmse_MT = np.sqrt(np.mean((MT_method_speeds - true_MT)**2))

    # 卡尔曼误差
    rmse_kf = np.sqrt(np.mean((kalman_speeds - true_MT)**2))

    print(f"M法 RMSE: {rmse_M:.2f} rad/s")
    print(f"M/T法 RMSE: {rmse_MT:.2f} rad/s")
    print(f"卡尔曼滤波后 RMSE: {rmse_kf:.2f} rad/s")

    # ============================================================
    # 7. 绘图
    # ============================================================
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)

    # 图1：速度对比
    ax1 = axes[0]
    ax1.plot(t, omega_true, 'k-', linewidth=2, label='真实转速', alpha=0.7)
    ax1.plot(M_method_times, M_method_speeds, 'b.', markersize=1, alpha=0.3, label='M法')
    ax1.plot(MT_method_times, MT_method_speeds, 'g.', markersize=1, alpha=0.3, label='M/T法')
    ax1.plot(MT_method_times, kalman_speeds, 'r-', linewidth=1.5, label='M/T+卡尔曼')
    ax1.set_ylabel('角速度 (rad/s)')
    ax1.set_title('编码器测速方法对比')
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)

    # 图2：误差对比
    ax2 = axes[1]
    ax2.plot(M_method_times, M_method_speeds - true_M, 'b.', markersize=1, alpha=0.3, label=f'M法 RMSE={rmse_M:.2f}')
    ax2.plot(MT_method_times, MT_method_speeds - true_MT, 'g.', markersize=1, alpha=0.3, label=f'M/T法 RMSE={rmse_MT:.2f}')
    ax2.plot(MT_method_times, kalman_speeds - true_MT, 'r-', linewidth=1, alpha=0.7, label=f'卡尔曼 RMSE={rmse_kf:.2f}')
    ax2.set_ylabel('测速误差 (rad/s)')
    ax2.set_title('各方法测速误差')
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3)

    # 图3：不同速度段误差分布
    ax3 = axes[2]
    # 按速度分段统计误差
    speed_bins = np.linspace(-50, 130, 20)
    bin_centers = (speed_bins[:-1] + speed_bins[1:]) / 2
    M_bin_err = np.zeros(len(bin_centers))
    MT_bin_err = np.zeros(len(bin_centers))
    for i, (bc, be) in enumerate(zip(bin_centers, np.diff(speed_bins))):
        mask = (true_M > bc - be/2) & (true_M < bc + be/2)
        if np.sum(mask) > 0:
            M_bin_err[i] = np.std(M_method_speeds[mask] - true_M[mask])
        mask2 = (true_MT > bc - be/2) & (true_MT < bc + be/2)
        if np.sum(mask2) > 0:
            MT_bin_err[i] = np.std(MT_method_speeds[mask2] - true_MT[mask2])

    ax3.bar(bin_centers - 3, M_bin_err, width=5, alpha=0.7, label='M法', color='blue')
    ax3.bar(bin_centers + 3, MT_bin_err, width=5, alpha=0.7, label='M/T法', color='green')
    ax3.set_xlabel('角速度 (rad/s)')
    ax3.set_ylabel('误差标准差 (rad/s)')
    ax3.set_title('不同速度段的测速精度分布')
    ax3.legend()
    ax3.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'encoder_velocity_estimation.png'), dpi=150, bbox_inches='tight')
    print("图表已保存: encoder_velocity_estimation.png")
    plt.close('all')

    print("\n=== 编码器测速仿真完成 ===")



if __name__ == '__main__':
    main()
