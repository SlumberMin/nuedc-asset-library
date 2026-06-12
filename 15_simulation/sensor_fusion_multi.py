#!/usr/bin/env python3
"""
多传感器融合仿真
================
融合方法: 加权平均、卡尔曼滤波、互补滤波、自适应融合
传感器: 编码器(速度)、IMU(加速度)、超声波(距离)、光电(位置)
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
matplotlib.rcParams['axes.unicode_minus'] = False


def main():
    dt = 0.01
    T = 20.0
    N = int(T / dt)
    t = np.arange(N) * dt
    np.random.seed(42)

    # ===== 真实信号: 小车运动 =====
    # 先加速，再匀速，再减速
    v_true = np.zeros(N)
    for i in range(N):
        ti = t[i]
        if ti < 3.0:
            v_true[i] = 2.0 * ti / 3.0  # 加速到2 m/s
        elif ti < 12.0:
            v_true[i] = 2.0  # 匀速
        elif ti < 15.0:
            v_true[i] = 2.0 * (15.0 - ti) / 3.0  # 减速
        else:
            v_true[i] = 0.0

    x_true = np.cumsum(v_true) * dt  # 真实位置

    # ===== 模拟各传感器 =====
    # 1. 编码器: 低噪声，但有量化误差
    ppr = 360
    wheel_circ = 0.204  # 周长(m)
    v_enc_raw = v_true + np.random.normal(0, 0.05, N)
    # 量化
    v_encoder = np.round(v_enc_raw * ppr / wheel_circ) * wheel_circ / ppr

    # 2. IMU加速度积分: 有漂移
    acc_true = np.gradient(v_true, dt)
    acc_imu = acc_true + 0.02 + np.random.normal(0, 0.1, N)  # 偏置+噪声
    v_imu = np.cumsum(acc_imu) * dt  # 积分有漂移

    # 3. 超声波测距: 低频(10Hz), 有噪声
    v_ultrasonic = np.zeros(N)
    for i in range(N):
        if i % 10 == 0:  # 10Hz
            v_ultrasonic[i] = v_true[i] + np.random.normal(0, 0.15)
        elif i > 0:
            v_ultrasonic[i] = v_ultrasonic[i-1]  # 保持上次值

    # 4. 光电编码器(高精度): 高频，低噪声
    v_photo = v_true + np.random.normal(0, 0.02, N)

    # ===== 融合方法 =====

    # 方法1: 简单加权平均
    def weighted_avg(sensors, weights):
        result = np.zeros_like(sensors[0])
        for s, w in zip(sensors, weights):
            result += w * s
        return result

    # 方法2: 卡尔曼滤波融合
    def kalman_fusion(z_list, R_list, dt):
        N = len(z_list[0])
        x = 0.0
        v = 0.0
        P = np.eye(2)
        A = np.array([[1, dt], [0, 1]])
        Q = np.array([[dt**4/4, dt**3/2], [dt**3/2, dt**2]]) * 0.5
        x_est = np.zeros(N)
        for i in range(N):
            # 预测
            xp = A @ np.array([x, v])
            Pp = A @ P @ A.T + Q
            # 多传感器顺序更新
            x, v = xp
            P = Pp
            for z, R in zip([s[i] for s in z_list], R_list):
                H = np.array([[0, 1]])  # 观测速度
                S = H @ P @ H.T + R
                K = P @ H.T @ np.linalg.inv(S)
                innov = z - (H @ np.array([x, v]))
                state = np.array([x, v]) + (K @ innov).flatten()
                x, v = state
                P = (np.eye(2) - K @ H) @ P
            x_est[i] = v
        return x_est

    # 方法3: 互补滤波
    def complementary_fusion(fast, slow, alpha=0.95):
        result = np.zeros_like(fast)
        result[0] = fast[0]
        for i in range(1, len(fast)):
            result[i] = alpha * fast[i] + (1-alpha) * slow[i]
        return result

    # 方法4: 自适应加权(基于方差估计)
    def adaptive_fusion(sensors, window=50):
        N = len(sensors[0])
        result = np.zeros(N)
        for i in range(N):
            start = max(0, i - window)
            variances = []
            values = []
            for s in sensors:
                seg = s[start:i+1]
                var = np.var(seg) if len(seg) > 1 else 1.0
                variances.append(max(var, 1e-6))
                values.append(s[i])
            inv_vars = [1.0/v for v in variances]
            total = sum(inv_vars)
            weights = [iv/total for iv in inv_vars]
            result[i] = sum(w*v for w, v in zip(weights, values))
        return result

    # ===== 执行融合 =====
    v_weighted = weighted_avg(
        [v_encoder, v_imu, v_ultrasonic, v_photo],
        [0.3, 0.1, 0.1, 0.5]
    )
    v_kalman = kalman_fusion(
        [v_encoder, v_ultrasonic, v_photo],
        [0.05**2, 0.15**2, 0.02**2],
        dt
    )
    v_comp = complementary_fusion(v_photo, v_imu, 0.95)
    v_adaptive = adaptive_fusion([v_encoder, v_imu, v_ultrasonic, v_photo])

    # ===== 计算RMSE =====
    def rmse(est, true):
        return np.sqrt(np.mean((est - true)**2))

    rmses = {
        '编码器': rmse(v_encoder, v_true),
        'IMU积分': rmse(v_imu, v_true),
        '超声波': rmse(v_ultrasonic, v_true),
        '光电': rmse(v_photo, v_true),
        '加权平均': rmse(v_weighted, v_true),
        '卡尔曼融合': rmse(v_kalman, v_true),
        '互补滤波': rmse(v_comp, v_true),
        '自适应融合': rmse(v_adaptive, v_true),
    }

    # ===== 绘图 =====
    fig, axes = plt.subplots(3, 2, figsize=(16, 14))
    fig.suptitle('多传感器融合仿真', fontsize=16, fontweight='bold')

    # 各传感器原始数据
    ax = axes[0, 0]
    ax.plot(t, v_true, 'k-', linewidth=2, label='真实值')
    ax.plot(t, v_encoder, 'b-', alpha=0.5, linewidth=0.5, label=f'编码器(RMSE={rmses["编码器"]:.3f})')
    ax.plot(t, v_photo, 'g-', alpha=0.5, linewidth=0.5, label=f'光电(RMSE={rmses["光电"]:.3f})')
    ax.set_title('高精度传感器 vs 真实值')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('速度 (m/s)')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[0, 1]
    ax.plot(t, v_true, 'k-', linewidth=2, label='真实值')
    ax.plot(t, v_imu, 'r-', alpha=0.5, linewidth=0.5, label=f'IMU积分(RMSE={rmses["IMU积分"]:.3f})')
    ax.plot(t, v_ultrasonic, 'm-', alpha=0.5, linewidth=0.5, label=f'超声波(RMSE={rmses["超声波"]:.3f})')
    ax.set_title('低精度传感器 vs 真实值')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('速度 (m/s)')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # 融合结果对比
    ax = axes[1, 0]
    ax.plot(t, v_true, 'k-', linewidth=2, label='真实值')
    ax.plot(t, v_weighted, 'b-', linewidth=1, alpha=0.8, label=f'加权平均(RMSE={rmses["加权平均"]:.3f})')
    ax.plot(t, v_kalman, 'r-', linewidth=1, alpha=0.8, label=f'卡尔曼融合(RMSE={rmses["卡尔曼融合"]:.3f})')
    ax.set_title('融合方法对比(1)')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('速度 (m/s)')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[1, 1]
    ax.plot(t, v_true, 'k-', linewidth=2, label='真实值')
    ax.plot(t, v_comp, 'g-', linewidth=1, alpha=0.8, label=f'互补滤波(RMSE={rmses["互补滤波"]:.3f})')
    ax.plot(t, v_adaptive, 'm-', linewidth=1, alpha=0.8, label=f'自适应融合(RMSE={rmses["自适应融合"]:.3f})')
    ax.set_title('融合方法对比(2)')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('速度 (m/s)')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # RMSE柱状图
    ax = axes[2, 0]
    names = list(rmses.keys())
    vals = list(rmses.values())
    colors = ['steelblue', 'coral', 'gold', 'limegreen',
              'blue', 'red', 'green', 'purple']
    bars = ax.bar(range(len(names)), vals, color=colors)
    ax.set_xticks(range(len(names)))
    ax.set_xticklabels(names, fontsize=8, rotation=20)
    ax.set_title('各方法RMSE对比')
    ax.set_ylabel('RMSE')
    ax.grid(True, alpha=0.3, axis='y')
    for bar, val in zip(bars, vals):
        ax.text(bar.get_x()+bar.get_width()/2., bar.get_height(),
                f'{val:.4f}', ha='center', va='bottom', fontsize=7)

    # 总结
    ax = axes[2, 1]
    ax.axis('off')
    best = min(rmses, key=rmses.get)
    worst = max(rmses, key=rmses.get)
    summary = (
        "多传感器融合总结\n"
        "================\n\n"
        f"最佳方法: {best} (RMSE={rmses[best]:.4f})\n"
        f"最差方法: {worst} (RMSE={rmses[worst]:.4f})\n\n"
        "传感器特性:\n"
        "• 编码器: 高频低噪，量化误差\n"
        "• IMU: 高频但有漂移\n"
        "• 超声波: 低频大噪声\n"
        "• 光电: 高精度高速率\n\n"
        "融合建议:\n"
        "• 卡尔曼融合适合实时系统\n"
        "• 自适应融合适合未知噪声环境\n"
        "• 互补滤波适合双传感器场景"
    )
    ax.text(0.05, 0.95, summary, transform=ax.transAxes, fontsize=10,
            verticalalignment='top', fontfamily='monospace',
            bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))

    plt.tight_layout()
    out_dir = os.path.dirname(os.path.abspath(__file__))
    plt.savefig(os.path.join(out_dir, 'sensor_fusion_multi_result.png'), dpi=150, bbox_inches='tight')
    print("图表已保存: sensor_fusion_multi_result.png")
    plt.close('all')

    print(f"\n仿真完成!")
    print(f"最佳融合方法: {best}, RMSE={rmses[best]:.4f}")


if __name__ == '__main__':
    main()
