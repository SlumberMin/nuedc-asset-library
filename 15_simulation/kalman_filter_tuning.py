#!/usr/bin/env python3
"""
卡尔曼滤波器调参仿真
====================
仿真内容：不同Q/R参数对滤波效果的影响
应用场景：编码器速度估计、IMU姿态估计、传感器融合
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
matplotlib.rcParams['axes.unicode_minus'] = False


def kalman_filter_1d(z_meas, dt, Q, R, x0=0.0):
    """1D卡尔曼滤波（恒速模型）"""
    N = len(z_meas)
    x_est = np.zeros(N)
    P = 1.0
    x = x0
    v = 0.0

    A = np.array([[1, dt], [0, 1]])
    H = np.array([1, 0])
    Q_mat = Q * np.array([[dt**4/4, dt**3/2], [dt**3/2, dt**2]])

    for i in range(N):
        # 预测
        x_pred = x + v * dt
        v_pred = v
        P_mat = np.array([[P, 0], [0, P]])
        P_pred = A @ P_mat @ A.T + Q_mat

        # 更新
        z = z_meas[i]
        S = H @ P_pred @ H + R
        if abs(S) < 1e-10:
            S = 1e-10
        K = P_pred @ H / S
        innov = z - (H @ np.array([x_pred, v_pred]))
        x = x_pred + K[0] * innov
        v = v_pred + K[1] * innov
        P = (1 - K[0]) * P_pred[0, 0]

        x_est[i] = x

    return x_est


def main():
    # ===== 仿真参数 =====
    dt = 0.01
    T = 5.0
    N = int(T / dt)
    t = np.arange(N) * dt

    # 真实信号: 正弦运动
    np.random.seed(42)
    x_true = np.sin(2 * np.pi * 0.5 * t) + 0.3 * np.sin(2 * np.pi * 1.2 * t)

    # 噪声测量
    noise_std = 0.3
    z_meas = x_true + np.random.normal(0, noise_std, N)

    # ===== 不同参数组合 =====
    params = [
        (0.1, 10.0, 'Q小/R大 (过度平滑)'),
        (1.0, 1.0, 'Q=R (平衡)'),
        (10.0, 0.1, 'Q大/R小 (跟踪快)'),
        (1.0, 10.0, 'Q中/R大'),
        (10.0, 1.0, 'Q大/R中'),
    ]

    fig, axes = plt.subplots(3, 2, figsize=(16, 14))
    fig.suptitle('卡尔曼滤波器调参仿真', fontsize=16, fontweight='bold')

    # 绘制测量和真实值
    ax = axes[0, 0]
    ax.plot(t, x_true, 'k-', linewidth=2, label='真实值', alpha=0.8)
    ax.plot(t, z_meas, '.', color='gray', markersize=1, alpha=0.3, label='测量值')
    ax.set_title('原始信号与测量')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('幅值')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 各参数对比
    ax = axes[0, 1]
    ax.plot(t, x_true, 'k-', linewidth=2, label='真实值', alpha=0.5)
    colors = ['blue', 'green', 'red', 'orange', 'purple']
    rmse_list = []
    labels_list = []

    for idx, (Q, R, label) in enumerate(params):
        x_est = kalman_filter_1d(z_meas, dt, Q, R)
        rmse = np.sqrt(np.mean((x_est - x_true)**2))
        rmse_list.append(rmse)
        labels_list.append(label)
        ax.plot(t, x_est, color=colors[idx], linewidth=1, label=f'{label} (RMSE={rmse:.3f})')

    ax.set_title('不同Q/R参数对比')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('幅值')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # RMSE柱状图
    ax = axes[1, 0]
    bars = ax.bar(range(len(rmse_list)), rmse_list, color=colors[:len(rmse_list)])
    ax.set_xticks(range(len(labels_list)))
    ax.set_xticklabels(labels_list, fontsize=8, rotation=15)
    ax.set_title('滤波RMSE对比')
    ax.set_ylabel('RMSE')
    ax.grid(True, alpha=0.3, axis='y')
    for bar, val in zip(bars, rmse_list):
        ax.text(bar.get_x() + bar.get_width()/2., bar.get_height(),
                f'{val:.4f}', ha='center', va='bottom', fontsize=8)

    # Q/R比值影响
    ax = axes[1, 1]
    qr_ratios = [0.01, 0.1, 0.5, 1.0, 2.0, 5.0, 10.0, 50.0]
    rmses = []
    for qr in qr_ratios:
        x_est = kalman_filter_1d(z_meas, dt, qr, 1.0)
        rmses.append(np.sqrt(np.mean((x_est - x_true)**2)))
    ax.semilogx(qr_ratios, rmses, 'bo-', linewidth=2)
    ax.set_title('Q/R比值 vs RMSE')
    ax.set_xlabel('Q/R比值')
    ax.set_ylabel('RMSE')
    ax.grid(True, alpha=0.3)

    # 最优参数局部放大
    ax = axes[2, 0]
    best_idx = np.argmin(rmse_list)
    Q_best, R_best, label_best = params[best_idx]
    x_est_best = kalman_filter_1d(z_meas, dt, Q_best, R_best)
    zoom_start = int(1.0 / dt)
    zoom_end = int(2.0 / dt)
    ax.plot(t[zoom_start:zoom_end], x_true[zoom_start:zoom_end], 'k-', linewidth=2, label='真实值')
    ax.plot(t[zoom_start:zoom_end], z_meas[zoom_start:zoom_end], '.', color='gray',
            markersize=2, alpha=0.5, label='测量值')
    ax.plot(t[zoom_start:zoom_end], x_est_best[zoom_start:zoom_end], 'r-', linewidth=1.5,
            label=f'最优滤波 ({label_best})')
    ax.set_title(f'最优参数局部放大 (RMSE={rmse_list[best_idx]:.4f})')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('幅值')
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    # 总结文字
    ax = axes[2, 1]
    ax.axis('off')
    summary = (
        "卡尔曼滤波器调参总结\n"
        "====================\n\n"
        "• Q(过程噪声): 控制对模型的信任度\n"
        "  Q小 → 过度平滑，延迟大\n"
        "  Q大 → 跟踪快，噪声大\n\n"
        "• R(测量噪声): 控制对测量的信任度\n"
        "  R小 → 信任测量，跟踪快\n"
        "  R大 → 不信任测量，平滑\n\n"
        "• Q/R比值决定滤波器特性\n"
        "  Q/R小 → 平滑滤波器\n"
        "  Q/R大 → 跟踪滤波器\n\n"
        f"最优参数: {label_best}\n"
        f"最优RMSE: {rmse_list[best_idx]:.4f}"
    )
    ax.text(0.1, 0.9, summary, transform=ax.transAxes, fontsize=11,
            verticalalignment='top', fontfamily='monospace',
            bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))

    plt.tight_layout()
    out_dir = os.path.dirname(os.path.abspath(__file__))
    plt.savefig(os.path.join(out_dir, 'kalman_filter_tuning_result.png'), dpi=150, bbox_inches='tight')
    print("图表已保存: kalman_filter_tuning_result.png")
    plt.close('all')

    print(f"\n仿真完成!")
    print(f"最优参数: {labels_list[best_idx]}, RMSE={rmse_list[best_idx]:.4f}")


if __name__ == '__main__':
    main()
