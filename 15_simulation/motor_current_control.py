# -*- coding: utf-8 -*-
"""
电机电流控制仿真 — PI控制 + 前馈 + 解耦

仿真内容：
  1. 电流PI控制器（带抗积分饱和）
  2. 前馈解耦（d-q轴解耦）
  3. FOC电流环响应分析
  4. 参数扰动下的鲁棒性测试

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
    # 1. 电机参数
    # ============================================================
    Rs = 0.5       # 定子电阻 (Ω)
    Ld = 0.005     # d轴电感 (H)
    Lq = 0.008     # q轴电感 (H)
    Ke = 0.05      # 反电动势常数 (V·s/rad)
    p = 4          # 极对数
    J = 0.001      # 转动惯量 (kg·m²)
    B = 0.0001     # 摩擦系数

    # ============================================================
    # 2. PI控制器参数
    # ============================================================
    # d轴PI
    Kp_d = 10.0
    Ki_d = 1000.0
    # q轴PI
    Kp_q = 10.0
    Ki_q = 1000.0
    # 抗积分饱和限幅
    I_limit = 20.0  # A
    U_limit = 24.0  # V (母线电压)

    # ============================================================
    # 3. 仿真参数
    # ============================================================
    dt = 1e-5       # 仿真步长 10μs
    T_total = 0.1   # 仿真时间
    N = int(T_total / dt)
    t = np.arange(N) * dt

    # ============================================================
    # 4. 电流参考信号
    # ============================================================
    Id_ref = np.zeros(N)        # d轴电流参考 = 0（最大转矩控制）
    Iq_ref = np.zeros(N)        # q轴电流参考

    for i in range(N):
        if t[i] < 0.02:
            Iq_ref[i] = 0           # 空载
        elif t[i] < 0.04:
            Iq_ref[i] = 5.0         # 阶跃5A
        elif t[i] < 0.06:
            Iq_ref[i] = 10.0        # 阶跃10A
        elif t[i] < 0.08:
            Iq_ref[i] = 3.0 * np.sin(2 * np.pi * 50 * t[i])  # 正弦
        else:
            Iq_ref[i] = -5.0        # 反向

    # ============================================================
    # 5. 仿真函数：不同控制策略
    # ============================================================
    def simulate_current_control(use_ff=False, use_decouple=False, param_error=0.0):
        """
        仿真电流控制环
        参数：
          use_ff: 是否使用前馈
          use_decouple: 是否使用解耦
          param_error: 参数偏差比例 (0~1)
        返回：
          Id, Iq, Ud, Uq, 误差
        """
        # 带参数偏差的电机模型
        Rs_m = Rs * (1 + param_error * 0.5)
        Ld_m = Ld * (1 + param_error * 0.3)
        Lq_m = Lq * (1 + param_error * 0.3)

        # 状态
        Id = np.zeros(N)
        Iq = np.zeros(N)
        Ud_out = np.zeros(N)
        Uq_out = np.zeros(N)

        # PI状态
        integral_d = 0.0
        integral_q = 0.0
        omega_e = 2 * np.pi * 200  # 电角速度 (rad/s) — 假设固定转速

        for i in range(1, N):
            # --- PI控制 ---
            err_d = Id_ref[i] - Id[i-1]
            err_q = Iq_ref[i] - Iq[i-1]

            integral_d += err_d * dt
            integral_q += err_q * dt

            # 抗积分饱和
            integral_d = np.clip(integral_d, -I_limit / Ki_d, I_limit / Ki_d)
            integral_q = np.clip(integral_q, -I_limit / Ki_q, I_limit / Ki_q)

            Ud_pi = Kp_d * err_d + Ki_d * integral_d
            Uq_pi = Kp_q * err_q + Ki_q * integral_q

            # --- 前馈补偿 ---
            Ud_ff = 0.0
            Uq_ff = 0.0
            if use_ff:
                Ud_ff = -Rs_m * Id_ref[i]  # 电阻压降前馈
                Uq_ff = Rs_m * Iq_ref[i] + Ke * omega_e  # 反电动势前馈

            # --- 交叉耦合解耦 ---
            Ud_dec = 0.0
            Uq_dec = 0.0
            if use_decouple:
                Ud_dec = -omega_e * Lq_m * Iq[i-1]   # -ωLq·Iq
                Uq_dec = omega_e * Ld_m * Id[i-1]      # +ωLd·Id

            # 总输出
            Ud = Ud_pi + Ud_ff + Ud_dec
            Uq = Uq_pi + Uq_ff + Uq_dec

            # 电压限幅
            U_mag = np.sqrt(Ud**2 + Uq**2)
            if U_mag > U_limit:
                Ud *= U_limit / U_mag
                Uq *= U_limit / U_mag

            Ud_out[i] = Ud
            Uq_out[i] = Uq

            # --- 电机模型（真实参数） ---
            dId = (Ud - Rs * Id[i-1] + omega_e * Lq * Iq[i-1]) / Ld
            dIq = (Uq - Rs * Iq[i-1] - omega_e * Ld * Id[i-1] - Ke * omega_e) / Lq

            Id[i] = Id[i-1] + dId * dt
            Iq[i] = Iq[i-1] + dIq * dt

        return Id, Iq, Ud_out, Uq_out

    # ============================================================
    # 6. 运行不同方案
    # ============================================================
    print("仿真1: 纯PI控制...")
    Id_pi, Iq_pi, _, _ = simulate_current_control(use_ff=False, use_decouple=False)

    print("仿真2: PI + 前馈...")
    Id_ff, Iq_ff, _, _ = simulate_current_control(use_ff=True, use_decouple=False)

    print("仿真3: PI + 前馈 + 解耦...")
    Id_full, Iq_full, _, _ = simulate_current_control(use_ff=True, use_decouple=True)

    print("仿真4: 参数偏差 + 全功能...")
    Id_rob, Iq_rob, _, _ = simulate_current_control(use_ff=True, use_decouple=True, param_error=0.3)

    # ============================================================
    # 7. 误差统计
    # ============================================================
    def calc_rmse(ref, est):
        return np.sqrt(np.mean((ref - est)**2))

    # 仅在有参考信号时计算
    mask = Iq_ref != 0
    rmse_pi = calc_rmse(Iq_ref[mask], Iq_pi[mask])
    rmse_ff = calc_rmse(Iq_ref[mask], Iq_ff[mask])
    rmse_full = calc_rmse(Iq_ref[mask], Iq_full[mask])
    rmse_rob = calc_rmse(Iq_ref[mask], Iq_rob[mask])

    print(f"纯PI RMSE: {rmse_pi:.4f} A")
    print(f"PI+前馈 RMSE: {rmse_ff:.4f} A")
    print(f"PI+前馈+解耦 RMSE: {rmse_full:.4f} A")
    print(f"参数偏差30% RMSE: {rmse_rob:.4f} A")

    # ============================================================
    # 8. 绘图
    # ============================================================
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)

    # q轴电流跟踪
    ax1 = axes[0]
    ax1.plot(t * 1000, Iq_ref, 'k--', linewidth=2, label='参考电流')
    ax1.plot(t * 1000, Iq_pi, 'b-', alpha=0.7, label='纯PI')
    ax1.plot(t * 1000, Iq_ff, 'r-', alpha=0.7, label='PI+前馈')
    ax1.plot(t * 1000, Iq_full, 'g-', alpha=0.7, label='PI+前馈+解耦')
    ax1.set_ylabel('Iq (A)')
    ax1.set_title('q轴电流跟踪对比')
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)

    # d轴电流（应保持为0）
    ax2 = axes[1]
    ax2.plot(t * 1000, Id_pi, 'b-', alpha=0.7, label='纯PI')
    ax2.plot(t * 1000, Id_ff, 'r-', alpha=0.7, label='PI+前馈')
    ax2.plot(t * 1000, Id_full, 'g-', alpha=0.7, label='PI+前馈+解耦')
    ax2.plot(t * 1000, Id_rob, 'm-', alpha=0.7, label='参数偏差30%')
    ax2.set_ylabel('Id (A)')
    ax2.set_title('d轴电流（应为0）')
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3)

    # 误差对比
    ax3 = axes[2]
    ax3.plot(t * 1000, Iq_ref - Iq_pi, 'b-', alpha=0.5, label=f'纯PI RMSE={rmse_pi:.3f}')
    ax3.plot(t * 1000, Iq_ref - Iq_ff, 'r-', alpha=0.5, label=f'PI+前馈 RMSE={rmse_ff:.3f}')
    ax3.plot(t * 1000, Iq_ref - Iq_full, 'g-', alpha=0.5, label=f'全功能 RMSE={rmse_full:.3f}')
    ax3.set_xlabel('时间 (ms)')
    ax3.set_ylabel('电流误差 (A)')
    ax3.set_title('q轴电流跟踪误差')
    ax3.legend(loc='upper right')
    ax3.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'motor_current_control.png'), dpi=150, bbox_inches='tight')
    print("图表已保存: motor_current_control.png")
    plt.close('all')

    print("\n=== 电机电流控制仿真完成 ===")



if __name__ == '__main__':
    main()
