# -*- coding: utf-8 -*-
"""
电池模型仿真 — SOC估计 + 放电曲线 + 内阻模型

仿真内容：
  1. 等效电路模型（Thevenin模型）
  2. 放电曲线仿真（不同倍率）
  3. SOC估计（安时积分 + 卡尔曼滤波）
  4. 内阻特性分析

依赖：numpy, matplotlib
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os

# ============================================================
# 全局设置
# ============================================================


def main():
    plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False

    np.random.seed(42)

    # ============================================================
    # 1. 电池参数（锂电池）
    # ============================================================
    Q_nominal = 2.2        # 标称容量 (Ah)
    V_max = 4.2            # 满充电压 (V)
    V_min = 3.0            # 截止电压 (V)
    V_nominal = 3.7        # 标称电压 (V)

    # 内阻模型参数（SOC相关）
    def battery_R0(soc):
        """欧姆内阻 (Ω) — 随SOC变化"""
        # 两端大，中间小
        if soc > 0.9:
            return 0.05 + 0.1 * (soc - 0.9) / 0.1
        elif soc < 0.1:
            return 0.05 + 0.15 * (0.1 - soc) / 0.1
        else:
            return 0.05

    R1 = 0.02       # 极化电阻 (Ω)
    C1 = 2000       # 极化电容 (F)

    # OCV-SOC曲线（多项式拟合）
    def ocv_curve(soc):
        """开路电压 vs SOC"""
        s = np.clip(soc, 0, 1)
        return 3.0 + 1.2*s - 0.8*s**2 + 0.5*s**3 - 0.2*s**4 + 0.1*s**5

    # ============================================================
    # 2. 仿真参数
    # ============================================================
    dt = 1.0             # 仿真步长 1s
    T_total = 3600 * 3   # 3小时
    N = int(T_total / dt)
    t = np.arange(N) * dt

    # ============================================================
    # 3. 放电曲线仿真（不同倍率）
    # ============================================================
    print("放电曲线仿真...")
    C_rates = [0.5, 1.0, 2.0, 3.0]
    discharge_results = {}

    for C_rate in C_rates:
        I_discharge = C_rate * Q_nominal  # 放电电流 (A)
        soc = 1.0
        V_terminal = []
        soc_curve = []
        V1 = 0  # RC网络电压

        for i in range(N):
            # 计算端电压
            R0 = battery_R0(soc)
            V_ocv = ocv_curve(soc)
            V_t = V_ocv - I_discharge * R0 - V1

            V_terminal.append(V_t)
            soc_curve.append(soc)

            # RC网络动态
            dV1 = (-V1 / (R1 * C1) + I_discharge / C1) * dt
            V1 += dV1

            # SOC更新（安时积分）
            dsoc = -I_discharge * dt / (Q_nominal * 3600)
            soc += dsoc

            if soc <= 0 or V_t < V_min:
                break

        discharge_results[C_rate] = {
            'V': np.array(V_terminal),
            'soc': np.array(soc_curve),
            't': np.arange(len(V_terminal)) * dt / 60  # 转为分钟
        }
        print(f"  {C_rate}C放电: {len(V_terminal)/60:.1f}分钟, 终止SOC={soc:.3f}")

    # ============================================================
    # 4. SOC估计仿真
    # ============================================================
    print("\nSOC估计仿真...")
    # 工况：间歇放电
    I_profile = np.zeros(N)
    for i in range(N):
        cycle = t[i] % 600  # 10分钟周期
        if cycle < 300:
            I_profile[i] = 3.0 + 0.5 * np.sin(2 * np.pi * t[i] / 30)  # 放电
        elif cycle < 360:
            I_profile[i] = -2.0  # 回充
        else:
            I_profile[i] = 0.05  # 搁置

    # 真实SOC（安时积分，高精度）
    soc_true = np.zeros(N)
    soc_true[0] = 0.8
    V1_true = 0
    V_term = np.zeros(N)

    for i in range(N):
        R0 = battery_R0(soc_true[i])
        V_ocv = ocv_curve(soc_true[i])
        V_term[i] = V_ocv - I_profile[i] * R0 - V1_true

        if i < N - 1:
            dV1 = (-V1_true / (R1 * C1) + I_profile[i] / C1) * dt
            V1_true += dV1
            soc_true[i+1] = soc_true[i] - I_profile[i] * dt / (Q_nominal * 3600)
            soc_true[i+1] = np.clip(soc_true[i+1], 0, 1)

    # 安时积分法（有累积误差）
    soc_ah = np.zeros(N)
    soc_ah[0] = 0.8
    Ah_efficiency = 0.98  # 充电效率
    for i in range(N - 1):
        dsoc = -I_profile[i] * dt / (Q_nominal * 3600)
        if I_profile[i] < 0:  # 充电有损耗
            dsoc *= Ah_efficiency
        soc_ah[i+1] = soc_ah[i] + dsoc
    # 添加初始误差
    soc_ah[0] = 0.82  # 初始SOC偏差

    # 卡尔曼滤波SOC估计
    print("卡尔曼SOC估计...")
    # 扩展卡尔曼：状态=[SOC], 观测=[端电压]
    soc_ekf = np.zeros(N)
    soc_ekf[0] = 0.8  # 初始估计
    P_soc = 0.01
    Q_soc = 1e-6
    R_soc = 0.001  # 电压测量噪声

    V1_est = 0  # RC网络估计

    for i in range(N - 1):
        # 预测
        dsoc = -I_profile[i] * dt / (Q_nominal * 3600)
        soc_pred = soc_ekf[i] + dsoc
        P_pred = P_soc + Q_soc

        # 观测更新
        V1_est_pred = V1_est + (-V1_est / (R1 * C1) + I_profile[i] / C1) * dt
        R0_est = battery_R0(soc_pred)
        V_pred = ocv_curve(soc_pred) - I_profile[i] * R0_est - V1_est_pred

        # 观测：端电压
        V_meas = V_term[i] + np.random.randn() * 0.005  # 加测量噪声

        # 卡尔曼增益（线性化OCV曲线斜率）
        dOCV = (ocv_curve(soc_pred + 0.001) - ocv_curve(soc_pred - 0.001)) / 0.002
        H = dOCV
        S = H * P_pred * H + R_soc
        K = P_pred * H / S

        # 更新
        soc_ekf[i+1] = soc_pred + K * (V_meas - V_pred)
        soc_ekf[i+1] = np.clip(soc_ekf[i+1], 0, 1)
        P_soc = (1 - K * H) * P_pred

        V1_est = V1_est_pred

    # ============================================================
    # 5. 内阻特性分析
    # ============================================================
    print("\n内阻特性分析...")
    soc_range = np.linspace(0, 1, 100)
    R0_range = [battery_R0(s) for s in soc_range]
    OCV_range = [ocv_curve(s) for s in soc_range]

    # ============================================================
    # 6. 绘图
    # ============================================================
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    # (a) 放电曲线
    ax1 = axes[0, 0]
    for C_rate, data in discharge_results.items():
        ax1.plot(data['t'], data['V'], label=f'{C_rate}C')
    ax1.axhline(y=V_min, color='r', linestyle='--', alpha=0.5, label='截止电压')
    ax1.set_xlabel('时间 (min)')
    ax1.set_ylabel('端电压 (V)')
    ax1.set_title('(a) 不同倍率放电曲线')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # (b) SOC vs OCV
    ax2 = axes[0, 1]
    ax2.plot(soc_range * 100, OCV_range, 'b-', linewidth=2)
    ax2.set_xlabel('SOC (%)')
    ax2.set_ylabel('开路电压 (V)')
    ax2.set_title('(b) SOC-OCV关系曲线')
    ax2.grid(True, alpha=0.3)

    # (c) SOC估计对比
    ax3 = axes[1, 0]
    t_min = t / 60
    ax3.plot(t_min, soc_true * 100, 'k-', linewidth=2, label='真实SOC')
    ax3.plot(t_min, soc_ah * 100, 'b--', alpha=0.7, label='安时积分法')
    ax3.plot(t_min, soc_ekf * 100, 'r-', alpha=0.7, label='卡尔曼滤波')
    ax3.set_xlabel('时间 (min)')
    ax3.set_ylabel('SOC (%)')
    ax3.set_title('(c) SOC估计方法对比')
    ax3.legend()
    ax3.grid(True, alpha=0.3)

    # (d) 内阻特性
    ax4 = axes[1, 1]
    ax4.plot(soc_range * 100, np.array(R0_range) * 1000, 'b-', linewidth=2)
    ax4.set_xlabel('SOC (%)')
    ax4.set_ylabel('内阻 (mΩ)')
    ax4.set_title('(d) 电池内阻 vs SOC')
    ax4.grid(True, alpha=0.3)

    plt.tight_layout()
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'battery_model_simulation.png')
    plt.savefig(out, dpi=150, bbox_inches='tight')
    print(f"图表已保存: {out}")

    # 误差统计
    mask = t < T_total
    rmse_ah = np.sqrt(np.mean((soc_true - soc_ah)**2)) * 100
    rmse_ekf = np.sqrt(np.mean((soc_true - soc_ekf)**2)) * 100
    print(f"\n安时积分法 SOC RMSE: {rmse_ah:.2f}%")
    print(f"卡尔曼滤波 SOC RMSE: {rmse_ekf:.2f}%")

    print("\n=== 电池模型仿真完成 ===")



if __name__ == '__main__':
    main()
