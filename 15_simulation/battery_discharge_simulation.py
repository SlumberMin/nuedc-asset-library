#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
电池放电仿真 — 不同负载下的放电曲线 + SOC估计
===============================================
模拟锂电池在不同放电倍率下的特性:
  - 放电曲线 (电压 vs SOC)
  - 内阻变化
  - 温升效应
  - SOC估计方法 (库仑积分 + OCV查表 + 卡尔曼滤波)
依赖: numpy, matplotlib
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

# ═══════════════════════════════════════════════════════════════
# 电池模型参数 (18650锂电池典型参数)
# ═══════════════════════════════════════════════════════════════

Q_NOMINAL = 2.6      # 标称容量 (Ah)
V_FULL = 4.2          # 满充电压 (V)
V_EMPTY = 3.0         # 放空电压 (V)
V_NOMINAL = 3.6       # 标称电压 (V)

# 内阻参数 (SOC相关)
R0_MIN = 0.05         # 最小内阻 (Ω, SOC=50%附近)
R0_MAX = 0.15         # 最大内阻 (Ω, SOC接近0%)

# 放电倍率 (C-rate)
C_RATES = [0.2, 0.5, 1.0, 2.0, 3.0]  # 0.2C, 0.5C, 1C, 2C, 3C

# 温度参数
T_AMBIENT = 25        # 环境温度 (°C)
T_COEFF = 0.3         # 温升系数 (°C/A²·Ω·热阻)

# ═══════════════════════════════════════════════════════════════
# 电池OCV-SOC查找表 (开路电压曲线)
# ═══════════════════════════════════════════════════════════════

# 典型锂电池OCV-SOC曲线
SOC_LUT = np.array([0, 0.05, 0.1, 0.15, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.85, 0.9, 0.95, 1.0])
OCV_LUT = np.array([3.0, 3.3, 3.45, 3.55, 3.6, 3.65, 3.7, 3.72, 3.75, 3.8, 3.85, 3.9, 4.0, 4.1, 4.2])

def ocv_from_soc(soc):
    """SOC查表得到OCV (线性插值)"""
    return np.interp(soc, SOC_LUT, OCV_LUT)

def internal_resistance(soc):
    """内阻随SOC变化 (U型曲线)"""
    # SOC在0.5附近内阻最小, 接近0或1时增大
    r = R0_MIN + (R0_MAX - R0_MIN) * (4 * (soc - 0.5)**2)
    # SOC接近0时急剧增大
    if soc < 0.1:
        r += (0.1 - soc) * 0.5
    return r

# ═══════════════════════════════════════════════════════════════
# 1. 放电曲线仿真
# ═══════════════════════════════════════════════════════════════

def simulate_discharge(c_rate, dt=1.0):
    """
    仿真单节电池放电过程
    c_rate: 放电倍率 (如1.0表示1C)
    dt: 仿真步长 (秒)
    返回: SOC, 电压, 电流, 温度, 时间
    """
    I_discharge = c_rate * Q_NOMINAL  # 放电电流 (A)
    
    soc = 1.0        # 初始SOC
    T = T_AMBIENT    # 初始温度
    
    soc_arr = [soc]
    v_arr = [ocv_from_soc(soc)]
    i_arr = [I_discharge]
    t_arr = [0.0]
    T_arr = [T]
    r_arr = [internal_resistance(soc)]
    
    t = 0.0
    while soc > 0.01 and t < 100 * 3600:  # 安全限制
        t += dt
        
        # 内阻
        R = internal_resistance(soc)
        
        # 端电压: V = OCV - I*R - 温度补偿
        ocv = ocv_from_soc(soc)
        v_term = ocv - I_discharge * R
        
        # 温度补偿 (低温降低容量)
        temp_factor = 1.0 - 0.005 * max(0, 25 - T)  # 25°C以下每度损失0.5%
        
        # SOC更新 (库仑积分)
        dQ = I_discharge * dt / 3600  # Ah
        soc -= dQ / (Q_NOMINAL * temp_factor)
        soc = max(soc, 0.0)
        
        # 温度更新 (简化热模型)
        P_heat = I_discharge**2 * R
        dT = (P_heat * T_COEFF - (T - T_AMBIENT) * 0.01) * dt  # 简化散热
        T += dT
        
        # 记录
        soc_arr.append(soc)
        v_arr.append(v_term)
        i_arr.append(I_discharge)
        t_arr.append(t)
        T_arr.append(T)
        r_arr.append(R)
        
        # 电压截止
        if v_term < V_EMPTY:
            break
    
    return (np.array(soc_arr), np.array(v_arr), np.array(i_arr),
            np.array(t_arr), np.array(T_arr), np.array(r_arr))

print("=" * 60)
print("电池放电仿真")
print("=" * 60)
print(f"电池容量: {Q_NOMINAL}Ah, 标称电压: {V_NOMINAL}V")
print(f"满充: {V_FULL}V, 放空: {V_EMPTY}V")
print(f"放电倍率: {C_RATES}C")

# 运行仿真
results = {}
for c_rate in C_RATES:
    soc, v, i, t, temp, r = simulate_discharge(c_rate)
    results[c_rate] = {
        'soc': soc, 'v': v, 'i': i, 't': t, 'temp': temp, 'r': r,
        'capacity_ah': t[-1] / 3600 * c_rate * Q_NOMINAL,
        'duration_h': t[-1] / 3600
    }
    print(f"  {c_rate}C: 放电时间={t[-1]/3600:.2f}h, "
          f"实际容量={t[-1]/3600 * c_rate * Q_NOMINAL:.3f}Ah, "
          f"最高温度={np.max(temp):.1f}°C")

# ═══════════════════════════════════════════════════════════════
# 2. SOC估计仿真
# ═══════════════════════════════════════════════════════════════

def soc_estimation_coulomb(dt_array, i_array, q_nominal):
    """库仑积分法估计SOC"""
    soc_est = np.ones(len(dt_array))
    for k in range(1, len(dt_array)):
        dQ = i_array[k] * (dt_array[k] - dt_array[k-1]) / 3600
        soc_est[k] = soc_est[k-1] - dQ / q_nominal
    return soc_est

def soc_estimation_ocv(v_array):
    """OCV查表法估计SOC"""
    # 反向查表
    soc_est = np.interp(v_array, OCV_LUT, SOC_LUT)
    # 限制在合理范围
    return np.clip(soc_est, 0, 1)

def soc_estimation_ekf(dt_array, v_array, i_array, q_nominal):
    """
    简化扩展卡尔曼滤波 (EKF) 估计SOC
    状态: x = SOC
    观测: z = OCV(SOC) - I*R
    """
    n = len(dt_array)
    soc_ekf = np.zeros(n)
    soc_ekf[0] = 1.0
    
    # EKF参数
    P = 0.01       # 初始误差协方差
    Q_noise = 1e-6  # 过程噪声
    R_noise = 0.01   # 观测噪声
    
    for k in range(1, n):
        dt = dt_array[k] - dt_array[k-1]
        I = i_array[k]
        
        # 预测
        soc_pred = soc_ekf[k-1] - I * dt / (3600 * q_nominal)
        P_pred = P + Q_noise
        
        # 观测
        R_int = internal_resistance(soc_pred)
        v_pred = ocv_from_soc(soc_pred) - I * R_int
        v_meas = v_array[k]
        
        # OCV-SOC曲线的斜率 (雅可比)
        dOCV_dsoc = np.gradient(OCV_LUT, SOC_LUT)
        H = np.interp(soc_pred, SOC_LUT, dOCV_dsoc)
        
        # 卡尔曼增益
        S = H * P_pred * H + R_noise
        K = P_pred * H / S
        
        # 更新
        soc_ekf[k] = soc_pred + K * (v_meas - v_pred)
        P = (1 - K * H) * P_pred
        
        # 限幅
        soc_ekf[k] = np.clip(soc_ekf[k], 0, 1)
    
    return soc_ekf

# 用1C放电数据做SOC估计对比
c_test = 1.0
soc_true = results[c_test]['soc']
v_data = results[c_test]['v']
i_data = results[c_test]['i']
t_data = results[c_test]['t']

# 给电压加噪声模拟实际测量
np.random.seed(42)
v_noisy = v_data + np.random.normal(0, 0.02, len(v_data))

soc_coulomb = soc_estimation_coulomb(t_data, i_data, Q_NOMINAL)
soc_ocv = soc_estimation_ocv(v_noisy)
soc_ekf = soc_estimation_ekf(t_data, v_noisy, i_data, Q_NOMINAL)

# 计算误差
err_coulomb = np.abs(soc_coulomb - soc_true) * 100
err_ocv = np.abs(soc_ocv - soc_true) * 100
err_ekf = np.abs(soc_ekf - soc_true) * 100

print(f"\nSOC估计误差 (1C放电):")
print(f"  库仑积分: 平均={np.mean(err_coulomb):.2f}%, 最大={np.max(err_coulomb):.2f}%")
print(f"  OCV查表:  平均={np.mean(err_ocv):.2f}%, 最大={np.max(err_ocv):.2f}%")
print(f"  EKF:      平均={np.mean(err_ekf):.2f}%, 最大={np.max(err_ekf):.2f}%")

# ═══════════════════════════════════════════════════════════════
# 3. 绘图
# ═══════════════════════════════════════════════════════════════

fig = plt.figure(figsize=(20, 18))
fig.suptitle('电池放电仿真 — 不同负载下的放电曲线 + SOC估计',
             fontsize=16, fontweight='bold')

gs = GridSpec(4, 3, hspace=0.45, wspace=0.35)

colors_c = plt.cm.coolwarm(np.linspace(0.1, 0.9, len(C_RATES)))

# --- 子图1: 放电曲线 (V vs SOC) ---
ax1 = fig.add_subplot(gs[0, 0])
for c_rate, color in zip(C_RATES, colors_c):
    d = results[c_rate]
    ax1.plot(d['soc']*100, d['v'], color=color, linewidth=1.5,
             label=f'{c_rate}C')
ax1.set_xlabel('SOC (%)')
ax1.set_ylabel('端电压 (V)')
ax1.set_title('不同倍率放电曲线')
ax1.legend()
ax1.grid(True, alpha=0.3)
ax1.set_xlim([0, 100])
ax1.axhline(y=V_EMPTY, color='red', linestyle='--', alpha=0.5, label='截止电压')

# --- 子图2: 放电曲线 (V vs 时间) ---
ax2 = fig.add_subplot(gs[0, 1])
for c_rate, color in zip(C_RATES, colors_c):
    d = results[c_rate]
    ax2.plot(d['t']/3600, d['v'], color=color, linewidth=1.5,
             label=f'{c_rate}C')
ax2.set_xlabel('时间 (h)')
ax2.set_ylabel('端电压 (V)')
ax2.set_title('放电电压 vs 时间')
ax2.legend()
ax2.grid(True, alpha=0.3)
ax2.axhline(y=V_EMPTY, color='red', linestyle='--', alpha=0.5)

# --- 子图3: 容量衰减 ---
ax3 = fig.add_subplot(gs[0, 2])
capacities = [results[c]['capacity_ah'] for c in C_RATES]
durations = [results[c]['duration_h'] for c in C_RATES]
bars = ax3.bar([str(c)+'C' for c in C_RATES], capacities,
               color=colors_c, alpha=0.8, edgecolor='black')
ax3.axhline(y=Q_NOMINAL, color='green', linestyle='--', alpha=0.5,
            label=f'标称容量 {Q_NOMINAL}Ah')
ax3.set_xlabel('放电倍率')
ax3.set_ylabel('实际容量 (Ah)')
ax3.set_title('容量衰减 vs 放电倍率')
ax3.legend()
ax3.grid(True, alpha=0.3, axis='y')
for bar, cap in zip(bars, capacities):
    ax3.text(bar.get_x() + bar.get_width()/2, bar.get_height(),
             f'{cap:.2f}', ha='center', va='bottom', fontsize=9)

# --- 子图4: 内阻变化 ---
ax4 = fig.add_subplot(gs[1, 0])
for c_rate, color in zip(C_RATES, colors_c):
    d = results[c_rate]
    ax4.plot(d['soc']*100, d['r']*1000, color=color, linewidth=1.5,
             label=f'{c_rate}C')
ax4.set_xlabel('SOC (%)')
ax4.set_ylabel('内阻 (mΩ)')
ax4.set_title('内阻 vs SOC')
ax4.legend()
ax4.grid(True, alpha=0.3)
ax4.set_xlim([0, 100])

# --- 子图5: 温度变化 ---
ax5 = fig.add_subplot(gs[1, 1])
for c_rate, color in zip(C_RATES, colors_c):
    d = results[c_rate]
    ax5.plot(d['t']/3600, d['temp'], color=color, linewidth=1.5,
             label=f'{c_rate}C')
ax5.set_xlabel('时间 (h)')
ax5.set_ylabel('温度 (°C)')
ax5.set_title('电池温度 vs 时间')
ax5.legend()
ax5.grid(True, alpha=0.3)
ax5.axhline(y=T_AMBIENT, color='gray', linestyle=':', alpha=0.5)

# --- 子图6: OCV-SOC曲线 ---
ax6 = fig.add_subplot(gs[1, 2])
ax6.plot(SOC_LUT*100, OCV_LUT, 'bo-', linewidth=2, markersize=8, label='OCV-SOC')
ax6.fill_between(SOC_LUT*100, OCV_LUT, V_EMPTY, alpha=0.1, color='blue')
ax6.set_xlabel('SOC (%)')
ax6.set_ylabel('开路电压 OCV (V)')
ax6.set_title('OCV-SOC 查找表')
ax6.legend()
ax6.grid(True, alpha=0.3)
ax6.set_xlim([0, 100])

# --- 子图7: SOC估计对比 ---
ax7 = fig.add_subplot(gs[2, 0])
ax7.plot(t_data/3600, soc_true*100, 'k-', linewidth=2, label='真实SOC')
ax7.plot(t_data/3600, soc_coulomb*100, 'r--', linewidth=1.2, label='库仑积分')
ax7.plot(t_data/3600, soc_ocv*100, 'b:', linewidth=1.2, label='OCV查表')
ax7.plot(t_data/3600, soc_ekf*100, 'g-.', linewidth=1.5, label='EKF')
ax7.set_xlabel('时间 (h)')
ax7.set_ylabel('SOC (%)')
ax7.set_title('SOC估计方法对比 (1C放电)')
ax7.legend()
ax7.grid(True, alpha=0.3)

# --- 子图8: SOC估计误差 ---
ax8 = fig.add_subplot(gs[2, 1])
ax8.plot(t_data/3600, err_coulomb, 'r-', linewidth=1.2, label='库仑积分')
ax8.plot(t_data/3600, err_ocv, 'b-', linewidth=1.2, label='OCV查表')
ax8.plot(t_data/3600, err_ekf, 'g-', linewidth=1.5, label='EKF')
ax8.set_xlabel('时间 (h)')
ax8.set_ylabel('SOC估计误差 (%)')
ax8.set_title('SOC估计误差对比')
ax8.legend()
ax8.grid(True, alpha=0.3)

# --- 子图9: 库仑积分累积误差分析 ---
ax9 = fig.add_subplot(gs[2, 2])
# 模拟不同初始SOC误差对库仑积分的影响
init_errors = [0, 2, 5, 10]  # 初始误差(%)
for err_init in init_errors:
    soc_cb = soc_estimation_coulomb(t_data, i_data, Q_NOMINAL)
    soc_cb = soc_cb + err_init / 100  # 加初始误差
    err = (soc_cb - soc_true) * 100
    ax9.plot(t_data/3600, err, linewidth=1.5, label=f'初始误差{err_init}%')
ax9.set_xlabel('时间 (h)')
ax9.set_ylabel('SOC误差 (%)')
ax9.set_title('库仑积分累积误差')
ax9.legend()
ax9.grid(True, alpha=0.3)

# --- 子图10: 功率输出能力 ---
ax10 = fig.add_subplot(gs[3, 0])
for c_rate, color in zip(C_RATES, colors_c):
    d = results[c_rate]
    power = d['v'] * d['i']
    ax10.plot(d['soc']*100, power, color=color, linewidth=1.5,
              label=f'{c_rate}C')
ax10.set_xlabel('SOC (%)')
ax10.set_ylabel('输出功率 (W)')
ax10.set_title('输出功率 vs SOC')
ax10.legend()
ax10.grid(True, alpha=0.3)

# --- 子图11: 能量密度 ---
ax11 = fig.add_subplot(gs[3, 1])
energies = []
for c_rate in C_RATES:
    d = results[c_rate]
    # 能量 = 积分(V*I*dt)
    energy_wh = np.trapezoid(d['v'] * d['i'], d['t']) / 3600
    energies.append(energy_wh)

bars = ax11.bar([str(c)+'C' for c in C_RATES], energies,
                color=colors_c, alpha=0.8, edgecolor='black')
ax11.set_xlabel('放电倍率')
ax11.set_ylabel('放电能量 (Wh)')
ax11.set_title('放电能量 vs 倍率')
ax11.grid(True, alpha=0.3, axis='y')
for bar, e in zip(bars, energies):
    ax11.text(bar.get_x() + bar.get_width()/2, bar.get_height(),
              f'{e:.2f}', ha='center', va='bottom', fontsize=9)

# --- 子图12: 说明文字 ---
ax12 = fig.add_subplot(gs[3, 2])
ax12.axis('off')
info = (
    "电池放电模型说明\n"
    "═══════════════════\n\n"
    f"电池: 18650锂电 {Q_NOMINAL}Ah\n"
    f"满充: {V_FULL}V → 放空: {V_EMPTY}V\n\n"
    "【放电特性】\n"
    "• 高倍率放电容量下降\n"
    "  (Peukert效应)\n"
    "• 内阻随SOC呈U型曲线\n"
    "• 大电流导致温升\n\n"
    "【SOC估计方法】\n"
    "• 库仑积分: 累积误差\n"
    "• OCV查表: 需静置\n"
    "• EKF: 自适应融合\n\n"
    "【实际应用建议】\n"
    "• 组合使用: EKF+库仑\n"
    "• 定期OCV校正\n"
    "• 温度补偿必不可少"
)
ax12.text(0.05, 0.95, info, transform=ax12.transAxes, fontsize=10,
          verticalalignment='top', fontfamily='monospace',
          bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))

plt.savefig('battery_discharge_simulation_result.png', dpi=150, bbox_inches='tight')
print("\n图表已保存: battery_discharge_simulation_result.png")
print("仿真完成!")
