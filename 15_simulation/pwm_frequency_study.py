#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PWM频率研究仿真 — 不同频率对电机噪音和平滑度的影响
==================================================
分析PWM频率对以下方面的影响:
  - 电流纹波
  - 转矩脉动
  - 声学噪声 (可听频段)
  - 电机平滑度
  - MOS管开关损耗
依赖: numpy, matplotlib
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

# ═══════════════════════════════════════════════════════════════
# 系统参数
# ═══════════════════════════════════════════════════════════════

V_BUS = 12.0       # 母线电压 (V)
R_MOTOR = 2.0      # 电机电阻 (Ω)
L_MOTOR = 0.005    # 电机电感 (H)
BACK_EMF = 6.0     # 反电动势 (V)
DUTY = 0.5         # 占空比

# PWM频率列表
FREQ_LIST = [1000, 5000, 10000, 20000, 50000, 100000]  # Hz

# 仿真参数
SIM_TIME = 0.02     # 仿真20ms
DT_FACTOR = 1000    # 每个PWM周期采1000个点

# ═══════════════════════════════════════════════════════════════
# 1. 电流纹波理论分析
# ═══════════════════════════════════════════════════════════════

def theoretical_current_ripple(V, R, L, back_emf, duty, freq):
    """
    理论电流纹波计算 (稳态)
    ΔI = (V - back_emf) * D / (L * f)  (导通期间)
    简化公式: ΔI = V * D * (1-D) / (L * f)
    """
    delta_I = (V - back_emf) * duty * (1.0 / freq) / L
    # 更精确的纹波公式
    delta_I = V * duty * (1 - duty) / (L * freq)
    return delta_I

def sim_motor_current(freq, sim_time, V, R, L, back_emf, duty):
    """
    仿真电机电流波形
    使用简单欧拉法求解 RL 电路:
    V_pwm = duty * V (平均, 忽略开关细节)
    L * di/dt + R * i + back_emf = V_pwm(t)
    """
    dt = 1.0 / (freq * DT_FACTOR)
    n_steps = int(sim_time / dt)
    
    t = np.zeros(n_steps)
    i = np.zeros(n_steps)
    v_pwm = np.zeros(n_steps)
    
    for k in range(1, n_steps):
        t[k] = k * dt
        # PWM信号
        phase = (t[k] * freq) % 1.0
        v_pwm[k] = V if phase < duty else 0.0
        
        # RL电路微分方程
        di = (v_pwm[k] - R * i[k-1] - back_emf) / L * dt
        i[k] = i[k-1] + di
    
    return t, i, v_pwm

# ═══════════════════════════════════════════════════════════════
# 2. 运行仿真
# ═══════════════════════════════════════════════════════════════

print("=" * 60)
print("PWM频率研究仿真")
print("=" * 60)
print(f"母线电压: {V_BUS}V, 电阻: {R_MOTOR}Ω, 电感: {L_MOTOR}H")
print(f"反电动势: {BACK_EMF}V, 占空比: {DUTY}")
print(f"PWM频率: {FREQ_LIST}")

# 计算理论纹波
print("\n理论电流纹波:")
print(f"{'频率(Hz)':<10} {'纹波(mA)':<12} {'纹波比(%)':<12}")
print("-" * 35)

ripple_theory = []
for f in FREQ_LIST:
    ripple = theoretical_current_ripple(V_BUS, R_MOTOR, L_MOTOR, BACK_EMF, DUTY, f)
    ripple_theory.append(ripple)
    avg_current = (V_BUS * DUTY - BACK_EMF) / R_MOTOR
    ripple_pct = ripple / max(abs(avg_current), 0.001) * 100
    print(f"{f:<10} {ripple*1000:<12.2f} {ripple_pct:<12.1f}")

# 运行瞬态仿真
sim_results = {}
for freq in FREQ_LIST:
    # 选择性仿真 (高频需要更多计算)
    sim_t = min(SIM_TIME, 10.0 / freq)  # 至少10个周期
    t, i, v = sim_motor_current(freq, sim_t, V_BUS, R_MOTOR, L_MOTOR, BACK_EMF, DUTY)
    sim_results[freq] = (t, i, v)
    print(f"  仿真 {freq}Hz: {len(t)} 点, {sim_t*1000:.1f}ms")

# ═══════════════════════════════════════════════════════════════
# 3. FFT分析 (声学噪声)
# ═══════════════════════════════════════════════════════════════

def compute_fft(signal, dt):
    """计算信号FFT"""
    N = len(signal)
    freqs = np.fft.rfftfreq(N, d=dt)
    fft_mag = np.abs(np.fft.rfft(signal)) / N * 2
    return freqs, fft_mag

# 对不同频率的电流做FFT
fft_results = {}
for freq in FREQ_LIST:
    t, i_signal, _ = sim_results[freq]
    dt = t[1] - t[0]
    # 去除直流分量
    i_ac = i_signal - np.mean(i_signal)
    freqs, mag = compute_fft(i_ac, dt)
    fft_results[freq] = (freqs, mag)

# ═══════════════════════════════════════════════════════════════
# 4. 开关损耗分析
# ═══════════════════════════════════════════════════════════════

# MOSFET参数 (典型值)
T_RISE = 50e-9     # 上升时间 (s)
T_FALL = 30e-9     # 下降时间 (s)
Q_G = 20e-9        # 栅极电荷 (C)
V_GS = 10.0        # 栅极驱动电压 (V)
R_DS_ON = 0.05     # 导通电阻 (Ω)

switching_losses = []
conduction_losses = []
total_losses = []

for f in FREQ_LIST:
    # 开关损耗: P_sw = 0.5 * V * I * (t_r + t_f) * f
    avg_I = (V_BUS * DUTY - BACK_EMF) / R_MOTOR
    P_sw = 0.5 * V_BUS * abs(avg_I) * (T_RISE + T_FALL) * f
    
    # 驱动损耗: P_gate = Q_G * V_GS * f
    P_gate = Q_G * V_GS * f
    
    # 导通损耗: P_cond = I² * R_DS_ON * D
    P_cond = avg_I**2 * R_DS_ON * DUTY
    
    switching_losses.append(P_sw + P_gate)
    conduction_losses.append(P_cond)
    total_losses.append(P_sw + P_gate + P_cond)

print("\n功率损耗分析:")
print(f"{'频率(kHz)':<10} {'开关损耗(mW)':<14} {'导通损耗(mW)':<14} {'总损耗(mW)':<12}")
print("-" * 52)
for i, f in enumerate(FREQ_LIST):
    print(f"{f/1000:<10.0f} {switching_losses[i]*1000:<14.2f} "
          f"{conduction_losses[i]*1000:<14.2f} {total_losses[i]*1000:<12.2f}")

# ═══════════════════════════════════════════════════════════════
# 5. 声学噪声分析
# ═══════════════════════════════════════════════════════════════

# 人耳可听范围: 20Hz ~ 20kHz
# 人耳敏感频率: 1kHz ~ 5kHz
def audible_noise_score(freq):
    """
    可听噪声评分 (越低越好)
    低于20kHz的PWM频率会产生可听噪声
    """
    if freq < 20:
        return 10  # 极低频, 但有振动
    elif freq < 200:
        return 9   # 低频嗡嗡声
    elif freq < 1000:
        return 7   # 可听, 不舒服
    elif freq < 5000:
        return 5   # 人耳敏感区
    elif freq < 10000:
        return 3   # 尖锐但可接受
    elif freq < 20000:
        return 1   # 基本不可听
    else:
        return 0   # 超声波, 完全不可听

noise_scores = [audible_noise_score(f) for f in FREQ_LIST]

# ═══════════════════════════════════════════════════════════════
# 6. 绘图
# ═══════════════════════════════════════════════════════════════

fig = plt.figure(figsize=(20, 18))
fig.suptitle('PWM频率研究仿真 — 不同频率对电机噪音和平滑度的影响',
             fontsize=16, fontweight='bold')

gs = GridSpec(4, 3, hspace=0.45, wspace=0.35)

colors_freq = plt.cm.plasma(np.linspace(0.1, 0.9, len(FREQ_LIST)))

# --- 子图1: 电流波形 (低频) ---
ax1 = fig.add_subplot(gs[0, 0])
for freq, color in zip([1000, 5000], colors_freq[:2]):
    t, i_sig, _ = sim_results[freq]
    # 只显示前几个周期
    n_show = min(len(t), int(5.0 / freq / (t[1]-t[0])))
    ax1.plot(t[:n_show]*1000, i_sig[:n_show]*1000, color=color,
             linewidth=1, label=f'{freq}Hz')
ax1.set_xlabel('时间 (ms)')
ax1.set_ylabel('电流 (mA)')
ax1.set_title('电流波形 (1kHz, 5kHz)')
ax1.legend()
ax1.grid(True, alpha=0.3)

# --- 子图2: 电流波形 (高频) ---
ax2 = fig.add_subplot(gs[0, 1])
for freq, color in zip([20000, 50000], colors_freq[3:5]):
    t, i_sig, _ = sim_results[freq]
    n_show = min(len(t), int(5.0 / freq / (t[1]-t[0])))
    ax2.plot(t[:n_show]*1000, i_sig[:n_show]*1000, color=color,
             linewidth=1, label=f'{freq}Hz')
ax2.set_xlabel('时间 (ms)')
ax2.set_ylabel('电流 (mA)')
ax2.set_title('电流波形 (20kHz, 50kHz)')
ax2.legend()
ax2.grid(True, alpha=0.3)

# --- 子图3: 纹波对比 (理论值) ---
ax3 = fig.add_subplot(gs[0, 2])
freq_labels = [f'{f/1000:.0f}k' for f in FREQ_LIST]
bars = ax3.bar(freq_labels, [r*1000 for r in ripple_theory],
               color=colors_freq, alpha=0.8, edgecolor='black')
ax3.set_xlabel('PWM频率')
ax3.set_ylabel('电流纹波 (mA)')
ax3.set_title('理论电流纹波 vs PWM频率')
ax3.grid(True, alpha=0.3, axis='y')
for bar, val in zip(bars, ripple_theory):
    ax3.text(bar.get_x() + bar.get_width()/2, bar.get_height(),
             f'{val*1000:.1f}', ha='center', va='bottom', fontsize=8)

# --- 子图4: 电流FFT (1kHz PWM) ---
ax4 = fig.add_subplot(gs[1, 0])
for freq in [1000, 5000, 20000]:
    freqs, mag = fft_results[freq]
    mask = freqs < freq * 5  # 显示到5次谐波
    ax4.semilogy(freqs[mask]/1000, mag[mask]*1000,
                 linewidth=1, label=f'{freq}Hz')
ax4.set_xlabel('频率 (kHz)')
ax4.set_ylabel('电流幅值 (mA)')
ax4.set_title('电流频谱分析')
ax4.legend()
ax4.grid(True, alpha=0.3)

# --- 子图5: 功率损耗分解 ---
ax5 = fig.add_subplot(gs[1, 1])
x = np.arange(len(FREQ_LIST))
width = 0.35
ax5.bar(x - width/2, [l*1000 for l in switching_losses], width,
        label='开关损耗', color='#e74c3c', alpha=0.8)
ax5.bar(x + width/2, [l*1000 for l in conduction_losses], width,
        label='导通损耗', color='#3498db', alpha=0.8)
ax5.plot(x, [l*1000 for l in total_losses], 'ko-', linewidth=2,
         markersize=8, label='总损耗')
ax5.set_xticks(x)
ax5.set_xticklabels(freq_labels)
ax5.set_xlabel('PWM频率')
ax5.set_ylabel('功率损耗 (mW)')
ax5.set_title('MOS管功率损耗 vs PWM频率')
ax5.legend()
ax5.grid(True, alpha=0.3, axis='y')

# --- 子图6: 声学噪声评分 ---
ax6 = fig.add_subplot(gs[1, 2])
colors_noise = ['#e74c3c' if s > 5 else '#f39c12' if s > 2 else '#2ecc71'
                for s in noise_scores]
bars = ax6.bar(freq_labels, noise_scores, color=colors_noise, alpha=0.8,
               edgecolor='black')
ax6.set_xlabel('PWM频率')
ax6.set_ylabel('噪声评分 (0=最好, 10=最差)')
ax6.set_title('可听噪声评分 (人耳感知)')
ax6.set_ylim([0, 11])
ax6.grid(True, alpha=0.3, axis='y')
# 标注可听范围
ax6.axvline(x='20k', color='green', linestyle='--', alpha=0.5)
ax6.text(5, 9.5, '人耳敏感区', fontsize=9, color='red', ha='center')

# --- 子图7: 电流平滑度分析 (纹波/平均电流比) ---
ax7 = fig.add_subplot(gs[2, 0])
avg_current = (V_BUS * DUTY - BACK_EMF) / R_MOTOR
smoothness = []
for f in FREQ_LIST:
    _, i_sig, _ = sim_results[f]
    # 平滑度 = 1 - (纹波RMS / 平均值)
    ripple_rms = np.sqrt(np.mean((i_sig - np.mean(i_sig))**2))
    smooth = max(0, 1 - ripple_rms / max(abs(np.mean(i_sig)), 0.001)) * 100
    smoothness.append(smooth)

ax7.plot([f/1000 for f in FREQ_LIST], smoothness, 'bo-', linewidth=2, markersize=10)
ax7.fill_between([f/1000 for f in FREQ_LIST], smoothness, alpha=0.2, color='blue')
ax7.set_xlabel('PWM频率 (kHz)')
ax7.set_ylabel('平滑度 (%)')
ax7.set_title('电流平滑度 vs PWM频率')
ax7.set_ylim([0, 105])
ax7.grid(True, alpha=0.3)

# --- 子图8: 综合评价雷达图 ---
ax8 = fig.add_subplot(gs[2, 1], projection='polar')
# 评价维度: 纹波、噪声、损耗、平滑度、响应速度
categories = ['低纹波', '低噪声', '低损耗', '高平滑', '快响应']
N_cat = len(categories)
angles = np.linspace(0, 2*np.pi, N_cat, endpoint=False).tolist()
angles += angles[:1]

# 为每个频率评分 (1-10)
scores_all = {
    1000:  [2, 2, 9, 2, 8],
    5000:  [5, 4, 7, 5, 7],
    10000: [7, 6, 5, 7, 6],
    20000: [8, 8, 3, 9, 5],
    50000: [9, 9, 2, 9, 4],
}

for idx, (freq, scores) in enumerate(scores_all.items()):
    values = scores + scores[:1]
    ax8.plot(angles, values, 'o-', linewidth=1.5, markersize=5,
             color=colors_freq[idx], label=f'{freq/1000:.0f}kHz')
    ax8.fill(angles, values, alpha=0.1, color=colors_freq[idx])

ax8.set_xticks(angles[:-1])
ax8.set_xticklabels(categories, fontsize=8)
ax8.set_ylim([0, 10])
ax8.set_title('综合评价雷达图', pad=20)
ax8.legend(loc='upper right', bbox_to_anchor=(1.3, 1.1), fontsize=7)

# --- 子图9: 选型建议 ---
ax9 = fig.add_subplot(gs[2, 2])
ax9.axis('off')
info = (
    "PWM频率选型建议\n"
    "═══════════════════\n\n"
    "【低频 (1-5kHz)】\n"
    "✓ 开关损耗低\n"
    "✓ 驱动电路简单\n"
    "✗ 可听噪声大\n"
    "✗ 电流纹波大\n"
    "适用: 大功率, 无需静音\n\n"
    "【中频 (10-20kHz)】\n"
    "✓ 平衡各指标\n"
    "✓ 刚超可听范围\n"
    "适用: 多数电赛项目\n\n"
    "【高频 (50-100kHz)】\n"
    "✓ 纹波极小\n"
    "✓ 完全静音\n"
    "✗ 开关损耗大\n"
    "✗ 需高速驱动\n"
    "适用: 精密伺服\n\n"
    "【推荐选择】\n"
    "一般用途: 10-20kHz\n"
    "静音要求: ≥20kHz\n"
    "精密控制: ≥50kHz"
)
ax9.text(0.05, 0.95, info, transform=ax9.transAxes, fontsize=10,
         verticalalignment='top', fontfamily='monospace',
         bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))

# --- 子图10-12: 电流波形细节 (不同频率, 统一时间尺度) ---
for idx, (freq, ax_idx) in enumerate(zip([1000, 10000, 50000],
                                          [gs[3, 0], gs[3, 1], gs[3, 2]])):
    ax = fig.add_subplot(ax_idx)
    t, i_sig, v_sig = sim_results[freq]
    n_period = int(1.0 / freq / (t[1] - t[0]))
    n_show = min(len(t), 3 * n_period)
    
    ax.plot(t[:n_show]*1000, i_sig[:n_show]*1000, 'b-', linewidth=1, label='电流')
    ax2 = ax.twinx()
    ax2.plot(t[:n_show]*1000, v_sig[:n_show], 'r-', linewidth=0.5, alpha=0.3, label='PWM电压')
    ax2.set_ylabel('电压 (V)', color='red')
    ax2.tick_params(axis='y', labelcolor='red')
    
    ax.set_xlabel('时间 (ms)')
    ax.set_ylabel('电流 (mA)', color='blue')
    ax.tick_params(axis='y', labelcolor='blue')
    ax.set_title(f'PWM={freq/1000:.0f}kHz 波形细节')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper left', fontsize=8)

plt.savefig('pwm_frequency_study_result.png', dpi=150, bbox_inches='tight')
print("\n图表已保存: pwm_frequency_study_result.png")
print("仿真完成!")
