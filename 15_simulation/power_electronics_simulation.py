#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
电力电子仿真 - MOSFET开关/死区/EMI/效率模型
============================================
适用于电赛电源类题目，Buck/Boost/半桥拓扑仿真
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import rcParams
rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
rcParams['axes.unicode_minus'] = False

# ============================================================
# 1. MOSFET开关模型
# ============================================================
class MOSFETSwitch:
    def __init__(self, Rds_on=0.01, Ciss=1000e-12, Coss=200e-12,
                 Qg=20e-9, Vth=2.5, gfs=50):
        self.Rds_on = Rds_on
        self.Ciss = Ciss
        self.Coss = Coss
        self.Qg = Qg
        self.Vth = Vth
        self.gfs = gfs

    def switching_loss(self, Vds, Id, fsw, t_rise=20e-9, t_fall=15e-9):
        """开关损耗 P = 0.5*V*I*(tr+tf)*fsw"""
        P_sw_on = 0.5 * Vds * Id * t_rise * fsw
        P_sw_off = 0.5 * Vds * Id * t_fall * fsw
        return P_sw_on, P_sw_off

    def conduction_loss(self, Id_rms):
        """导通损耗 P = Id^2 * Rds_on"""
        return Id_rms**2 * self.Rds_on

    def gate_drive_loss(self, Vgs, fsw):
        """栅极驱动损耗 P = Qg*Vgs*fsw"""
        return self.Qg * Vgs * fsw

    def switching_trajectory(self, Vds_max, Id_max, t_ns=100):
        """开关过程V-I轨迹"""
        t = np.linspace(0, t_ns, 200)
        # 开通过程
        V_on = Vds_max * (1 - t/t_ns)
        I_on = Id_max * (t/t_ns)
        # 关断过程
        V_off = Vds_max * (t/t_ns)
        I_off = Id_max * (1 - t/t_ns)
        return t, V_on, I_on, V_off, I_off


# ============================================================
# 2. Buck变换器仿真
# ============================================================
def buck_converter_sim(Vin=12, Vout=5, L=47e-6, C=220e-6, Rload=2.5,
                       fsw=200e3, R_L=0.05, R_C=0.02, t_end=0.005):
    """Buck变换器时域仿真"""
    dt = 1e-8
    n = int(t_end / dt)
    D = Vout / Vin

    v_out = np.zeros(n)
    i_L = np.zeros(n)
    v_sw = np.zeros(n)

    for k in range(1, n):
        # PWM占空比
        t_ratio = (k % int(1/(fsw*dt))) / int(1/(fsw*dt))
        sw_on = t_ratio < D

        if sw_on:
            v_L = Vin - i_L[k-1]*R_L - v_out[k-1]
        else:
            v_L = -i_L[k-1]*R_L - v_out[k-1] - 0.3  # 二极管压降

        i_L[k] = i_L[k-1] + v_L/L * dt
        i_L[k] = max(i_L[k], 0)  # CCM/DCM边界

        v_out[k] = v_out[k-1] + (i_L[k-1] - v_out[k-1]/Rload) / C * dt
        v_sw[k] = Vin - v_L if not sw_on else 0

    t = np.arange(n) * dt * 1e3  # ms
    return t, v_out, i_L, v_sw


# ============================================================
# 3. 死区时间分析
# ============================================================
def dead_time_analysis(dead_time_ns=100, fsw=200e3, n_cycles=5):
    """死区时间对输出影响分析"""
    t_sw = 1/fsw
    dt = 1e-9
    n = int(n_cycles * t_sw / dt)
    t = np.arange(n) * dt * 1e9  # ns

    gate_hi = np.zeros(n)
    gate_lo = np.zeros(n)

    period = int(t_sw / dt)
    duty_ticks = int(0.5 * period)

    for k in range(n):
        pos = k % period
        if pos < duty_ticks:
            gate_hi[k] = 1
        if pos >= int(dead_time_ns*1e-9/dt) and pos < duty_ticks + int(dead_time_ns*1e-9/dt):
            gate_lo[k] = 1

    # 死区期间体二极管导通
    body_diode = (gate_hi == 0) & (gate_lo == 0)
    return t[:2000], gate_hi[:2000], gate_lo[:2000], body_diode[:2000]


# ============================================================
# 4. EMI频谱分析
# ============================================================
def emi_spectrum(Vin, D, fsw, n_harmonics=50):
    """开关波形EMI频谱（方波傅里叶展开）"""
    freqs = np.arange(1, n_harmonics+1) * fsw
    # 方波傅里叶系数
    magnitudes = np.zeros(n_harmonics)
    for k in range(1, n_harmonics+1):
        magnitudes[k-1] = 2*Vin*D*np.abs(np.sin(k*np.pi*D))/(k*np.pi)

    magnitudes_dB = 20*np.log10(magnitudes + 1e-12)
    return freqs, magnitudes_dB


# ============================================================
# 5. 效率计算
# ============================================================
def efficiency_calc(Vin, Vout, Iout, fsw, mosfet: MOSFETSwitch):
    """计算变换器效率"""
    Pout = Vout * Iout
    Id_rms = Iout  # 简化

    P_cond = mosfet.conduction_loss(Id_rms)
    P_sw_on, P_sw_off = mosfet.switching_loss(Vin, Iout, fsw)
    P_gate = mosfet.gate_drive_loss(10, fsw)
    P_diode = 0.3 * Iout * (1 - Vout/Vin)  # 二极管损耗

    P_loss = P_cond + P_sw_on + P_sw_off + P_gate + P_diode
    eta = Pout / (Pout + P_loss) * 100
    return eta, P_cond, P_sw_on+P_sw_off, P_gate, P_diode


# ============================================================
# 主仿真
# ============================================================
def main():
    fig, axes = plt.subplots(3, 2, figsize=(14, 16))
    fig.suptitle('电力电子仿真综合', fontsize=16, fontweight='bold')

    mosfet = MOSFETSwitch(Rds_on=0.01, Qg=20e-9)

    # --- 1. Buck输出纹波 ---
    ax = axes[0, 0]
    t, v_out, i_L, v_sw = buck_converter_sim()
    ax.plot(t, v_out, linewidth=0.5)
    ax.axhline(5, color='r', linestyle='--', label='目标5V')
    ax.set_xlabel('时间 (ms)')
    ax.set_ylabel('输出电压 (V)')
    ax.set_title('Buck变换器输出电压')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_xlim(4.5, 5.0)

    # --- 2. 电感电流 ---
    ax = axes[0, 1]
    ax.plot(t, i_L, linewidth=0.5, color='orange')
    ax.set_xlabel('时间 (ms)')
    ax.set_ylabel('电感电流 (A)')
    ax.set_title('电感电流波形')
    ax.grid(True, alpha=0.3)
    ax.set_xlim(4.5, 5.0)

    # --- 3. 死区分析 ---
    ax = axes[1, 0]
    t_dt, g_hi, g_lo, bd = dead_time_analysis(dead_time_ns=100)
    t_us = t_dt * 1e-3  # ns -> us
    ax.plot(t_us, g_hi, label='高侧栅极', linewidth=1)
    ax.plot(t_us, g_lo+1.1, label='低侧栅极(偏移)', linewidth=1)
    ax.fill_between(t_us, 0, 2.2, where=bd[:len(t_us)], alpha=0.3, color='red', label='体二极管导通')
    ax.set_xlabel('时间 (μs)')
    ax.set_ylabel('栅极信号')
    ax.set_title('死区时间分析 (100ns)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # --- 4. EMI频谱 ---
    ax = axes[1, 1]
    freqs, mag = emi_spectrum(12, 0.5, 200e3)
    ax.bar(freqs/1e6, mag, width=freqs[0]/1e6*0.8, alpha=0.7)
    ax.set_xlabel('频率 (MHz)')
    ax.set_ylabel('幅度 (dBV)')
    ax.set_title('开关波形EMI频谱 (200kHz, D=0.5)')
    ax.grid(True, alpha=0.3)

    # --- 5. 效率 vs 负载电流 ---
    ax = axes[2, 0]
    Iout_range = np.linspace(0.1, 5, 50)
    etas = []
    for I in Iout_range:
        eta, *_ = efficiency_calc(12, 5, I, 200e3, mosfet)
        etas.append(eta)
    ax.plot(Iout_range, etas, 'b-', linewidth=2)
    ax.set_xlabel('负载电流 (A)')
    ax.set_ylabel('效率 (%)')
    ax.set_title('Buck变换器效率 vs 负载')
    ax.grid(True, alpha=0.3)
    ax.set_ylim(70, 100)

    # --- 6. 效率 vs 开关频率 ---
    ax = axes[2, 1]
    fsw_range = np.linspace(50e3, 1e6, 50)
    etas_fsw = []
    for f in fsw_range:
        eta, *_ = efficiency_calc(12, 5, 2, f, mosfet)
        etas_fsw.append(eta)
    ax.plot(fsw_range/1e3, etas_fsw, 'r-', linewidth=2)
    ax.set_xlabel('开关频率 (kHz)')
    ax.set_ylabel('效率 (%)')
    ax.set_title('效率 vs 开关频率 (Iout=2A)')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out = r'./nuedc-asset-library\15_simulation\power_electronics_simulation_result.png'
    plt.savefig(out, dpi=150, bbox_inches='tight')
    print(f'已保存: {out}')

    # 统计
    eta, Pc, Psw, Pg, Pd = efficiency_calc(12, 5, 2, 200e3, mosfet)
    print(f'\n=== 电力电子仿真统计 ===')
    print(f'Buck效率@2A: {eta:.2f}%')
    print(f'导通损耗: {Pc*1000:.2f}mW, 开关损耗: {Psw*1000:.2f}mW')
    print(f'驱动损耗: {Pg*1000:.2f}mW, 二极管损耗: {Pd*1000:.2f}mW')


if __name__ == '__main__':
    main()
