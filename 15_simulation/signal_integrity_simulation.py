#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
信号完整性仿真 - 传输线/串扰/阻抗匹配/眼图
============================================
适用于电赛高速信号题目，PCB信号完整性分析
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import rcParams
rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
rcParams['axes.unicode_minus'] = False

# ============================================================
# 1. 传输线模型 (RLGC)
# ============================================================
class TransmissionLine:
    def __init__(self, R=0.1, L=250e-9, G=1e-9, C=100e-12, length=0.1):
        self.R = R      # Ω/m
        self.L = L      # H/m
        self.G = G      # S/m
        self.C = C      # F/m
        self.length = length  # m

    def characteristic_impedance(self, freq):
        """Z0 = sqrt((R+jωL)/(G+jωC))"""
        w = 2*np.pi*freq
        Z0 = np.sqrt((self.R + 1j*w*self.L) / (self.G + 1j*w*self.C))
        return Z0

    def propagation_constant(self, freq):
        """γ = sqrt((R+jωL)(G+jωC))"""
        w = 2*np.pi*freq
        gamma = np.sqrt((self.R + 1j*w*self.L) * (self.G + 1j*w*self.C))
        return gamma

    def delay(self):
        """传播延迟 Td = length * sqrt(LC)"""
        return self.length * np.sqrt(self.L * self.C)

    def attenuation(self, freq):
        """衰减 dB/m"""
        gamma = self.propagation_constant(freq)
        return np.real(gamma) * 8.686  # Np -> dB


# ============================================================
# 2. 反射分析 (Bounce Diagram)
# ============================================================
def reflection_analysis(tline: TransmissionLine, Zs, Zl, Vsrc=3.3,
                        rise_time=1e-9, t_end=None):
    """传输线反射时域仿真"""
    Td = tline.delay()
    if t_end is None:
        t_end = 10 * Td
    dt = Td / 50
    t = np.arange(0, t_end, dt)
    n = len(t)

    Z0 = np.real(tline.characteristic_impedance(1e6))
    Gamma_s = (Zs - Z0) / (Zs + Z0)
    Gamma_l = (Zl - Z0) / (Zl + Z0)

    # 源端电压 (含上升沿)
    Vs = np.minimum(Vsrc * t / rise_time, Vsrc)

    # 入射波
    V_incident = Vs * Z0 / (Zs + Z0)

    # 负载端电压 (多次反射叠加)
    V_load = np.zeros(n)
    V_source = np.zeros(n)

    n_reflections = 10
    for k in range(n_reflections):
        delay_out = (2*k + 1) * Td
        delay_back = (2*k + 2) * Td

        coeff = Gamma_l * Gamma_s**k if k > 0 else 1
        for i in range(n):
            t_now = t[i]
            if t_now >= delay_out:
                idx = int((t_now - delay_out) / dt)
                if idx < n:
                    V_load[i] += V_incident[min(idx, n-1)] * coeff * Gamma_l
            if t_now >= delay_back:
                idx = int((t_now - delay_back) / dt)
                if idx < n:
                    V_source[i] += V_incident[min(idx, n-1)] * coeff * Gamma_l * Gamma_s

    V_load_total = V_load + V_incident * (t >= Td)
    V_source_total = Vs + V_source

    return t*1e9, V_source_total, V_load_total, Z0, Gamma_s, Gamma_l


# ============================================================
# 3. 串扰 (NEXT/FEXT)
# ============================================================
def crosstalk_coupled_lines(length=0.1, n_segments=200, Vpulse=3.3,
                             rise_time=0.2e-9, Z0=50, coupling_coeff=0.05):
    """耦合传输线串扰仿真"""
    dt = rise_time / 50
    t_end = 4e-9
    t = np.arange(0, t_end, dt)
    n = len(t)

    Td_seg = length / n_segments * np.sqrt(1/(3e8)**2) * 1
    # 简化的NEXT和FEXT模型
    # NEXT ≈ Kb * (tr/2) * δ(t)  近端串扰
    # FEXT ≈ Kb * l/(2*td) * dVi/dt(t - td)  远端串扰

    td = length / 3e8 * np.sqrt(3.5)  # FR4

    # 入射信号
    V_in = np.minimum(Vpulse * t / rise_time, Vpulse)

    # NEXT (近端，与dV/dt成正比)
    dVdt = np.gradient(V_in, dt)
    Kb = coupling_coeff
    V_next = Kb * rise_time / 2 * dVdt  # 简化

    # FEXT (远端，延迟td后出现)
    delay_pts = int(td / dt)
    V_fext = np.zeros(n)
    if delay_pts < n:
        V_fext[delay_pts:] = Kb * length / (2*td) * dVdt[:n-delay_pts]

    return t*1e9, V_in, V_next, V_fext


# ============================================================
# 4. 阻抗匹配网络
# ============================================================
def impedance_matching(Zs_real, Zs_imag, Zl_real, Zl_imag, freq):
    """L型阻抗匹配网络设计"""
    Zs = complex(Zs_real, Zs_imag)
    Zl = complex(Zl_real, Zl_imag)

    # 两种L型拓扑选择
    Q = np.sqrt(abs(Zl/Zs) - 1) if abs(Zl) > abs(Zs) else np.sqrt(abs(Zs/Zl) - 1)

    if abs(Zl) > abs(Zs):
        # 并联-串联
        Xp = abs(Zl) / Q
        Xs = Q * abs(Zs)
        L_match = Xs / (2*np.pi*freq)
        C_match = 1 / (2*np.pi*freq*Xp)
    else:
        # 串联-并联
        Xs = Q * abs(Zl)
        Xp = abs(Zs) / Q
        L_match = Xs / (2*np.pi*freq)
        C_match = 1 / (2*np.pi*freq*Xp)

    return Q, L_match, C_match


# ============================================================
# 5. 眼图生成
# ============================================================
def generate_eye_diagram(data_rate=1e9, rise_time=0.1e-9, n_bits=1000,
                          noise_std=0.05, jitter_std=0.02):
    """生成眼图数据"""
    samples_per_bit = 64
    n_samples = n_bits * samples_per_bit
    dt = 1.0 / (data_rate * samples_per_bit)
    t = np.arange(n_samples) * dt

    # 随机NRZ数据
    bits = np.random.randint(0, 2, n_bits)
    signal = np.repeat(bits, samples_per_bit).astype(float) * 3.3

    # 上升/下降沿
    rise_pts = int(rise_time / dt)
    for i in range(n_bits):
        idx = i * samples_per_bit
        if i > 0 and bits[i] != bits[i-1]:
            for j in range(min(rise_pts, samples_per_bit)):
                frac = j / rise_pts
                if bits[i] > bits[i-1]:
                    signal[idx+j] = 3.3 * frac
                else:
                    signal[idx+j] = 3.3 * (1-frac)

    # 添加噪声和抖动
    signal += np.random.randn(n_samples) * noise_std
    jitter = np.random.randn(n_bits) * jitter_std / data_rate
    jitter_samples = np.repeat(jitter, samples_per_bit)
    t_jittered = t + jitter_samples

    # 折叠到UI
    ui = 1.0 / data_rate
    t_folded = t % ui
    t_norm = t_folded / ui  # 0~1

    return t_norm, signal


# ============================================================
# 主仿真
# ============================================================
def main():
    fig, axes = plt.subplots(3, 2, figsize=(14, 16))
    fig.suptitle('信号完整性仿真综合', fontsize=16, fontweight='bold')

    tline = TransmissionLine(R=0.1, L=250e-9, C=100e-12, length=0.1)

    # --- 1. 特性阻抗 vs 频率 ---
    ax = axes[0, 0]
    freq = np.logspace(3, 10, 200)
    Z0 = tline.characteristic_impedance(freq)
    ax.semilogx(freq/1e6, np.abs(Z0), 'b-', linewidth=2)
    ax.set_xlabel('频率 (MHz)')
    ax.set_ylabel('|Z0| (Ω)')
    ax.set_title('特性阻抗 vs 频率')
    ax.grid(True, alpha=0.3)
    ax.axhline(50, color='r', linestyle='--', alpha=0.5, label='50Ω')
    ax.legend()

    # --- 2. 传输线反射 ---
    ax = axes[0, 1]
    t_ns, Vs, Vl, Z0, Gs, Gl = reflection_analysis(tline, Zs=10, Zl=100)
    ax.plot(t_ns, Vs, label='源端', linewidth=1)
    ax.plot(t_ns, Vl, label='负载端', linewidth=1)
    ax.set_xlabel('时间 (ns)')
    ax.set_ylabel('电压 (V)')
    ax.set_title(f'传输线反射 (Zs=10Ω, Zl=100Ω, Z0={Z0:.0f}Ω)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # --- 3. 阶跃响应对比 (不同终端) ---
    ax = axes[1, 0]
    for Zl_val, color in [(25, 'r'), (50, 'g'), (100, 'b'), (200, 'm')]:
        t_ns, _, Vl, *_ = reflection_analysis(tline, Zs=50, Zl=Zl_val)
        ax.plot(t_ns, Vl, color=color, label=f'Zl={Zl_val}Ω')
    ax.set_xlabel('时间 (ns)')
    ax.set_ylabel('负载电压 (V)')
    ax.set_title('不同终端阻抗阶跃响应')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # --- 4. 串扰 ---
    ax = axes[1, 1]
    t_cx, V_in, V_next, V_fext = crosstalk_coupled_lines()
    ax.plot(t_cx, V_in, label='入射信号', linewidth=1.5)
    ax.plot(t_cx, V_next*100, label='NEXT (×100)', linewidth=1)
    ax.plot(t_cx, V_fext*100, label='FEXT (×100)', linewidth=1)
    ax.set_xlabel('时间 (ns)')
    ax.set_ylabel('电压 (V)')
    ax.set_title('耦合传输线串扰')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # --- 5. 衰减 vs 频率 ---
    ax = axes[2, 0]
    freq_att = np.logspace(6, 10, 200)
    att = tline.attenuation(freq_att) * tline.length
    ax.semilogx(freq_att/1e6, att, 'r-', linewidth=2)
    ax.set_xlabel('频率 (MHz)')
    ax.set_ylabel('总衰减 (dB)')
    ax.set_title(f'传输线衰减 (长度={tline.length*100:.0f}cm, FR4)')
    ax.grid(True, alpha=0.3)

    # --- 6. 眼图 ---
    ax = axes[2, 1]
    t_norm, signal = generate_eye_diagram(data_rate=1e9, noise_std=0.08)
    ax.scatter(t_norm[::4], signal[::4], s=0.5, alpha=0.1, c='blue')
    ax.set_xlabel('归一化时间 (UI)')
    ax.set_ylabel('电压 (V)')
    ax.set_title('眼图 (1Gbps NRZ)')
    ax.set_xlim(0, 1)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out = r'./nuedc-asset-library\15_simulation\signal_integrity_simulation_result.png'
    plt.savefig(out, dpi=150, bbox_inches='tight')
    print(f'已保存: {out}')

    # 统计
    print(f'\n=== 信号完整性仿真统计 ===')
    Z0_dc = tline.characteristic_impedance(1e3)
    print(f'Z0@DC: {abs(Z0_dc):.1f}Ω')
    print(f'传播延迟: {tline.delay()*1e9:.2f} ns')
    print(f'衰减@1GHz: {tline.attenuation(1e9)*tline.length:.2f} dB')


if __name__ == '__main__':
    main()
