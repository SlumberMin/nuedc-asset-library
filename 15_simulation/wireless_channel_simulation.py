#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
无线信道仿真 - 衰落/多径/阴影/干扰模型
=======================================
适用于电赛无线通信题目，支持多种信道模型
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import rcParams
rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
rcParams['axes.unicode_minus'] = False

# ============================================================
# 1. 自由空间路径损耗 (FSPL)
# ============================================================
def free_space_path_loss(distance_m, freq_Hz, c=3e8):
    """FSPL(dB) = 20*log10(4*pi*d*f/c)"""
    return 20*np.log10(4*np.pi*distance_m*freq_Hz/c)


# ============================================================
# 2. 多径衰落信道 - Rayleigh & Rician
# ============================================================
def rayleigh_fading(n_samples, n_paths=6):
    """Rayleigh衰落（无LOS）"""
    I = np.sum(np.random.randn(n_paths, n_samples) * np.cos(
        2*np.pi*np.random.rand(n_paths, n_samples)), axis=0)
    Q = np.sum(np.random.randn(n_paths, n_samples) * np.sin(
        2*np.pi*np.random.rand(n_paths, n_samples)), axis=0)
    envelope = np.sqrt(I**2 + Q**2) / np.sqrt(n_paths)
    return envelope


def rician_fading(n_samples, K_dB=6, n_paths=6):
    """Rician衰落（有LOS），K因子(dB)"""
    K = 10**(K_dB/10)
    rayleigh = rayleigh_fading(n_samples, n_paths)
    los = np.sqrt(K) * np.ones(n_samples)
    envelope = np.sqrt((los + rayleigh * np.cos(np.random.uniform(0, 2*np.pi, n_samples)))**2 +
                       (rayleigh * np.sin(np.random.uniform(0, 2*np.pi, n_samples)))**2)
    return envelope / np.sqrt(K + 1)


# ============================================================
# 3. 阴影衰落 (Log-normal Shadowing)
# ============================================================
def shadow_fading(distance_m, d0=1.0, path_loss_exp=3.0, sigma_dB=8.0, PL_d0=40):
    """对数正态阴影衰落"""
    PL = PL_d0 + 10*path_loss_exp*np.log10(distance_m/d0) + \
         np.random.randn(len(distance_m)) * sigma_dB
    return PL


# ============================================================
# 4. 多径信道冲激响应 (CIR)
# ============================================================
def multipath_cir(n_taps=6, max_delay_ns=500, rms_delay_ns=100):
    """生成多径信道冲激响应"""
    delays_ns = np.sort(np.random.exponential(rms_delay_ns, n_taps))
    delays_ns = np.clip(delays_ns, 0, max_delay_ns)
    gains_dB = -delays_ns / rms_delay_ns * 5 + np.random.randn(n_taps)*3
    gains = 10**(gains_dB/20)
    gains /= np.sqrt(np.sum(gains**2))
    phases = np.random.uniform(0, 2*np.pi, n_taps)
    return delays_ns, gains, phases


# ============================================================
# 5. 同频干扰模型 (C/I)
# ============================================================
def cochannel_interference(n_interferers=6, d0=100, freq_Hz=2.4e9,
                           path_loss_exp=3.5, sigma_dB=6.0):
    """计算C/I比(dB)"""
    d_i = d0 * (1 + np.random.exponential(1, n_interferers))
    interf_power = free_space_path_loss(d_i, freq_Hz) + \
                   np.random.randn(n_interferers)*sigma_dB
    signal_power = free_space_path_loss(d0, freq_Hz)
    Ci_dB = signal_power - 10*np.log10(np.sum(10**(-interf_power/10)))
    return Ci_dB


# ============================================================
# 6. OFDM信道仿真
# ============================================================
def ofdm_channel_sim(n_subcarriers=64, snr_dB=20):
    """OFDM子载波信道频率响应"""
    delays_ns, gains, phases = multipath_cir()
    freq = np.arange(n_subcarriers)
    H = np.zeros(n_subcarriers, dtype=complex)
    for d, g, p in zip(delays_ns, gains, phases):
        H += g * np.exp(1j*(p - 2*np.pi*freq*d/1e3))
    noise = (np.random.randn(n_subcarriers) +
             1j*np.random.randn(n_subcarriers)) / np.sqrt(2*10**(snr_dB/10))
    return freq, H, H + noise


# ============================================================
# 7. 信道容量 (Shannon)
# ============================================================
def shannon_capacity(snr_dB_array, bw_Hz=1e6):
    """Shannon容量 C = B*log2(1+SNR)"""
    snr_lin = 10**(snr_dB_array/10)
    return bw_Hz * np.log2(1 + snr_lin)


# ============================================================
# 主仿真
# ============================================================
def main():
    fig, axes = plt.subplots(3, 2, figsize=(14, 16))
    fig.suptitle('无线信道仿真综合', fontsize=16, fontweight='bold')

    # --- 1. 路径损耗 vs 距离 ---
    ax = axes[0, 0]
    dist = np.linspace(1, 1000, 200)
    for f, label in [(433e6, '433MHz'), (2.4e9, '2.4GHz'), (5.8e9, '5.8GHz')]:
        pl = free_space_path_loss(dist, f)
        ax.plot(dist, pl, label=label)
    ax.set_xlabel('距离 (m)')
    ax.set_ylabel('路径损耗 (dB)')
    ax.set_title('自由空间路径损耗')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_xscale('log')

    # --- 2. Rayleigh vs Rician 衰落 ---
    ax = axes[0, 1]
    n = 1000
    ray = rayleigh_fading(n)
    ric = rician_fading(n, K_dB=6)
    ax.plot(20*np.log10(ric[:200]+1e-10), label='Rician (K=6dB)', alpha=0.8)
    ax.plot(20*np.log10(ray[:200]+1e-10), label='Rayleigh', alpha=0.8)
    ax.set_xlabel('样本')
    ax.set_ylabel('包络 (dB)')
    ax.set_title('多径衰落: Rayleigh vs Rician')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # --- 3. 阴影衰落分布 ---
    ax = axes[1, 0]
    d = np.linspace(10, 500, 200)
    for _ in range(50):
        pl = shadow_fading(d, path_loss_exp=3.0, sigma_dB=8)
        ax.plot(d, pl, alpha=0.05, color='blue')
    pl_mean = 40 + 30*np.log10(d/1.0)
    ax.plot(d, pl_mean, 'r-', linewidth=2, label='均值')
    ax.set_xlabel('距离 (m)')
    ax.set_ylabel('路径损耗 (dB)')
    ax.set_title('对数正态阴影衰落 (σ=8dB)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # --- 4. 多径冲激响应 ---
    ax = axes[1, 1]
    for _ in range(20):
        delays, gains, _ = multipath_cir()
        ax.vlines(delays, 0, gains, alpha=0.3, linewidth=1.5)
    ax.set_xlabel('时延 (ns)')
    ax.set_ylabel('归一化增益')
    ax.set_title('多径信道冲激响应 (20次实现)')
    ax.grid(True, alpha=0.3)

    # --- 5. CDF / CCDF ---
    ax = axes[2, 0]
    ray_samples = rayleigh_fading(10000)
    ric_samples = rician_fading(10000, K_dB=6)
    ray_dB = 20*np.log10(ray_samples+1e-10)
    ric_dB = 20*np.log10(ric_samples+1e-10)
    ax.hist(ray_dB, bins=100, density=True, alpha=0.5, label='Rayleigh')
    ax.hist(ric_dB, bins=100, density=True, alpha=0.5, label='Rician')
    ax.set_xlabel('包络 (dB)')
    ax.set_ylabel('PDF')
    ax.set_title('衰落包络分布')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # --- 6. Shannon容量 vs SNR ---
    ax = axes[2, 1]
    snr = np.linspace(-10, 30, 100)
    for bw, label in [(1e6, '1MHz'), (5e6, '5MHz'), (20e6, '20MHz')]:
        cap = shannon_capacity(snr, bw) / 1e6
        ax.plot(snr, cap, label=label)
    ax.set_xlabel('SNR (dB)')
    ax.set_ylabel('容量 (Mbps)')
    ax.set_title('Shannon信道容量')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out = r'./nuedc-asset-library\15_simulation\wireless_channel_simulation_result.png'
    plt.savefig(out, dpi=150, bbox_inches='tight')
    print(f'已保存: {out}')

    # 打印统计
    print(f'\n=== 无线信道仿真统计 ===')
    print(f'FSPL@100m/2.4GHz: {free_space_path_loss(100, 2.4e9):.1f} dB')
    print(f'Rayleigh 均值包络: {np.mean(ray_samples):.3f}')
    print(f'Rician 均值包络: {np.mean(ric_samples):.3f}')
    print(f'Shannon@20dB/1MHz: {shannon_capacity(np.array([20]), 1e6)[0]/1e6:.2f} Mbps')


if __name__ == '__main__':
    main()
