#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
编码器分辨率研究仿真 — 不同线数编码器的测速精度
================================================
模拟不同线数(PPR)编码器在不同转速下的测速方法比较:
  - M法(测频法): 固定时间计脉冲数
  - T法(测周法): 固定脉冲测时间
  - M/T法: 综合法
依赖: numpy, matplotlib
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

# ═══════════════════════════════════════════════════════════════
# 仿真参数
# ═══════════════════════════════════════════════════════════════

# 编码器线数 (PPR, pulses per revolution)
PPR_LIST = [100, 200, 500, 1000, 2000, 5000]

# 倍频模式 (4倍频)
MULTIPLIER = 4

# 定时器时钟频率 (用于T法测时间)
TIMER_FREQ = 72e6  # 72MHz (STM32典型)

# 测速周期 (M法的采样时间)
SAMPLE_TIME = 0.01  # 10ms

# 真实转速范围 (RPM)
RPM_RANGE = np.logspace(0, 3.5, 200)  # 1 ~ 3000 RPM

# ═══════════════════════════════════════════════════════════════
# 测速误差分析
# ═══════════════════════════════════════════════════════════════

def m_method_error(rpm, ppr, t_sample):
    """
    M法(测频法)测速误差分析
    原理: 固定时间T内计脉冲数M, 转速 = M/(PPR*T)
    误差来源: ±1个脉冲的量化误差
    最大误差: 1/(M) = 1/(PPR * rpm/60 * T)
    """
    freq = rpm / 60.0 * ppr  # 脉冲频率 (Hz)
    M = freq * t_sample  # 采样周期内的脉冲数
    M = np.maximum(M, 1)  # 防止除零
    # 相对误差 = 1/M
    rel_error = 1.0 / M
    return rel_error * 100  # 百分比

def t_method_error(rpm, ppr, timer_freq):
    """
    T法(测周法)测速误差分析
    原理: 测量相邻脉冲之间的时间T_p, 转速 = 1/(PPR*T_p)
    误差来源: ±1个定时器计数
    最大误差: 1/(timer_freq * T_p) = PPR * rpm / (60 * timer_freq)
    """
    T_pulse = 60.0 / (rpm * ppr)  # 脉冲周期 (s)
    counts = timer_freq * T_pulse  # 定时器计数值
    counts = np.maximum(counts, 1)
    rel_error = 1.0 / counts
    return rel_error * 100

def mt_method_error(rpm, ppr, t_sample, timer_freq):
    """
    M/T法测速误差分析 (综合法)
    在固定采样时间结束后, 等待下一个脉冲到来
    同时记录脉冲数M和精确时间T
    误差: 取决于定时器分辨率和实际脉冲数
    """
    rpm = np.asarray(rpm, dtype=float)
    freq = rpm / 60.0 * ppr
    M = np.maximum(freq * t_sample, 1.0)
    # M/T法: 误差 = 1/(实际定时器计数)
    # 实际计数 = timer_freq * (M / freq) ≈ timer_freq * t_sample (高速)
    # 低速时: 实际计数 = timer_freq * (1/freq) = timer_freq * 60/(rpm*ppr)
    actual_time = M / np.maximum(freq, 1e-10)
    timer_counts = timer_freq * actual_time
    rel_error = 1.0 / np.maximum(timer_counts, 1.0)
    return rel_error * 100

def monte_carlo_speed_estimation(rpm_true, ppr, method='M', n_trials=1000):
    """
    Monte Carlo仿真: 模拟实际测速过程
    """
    actual_ppr = ppr * MULTIPLIER
    speeds_est = []

    for _ in range(n_trials):
        if method == 'M':
            # M法: 计数脉冲数 (含±1量化)
            freq = rpm_true / 60.0 * actual_ppr
            M_ideal = freq * SAMPLE_TIME
            M_count = int(M_ideal + np.random.uniform(-0.5, 0.5))
            M_count = max(M_count, 1)
            rpm_est = M_count / (actual_ppr * SAMPLE_TIME) * 60

        elif method == 'T':
            # T法: 测量脉冲间隔时间
            T_pulse = 60.0 / (rpm_true * actual_ppr)
            timer_counts = TIMER_FREQ * T_pulse
            timer_counts_measured = int(timer_counts + np.random.uniform(-0.5, 0.5))
            timer_counts_measured = max(timer_counts_measured, 1)
            T_measured = timer_counts_measured / TIMER_FREQ
            rpm_est = 60.0 / (actual_ppr * T_measured)

        elif method == 'MT':
            # M/T法
            freq = rpm_true / 60.0 * actual_ppr
            M_ideal = freq * SAMPLE_TIME
            M_count = int(M_ideal + np.random.uniform(-0.5, 0.5))
            M_count = max(M_count, 1)
            # 实际时间由定时器精确测量
            T_exact = M_count / (rpm_true / 60.0 * actual_ppr)
            T_measured = T_exact + np.random.uniform(-0.5, 0.5) / TIMER_FREQ
            rpm_est = M_count / (actual_ppr * max(T_measured, 1e-10)) * 60

        speeds_est.append(rpm_est)

    return np.array(speeds_est)


if __name__ == '__main__':
    plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False

    print("=" * 60)
    print("编码器分辨率研究仿真")
    print("=" * 60)
    print(f"编码器线数: {PPR_LIST}")
    print(f"倍频模式: {MULTIPLIER}x")
    print(f"测速周期: {SAMPLE_TIME*1000:.0f} ms")
    print(f"定时器频率: {TIMER_FREQ/1e6:.0f} MHz")

    # ═══════════════════════════════════════════════════════════════
    # 2. Monte Carlo 仿真验证
    # ═══════════════════════════════════════════════════════════════

    # 运行Monte Carlo仿真
    test_rpms = [10, 50, 200, 1000]  # 测试转速点
    test_ppr = 500  # 固定一个编码器线数

    print("\nMonte Carlo仿真 (PPR=500, 4倍频):")
    print(f"{'转速(RPM)':<12} {'M法误差%':<12} {'T法误差%':<12} {'M/T法误差%':<12}")
    print("-" * 50)

    mc_results = {}
    for rpm in test_rpms:
        mc_m = monte_carlo_speed_estimation(rpm, test_ppr, 'M')
        mc_t = monte_carlo_speed_estimation(rpm, test_ppr, 'T')
        mc_mt = monte_carlo_speed_estimation(rpm, test_ppr, 'MT')

        err_m = np.std(mc_m) / rpm * 100
        err_t = np.std(mc_t) / rpm * 100
        err_mt = np.std(mc_mt) / rpm * 100

        mc_results[rpm] = (mc_m, mc_t, mc_mt)
        print(f"{rpm:<12} {err_m:<12.4f} {err_t:<12.4f} {err_mt:<12.4f}")

    # ═══════════════════════════════════════════════════════════════
    # 3. 绘图
    # ═══════════════════════════════════════════════════════════════

    fig = plt.figure(figsize=(20, 16))
    fig.suptitle('编码器分辨率研究仿真 — 不同线数编码器的测速精度分析',
                 fontsize=16, fontweight='bold')

    gs = GridSpec(3, 3, hspace=0.4, wspace=0.35)

    # --- 子图1: M法误差 vs 转速 (不同PPR) ---
    ax1 = fig.add_subplot(gs[0, 0])
    colors_ppr = plt.cm.viridis(np.linspace(0.1, 0.9, len(PPR_LIST)))
    for ppr, color in zip(PPR_LIST, colors_ppr):
        err = m_method_error(RPM_RANGE, ppr * MULTIPLIER, SAMPLE_TIME)
        ax1.loglog(RPM_RANGE, err, color=color, linewidth=1.5, label=f'{ppr}PPR')
    ax1.set_xlabel('转速 (RPM)')
    ax1.set_ylabel('相对误差 (%)')
    ax1.set_title('M法(测频法)测速误差')
    ax1.legend(fontsize=8)
    ax1.grid(True, alpha=0.3, which='both')
    ax1.set_ylim([1e-6, 100])

    # --- 子图2: T法误差 vs 转速 ---
    ax2 = fig.add_subplot(gs[0, 1])
    for ppr, color in zip(PPR_LIST, colors_ppr):
        err = t_method_error(RPM_RANGE, ppr * MULTIPLIER, TIMER_FREQ)
        ax2.loglog(RPM_RANGE, err, color=color, linewidth=1.5, label=f'{ppr}PPR')
    ax2.set_xlabel('转速 (RPM)')
    ax2.set_ylabel('相对误差 (%)')
    ax2.set_title('T法(测周法)测速误差')
    ax2.legend(fontsize=8)
    ax2.grid(True, alpha=0.3, which='both')
    ax2.set_ylim([1e-6, 100])

    # --- 子图3: M/T法误差 ---
    ax3 = fig.add_subplot(gs[0, 2])
    for ppr, color in zip(PPR_LIST, colors_ppr):
        err = mt_method_error(RPM_RANGE, ppr * MULTIPLIER, SAMPLE_TIME, TIMER_FREQ)
        ax3.loglog(RPM_RANGE, err, color=color, linewidth=1.5, label=f'{ppr}PPR')
    ax3.set_xlabel('转速 (RPM)')
    ax3.set_ylabel('相对误差 (%)')
    ax3.set_title('M/T法(综合法)测速误差')
    ax3.legend(fontsize=8)
    ax3.grid(True, alpha=0.3, which='both')
    ax3.set_ylim([1e-6, 100])

    # --- 子图4: 三种方法对比 (PPR=1000) ---
    ax4 = fig.add_subplot(gs[1, 0])
    ppr_compare = 1000
    err_m = m_method_error(RPM_RANGE, ppr_compare * MULTIPLIER, SAMPLE_TIME)
    err_t = t_method_error(RPM_RANGE, ppr_compare * MULTIPLIER, TIMER_FREQ)
    err_mt = mt_method_error(RPM_RANGE, ppr_compare * MULTIPLIER, SAMPLE_TIME, TIMER_FREQ)
    ax4.loglog(RPM_RANGE, err_m, 'r-', linewidth=2, label='M法(测频法)')
    ax4.loglog(RPM_RANGE, err_t, 'b-', linewidth=2, label='T法(测周法)')
    ax4.loglog(RPM_RANGE, err_mt, 'g-', linewidth=2, label='M/T法')
    ax4.axhline(y=0.1, color='gray', linestyle=':', alpha=0.5, label='0.1%基准线')
    ax4.set_xlabel('转速 (RPM)')
    ax4.set_ylabel('相对误差 (%)')
    ax4.set_title(f'三种测速方法对比 (PPR={ppr_compare})')
    ax4.legend(fontsize=9)
    ax4.grid(True, alpha=0.3, which='both')

    # --- 子图5: Monte Carlo 直方图 ---
    ax5 = fig.add_subplot(gs[1, 1])
    rpm_mc = 200
    mc_m, mc_t, mc_mt = mc_results[rpm_mc]
    ax5.hist(mc_m - rpm_mc, bins=50, alpha=0.6, color='red', label='M法', density=True)
    ax5.hist(mc_t - rpm_mc, bins=50, alpha=0.6, color='blue', label='T法', density=True)
    ax5.hist(mc_mt - rpm_mc, bins=50, alpha=0.6, color='green', label='M/T法', density=True)
    ax5.set_xlabel('速度估计误差 (RPM)')
    ax5.set_ylabel('概率密度')
    ax5.set_title(f'Monte Carlo测速误差分布 ({rpm_mc}RPM)')
    ax5.legend()
    ax5.grid(True, alpha=0.3)

    # --- 子图6: Monte Carlo 低速 ---
    ax6 = fig.add_subplot(gs[1, 2])
    rpm_mc_low = 10
    mc_m2, mc_t2, mc_mt2 = mc_results[rpm_mc_low]
    ax6.hist(mc_m2 - rpm_mc_low, bins=50, alpha=0.6, color='red', label='M法', density=True)
    ax6.hist(mc_t2 - rpm_mc_low, bins=50, alpha=0.6, color='blue', label='T法', density=True)
    ax6.hist(mc_mt2 - rpm_mc_low, bins=50, alpha=0.6, color='green', label='M/T法', density=True)
    ax6.set_xlabel('速度估计误差 (RPM)')
    ax6.set_ylabel('概率密度')
    ax6.set_title(f'Monte Carlo测速误差分布 ({rpm_mc_low}RPM)')
    ax6.legend()
    ax6.grid(True, alpha=0.3)

    # --- 子图7: 分辨率 vs 最小可测转速 ---
    ax7 = fig.add_subplot(gs[2, 0])
    min_speeds_m = []
    min_speeds_t = []
    for ppr in PPR_LIST:
        # M法: 最小可测转速 = 1脉冲/采样时间 → 60/(PPR*T)
        min_m = 60.0 / (ppr * MULTIPLIER * SAMPLE_TIME)
        # T法: 最大可测转速受限于定时器分辨率
        # 但最小可测转速基本不受限
        min_t = 60.0 / (ppr * MULTIPLIER) * TIMER_FREQ / (TIMER_FREQ * 10)  # 约定10个计数最小
        min_speeds_m.append(min_m)
        min_speeds_t.append(min_t)

    ax7.bar(np.arange(len(PPR_LIST)) - 0.2, min_speeds_m, 0.4,
            color='#e74c3c', alpha=0.8, label='M法最小转速')
    ax7.bar(np.arange(len(PPR_LIST)) + 0.2, min_speeds_t, 0.4,
            color='#3498db', alpha=0.8, label='T法最小转速')
    ax7.set_xticks(np.arange(len(PPR_LIST)))
    ax7.set_xticklabels([str(p) for p in PPR_LIST])
    ax7.set_xlabel('编码器线数 (PPR)')
    ax7.set_ylabel('最小可测转速 (RPM)')
    ax7.set_title('不同PPR编码器的最小可测转速')
    ax7.set_yscale('log')
    ax7.legend()
    ax7.grid(True, alpha=0.3, axis='y')

    # --- 子图8: 推荐选择区域 ---
    ax8 = fig.add_subplot(gs[2, 1])
    # 热力图: 不同PPR在不同转速下的最佳方法
    rpm_grid = np.array([5, 10, 50, 100, 500, 1000, 3000])
    ppr_grid = np.array([100, 500, 1000, 2000, 5000])

    best_method = np.zeros((len(ppr_grid), len(rpm_grid)))
    for i, ppr in enumerate(ppr_grid):
        for j, rpm in enumerate(rpm_grid):
            err_m = m_method_error(rpm, ppr * MULTIPLIER, SAMPLE_TIME)
            err_t = t_method_error(rpm, ppr * MULTIPLIER, TIMER_FREQ)
            err_mt = mt_method_error(rpm, ppr * MULTIPLIER, SAMPLE_TIME, TIMER_FREQ)
            # 选择误差最小的方法: 0=M, 1=T, 2=MT
            errors = [err_m, err_t, err_mt]
            best_method[i, j] = np.argmin(errors)

    im = ax8.imshow(best_method, cmap='RdYlGn', aspect='auto', vmin=0, vmax=2)
    ax8.set_xticks(np.arange(len(rpm_grid)))
    ax8.set_xticklabels([str(r) for r in rpm_grid])
    ax8.set_yticks(np.arange(len(ppr_grid)))
    ax8.set_yticklabels([str(p) for p in ppr_grid])
    ax8.set_xlabel('转速 (RPM)')
    ax8.set_ylabel('编码器线数 (PPR)')
    ax8.set_title('最佳测速方法推荐')
    # 添加文字标注
    method_names = ['M法', 'T法', 'M/T法']
    for i in range(len(ppr_grid)):
        for j in range(len(rpm_grid)):
            ax8.text(j, i, method_names[int(best_method[i, j])],
                    ha='center', va='center', fontsize=8, fontweight='bold')
    fig.colorbar(im, ax=ax8, ticks=[0, 1, 2], label='方法')

    # --- 子图9: 说明 ---
    ax9 = fig.add_subplot(gs[2, 2])
    ax9.axis('off')
    info = (
        "测速方法说明\n"
        "═══════════════════\n\n"
        "【M法(测频法)】\n"
        "固定时间T内计脉冲数M\n"
        "高速精度高, 低速误差大\n\n"
        "【T法(测周法)】\n"
        "测相邻脉冲间隔时间T_p\n"
        "低速精度高, 高速误差大\n\n"
        "【M/T法(综合法)】\n"
        "结合M法和T法优点\n"
        "全速度范围精度高\n\n"
        "【编码器线数选择】\n"
        "• 低速高精度: 高PPR+T法\n"
        "• 高速响应: 中PPR+M法\n"
        "• 全范围: 高PPR+M/T法\n\n"
        f"定时器: {TIMER_FREQ/1e6:.0f}MHz\n"
        f"采样周期: {SAMPLE_TIME*1000:.0f}ms"
    )
    ax9.text(0.05, 0.95, info, transform=ax9.transAxes, fontsize=10,
             verticalalignment='top', fontfamily='monospace',
             bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))

    plt.savefig('encoder_resolution_study_result.png', dpi=150, bbox_inches='tight')
    print("\n图表已保存: encoder_resolution_study_result.png")
    print("仿真完成!")
