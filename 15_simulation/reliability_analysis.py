#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
可靠性分析仿真 - MTBF/应力分析/加速寿命试验
============================================
适用于电赛系统可靠性评估与设计
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy import stats
from matplotlib import rcParams
rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
rcParams['axes.unicode_minus'] = False

# ============================================================
# 1. 失效分布模型
# ============================================================
class FailureDistribution:
    """指数分布、威布尔分布、对数正态分布"""

    @staticmethod
    def exponential(t, lam):
        """指数分布 f(t) = λ*exp(-λt)"""
        return lam * np.exp(-lam * t)

    @staticmethod
    def exponential_reliability(t, lam):
        """R(t) = exp(-λt)"""
        return np.exp(-lam * t)

    @staticmethod
    def weibull(t, beta, eta):
        """威布尔分布 f(t) = (β/η)*(t/η)^(β-1)*exp(-(t/η)^β)"""
        return (beta/eta) * (t/eta)**(beta-1) * np.exp(-(t/eta)**beta)

    @staticmethod
    def weibull_reliability(t, beta, eta):
        """R(t) = exp(-(t/η)^β)"""
        return np.exp(-(t/eta)**beta)

    @staticmethod
    def lognormal_reliability(t, mu, sigma):
        """对数正态可靠性"""
        return 1 - stats.lognorm.cdf(t, sigma, scale=np.exp(mu))


# ============================================================
# 2. MTBF计算
# ============================================================
def mtbf_exponential(lam):
    """指数分布MTBF = 1/λ"""
    return 1/lam


def mtbf_weibull(beta, eta):
    """威布尔分布MTBF = η*Γ(1+1/β)"""
    from scipy.special import gamma
    return eta * gamma(1 + 1/beta)


def system_mtbf_parallel(mtbf_list):
    """并联冗余系统MTBF (2冗余)"""
    lam_sum = sum(1/m for m in mtbf_list)
    # 2/λ for dual redundancy
    return 2/lam_sum if len(mtbf_list) == 1 else 1/lam_sum


def system_mtbf_series(mtbf_list):
    """串联可靠性系统MTBF"""
    lam_sum = sum(1/m for m in mtbf_list)
    return 1/lam_sum


# ============================================================
# 3. 应力分析
# ============================================================
class StressAnalysis:
    """应力-强度干涉模型"""

    @staticmethod
    def interference_reliability(mu_s, sigma_s, mu_l, sigma_l):
        """应力-强度干涉可靠度"""
        # R = P(强度 > 应力) = Φ((μ_l-μ_s)/sqrt(σ_l²+σ_s²))
        z = (mu_l - mu_s) / np.sqrt(sigma_l**2 + sigma_s**2)
        return stats.norm.cdf(z)

    @staticmethod
    def derating_analysis(rated_value, actual_value, temp_coeff=0.01, T=25):
        """降额分析"""
        derating_pct = actual_value / rated_value * 100
        effective_rating = rated_value * (1 - temp_coeff * (T - 25))
        return derating_pct, effective_rating


# ============================================================
# 4. 加速寿命试验 (ALT)
# ============================================================
class AcceleratedLifeTest:
    """Arrhenius, Coffin-Manson, Eyring加速模型"""

    @staticmethod
    def arrhenius(A, Ea, T_K):
        """Arrhenius: AF = exp(Ea/k * (1/T_use - 1/T_stress))"""
        k = 8.617e-5  # eV/K
        return A * np.exp(-Ea / (k * T_K))

    @staticmethod
    def arrhenius_acceleration_factor(Ea, T_use_K, T_stress_K):
        """Arrhenius加速因子"""
        k = 8.617e-5
        return np.exp(Ea/k * (1/T_use_K - 1/T_stress_K))

    @staticmethod
    def acceleration_factor_coffin_manson(n, T_cycle_use, T_cycle_stress):
        """热循环加速因子 (Coffin-Manson)"""
        return (T_cycle_stress / T_cycle_use)**n

    @staticmethod
    def voltage_acceleration(gamma, V_use, V_stress):
        """电压加速因子"""
        return np.exp(gamma * (V_stress - V_use))


# ============================================================
# 5. 可靠性框图分析
# ============================================================
def reliability_block_diagram(n_sim=10000, components=None):
    """蒙特卡洛可靠性框图仿真"""
    if components is None:
        components = [
            {'name': 'MCU', 'mtbf': 50000, 'type': 'series'},
            {'name': '电源', 'mtbf': 30000, 'type': 'series'},
            {'name': '传感器A', 'mtbf': 40000, 'type': 'series'},
            {'name': '传感器B', 'mtbf': 40000, 'type': 'parallel', 'n': 2},
            {'name': '通信', 'mtbf': 35000, 'type': 'series'},
        ]

    t_test = np.linspace(100, 20000, 200)
    R_system = np.zeros(len(t_test))

    for _ in range(n_sim):
        sys_fail_time = float('inf')
        for comp in components:
            lam = 1/comp['mtbf']
            if comp['type'] == 'parallel':
                # 并联: max of n lifetimes
                fail_times = np.random.exponential(1/lam, comp.get('n', 2))
                comp_fail = np.max(fail_times)
            else:
                comp_fail = np.random.exponential(1/lam)
            sys_fail_time = min(sys_fail_time, comp_fail)

        R_system += (t_test < sys_fail_time).astype(float)

    R_system /= n_sim
    return t_test, R_system


# ============================================================
# 主仿真
# ============================================================
def main():
    fig, axes = plt.subplots(3, 2, figsize=(14, 16))
    fig.suptitle('可靠性分析仿真综合', fontsize=16, fontweight='bold')

    fd = FailureDistribution()
    alt = AcceleratedLifeTest()

    # --- 1. 不同失效分布对比 ---
    ax = axes[0, 0]
    t = np.linspace(0, 5000, 500)
    R_exp = fd.exponential_reliability(t, 1/1000)
    R_weibull1 = fd.weibull_reliability(t, 1.5, 1200)
    R_weibull2 = fd.weibull_reliability(t, 3.0, 1500)
    R_logn = fd.lognormal_reliability(t, 7.0, 0.5)

    ax.plot(t, R_exp, 'b-', linewidth=2, label='指数 (λ=1/1000h)')
    ax.plot(t, R_weibull1, 'r-', linewidth=2, label='威布尔 (β=1.5)')
    ax.plot(t, R_weibull2, 'g-', linewidth=2, label='威布尔 (β=3.0)')
    ax.plot(t, R_logn, 'm-', linewidth=2, label='对数正态')
    ax.set_xlabel('时间 (h)')
    ax.set_ylabel('可靠度 R(t)')
    ax.set_title('不同失效分布可靠度曲线')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # --- 2. 威布尔形状参数影响 ---
    ax = axes[0, 1]
    for beta in [0.5, 1.0, 1.5, 2.0, 3.0, 5.0]:
        R = fd.weibull_reliability(t, beta, 2000)
        ax.plot(t, R, linewidth=2, label=f'β={beta}')
    ax.set_xlabel('时间 (h)')
    ax.set_ylabel('可靠度')
    ax.set_title('威布尔分布形状参数影响')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # --- 3. 失效率曲线 (浴盆曲线) ---
    ax = axes[1, 0]
    t_bath = np.linspace(0, 10000, 500)
    # 早期失效 (递减)
    early = 0.001 * np.exp(-t_bath/500)
    # 偶然失效 (恒定)
    random_fail = 0.0001 * np.ones_like(t_bath)
    # 磨损失效 (递增)
    wearout = 0.0001 * (t_bath/5000)**2

    total = early + random_fail + wearout
    ax.plot(t_bath, total*1000, 'k-', linewidth=2, label='总失效率')
    ax.plot(t_bath, early*1000, '--', linewidth=1, label='早期失效')
    ax.plot(t_bath, random_fail*1000, '--', linewidth=1, label='偶然失效')
    ax.plot(t_bath, wearout*1000, '--', linewidth=1, label='磨损失效')
    ax.set_xlabel('时间 (h)')
    ax.set_ylabel('失效率 (×10⁻³/h)')
    ax.set_title('浴盆曲线')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # --- 4. 应力-强度干涉 ---
    ax = axes[1, 1]
    sa = StressAnalysis()
    x = np.linspace(0, 100, 500)
    # 强度分布 (材料强度)
    mu_l, sigma_l = 60, 8
    # 应力分布 (工作应力)
    mu_s, sigma_s = 35, 7

    f_strength = stats.norm.pdf(x, mu_l, sigma_l)
    f_stress = stats.norm.pdf(x, mu_s, sigma_s)
    ax.fill_between(x, np.minimum(f_strength, f_stress), alpha=0.3, color='red', label='干涉区')
    ax.plot(x, f_strength, 'g-', linewidth=2, label=f'强度 (μ={mu_l},σ={sigma_l})')
    ax.plot(x, f_stress, 'r-', linewidth=2, label=f'应力 (μ={mu_s},σ={sigma_s})')

    R_interf = sa.interference_reliability(mu_s, sigma_s, mu_l, sigma_l)
    ax.set_xlabel('应力/强度')
    ax.set_ylabel('概率密度')
    ax.set_title(f'应力-强度干涉 (R={R_interf:.4f})')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # --- 5. Arrhenius加速寿命 ---
    ax = axes[2, 0]
    T_stress = np.array([373, 398, 423, 448, 473])  # K (100~200°C)
    Ea = 0.7  # eV
    AF = alt.arrhenius_acceleration_factor(Ea, 298, T_stress)

    # 模拟试验数据
    lifetime_stress = 1000 / AF  # 加速后寿命
    T_C = T_stress - 273

    ax.semilogy(1000/T_stress, lifetime_stress, 'ro-', markersize=8, linewidth=2)
    # 外推到使用温度
    T_use_line = np.linspace(298, 473, 50)
    lifetime_use = 1000 / alt.arrhenius_acceleration_factor(Ea, 298, T_use_line)
    ax.semilogy(1000/T_use_line, lifetime_use, 'b--', linewidth=2, label='Arrhenius外推')

    ax.axvline(1000/298, color='g', linestyle=':', label='使用温度25°C')
    ax.set_xlabel('1000/T (1/K)')
    ax.set_ylabel('寿命 (h)')
    ax.set_title(f'Arrhenius加速寿命试验 (Ea={Ea}eV)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # --- 6. 系统可靠性框图 (蒙特卡洛) ---
    ax = axes[2, 1]
    t_sys, R_sys = reliability_block_diagram()
    ax.plot(t_sys, R_sys, 'b-', linewidth=2, label='系统可靠度')

    # 单组件对比
    for comp in [{'name': 'MCU', 'mtbf': 50000}, {'name': '电源', 'mtbf': 30000}]:
        R_comp = fd.exponential_reliability(t_sys, 1/comp['mtbf'])
        ax.plot(t_sys, R_comp, '--', linewidth=1, label=f"{comp['name']} (MTBF={comp['mtbf']}h)")

    ax.axhline(0.9, color='r', linestyle=':', alpha=0.5, label='R=0.9')
    ax.set_xlabel('任务时间 (h)')
    ax.set_ylabel('可靠度')
    ax.set_title('系统可靠性框图 (蒙特卡洛)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out = r'./nuedc-asset-library\15_simulation\reliability_analysis_result.png'
    plt.savefig(out, dpi=150, bbox_inches='tight')
    print(f'已保存: {out}')

    # 统计
    print(f'\n=== 可靠性分析统计 ===')
    print(f'指数分布 MTBF=1000h: R(500h)={fd.exponential_reliability(500, 1/1000):.4f}')
    print(f'威布尔(β=3,η=1500h) MTBF={mtbf_weibull(3, 1500):.0f}h')
    print(f'应力-强度干涉可靠度: {R_interf:.6f}')
    print(f'Arrhenius加速因子@150°C: {alt.arrhenius_acceleration_factor(0.7, 298, 423):.1f}x')


if __name__ == '__main__':
    main()
