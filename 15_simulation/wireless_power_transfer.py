# -*- coding: utf-8 -*-
"""
无线功率传输仿真模块
====================
模拟磁耦合谐振式无线电能传输系统，分析传输效率、功率和频率特性。
适用于电赛中涉及无线充电、无线电能传输相关赛题的仿真验证。

主要功能:
    - 耦合模理论建模
    - 传输效率随距离变化分析
    - 频率分裂现象仿真
    - 负载对传输性能影响分析

作者: nuedc-asset-library
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize_scalar

# 中文字体设置
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
plt.rcParams['axes.unicode_minus'] = False


class WirelessPowerTransfer:
    """
    无线功率传输系统类
    
    基于耦合模理论(CMT)建模双线圈磁耦合谐振系统。
    
    参数:
        freq: 工作频率 (Hz)
        L1: 发射线圈电感 (H)
        L2: 接收线圈电感 (H)
        R1: 发射线圈等效电阻 (Ω)
        R2: 接收线圈等效电阻 (Ω)
        RL: 负载电阻 (Ω)
        Vs: 电源电压幅值 (V)
    """
    
    def __init__(self, freq=100e3, L1=100e-6, L2=100e-6,
                 R1=1.0, R2=1.0, RL=50.0, Vs=12.0):
        self.freq = freq
        self.omega = 2 * np.pi * freq
        self.L1 = L1
        self.L2 = L2
        self.R1 = R1
        self.R2 = R2
        self.RL = RL
        self.Vs = Vs
        
        # 谐振电容 (使系统工作在谐振状态)
        self.C1 = 1.0 / (self.omega**2 * L1)
        self.C2 = 1.0 / (self.omega**2 * L2)
    
    def mutual_inductance(self, distance, k_factor=0.1):
        """
        计算互感 (基于距离的简化模型)
        
        参数:
            distance: 线圈间距离 (m)
            k_factor: 耦合系数因子
            
        返回:
            M: 互感值 (H)
        """
        # 耦合系数随距离衰减的简化模型
        k = k_factor / (1 + (distance / 0.1)**2)**1.5
        M = k * np.sqrt(self.L1 * self.L2)
        return M
    
    def coupling_coefficient(self, distance, k_factor=0.1):
        """
        计算耦合系数
        
        参数:
            distance: 线圈间距离 (m)
            k_factor: 耦合系数因子
            
        返回:
            k: 耦合系数 (0-1)
        """
        k = k_factor / (1 + (distance / 0.1)**2)**1.5
        return min(k, 1.0)
    
    def transfer_efficiency(self, distance, k_factor=0.1):
        """
        计算传输效率
        
        参数:
            distance: 线圈间距离 (m)
            k_factor: 耦合系数因子
            
        返回:
            eta: 传输效率 (0-1)
        """
        k = self.coupling_coefficient(distance, k_factor)
        omega = self.omega
        
        # 品质因数
        Q1 = omega * self.L1 / self.R1
        Q2 = omega * self.L2 / self.R2
        
        # 负载品质因数
        QL = omega * self.L2 / (self.R2 + self.RL)
        
        # 传输效率 (谐振条件下)
        F = k**2 * Q1 * Q2
        eta = F / ((1 + F) * (1 + self.R2 / self.RL))
        
        # 限制在合理范围
        return np.clip(eta, 0, 1)
    
    def output_power(self, distance, k_factor=0.1):
        """
        计算输出功率
        
        参数:
            distance: 线圈间距离 (m)
            k_factor: 耦合系数因子
            
        返回:
            P_out: 输出功率 (W)
        """
        k = self.coupling_coefficient(distance, k_factor)
        omega = self.omega
        M = self.mutual_inductance(distance, k_factor)
        
        # 反射阻抗
        Z22 = self.R2 + self.RL + 1j * omega * self.L2 + 1 / (1j * omega * self.C2)
        Z_reflect = (omega * M)**2 / Z22
        
        # 输入阻抗
        Z11 = self.R1 + 1j * omega * self.L1 + 1 / (1j * omega * self.C1)
        Z_in = Z11 + Z_reflect
        
        # 输入电流
        I1 = self.Vs / Z_in
        
        # 输出电流和功率
        I2 = 1j * omega * M * I1 / Z22
        P_out = 0.5 * np.abs(I2)**2 * self.RL
        
        return P_out
    
    def frequency_response(self, freq_range, distance, k_factor=0.1):
        """
        频率响应分析
        
        参数:
            freq_range: 频率范围数组 (Hz)
            distance: 线圈间距离 (m)
            k_factor: 耦合系数因子
            
        返回:
            efficiencies: 各频率下的效率数组
            powers: 各频率下的功率数组
        """
        efficiencies = []
        powers = []
        
        for f in freq_range:
            old_freq = self.freq
            old_omega = self.omega
            old_C1 = self.C1
            old_C2 = self.C2
            
            # 临时修改频率
            self.freq = f
            self.omega = 2 * np.pi * f
            # 保持原谐振电容不变 (分析频率失谐)
            
            efficiencies.append(self.transfer_efficiency(distance, k_factor))
            powers.append(self.output_power(distance, k_factor))
            
            # 恢复原参数
            self.freq = old_freq
            self.omega = old_omega
            self.C1 = old_C1
            self.C2 = old_C2
        
        return np.array(efficiencies), np.array(powers)
    
    def find_optimal_frequency(self, distance, k_factor=0.1, 
                                freq_range=(50e3, 200e3)):
        """
        寻找最优工作频率
        
        参数:
            distance: 线圈间距离 (m)
            k_factor: 耦合系数因子
            freq_range: 频率搜索范围 (Hz)
            
        返回:
            f_opt: 最优频率 (Hz)
            eta_max: 最大效率
        """
        def neg_efficiency(f):
            old_freq = self.freq
            old_omega = self.omega
            self.freq = f
            self.omega = 2 * np.pi * f
            eta = self.transfer_efficiency(distance, k_factor)
            self.freq = old_freq
            self.omega = old_omega
            return -eta
        
        result = minimize_scalar(neg_efficiency, bounds=freq_range, method='bounded')
        return result.x, -result.fun


def efficiency_vs_distance(wpt, distances, k_factors=None):
    """
    分析效率随距离和耦合系数的变化
    
    参数:
        wpt: WirelessPowerTransfer实例
        distances: 距离数组 (m)
        k_factors: 耦合系数因子列表
        
    返回:
        results: 字典，包含各k_factor下的效率数据
    """
    if k_factors is None:
        k_factors = [0.05, 0.1, 0.2, 0.3]
    
    results = {}
    for k in k_factors:
        efficiencies = [wpt.transfer_efficiency(d, k) for d in distances]
        powers = [wpt.output_power(d, k) for d in distances]
        results[k] = {
            'efficiencies': np.array(efficiencies),
            'powers': np.array(powers)
        }
    
    return results


def frequency_splitting_analysis(wpt, distance, k_factors=None, n_points=200):
    """
    频率分裂现象分析
    
    在强耦合条件下，传输效率-频率曲线出现双峰现象。
    
    参数:
        wpt: WirelessPowerTransfer实例
        distance: 线圈间距离 (m)
        k_factors: 耦合系数因子列表
        n_points: 频率采样点数
        
    返回:
        freq_range: 频率数组
        results: 各k_factor下的效率数据
    """
    if k_factors is None:
        k_factors = [0.05, 0.1, 0.2, 0.3]
    
    # 频率范围 (以谐振频率为中心)
    f0 = wpt.freq
    freq_range = np.linspace(f0 * 0.5, f0 * 1.5, n_points)
    
    results = {}
    for k in k_factors:
        efficiencies, powers = wpt.frequency_response(freq_range, distance, k)
        results[k] = {
            'efficiencies': efficiencies,
            'powers': powers
        }
    
    return freq_range, results


def load_impact_analysis(wpt, distance, load_range=None, k_factor=0.1):
    """
    负载对传输性能的影响分析
    
    参数:
        wpt: WirelessPowerTransfer实例
        distance: 线圈间距离 (m)
        load_range: 负载电阻范围 (Ω)
        k_factor: 耦合系数因子
        
    返回:
        loads: 负载数组
        efficiencies: 效率数组
        powers: 功率数组
    """
    if load_range is None:
        load_range = np.linspace(1, 200, 100)
    
    efficiencies = []
    powers = []
    
    for RL in load_range:
        old_RL = wpt.RL
        wpt.RL = RL
        
        efficiencies.append(wpt.transfer_efficiency(distance, k_factor))
        powers.append(wpt.output_power(distance, k_factor))
        
        wpt.RL = old_RL
    
    return load_range, np.array(efficiencies), np.array(powers)


def plot_efficiency_vs_distance(wpt, distances, k_factors=None):
    """绘制效率-距离曲线"""
    results = efficiency_vs_distance(wpt, distances, k_factors)
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))
    
    for k, data in results.items():
        ax1.plot(distances * 100, data['efficiencies'] * 100, 
                label=f'k₀={k}', linewidth=2)
        ax2.plot(distances * 100, data['powers'], 
                label=f'k₀={k}', linewidth=2)
    
    ax1.set_xlabel('距离 (cm)', fontsize=12)
    ax1.set_ylabel('传输效率 (%)', fontsize=12)
    ax1.set_title('传输效率 vs 距离', fontsize=14)
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    ax2.set_xlabel('距离 (cm)', fontsize=12)
    ax2.set_ylabel('输出功率 (W)', fontsize=12)
    ax2.set_title('输出功率 vs 距离', fontsize=14)
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    return fig


def plot_frequency_splitting(wpt, distance, k_factors=None):
    """绘制频率分裂曲线"""
    freq_range, results = frequency_splitting_analysis(wpt, distance, k_factors)
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))
    
    for k, data in results.items():
        freq_norm = freq_range / wpt.freq
        ax1.plot(freq_norm, data['efficiencies'] * 100, 
                label=f'k₀={k}', linewidth=2)
        ax2.plot(freq_norm, data['powers'], 
                label=f'k₀={k}', linewidth=2)
    
    ax1.axvline(x=1.0, color='r', linestyle='--', alpha=0.5, label='谐振频率')
    ax1.set_xlabel('归一化频率 (f/f₀)', fontsize=12)
    ax1.set_ylabel('传输效率 (%)', fontsize=12)
    ax1.set_title('频率分裂现象 - 效率', fontsize=14)
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    ax2.axvline(x=1.0, color='r', linestyle='--', alpha=0.5, label='谐振频率')
    ax2.set_xlabel('归一化频率 (f/f₀)', fontsize=12)
    ax2.set_ylabel('输出功率 (W)', fontsize=12)
    ax2.set_title('频率分裂现象 - 功率', fontsize=14)
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    return fig


def plot_load_impact(wpt, distance, load_range=None, k_factor=0.1):
    """绘制负载影响曲线"""
    loads, efficiencies, powers = load_impact_analysis(wpt, distance, 
                                                        load_range, k_factor)
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))
    
    ax1.plot(loads, efficiencies * 100, 'b-', linewidth=2)
    ax1.set_xlabel('负载电阻 (Ω)', fontsize=12)
    ax1.set_ylabel('传输效率 (%)', fontsize=12)
    ax1.set_title(f'效率 vs 负载 (d={distance*100:.1f}cm)', fontsize=14)
    ax1.grid(True, alpha=0.3)
    
    ax2.plot(loads, powers, 'r-', linewidth=2)
    ax2.set_xlabel('负载电阻 (Ω)', fontsize=12)
    ax2.set_ylabel('输出功率 (W)', fontsize=12)
    ax2.set_title(f'功率 vs 负载 (d={distance*100:.1f}cm)', fontsize=14)
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    return fig


def plot_comprehensive_analysis(wpt, distances, distance_fixed=0.05):
    """综合分析图"""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    # 子图1: 效率-距离
    k_factors = [0.05, 0.1, 0.2, 0.3]
    results = efficiency_vs_distance(wpt, distances, k_factors)
    for k, data in results.items():
        axes[0, 0].plot(distances * 100, data['efficiencies'] * 100, 
                       label=f'k₀={k}', linewidth=2)
    axes[0, 0].set_xlabel('距离 (cm)')
    axes[0, 0].set_ylabel('效率 (%)')
    axes[0, 0].set_title('效率-距离特性')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)
    
    # 子图2: 频率分裂
    freq_range, freq_results = frequency_splitting_analysis(wpt, distance_fixed)
    for k, data in freq_results.items():
        freq_norm = freq_range / wpt.freq
        axes[0, 1].plot(freq_norm, data['efficiencies'] * 100, 
                       label=f'k₀={k}', linewidth=2)
    axes[0, 1].axvline(x=1.0, color='r', linestyle='--', alpha=0.5)
    axes[0, 1].set_xlabel('归一化频率')
    axes[0, 1].set_ylabel('效率 (%)')
    axes[0, 1].set_title('频率分裂现象')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)
    
    # 子图3: 负载影响
    loads, eff, pwr = load_impact_analysis(wpt, distance_fixed)
    axes[1, 0].plot(loads, eff * 100, 'b-', linewidth=2)
    axes[1, 0].set_xlabel('负载电阻 (Ω)')
    axes[1, 0].set_ylabel('效率 (%)')
    axes[1, 0].set_title(f'效率-负载特性 (d={distance_fixed*100}cm)')
    axes[1, 0].grid(True, alpha=0.3)
    
    axes[1, 1].plot(loads, pwr, 'r-', linewidth=2)
    axes[1, 1].set_xlabel('负载电阻 (Ω)')
    axes[1, 1].set_ylabel('功率 (W)')
    axes[1, 1].set_title(f'功率-负载特性 (d={distance_fixed*100}cm)')
    axes[1, 1].grid(True, alpha=0.3)
    
    plt.suptitle('无线功率传输系统综合分析', fontsize=16, y=1.02)
    plt.tight_layout()
    return fig


def run_demo():
    """
    运行完整仿真演示
    
    演示内容:
        1. 创建WPT系统实例
        2. 效率-距离分析
        3. 频率分裂分析
        4. 负载影响分析
        5. 综合分析图
    """
    print("=" * 60)
    print("无线功率传输系统仿真")
    print("=" * 60)
    
    # 创建系统实例
    wpt = WirelessPowerTransfer(
        freq=100e3,      # 100kHz
        L1=100e-6,       # 100μH
        L2=100e-6,       # 100μH
        R1=1.0,          # 1Ω
        R2=1.0,          # 1Ω
        RL=50.0,         # 50Ω
        Vs=12.0          # 12V
    )
    
    print(f"\n系统参数:")
    print(f"  工作频率: {wpt.freq/1e3:.0f} kHz")
    print(f"  发射电感: {wpt.L1*1e6:.0f} μH")
    print(f"  接收电感: {wpt.L2*1e6:.0f} μH")
    print(f"  谐振电容: {wpt.C1*1e9:.2f} nF")
    print(f"  负载电阻: {wpt.RL:.0f} Ω")
    print(f"  电源电压: {wpt.Vs:.0f} V")
    
    # 1. 效率-距离分析
    print("\n" + "-" * 40)
    print("1. 效率-距离分析")
    distances = np.linspace(0.01, 0.3, 50)  # 1cm - 30cm
    
    test_distances = [0.02, 0.05, 0.1, 0.15, 0.2]
    for d in test_distances:
        eta = wpt.transfer_efficiency(d, k_factor=0.1)
        P = wpt.output_power(d, k_factor=0.1)
        print(f"  距离 {d*100:.0f}cm: 效率={eta*100:.1f}%, 功率={P:.3f}W")
    
    # 2. 频率分裂分析
    print("\n" + "-" * 40)
    print("2. 频率分裂分析")
    print("  强耦合下频率分裂现象:")
    for k in [0.05, 0.1, 0.2, 0.3]:
        eta_res = wpt.transfer_efficiency(0.05, k)
        print(f"  k₀={k}: 谐振效率={eta_res*100:.1f}%")
    
    # 3. 最优频率
    print("\n" + "-" * 40)
    print("3. 最优频率搜索")
    for d in [0.05, 0.1, 0.15]:
        f_opt, eta_max = wpt.find_optimal_frequency(d, k_factor=0.1)
        print(f"  距离 {d*100:.0f}cm: 最优频率={f_opt/1e3:.1f}kHz, 最大效率={eta_max*100:.1f}%")
    
    # 4. 生成图表
    print("\n" + "-" * 40)
    print("4. 生成分析图表...")
    
    try:
        fig1 = plot_efficiency_vs_distance(wpt, distances)
        fig1.savefig('wpt_efficiency_distance.png', dpi=150, bbox_inches='tight')
        print("  ✓ 效率-距离图已保存")
    except Exception as e:
        print(f"  ✗ 效率-距离图生成失败: {e}")
    
    try:
        fig2 = plot_frequency_splitting(wpt, distance=0.05)
        fig2.savefig('wpt_frequency_splitting.png', dpi=150, bbox_inches='tight')
        print("  ✓ 频率分裂图已保存")
    except Exception as e:
        print(f"  ✗ 频率分裂图生成失败: {e}")
    
    try:
        fig3 = plot_comprehensive_analysis(wpt, distances, distance_fixed=0.05)
        fig3.savefig('wpt_comprehensive.png', dpi=150, bbox_inches='tight')
        print("  ✓ 综合分析图已保存")
    except Exception as e:
        print(f"  ✗ 综合分析图生成失败: {e}")
    
    print("\n" + "=" * 60)
    print("仿真完成!")
    print("=" * 60)
    
    return wpt


if __name__ == '__main__':
    wpt = run_demo()
    plt.show()
