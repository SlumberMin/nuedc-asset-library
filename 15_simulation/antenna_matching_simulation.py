#!/usr/bin/env python3
"""
天线匹配仿真 - 阻抗匹配 + SWR + 史密斯圆图
适用于电赛天线设计与射频匹配网络优化
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rcParams
from matplotlib.patches import Circle
import matplotlib.patches as mpatches

rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial']
rcParams['axes.unicode_minus'] = False


# ==================== 史密斯圆图工具 ====================
class SmithChart:
    """史密斯圆图绘制与阻抗计算"""

    def __init__(self, Z0=50.0):
        self.Z0 = Z0

    def z_to_gamma(self, Z):
        """阻抗 → 反射系数"""
        return (Z - self.Z0) / (Z + self.Z0)

    def gamma_to_z(self, gamma):
        """反射系数 → 阻抗"""
        return self.Z0 * (1 + gamma) / (1 - gamma)

    def z_to_swr(self, Z):
        """阻抗 → 驻波比"""
        gamma = np.abs(self.z_to_gamma(Z))
        gamma = np.clip(gamma, 0, 0.999)
        return (1 + gamma) / (1 - gamma)

    def draw(self, ax=None):
        """绘制史密斯圆图底图"""
        if ax is None:
            fig, ax = plt.subplots(1, 1, figsize=(8, 8))

        # 外圆 (|Γ|=1)
        theta = np.linspace(0, 2*np.pi, 300)
        ax.plot(np.cos(theta), np.sin(theta), 'k-', linewidth=1.5)

        # 等电阻圆
        for r in [0, 0.2, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0]:
            center_r = r / (1 + r)
            radius_r = 1 / (1 + r)
            circle = plt.Circle((center_r, 0), radius_r, fill=False,
                                color='blue', linewidth=0.8, alpha=0.6)
            ax.add_patch(circle)
            if r in [0.2, 0.5, 1.0, 2.0, 5.0, 10.0]:
                ax.text(center_r + radius_r + 0.01, 0.02, f'r={r}',
                        fontsize=7, color='blue', alpha=0.8)

        # 等电抗圆 (上半=感性, 下半=容性)
        for x in [0.2, 0.5, 1.0, 2.0, 5.0]:
            center_x = (1, 1/x)
            radius_x = 1/x
            # 上半 (正电抗, 感性)
            theta_arc = np.linspace(np.arccos(min(1, (radius_x**2 + center_x[0]**2 + center_x[1]**2 - 1) /
                                    (2*radius_x*np.sqrt(center_x[0]**2 + center_x[1]**2)) + 0.001)),
                                    2*np.pi - np.arccos(min(1, 0.001)), 100)
            # 简化: 直接画圆弧
            t = np.linspace(0, 2*np.pi, 200)
            cx, cy = center_x
            x_arc = cx + radius_x * np.cos(t)
            y_arc = cy + radius_x * np.sin(t)
            # 只取在单位圆内的部分
            mask = x_arc**2 + y_arc**2 <= 1.02
            ax.plot(x_arc[mask], y_arc[mask], 'r-', linewidth=0.8, alpha=0.5)
            ax.text(cx + radius_x * 0.7, cy + radius_x * 0.3, f'x={x}',
                    fontsize=7, color='red', alpha=0.7)

            # 下半 (负电抗, 容性)
            y_arc2 = -y_arc + 2*(0)  # 镜像
            x_arc2 = cx + radius_x * np.cos(t)
            y_arc2 = -cy + radius_x * np.sin(t)
            mask2 = x_arc2**2 + y_arc2**2 <= 1.02
            ax.plot(x_arc2[mask2], y_arc2[mask2], 'r-', linewidth=0.8, alpha=0.5)
            if x <= 2:
                ax.text(cx + radius_x * 0.7, -cy - radius_x * 0.5, f'x=-{x}',
                        fontsize=7, color='red', alpha=0.7)

        # 轴标签
        ax.set_xlim(-1.15, 1.15)
        ax.set_ylim(-1.15, 1.15)
        ax.set_aspect('equal')
        ax.axhline(y=0, color='gray', linewidth=0.5)
        ax.set_xlabel('实部 (Γ)')
        ax.set_ylabel('虚部 (Γ)')
        ax.set_title(f'史密斯圆图 (Z₀={self.Z0}Ω)')
        ax.grid(True, alpha=0.2)

        return ax


# ==================== 匹配网络仿真 ====================
class MatchingNetwork:
    """阻抗匹配网络计算器"""

    def __init__(self, Z0=50.0):
        self.Z0 = Z0

    def L_match(self, Zload, f_hz, topology='highpass'):
        """
        L型匹配网络计算
        Zload: 负载阻抗 (复数)
        topology: 'highpass' 或 'lowpass'
        返回: L和C值
        """
        Z0 = self.Z0
        Rs = np.real(Zload)
        Xs = np.imag(Zload)

        # 匹配到Z0
        Q_match = np.sqrt(Z0 / Rs - 1) if Z0 > Rs else np.sqrt(Rs / Z0 - 1)
        omega = 2 * np.pi * f_hz

        if Z0 > Rs:
            # 负载电阻小于Z0
            Xp = Z0 / Q_match  # 并联电抗
            Xs_match = Q_match * Rs  # 串联电抗 (补偿负载电抗)

            if topology == 'highpass':
                # 并联L, 串联C
                C_series = 1 / (omega * abs(Xs_match - Xs))
                L_shunt = Xp / omega
                return {'L': L_shunt, 'C': C_series, 'Q': Q_match, 'topology': 'L-高通'}
            else:
                L_series = abs(Xs_match - Xs) / omega
                C_shunt = 1 / (omega * Xp)
                return {'L': L_series, 'C': C_shunt, 'Q': Q_match, 'topology': 'L-低通'}
        else:
            Xp = Rs / Q_match
            Xs_match = Q_match * Z0
            if topology == 'highpass':
                C_series = 1 / (omega * abs(Xs_match - Xs))
                L_shunt = Xp / omega
                return {'L': L_shunt, 'C': C_series, 'Q': Q_match, 'topology': 'L-高通'}
            else:
                L_series = abs(Xs_match - Xs) / omega
                C_shunt = 1 / (omega * Xp)
                return {'L': L_series, 'C': C_shunt, 'Q': Q_match, 'topology': 'L-低通'}

    def pi_match(self, Zload, f_hz, Q=10):
        """Π型匹配网络 (窄带, 高Q)"""
        omega = 2 * np.pi * f_hz
        Rs = np.real(Zload)

        # 两级L型匹配的级联
        R_intermediate = Rs * (1 + Q**2)
        R_intermediate = min(R_intermediate, self.Z0 * (1 + Q**2))

        # 并联元件
        Xp1 = R_intermediate / Q  # 输入并联
        Xp2 = self.Z0 / Q         # 输出并联

        # 串联元件
        Xs = Q * min(Rs, self.Z0)

        C1 = 1 / (omega * Xp1)
        L = Xs / omega
        C2 = 1 / (omega * Xp2)

        return {'C1': C1, 'L': L, 'C2': C2, 'Q': Q, 'topology': 'Π型'}


# ==================== 天线匹配仿真 ====================
class AntennaMatchingSim:
    """天线匹配综合仿真"""

    def __init__(self, params=None):
        p = params or {}
        self.Z0 = p.get('Z0', 50.0)
        self.freq_center = p.get('freq_center', 433e6)   # 中心频率
        self.bandwidth = p.get('bandwidth', 10e6)          # 带宽
        self.smith = SmithChart(self.Z0)
        self.matcher = MatchingNetwork(self.Z0)

    def antenna_impedance_model(self, freq):
        """
        天线阻抗随频率变化的简化模型
        模拟一个偶极子天线的阻抗
        """
        f0 = self.freq_center
        # 谐振频率处: Z ≈ 73 + j0 (自由空间偶极子)
        # 偏离谐振: 电阻和电抗都变化
        R_rad = 73.0 / (1 + ((freq - f0) / (self.bandwidth))**2)
        R_loss = 5.0  # 损耗电阻
        R = R_rad + R_loss

        # 电抗: 谐振时为0, 低频容性(-), 高频感性(+)
        Q_ant = f0 / self.bandwidth
        X = 2 * Q_ant * (freq - f0) / f0 * 50

        return R + 1j * X

    def matching_comparison(self, freq_range=None):
        """对比不同匹配方案的性能"""
        if freq_range is None:
            f0 = self.freq_center
            freq_range = np.linspace(f0 * 0.7, f0 * 1.3, 500)

        smith = self.smith

        fig, axes = plt.subplots(2, 2, figsize=(14, 11))
        fig.suptitle(f'天线匹配仿真 (f₀={self.freq_center/1e6:.0f}MHz, Z₀={self.Z0}Ω)', fontsize=14)

        # 计算未匹配天线阻抗
        Z_ant = np.array([self.antenna_impedance_model(f) for f in freq_range])
        swr_unmatched = smith.z_to_swr(Z_ant)

        # 匹配网络 (在中心频率处匹配)
        Z_load_center = self.antenna_impedance_model(self.freq_center)
        match_result = self.matcher.L_match(Z_load_center, self.freq_center, 'lowpass')

        # 1) SWR vs 频率
        ax = axes[0, 0]
        ax.plot(freq_range / 1e6, swr_unmatched, 'r-', linewidth=2, label='未匹配')

        # 模拟L型匹配后的SWR (简化: 只在中心频率附近匹配)
        swr_matched = []
        for f in freq_range:
            Z = self.antenna_impedance_model(f)
            # 简化匹配模型: 假设匹配网络在f0处完美匹配
            # 偏离f0时阻抗失配增加
            delta_f = (f - self.freq_center) / self.bandwidth
            match_factor = 1 + delta_f**2 * 2  # 简化频率响应
            gamma = np.abs(smith.z_to_gamma(Z)) / match_factor
            gamma = np.clip(gamma, 0, 0.999)
            swr_m = (1 + gamma) / (1 - gamma)
            swr_matched.append(swr_m)

        ax.plot(freq_range / 1e6, swr_matched, 'b-', linewidth=2, label='L型匹配后')

        # SWR=2线
        ax.axhline(y=2, color='green', linestyle='--', linewidth=1.5, alpha=0.7, label='SWR=2 (可接受)')
        ax.axhline(y=3, color='orange', linestyle=':', linewidth=1.5, alpha=0.7, label='SWR=3')
        ax.axvline(x=self.freq_center / 1e6, color='gray', linestyle=':', alpha=0.5)

        ax.set_xlabel('频率 (MHz)')
        ax.set_ylabel('SWR')
        ax.set_title('驻波比 vs 频率')
        ax.legend(fontsize=9)
        ax.grid(True, alpha=0.3)
        ax.set_ylim(1, 20)

        # 2) 反射系数 vs 频率
        ax = axes[0, 1]
        gamma_unmatched = np.array([np.abs(smith.z_to_gamma(z)) for z in Z_ant])
        gamma_matched = np.array([np.abs(smith.z_to_gamma(z)) / (1 + ((f - self.freq_center) / self.bandwidth)**2 * 2)
                                   for z, f in zip(Z_ant, freq_range)])
        gamma_matched = np.clip(gamma_matched, 0, 1)

        ax.plot(freq_range / 1e6, 20*np.log10(gamma_unmatched + 1e-10), 'r-', linewidth=2, label='未匹配')
        ax.plot(freq_range / 1e6, 20*np.log10(gamma_matched + 1e-10), 'b-', linewidth=2, label='L型匹配后')
        ax.axhline(y=-10, color='green', linestyle='--', alpha=0.7, label='-10dB (90%功率传输)')
        ax.axhline(y=-14, color='blue', linestyle=':', alpha=0.7, label='-14dB (96%功率传输)')

        ax.set_xlabel('频率 (MHz)')
        ax.set_ylabel('|S₁₁| (dB)')
        ax.set_title('回波损耗 vs 频率')
        ax.legend(fontsize=9)
        ax.grid(True, alpha=0.3)

        # 3) 史密斯圆图上的阻抗轨迹
        ax = axes[1, 0]
        smith.draw(ax)

        # 绘制阻抗轨迹
        gamma_unmatched_complex = np.array([smith.z_to_gamma(z) for z in Z_ant])
        ax.plot(np.real(gamma_unmatched_complex), np.imag(gamma_unmatched_complex),
                'r-', linewidth=2, label='天线阻抗轨迹')
        # 标注中心频率点
        gamma_center = smith.z_to_gamma(Z_load_center)
        ax.plot(np.real(gamma_center), np.imag(gamma_center), 'ro', markersize=8,
                label=f'f₀={self.freq_center/1e6:.0f}MHz')

        # 50Ω参考点
        ax.plot(0, 0, 'g+', markersize=15, markeredgewidth=2, label=f'Z₀={self.Z0}Ω')

        # 标注几个频率点
        for f_mhz in [self.freq_center*0.8, self.freq_center*0.9, self.freq_center,
                       self.freq_center*1.1, self.freq_center*1.2]:
            z = self.antenna_impedance_model(f_mhz)
            g = smith.z_to_gamma(z)
            ax.plot(np.real(g), np.imag(g), 'k.', markersize=6)
            ax.text(np.real(g)+0.02, np.imag(g)+0.02, f'{f_mhz/1e6:.0f}M', fontsize=7)

        ax.legend(fontsize=9, loc='upper left')
        ax.set_title('史密斯圆图 - 天线阻抗轨迹')

        # 4) 匹配网络元件值与效率
        ax = axes[1, 1]

        # 计算不同频率下的匹配效率
        efficiencies = []
        for f in freq_range:
            Z = self.antenna_impedance_model(f)
            gamma_val = np.abs(smith.z_to_gamma(Z))
            # 匹配后 (简化)
            delta_f = (f - self.freq_center) / self.bandwidth
            gamma_m = gamma_val / (1 + delta_f**2 * 2)
            gamma_m = np.clip(gamma_m, 0, 0.999)
            eff = (1 - gamma_m**2) * 100
            efficiencies.append(eff)

        ax.plot(freq_range / 1e6, efficiencies, 'g-', linewidth=2, label='匹配后功率传输效率')

        # 未匹配效率
        eff_unmatched = (1 - np.clip(gamma_unmatched, 0, 0.999)**2) * 100
        ax.plot(freq_range / 1e6, eff_unmatched, 'r--', linewidth=2, label='未匹配效率')

        ax.fill_between(freq_range / 1e6, efficiencies, alpha=0.1, color='green')
        ax.set_xlabel('频率 (MHz)')
        ax.set_ylabel('功率传输效率 (%)')
        ax.set_title('天线功率传输效率')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.set_ylim(0, 100)

        plt.tight_layout()
        plt.savefig('antenna_matching.png', dpi=150, bbox_inches='tight')
        plt.show()

        return match_result

    def swr_impact_analysis(self):
        """SWR对功率传输的影响分析"""
        fig, axes = plt.subplots(1, 2, figsize=(14, 5))
        fig.suptitle('SWR对系统性能的影响', fontsize=14)

        smith = self.smith

        # 1) 功率传输效率 vs SWR
        ax = axes[0]
        swr = np.linspace(1, 10, 200)
        gamma = (swr - 1) / (swr + 1)
        power_transmitted = (1 - gamma**2) * 100
        power_reflected = gamma**2 * 100

        ax.fill_between(swr, 0, power_transmitted, alpha=0.3, color='green', label='传输功率')
        ax.fill_between(swr, power_transmitted, 100, alpha=0.3, color='red', label='反射功率')
        ax.plot(swr, power_transmitted, 'g-', linewidth=2)

        # 标注常用SWR
        for swr_val in [1.5, 2.0, 3.0]:
            pt = (1 - ((swr_val-1)/(swr_val+1))**2) * 100
            ax.plot(swr_val, pt, 'ko', markersize=8)
            ax.annotate(f'SWR={swr_val}\n{pt:.1f}%传输',
                        xy=(swr_val, pt), xytext=(swr_val+0.5, pt-10),
                        arrowprops=dict(arrowstyle='->'), fontsize=9)

        ax.set_xlabel('SWR')
        ax.set_ylabel('功率 (%)')
        ax.set_title('功率传输 vs 驻波比')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.set_xlim(1, 10)

        # 2) 回波损耗转换表
        ax = axes[1]
        swr_values = [1.0, 1.1, 1.2, 1.3, 1.5, 2.0, 2.5, 3.0, 5.0, 10.0]
        rl_values = []
        for s in swr_values:
            g = (s-1)/(s+1)
            rl = -20*np.log10(g) if g > 0 else 60
            rl_values.append(rl)

        table_data = []
        for s, rl in zip(swr_values, rl_values):
            g = (s-1)/(s+1)
            pt = (1-g**2)*100
            table_data.append([f'{s:.1f}', f'{rl:.1f}', f'{g:.3f}', f'{pt:.1f}%'])

        ax.axis('off')
        table = ax.table(cellText=table_data,
                        colLabels=['SWR', '回波损耗(dB)', '反射系数|Γ|', '传输效率'],
                        loc='center', cellLoc='center')
        table.auto_set_font_size(False)
        table.set_fontsize(11)
        table.scale(1.2, 1.8)

        # 颜色编码
        for i, (s, rl) in enumerate(zip(swr_values, rl_values)):
            color = 'lightgreen' if s <= 1.5 else 'lightyellow' if s <= 2.0 else 'lightsalmon' if s <= 3.0 else 'lightcoral'
            for j in range(4):
                table[i+1, j].set_facecolor(color)

        ax.set_title('SWR参数转换速查表')

        plt.tight_layout()
        plt.savefig('swr_analysis.png', dpi=150, bbox_inches='tight')
        plt.show()


def demo():
    print("=" * 60)
    print("天线匹配仿真系统 - 阻抗匹配+SWR+史密斯圆图")
    print("=" * 60)

    # 常用电赛频率
    configs = [
        {'name': '433MHz LoRa', 'freq_center': 433e6, 'bandwidth': 10e6},
        {'name': '2.4GHz WiFi/BLE', 'freq_center': 2440e6, 'bandwidth': 80e6},
        {'name': '915MHz Sub-GHz', 'freq_center': 915e6, 'bandwidth': 26e6},
    ]

    for cfg in configs:
        print(f"\n=== {cfg['name']} 天线匹配 ===")
        sim = AntennaMatchingSim(cfg)
        Z_center = sim.antenna_impedance_model(cfg['freq_center'])
        swr = sim.smith.z_to_swr(Z_center)
        print(f"  天线阻抗@f₀: {Z_center:.1f}Ω")
        print(f"  SWR(未匹配): {swr:.2f}")

        match = sim.matcher.L_match(Z_center, cfg['freq_center'])
        print(f"  匹配方案: {match['topology']}")
        if 'L' in match:
            print(f"  L = {match['L']*1e9:.2f} nH")
        if 'C' in match:
            print(f"  C = {match['C']*1e12:.2f} pF")

    # 主仿真 (433MHz)
    sim = AntennaMatchingSim({'freq_center': 433e6, 'bandwidth': 10e6})
    sim.matching_comparison()
    sim.swr_impact_analysis()

    print("\n仿真完成！图表已保存。")


if __name__ == '__main__':
    demo()
