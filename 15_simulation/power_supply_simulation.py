#!/usr/bin/env python3
"""
电源仿真 - Buck/Boost/LDO效率 + 纹波 + 瞬态响应
适用于电赛电源模块选型与设计优化
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rcParams

rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial']
rcParams['axes.unicode_minus'] = False


# ==================== Buck变换器仿真 ====================
class BuckConverterSim:
    """同步/异步Buck变换器仿真"""

    def __init__(self, params=None):
        p = params or {}
        self.Vin = p.get('Vin', 12.0)
        self.Vout = p.get('Vout', 3.3)
        self.fsw = p.get('fsw', 500e3)       # 开关频率 500kHz
        self.L = p.get('L', 10e-6)            # 电感 10µH
        self.C = p.get('C', 22e-6)            # 输出电容 22µF
        self.Rdson = p.get('Rdson', 0.02)     # MOSFET导通电阻
        self.Rdiode = p.get('Rdiode', 0.05)   # 二极管等效电阻(异步)
        self.RL = p.get('RL', 0.01)           # 电感DCR
        self.RCesr = p.get('RCesr', 0.01)     # 电容ESR
        self.Iload = p.get('Iload', 2.0)      # 负载电流
        self.sync = p.get('sync', True)       # 同步整流

    @property
    def duty(self):
        return self.Vout / self.Vin

    def efficiency(self, Iload=None):
        """计算效率 vs 负载电流"""
        if Iload is None:
            Iload = np.linspace(0.01, 5, 100)
        D = self.duty

        # 导通损耗
        P_sw_high = Iload**2 * self.Rdson * D
        if self.sync:
            P_sw_low = Iload**2 * self.Rdson * (1 - D)
        else:
            P_sw_low = Iload**2 * self.Rdiode * (1 - D) + 0.3 * Iload * (1 - D)  # 含二极管压降
        P_inductor = Iload**2 * self.RL
        P_cond = P_sw_high + P_sw_low + P_inductor

        # 开关损耗 (简化)
        Qg = 10e-9  # 栅极电荷
        Vg = 5.0
        P_gate = Qg * Vg * self.fsw
        # 开关过渡损耗
        tr = 10e-9
        P_switch = 0.5 * self.Vin * Iload * tr * self.fsw * 2

        P_total = P_cond + P_gate + P_switch
        P_out = self.Vout * Iload
        eta = P_out / (P_out + P_total) * 100
        return eta, P_total

    def output_ripple(self):
        """输出纹波分析"""
        D = self.duty
        fsw = self.fsw
        # 电感纹波电流
        delta_IL = (self.Vin - self.Vout) * D / (self.L * fsw)

        # 输出纹波电压 (ESR + 容性)
        V_ripple_esr = delta_IL * self.RCesr
        V_ripple_cap = delta_IL / (8 * fsw * self.C)
        V_ripple_total = V_ripple_esr + V_ripple_cap

        return delta_IL, V_ripple_total, V_ripple_esr, V_ripple_cap

    def transient_response(self, Istep=1.0, dt=50e-6):
        """
        负载瞬态响应仿真
        Args:
            Istep: 负载阶跃幅度 (A)
            dt: 仿真时长 (s)
        """
        t = np.arange(0, dt, 1e-9)
        n = len(t)

        # 状态变量
        iL = np.zeros(n)  # 电感电流
        vC = np.zeros(n)  # 电容电压
        iL[0] = self.Iload

        # 负载电流阶跃 (在t=dt/4处跳变)
        I_load = np.ones(n) * self.Iload
        I_load[int(n/4):] = self.Iload + Istep

        D = self.duty
        fsw = self.fsw
        Tsw = 1 / fsw

        # 简化平均模型 (状态空间)
        # VL = Vin*D - Vout - iL*(RL + Rdson)
        # iC = iL - I_load
        dt_sim = t[1] - t[0]
        for k in range(n - 1):
            # 电感方程: L * diL/dt = Vin*D - vC - iL*R_parasitic
            VL = self.Vin * D - vC[k] - iL[k] * (self.RL + self.Rdson)
            iL[k+1] = iL[k] + VL / self.L * dt_sim

            # 电容方程: C * dVc/dt = iL - I_load
            iC = iL[k] - I_load[k]
            vC[k+1] = vC[k] + iC / self.C * dt_sim

        return t * 1e6, vC, iL, I_load

    def plot_all(self):
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        fig.suptitle(f'Buck变换器仿真 (Vin={self.Vin}V→Vout={self.Vout}V, fsw={self.fsw/1e3:.0f}kHz, '
                     f'{"同步" if self.sync else "异步"}整流)', fontsize=13)

        # 1) 效率曲线
        ax = axes[0, 0]
        Iload = np.linspace(0.05, 5, 100)
        eta_sync, _ = BuckConverterSim({**self.__dict__, 'sync': True}).efficiency(Iload)
        eta_async, _ = BuckConverterSim({**self.__dict__, 'sync': False}).efficiency(Iload)
        ax.plot(Iload, eta_sync, 'b-', linewidth=2, label='同步整流')
        ax.plot(Iload, eta_async, 'r--', linewidth=2, label='异步整流(肖特基)')
        ax.set_xlabel('负载电流 (A)')
        ax.set_ylabel('效率 (%)')
        ax.set_title('效率 vs 负载电流')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.set_ylim(80, 100)

        # 2) 纹波 vs 开关频率
        ax = axes[0, 1]
        fsws = np.array([100, 200, 500, 1000, 2000]) * 1e3
        ripples = []
        for f in fsws:
            sim = BuckConverterSim({**self.__dict__, 'fsw': f})
            _, Vr, _, _ = sim.output_ripple()
            ripples.append(Vr * 1000)
        ax.bar(fsws/1e3, ripples, width=150, color='steelblue', alpha=0.7)
        ax.set_xlabel('开关频率 (kHz)')
        ax.set_ylabel('输出纹波 (mV)')
        ax.set_title('输出纹波 vs 开关频率')
        ax.grid(True, alpha=0.3, axis='y')
        for f, r in zip(fsws/1e3, ripples):
            ax.text(f, r + 0.1, f'{r:.1f}mV', ha='center', fontsize=9)

        # 3) 瞬态响应
        ax = axes[1, 0]
        t, vC, iL, I_load = self.transient_response(Istep=1.0)
        ax.plot(t, vC, 'b-', linewidth=1.5, label='输出电压')
        ax.axhline(y=self.Vout, color='gray', linestyle=':', alpha=0.5)
        ax.set_xlabel('时间 (µs)')
        ax.set_ylabel('输出电压 (V)')
        ax.set_title('负载瞬态响应 (1A阶跃)')
        ax.legend()
        ax.grid(True, alpha=0.3)

        # 标注电压偏差
        v_deviation = np.max(np.abs(vC - self.Vout))
        ax.text(0.95, 0.05, f'电压偏差: ±{v_deviation*1000:.0f}mV',
                transform=ax.transAxes, ha='right', va='bottom',
                bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.5))

        # 4) 电感电流波形
        ax = axes[1, 1]
        t_zoom = t[:500]
        iL_zoom = iL[:500]
        ax.plot(t_zoom, iL_zoom, 'g-', linewidth=1.5, label='电感电流')
        ax.axhline(y=self.Iload, color='gray', linestyle=':', alpha=0.5, label='负载电流')
        ax.set_xlabel('时间 (µs)')
        ax.set_ylabel('电流 (A)')
        ax.set_title('电感电流纹波 (稳态)')
        ax.legend()
        ax.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig('buck_converter.png', dpi=150, bbox_inches='tight')
        plt.show()

        delta_IL, Vr, Vr_esr, Vr_cap = self.output_ripple()
        eta, P_loss = self.efficiency(np.array([self.Iload]))
        print(f"\n=== Buck变换器关键指标 ===")
        print(f"  占空比: {self.duty*100:.1f}%")
        print(f"  效率@{self.Iload}A: {eta[0]:.1f}%")
        print(f"  电感纹波电流: {delta_IL*1000:.1f}mA")
        print(f"  输出纹波电压: {Vr*1000:.1f}mV (ESR: {Vr_esr*1000:.1f}mV, 容性: {Vr_cap*1000:.1f}mV)")


# ==================== Boost变换器仿真 ====================
class BoostConverterSim:
    """Boost升压变换器仿真"""

    def __init__(self, params=None):
        p = params or {}
        self.Vin = p.get('Vin', 3.7)       # 锂电池电压
        self.Vout = p.get('Vout', 12.0)
        self.fsw = p.get('fsw', 400e3)
        self.L = p.get('L', 22e-6)
        self.C = p.get('C', 47e-6)
        self.Rdson = p.get('Rdson', 0.05)
        self.Rdiode = p.get('Rdiode', 0.3)
        self.RL = p.get('RL', 0.02)
        self.RCesr = p.get('RCesr', 0.02)
        self.Iload = p.get('Iload', 1.0)

    @property
    def duty(self):
        return 1 - self.Vin / self.Vout

    def efficiency(self, Iload=None):
        if Iload is None:
            Iload = np.linspace(0.01, 3, 100)
        D = self.duty
        # 平均电感电流
        I_L = Iload / (1 - D)

        # 导通损耗
        P_sw = I_L**2 * self.Rdson * D
        P_diode = I_L**2 * self.Rdiode * (1 - D) + 0.4 * I_L * (1 - D)
        P_L = I_L**2 * self.RL

        # 开关损耗
        P_gate = 8e-9 * 5 * self.fsw
        P_switch = 0.5 * self.Vout * I_L * 15e-9 * self.fsw

        P_total = P_sw + P_diode + P_L + P_gate + P_switch
        P_out = self.Vout * Iload
        eta = P_out / (P_out + P_total) * 100
        return eta, P_total

    def plot_efficiency(self):
        fig, ax = plt.subplots(figsize=(10, 6))
        fig.suptitle(f'Boost变换器效率 ({self.Vin}V → {self.Vout}V)', fontsize=14)

        Iload = np.linspace(0.05, 3, 100)
        eta, P_loss = self.efficiency(Iload)

        ax.plot(Iload, eta, 'b-', linewidth=2)
        ax.fill_between(Iload, eta, alpha=0.1)
        ax.set_xlabel('负载电流 (A)')
        ax.set_ylabel('效率 (%)')
        ax.set_title(f'效率曲线 (D={self.duty*100:.1f}%, fsw={self.fsw/1e3:.0f}kHz)')
        ax.grid(True, alpha=0.3)
        ax.set_ylim(70, 100)

        # 标注峰值效率
        peak_idx = np.argmax(eta)
        ax.annotate(f'峰值效率: {eta[peak_idx]:.1f}%\n@ Iload={Iload[peak_idx]:.2f}A',
                    xy=(Iload[peak_idx], eta[peak_idx]),
                    xytext=(Iload[peak_idx]+0.5, eta[peak_idx]-2),
                    arrowprops=dict(arrowstyle='->', color='red'),
                    fontsize=11, color='red')

        plt.tight_layout()
        plt.savefig('boost_converter.png', dpi=150, bbox_inches='tight')
        plt.show()


# ==================== LDO仿真 ====================
class LDOSim:
    """LDO线性稳压器仿真"""

    def __init__(self, params=None):
        p = params or {}
        self.Vin = p.get('Vin', 5.0)
        self.Vout = p.get('Vout', 3.3)
        self.Iq = p.get('Iq', 50e-6)       # 静态电流 50µA
        self.Iload_max = p.get('Iload_max', 1.0)
        self.Rdropout = p.get('Rdropout', 0.2)  # Dropout等效电阻
        self.PSRRea = p.get('PSRRea', 60)   # 电源抑制比 @1kHz (dB)

    @property
    def dropout(self):
        return self.Rdropout * self.Iload_max

    def efficiency(self, Iload=None):
        if Iload is None:
            Iload = np.linspace(0.001, self.Iload_max, 100)
        eta = (self.Vout * Iload) / (self.Vin * (Iload + self.Iq)) * 100
        return eta

    def output_noise(self, freq_range=(10, 1e6)):
        """输出噪声谱密度"""
        freqs = np.logspace(np.log10(freq_range[0]), np.log10(freq_range[1]), 200)
        # 典型LDO噪声谱: 低频1/f + 白噪声
        noise_floor = 10e-9  # 10nV/√Hz
        flicker_corner = 10e3
        noise = noise_floor * np.sqrt(1 + flicker_corner / freqs)
        return freqs, noise * 1e9  # nV/√Hz

    def psrr(self):
        """PSRR频率响应"""
        freqs = np.logspace(1, 7, 300)
        # 简化PSRR模型: 低频高抑制，高频下降
        psrr = self.PSRRea / np.sqrt(1 + (freqs / 1e4)**2)
        psrr = np.maximum(psrr, 0.5)  # 最小0.5dB
        return freqs, psrr

    def plot_all(self):
        fig, axes = plt.subplots(1, 3, figsize=(16, 5))
        fig.suptitle(f'LDO稳压器仿真 ({self.Vin}V → {self.Vout}V, Iq={self.Iq*1e6:.0f}µA)', fontsize=13)

        # 1) 效率对比 (LDO vs Buck)
        ax = axes[0]
        Iload = np.linspace(0.001, self.Iload_max, 100)
        eta_ldo = self.efficiency(Iload)
        eta_buck = BuckConverterSim({'Vin': self.Vin, 'Vout': self.Vout}).efficiency(Iload)[0]

        ax.plot(Iload*1000, eta_ldo, 'r-', linewidth=2, label='LDO')
        ax.plot(Iload*1000, eta_buck, 'b-', linewidth=2, label='Buck')
        ax.set_xlabel('负载电流 (mA)')
        ax.set_ylabel('效率 (%)')
        ax.set_title('LDO vs Buck 效率对比')
        ax.legend()
        ax.grid(True, alpha=0.3)

        # 标注LDO理论最大效率
        max_eta = self.Vout / self.Vin * 100
        ax.axhline(y=max_eta, color='r', linestyle=':', alpha=0.5)
        ax.text(Iload[-1]*1000*0.5, max_eta+1, f'LDO理论最大: {max_eta:.1f}%', fontsize=9, color='red')

        # 2) PSRR
        ax = axes[1]
        freqs, psrr_db = self.psrr()
        ax.semilogx(freqs, psrr_db, 'g-', linewidth=2)
        ax.set_xlabel('频率 (Hz)')
        ax.set_ylabel('PSRR (dB)')
        ax.set_title('电源抑制比 (PSRR)')
        ax.grid(True, which='both', alpha=0.3)
        ax.axhline(y=20, color='r', linestyle=':', alpha=0.5, label='20dB线')
        ax.legend()

        # 3) 输出噪声
        ax = axes[2]
        freqs, noise = self.output_noise()
        ax.loglog(freqs, noise, 'm-', linewidth=2)
        ax.set_xlabel('频率 (Hz)')
        ax.set_ylabel('噪声谱密度 (nV/√Hz)')
        ax.set_title('输出噪声谱密度')
        ax.grid(True, which='both', alpha=0.3)

        # RMS噪声积分
        total_noise_rms = np.sqrt(np.trapz((noise*1e-9)**2, freqs)) * 1e6  # µV
        ax.text(0.95, 0.95, f'RMS噪声: {total_noise_rms:.1f}µV',
                transform=ax.transAxes, ha='right', va='top',
                bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.5))

        plt.tight_layout()
        plt.savefig('ldo_analysis.png', dpi=150, bbox_inches='tight')
        plt.show()


def demo():
    print("=" * 60)
    print("电源仿真系统 - Buck/Boost/LDO综合分析")
    print("=" * 60)

    # 1) Buck
    print("\n[1/3] Buck变换器仿真...")
    buck = BuckConverterSim({'Vin': 12, 'Vout': 3.3, 'fsw': 500e3, 'Iload': 2.0})
    buck.plot_all()

    # 2) Boost
    print("\n[2/3] Boost变换器仿真...")
    boost = BoostConverterSim({'Vin': 3.7, 'Vout': 12, 'fsw': 400e3})
    boost.plot_efficiency()

    # 3) LDO
    print("\n[3/3] LDO稳压器仿真...")
    ldo = LDOSim({'Vin': 5, 'Vout': 3.3})
    ldo.plot_all()

    # 综合对比
    print("\n=== 电源拓扑选型指南 ===")
    print("  Buck:  降压场景首选，效率90%+，纹波可控")
    print("  Boost: 升压场景，锂电池→12V/5V")
    print("  LDO:   低噪声场景(MCU/ADC供电)，压差小效率高")
    print("  建议: 大压差用Buck+LDO级联(低噪声+高效率)")

    print("\n仿真完成！")


if __name__ == '__main__':
    demo()
