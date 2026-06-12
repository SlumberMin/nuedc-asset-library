"""
射频电路仿真模块 - RF Circuit Simulation
==========================================
功能: 混频器、滤波器、放大器、链路预算仿真
适用: 电赛射频前端设计与验证
"""

import numpy as np
from scipy import signal
from typing import Tuple, List, Optional, Dict
import warnings
warnings.filterwarnings('ignore')


# ─────────────────────────────────────────────
# 1. 混频器仿真
# ─────────────────────────────────────────────

class MixerSimulator:
    """混频器仿真"""
    
    def __init__(self, lo_frequency: float, lo_power_dBm: float = 10,
                 if_bandwidth: float = None):
        """
        Args:
            lo_frequency: 本振频率 (Hz)
            lo_power_dBm: 本振功率 (dBm)
            if_bandwidth: 中频带宽 (Hz)
        """
        self.f_lo = lo_frequency
        self.P_lo = lo_power_dBm
        self.BW_if = if_bandwidth
    
    def mixing_products(self, f_rf: float, harmonics: int = 3) -> Dict:
        """
        计算混频产物 m*f_LO ± n*f_IF
        
        Args:
            f_rf: RF信号频率
            harmonics: 考虑的谐波阶数
        Returns:
            混频产物字典
        """
        f_if = abs(f_rf - self.f_lo)
        
        products = {}
        for m in range(1, harmonics + 1):
            for n in range(0, harmonics + 1):
                # 和频与差频
                f_sum = m * self.f_lo + n * f_if
                f_diff = m * self.f_lo - n * f_if
                
                key_plus = f"{m}*fLO + {n}*fIF"
                key_minus = f"{m}*fLO - {n}*fIF"
                
                if f_sum > 0:
                    products[key_plus] = f_sum
                if f_diff > 0:
                    products[key_minus] = f_diff
        
        return products
    
    def convert_loss_dB(self, conversion_loss: float = 6.0) -> float:
        """变频损耗 (dB), 典型值6-8dB"""
        return conversion_loss
    
    def if_output(self, rf_power_dBm: float, conversion_loss_dB: float = 6.0) -> float:
        """IF输出功率"""
        return rf_power_dBm - conversion_loss_dB
    
    def image_rejection(self, image_rejection_dB: float = 30) -> float:
        """镜像抑制比"""
        return image_rejection_dB
    
    def noise_figure(self, conversion_loss_dB: float = 6.0,
                     nf_if_amp: float = 0) -> float:
        """
        混频器级联噪声系数
        混频器NF ≈ 变频损耗
        """
        return conversion_loss_dB + nf_if_amp
    
    def spur_analysis(self, f_rf: float, max_order: int = 5) -> Dict:
        """
        杂散分析
        
        Returns:
            杂散频率和相对电平
        """
        f_if = abs(f_rf - self.f_lo)
        spurs = {}
        
        for m in range(0, max_order + 1):
            for n in range(0, max_order + 1):
                if m == 0 and n == 0:
                    continue
                
                # 杂散频率
                spur_freq = abs(m * self.f_lo - n * f_rf)
                if spur_freq == 0:
                    continue
                
                # 简化的杂散电平估算
                spur_level = -20 * (m + n - 1)  # 随阶数衰减
                
                # 检查是否落在IF带宽内
                if self.BW_if and abs(spur_freq - f_if) < self.BW_if / 2:
                    spurs[f"{m}*fLO-{n}*rf @ {spur_freq/1e6:.1f}MHz"] = spur_level
        
        return spurs


class ImageRejectMixer(MixerSimulator):
    """镜像抑制混频器 (Hartley/Weaver架构)"""
    
    def __init__(self, lo_frequency: float, image_rejection_dB: float = 30):
        super().__init__(lo_frequency)
        self.irr = image_rejection_dB
    
    def rejection_ratio(self) -> float:
        """镜像抑制度"""
        return 10**(self.irr / 10)
    
    def output_with_image(self, rf_power_dBm: float, image_power_dBm: float,
                          conversion_loss_dB: float = 7.0) -> Tuple[float, float]:
        """
        计算含镜像信号的输出
        
        Returns:
            (IF信号功率, 镜像泄漏功率) dBm
        """
        P_if = rf_power_dBm - conversion_loss_dB
        P_image_leak = image_power_dBm - conversion_loss_dB - self.irr
        return P_if, P_image_leak


# ─────────────────────────────────────────────
# 2. 射频滤波器仿真
# ─────────────────────────────────────────────

class RFFilter:
    """射频滤波器仿真"""
    
    def __init__(self, filter_type: str, order: int, cutoff_freq: float,
                 bandwidth: float = None, ripple_dB: float = 0.5):
        """
        Args:
            filter_type: 'lowpass', 'highpass', 'bandpass', 'bandstop'
            order: 滤波器阶数
            cutoff_freq: 截止频率 (Hz)
            bandwidth: 带宽 (Hz), bandpass/bandstop用
            ripple_dB: 通带纹波 (dB), Chebyshev用
        """
        self.type = filter_type
        self.order = order
        self.fc = cutoff_freq
        self.bw = bandwidth
        self.ripple = ripple_dB
    
    def butterworth(self, freq_array: np.ndarray, fs: float = None) -> np.ndarray:
        """
        巴特沃斯滤波器频率响应
        
        Args:
            freq_array: 频率数组 (Hz)
            fs: 采样率 (Hz), 用于数字滤波器
        Returns:
            频率响应幅度 (dB)
        """
        if fs:
            # 数字滤波器
            wn = self.fc / (fs / 2)
            if self.type == 'bandpass' and self.bw:
                f_low = (self.fc - self.bw/2) / (fs/2)
                f_high = (self.fc + self.bw/2) / (fs/2)
                b, a = signal.butter(self.order, [f_low, f_high], btype='band')
            elif self.type == 'bandstop' and self.bw:
                f_low = (self.fc - self.bw/2) / (fs/2)
                f_high = (self.fc + self.bw/2) / (fs/2)
                b, a = signal.butter(self.order, [f_low, f_high], btype='bandstop')
            else:
                b, a = signal.butter(self.order, wn, btype=self.type)
            
            w, h = signal.freqz(b, a, worN=freq_array, fs=fs)
            return 20 * np.log10(np.abs(h) + 1e-10)
        else:
            # 模拟滤波器
            w, h = signal.freqs(*signal.butter(self.order, self.fc, btype=self.type, analog=True),
                               worN=2*np.pi*freq_array)
            return 20 * np.log10(np.abs(h) + 1e-10)
    
    def chebyshev(self, freq_array: np.ndarray, fs: float = None,
                  type1: bool = True) -> np.ndarray:
        """切比雪夫滤波器频率响应"""
        if fs:
            wn = self.fc / (fs / 2)
            if type1:
                b, a = signal.cheby1(self.order, self.ripple, wn, btype=self.type)
            else:
                b, a = signal.cheby2(self.order, 40, wn, btype=self.type)
            w, h = signal.freqz(b, a, worN=freq_array, fs=fs)
        else:
            if type1:
                b, a = signal.cheby1(self.order, self.ripple, self.fc, btype=self.type, analog=True)
            else:
                b, a = signal.cheby2(self.order, 40, self.fc, btype=self.type, analog=True)
            w, h = signal.freqs(b, a, worN=2*np.pi*freq_array)
        
        return 20 * np.log10(np.abs(h) + 1e-10)
    
    def elliptic(self, freq_array: np.ndarray, fs: float = None,
                 rp: float = 0.5, rs: float = 40) -> np.ndarray:
        """椭圆滤波器"""
        wn = self.fc / (fs / 2) if fs else self.fc
        if fs:
            b, a = signal.ellip(self.order, rp, rs, wn, btype=self.type)
            w, h = signal.freqz(b, a, worN=freq_array, fs=fs)
        else:
            b, a = signal.ellip(self.order, rp, rs, wn, btype=self.type, analog=True)
            w, h = signal.freqs(b, a, worN=2*np.pi*freq_array)
        
        return 20 * np.log10(np.abs(h) + 1e-10)
    
    def group_delay(self, freq_array: np.ndarray, fs: float) -> np.ndarray:
        """群延迟"""
        b, a = signal.butter(self.order, self.fc / (fs/2), btype=self.type)
        w, gd = signal.group_delay((b, a), w=freq_array, fs=fs)
        return gd
    
    def insertion_loss(self, freq_array: np.ndarray, fs: float) -> np.ndarray:
        """插入损耗 (假设理想滤波器, 插损=0dB)"""
        H = self.butterworth(freq_array, fs)
        return -H  # 插损为负的频率响应


class LCFilter:
    """LC集总参数滤波器设计"""
    
    @staticmethod
    def butterworth_lpf(order: int, fc: float, Z0: float = 50) -> Dict:
        """
        巴特沃斯低通滤波器元件值
        
        Args:
            order: 阶数
            fc: 截止频率
            Z0: 特性阻抗
        Returns:
            元件值字典
        """
        omega_c = 2 * np.pi * fc
        
        # Butterworth原型元件值
        g = []
        for k in range(1, order + 1):
            gk = 2 * np.sin((2*k - 1) * np.pi / (2 * order))
            g.append(gk)
        
        # 阻抗变换
        components = []
        for k in range(order):
            if k % 2 == 0:
                # 电感
                L = g[k] * Z0 / omega_c
                components.append({'type': 'L', 'value_H': L, 'value_nH': L*1e9})
            else:
                # 电容
                C = g[k] / (Z0 * omega_c)
                components.append({'type': 'C', 'value_F': C, 'value_pF': C*1e12})
        
        return {
            'topology': 'T型' if order % 2 == 1 else 'π型',
            'order': order,
            'fc': fc,
            'Z0': Z0,
            'components': components
        }
    
    @staticmethod
    def chebyshev_lpf(order: int, fc: float, ripple_dB: float,
                      Z0: float = 50) -> Dict:
        """切比雪夫低通滤波器元件值"""
        omega_c = 2 * np.pi * fc
        epsilon = np.sqrt(10**(ripple_dB/10) - 1)
        
        g = []
        beta = np.log(1/np.tanh(ripple_dB / 17.37))
        gamma = np.sinh(beta / (2 * order))
        
        for k in range(1, order + 1):
            ak = np.sin((2*k - 1) * np.pi / (2 * order))
            bk = gamma**2 + np.sin(k * np.pi / order)**2
            
            if k == 1:
                gk = 2 * ak / gamma
            else:
                gk_prev = g[-1]
                gk = 4 * ak * np.sin((2*(k-1)-1)*np.pi/(2*order)) / (bk * g[-2]) if len(g) >= 2 else 2*ak/gamma
                gk = 4 * ak * np.sin((2*k-3)*np.pi/(2*order)) / (bk * (g[-1] if g else 1))
            
            g.append(gk)
        
        components = []
        for k in range(order):
            if k % 2 == 0:
                L = g[k] * Z0 / omega_c
                components.append({'type': 'L', 'value_H': L, 'value_nH': L*1e9})
            else:
                C = g[k] / (Z0 * omega_c)
                components.append({'type': 'C', 'value_F': C, 'value_pF': C*1e12})
        
        return {
            'topology': '切比雪夫',
            'order': order,
            'fc': fc,
            'ripple_dB': ripple_dB,
            'Z0': Z0,
            'components': components
        }


class MicrostripFilter:
    """微带滤波器设计辅助"""
    
    @staticmethod
    def hairpin_filter(order: int, fc: float, fractional_bw: float,
                       er: float = 4.4, h: float = 1.6e-3) -> Dict:
        """
        发夹型微带滤波器设计参数
        
        Args:
            order: 阶数
            fc: 中心频率
            fractional_bw: 相对带宽
            er: 介电常数
            h: 基板厚度
        Returns:
            设计参数
        """
        c = 3e8
        wl = c / fc
        
        # 有效介电常数 (近似)
        er_eff = (er + 1) / 2 + (er - 1) / 2 * (1 + 12*h/(wl*100))**(-0.5)
        
        # 相速度
        v_p = c / np.sqrt(er_eff)
        
        # 半波长谐振器长度
        L_resonator = v_p / (2 * fc) * 1000  # mm
        
        # 耦合系数 (简化)
        J_inverters = []
        for k in range(order):
            J = fractional_bw * np.pi / 2  # 简化
            J_inverters.append(J)
        
        return {
            'type': '发夹型微带滤波器',
            'order': order,
            'center_freq_GHz': fc / 1e9,
            'fractional_bw': fractional_bw,
            'er_eff': er_eff,
            'resonator_length_mm': L_resonator,
            'substrate_er': er,
            'substrate_height_mm': h * 1000,
            'num_resonators': order
        }


# ─────────────────────────────────────────────
# 3. 射频放大器仿真
# ─────────────────────────────────────────────

class RFAmplifier:
    """射频放大器仿真"""
    
    def __init__(self, gain_dB: float, nf_dB: float, p1dB_dBm: float = None,
                 ip3_dBm: float = None, bandwidth_Hz: float = None):
        """
        Args:
            gain_dB: 小信号增益 (dB)
            nf_dB: 噪声系数 (dB)
            p1dB_dBm: 1dB压缩点 (dBm)
            ip3_dBm: 三阶截点 (dBm)
            bandwidth_Hz: 工作带宽
        """
        self.gain_dB = gain_dB
        self.nf_dB = nf_dB
        self.p1dB = p1dB_dBm
        self.ip3 = ip3_dBm
        self.bw = bandwidth_Hz
    
    def output_power(self, input_power_dBm: float) -> float:
        """输出功率 (考虑压缩)"""
        P_linear = input_power_dBm + self.gain_dB
        
        if self.p1dB is None:
            return P_linear
        
        # Rapp模型简化: 1dB压缩点附近
        P_sat = self.p1dB + 1  # 饱和功率近似
        if P_linear < self.p1dB - 10:
            return P_linear
        
        # 简化的AM-AM压缩
        backoff = P_linear - P_sat
        compression = 1 / (1 + 10**(backoff/10))
        return P_sat + 10 * np.log10(compression) if compression > 0 else P_sat
    
    def noise_figure_linear(self) -> float:
        """线性噪声系数"""
        return 10**(self.nf_dB / 10)
    
    def output_noise_power(self, temperature_K: float = 290) -> float:
        """
        输出噪声功率密度 (dBm/Hz)
        N_out = k*T*G*F
        """
        k_B = 1.38e-23  # 玻尔兹曼常数
        NF_linear = self.noise_figure_linear()
        G_linear = 10**(self.gain_dB / 10)
        
        N_out = k_B * temperature_K * G_linear * NF_linear  # W/Hz
        N_out_dBm_Hz = 10 * np.log10(N_out * 1000)  # dBm/Hz
        return N_out_dBm_Hz
    
    def output_noise_total(self, temperature_K: float = 290) -> float:
        """带宽内总输出噪声功率 (dBm)"""
        N_density = self.output_noise_power(temperature_K)
        if self.bw:
            return N_density + 10 * np.log10(self.bw)
        return N_density
    
    def ip3_analysis(self, f1: float, f2: float, tone_spacing: float) -> Dict:
        """
        双音三阶互调分析
        
        Args:
            f1, f2: 双音频率
            tone_spacing: 音频间隔
        Returns:
            互调产物信息
        """
        # 三阶互调产物: 2f1-f2, 2f2-f1
        im3_1 = 2 * f1 - f2
        im3_2 = 2 * f2 - f1
        
        return {
            'input_tones': [f1, f2],
            'IM3_products': [im3_1, im3_2],
            'IM3_frequency_offset': tone_spacing,
            'OIP3_dBm': self.ip3,
            'IIP3_dBm': self.ip3 - self.gain_dB if self.ip3 else None,
            'SFDR_dB': (2/3 * (self.ip3 - (-174 + self.nf_dB))) if self.ip3 and self.bw else None
        }
    
    def stability_factor(self, S11: complex, S12: complex,
                         S21: complex, S22: complex) -> Tuple[float, float]:
        """
        计算放大器稳定性因子 (Rollett)
        
        Returns:
            (K, |Δ|): K>1且|Δ|<1时无条件稳定
        """
        delta = S11 * S22 - S12 * S21
        K = (1 - abs(S11)**2 - abs(S22)**2 + abs(delta)**2) / (2 * abs(S12 * S21))
        return K, abs(delta)


class LNAAmplifier(RFAmplifier):
    """低噪声放大器 (LNA)"""
    
    def __init__(self, gain_dB: float, nf_dB: float,
                 p1dB_dBm: float = -10, ip3_dBm: float = 0):
        super().__init__(gain_dB, nf_dB, p1dB_dBm, ip3_dBm)
        if nf_dB > 3:
            warnings.warn(f"LNA噪声系数{nf_dB}dB偏高, 典型值<1.5dB")
    
    def sensitivity(self, snr_required_dB: float, bandwidth_Hz: float) -> float:
        """
        灵敏度计算
        P_sens = kTB + NF + SNR_min
        """
        kTB_dBm = -174 + 10 * np.log10(bandwidth_Hz)
        return kTB_dBm + self.nf_dB + snr_required_dB
    
    def cascade_analysis(self, next_stage: 'RFAmplifier') -> Dict:
        """
        两级级联分析 (Friis公式)
        """
        G1 = 10**(self.gain_dB / 10)
        F1 = 10**(self.nf_dB / 10)
        G2 = 10**(next_stage.gain_dB / 10)
        F2 = 10**(next_stage.nf_dB / 10)
        
        # 级联噪声系数
        F_cascade = F1 + (F2 - 1) / G1
        NF_cascade = 10 * np.log10(F_cascade)
        
        # 级联增益
        G_cascade_dB = self.gain_dB + next_stage.gain_dB
        
        # 级联IP3
        if self.ip3 and next_stage.ip3:
            P_ip3_1_mW = 10**(self.ip3 / 10)
            P_ip3_2_mW = 10**(next_stage.ip3 / 10)
            # 1/OIP3_cascade = 1/OIP3_1 * G2 + 1/OIP3_2
            P_cascade = 1 / (1/P_ip3_1_m_W / G2 + 1/P_ip3_2_mW) if False else None
            # 修正
            oip3_cascade = 10 * np.log10(
                1 / (10**(-self.ip3/10) * G2 + 10**(-next_stage.ip3/10))
            ) if (self.ip3 and next_stage.ip3) else None
        else:
            oip3_cascade = None
        
        return {
            'gain_dB': G_cascade_dB,
            'NF_dB': NF_cascade,
            'OIP3_dBm': oip3_cascade
        }


# ─────────────────────────────────────────────
# 4. 链路预算仿真
# ─────────────────────────────────────────────

class RFLinkBudget:
    """射频链路预算"""
    
    def __init__(self):
        self.components = []
        self.kT = -174  # dBm/Hz @ 290K
    
    def add_component(self, name: str, gain_dB: float, nf_dB: float = 0,
                      p1dB_dBm: float = None, ip3_dBm: float = None):
        """添加链路组件"""
        self.components.append({
            'name': name,
            'gain_dB': gain_dB,
            'nf_dB': nf_dB,
            'p1dB': p1dB_dBm,
            'ip3': ip3_dBm
        })
    
    def cascade_gain(self) -> float:
        """总增益"""
        return sum(c['gain_dB'] for c in self.components)
    
    def cascade_noise_figure(self) -> float:
        """级联噪声系数 (Friis公式)"""
        F_total = 0
        G_product = 1
        
        for i, c in enumerate(self.components):
            F_i = 10**(c['nf_dB'] / 10)
            if i == 0:
                F_total = F_i
            else:
                F_total += (F_i - 1) / G_product
            G_product *= 10**(c['gain_dB'] / 10)
        
        return 10 * np.log10(F_total)
    
    def sensitivity(self, snr_dB: float, bandwidth_Hz: float) -> float:
        """系统灵敏度"""
        return self.kT + 10 * np.log10(bandwidth_Hz) + self.cascade_noise_figure() + snr_dB
    
    def dynamic_range_sfdr(self, bandwidth_Hz: float) -> float:
        """
        无杂散动态范围 (SFDR)
        SFDR = 2/3 * (OIP3 - kTB - NF)
        """
        # 找到系统OIP3
        oip3 = self._cascade_oip3()
        if oip3 is None:
            return float('inf')
        
        noise_floor = self.kT + 10 * np.log10(bandwidth_Hz) + self.cascade_noise_figure()
        return 2 / 3 * (oip3 - noise_floor)
    
    def _cascade_oip3(self) -> Optional[float]:
        """级联OIP3"""
        oip3_mW_total = 0
        G_product = 1
        
        for i, c in enumerate(reversed(self.components)):
            if c['ip3'] is None:
                return None
            
            p_mW = 10**(c['ip3'] / 10)
            if i == 0:
                oip3_mW_total = p_mW
            else:
                # 1/OIP3 = 1/OIP3_stage + G_stage/OIP3_prev
                oip3_mW_total = 1 / (1/p_mW + G_product/oip3_mW_total) if (p_mW > 0 and oip3_mW_total > 0) else p_mW
            G_product *= 10**(c['gain_dB'] / 10)
        
        return 10 * np.log10(oip3_mW_total) if oip3_mW_total > 0 else None
    
    def link_analysis(self, input_power_dBm: float, bandwidth_Hz: float,
                      snr_required_dB: float) -> Dict:
        """完整链路分析"""
        output_power = input_power_dBm + self.cascade_gain()
        nf = self.cascade_noise_figure()
        sensitivity = self.sensitivity(snr_required_dB, bandwidth_Hz)
        noise_floor = self.kT + 10 * np.log10(bandwidth_Hz) + nf
        snr_actual = output_power - noise_floor
        
        return {
            'input_power_dBm': input_power_dBm,
            'output_power_dBm': output_power,
            'total_gain_dB': self.cascade_gain(),
            'cascade_NF_dB': nf,
            'noise_floor_dBm': noise_floor,
            'sensitivity_dBm': sensitivity,
            'SNR_dB': snr_actual,
            'SNR_margin_dB': snr_actual - snr_required_dB,
            'num_components': len(self.components)
        }
    
    def print_report(self, input_power_dBm: float = -80,
                     bandwidth_Hz: float = 10e6, snr_dB: float = 10):
        """打印链路预算报告"""
        result = self.link_analysis(input_power_dBm, bandwidth_Hz, snr_dB)
        
        print("=" * 60)
        print("  射频链路预算报告")
        print("=" * 60)
        print(f"  {'组件':<20} {'增益(dB)':<10} {'NF(dB)':<10} {'P1dB(dBm)':<10}")
        print("  " + "-" * 50)
        
        gain_accum = 0
        for c in self.components:
            gain_accum += c['gain_dB']
            p1db_str = f"{c['p1dB']:.1f}" if c['p1dB'] else "N/A"
            print(f"  {c['name']:<20} {c['gain_dB']:<10.1f} {c['nf_dB']:<10.1f} {p1db_str:<10}")
        
        print("  " + "-" * 50)
        print(f"  总增益: {result['total_gain_dB']:.1f} dB")
        print(f"  级联NF: {result['cascade_NF_dB']:.1f} dB")
        print(f"  噪底: {result['noise_floor_dBm']:.1f} dBm")
        print(f"  灵敏度: {result['sensitivity_dBm']:.1f} dBm")
        print(f"  输入: {result['input_power_dBm']:.1f} dBm")
        print(f"  输出: {result['output_power_dBm']:.1f} dBm")
        print(f"  SNR: {result['SNR_dB']:.1f} dB (余量: {result['SNR_margin_dB']:.1f} dB)")


# ─────────────────────────────────────────────
# 综合演示
# ─────────────────────────────────────────────

def demo_mixer():
    """混频器演示"""
    print("=" * 60)
    print("  混频器仿真演示")
    print("=" * 60)
    
    mixer = MixerSimulator(lo_frequency=2.0e9, lo_power_dBm=10, if_bandwidth=10e6)
    
    f_rf = 2.4e9
    products = mixer.mixing_products(f_rf, harmonics=3)
    
    print(f"  RF={f_rf/1e9}GHz, LO={mixer.f_lo/1e9}GHz, IF={abs(f_rf-mixer.f_lo)/1e6}MHz")
    print(f"\n  主要混频产物:")
    for name, freq in sorted(products.items(), key=lambda x: x[1])[:10]:
        print(f"    {name} = {freq/1e6:.1f} MHz")
    
    # 杂散分析
    spurs = mixer.spur_analysis(f_rf)
    print(f"\n  落入IF带宽的杂散:")
    for spur, level in spurs.items():
        print(f"    {spur}: {level} dBc")


def demo_filter():
    """滤波器演示"""
    print("\n" + "=" * 60)
    print("  射频滤波器仿真演示")
    print("=" * 60)
    
    # 5阶巴特沃斯低通
    lpf = RFFilter('lowpass', order=5, cutoff_freq=100e6)
    freqs = np.logspace(6, 9, 1000)
    H_butter = lpf.butterworth(freqs)
    
    print(f"  5阶巴特沃斯低通 (fc=100MHz):")
    # 找-3dB点
    idx_3dB = np.argmin(np.abs(H_butter + 3))
    print(f"    -3dB频率: {freqs[idx_3dB]/1e6:.1f} MHz")
    print(f"    100MHz处衰减: {H_butter[np.argmin(np.abs(freqs-100e6))]:.2f} dB")
    print(f"    1GHz处衰减: {H_butter[np.argmin(np.abs(freqs-1e9))]:.2f} dB")
    
    # LC滤波器设计
    lc = LCFilter.butterworth_lpf(order=5, fc=100e6, Z0=50)
    print(f"\n  LC低通滤波器设计:")
    for comp in lc['components']:
        if comp['type'] == 'L':
            print(f"    电感: {comp['value_nH']:.2f} nH")
        else:
            print(f"    电容: {comp['value_pF']:.2f} pF")


def demo_amplifier():
    """放大器演示"""
    print("\n" + "=" * 60)
    print("  射频放大器仿真演示")
    print("=" * 60)
    
    lna = LNAAmplifier(gain_dB=20, nf_dB=1.0, p1dB_dBm=-10, ip3_dBm=0)
    
    print(f"  LNA参数:")
    print(f"    增益: {lna.gain_dB} dB")
    print(f"    NF: {lna.nf_dB} dB")
    print(f"    P1dB: {lna.p1dB} dBm")
    print(f"    OIP3: {lna.ip3} dBm")
    print(f"    灵敏度 (10MHz BW, SNR=10dB): {lna.sensitivity(10, 10e6):.1f} dBm")
    print(f"    输出噪底: {lna.output_noise_total():.1f} dBm")
    
    # 双音互调
    ip3_info = lna.ip3_analysis(1e9, 1.01e9, 10e6)
    print(f"\n  双音互调分析:")
    print(f"    输入音: {ip3_info['input_tones'][0]/1e9:.2f}GHz, "
          f"{ip3_info['input_tones'][1]/1e9:.2f}GHz")
    print(f"    IM3产物: {ip3_info['IM3_products'][0]/1e9:.4f}GHz, "
          f"{ip3_info['IM3_products'][1]/1e9:.4f}GHz")
    print(f"    IIP3: {ip3_info['IIP3_dBm']:.1f} dBm")


def demo_link_budget():
    """链路预算演示"""
    print("\n" + "=" * 60)
    print("  链路预算演示")
    print("=" * 60)
    
    link = RFLinkBudget()
    link.add_component("LNA", gain_dB=20, nf_dB=1.0, p1dB_dBm=-10, ip3_dBm=0)
    link.add_component("SAW滤波器", gain_dB=-3, nf_dB=3.0)
    link.add_component("混频器", gain_dB=-7, nf_dB=7.0, ip3_dBm=10)
    link.add_component("IF滤波器", gain_dB=-2, nf_dB=2.0)
    link.add_component("IF放大器", gain_dB=30, nf_dB=3.0, p1dB_dBm=10, ip3_dBm=25)
    
    link.print_report(input_power_dBm=-90, bandwidth_Hz=10e6, snr_dB=10)


if __name__ == "__main__":
    demo_mixer()
    demo_filter()
    demo_amplifier()
    demo_link_budget()
    print("\n✓ 射频电路仿真演示完成")
