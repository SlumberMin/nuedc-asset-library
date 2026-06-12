"""
ADC动态性能仿真 - ADC Dynamic Performance Simulation
=====================================================
仿真内容: SFDR / SINAD / ENOB / 孔径抖动 / 量化噪声 / 谐波失真
适用场景: ADC选型评估、数据采集系统设计、信号链性能预测

电赛应用: 高速ADC选型、采集系统动态性能评估
"""

import numpy as np
from scipy import signal as sig
from scipy.special import erfc
import matplotlib.pyplot as plt
import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False


class ADCModel:
    """ADC行为级模型"""

    def __init__(self, n_bits, f_sample, full_scale=2.0, enob_spec=None):
        """
        参数:
            n_bits: ADC标称分辨率
            f_sample: 采样频率 (Hz)
            full_scale: 满量程电压 (Vpp)
            enob_spec: 标称ENOB (None则等于n_bits)
        """
        self.n_bits = n_bits
        self.f_sample = f_sample
        self.full_scale = full_scale
        self.n_levels = 2**n_bits
        self.lsb = full_scale / self.n_levels
        self.enob_spec = enob_spec if enob_spec is not None else n_bits

    def quantize(self, signal_in):
        """理想量化"""
        clipped = np.clip(signal_in, -self.full_scale / 2, self.full_scale / 2)
        return np.round(clipped / self.lsb) * self.lsb

    def add_dnl_inl(self, signal_quantized, dnl_std=0.3, inl_std=0.5):
        """添加DNL/INL误差"""
        code = np.round(signal_quantized / self.lsb + self.n_levels / 2).astype(int)
        code = np.clip(code, 0, self.n_levels - 1)

        # 生成DNL (每个码值不同)
        dnl = np.random.normal(0, dnl_std, self.n_levels) * self.lsb
        # INL是DNL的积分
        inl = np.cumsum(dnl) * inl_std / dnl_std

        return signal_quantized + inl[code]

    def add_aperture_jitter(self, t, signal_in, jitter_rms_s):
        """添加孔径抖动影响"""
        jitter = np.random.normal(0, jitter_rms_s, len(t))
        # 抖动等效于在信号导数上加噪声
        if len(signal_in) > 1:
            derivative = np.gradient(signal_in, t)
            return signal_in + jitter * derivative
        return signal_in

    def simulate_sampling(self, f_in, n_samples=65536, aperture_jitter_ps=0,
                          dnl_std=0.3, thd_percent=0.0, add_noise_uv=0):
        """
        完整ADC采样仿真

        参数:
            f_in: 输入信号频率 (Hz)
            n_samples: 采样点数
            aperture_jitter_ps: 孔径抖动 (ps RMS)
            dnl_std: DNL标准差 (LSB)
            thd_percent: 总谐波失真 (%)
            add_noise_uv: 附加热噪声 (μV RMS)
        """
        t = np.arange(n_samples) / self.f_sample
        amplitude = self.full_scale / 2 * 0.9  # -0.9dBFS

        # 理想信号
        signal_ideal = amplitude * np.sin(2 * np.pi * f_in * t)

        # 添加谐波失真 (模拟前端放大器非线性)
        if thd_percent > 0:
            thd_ratio = thd_percent / 100
            signal_ideal += amplitude * thd_ratio * np.sin(2 * 2 * np.pi * f_in * t)  # 2次谐波
            signal_ideal += amplitude * thd_ratio * 0.5 * np.sin(2 * 3 * np.pi * f_in * t)  # 3次

        # 添加热噪声
        if add_noise_uv > 0:
            noise = np.random.normal(0, add_noise_uv * 1e-6, n_samples)
            signal_ideal += noise

        # 孔径抖动
        if aperture_jitter_ps > 0:
            jitter_s = aperture_jitter_ps * 1e-12
            signal_jittered = self.add_aperture_jitter(t, signal_ideal, jitter_s)
        else:
            signal_jittered = signal_ideal

        # 量化
        signal_quantized = self.quantize(signal_jittered)

        # DNL/INL
        signal_final = self.add_dnl_inl(signal_quantized, dnl_std)

        return {
            't': t,
            'signal_ideal': signal_ideal,
            'signal_quantized': signal_quantized,
            'signal_final': signal_final,
            'quantization_error': signal_final - signal_ideal,
        }

    def analyze_dynamic(self, signal_out, f_in, n_fft=None, exclude_dc=True):
        """
        分析ADC动态性能指标

        返回: SFDR, SINAD, THD, ENOB, SNR, 各次谐波位置和幅度
        """
        if n_fft is None:
            n_fft = len(signal_out)

        # 加窗FFT
        window = sig.windows.hann(n_fft)
        spectrum = np.fft.rfft(signal_out[:n_fft] * window) / (n_fft / 2)
        freq = np.fft.rfftfreq(n_fft, 1.0 / self.f_sample)
        psd = np.abs(spectrum)

        # 窗函数校正
        coherent_gain = np.mean(window)
        psd_corrected = psd / coherent_gain

        # 找信号频率bin
        bin_resolution = self.f_sample / n_fft
        sig_bin = int(f_in / bin_resolution)
        sig_bin = min(sig_bin, len(psd_corrected) - 1)

        # 在信号bin附近搜索峰值
        search_range = max(3, int(100 / bin_resolution))
        sig_region = slice(max(0, sig_bin - search_range), min(len(psd_corrected), sig_bin + search_range + 1))
        actual_sig_bin = sig_bin - search_range + np.argmax(psd_corrected[sig_region])
        actual_sig_bin = int(actual_sig_bin)

        sig_power = psd_corrected[actual_sig_bin] ** 2

        # 谐波分析 (到10次)
        harmonics = []
        harmonic_power = 0
        margin = max(3, int(100 / bin_resolution))
        for h in range(2, 11):
            harm_bin = actual_sig_bin * h
            if harm_bin < len(psd_corrected) - margin:
                harm_region = psd_corrected[max(0, harm_bin - margin):harm_bin + margin + 1]
                harm_amp = np.max(harm_region)
                harmonics.append({
                    'order': h,
                    'bin': harm_bin,
                    'freq': freq[harm_bin],
                    'amplitude_dbc': 20 * np.log10(harm_amp / psd_corrected[actual_sig_bin] + 1e-20),
                })
                harmonic_power += harm_amp ** 2

        # THD
        thd = np.sqrt(harmonic_power / sig_power) * 100
        thd_db = 20 * np.log10(thd / 100) if thd > 0 else -200

        # 总噪声功率 (排除信号和谐波区域)
        noise_mask = np.ones(len(psd_corrected), dtype=bool)
        # 排除直流
        if exclude_dc:
            noise_mask[0:margin] = False
        # 排除信号
        noise_mask[max(0, actual_sig_bin - margin):actual_sig_bin + margin + 1] = False
        # 排除谐波
        for h in range(2, 11):
            hb = actual_sig_bin * h
            if hb < len(psd_corrected) - margin:
                noise_mask[max(0, hb - margin):hb + margin + 1] = False

        noise_power = np.sum(psd_corrected[noise_mask] ** 2)

        # SFDR
        max_spur = np.max(psd_corrected[noise_mask]) if np.any(noise_mask) else 1e-20
        sfdr_db = 20 * np.log10(psd_corrected[actual_sig_bin] / max_spur)

        # SINAD (信纳比)
        total_noise_distortion = noise_power + harmonic_power
        sinad_db = 10 * np.log10(sig_power / total_noise_distortion) if total_noise_distortion > 0 else 200

        # SNR (不含谐波)
        snr_db = 10 * np.log10(sig_power / noise_power) if noise_power > 0 else 200

        # ENOB
        enob = (sinad_db - 1.76) / 6.02

        return {
            'freq': freq,
            'psd_db': 20 * np.log10(psd_corrected + 1e-20),
            'sfdr_db': sfdr_db,
            'sinad_db': sinad_db,
            'snr_db': snr_db,
            'thd_percent': thd,
            'thd_db': thd_db,
            'enob': enob,
            'harmonics': harmonics,
            'sig_bin': actual_sig_bin,
            'sig_freq': freq[actual_sig_bin],
        }


def aperture_jitter_analysis(f_in_range, n_bits=12, jitter_ps_list=[0, 1, 5, 10, 50]):
    """分析孔径抖动对不同输入频率的影响"""
    results = {}
    for jitter in jitter_ps_list:
        enob_list = []
        sfdr_list = []
        for f_in in f_in_range:
            # 理论极限: 孔径抖动限制的SNR
            # SNR_jitter = -20*log10(2*pi*f_in*t_jitter)
            if jitter > 0:
                snr_jitter = -20 * np.log10(2 * np.pi * f_in * jitter * 1e-12)
                snr_total = -10 * np.log10(10**(-6.02 * n_bits / 10) + 10**(-snr_jitter / 10))
            else:
                snr_total = 6.02 * n_bits + 1.76
            enob = (snr_total - 1.76) / 6.02
            enob_list.append(max(enob, 0))
            sfdr_list.append(min(snr_total + 10, 6.02 * n_bits + 1.76))
        results[jitter] = {'enob': enob_list, 'sfdr': sfdr_list}
    return results


def plot_adc_dynamic_results():
    """综合绘图: ADC动态性能仿真"""
    fig, axes = plt.subplots(3, 2, figsize=(16, 14))

    f_sample = 100e6
    f_in = 10.1e6  # 非相干
    n_bits = 12
    n_samples = 65536

    # 1. 理想 vs 实际频谱
    adc = ADCModel(n_bits, f_sample)
    result = adc.simulate_sampling(f_in, n_samples, aperture_jitter_ps=5,
                                    dnl_std=0.3, thd_percent=0.1)
    analysis = adc.analyze_dynamic(result['signal_final'], f_in)

    freq_mhz = analysis['freq'] / 1e6
    axes[0, 0].plot(freq_mhz, analysis['psd_db'], 'b-', linewidth=0.5)
    axes[0, 0].set_xlabel('频率 (MHz)')
    axes[0, 0].set_ylabel('幅度 (dB)')
    axes[0, 0].set_title(f'ADC频谱 ({n_bits}bit, fs={f_sample/1e6}MHz, fin={f_in/1e6}MHz)')
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].set_xlim([0, f_sample / 2 / 1e6])

    # 标注指标
    info_text = (f'SFDR={analysis["sfdr_db"]:.1f}dB\n'
                 f'SINAD={analysis["sinad_db"]:.1f}dB\n'
                 f'ENOB={analysis["enob"]:.1f}bit\n'
                 f'THD={analysis["thd_percent"]:.2f}%')
    axes[0, 0].text(0.95, 0.95, info_text, transform=axes[0, 0].transAxes,
                    fontsize=10, verticalalignment='top', horizontalalignment='right',
                    bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

    # 2. 谐波分析
    if analysis['harmonics']:
        harm_orders = [h['order'] for h in analysis['harmonics']]
        harm_amps = [h['amplitude_dbc'] for h in analysis['harmonics']]
        axes[0, 1].bar(harm_orders, harm_amps, color='steelblue', alpha=0.8)
        axes[0, 1].set_xlabel('谐波次数')
        axes[0, 1].set_ylabel('幅度 (dBc)')
        axes[0, 1].set_title('谐波失真分析')
        axes[0, 1].grid(True, alpha=0.3)

    # 3. 孔径抖动 vs ENOB
    f_in_range = np.logspace(5, 8, 50)
    jitter_results = aperture_jitter_analysis(f_in_range, n_bits=n_bits)
    for jitter, data in jitter_results.items():
        axes[1, 0].semilogx(f_in_range / 1e6, data['enob'], linewidth=2,
                            label=f'jitter={jitter}ps')
    axes[1, 0].set_xlabel('输入频率 (MHz)')
    axes[1, 0].set_ylabel('ENOB (bit)')
    axes[1, 0].set_title(f'孔径抖动对ENOB的限制 ({n_bits}bit ADC)')
    axes[1, 0].grid(True, alpha=0.3)
    axes[1, 0].legend()
    axes[1, 0].axhline(y=n_bits, color='k', linestyle='--', alpha=0.3)

    # 4. 不同ADC分辨率对比
    for nb in [8, 10, 12, 14, 16]:
        snr_theory = 6.02 * nb + 1.76
        jitter_snr = []
        for f in f_in_range:
            if 5 > 0:
                snr_j = -20 * np.log10(2 * np.pi * f * 5e-12)
                snr_t = -10 * np.log10(10**(-snr_theory / 10) + 10**(-snr_j / 10))
            else:
                snr_t = snr_theory
            jitter_snr.append(snr_t)
        axes[1, 1].semilogx(f_in_range / 1e6, jitter_snr, linewidth=2, label=f'{nb}bit')
    axes[1, 1].set_xlabel('输入频率 (MHz)')
    axes[1, 1].set_ylabel('SNR (dB)')
    axes[1, 1].set_title('不同分辨率ADC的SNR vs 频率 (jitter=5ps)')
    axes[1, 1].grid(True, alpha=0.3)
    axes[1, 1].legend()

    # 5. DNL对SFDR的影响
    dnl_stds = [0, 0.1, 0.2, 0.3, 0.5, 1.0]
    sfdr_dnl = []
    for dnl in dnl_stds:
        adc_test = ADCModel(n_bits, f_sample)
        wf = adc_test.simulate_sampling(f_in, n_samples, dnl_std=dnl)
        an = adc_test.analyze_dynamic(wf['signal_final'], f_in)
        sfdr_dnl.append(an['sfdr_db'])
    axes[2, 0].plot(dnl_stds, sfdr_dnl, 'bo-', linewidth=2, markersize=8)
    axes[2, 0].set_xlabel('DNL标准差 (LSB)')
    axes[2, 0].set_ylabel('SFDR (dB)')
    axes[2, 0].set_title('DNL对SFDR的影响')
    axes[2, 0].grid(True, alpha=0.3)

    # 6. 量化噪声功率谱密度 (理论)
    # 理论量化噪声: S_q(f) = (LSB^2/12) / (f_s/2) = LSB^2 / (6*f_s)
    n_bits_list = [8, 10, 12, 14, 16]
    thermal_noise_uV = np.logspace(0, 4, 50)
    for nb in n_bits_list:
        lsb = 2.0 / 2**nb
        q_noise = lsb / np.sqrt(12)  # 量化噪声RMS
        total_noise = np.sqrt(q_noise**2 + (thermal_noise_uV * 1e-6)**2)
        snr = 20 * np.log10((2.0 / 2 / np.sqrt(2)) / total_noise)
        axes[2, 1].semilogx(thermal_noise_uV, snr, linewidth=2, label=f'{nb}bit')
    axes[2, 1].set_xlabel('热噪声 (μV RMS)')
    axes[2, 1].set_ylabel('SNR (dB)')
    axes[2, 1].set_title('量化噪声 vs 热噪声对SNR的影响')
    axes[2, 1].grid(True, alpha=0.3)
    axes[2, 1].legend()

    plt.tight_layout()
    save_path = './nuedc-asset-library/15_simulation/adc_dynamic_simulation_result.png'
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"[OK] 图表已保存: {save_path}")


def demo():
    """演示: ADC动态性能仿真"""
    print("=" * 60)
    print("ADC动态性能仿真 - ADC Dynamic Performance Simulation")
    print("=" * 60)

    f_sample = 100e6
    f_in = 10.1e6

    # 1. 不同分辨率ADC
    print(f"\n[1] 不同分辨率ADC动态性能 (fs={f_sample/1e6}MHz, fin={f_in/1e6}MHz):")
    for n_bits in [8, 10, 12, 14, 16]:
        adc = ADCModel(n_bits, f_sample)
        wf = adc.simulate_sampling(f_in, 65536)
        an = adc.analyze_dynamic(wf['signal_final'], f_in)
        print(f"  {n_bits}bit: SFDR={an['sfdr_db']:.1f}dB, SINAD={an['sinad_db']:.1f}dB, "
              f"ENOB={an['enob']:.1f}bit, THD={an['thd_percent']:.2f}%")

    # 2. 孔径抖动影响
    print(f"\n[2] 孔径抖动影响 ({12}bit, fs={f_sample/1e6}MHz):")
    for jitter in [0, 1, 5, 10, 20, 50]:
        adc = ADCModel(12, f_sample)
        wf = adc.simulate_sampling(f_in, 65536, aperture_jitter_ps=jitter)
        an = adc.analyze_dynamic(wf['signal_final'], f_in)
        print(f"  jitter={jitter:2d}ps: SFDR={an['sfdr_db']:.1f}dB, ENOB={an['enob']:.1f}bit")

    # 3. 不同输入频率
    print(f"\n[3] 输入频率扫描 (12bit, fs={f_sample/1e6}MHz, jitter=10ps):")
    adc = ADCModel(12, f_sample)
    for f_in_test in [1e6, 5e6, 10e6, 20e6, 40e6]:
        wf = adc.simulate_sampling(f_in_test, 65536, aperture_jitter_ps=10)
        an = adc.analyze_dynamic(wf['signal_final'], f_in_test)
        print(f"  fin={f_in_test/1e6:5.1f}MHz: SFDR={an['sfdr_db']:.1f}dB, "
              f"ENOB={an['enob']:.1f}bit")

    # 4. 绘图
    print("\n[4] 生成图表...")
    try:
        plot_adc_dynamic_results()
    except Exception as e:
        print(f"  绘图跳过: {e}")

    print("\n仿真完成!")


if __name__ == '__main__':
    demo()
