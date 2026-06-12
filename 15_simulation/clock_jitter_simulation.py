"""
时钟抖动仿真 - Clock Jitter Simulation
========================================
仿真内容: 相位噪声 / 时钟抖动 / BER影响
适用场景: 高速数据通信、ADC/DAC采样时钟评估、SerDes时钟质量分析

电赛应用: 高速数据采集系统的时钟选型与评估
"""

import numpy as np
from scipy import signal
import matplotlib.pyplot as plt
import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False


class PhaseNoiseModel:
    """相位噪声模型"""

    def __init__(self, f_offset, L_df):
        """
        参数:
            f_offset: 偏离载波频率数组 (Hz)
            L_df: 对应的相位噪声功率谱密度 (dBc/Hz)
        """
        self.f_offset = np.array(f_offset)
        self.L_df = np.array(L_df)
        self.L_df_linear = 10 ** (np.array(L_df) / 10)

    @staticmethod
    def typical_crystal_oscillator():
        """典型晶体振荡器相位噪声模型"""
        f_offset = [1, 10, 100, 1e3, 10e3, 100e3, 1e6, 10e6]
        L_df = [-80, -110, -130, -145, -155, -158, -160, -160]
        return PhaseNoiseModel(f_offset, L_df)

    @staticmethod
    def typical_pll_output():
        """典型PLL输出相位噪声模型"""
        f_offset = [100, 1e3, 10e3, 100e3, 1e6, 10e6, 100e6]
        L_df = [-60, -80, -95, -110, -120, -130, -140]
        return PhaseNoiseModel(f_offset, L_df)

    @staticmethod
    def typical_synth_si5351():
        """Si5351等廉价时钟合成器相位噪声"""
        f_offset = [100, 1e3, 10e3, 100e3, 1e6, 10e6]
        L_df = [-50, -70, -90, -105, -115, -120]
        return PhaseNoiseModel(f_offset, L_df)


class ClockJitterAnalyzer:
    """时钟抖动分析器"""

    def __init__(self, f_clk, pn_model=None):
        """
        参数:
            f_clk: 时钟频率 (Hz)
            pn_model: PhaseNoiseModel实例
        """
        self.f_clk = f_clk
        self.pn_model = pn_model

    def compute_integrated_jitter(self, f_offset_min=None, f_offset_max=None, n_points=10000):
        """
        计算积分时间抖动 (Phase Jitter, RMS)
        通过对相位噪声功率谱密度在偏移频率范围内积分
        """
        if self.pn_model is None:
            raise ValueError("需要设置相位噪声模型 pn_model")

        if f_offset_min is None:
            f_offset_min = self.pn_model.f_offset[0]
        if f_offset_max is None:
            f_offset_max = self.pn_model.f_offset[-1]

        # 在对数频率上插值
        f_interp = np.logspace(np.log10(f_offset_min), np.log10(f_offset_max), n_points)
        L_interp = np.interp(np.log10(f_interp),
                             np.log10(self.pn_model.f_offset),
                             self.pn_model.L_df)

        L_linear = 10 ** (L_interp / 10)

        # 积分: J_rms^2 = 2 * integral(L(f) df)
        jitter_rms_squared = 2 * np.trapezoid(L_linear, f_interp)
        jitter_rms = np.sqrt(jitter_rms_squared)

        # 转换为时间单位 (秒) 和 UI
        jitter_time = jitter_rms / (2 * np.pi * self.f_clk)
        jitter_ui = jitter_time * self.f_clk  # 单位间隔

        return {
            'jitter_rms_rad': jitter_rms,
            'jitter_rms_deg': np.degrees(jitter_rms),
            'jitter_time_s': jitter_time,
            'jitter_time_ps': jitter_time * 1e12,
            'jitter_ui': jitter_ui,
            'f_interp': f_interp,
            'L_interp': L_interp,
            'integrand': L_linear,
        }

    def compute_period_jitter(self, f_offset_min=None, f_offset_max=None):
        """
        计算周期抖动 (Period Jitter)
        周期抖动 ≈ √2 × 相位抖动 (对于白相位噪声近似)
        """
        result = self.compute_integrated_jitter(f_offset_min, f_offset_max)
        period_jitter_ps = result['jitter_time_ps'] * np.sqrt(2)
        period = 1.0 / self.f_clk
        period_jitter_ppm = (result['jitter_time_s'] * np.sqrt(2) / period) * 1e6
        return {
            'period_jitter_ps': period_jitter_ps,
            'period_jitter_ppm': period_jitter_ppm,
            'cycle_to_cycle_ps': period_jitter_ps * 0.7,  # 经验比例
        }


class BERImpactAnalyzer:
    """时钟抖动对误码率(BER)的影响分析"""

    def __init__(self, data_rate, tx_jitter_ps, rx_jitter_ps, channel_jitter_ps=0):
        """
        参数:
            data_rate: 数据速率 (bps)
            tx_jitter_ps: 发射端时钟抖动 (ps RMS)
            rx_jitter_ps: 接收端时钟抖动 (ps RMS)
            channel_jitter_ps: 信道引入抖动 (ps RMS)
        """
        self.data_rate = data_rate
        self.tx_jitter = tx_jitter_ps * 1e-12
        self.rx_jitter = rx_jitter_ps * 1e-12
        self.channel_jitter = channel_jitter_ps * 1e-12
        self.ui = 1.0 / data_rate  # 单位间隔

    def compute_total_jitter(self):
        """计算系统总抖动 (RSS方法)"""
        tj_rss = np.sqrt(self.tx_jitter**2 + self.rx_jitter**2 + self.channel_jitter**2)
        return {
            'tj_rms_s': tj_rss,
            'tj_rms_ps': tj_rss * 1e12,
            'tj_rms_ui': tj_rss / self.ui,
            'tx_jitter_ui': self.tx_jitter / self.ui,
            'rx_jitter_ui': self.rx_jitter / self.ui,
        }

    def compute_jitter_ber_sweep(self, snr_db_range=np.linspace(5, 20, 100)):
        """
        扫描SNR, 计算有/无抖动时的BER
        使用Q函数: BER = 0.5 * erfc(SNR_linear / sqrt(2)) / 2
        抖动等效降低SNR
        """
        from scipy.special import erfc

        tj = self.compute_total_jitter()
        jitter_penalty_db = 20 * np.log10(1 + tj['tj_rms_ui'])  # 简化抖动惩罚

        snr_linear = 10 ** (snr_db_range / 10)

        # 无抖动BER (NRZ信号)
        ber_no_jitter = 0.5 * erfc(np.sqrt(snr_linear) / np.sqrt(2))

        # 有抖动BER (等效降低SNR)
        snr_degraded = snr_db_range - jitter_penalty_db
        snr_degraded_linear = 10 ** (np.maximum(snr_degraded, -10) / 10)
        ber_with_jitter = 0.5 * erfc(np.sqrt(snr_degraded_linear) / np.sqrt(2))

        return {
            'snr_db': snr_db_range,
            'ber_no_jitter': ber_no_jitter,
            'ber_with_jitter': ber_with_jitter,
            'jitter_penalty_db': jitter_penalty_db,
        }


def simulate_clock_jitter_on_adc(n_bits=12, f_signal=1e6, f_clk=100e6,
                                  jitter_rms_ps=5, n_samples=65536):
    """
    仿真时钟抖动对ADC采样的影响

    参数:
        n_bits: ADC位数
        f_signal: 输入信号频率 (Hz)
        f_clk: 采样时钟频率 (Hz)
        jitter_rms_ps: 时钟抖动RMS (ps)
        n_samples: 采样点数
    """
    dt = 1.0 / f_clk
    t = np.arange(n_samples) * dt
    jitter_rms_s = jitter_rms_ps * 1e-12

    # 理想采样
    ideal_sample = np.sin(2 * np.pi * f_signal * t)

    # 抖动采样 (采样时刻偏移)
    jitter = np.random.normal(0, jitter_rms_s, n_samples)
    t_jittered = t + jitter
    jittered_sample = np.sin(2 * np.pi * f_signal * t_jittered)

    # 量化
    lsb = 2.0 / (2 ** n_bits)
    ideal_quantized = np.round(ideal_sample / lsb) * lsb
    jittered_quantized = np.round(jittered_sample / lsb) * lsb

    # 频谱分析
    n_fft = n_samples
    freq = np.fft.rfftfreq(n_fft, dt)

    fft_ideal = np.fft.rfft(ideal_quantized) / (n_fft / 2)
    fft_jittered = np.fft.rfft(jittered_quantized) / (n_fft / 2)

    psd_ideal = 20 * np.log10(np.abs(fft_ideal) + 1e-20)
    psd_jittered = 20 * np.log10(np.abs(fft_jittered) + 1e-20)

    # SFDR计算
    def compute_sfdr(psd_linear, sig_bin, n_harmonics=10):
        sig_power = psd_linear[sig_bin] ** 2
        spur_max = 0
        for h in range(2, n_harmonics + 1):
            harm_bin = min(sig_bin * h, len(psd_linear) - 1)
            if psd_linear[harm_bin] ** 2 > spur_max:
                spur_max = psd_linear[harm_bin] ** 2
        # 也检查非谐波杂散
        noise_bins = np.ones(len(psd_linear), dtype=bool)
        margin = max(5, sig_bin // 20)
        noise_bins[max(0, sig_bin - margin):sig_bin + margin + 1] = False
        for h in range(2, n_harmonics + 1):
            harm_bin = min(sig_bin * h, len(psd_linear) - 1)
            noise_bins[max(0, harm_bin - margin):harm_bin + margin + 1] = False
        if np.any(noise_bins):
            max_spur = np.max(psd_linear[noise_bins] ** 2)
            spur_max = max(spur_max, max_spur)
        return 10 * np.log10(sig_power / spur_max) if spur_max > 0 else 200

    sig_bin = int(f_signal / (f_clk / n_fft))
    sig_bin = min(sig_bin, len(np.abs(fft_ideal)) - 1)

    sfdr_ideal = compute_sfdr(np.abs(fft_ideal), sig_bin)
    sfdr_jittered = compute_sfdr(np.abs(fft_jittered), sig_bin)

    return {
        't': t[:2000],
        'ideal': ideal_sample[:2000],
        'jittered': jittered_sample[:2000],
        'error': (jittered_sample - ideal_sample)[:2000],
        'freq': freq[:n_fft // 8],
        'psd_ideal': psd_ideal[:n_fft // 8],
        'psd_jittered': psd_jittered[:n_fft // 8],
        'sfdr_ideal_db': sfdr_ideal,
        'sfdr_jittered_db': sfdr_jittered,
        'sfdr_degradation_db': sfdr_ideal - sfdr_jittered,
    }


def plot_clock_jitter_results():
    """综合绘图: 相位噪声、抖动分析、BER影响、ADC影响"""
    fig, axes = plt.subplots(3, 2, figsize=(16, 14))

    # 1. 相位噪声对比
    models = {
        '晶体振荡器': PhaseNoiseModel.typical_crystal_oscillator(),
        'PLL输出': PhaseNoiseModel.typical_pll_output(),
        'Si5351合成器': PhaseNoiseModel.typical_synth_si5351(),
    }
    for name, model in models.items():
        axes[0, 0].semilogx(model.f_offset, model.L_df, 'o-', label=name, linewidth=2)
    axes[0, 0].set_xlabel('偏移频率 (Hz)')
    axes[0, 0].set_ylabel('相位噪声 (dBc/Hz)')
    axes[0, 0].set_title('不同类型时钟源的相位噪声')
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].legend()
    axes[0, 0].set_xlim([1, 1e8])

    # 2. 积分抖动 vs 带宽
    analyzer = ClockJitterAnalyzer(100e6, PhaseNoiseModel.typical_pll_output())
    bw_list = np.logspace(2, 7, 50)
    jitter_vs_bw = []
    for bw in bw_list:
        result = analyzer.compute_integrated_jitter(f_offset_max=bw)
        jitter_vs_bw.append(result['jitter_time_ps'])
    axes[0, 1].semilogx(bw_list, jitter_vs_bw, 'b-', linewidth=2)
    axes[0, 1].set_xlabel('积分上限带宽 (Hz)')
    axes[0, 1].set_ylabel('RMS相位抖动 (ps)')
    axes[0, 1].set_title('积分抖动 vs 带宽 (100MHz PLL时钟)')
    axes[0, 1].grid(True, alpha=0.3)
    axes[0, 1].axhline(y=1, color='r', linestyle='--', label='1ps目标')
    axes[0, 1].legend()

    # 3. BER vs SNR
    ber_analyzer = BERImpactAnalyzer(
        data_rate=1e9, tx_jitter_ps=5, rx_jitter_ps=3, channel_jitter_ps=2)
    ber_result = ber_analyzer.compute_jitter_ber_sweep()
    axes[1, 0].semilogy(ber_result['snr_db'], ber_result['ber_no_jitter'],
                        'g-', linewidth=2, label='无抖动')
    axes[1, 0].semilogy(ber_result['snr_db'], ber_result['ber_with_jitter'],
                        'r-', linewidth=2, label='有抖动')
    axes[1, 0].set_xlabel('SNR (dB)')
    axes[1, 0].set_ylabel('BER')
    axes[1, 0].set_title('抖动对BER的影响 (1Gbps)')
    axes[1, 0].grid(True, alpha=0.3)
    axes[1, 0].legend()
    axes[1, 0].set_ylim([1e-15, 1e-1])

    # 4. 不同抖动水平对比
    jitter_levels = [1, 5, 10, 20, 50]
    for tj in jitter_levels:
        ber_a = BERImpactAnalyzer(1e9, tj, tj * 0.6, 0)
        br = ber_a.compute_jitter_ber_sweep()
        axes[1, 1].semilogy(br['snr_db'], br['ber_with_jitter'],
                            linewidth=2, label=f'Tj={tj}ps')
    axes[1, 1].set_xlabel('SNR (dB)')
    axes[1, 1].set_ylabel('BER')
    axes[1, 1].set_title('不同系统总抖动下的BER')
    axes[1, 1].grid(True, alpha=0.3)
    axes[1, 1].legend()
    axes[1, 1].set_ylim([1e-15, 1e-1])

    # 5. ADC采样时钟抖动影响 - 时域
    adc_result = simulate_clock_jitter_on_adc(n_bits=12, f_signal=1e6, f_clk=100e6,
                                               jitter_rms_ps=10, n_samples=65536)
    axes[2, 0].plot(adc_result['t'] * 1e6, adc_result['error'] * 1e6, 'r-', alpha=0.7, linewidth=0.5)
    axes[2, 0].set_xlabel('时间 (μs)')
    axes[2, 0].set_ylabel('采样误差 (μV)')
    axes[2, 0].set_title(f'ADC采样误差 (抖动={10}ps, {12}bit, fin={1}MHz)')
    axes[2, 0].grid(True, alpha=0.3)

    # 6. ADC频谱对比
    axes[2, 1].plot(adc_result['freq'] / 1e6, adc_result['psd_ideal'],
                    'g-', linewidth=1, label='无抖动', alpha=0.8)
    axes[2, 1].plot(adc_result['freq'] / 1e6, adc_result['psd_jittered'],
                    'r-', linewidth=1, label='有抖动', alpha=0.8)
    axes[2, 1].set_xlabel('频率 (MHz)')
    axes[2, 1].set_ylabel('幅度 (dB)')
    axes[2, 1].set_title(f'ADC输出频谱 (SFDR: {adc_result["sfdr_ideal_db"]:.0f} → {adc_result["sfdr_jittered_db"]:.0f} dB)')
    axes[2, 1].grid(True, alpha=0.3)
    axes[2, 1].legend()

    plt.tight_layout()
    save_path = './nuedc-asset-library/15_simulation/clock_jitter_simulation_result.png'
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"[OK] 图表已保存: {save_path}")


def demo():
    """演示: 时钟抖动仿真"""
    print("=" * 60)
    print("时钟抖动仿真 - Clock Jitter Simulation")
    print("=" * 60)

    # 1. 相位噪声与抖动分析
    print("\n[1] 相位噪声模型与积分抖动:")
    models = {
        '晶体振荡器': PhaseNoiseModel.typical_crystal_oscillator(),
        'PLL输出': PhaseNoiseModel.typical_pll_output(),
        'Si5351合成器': PhaseNoiseModel.typical_synth_si5351(),
    }
    for name, model in models.items():
        analyzer = ClockJitterAnalyzer(100e6, model)
        result = analyzer.compute_integrated_jitter()
        print(f"  {name}:")
        print(f"    RMS相位抖动: {result['jitter_time_ps']:.2f} ps")
        print(f"    RMS相位抖动: {result['jitter_rms_deg']:.3f}°")
        print(f"    单位间隔占比: {result['jitter_ui'] * 100:.4f}%")

    # 2. BER影响
    print("\n[2] 抖动对BER的影响 (1Gbps系统):")
    for tj in [5, 10, 20, 50]:
        ber_a = BERImpactAnalyzer(1e9, tj, tj * 0.6, 0)
        tj_result = ber_a.compute_total_jitter()
        print(f"  系统总抖动={tj}ps RMS → 总抖动={tj_result['tj_rms_ps']:.1f}ps, "
              f"= {tj_result['tj_rms_ui'] * 100:.2f}% UI, "
              f"抖动惩罚≈{ber_a.compute_jitter_ber_sweep()['jitter_penalty_db']:.2f}dB")

    # 3. ADC时钟抖动影响
    print("\n[3] ADC时钟抖动影响:")
    for jitter in [1, 5, 10, 20, 50]:
        result = simulate_clock_jitter_on_adc(n_bits=12, f_signal=1e6,
                                               f_clk=100e6, jitter_rms_ps=jitter,
                                               n_samples=65536)
        print(f"  抖动={jitter}ps: SFDR = {result['sfdr_jittered_db']:.1f} dB "
              f"(退化: {result['sfdr_degradation_db']:.1f} dB)")

    # 4. 绘图
    print("\n[4] 生成图表...")
    try:
        plot_clock_jitter_results()
    except Exception as e:
        print(f"  绘图跳过: {e}")

    print("\n仿真完成!")


if __name__ == '__main__':
    demo()
