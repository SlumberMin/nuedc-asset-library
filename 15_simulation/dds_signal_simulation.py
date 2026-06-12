"""
DDS信号仿真 - Direct Digital Synthesis Simulation
===================================================
仿真内容: 相位截断效应 / 幅度量化 / DAC非线性 / 杂散分析
适用场景: 任意波形发生器设计、频率合成器评估

电赛应用: 信号发生器设计、DDS芯片选型与性能评估
"""

import numpy as np
from scipy import signal as sig
import matplotlib.pyplot as plt
import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False


class DDSModel:
    """DDS (直接数字频率合成) 行为级模型"""

    def __init__(self, f_clk, phase_bits=32, amplitude_bits=12, dac_bits=12):
        """
        参数:
            f_clk: 系统时钟频率 (Hz)
            phase_bits: 相位累加器位数
            amplitude_bits: 幅度查找表位数 (截断后)
            dac_bits: DAC分辨率 (位)
        """
        self.f_clk = f_clk
        self.phase_bits = phase_bits
        self.amplitude_bits = amplitude_bits
        self.dac_bits = dac_bits
        self.phase_trunc_bits = phase_bits - amplitude_bits  # 相位截断位数

    def compute_frequency_word(self, f_out):
        """计算频率调谐字"""
        return int(f_out * 2**self.phase_bits / self.f_clk)

    def generate_waveform(self, f_out, n_cycles=100, add_phase_noise_db=None):
        """
        生成DDS波形, 包含各种非理想效应

        参数:
            f_out: 输出频率 (Hz)
            n_cycles: 输出周期数
            add_phase_noise_db: 附加相位噪声 (dBc/Hz @ offset), None则不加
        """
        n_samples = int(n_cycles * self.f_clk / f_out)
        n_samples = min(n_samples, 2**20)  # 限制最大样本数

        t = np.arange(n_samples) / self.f_clk

        # 频率调谐字
        FTW = self.compute_frequency_word(f_out)

        # 相位累加器 (完整精度)
        phase_acc = np.cumsum(np.ones(n_samples) * FTW).astype(np.int64) % (2**self.phase_bits)

        # === 相位截断 ===
        # 截断: 只取高位
        phase_truncated = (phase_acc >> self.phase_trunc_bits) << self.phase_trunc_bits
        phase_error = phase_acc - phase_truncated  # 截断误差

        # 用截断后的相位查表
        phase_for_lut = phase_truncated / (2**self.phase_bits) * 2 * np.pi

        # === 幅度查找表 ===
        n_lut = 2**self.amplitude_bits
        # 完整正弦表
        lut_full = np.sin(np.linspace(0, 2 * np.pi, n_lut, endpoint=False))

        # 幅度量化 (查找表精度)
        phase_index = (phase_truncated >> self.phase_trunc_bits).astype(np.int64) % n_lut
        amplitude_lut = lut_full[phase_index]

        # === DAC量化 ===
        n_dac_levels = 2**self.dac_bits
        amplitude_quantized = np.round(amplitude_lut * (n_dac_levels / 2)) / (n_dac_levels / 2)

        # === DAC非线性 (DNL/INL) ===
        # 模拟典型DNL = ±0.5 LSB, INL = ±1 LSB
        dnl = np.random.uniform(-0.5, 0.5, n_dac_levels) / n_dac_levels
        # 逐样本应用DNL
        dac_codes = np.round((amplitude_quantized + 1) / 2 * (n_dac_levels - 1)).astype(int)
        dac_codes = np.clip(dac_codes, 0, n_dac_levels - 1)
        amplitude_dnl = amplitude_quantized + dnl[dac_codes]

        # === 附加相位噪声 ===
        if add_phase_noise_db is not None:
            # 简化: 添加白相位噪声
            pn_power = 10**(add_phase_noise_db / 10) * self.f_clk / 2
            phase_noise = np.random.normal(0, np.sqrt(pn_power), n_samples)
            amplitude_dnl += phase_noise * np.cos(phase_for_lut)  # 小相位扰动

        return {
            't': t,
            'output': amplitude_dnl,
            'ideal': np.sin(2 * np.pi * f_out * t),
            'phase_error': phase_error,
            'phase_truncated': phase_for_lut,
        }

    def analyze_spectrum(self, waveform, f_out, n_fft=None):
        """分析DDS输出频谱"""
        if n_fft is None:
            n_fft = len(waveform)

        freq = np.fft.rfftfreq(n_fft, 1.0 / self.f_clk)
        spectrum = np.fft.rfft(waveform[:n_fft]) / (n_fft / 2)
        psd_db = 20 * np.log10(np.abs(spectrum) + 1e-20)

        # 找信号频率对应的bin
        sig_bin = int(f_out / (self.f_clk / n_fft))
        sig_bin = min(sig_bin, len(spectrum) - 1)

        # SFDR: 信号与最大杂散的比值
        sig_power = np.abs(spectrum[sig_bin])**2
        margin = max(5, sig_bin // 20)
        spur_mask = np.ones(len(spectrum), dtype=bool)
        spur_mask[max(0, sig_bin - margin):sig_bin + margin + 1] = False
        # 排除直流
        spur_mask[0:3] = False

        if np.any(spur_mask):
            max_spur_power = np.max(np.abs(spectrum[spur_mask])**2)
            sfdr_db = 10 * np.log10(sig_power / max_spur_power)
            max_spur_bin = np.argmax(np.abs(spectrum[spur_mask])**2)
        else:
            sfdr_db = 200
            max_spur_bin = 0

        # SINAD
        noise_mask = spur_mask.copy()
        noise_and_distortion = np.sum(np.abs(spectrum[noise_mask])**2)
        sinad_db = 10 * np.log10(sig_power / noise_and_distortion) if noise_and_distortion > 0 else 200
        enob = (sinad_db - 1.76) / 6.02

        return {
            'freq': freq,
            'psd_db': psd_db,
            'sfdr_db': sfdr_db,
            'sinad_db': sinad_db,
            'enob': enob,
        }


def analyze_phase_truncation_spur(f_clk=100e6, f_out=30.1e6, phase_bits_range=[16, 20, 24, 28, 32]):
    """分析相位截断导致的杂散"""
    results = {}
    for pb in phase_bits_range:
        for amp_bits in [8, 10, 12]:
            dds = DDSModel(f_clk, phase_bits=pb, amplitude_bits=amp_bits, dac_bits=12)
            wf = dds.generate_waveform(f_out, n_cycles=50)
            spec = dds.analyze_spectrum(wf['output'], f_out)
            key = f"phase={pb},amp={amp_bits}"
            results[key] = {
                'phase_bits': pb,
                'amp_bits': amp_bits,
                'sfdr': spec['sfdr_db'],
                'sinad': spec['sinad_db'],
                'enob': spec['enob'],
            }
    return results


def analyze_dac_nonlinearity(f_clk=100e6, f_out=10e6, dac_bits_range=[8, 10, 12, 14, 16]):
    """分析DAC分辨率对SFDR的影响"""
    results = {}
    for db in dac_bits_range:
        dds = DDSModel(f_clk, phase_bits=32, amplitude_bits=14, dac_bits=db)
        wf = dds.generate_waveform(f_out, n_cycles=100)
        spec = dds.analyze_spectrum(wf['output'], f_out)
        results[db] = {
            'sfdr': spec['sfdr_db'],
            'sinad': spec['sinad_db'],
            'enob': spec['enob'],
        }
    return results


def compute_spur_frequency(f_clk, f_out, n_harmonics=5):
    """
    计算DDS输出中相位截断杂散的频率位置

    相位截断杂散位于: f_out ± k * f_trunc
    其中 f_trunc = f_clk / 2^(phase_trunc_bits)
    """
    # 信号谐波
    harmonics = [(n * f_out) % f_clk for n in range(1, n_harmonics + 1)]
    return {
        'signal_freq': f_out,
        'harmonics': harmonics,
        'aliased_harmonics': [min(h, f_clk - h) for h in harmonics],
    }


def plot_dds_results():
    """综合绘图: DDS仿真结果"""
    fig, axes = plt.subplots(3, 2, figsize=(16, 14))

    f_clk = 100e6
    f_out = 30.1e6

    # 1. 理想 vs 实际波形
    dds = DDSModel(f_clk, phase_bits=24, amplitude_bits=10, dac_bits=10)
    wf = dds.generate_waveform(f_out, n_cycles=10)
    n_show = min(500, len(wf['t']))
    axes[0, 0].plot(wf['t'][:n_show] * 1e6, wf['ideal'][:n_show], 'g-', linewidth=1, label='理想')
    axes[0, 0].plot(wf['t'][:n_show] * 1e6, wf['output'][:n_show], 'r-', linewidth=1, alpha=0.7, label='DDS输出')
    axes[0, 0].set_xlabel('时间 (μs)')
    axes[0, 0].set_ylabel('幅度')
    axes[0, 0].set_title(f'DDS波形 (f_clk={f_clk/1e6}MHz, f_out={f_out/1e6}MHz)')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)

    # 2. 频谱
    spec = dds.analyze_spectrum(wf['output'], f_out)
    freq_mhz = spec['freq'] / 1e6
    axes[0, 1].plot(freq_mhz, spec['psd_db'], 'b-', linewidth=0.5)
    axes[0, 1].set_xlabel('频率 (MHz)')
    axes[0, 1].set_ylabel('幅度 (dB)')
    axes[0, 1].set_title(f'DDS输出频谱 (SFDR={spec["sfdr_db"]:.1f}dB, ENOB={spec["enob"]:.1f})')
    axes[0, 1].grid(True, alpha=0.3)
    axes[0, 1].set_xlim([0, f_clk / 2 / 1e6])

    # 3. 相位截断误差
    axes[1, 0].plot(wf['t'][:n_show] * 1e6, wf['phase_error'][:n_show] / 2**24 * 360,
                    'r-', linewidth=0.5)
    axes[1, 0].set_xlabel('时间 (μs)')
    axes[1, 0].set_ylabel('相位误差 (°)')
    axes[1, 0].set_title('相位截断误差')
    axes[1, 0].grid(True, alpha=0.3)

    # 4. SFDR vs 相位/幅度位数
    phase_bits_list = [16, 20, 24, 28, 32]
    amp_bits_list = [8, 10, 12]
    for ab in amp_bits_list:
        sfdr_vals = []
        for pb in phase_bits_list:
            dds_test = DDSModel(f_clk, phase_bits=pb, amplitude_bits=ab, dac_bits=12)
            wf_test = dds_test.generate_waveform(f_out, n_cycles=50)
            spec_test = dds_test.analyze_spectrum(wf_test['output'], f_out)
            sfdr_vals.append(spec_test['sfdr_db'])
        axes[1, 1].plot(phase_bits_list, sfdr_vals, 'o-', linewidth=2, markersize=6,
                        label=f'幅度位数={ab}')
    axes[1, 1].set_xlabel('相位累加器位数')
    axes[1, 1].set_ylabel('SFDR (dB)')
    axes[1, 1].set_title('SFDR vs 相位/幅度位数')
    axes[1, 1].grid(True, alpha=0.3)
    axes[1, 1].legend()

    # 5. DAC位数影响
    dac_bits_list = [6, 8, 10, 12, 14, 16]
    sfdr_dac = []
    sinad_dac = []
    for db in dac_bits_list:
        dds_test = DDSModel(f_clk, phase_bits=32, amplitude_bits=14, dac_bits=db)
        wf_test = dds_test.generate_waveform(f_out, n_cycles=100)
        spec_test = dds_test.analyze_spectrum(wf_test['output'], f_out)
        sfdr_dac.append(spec_test['sfdr_db'])
        sinad_dac.append(spec_test['sinad_db'])
    axes[2, 0].plot(dac_bits_list, sfdr_dac, 'bo-', linewidth=2, label='SFDR')
    axes[2, 0].plot(dac_bits_list, sinad_dac, 'rs-', linewidth=2, label='SINAD')
    axes[2, 0].set_xlabel('DAC位数')
    axes[2, 0].set_ylabel('dB')
    axes[2, 0].set_title('SFDR/SINAD vs DAC分辨率')
    axes[2, 0].grid(True, alpha=0.3)
    axes[2, 0].legend()

    # 6. 输出频率扫描 SFDR
    freq_ratios = np.linspace(0.05, 0.45, 20)
    sfdr_freq = []
    for ratio in freq_ratios:
        f_test = f_clk * ratio
        dds_test = DDSModel(f_clk, phase_bits=24, amplitude_bits=10, dac_bits=10)
        wf_test = dds_test.generate_waveform(f_test, n_cycles=100)
        spec_test = dds_test.analyze_spectrum(wf_test['output'], f_test)
        sfdr_freq.append(spec_test['sfdr_db'])
    axes[2, 1].plot(freq_ratios, sfdr_freq, 'bo-', linewidth=2)
    axes[2, 1].set_xlabel('f_out / f_clk')
    axes[2, 1].set_ylabel('SFDR (dB)')
    axes[2, 1].set_title('SFDR vs 输出频率比')
    axes[2, 1].grid(True, alpha=0.3)

    plt.tight_layout()
    save_path = './nuedc-asset-library/15_simulation/dds_signal_simulation_result.png'
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"[OK] 图表已保存: {save_path}")


def demo():
    """演示: DDS信号仿真"""
    print("=" * 60)
    print("DDS信号仿真 - Direct Digital Synthesis Simulation")
    print("=" * 60)

    f_clk = 100e6
    f_out = 30.1e6

    # 1. 基本DDS参数
    print(f"\n[1] DDS系统参数:")
    print(f"  时钟频率: {f_clk/1e6:.0f} MHz")
    print(f"  输出频率: {f_out/1e6:.1f} MHz")
    dds = DDSModel(f_clk, phase_bits=32, amplitude_bits=12, dac_bits=12)
    ftw = dds.compute_frequency_word(f_out)
    print(f"  频率分辨率: {f_clk / 2**32:.6f} Hz")
    print(f"  频率调谐字: {ftw} (0x{ftw:08X})")

    # 2. 相位截断影响
    print(f"\n[2] 相位截断影响分析 (f_out={f_out/1e6}MHz):")
    for amp_bits in [8, 10, 12]:
        for pb in [16, 20, 24, 28]:
            dds_test = DDSModel(f_clk, phase_bits=pb, amplitude_bits=amp_bits, dac_bits=12)
            wf = dds_test.generate_waveform(f_out, n_cycles=50)
            spec = dds_test.analyze_spectrum(wf['output'], f_out)
            print(f"  Phase={pb}bit, Amp={amp_bits}bit: SFDR={spec['sfdr_db']:.1f}dB, "
                  f"ENOB={spec['enob']:.1f}")

    # 3. DAC位数影响
    print(f"\n[3] DAC分辨率影响:")
    for db in [8, 10, 12, 14]:
        dds_test = DDSModel(f_clk, phase_bits=32, amplitude_bits=14, dac_bits=db)
        wf = dds_test.generate_waveform(f_out, n_cycles=100)
        spec = dds_test.analyze_spectrum(wf['output'], f_out)
        print(f"  DAC={db}bit: SFDR={spec['sfdr_db']:.1f}dB, SINAD={spec['sinad_db']:.1f}dB, "
              f"ENOB={spec['enob']:.1f}")

    # 4. 绘图
    print("\n[4] 生成图表...")
    try:
        plot_dds_results()
    except Exception as e:
        print(f"  绘图跳过: {e}")

    print("\n仿真完成!")


if __name__ == '__main__':
    demo()
