"""
混合信号仿真 - Mixed Signal System Simulation
===============================================
仿真内容: ADC+DAC+滤波器级联 / 误差预算分析 / 信噪比级联
适用场景: 混合信号系统设计、数据采集+回放系统评估

电赛应用: 信号采集与回放系统设计、模拟前端性能评估
"""

import numpy as np
from scipy import signal as sig
import matplotlib.pyplot as plt
import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False


class AnalogFilter:
    """模拟滤波器模型"""

    def __init__(self, order, cutoff, f_sample, ftype='low', ripple=0.5):
        """
        参数:
            order: 滤波器阶数
            cutoff: 截止频率 (Hz), 对于带通是 [f_low, f_high]
            f_sample: 仿真采样率 (Hz)
            ftype: 类型 'low'/'high'/'band'
            ripple: 纹波 (dB, 仅Chebyshev)
        """
        self.order = order
        self.cutoff = cutoff
        self.f_sample = f_sample
        self.ftype = ftype
        self.ripple = ripple

        # 设计Butterworth滤波器
        nyq = f_sample / 2
        if isinstance(cutoff, (list, tuple)):
            Wn = [c / nyq for c in cutoff]
        else:
            Wn = cutoff / nyq
        Wn = np.clip(Wn, 0.001, 0.999)

        self.b, self.a = sig.butter(order, Wn, btype=ftype, analog=False)

    def filter(self, x):
        """滤波 (零相移)"""
        return sig.filtfilt(self.b, self.a, x)

    def frequency_response(self, f):
        """频率响应"""
        w, h = sig.freqz(self.b, a=self.a, worN=f, fs=self.f_sample)
        return h

    @property
    def insertion_loss_at_dc(self):
        """直流插入损耗 (dB)"""
        h = self.frequency_response([0.001])
        return 20 * np.log10(np.abs(h[0]) + 1e-20)


class DACModel:
    """DAC行为级模型"""

    def __init__(self, n_bits, f_update, settling_error=0.001, glitch_area=0):
        """
        参数:
            n_bits: DAC分辨率
            f_update: 更新速率 (Hz)
            settling_error: 建立误差 (比例)
            glitch_area: 毛刺面积 (V·s)
        """
        self.n_bits = n_bits
        self.f_update = f_update
        self.settling_error = settling_error
        self.glitch_area = glitch_area
        self.n_levels = 2**n_bits

    def convert(self, digital_in, add_noise_uv=0):
        """
        DAC转换

        参数:
            digital_in: 数字输入 (-1 到 1 归一化)
            add_noise_uv: 输出噪声 (μV RMS)
        """
        # 量化到DAC码
        codes = np.round((digital_in + 1) / 2 * (self.n_levels - 1))
        codes = np.clip(codes, 0, self.n_levels - 1)

        # 转换为模拟输出
        output = codes / (self.n_levels - 1) * 2 - 1

        # 建立误差 (一阶近似)
        if self.settling_error > 0:
            # RC建立模型
            tau = 1.0 / (2 * np.pi * self.f_update * 5)  # 假设5倍过采样带宽
            dt = 1.0 / self.f_update
            alpha = 1 - np.exp(-dt / tau)
            settled = np.zeros_like(output)
            settled[0] = output[0]
            for i in range(1, len(output)):
                settled[i] = settled[i-1] + alpha * (output[i] - settled[i-1])
            output = settled

        # 输出噪声
        if add_noise_uv > 0:
            output += np.random.normal(0, add_noise_uv * 1e-6, len(output))

        return output


class MixedSignalSystem:
    """混合信号系统: 信号源 → 模拟前端 → ADC → DSP → DAC → 模拟后端"""

    def __init__(self, adc_bits=12, dac_bits=12, f_sample=100e6,
                 anti_alias_bw=None, recon_bw=None):
        """
        参数:
            adc_bits: ADC分辨率
            dac_bits: DAC分辨率
            f_sample: 系统采样率
            anti_alias_bw: 抗混叠滤波器带宽 (Hz)
            recon_bw: 重建滤波器带宽 (Hz)
        """
        self.f_sample = f_sample

        # ADC
        self.adc_bits = adc_bits
        self.adc_lsb = 2.0 / 2**adc_bits

        # DAC
        self.dac = DACModel(dac_bits, f_sample)

        # 抗混叠滤波器
        if anti_alias_bw is None:
            anti_alias_bw = f_sample / 2.5
        self.anti_alias = AnalogFilter(order=4, cutoff=anti_alias_bw, f_sample=f_sample)

        # 重建滤波器
        if recon_bw is None:
            recon_bw = f_sample / 2.5
        self.recon_filter = AnalogFilter(order=4, cutoff=recon_bw, f_sample=f_sample)

    def process(self, analog_input, n_samples=None, adc_jitter_ps=0,
                adc_noise_uv=0, dac_noise_uv=0, add_dsp_gain_error=0,
                add_dsp_offset=0):
        """
        完整信号链处理

        参数:
            analog_input: 模拟输入信号
            n_samples: 处理样本数
            adc_jitter_ps: ADC孔径抖动 (ps)
            adc_noise_uv: ADC输入噪声 (μV)
            dac_noise_uv: DAC输出噪声 (μV)
            add_dsp_gain_error: DSP增益误差 (比例, 如0.01=1%)
            add_dsp_offset: DSP偏移误差 (V)
        """
        if n_samples is None:
            n_samples = len(analog_input)

        x = analog_input[:n_samples].copy()

        # 1. 抗混叠滤波
        x_filtered = self.anti_alias.filter(x)

        # 2. ADC采样
        # 孔径抖动
        if adc_jitter_ps > 0:
            t = np.arange(n_samples) / self.f_sample
            jitter = np.random.normal(0, adc_jitter_ps * 1e-12, n_samples)
            deriv = np.gradient(x_filtered, t)
            x_filtered += jitter * deriv

        # ADC噪声
        if adc_noise_uv > 0:
            x_filtered += np.random.normal(0, adc_noise_uv * 1e-6, n_samples)

        # 量化
        x_quantized = np.round(x_filtered / self.adc_lsb) * self.adc_lsb
        x_quantized = np.clip(x_quantized, -1.0, 1.0)

        # 3. DSP处理 (可加增益/偏移误差)
        x_dsp = x_quantized * (1 + add_dsp_gain_error) + add_dsp_offset

        # 4. DAC重建
        x_dac = self.dac.convert(digital_in=x_dsp, add_noise_uv=dac_noise_uv)

        # 5. 重建滤波
        x_output = self.recon_filter.filter(x_dac)

        return {
            'input': analog_input[:n_samples],
            'after_aa_filter': x_filtered,
            'after_adc': x_quantized,
            'after_dsp': x_dsp,
            'after_dac': x_dac,
            'output': x_output,
            'error': x_output - analog_input[:n_samples],
        }

    def compute_error_budget(self, f_in, amplitude=0.9, adc_jitter_ps=5,
                             adc_noise_uv=100, dac_noise_uv=200,
                             gain_error=0.01, offset_error=0.001):
        """
        计算系统误差预算

        返回各环节引入的误差功率
        """
        t = np.arange(65536) / self.f_sample
        signal_in = amplitude * np.sin(2 * np.pi * f_in * t)
        signal_power = amplitude**2 / 2

        budget = {}

        # 量化噪声
        lsb = self.adc_lsb
        q_noise_power = lsb**2 / 12
        budget['量化噪声'] = {
            'power': q_noise_power,
            'snr_db': 10 * np.log10(signal_power / q_noise_power),
        }

        # 孔径抖动噪声
        if adc_jitter_ps > 0:
            jitter_noise_power = (2 * np.pi * f_in * adc_jitter_ps * 1e-12 * amplitude)**2 / 2
        else:
            jitter_noise_power = 0
        budget['孔径抖动'] = {
            'power': jitter_noise_power,
            'snr_db': 10 * np.log10(signal_power / max(jitter_noise_power, 1e-30)),
        }

        # ADC热噪声
        adc_thermal_power = (adc_noise_uv * 1e-6)**2
        budget['ADC热噪声'] = {
            'power': adc_thermal_power,
            'snr_db': 10 * np.log10(signal_power / max(adc_thermal_power, 1e-30)),
        }

        # DAC噪声
        dac_thermal_power = (dac_noise_uv * 1e-6)**2
        budget['DAC噪声'] = {
            'power': dac_thermal_power,
            'snr_db': 10 * np.log10(signal_power / max(dac_thermal_power, 1e-30)),
        }

        # 增益误差
        gain_error_power = (amplitude * gain_error)**2 / 2
        budget['增益误差'] = {
            'power': gain_error_power,
            'snr_db': 10 * np.log10(signal_power / max(gain_error_power, 1e-30)),
        }

        # 偏移误差
        offset_power = offset_error**2
        budget['偏移误差'] = {
            'power': offset_power,
            'snr_db': 10 * np.log10(signal_power / max(offset_power, 1e-30)),
        }

        # 总误差
        total_noise = sum(v['power'] for v in budget.values())
        budget['系统总计'] = {
            'power': total_noise,
            'snr_db': 10 * np.log10(signal_power / total_noise) if total_noise > 0 else 200,
        }

        # ENOB
        budget['系统ENOB'] = (budget['系统总计']['snr_db'] - 1.76) / 6.02

        return budget


def simulate_cascade_filter(f_sample=100e6, signal_freq=5e6):
    """
    仿真ADC+DAC+滤波器级联性能
    """
    n_samples = 65536
    t = np.arange(n_samples) / f_sample
    signal_in = 0.9 * np.sin(2 * np.pi * signal_freq * t)

    system = MixedSignalSystem(adc_bits=12, dac_bits=12, f_sample=f_sample)

    result = system.process(signal_in, adc_jitter_ps=5, adc_noise_uv=50, dac_noise_uv=100)

    # 频谱分析
    freq = np.fft.rfftfreq(n_samples, 1.0 / f_sample)
    spec_in = 20 * np.log10(np.abs(np.fft.rfft(signal_in)) / (n_samples / 2) + 1e-20)
    spec_out = 20 * np.log10(np.abs(np.fft.rfft(result['output'])) / (n_samples / 2) + 1e-20)
    spec_err = 20 * np.log10(np.abs(np.fft.rfft(result['error'])) / (n_samples / 2) + 1e-20)

    return {
        't': t[:1000],
        'input': signal_in[:1000],
        'output': result['output'][:1000],
        'error': result['error'][:1000],
        'freq': freq,
        'spec_in': spec_in,
        'spec_out': spec_out,
        'spec_err': spec_err,
        'rms_error': np.std(result['error']),
        'peak_error': np.max(np.abs(result['error'])),
    }


def plot_mixed_signal_results():
    """综合绘图: 混合信号系统仿真"""
    fig, axes = plt.subplots(3, 2, figsize=(16, 14))

    f_sample = 100e6
    f_in = 5.05e6

    # 1. 时域信号对比
    cascade = simulate_cascade_filter(f_sample, f_in)
    axes[0, 0].plot(cascade['t'] * 1e6, cascade['input'], 'g-', linewidth=1.5, label='输入')
    axes[0, 0].plot(cascade['t'] * 1e6, cascade['output'], 'r-', linewidth=1, alpha=0.7, label='输出')
    axes[0, 0].set_xlabel('时间 (μs)')
    axes[0, 0].set_ylabel('幅度 (V)')
    axes[0, 0].set_title(f'混合信号系统时域响应 (fin={f_in/1e6}MHz)')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)

    # 2. 误差信号
    axes[0, 1].plot(cascade['t'] * 1e6, cascade['error'] * 1e6, 'r-', linewidth=0.5)
    axes[0, 1].set_xlabel('时间 (μs)')
    axes[0, 1].set_ylabel('误差 (μV)')
    axes[0, 1].set_title(f'系统误差 (RMS={cascade["rms_error"]*1e6:.1f}μV, Peak={cascade["peak_error"]*1e6:.1f}μV)')
    axes[0, 1].grid(True, alpha=0.3)

    # 3. 频谱对比
    axes[1, 0].plot(cascade['freq'] / 1e6, cascade['spec_in'], 'g-', linewidth=1, label='输入', alpha=0.7)
    axes[1, 0].plot(cascade['freq'] / 1e6, cascade['spec_out'], 'r-', linewidth=1, label='输出', alpha=0.7)
    axes[1, 0].set_xlabel('频率 (MHz)')
    axes[1, 0].set_ylabel('幅度 (dB)')
    axes[1, 0].set_title('输入/输出频谱')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)
    axes[1, 0].set_xlim([0, f_sample / 2 / 1e6])

    # 4. 误差预算饼图
    system = MixedSignalSystem(12, 12, f_sample)
    budget = system.compute_error_budget(f_in, adc_jitter_ps=5, adc_noise_uv=100,
                                          dac_noise_uv=200, gain_error=0.01, offset_error=0.001)

    labels = [k for k in budget.keys() if k not in ['系统总计', '系统ENOB']]
    powers = [budget[k]['power'] for k in labels]
    # 过滤掉极小值
    threshold = max(powers) * 0.001
    filtered = [(l, p) for l, p in zip(labels, powers) if p > threshold]
    if filtered:
        f_labels, f_powers = zip(*filtered)
        axes[1, 1].pie(f_powers, labels=f_labels, autopct='%1.1f%%', startangle=90)
        axes[1, 1].set_title(f'误差预算 (系统ENOB={budget["系统ENOB"]:.1f}bit, SNR={budget["系统总计"]["snr_db"]:.1f}dB)')

    # 5. ADC位数 vs 系统性能
    adc_bits_range = [8, 10, 12, 14, 16]
    system_enob = []
    system_snr = []
    for nb in adc_bits_range:
        sys = MixedSignalSystem(nb, nb, f_sample)
        b = sys.compute_error_budget(f_in, adc_jitter_ps=5, adc_noise_uv=100,
                                      dac_noise_uv=200, gain_error=0.01, offset_error=0.001)
        system_enob.append(b['系统ENOB'])
        system_snr.append(b['系统总计']['snr_db'])
    axes[2, 0].plot(adc_bits_range, system_enob, 'bo-', linewidth=2, markersize=8, label='系统ENOB')
    axes[2, 0].plot(adc_bits_range, adc_bits_range, 'g--', linewidth=1, label='理想ENOB')
    axes[2, 0].set_xlabel('ADC/DAC分辨率 (bit)')
    axes[2, 0].set_ylabel('ENOB (bit)')
    axes[2, 0].set_title('系统ENOB vs ADC/DAC分辨率')
    axes[2, 0].legend()
    axes[2, 0].grid(True, alpha=0.3)

    # 6. 输入频率扫描
    freq_range = np.logspace(4, 7, 30)
    snr_vs_freq = []
    enob_vs_freq = []
    for f in freq_range:
        b = system.compute_error_budget(f, adc_jitter_ps=10, adc_noise_uv=100,
                                         dac_noise_uv=200, gain_error=0.005, offset_error=0.0005)
        snr_vs_freq.append(b['系统总计']['snr_db'])
        enob_vs_freq.append(b['系统ENOB'])
    axes[2, 1].semilogx(freq_range / 1e6, enob_vs_freq, 'b-', linewidth=2, label='系统ENOB')
    ax2 = axes[2, 1].twinx()
    ax2.semilogx(freq_range / 1e6, snr_vs_freq, 'r--', linewidth=2, label='系统SNR')
    axes[2, 1].set_xlabel('输入频率 (MHz)')
    axes[2, 1].set_ylabel('ENOB (bit)', color='b')
    ax2.set_ylabel('SNR (dB)', color='r')
    axes[2, 1].set_title('系统性能 vs 输入频率 (jitter=10ps)')
    axes[2, 1].grid(True, alpha=0.3)
    lines1, labels1 = axes[2, 1].get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    axes[2, 1].legend(lines1 + lines2, labels1 + labels2)

    plt.tight_layout()
    save_path = './nuedc-asset-library/15_simulation/mixed_signal_simulation_result.png'
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"[OK] 图表已保存: {save_path}")


def demo():
    """演示: 混合信号系统仿真"""
    print("=" * 60)
    print("混合信号仿真 - Mixed Signal System Simulation")
    print("=" * 60)

    f_sample = 100e6
    f_in = 5.05e6

    # 1. 系统误差预算
    print(f"\n[1] 系统误差预算 (12bit ADC/DAC, fs={f_sample/1e6}MHz):")
    system = MixedSignalSystem(12, 12, f_sample)
    budget = system.compute_error_budget(f_in, adc_jitter_ps=5, adc_noise_uv=100,
                                          dac_noise_uv=200, gain_error=0.01, offset_error=0.001)
    for name, val in budget.items():
        if isinstance(val, dict):
            print(f"  {name}: 功率={val['power']:.2e}, SNR={val['snr_db']:.1f}dB")
        else:
            print(f"  {name}: {val:.2f} bit")

    # 2. 不同分辨率对比
    print(f"\n[2] ADC/DAC分辨率对系统性能的影响:")
    for nb in [8, 10, 12, 14, 16]:
        sys = MixedSignalSystem(nb, nb, f_sample)
        b = sys.compute_error_budget(f_in, adc_jitter_ps=5, adc_noise_uv=100,
                                      dac_noise_uv=200, gain_error=0.01, offset_error=0.001)
        print(f"  {nb}bit: 系统ENOB={b['系统ENOB']:.1f}bit, SNR={b['系统总计']['snr_db']:.1f}dB")

    # 3. 级联仿真
    print(f"\n[3] 级联仿真 (fin={f_in/1e6}MHz):")
    cascade = simulate_cascade_filter(f_sample, f_in)
    print(f"  RMS误差: {cascade['rms_error']*1e6:.2f} μV")
    print(f"  峰值误差: {cascade['peak_error']*1e6:.2f} μV")

    # 4. 绘图
    print("\n[4] 生成图表...")
    try:
        plot_mixed_signal_results()
    except Exception as e:
        print(f"  绘图跳过: {e}")

    print("\n仿真完成!")


if __name__ == '__main__':
    demo()
