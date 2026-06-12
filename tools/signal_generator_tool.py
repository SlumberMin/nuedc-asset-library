#!/usr/bin/env python3
"""
信号生成工具 - 正弦/方波/三角波/噪声/任意波形
用于电赛信号发生器题和测试信号生成
"""
import argparse
import json
import math
import random
import struct
import sys


# ── 波形生成函数 ──────────────────────────────────────────────

def generate_sine(freq, amplitude, phase_deg, offset, sample_rate, num_samples):
    """生成正弦波"""
    phase_rad = math.radians(phase_deg)
    return [
        offset + amplitude * math.sin(2 * math.pi * freq * i / sample_rate + phase_rad)
        for i in range(num_samples)
    ]


def generate_square(freq, amplitude, duty_cycle, offset, sample_rate, num_samples):
    """生成方波"""
    period = sample_rate / freq
    return [
        offset + amplitude if (i % period) < period * duty_cycle else offset - amplitude
        for i in range(num_samples)
    ]


def generate_triangle(freq, amplitude, offset, sample_rate, num_samples):
    """生成三角波"""
    period = sample_rate / freq
    result = []
    for i in range(num_samples):
        t = (i % period) / period
        if t < 0.5:
            val = 4 * t - 1  # -1 to 1
        else:
            val = 3 - 4 * t  # 1 to -1
        result.append(offset + amplitude * val)
    return result


def generate_sawtooth(freq, amplitude, offset, sample_rate, num_samples):
    """生成锯齿波"""
    period = sample_rate / freq
    return [
        offset + amplitude * (2 * (i % period) / period - 1)
        for i in range(num_samples)
    ]


def generate_noise(amplitude, noise_type, offset, num_samples):
    """生成噪声信号"""
    result = []
    for _ in range(num_samples):
        if noise_type == 'white':
            val = random.gauss(0, amplitude)
        elif noise_type == 'uniform':
            val = random.uniform(-amplitude, amplitude)
        else:
            val = random.gauss(0, amplitude)
        result.append(offset + val)
    return result


def generate_pulse(freq, amplitude, rise_ns, fall_ns, offset, sample_rate, num_samples):
    """生成脉冲信号（含上升/下降沿时间）"""
    period = sample_rate / freq
    rise_samples = max(1, int(rise_ns * 1e-9 * sample_rate))
    fall_samples = max(1, int(fall_ns * 1e-9 * sample_rate))
    result = []
    for i in range(num_samples):
        pos = i % period
        if pos < rise_samples:
            val = pos / rise_samples
        elif pos < period - fall_samples:
            val = 1.0
        elif pos < period:
            val = 1.0 - (pos - (period - fall_samples)) / fall_samples
        else:
            val = 0.0
        result.append(offset + amplitude * val)
    return result


def generate_am_modulation(carrier_freq, mod_freq, mod_depth, amplitude,
                           offset, sample_rate, num_samples):
    """生成AM调幅信号"""
    result = []
    for i in range(num_samples):
        t = i / sample_rate
        envelope = 1 + mod_depth * math.sin(2 * math.pi * mod_freq * t)
        carrier = math.sin(2 * math.pi * carrier_freq * t)
        result.append(offset + amplitude * envelope * carrier)
    return result


def generate_fm_modulation(carrier_freq, mod_freq, freq_dev, amplitude,
                           offset, sample_rate, num_samples):
    """生成FM调频信号"""
    result = []
    for i in range(num_samples):
        t = i / sample_rate
        mod_integral = (mod_freq / (2 * math.pi)) * (-math.cos(2 * math.pi * mod_freq * t))
        val = math.sin(2 * math.pi * carrier_freq * t + 2 * math.pi * freq_dev * mod_integral)
        result.append(offset + amplitude * val)
    return result


# ── 多音信号 ─────────────────────────────────────────────────
def generate_multitone(tones, sample_rate, num_samples):
    """
    生成多音叠加信号
    tones: [{'freq': Hz, 'amplitude': V, 'phase_deg': deg}, ...]
    """
    result = [0.0] * num_samples
    for tone in tones:
        phase_rad = math.radians(tone.get('phase_deg', 0))
        amp = tone['amplitude']
        freq = tone['freq']
        for i in range(num_samples):
            result[i] += amp * math.sin(2 * math.pi * freq * i / sample_rate + phase_rad)
    return result


# ── 量化与编码 ────────────────────────────────────────────────
def quantize_signal(samples, resolution_bits=12, vref=3.3):
    """将模拟信号量化为DAC数值"""
    levels = 2 ** resolution_bits
    max_val = vref
    result = []
    for s in samples:
        val = max(0, min(max_val, s))
        result.append(int(val / max_val * (levels - 1)))
    return result


def generate_c_array(values, name='waveform_data', elem_type='uint16_t'):
    """生成C语言数组"""
    lines = [f"// 自动生成的波形数据 ({len(values)}个采样点)"]
    lines.append(f"const {elem_type} {name}[{len(values)}] = {{")
    # 每行8个值
    for i in range(0, len(values), 8):
        chunk = values[i:i+8]
        line = "    " + ", ".join(f"{v}" for v in chunk)
        if i + 8 < len(values):
            line += ","
        lines.append(line)
    lines.append("};")
    return "\n".join(lines)


# ── DAC芯片适配 ───────────────────────────────────────────────
DAC_CONFIGS = {
    'dac0808': {'bits': 8, 'vref': 5.0, 'settling_us': 1},
    'tlv5618': {'bits': 12, 'vref': 3.3, 'settling_us': 3},
    'mcp4725': {'bits': 12, 'vref': 3.3, 'settling_us': 6},
    'ad9767': {'bits': 14, 'vref': 3.3, 'settling_us': 0.035},
    'internal_12bit': {'bits': 12, 'vref': 3.3, 'settling_us': 1},
    'internal_8bit': {'bits': 8, 'vref': 3.3, 'settling_us': 1},
}


# ── CLI ──────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description='信号生成工具 - 多种波形生成')
    sub = parser.add_subparsers(dest='cmd')

    # 通用参数
    def add_common_args(p):
        p.add_argument('--freq', type=float, default=1000, help='频率Hz')
        p.add_argument('--amp', type=float, default=1.0, help='幅度V')
        p.add_argument('--offset', type=float, default=1.65, help='偏移V')
        p.add_argument('--rate', type=int, default=100000, help='采样率Hz')
        p.add_argument('--samples', type=int, default=1000, help='采样点数')
        p.add_argument('--dac', default='internal_12bit', choices=list(DAC_CONFIGS.keys()), help='DAC芯片')
        p.add_argument('--out', help='输出文件(.c/.csv/.json)')

    # 正弦波
    p_sine = sub.add_parser('sine', help='正弦波')
    add_common_args(p_sine)
    p_sine.add_argument('--phase', type=float, default=0, help='初始相位°')

    # 方波
    p_sq = sub.add_parser('square', help='方波')
    add_common_args(p_sq)
    p_sq.add_argument('--duty', type=float, default=0.5, help='占空比0-1')

    # 三角波
    p_tri = sub.add_parser('triangle', help='三角波')
    add_common_args(p_tri)

    # 锯齿波
    p_saw = sub.add_parser('sawtooth', help='锯齿波')
    add_common_args(p_saw)

    # 脉冲
    p_pulse = sub.add_parser('pulse', help='脉冲信号')
    add_common_args(p_pulse)
    p_pulse.add_argument('--rise', type=float, default=10, help='上升沿ns')
    p_pulse.add_argument('--fall', type=float, default=10, help='下降沿ns')

    # AM调制
    p_am = sub.add_parser('am', help='AM调幅')
    add_common_args(p_am)
    p_am.add_argument('--mod-freq', type=float, default=100, help='调制频率Hz')
    p_am.add_argument('--mod-depth', type=float, default=0.5, help='调制深度0-1')

    # 噪声
    p_noise = sub.add_parser('noise', help='噪声信号')
    p_noise.add_argument('--amp', type=float, default=0.1, help='幅度')
    p_noise.add_argument('--type', default='white', choices=['white', 'uniform'])
    p_noise.add_argument('--offset', type=float, default=0)
    p_noise.add_argument('--samples', type=int, default=1000)
    p_noise.add_argument('--out', help='输出文件')

    # 多音
    p_multi = sub.add_parser('multitone', help='多音叠加')
    p_multi.add_argument('--tones', required=True, help='多音JSON [{freq,amplitude,phase_deg}]')
    p_multi.add_argument('--rate', type=int, default=100000)
    p_multi.add_argument('--samples', type=int, default=1000)
    p_multi.add_argument('--dac', default='internal_12bit', choices=list(DAC_CONFIGS.keys()))
    p_multi.add_argument('--out', help='输出文件')

    args = parser.parse_args()
    if not args.cmd:
        parser.print_help()
        return

    # 生成波形
    if args.cmd == 'sine':
        samples = generate_sine(args.freq, args.amp, args.phase, args.offset, args.rate, args.samples)
    elif args.cmd == 'square':
        samples = generate_square(args.freq, args.amp, args.duty, args.offset, args.rate, args.samples)
    elif args.cmd == 'triangle':
        samples = generate_triangle(args.freq, args.amp, args.offset, args.rate, args.samples)
    elif args.cmd == 'sawtooth':
        samples = generate_sawtooth(args.freq, args.amp, args.offset, args.rate, args.samples)
    elif args.cmd == 'pulse':
        samples = generate_pulse(args.freq, args.amp, args.rise, args.fall, args.offset, args.rate, args.samples)
    elif args.cmd == 'am':
        samples = generate_am_modulation(args.freq, args.mod_freq, args.mod_depth, args.amp, args.offset, args.rate, args.samples)
    elif args.cmd == 'noise':
        samples = generate_noise(args.amp, args.type, args.offset, args.samples)
    elif args.cmd == 'multitone':
        tones = json.loads(args.tones)
        samples = generate_multitone(tones, args.rate, args.samples)
    else:
        print("未知命令")
        return

    # DAC量化
    dac = DAC_CONFIGS.get(args.dac, DAC_CONFIGS['internal_12bit'])
    quantized = quantize_signal(samples, dac['bits'], dac['vref'])

    # 输出
    output_lines = []
    if args.out:
        if args.out.endswith('.c'):
            c_code = generate_c_array(quantized)
            with open(args.out, 'w', encoding='utf-8') as f:
                f.write(c_code)
            print(f"C数组已保存: {args.out} ({len(quantized)}点)")
            return
        elif args.out.endswith('.csv'):
            with open(args.out, 'w') as f:
                f.write("index,raw_value,dac_value\n")
                for i, (raw, q) in enumerate(zip(samples, quantized)):
                    f.write(f"{i},{raw:.6f},{q}\n")
            print(f"CSV已保存: {args.out}")
            return
        elif args.out.endswith('.json'):
            with open(args.out, 'w') as f:
                json.dump({'samples': [round(s, 6) for s in samples],
                           'quantized': quantized,
                           'dac': args.dac,
                           'config': dac}, f, indent=2)
            print(f"JSON已保存: {args.out}")
            return

    # 默认输出到终端（前20个点）
    print(f"波形类型: {args.cmd}, 采样点数: {len(samples)}")
    print(f"DAC: {args.dac} ({dac['bits']}bit, Vref={dac['vref']}V)")
    print(f"\n前20个采样点 (原始值 -> DAC值):")
    for i in range(min(20, len(samples))):
        print(f"  [{i:4d}] {samples[i]:>10.6f} V -> {quantized[i]}")
    if len(samples) > 20:
        print(f"  ... (共{len(samples)}点)")


if __name__ == '__main__':
    main()
