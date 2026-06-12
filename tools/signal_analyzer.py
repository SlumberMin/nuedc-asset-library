#!/usr/bin/env python3
"""
信号分析工具 - nuedc-asset-library
功能：FFT频谱分析、THD计算、SNR计算、ENOB计算、功率谱密度
作者：电赛自动迭代引擎 V3
"""

import argparse
import json
import sys
import math
import struct
import os

# ─── 纯Python FFT实现（Cooley-Tukey基2） ───────────────────────────

def _fft_radix2(x):
    """基2 FFT，输入长度必须为2的幂"""
    N = len(x)
    if N <= 1:
        return x
    if N & (N - 1):
        raise ValueError(f"FFT长度必须为2的幂，当前长度={N}")
    # 位反转排列
    bits = int(math.log2(N))
    j = 0
    a = list(x)
    for i in range(N):
        if i < j:
            a[i], a[j] = a[j], a[i]
        mask = N >> 1
        while j & mask:
            j ^= mask
            mask >>= 1
        j |= mask
    # 蝶形运算
    length = 2
    while length <= N:
        half = length // 2
        angle = -2.0 * math.pi / length
        wn_r = math.cos(angle)
        wn_i = math.sin(angle)
        for start in range(0, N, length):
            w_r, w_i = 1.0, 0.0
            for k in range(half):
                u = a[start + k]
                v_r = a[start + k + half].real * w_r - a[start + k + half].imag * w_i
                v_i = a[start + k + half].real * w_i + a[start + k + half].imag * w_r
                a[start + k] = complex(u.real + v_r, u.imag + v_i)
                a[start + k + half] = complex(u.real - v_r, u.imag - v_i)
                new_w_r = w_r * wn_r - w_i * wn_i
                new_w_i = w_r * wn_i + w_i * wn_r
                w_r, w_i = new_w_r, new_w_i
        length <<= 1
    return a

def _next_pow2(n):
    """返回>=n的最小2的幂"""
    p = 1
    while p < n:
        p <<= 1
    return p

def _pad_to_pow2(x):
    """补零到2的幂长度"""
    N = len(x)
    target = _next_pow2(N)
    return x + [0.0] * (target - N), N  # 返回补零后数据和原始长度


# ─── 频谱分析 ────────────────────────────────────────────────────

def fft_spectrum(signal, fs, n_fft=None):
    """
    计算FFT频谱
    返回: {freqs[], magnitudes[], phases[], n_fft, df}
    """
    N = len(signal)
    if n_fft is None:
        n_fft = _next_pow2(N)
    # 加窗（汉宁窗）
    windowed = []
    for i in range(N):
        w = 0.5 * (1.0 - math.cos(2.0 * math.pi * i / (N - 1)))
        windowed.append(signal[i] * w)
    # 补零
    while len(windowed) < n_fft:
        windowed.append(0.0)
    # FFT
    X = _fft_radix2([complex(v, 0) for v in windowed])
    # 取单边谱
    half = n_fft // 2
    df = fs / n_fft
    freqs = [i * df for i in range(half)]
    magnitudes = [2.0 * abs(X[i]) / N for i in range(half)]
    phases = [math.atan2(X[i].imag, X[i].real) for i in range(half)]
    return {"freqs": freqs, "magnitudes": magnitudes, "phases": phases,
            "n_fft": n_fft, "df": df, "fs": fs}


def power_spectral_density(signal, fs, n_fft=None):
    """
    功率谱密度(PSD)，单位 dB/Hz
    返回: {freqs[], psd_db[]}
    """
    spec = fft_spectrum(signal, fs, n_fft)
    freqs = spec["freqs"]
    mags = spec["magnitudes"]
    psd_db = []
    for m in mags:
        p = (m / 2.0) ** 2
        psd_db.append(10.0 * math.log10(max(p, 1e-30)))
    return {"freqs": freqs, "psd_db": psd_db}


# ─── THD (总谐波失真) ────────────────────────────────────────────

def calc_thd(signal, fs, f0=None, n_fft=None, num_harmonics=5):
    """
    计算总谐波失真 THD
    f0: 基波频率，None则自动检测
    返回: {thd_percent, thd_db, f0, harmonics[], fundamental_power, harmonic_powers[]}
    """
    spec = fft_spectrum(signal, fs, n_fft)
    freqs = spec["freqs"]
    mags = spec["magnitudes"]
    df = spec["df"]

    # 找基波（最大分量，排除DC附近）
    min_idx = max(1, int(20 / df))  # 排除20Hz以下
    if f0 is None:
        peak_idx = min_idx
        for i in range(min_idx, len(mags)):
            if mags[i] > mags[peak_idx]:
                peak_idx = i
        f0 = freqs[peak_idx]
    else:
        peak_idx = round(f0 / df)

    fund_power = mags[peak_idx] ** 2
    harm_powers = []
    harmonics = []
    for h in range(2, num_harmonics + 2):
        h_idx = round(h * f0 / df)
        if h_idx < len(mags):
            hp = mags[h_idx] ** 2
            harm_powers.append(hp)
            harmonics.append({"harmonic": h, "freq": h * f0,
                              "magnitude": mags[h_idx],
                              "power_ratio_db": 10 * math.log10(max(hp, 1e-30) / max(fund_power, 1e-30))})
        else:
            harm_powers.append(0.0)
            harmonics.append({"harmonic": h, "freq": h * f0, "magnitude": 0, "power_ratio_db": -999})

    total_harm_power = sum(harm_powers)
    thd = math.sqrt(total_harm_power / max(fund_power, 1e-30))
    thd_percent = thd * 100.0
    thd_db = 20.0 * math.log10(max(thd, 1e-15))

    return {"thd_percent": thd_percent, "thd_db": thd_db,
            "f0": f0, "harmonics": harmonics,
            "fundamental_power": fund_power,
            "harmonic_powers": harm_powers}


# ─── SNR (信噪比) ────────────────────────────────────────────────

def calc_snr(signal, fs, signal_freq, n_fft=None, noise_bandwidth=None):
    """
    计算信噪比 SNR
    signal_freq: 信号频率(Hz)
    noise_bandwidth: 噪声积分带宽(Hz)，默认 fs/2
    返回: {snr_db, signal_power, noise_power, signal_freq}
    """
    spec = fft_spectrum(signal, fs, n_fft)
    freqs = spec["freqs"]
    mags = spec["magnitudes"]
    df = spec["df"]

    if noise_bandwidth is None:
        noise_bandwidth = fs / 2.0

    # 找信号频率附近的最大分量
    sig_idx = round(signal_freq / df)
    search_range = max(2, int(50 / df))
    peak_idx = max(0, sig_idx - search_range)
    for i in range(max(0, sig_idx - search_range), min(len(mags), sig_idx + search_range + 1)):
        if mags[i] > mags[peak_idx]:
            peak_idx = i

    signal_power = mags[peak_idx] ** 2
    # 噪声功率 = 总功率 - 信号功率
    noise_threshold_idx = round(noise_bandwidth / df)
    total_power = sum(m ** 2 for m in mags[:noise_threshold_idx])
    noise_power = max(total_power - signal_power, 1e-30)
    snr_db = 10.0 * math.log10(signal_power / noise_power)

    return {"snr_db": snr_db, "signal_power": signal_power,
            "noise_power": noise_power, "signal_freq": freqs[peak_idx]}


# ─── ENOB (有效位数) ─────────────────────────────────────────────

def calc_enob(signal, fs, signal_freq, n_fft=None):
    """
    计算有效位数 ENOB
    ENOB = (SINAD - 1.76) / 6.02
    返回: {enob, sinad_db, snr_db, thd_percent}
    """
    # 先计算SNR
    snr_result = calc_snr(signal, fs, signal_freq, n_fft)
    # 计算THD
    thd_result = calc_thd(signal, fs, signal_freq, n_fft)

    # SINAD = 总信号功率 / (噪声功率 + 谐波功率)
    total_distortion = sum(thd_result["harmonic_powers"])
    noise_power = snr_result["noise_power"]
    denom = max(noise_power + total_distortion, 1e-30)
    sinad = snr_result["signal_power"] / denom
    sinad_db = 10.0 * math.log10(sinad)

    enob = (sinad_db - 1.76) / 6.02

    return {"enob": enob, "sinad_db": sinad_db,
            "snr_db": snr_result["snr_db"],
            "thd_percent": thd_result["thd_percent"]}


# ─── 信号生成（测试用） ───────────────────────────────────────────

def generate_test_signal(fs, duration, f0, amplitude=1.0, thd_target=0.0, noise_level=0.0):
    """生成测试信号：正弦波 + 可选谐波失真 + 白噪声"""
    N = int(fs * duration)
    signal = []
    for i in range(N):
        t = i / fs
        v = amplitude * math.sin(2 * math.pi * f0 * t)
        # 添加谐波失真
        if thd_target > 0:
            for h in range(2, 6):
                harm_amp = amplitude * thd_target / (h * h)
                v += harm_amp * math.sin(2 * math.pi * h * f0 * t)
        # 添加噪声（简易伪随机）
        if noise_level > 0:
            # Box-Muller transform
            import random
            random.seed(i + 42)
            u1 = random.random()
            u2 = random.random()
            noise = noise_level * math.sqrt(-2 * math.log(max(u1, 1e-10))) * math.cos(2 * math.pi * u2)
            v += noise
        signal.append(v)
    return signal


# ─── 文件读取 ────────────────────────────────────────────────────

def load_csv(filepath, column=0, skip_header=True):
    """从CSV加载信号数据"""
    data = []
    with open(filepath, 'r', encoding='utf-8') as f:
        for i, line in enumerate(f):
            if i == 0 and skip_header:
                continue
            parts = line.strip().split(',')
            if len(parts) > column:
                try:
                    data.append(float(parts[column]))
                except ValueError:
                    continue
    return data


def load_binary(filepath, fmt='h', count=None):
    """从二进制文件加载（ADC原始数据）"""
    fmt_map = {'h': 2, 'H': 2, 'i': 4, 'I': 4, 'f': 4, 'd': 8}
    byte_size = fmt_map.get(fmt, 2)
    if count is None:
        fsize = os.path.getsize(filepath)
        count = fsize // byte_size
    with open(filepath, 'rb') as f:
        data = struct.unpack(f'<{count}{fmt}', f.read(count * byte_size))
    return list(data)


# ─── CLI ─────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='信号分析工具 - 电赛资产库', formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='command', help='子命令')

    # fft 子命令
    p_fft = sub.add_parser('fft', help='FFT频谱分析')
    p_fft.add_argument('--input', '-i', required=True, help='输入CSV文件')
    p_fft.add_argument('--fs', type=float, required=True, help='采样率(Hz)')
    p_fft.add_argument('--column', type=int, default=0, help='数据列(默认0)')
    p_fft.add_argument('--nfft', type=int, default=None, help='FFT点数')
    p_fft.add_argument('--output', '-o', help='输出JSON文件')
    p_fft.add_argument('--top', type=int, default=10, help='显示前N个频率分量')

    # thd 子命令
    p_thd = sub.add_parser('thd', help='THD总谐波失真')
    p_thd.add_argument('--input', '-i', required=True, help='输入CSV文件')
    p_thd.add_argument('--fs', type=float, required=True, help='采样率(Hz)')
    p_thd.add_argument('--f0', type=float, default=None, help='基波频率(Hz)')
    p_thd.add_argument('--column', type=int, default=0, help='数据列')
    p_thd.add_argument('--harmonics', type=int, default=5, help='谐波次数')

    # snr 子命令
    p_snr = sub.add_parser('snr', help='SNR信噪比')
    p_snr.add_argument('--input', '-i', required=True, help='输入CSV文件')
    p_snr.add_argument('--fs', type=float, required=True, help='采样率(Hz)')
    p_snr.add_argument('--signal-freq', type=float, required=True, help='信号频率(Hz)')
    p_snr.add_argument('--column', type=int, default=0, help='数据列')

    # enob 子命令
    p_enob = sub.add_parser('enob', help='ENOB有效位数')
    p_enob.add_argument('--input', '-i', required=True, help='输入CSV文件')
    p_enob.add_argument('--fs', type=float, required=True, help='采样率(Hz)')
    p_enob.add_argument('--signal-freq', type=float, required=True, help='信号频率(Hz)')
    p_enob.add_argument('--column', type=int, default=0, help='数据列')

    # psd 子命令
    p_psd = sub.add_parser('psd', help='功率谱密度')
    p_psd.add_argument('--input', '-i', required=True, help='输入CSV文件')
    p_psd.add_argument('--fs', type=float, required=True, help='采样率(Hz)')
    p_psd.add_argument('--column', type=int, default=0, help='数据列')

    # generate 子命令
    p_gen = sub.add_parser('generate', help='生成测试信号')
    p_gen.add_argument('--fs', type=float, required=True, help='采样率(Hz)')
    p_gen.add_argument('--duration', type=float, required=True, help='时长(s)')
    p_gen.add_argument('--f0', type=float, required=True, help='信号频率(Hz)')
    p_gen.add_argument('--amplitude', type=float, default=1.0, help='幅度')
    p_gen.add_argument('--thd', type=float, default=0.0, help='目标THD(0~1)')
    p_gen.add_argument('--noise', type=float, default=0.0, help='噪声幅度')
    p_gen.add_argument('--output', '-o', required=True, help='输出CSV')

    args = parser.parse_args()

    if args.command is None:
        parser.print_help()
        return

    if args.command == 'generate':
        sig = generate_test_signal(args.fs, args.duration, args.f0,
                                    args.amplitude, args.thd, args.noise)
        with open(args.output, 'w', encoding='utf-8') as f:
            f.write('sample,value\n')
            for i, v in enumerate(sig):
                f.write(f'{i},{v:.10f}\n')
        print(f'已生成 {len(sig)} 个采样点 -> {args.output}')
        return

    # 加载数据
    sig = load_csv(args.input, args.column)
    print(f'加载了 {len(sig)} 个采样点')

    if args.command == 'fft':
        result = fft_spectrum(sig, args.fs, args.nfft)
        # 按幅度排序显示top频率分量
        indexed = sorted(enumerate(result["magnitudes"]),
                        key=lambda x: x[1], reverse=True)
        print(f'\nFFT结果 (N={result["n_fft"]}, df={result["df"]:.2f}Hz):')
        print(f'{"排名":<6} {"频率(Hz)":<14} {"幅度":<14} {"相位(度)":<14}')
        print('-' * 48)
        for rank, (idx, mag) in enumerate(indexed[:args.top], 1):
            phase_deg = result["phases"][idx] * 180.0 / math.pi
            print(f'{rank:<6} {result["freqs"][idx]:<14.4f} {mag:<14.6f} {phase_deg:<14.2f}')
        if args.output:
            with open(args.output, 'w', encoding='utf-8') as f:
                json.dump({"freqs": result["freqs"][:200],
                          "magnitudes": result["magnitudes"][:200]}, f, indent=2)
            print(f'\n完整频谱已保存至 {args.output}')

    elif args.command == 'thd':
        result = calc_thd(sig, args.fs, args.f0, num_harmonics=args.harmonics)
        print(f'\nTHD分析结果:')
        print(f'  基波频率: {result["f0"]:.4f} Hz')
        print(f'  THD: {result["thd_percent"]:.4f}% ({result["thd_db"]:.2f} dB)')
        print(f'\n  {"谐波":<8} {"频率(Hz)":<14} {"幅度":<14} {"功率比(dB)":<14}')
        print('  ' + '-' * 50)
        for h in result["harmonics"]:
            print(f'  {h["harmonic"]:<8} {h["freq"]:<14.4f} {h["magnitude"]:<14.6f} {h["power_ratio_db"]:<14.2f}')

    elif args.command == 'snr':
        result = calc_snr(sig, args.fs, args.signal_freq)
        print(f'\nSNR分析结果:')
        print(f'  信号频率: {result["signal_freq"]:.4f} Hz')
        print(f'  SNR: {result["snr_db"]:.2f} dB')
        print(f'  信号功率: {result["signal_power"]:.6e}')
        print(f'  噪声功率: {result["noise_power"]:.6e}')

    elif args.command == 'enob':
        result = calc_enob(sig, args.fs, args.signal_freq)
        print(f'\nENOB分析结果:')
        print(f'  有效位数(ENOB): {result["enob"]:.2f} bits')
        print(f'  SINAD: {result["sinad_db"]:.2f} dB')
        print(f'  SNR: {result["snr_db"]:.2f} dB')
        print(f'  THD: {result["thd_percent"]:.4f}%')

    elif args.command == 'psd':
        result = power_spectral_density(sig, args.fs)
        print(f'\nPSD (前20个频率分量):')
        print(f'{"频率(Hz)":<14} {"PSD(dB/Hz)":<14}')
        print('-' * 28)
        for i in range(min(20, len(result["freqs"]))):
            print(f'{result["freqs"][i]:<14.4f} {result["psd_db"][i]:<14.2f}')


if __name__ == '__main__':
    main()
