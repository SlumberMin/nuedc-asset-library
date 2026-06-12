#!/usr/bin/env python3
"""
通信仿真工具 - nuedc-asset-library
功能：数字调制(ASK/FSK/PSK/QAM)、信道编码、AWGN信道、误码率计算
作者：电赛自动迭代引擎 V3
"""

import argparse
import json
import math
import random
import sys


# ─── 随机比特流生成 ───────────────────────────────────────────────

def generate_bits(n, seed=42):
    """生成n个随机比特"""
    rng = random.Random(seed)
    return [rng.randint(0, 1) for _ in range(n)]


# ─── 数字调制 ────────────────────────────────────────────────────

def modulate_ask(bits, fc, fs, bit_rate, amplitude_high=1.0, amplitude_low=0.0):
    """
    ASK (幅移键控) 调制
    返回: {samples[], t[], fs, bit_rate}
    """
    samples_per_bit = int(fs / bit_rate)
    signal = []
    t = []
    for i, bit in enumerate(bits):
        amp = amplitude_high if bit == 1 else amplitude_low
        for j in range(samples_per_bit):
            idx = i * samples_per_bit + j
            tt = idx / fs
            signal.append(amp * math.sin(2 * math.pi * fc * tt))
            t.append(tt)
    return {"signal": signal, "t": t, "fs": fs, "bit_rate": bit_rate, "modulation": "ASK"}


def modulate_fsk(bits, fc, fs, bit_rate, freq_dev=1000):
    """
    FSK (频移键控) 调制
    f1 = fc + freq_dev (bit=1), f0 = fc - freq_dev (bit=0)
    """
    samples_per_bit = int(fs / bit_rate)
    signal = []
    t = []
    for i, bit in enumerate(bits):
        freq = fc + freq_dev if bit == 1 else fc - freq_dev
        for j in range(samples_per_bit):
            idx = i * samples_per_bit + j
            tt = idx / fs
            signal.append(math.sin(2 * math.pi * freq * tt))
            t.append(tt)
    return {"signal": signal, "t": t, "fs": fs, "bit_rate": bit_rate, "modulation": "FSK"}


def modulate_bpsk(bits, fc, fs, bit_rate):
    """
    BPSK (二进制相移键控) 调制
    bit=1: 相位0, bit=0: 相位π
    """
    samples_per_bit = int(fs / bit_rate)
    signal = []
    t = []
    for i, bit in enumerate(bits):
        phase = 0 if bit == 1 else math.pi
        for j in range(samples_per_bit):
            idx = i * samples_per_bit + j
            tt = idx / fs
            signal.append(math.sin(2 * math.pi * fc * tt + phase))
            t.append(tt)
    return {"signal": signal, "t": t, "fs": fs, "bit_rate": bit_rate, "modulation": "BPSK"}


def modulate_qpsk(bits, fc, fs, bit_rate):
    """
    QPSK (四相相移键控) 调制
    每2bit映射一个符号
    """
    samples_per_symbol = int(fs / bit_rate) * 2  # 每符号2bit
    # 映射表: 00->π/4, 01->3π/4, 11->5π/4, 10->7π/4
    phase_map = {(0,0): math.pi/4, (0,1): 3*math.pi/4,
                 (1,1): 5*math.pi/4, (1,0): 7*math.pi/4}
    signal = []
    t = []
    symbol_idx = 0
    for i in range(0, len(bits) - 1, 2):
        pair = (bits[i], bits[i+1])
        phase = phase_map.get(pair, 0)
        for j in range(samples_per_symbol):
            idx = symbol_idx * samples_per_symbol + j
            tt = idx / fs
            signal.append(math.sin(2 * math.pi * fc * tt + phase))
            t.append(tt)
        symbol_idx += 1
    return {"signal": signal, "t": t, "fs": fs, "bit_rate": bit_rate, "modulation": "QPSK",
            "symbols": symbol_idx}


def modulate_16qam(bits, fc, fs, bit_rate):
    """
    16-QAM 调制
    每4bit映射一个复数符号
    """
    samples_per_symbol = int(fs / bit_rate) * 4
    # Gray编码映射（简化）
    amp_levels = [-3, -1, 1, 3]
    signal = []
    t = []
    symbol_idx = 0
    for i in range(0, len(bits) - 3, 4):
        # 4bit -> I, Q
        i_val = amp_levels[bits[i]*2 + bits[i+1]]
        q_val = amp_levels[bits[i+2]*2 + bits[i+3]]
        # 归一化
        norm = math.sqrt(10)
        i_val /= norm
        q_val /= norm
        for j in range(samples_per_symbol):
            idx = symbol_idx * samples_per_symbol + j
            tt = idx / fs
            signal.append(i_val * math.sin(2*math.pi*fc*tt) +
                         q_val * math.cos(2*math.pi*fc*tt))
            t.append(tt)
        symbol_idx += 1
    return {"signal": signal, "t": t, "fs": fs, "bit_rate": bit_rate,
            "modulation": "16QAM", "symbols": symbol_idx}


# ─── 信道编码 ────────────────────────────────────────────────────

def hamming_encode(data_bits):
    """
    Hamming(7,4)编码
    输入: 4bit数据 -> 输出: 7bit编码
    返回: 编码后的比特列表
    """
    encoded = []
    for i in range(0, len(data_bits) - 3, 4):
        d = data_bits[i:i+4]
        # 校验位计算
        p1 = d[0] ^ d[1] ^ d[3]
        p2 = d[0] ^ d[2] ^ d[3]
        p3 = d[1] ^ d[2] ^ d[3]
        encoded.extend([p1, p2, d[0], p3, d[1], d[2], d[3]])
    return encoded


def hamming_decode(encoded_bits):
    """
    Hamming(7,4)解码（含纠错）
    返回: {data_bits, errors_corrected}
    """
    data_bits = []
    errors = 0
    for i in range(0, len(encoded_bits) - 6, 7):
        c = encoded_bits[i:i+7]
        if len(c) < 7:
            break
        # 计算校验子
        s1 = c[0] ^ c[2] ^ c[4] ^ c[6]
        s2 = c[1] ^ c[2] ^ c[5] ^ c[6]
        s3 = c[3] ^ c[4] ^ c[5] ^ c[6]
        syndrome = s1 + (s2 << 1) + (s3 << 2)
        if syndrome != 0:
            # 纠正错误位
            c[syndrome - 1] ^= 1
            errors += 1
        data_bits.extend([c[2], c[4], c[5], c[6]])
    return {"data_bits": data_bits, "errors_corrected": errors}


def convolutional_encode(bits, constraint_length=3):
    """
    卷积编码 (K=3, rate=1/2)
    生成多项式: g1=111(7), g2=101(5)
    """
    g1 = 0b111
    g2 = 0b101
    K = constraint_length
    state = 0
    encoded = []
    for bit in bits:
        state = ((state << 1) | bit) & ((1 << K) - 1)
        out1 = bin(state & g1).count('1') % 2
        out2 = bin(state & g2).count('1') % 2
        encoded.extend([out1, out2])
    return encoded


def viterbi_decode(encoded_bits, constraint_length=3):
    """
    Viterbi译码（简化版，K=3, rate=1/2）
    """
    g1 = 0b111
    g2 = 0b101
    K = constraint_length
    n_states = 1 << (K - 1)

    # 初始化路径度量
    path_metrics = [float('inf')] * n_states
    path_metrics[0] = 0
    paths = [[] for _ in range(n_states)]

    for i in range(0, len(encoded_bits) - 1, 2):
        rx = (encoded_bits[i], encoded_bits[i+1])
        new_metrics = [float('inf')] * n_states
        new_paths = [[] for _ in range(n_states)]

        for state in range(n_states):
            if path_metrics[state] == float('inf'):
                continue
            for bit in [0, 1]:
                next_state = ((state << 1) | bit) & ((1 << K) - 1)
                # 计算期望输出
                exp1 = bin(next_state & g1).count('1') % 2
                exp2 = bin(next_state & g2).count('1') % 2
                # 距离
                dist = (rx[0] - exp1)**2 + (rx[1] - exp2)**2
                metric = path_metrics[state] + dist
                if metric < new_metrics[next_state]:
                    new_metrics[next_state] = metric
                    new_paths[next_state] = paths[state] + [bit]

        path_metrics = new_metrics
        paths = new_paths

    # 选择最优路径
    best = min(range(n_states), key=lambda s: path_metrics[s])
    return {"data_bits": paths[best], "metric": path_metrics[best]}


# ─── 信道模型 ────────────────────────────────────────────────────

def awgn_channel(signal, snr_db):
    """
    AWGN信道
    snr_db: 信噪比(dB)
    """
    # 计算信号功率
    sig_power = sum(s**2 for s in signal) / len(signal)
    snr_linear = 10 ** (snr_db / 10)
    noise_power = sig_power / snr_linear
    noise_std = math.sqrt(noise_power)

    rng = random.Random(123)
    noisy = []
    for s in signal:
        # Box-Muller
        u1 = max(rng.random(), 1e-10)
        u2 = rng.random()
        noise = noise_std * math.sqrt(-2 * math.log(u1)) * math.cos(2 * math.pi * u2)
        noisy.append(s + noise)
    return noisy


def rayleigh_channel(signal, snr_db, n_taps=3):
    """
    瑞利衰落信道（简化多径模型）
    """
    rng = random.Random(456)
    # 生成瑞利衰落系数
    taps = []
    for _ in range(n_taps):
        re = rng.gauss(0, 1/math.sqrt(2))
        im = rng.gauss(0, 1/math.sqrt(2))
        taps.append(math.sqrt(re**2 + im**2))

    # 多径
    output = [0.0] * len(signal)
    for tap_idx, h in enumerate(taps):
        for i in range(len(signal)):
            if i - tap_idx >= 0:
                output[i] += h * signal[i - tap_idx]

    # 加噪声
    output = awgn_channel(output, snr_db)
    return output


# ─── BPSK解调 ────────────────────────────────────────────────────

def demodulate_bpsk(signal, fc, fs, bit_rate):
    """BPSK相干解调"""
    samples_per_bit = int(fs / bit_rate)
    bits = []
    for i in range(0, len(signal) - samples_per_bit + 1, samples_per_bit):
        # 相干检测：与载波相乘并积分
        corr = 0.0
        for j in range(samples_per_bit):
            t = (i + j) / fs
            corr += signal[i + j] * math.sin(2 * math.pi * fc * t)
        corr /= samples_per_bit
        bits.append(1 if corr > 0 else 0)
    return bits


# ─── BER计算 ─────────────────────────────────────────────────────

def calc_ber(tx_bits, rx_bits):
    """计算误码率"""
    n = min(len(tx_bits), len(rx_bits))
    errors = sum(1 for i in range(n) if tx_bits[i] != rx_bits[i])
    return {"ber": errors / max(n, 1), "errors": errors, "total": n}


def bpsk_theoretical_ber(snr_db):
    """BPSK理论误码率"""
    snr_linear = 10 ** (snr_db / 10)
    # Q函数近似
    x = math.sqrt(2 * snr_linear)
    # 使用erfc近似: BER = 0.5 * erfc(sqrt(Eb/N0))
    # erfc近似
    t = 1.0 / (1.0 + 0.3275911 * x / math.sqrt(2))
    poly = t * (0.254829592 + t * (-0.284496736 + t * (1.421413741 + t * (-1.453152027 + t * 1.061405429))))
    erfc_approx = poly * math.exp(-x**2 / 2)
    return 0.5 * erfc_approx


# ─── CSV加载 ─────────────────────────────────────────────────────

def load_csv_signal(filepath, column=0):
    data = []
    with open(filepath, 'r', encoding='utf-8') as f:
        for i, line in enumerate(f):
            if i == 0:
                continue
            parts = line.strip().split(',')
            if len(parts) > column:
                try:
                    data.append(float(parts[column]))
                except ValueError:
                    continue
    return data


# ─── CLI ─────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='通信仿真工具 - 电赛资产库')
    sub = parser.add_subparsers(dest='command')

    # 调制
    p_mod = sub.add_parser('modulate', help='数字调制')
    p_mod.add_argument('--type', choices=['ask', 'fsk', 'bpsk', 'qpsk', '16qam'],
                       default='bpsk', help='调制方式')
    p_mod.add_argument('--bits', type=int, default=100, help='比特数')
    p_mod.add_argument('--fc', type=float, default=10000, help='载波频率(Hz)')
    p_mod.add_argument('--fs', type=float, default=100000, help='采样率(Hz)')
    p_mod.add_argument('--bit-rate', type=float, default=1000, help='比特率(bps)')
    p_mod.add_argument('--freq-dev', type=float, default=1000, help='FSK频偏(Hz)')
    p_mod.add_argument('--snr', type=float, default=None, help='信噪比(dB)，加AWGN噪声')
    p_mod.add_argument('--output', '-o', help='输出CSV')

    # 编码
    p_enc = sub.add_parser('encode', help='信道编码')
    p_enc.add_argument('--type', choices=['hamming', 'convolutional'],
                       default='hamming')
    p_enc.add_argument('--bits', type=int, default=16, help='数据比特数')
    p_enc.add_argument('--seed', type=int, default=42)
    p_enc.add_argument('--decode-test', action='store_true', help='编解码往返测试')
    p_enc.add_argument('--error-rate', type=float, default=0.01, help='注入错误率')

    # BER测试
    p_ber = sub.add_parser('ber', help='误码率测试')
    p_ber.add_argument('--type', choices=['bpsk'], default='bpsk')
    p_ber.add_argument('--snr-start', type=float, default=0, help='起始SNR(dB)')
    p_ber.add_argument('--snr-stop', type=float, default=12, help='终止SNR(dB)')
    p_ber.add_argument('--snr-step', type=float, default=1, help='SNR步长(dB)')
    p_ber.add_argument('--bits', type=int, default=10000, help='测试比特数')
    p_ber.add_argument('--fc', type=float, default=10000)
    p_ber.add_argument('--fs', type=float, default=100000)
    p_ber.add_argument('--bit-rate', type=float, default=1000)
    p_ber.add_argument('--output', '-o', help='输出BER曲线JSON')

    # 理论BER
    p_theory = sub.add_parser('theory', help='理论误码率计算')
    p_theory.add_argument('--type', choices=['bpsk'], default='bpsk')
    p_theory.add_argument('--snr-start', type=float, default=-2)
    p_theory.add_argument('--snr-stop', type=float, default=12)
    p_theory.add_argument('--snr-step', type=float, default=1)

    args = parser.parse_args()

    if args.command is None:
        parser.print_help()
        return

    if args.command == 'modulate':
        bits = generate_bits(args.bits)
        print(f'生成 {args.bits} 个随机比特')

        if args.type == 'ask':
            result = modulate_ask(bits, args.fc, args.fs, args.bit_rate)
        elif args.type == 'fsk':
            result = modulate_fsk(bits, args.fc, args.fs, args.bit_rate, args.freq_dev)
        elif args.type == 'bpsk':
            result = modulate_bpsk(bits, args.fc, args.fs, args.bit_rate)
        elif args.type == 'qpsk':
            result = modulate_qpsk(bits, args.fc, args.fs, args.bit_rate)
        else:
            result = modulate_16qam(bits, args.fc, args.fs, args.bit_rate)

        sig = result["signal"]
        # 加噪声
        if args.snr is not None:
            sig = awgn_channel(sig, args.snr)
            print(f'已添加AWGN噪声 (SNR={args.snr}dB)')

        print(f'调制方式: {args.type.upper()}')
        print(f'采样点数: {len(sig)}')
        print(f'信号时长: {len(sig)/args.fs:.4f}s')

        if args.output:
            with open(args.output, 'w') as f:
                f.write('sample,value\n')
                for i, v in enumerate(sig):
                    f.write(f'{i},{v:.10f}\n')
            print(f'已保存至 {args.output}')

    elif args.command == 'encode':
        bits = generate_bits(args.bits, args.seed)
        print(f'原始比特 ({len(bits)}): {bits[:20]}...')

        if args.type == 'hamming':
            encoded = hamming_encode(bits)
            print(f'Hamming编码后 ({len(encoded)}): {encoded[:28]}...')

            if args.decode_test:
                # 注入错误
                err_count = max(1, int(len(encoded) * args.error_rate))
                rng = random.Random(args.seed + 1)
                err_positions = rng.sample(range(len(encoded)), min(err_count, len(encoded)))
                corrupted = list(encoded)
                for pos in err_positions:
                    corrupted[pos] ^= 1
                print(f'注入 {len(err_positions)} 个错误 @ {err_positions[:10]}...')

                decoded = hamming_decode(corrupted)
                print(f'解码后 ({len(decoded["data_bits"])}): {decoded["data_bits"][:20]}...')
                print(f'纠正错误: {decoded["errors_corrected"]} 个')
                match = sum(1 for i in range(min(len(bits), len(decoded["data_bits"])))
                           if bits[i] == decoded["data_bits"][i])
                print(f'比特匹配率: {match/min(len(bits), len(decoded["data_bits"]))*100:.2f}%')

        elif args.type == 'convolutional':
            encoded = convolutional_encode(bits)
            print(f'卷积编码后 ({len(encoded)}): {encoded[:20]}...')

            if args.decode_test:
                decoded = viterbi_decode(encoded)
                print(f'Viterbi译码后 ({len(decoded["data_bits"])}): {decoded["data_bits"][:20]}...')
                match = sum(1 for i in range(min(len(bits), len(decoded["data_bits"])))
                           if bits[i] == decoded["data_bits"][i])
                print(f'比特匹配率: {match/min(len(bits), len(decoded["data_bits"]))*100:.2f}%')

    elif args.command == 'ber':
        print(f'BER测试: {args.type.upper()}')
        print(f'SNR范围: {args.snr_start}~{args.snr_stop}dB, 步长={args.snr_step}dB')
        print(f'\n{"SNR(dB)":<12} {"BER":<14} {"错误数":<12} {"理论BER":<14}')
        print('-' * 52)

        bits = generate_bits(args.bits)
        results = []
        snr = args.snr_start
        while snr <= args.snr_stop + 0.01:
            # 调制+信道+解调
            mod = modulate_bpsk(bits, args.fc, args.fs, args.bit_rate)
            noisy = awgn_channel(mod["signal"], snr)
            rx_bits = demodulate_bpsk(noisy, args.fc, args.fs, args.bit_rate)
            ber_result = calc_ber(bits, rx_bits)
            theory_ber = bpsk_theoretical_ber(snr)
            print(f'{snr:<12.1f} {ber_result["ber"]:<14.6f} {ber_result["errors"]:<12} {theory_ber:<14.6e}')
            results.append({"snr_db": snr, "ber": ber_result["ber"],
                           "errors": ber_result["errors"], "theoretical_ber": theory_ber})
            snr += args.snr_step

        if args.output:
            with open(args.output, 'w') as f:
                json.dump(results, f, indent=2)
            print(f'\nBER数据已保存至 {args.output}')

    elif args.command == 'theory':
        print(f'理论BER ({args.type.upper()}):')
        print(f'{"SNR(dB)":<12} {"BER":<14}')
        print('-' * 26)
        snr = args.snr_start
        while snr <= args.snr_stop + 0.01:
            ber = bpsk_theoretical_ber(snr)
            print(f'{snr:<12.1f} {ber:<14.6e}')
            snr += args.snr_step


if __name__ == '__main__':
    main()
