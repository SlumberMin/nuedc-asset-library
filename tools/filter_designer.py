#!/usr/bin/env python3
"""
滤波器设计工具 - nuedc-asset-library
功能：FIR/IIR滤波器设计、Butterworth/Chebyshev/椭圆滤波器、频率响应计算
作者：电赛自动迭代引擎 V3
"""

import argparse
import json
import math
import sys

# ─── 复数运算辅助 ─────────────────────────────────────────────────

def _cadd(a, b):
    return (a[0]+b[0], a[1]+b[1])

def _csub(a, b):
    return (a[0]-b[0], a[1]-b[1])

def _cmul(a, b):
    return (a[0]*b[0]-a[1]*b[1], a[0]*b[1]+a[1]*b[0])

def _cdiv(a, b):
    d = b[0]**2 + b[1]**2
    return ((a[0]*b[0]+a[1]*b[1])/d, (a[1]*b[0]-a[0]*b[1])/d)

def _cabs(a):
    return math.sqrt(a[0]**2 + a[1]**2)

def _clog(a):
    return (math.log(_cabs(a)), math.atan2(a[1], a[0]))

def _cexp(a):
    r = math.exp(a[0])
    return (r * math.cos(a[1]), r * math.sin(a[1]))


# ─── Butterworth滤波器 ───────────────────────────────────────────

def butterworth_poles(n):
    """计算Butterworth滤波器的极点（归一化模拟）"""
    poles = []
    for k in range(n):
        theta = math.pi * (2*k + n + 1) / (2*n)
        poles.append((math.cos(theta), math.sin(theta)))
    return poles


def butterworth_order(wp, ws, gpass, gstop):
    """
    计算Butterworth滤波器阶数
    wp: 通带角频率, ws: 阻带角频率
    gpass: 通带最大衰减(dB), gstop: 阻带最小衰减(dB)
    """
    num = math.log10((10**(gstop/10) - 1) / (10**(gpass/10) - 1))
    den = 2 * math.log10(ws / wp)
    return math.ceil(num / den)


# ─── Chebyshev Type I ────────────────────────────────────────────

def chebyshev1_poles(n, epsilon):
    """Chebyshev I型极点"""
    a = 1.0 / n
    asinh_eps = math.log(epsilon + math.sqrt(epsilon**2 + 1))  # arcsinh
    poles = []
    for k in range(n):
        theta = math.pi * (2*k + 1) / (2*n)
        sigma = -math.sin(theta) * math.sinh(a * asinh_eps)
        omega = math.cos(theta) * math.cosh(a * asinh_eps)
        poles.append((sigma, omega))
    return poles


def chebyshev1_order(wp, ws, gpass, gstop):
    """计算Chebyshev I型阶数"""
    epsilon = math.sqrt(10**(gpass/10) - 1)
    num = math.log((10**(gstop/10) - 1) / (epsilon**2))
    den = 2 * math.log(ws / wp + math.sqrt((ws/wp)**2 - 1))
    return math.ceil(num / den)


# ─── 椭圆滤波器（简化阶数计算） ──────────────────────────────────

def elliptic_order(wp, ws, gpass, gstop):
    """
    估算椭圆滤波器阶数
    使用近似公式：n ≈ K(k)*K(k1') / (K(k')*K(k1))
    其中K为第一类完全椭圆积分
    """
    eps_p = math.sqrt(10**(gpass/10) - 1)
    eps_s = math.sqrt(10**(gstop/10) - 1)
    k = wp / ws
    k1 = eps_p / eps_s

    def _elliptic_k(m):
        """第一类完全椭圆积分的近似计算"""
        a = 1.0
        b = math.sqrt(1.0 - m)
        for _ in range(20):
            a_new = (a + b) / 2
            b_new = math.sqrt(a * b)
            a, b = a_new, b_new
        return math.pi / (2 * a)

    k_prime = math.sqrt(1 - k**2)
    k1_prime = math.sqrt(1 - k1**2)

    if k_prime < 1e-10 or k1_prime < 1e-10:
        return 1

    n = (_elliptic_k(k**2) * _elliptic_k(k1_prime**2)) / \
        (_elliptic_k(k_prime**2) * _elliptic_k(k1**2))
    return math.ceil(n)


# ─── FIR滤波器设计（窗函数法） ───────────────────────────────────

def fir_window(window_type, N):
    """计算窗函数"""
    w = []
    for n in range(N):
        if window_type == 'rect':
            w.append(1.0)
        elif window_type == 'hann':
            w.append(0.5 * (1 - math.cos(2*math.pi*n/(N-1))))
        elif window_type == 'hamming':
            w.append(0.54 - 0.46 * math.cos(2*math.pi*n/(N-1)))
        elif window_type == 'blackman':
            w.append(0.42 - 0.5*math.cos(2*math.pi*n/(N-1)) +
                    0.08*math.cos(4*math.pi*n/(N-1)))
        elif window_type == 'kaiser':
            beta = 4.0  # 默认beta
            # Kaiser窗近似
            alpha = (N - 1) / 2
            x = (n - alpha) / alpha
            if abs(x) <= 1:
                w.append(math.sqrt(1 - x**2))
            else:
                w.append(0.0)
        else:
            w.append(1.0)
    return w


def design_fir_lowpass(fc, fs, N, window='hamming'):
    """
    设计FIR低通滤波器（窗函数法）
    fc: 截止频率(Hz), fs: 采样率(Hz), N: 阶数(奇数)
    返回: 系数列表
    """
    if N % 2 == 0:
        N += 1  # 确保奇数阶
    wc = 2 * math.pi * fc / fs  # 归一化角频率
    M = (N - 1) // 2
    h = []
    for n in range(N):
        if n == M:
            h.append(wc / math.pi)
        else:
            h.append(math.sin(wc * (n - M)) / (math.pi * (n - M)))
    # 加窗
    w = fir_window(window, N)
    h = [h[i] * w[i] for i in range(N)]
    # 归一化使直流增益=1
    total = sum(h)
    if abs(total) > 1e-12:
        h = [hi / total for hi in h]
    return h


def design_fir_highpass(fc, fs, N, window='hamming'):
    """设计FIR高通滤波器"""
    h_lp = design_fir_lowpass(fc, fs, N, window)
    h = [-h_lp[i] for i in range(N)]
    M = (N - 1) // 2
    h[M] += 1.0
    return h


def design_fir_bandpass(f1, f2, fs, N, window='hamming'):
    """设计FIR带通滤波器"""
    h_lp1 = design_fir_lowpass(f1, fs, N, window)
    h_lp2 = design_fir_lowpass(f2, fs, N, window)
    h = [h_lp2[i] - h_lp1[i] for i in range(N)]
    return h


# ─── IIR滤波器设计（双线性变换法，Butterworth） ───────────────────

def design_butter_lowpass(fc, fs, order):
    """
    设计Butterworth低通IIR滤波器
    返回: (b系数, a系数) - 差分方程系数
    """
    wc = 2 * math.pi * fc
    T = 1.0 / fs
    # 预畸变
    wc_d = (2.0 / T) * math.tan(wc * T / 2.0)

    # 获取模拟极点
    poles = butterworth_poles(order)
    # 双线性变换
    b_coeffs = []
    a_coeffs = []
    # 级联二阶节
    sos_sections = []
    k = 0
    while k < len(poles):
        if k + 1 < len(poles) and abs(poles[k][1] - poles[k+1][1]) < 0.01:
            # 共轭极点对 -> 二阶节
            p = poles[k]
            # s域极点 -> z域
            sp = (p[0] * wc_d, p[1] * wc_d)
            # 双线性: z = (1 + sT/2) / (1 - sT/2)
            half_t = T / 2.0
            num = (1 + sp[0]*half_t, sp[1]*half_t)
            den = (1 - sp[0]*half_t, -sp[1]*half_t)
            z_pole = _cdiv(num, den)
            # 二阶节: (1 - z^{-1})^2 / (1 - p1*z^{-1})(1 - p2*z^{-1})
            # 展开: b = [1, -2, 1], a = [1, -(p1+p2), p1*p2]
            sum_p = 2 * z_pole[0]
            prod_p = _cabs(z_pole)**2
            a_sec = [1.0, -sum_p, prod_p]
            b_sec = [1.0, -2.0, 1.0]
            sos_sections.append((b_sec, a_sec))
            k += 2
        else:
            # 实极点 -> 一阶节
            p = poles[k]
            sp = (p[0] * wc_d, 0)
            half_t = T / 2.0
            num = (1 + sp[0]*half_t, 0)
            den = (1 - sp[0]*half_t, 0)
            z_pole = _cdiv(num, den)
            b_sec = [1.0, -1.0]
            a_sec = [1.0, -z_pole[0]]
            sos_sections.append((b_sec, a_sec))
            k += 1

    # 合并所有节为总传递函数
    b_total = [1.0]
    a_total = [1.0]
    for b_sec, a_sec in sos_sections:
        # 多项式卷积
        new_b = [0.0] * (len(b_total) + len(b_sec) - 1)
        for i in range(len(b_total)):
            for j in range(len(b_sec)):
                new_b[i+j] += b_total[i] * b_sec[j]
        new_a = [0.0] * (len(a_total) + len(a_sec) - 1)
        for i in range(len(a_total)):
            for j in range(len(a_sec)):
                new_a[i+j] += a_total[i] * a_sec[j]
        b_total = new_b
        a_total = new_a

    # 归一化
    gain = a_total[0]
    b_total = [b / gain for b in b_total]
    a_total = [a / gain for a in a_total]

    # DC增益校正
    sum_b = sum(b_total)
    sum_a = sum(a_total)
    dc_gain = sum_b / sum_a
    b_total = [b / dc_gain for b in b_total]

    return b_total, a_total


# ─── 频率响应 ────────────────────────────────────────────────────

def freq_response(b, a, fs, n_points=512):
    """
    计算IIR/FIR滤波器的频率响应
    返回: {freqs[], magnitude_db[], phase_deg[]}
    """
    freqs = []
    mag_db = []
    phase_deg = []
    for i in range(n_points):
        f = i * fs / (2 * n_points)
        w = 2 * math.pi * f / fs
        # H(e^{jw}) = sum(b[k]*z^{-k}) / sum(a[k]*z^{-k})
        num_r, num_i = 0.0, 0.0
        for k, bk in enumerate(b):
            num_r += bk * math.cos(-k * w)
            num_i += bk * math.sin(-k * w)
        den_r, den_i = 0.0, 0.0
        for k, ak in enumerate(a):
            den_r += ak * math.cos(-k * w)
            den_i += ak * math.sin(-k * w)
        # 复数除法
        den_mag2 = den_r**2 + den_i**2
        if den_mag2 < 1e-30:
            den_mag2 = 1e-30
        h_r = (num_r*den_r + num_i*den_i) / den_mag2
        h_i = (num_i*den_r - num_r*den_i) / den_mag2
        h_mag = math.sqrt(h_r**2 + h_i**2)
        h_phase = math.atan2(h_i, h_r)
        freqs.append(f)
        mag_db.append(20 * math.log10(max(h_mag, 1e-15)))
        phase_deg.append(h_phase * 180 / math.pi)
    return {"freqs": freqs, "magnitude_db": mag_db, "phase_deg": phase_deg}


# ─── 应用滤波器（时域滤波） ────────────────────────────────────────

def apply_filter(b, a, signal):
    """IIR/FIR滤波（直接II型差分方程）"""
    N = len(signal)
    output = [0.0] * N
    order_b = len(b) - 1
    order_a = len(a) - 1
    for n in range(N):
        y = 0.0
        for i in range(min(order_b + 1, n + 1)):
            y += b[i] * signal[n - i]
        for j in range(1, min(order_a + 1, n + 1)):
            y -= a[j] * output[n - j]
        output[n] = y
    return output


# ─── 群延迟 ──────────────────────────────────────────────────────

def group_delay(b, a, fs, n_points=512):
    """计算群延迟"""
    result = []
    for i in range(n_points):
        f = i * fs / (2 * n_points)
        w = 2 * math.pi * f / fs
        # 数值微分法
        dw = 0.001
        phases = []
        for sign in [-1, 0, 1]:
            w_eval = w + sign * dw
            num_r = sum(b[k]*math.cos(-k*w_eval) for k in range(len(b)))
            num_i = sum(b[k]*math.sin(-k*w_eval) for k in range(len(b)))
            den_r = sum(a[k]*math.cos(-k*w_eval) for k in range(len(a)))
            den_i = sum(a[k]*math.sin(-k*w_eval) for k in range(len(a)))
            den_mag2 = den_r**2 + den_i**2
            h_r = (num_r*den_r + num_i*den_i) / max(den_mag2, 1e-30)
            h_i = (num_i*den_r - num_r*den_i) / max(den_mag2, 1e-30)
            phases.append(math.atan2(h_i, h_r))
        dphi = (phases[2] - phases[0]) / (2 * dw)
        gd = -dphi / (2 * math.pi)
        result.append({"freq": f, "delay": gd})
    return result


# ─── CSV滤波 ─────────────────────────────────────────────────────

def load_csv(filepath, column=0):
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
    parser = argparse.ArgumentParser(description='滤波器设计工具 - 电赛资产库')
    sub = parser.add_subparsers(dest='command')

    # FIR设计
    p_fir = sub.add_parser('fir', help='FIR滤波器设计')
    p_fir.add_argument('--type', choices=['lowpass', 'highpass', 'bandpass'], default='lowpass')
    p_fir.add_argument('--fc', type=float, help='截止频率(Hz)')
    p_fir.add_argument('--f1', type=float, help='带通下限(Hz)')
    p_fir.add_argument('--f2', type=float, help='带通上限(Hz)')
    p_fir.add_argument('--fs', type=float, required=True, help='采样率(Hz)')
    p_fir.add_argument('--order', type=int, default=31, help='滤波器阶数')
    p_fir.add_argument('--window', default='hamming',
                       choices=['rect', 'hann', 'hamming', 'blackman'])
    p_fir.add_argument('--output', '-o', help='输出系数JSON')

    # IIR设计
    p_iir = sub.add_parser('iir', help='IIR滤波器设计(Butterworth)')
    p_iir.add_argument('--type', choices=['lowpass'], default='lowpass')
    p_iir.add_argument('--fc', type=float, required=True, help='截止频率(Hz)')
    p_iir.add_argument('--fs', type=float, required=True, help='采样率(Hz)')
    p_iir.add_argument('--order', type=int, default=4, help='滤波器阶数')
    p_iir.add_argument('--output', '-o', help='输出系数JSON')

    # 阶数计算
    p_calc = sub.add_parser('order', help='计算滤波器阶数')
    p_calc.add_argument('--type', choices=['butterworth', 'chebyshev1', 'elliptic'], default='butterworth')
    p_calc.add_argument('--wp', type=float, required=True, help='通带频率(Hz)')
    p_calc.add_argument('--ws', type=float, required=True, help='阻带频率(Hz)')
    p_calc.add_argument('--gpass', type=float, default=1.0, help='通带衰减(dB)')
    p_calc.add_argument('--gstop', type=float, default=40.0, help='阻带衰减(dB)')

    # 滤波（应用滤波器）
    p_filt = sub.add_parser('filter', help='对信号应用滤波')
    p_filt.add_argument('--input', '-i', required=True, help='输入CSV')
    p_filt.add_argument('--output', '-o', required=True, help='输出CSV')
    p_filt.add_argument('--b', required=True, help='b系数(JSON数组)')
    p_filt.add_argument('--a', required=True, help='a系数(JSON数组)')
    p_filt.add_argument('--column', type=int, default=0)

    # 频率响应
    p_freq = sub.add_parser('response', help='计算频率响应')
    p_freq.add_argument('--b', required=True, help='b系数(JSON)')
    p_freq.add_argument('--a', required=True, help='a系数(JSON)')
    p_freq.add_argument('--fs', type=float, required=True, help='采样率')
    p_freq.add_argument('--output', '-o', help='输出JSON')

    args = parser.parse_args()

    if args.command is None:
        parser.print_help()
        return

    if args.command == 'order':
        if args.type == 'butterworth':
            n = butterworth_order(args.wp, args.ws, args.gpass, args.gstop)
        elif args.type == 'chebyshev1':
            n = chebyshev1_order(args.wp, args.ws, args.gpass, args.gstop)
        else:
            n = elliptic_order(args.wp, args.ws, args.gpass, args.gstop)
        print(f'{args.type}滤波器最小阶数: {n}')
        return

    if args.command == 'fir':
        if args.type == 'lowpass':
            h = design_fir_lowpass(args.fc, args.fs, args.order, args.window)
        elif args.type == 'highpass':
            h = design_fir_highpass(args.fc, args.fs, args.order, args.window)
        else:
            h = design_fir_bandpass(args.f1, args.f2, args.fs, args.order, args.window)
        print(f'FIR {args.type}滤波器 (阶数={len(h)}, 窗={args.window}):')
        print(f'系数: {[round(c, 8) for c in h[:10]]}...')
        if args.output:
            with open(args.output, 'w') as f:
                json.dump({"b": h, "a": [1.0], "type": f"fir_{args.type}",
                          "fs": args.fs}, f, indent=2)
            print(f'系数已保存至 {args.output}')

    elif args.command == 'iir':
        b, a = design_butter_lowpass(args.fc, args.fs, args.order)
        print(f'IIR Butterworth低通 (阶数={args.order}):')
        print(f'b = {[round(c, 8) for c in b]}')
        print(f'a = {[round(c, 8) for c in a]}')
        if args.output:
            with open(args.output, 'w') as f:
                json.dump({"b": b, "a": a, "type": "iir_butterworth",
                          "fs": args.fs, "order": args.order}, f, indent=2)
            print(f'系数已保存至 {args.output}')

    elif args.command == 'filter':
        sig = load_csv(args.input, args.column)
        b = json.loads(args.b)
        a = json.loads(args.a)
        filtered = apply_filter(b, a, sig)
        with open(args.output, 'w') as f:
            f.write('sample,value\n')
            for i, v in enumerate(filtered):
                f.write(f'{i},{v:.10f}\n')
        print(f'滤波完成，{len(filtered)}个采样点 -> {args.output}')

    elif args.command == 'response':
        b = json.loads(args.b)
        a = json.loads(args.a)
        resp = freq_response(b, a, args.fs)
        print(f'{"频率(Hz)":<14} {"增益(dB)":<14} {"相位(度)":<14}')
        print('-' * 42)
        step = max(1, len(resp["freqs"]) // 20)
        for i in range(0, len(resp["freqs"]), step):
            print(f'{resp["freqs"][i]:<14.2f} {resp["magnitude_db"][i]:<14.2f} {resp["phase_deg"][i]:<14.2f}')
        if args.output:
            with open(args.output, 'w') as f:
                json.dump(resp, f, indent=2)


if __name__ == '__main__':
    main()
