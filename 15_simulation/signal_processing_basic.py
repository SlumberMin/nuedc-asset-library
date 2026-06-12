"""
基础信号处理仿真
方法：FFT频谱分析 + FIR滤波器 + IIR滤波器 + 小波变换
应用：信号去噪、频谱分析、特征提取
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt



def main():
    plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
    plt.rcParams['axes.unicode_minus'] = False

    # ============================================================
    # 1. 生成测试信号：多频信号 + 噪声
    # ============================================================
    fs = 1000           # 采样率 1000Hz
    dt = 1.0 / fs
    N = 2000            # 采样点数
    t = np.arange(N) * dt

    # 信号组成：50Hz + 120Hz + 200Hz 正弦波
    signal_clean = (1.0 * np.sin(2*np.pi*50*t) +
                    0.5 * np.sin(2*np.pi*120*t) +
                    0.3 * np.sin(2*np.pi*200*t))

    # 添加高斯白噪声
    np.random.seed(42)
    noise = 0.8 * np.random.randn(N)
    signal_noisy = signal_clean + noise

    print("=" * 60)
    print("基础信号处理仿真")
    print("=" * 60)

    # ============================================================
    # 2. FFT频谱分析
    # ============================================================
    def fft_analysis(signal, fs):
        """FFT频谱分析"""
        N = len(signal)
        fft_result = np.fft.fft(signal)
        freq = np.fft.fftfreq(N, 1/fs)

        # 单边频谱
        n_half = N // 2
        magnitude = 2.0 / N * np.abs(fft_result[:n_half])
        phase = np.angle(fft_result[:n_half])
        freq_half = freq[:n_half]

        return freq_half, magnitude, phase

    freq_clean, mag_clean, _ = fft_analysis(signal_clean, fs)
    freq_noisy, mag_noisy, _ = fft_analysis(signal_noisy, fs)

    print(f"\n[FFT分析]")
    print(f"  采样率: {fs} Hz, 采样点数: {N}")
    print(f"  频率分辨率: {fs/N:.2f} Hz")
    # 找峰值频率
    peak_indices = []
    for i in range(1, len(mag_noisy)-1):
        if mag_noisy[i] > mag_noisy[i-1] and mag_noisy[i] > mag_noisy[i+1] and mag_noisy[i] > 0.1:
            peak_indices.append(i)
    print(f"  检测到的主频: {[f'{freq_noisy[i]:.1f}Hz (幅值={mag_noisy[i]:.2f})' for i in peak_indices]}")

    # ============================================================
    # 3. FIR低通滤波器（窗函数法）
    # ============================================================
    def design_fir_lowpass(fc, fs, num_taps=101, window='hamming'):
        """
        设计FIR低通滤波器
        fc: 截止频率, fs: 采样率, num_taps: 滤波器阶数
        """
        M = num_taps - 1
        n = np.arange(num_taps)
        fc_norm = fc / fs

        # 理想低通滤波器（sinc函数）
        h = np.sinc(2 * fc_norm * (n - M/2))

        # 加窗
        if window == 'hamming':
            w = 0.54 - 0.46 * np.cos(2*np.pi*n/M)
        elif window == 'hanning':
            w = 0.5 - 0.5 * np.cos(2*np.pi*n/M)
        elif window == 'blackman':
            w = 0.42 - 0.5*np.cos(2*np.pi*n/M) + 0.08*np.cos(4*np.pi*n/M)
        else:
            w = np.ones(num_taps)

        h = h * w
        h = h / np.sum(h)  # 归一化

        return h

    def fir_filter(h, signal):
        """FIR滤波"""
        output = np.convolve(signal, h, mode='same')
        return output

    # 设计截止频率150Hz的FIR低通滤波器
    fir_coeffs = design_fir_lowpass(fc=150, fs=fs, num_taps=101, window='hamming')
    signal_fir = fir_filter(fir_coeffs, signal_noisy)

    print(f"\n[FIR滤波器]")
    print(f"  类型: Hamming窗低通, 阶数: {len(fir_coeffs)-1}")
    print(f"  截止频率: 150 Hz")
    print(f"  滤波前SNR: {10*np.log10(np.var(signal_clean)/np.var(noise)):.1f} dB")
    noise_residual = signal_fir - signal_clean
    print(f"  滤波后残余噪声功率: {np.var(noise_residual):.4f}")

    # ============================================================
    # 4. IIR滤波器（Butterworth）
    # ============================================================
    def design_butterworth_lowpass(fc, fs, order=4):
        """
        设计Butterworth IIR低通滤波器（手动实现双线性变换）
        fc: 截止频率, fs: 采样率, order: 阶数
        """
        # 预畸变
        wc = 2 * fs * np.tan(np.pi * fc / fs)

        # 模拟Butterworth极点
        poles = []
        for k in range(order):
            angle = np.pi * (2*k + order + 1) / (2*order)
            pole = wc * np.exp(1j * angle)
            if np.real(pole) < 0:
                poles.append(pole)

        # 双线性变换 z = (2*fs + s) / (2*fs - s)
        z_poles = [(2*fs + p) / (2*fs - p) for p in poles]

        # 构造传递函数系数
        # 分母 = product of (1 - p*z^-1)
        b = np.array([1.0])
        a = np.array([1.0])
        for p in z_poles:
            a = np.convolve(a, [1, -p])

        # 归一化使直流增益为1
        dc_gain = np.abs(np.sum(a))
        b = np.array([dc_gain])  # 简化

        return a.real, b

    # 简化实现：使用直接的差分方程
    def butterworth_filter_manual(signal, fs, fc=150, order=4):
        """手动实现Butterworth滤波（级联一阶节）"""
        output = signal.copy()

        # 使用2个级联的二阶IIR节（简化Butterworth近似）
        # 节1: fc=150Hz, Q=0.541 (Butterworth Q)
        # 节2: fc=150Hz, Q=1.307

        sections = [
            {'fc': 150, 'Q': 0.541},
            {'fc': 150, 'Q': 1.307},
        ]

        for sec in sections:
            fc = sec['fc']
            Q = sec['Q']
            w0 = 2 * np.pi * fc / fs
            alpha = np.sin(w0) / (2 * Q)

            b0 = (1 - np.cos(w0)) / 2
            b1 = 1 - np.cos(w0)
            b2 = (1 - np.cos(w0)) / 2
            a0 = 1 + alpha
            a1 = -2 * np.cos(w0)
            a2 = 1 - alpha

            # 归一化
            b = np.array([b0/a0, b1/a0, b2/a0])
            a = np.array([1, a1/a0, a2/a0])

            # 差分方程滤波
            y = np.zeros_like(output)
            x1, x2, y1, y2 = 0, 0, 0, 0
            for i in range(len(output)):
                y[i] = b[0]*output[i] + b[1]*x1 + b[2]*x2 - a[1]*y1 - a[2]*y2
                x2, x1 = x1, output[i]
                y2, y1 = y1, y[i]

            output = y

        return output

    signal_iir = butterworth_filter_manual(signal_noisy, fs, fc=150)

    print(f"\n[IIR滤波器]")
    print(f"  类型: Butterworth低通, 阶数: 4")
    print(f"  截止频率: 150 Hz")
    noise_residual_iir = signal_iir - signal_clean
    print(f"  滤波后残余噪声功率: {np.var(noise_residual_iir):.4f}")

    # ============================================================
    # 5. 小波变换（简化离散小波变换DWT）
    # ============================================================
    def haar_dwt(signal, level=4):
        """
        Haar小波分解（简化DWT）
        返回各层近似系数和细节系数
        """
        coeffs = []
        approx = signal.copy()

        for l in range(level):
            n = len(approx)
            if n < 2:
                break

            # 确保长度为偶数
            if n % 2 != 0:
                approx = np.append(approx, approx[-1])
                n = len(approx)

            n_half = n // 2
            new_approx = np.zeros(n_half)
            detail = np.zeros(n_half)

            for i in range(n_half):
                new_approx[i] = (approx[2*i] + approx[2*i+1]) / np.sqrt(2)
                detail[i] = (approx[2*i] - approx[2*i+1]) / np.sqrt(2)

            coeffs.append(detail)
            approx = new_approx

        coeffs.append(approx)  # 最终近似系数
        return coeffs

    def haar_idwt(coeffs, target_len):
        """Haar小波重构"""
        approx = coeffs[-1]

        for l in range(len(coeffs)-2, -1, -1):
            detail = coeffs[l]
            # 确保近似和细节系数长度匹配
            min_len = min(len(approx), len(detail))
            n = min_len * 2
            signal = np.zeros(n)

            for i in range(min_len):
                signal[2*i] = (approx[i] + detail[i]) / np.sqrt(2)
                signal[2*i+1] = (approx[i] - detail[i]) / np.sqrt(2)

            approx = signal

        # 调整长度
        if len(approx) > target_len:
            approx = approx[:target_len]
        elif len(approx) < target_len:
            approx = np.pad(approx, (0, target_len - len(approx)))

        return approx

    # 小波去噪（软阈值）
    coeffs = haar_dwt(signal_noisy, level=4)

    # 对细节系数施加软阈值
    sigma_est = np.median(np.abs(coeffs[0])) / 0.6745  # 噪声标准差估计
    threshold = sigma_est * np.sqrt(2 * np.log(N))  # 通用阈值

    denoised_coeffs = [coeffs[-1]]  # 保留近似系数
    for c in coeffs[:-1]:
        # 软阈值
        c_thresh = np.sign(c) * np.maximum(np.abs(c) - threshold, 0)
        denoised_coeffs.append(c_thresh)
    denoised_coeffs.reverse()

    signal_wavelet = haar_idwt(denoised_coeffs, N)

    print(f"\n[小波去噪]")
    print(f"  小波: Haar, 分解层数: 4")
    print(f"  阈值方法: 通用阈值 (Universal)")
    print(f"  阈值: {threshold:.4f}")
    noise_residual_wt = signal_wavelet - signal_clean
    print(f"  滤波后残余噪声功率: {np.var(noise_residual_wt):.4f}")

    # ============================================================
    # 6. 绘图
    # ============================================================
    fig, axes = plt.subplots(3, 2, figsize=(16, 14))

    # 图1：原始信号
    ax = axes[0, 0]
    ax.plot(t[:500]*1000, signal_clean[:500], 'b-', linewidth=1.5, label='纯净信号')
    ax.plot(t[:500]*1000, signal_noisy[:500], 'gray', alpha=0.5, label='含噪信号')
    ax.set_xlabel('时间 (ms)')
    ax.set_ylabel('幅值')
    ax.set_title('原始信号（50+120+200Hz + 噪声）')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 图2：FFT频谱
    ax = axes[0, 1]
    ax.plot(freq_clean, mag_clean, 'b-', linewidth=1, label='纯净信号频谱')
    ax.plot(freq_noisy, mag_noisy, 'gray', alpha=0.5, linewidth=0.8, label='含噪信号频谱')
    ax.set_xlabel('频率 (Hz)')
    ax.set_ylabel('幅值')
    ax.set_title('FFT频谱分析')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_xlim([0, fs/2])

    # 图3：FIR滤波
    ax = axes[1, 0]
    ax.plot(t[:500]*1000, signal_clean[:500], 'b-', linewidth=1.5, label='纯净信号')
    ax.plot(t[:500]*1000, signal_fir[:500], 'r-', linewidth=1, alpha=0.8, label='FIR滤波后')
    ax.set_xlabel('时间 (ms)')
    ax.set_ylabel('幅值')
    ax.set_title(f'FIR低通滤波 (150Hz截止)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 图4：IIR滤波
    ax = axes[1, 1]
    ax.plot(t[:500]*1000, signal_clean[:500], 'b-', linewidth=1.5, label='纯净信号')
    ax.plot(t[:500]*1000, signal_iir[:500], 'g-', linewidth=1, alpha=0.8, label='IIR滤波后')
    ax.set_xlabel('时间 (ms)')
    ax.set_ylabel('幅值')
    ax.set_title(f'IIR Butterworth低通滤波 (4阶, 150Hz)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 图5：小波去噪
    ax = axes[2, 0]
    ax.plot(t[:500]*1000, signal_clean[:500], 'b-', linewidth=1.5, label='纯净信号')
    ax.plot(t[:500]*1000, signal_wavelet[:500], 'm-', linewidth=1, alpha=0.8, label='小波去噪后')
    ax.set_xlabel('时间 (ms)')
    ax.set_ylabel('幅值')
    ax.set_title('Haar小波去噪')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 图6：FIR频率响应
    ax = axes[2, 1]
    freq_response = np.abs(np.fft.fft(fir_coeffs, 2048))
    freq_axis = np.linspace(0, fs/2, 1024)
    ax.plot(freq_axis, 20*np.log10(freq_response[:1024] + 1e-10), 'r-', linewidth=2)
    ax.axvline(150, color='gray', linestyle='--', label='截止频率 150Hz')
    ax.axhline(-3, color='gray', linestyle=':', alpha=0.5)
    ax.set_xlabel('频率 (Hz)')
    ax.set_ylabel('幅值 (dB)')
    ax.set_title('FIR滤波器频率响应')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_ylim([-80, 5])
    ax.set_xlim([0, fs/2])

    plt.suptitle('基础信号处理仿真（FFT + FIR + IIR + 小波）', fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'signal_processing_basic.png'), dpi=150, bbox_inches='tight')
    print("\n图表已保存: signal_processing_basic.png")
    plt.close('all')



if __name__ == '__main__':
    main()
