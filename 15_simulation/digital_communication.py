"""
数字通信仿真模块 - Digital Communication Simulation
=====================================================
功能: 调制解调、信道编码、误码率分析
适用: 电赛通信系统设计与性能评估
"""

import numpy as np
from typing import Tuple, Optional, Dict, List
from scipy.special import erfc
import warnings
warnings.filterwarnings('ignore')


# ─────────────────────────────────────────────
# 1. 数字调制
# ─────────────────────────────────────────────

class DigitalModulator:
    """数字调制器"""
    
    @staticmethod
    def ask_modulate(bits: np.ndarray, M: int = 2, A: float = 1.0) -> np.ndarray:
        """
        ASK调制 (OOK为M=2特例)
        
        Args:
            bits: 比特流
            M: 调制阶数
            A: 振幅
        Returns:
            调制信号
        """
        bits_per_symbol = int(np.log2(M))
        # 分组
        n_symbols = len(bits) // bits_per_symbol
        symbols = np.array([int(''.join(str(int(b)) for b in bits[i*bits_per_symbol:(i+1)*bits_per_symbol]), 2)
                           for i in range(n_symbols)])
        
        # ASK: 振幅与符号成正比
        signal = A * symbols / (M - 1) if M > 1 else A * symbols
        return signal.astype(float)
    
    @staticmethod
    def fsk_modulate(bits: np.ndarray, f0: float, f1: float,
                     fs: float, samples_per_bit: int) -> Tuple[np.ndarray, np.ndarray]:
        """
        2FSK调制
        
        Args:
            bits: 比特流
            f0, f1: 两个载波频率
            fs: 采样率
            samples_per_bit: 每比特采样数
        Returns:
            (t, 调制信号)
        """
        N = len(bits) * samples_per_bit
        t = np.arange(N) / fs
        signal = np.zeros(N)
        
        for i, bit in enumerate(bits):
            f = f1 if bit else f0
            idx = slice(i * samples_per_bit, (i + 1) * samples_per_bit)
            signal[idx] = np.cos(2 * np.pi * f * t[idx])
        
        return t, signal
    
    @staticmethod
    def psk_modulate(bits: np.ndarray, M: int = 2, fc: float = None,
                     fs: float = None, samples_per_symbol: int = None) -> Tuple[np.ndarray, np.ndarray]:
        """
        PSK调制 (BPSK/QPSK/8PSK)
        
        Args:
            bits: 比特流
            M: 调制阶数 (2=BPSK, 4=QPSK, 8=8PSK)
            fc: 载波频率 (None=基带)
            fs: 采样率
            samples_per_symbol: 每符号采样数
        Returns:
            (I路, Q路) 或 载波调制信号
        """
        bits_per_symbol = int(np.log2(M))
        n_symbols = len(bits) // bits_per_symbol
        symbols = np.array([int(''.join(str(int(b)) for b in bits[i*bits_per_symbol:(i+1)*bits_per_symbol]), 2)
                           for i in range(n_symbols)])
        
        # 相位映射
        phases = 2 * np.pi * symbols / M
        
        I = np.cos(phases)
        Q = np.sin(phases)
        
        if fc is not None and fs is not None and samples_per_symbol:
            # 载波调制
            N = n_symbols * samples_per_symbol
            t = np.arange(N) / fs
            signal = np.zeros(N)
            for i in range(n_symbols):
                idx = slice(i * samples_per_symbol, (i + 1) * samples_per_symbol)
                signal[idx] = I[i] * np.cos(2*np.pi*fc*t[idx]) - Q[i] * np.sin(2*np.pi*fc*t[idx])
            return t, signal
        
        return I, Q
    
    @staticmethod
    def qam_modulate(bits: np.ndarray, M: int = 16) -> Tuple[np.ndarray, np.ndarray]:
        """
        QAM调制
        
        Args:
            bits: 比特流
            M: 调制阶数 (16, 64, 256)
        Returns:
            (I路, Q路) 星座点
        """
        bits_per_symbol = int(np.log2(M))
        sqrt_M = int(np.sqrt(M))
        
        n_symbols = len(bits) // bits_per_symbol
        symbols = np.array([int(''.join(str(int(b)) for b in bits[i*bits_per_symbol:(i+1)*bits_per_symbol]), 2)
                           for i in range(n_symbols)])
        
        # QAM星座映射 (Gray编码简化)
        I_idx = symbols % sqrt_M
        Q_idx = symbols // sqrt_M
        
        I = 2 * I_idx - (sqrt_M - 1)
        Q = 2 * Q_idx - (sqrt_M - 1)
        
        # 归一化
        Es = np.mean(I**2 + Q**2)
        I = I / np.sqrt(Es)
        Q = Q / np.sqrt(Es)
        
        return I, Q
    
    @staticmethod
    def ofdm_modulate(data_symbols: np.ndarray, n_subcarriers: int = 64,
                      n_cyclic_prefix: int = 16) -> np.ndarray:
        """
        OFDM调制 (IFFT + 循环前缀)
        
        Args:
            data_symbols: 数据符号 (复数)
            n_subcarriers: 子载波数
            n_cyclic_prefix: 循环前缀长度
        Returns:
            OFDM时域信号
        """
        # 补零到n_subcarriers
        if len(data_symbols) < n_subcarriers:
            padded = np.zeros(n_subcarriers, dtype=complex)
            padded[:len(data_symbols)] = data_symbols
        else:
            padded = data_symbols[:n_subcarriers]
        
        # IFFT
        ofdm_symbol = np.fft.ifft(padded)
        
        # 添加循环前缀
        cp = ofdm_symbol[-n_cyclic_prefix:]
        ofdm_signal = np.concatenate([cp, ofdm_symbol])
        
        return ofdm_signal


class DigitalDemodulator:
    """数字解调器"""
    
    @staticmethod
    def ask_demodulate(signal: np.ndarray, M: int = 2, threshold: float = None) -> np.ndarray:
        """ASK解调"""
        if threshold is None:
            threshold = (M - 1) / 2
        
        symbols = np.round(signal * (M - 1)).astype(int)
        symbols = np.clip(symbols, 0, M - 1)
        
        bits_per_symbol = int(np.log2(M))
        bits = []
        for s in symbols:
            bits.extend([int(b) for b in format(s, f'0{bits_per_symbol}b')])
        
        return np.array(bits)
    
    @staticmethod
    def psk_demodulate(I: np.ndarray, Q: np.ndarray, M: int = 2) -> np.ndarray:
        """
        PSK相干解调
        
        Args:
            I, Q: 接收信号的同相和正交分量
            M: 调制阶数
        Returns:
            解调比特流
        """
        # 计算接收相位
        phase = np.arctan2(Q, I) % (2 * np.pi)
        
        # 判决
        symbols = np.round(phase * M / (2 * np.pi)).astype(int) % M
        
        bits_per_symbol = int(np.log2(M))
        bits = []
        for s in symbols:
            bits.extend([int(b) for b in format(s, f'0{bits_per_symbol}b')])
        
        return np.array(bits)
    
    @staticmethod
    def qam_demodulate(I: np.ndarray, Q: np.ndarray, M: int = 16) -> np.ndarray:
        """QAM解调"""
        sqrt_M = int(np.sqrt(M))
        
        # 判决到最近的星座点
        Es = np.mean(I**2 + Q**2) if np.mean(I**2 + Q**2) > 0 else 1
        I_norm = I / np.sqrt(Es) * (sqrt_M - 1)  # 反归一化
        Q_norm = Q / np.sqrt(Es) * (sqrt_M - 1)
        
        I_idx = np.clip(np.round((I_norm + (sqrt_M - 1)) / 2).astype(int), 0, sqrt_M - 1)
        Q_idx = np.clip(np.round((Q_norm + (sqrt_M - 1)) / 2).astype(int), 0, sqrt_M - 1)
        
        symbols = Q_idx * sqrt_M + I_idx
        
        bits_per_symbol = int(np.log2(M))
        bits = []
        for s in symbols:
            bits.extend([int(b) for b in format(s, f'0{bits_per_symbol}b')])
        
        return np.array(bits)
    
    @staticmethod
    def ofdm_demodulate(ofdm_signal: np.ndarray, n_subcarriers: int = 64,
                        n_cyclic_prefix: int = 16) -> np.ndarray:
        """OFDM解调 (去CP + FFT)"""
        # 去除循环前缀
        signal_no_cp = ofdm_signal[n_cyclic_prefix:n_cyclic_prefix + n_subcarriers]
        
        # FFT
        data_symbols = np.fft.fft(signal_no_cp)
        
        return data_symbols


# ─────────────────────────────────────────────
# 2. 信道编码
# ─────────────────────────────────────────────

class ChannelEncoder:
    """信道编码器"""
    
    @staticmethod
    def hamming_encode(data: np.ndarray, m: int = 3) -> Tuple[np.ndarray, np.ndarray]:
        """
        汉明码编码 (7,4) 为例
        
        Args:
            data: 数据比特
            m: 汉明码参数 (码长=2^m-1, 数据位=2^m-1-m)
        Returns:
            (编码后码字, 校验矩阵)
        """
        n = 2**m - 1  # 码长
        k = n - m      # 数据位数
        
        # 确保数据长度是k的倍数
        padded = np.zeros(((len(data) + k - 1) // k) * k, dtype=int)
        padded[:len(data)] = data
        
        # 生成矩阵
        # P矩阵 (系统码)
        P = np.zeros((k, m), dtype=int)
        for i in range(k):
            val = i + 1
            for j in range(m):
                P[i, j] = (val >> j) & 1
        
        G = np.hstack([np.eye(k, dtype=int), P])
        
        # 编码
        codewords = []
        for i in range(0, len(padded), k):
            block = padded[i:i+k]
            cw = (block @ G) % 2
            codewords.append(cw)
        
        H = np.vstack([np.eye(m, dtype=int), P.T])
        
        return np.concatenate(codewords), H
    
    @staticmethod
    def hamming_decode(received: np.ndarray, H: np.ndarray, m: int = 3) -> Tuple[np.ndarray, int]:
        """
        汉明码解码与纠错
        
        Returns:
            (解码后数据, 纠正的错误数)
        """
        n = 2**m - 1
        k = n - m
        
        decoded = []
        errors_corrected = 0
        
        for i in range(0, len(received), n):
            cw = received[i:i+n]
            if len(cw) < n:
                break
            
            # 计算伴随式
            syndrome = (H.T @ cw) % 2
            syndrome_val = sum(syndrome[j] * 2**j for j in range(m))
            
            # 纠错
            if syndrome_val > 0 and syndrome_val <= n:
                cw[syndrome_val - 1] ^= 1  # 翻转错误位
                errors_corrected += 1
            
            decoded.extend(cw[:k].tolist())
        
        return np.array(decoded, dtype=int), errors_corrected
    
    @staticmethod
    def convolutional_encode(data: np.ndarray, g1: int = 0b111, g2: int = 0b101,
                             constraint_length: int = 3) -> np.ndarray:
        """
        卷积码编码 (K=3, R=1/2)
        
        Args:
            data: 输入比特流
            g1, g2: 生成多项式 (八进制转二进制)
            constraint_length: 约束长度
        Returns:
            编码输出 (交替输出g1和g2)
        """
        # 移位寄存器
        shift_reg = np.zeros(constraint_length, dtype=int)
        output = []
        
        for bit in data:
            shift_reg = np.roll(shift_reg, 1)
            shift_reg[0] = bit
            
            # 计算输出
            out1 = 0
            out2 = 0
            for i in range(constraint_length):
                if (g1 >> i) & 1:
                    out1 ^= shift_reg[i]
                if (g2 >> i) & 1:
                    out2 ^= shift_reg[i]
            
            output.extend([out1, out2])
        
        return np.array(output, dtype=int)
    
    @staticmethod
    def viterbi_decode(received: np.ndarray, g1: int = 0b111, g2: int = 0b101,
                       constraint_length: int = 3) -> np.ndarray:
        """
        维特比译码 (硬判决)
        
        Args:
            received: 接收序列 (软值或硬判决)
            g1, g2: 生成多项式
            constraint_length: 约束长度
        Returns:
            译码输出
        """
        # 状态数
        n_states = 2**(constraint_length - 1)
        n_bits = len(received) // 2
        
        # 路径度量初始化
        path_metrics = np.full(n_states, np.inf)
        path_metrics[0] = 0
        
        # 路径存储
        paths = {s: [] for s in range(n_states)}
        
        # 转移计算
        def get_output(state, input_bit):
            shift_reg = np.zeros(constraint_length, dtype=int)
            # 将状态放入移位寄存器
            for i in range(constraint_length - 1):
                shift_reg[i+1] = (state >> i) & 1
            shift_reg[0] = input_bit
            
            out1 = 0
            out2 = 0
            for i in range(constraint_length):
                if (g1 >> i) & 1:
                    out1 ^= shift_reg[i]
                if (g2 >> i) & 1:
                    out2 ^= shift_reg[i]
            
            next_state = 0
            for i in range(constraint_length - 1):
                next_state |= shift_reg[i] << i
            
            return out1, out2, next_state
        
        # Viterbi算法
        for t in range(n_bits):
            r = received[2*t:2*t+2]
            new_metrics = np.full(n_states, np.inf)
            new_paths = {s: [] for s in range(n_states)}
            
            for state in range(n_states):
                if path_metrics[state] == np.inf:
                    continue
                
                for inp in [0, 1]:
                    out1, out2, next_state = get_output(state, inp)
                    
                    # 分支度量 (汉明距离)
                    branch_metric = (r[0] != out1) + (r[1] != out2)
                    total_metric = path_metrics[state] + branch_metric
                    
                    if total_metric < new_metrics[next_state]:
                        new_metrics[next_state] = total_metric
                        new_paths[next_state] = paths[state] + [inp]
            
            path_metrics = new_metrics
            paths = new_paths
        
        # 选择最佳路径
        best_state = np.argmin(path_metrics)
        return np.array(paths[best_state], dtype=int)
    
    @staticmethod
    def crc_encode(data: np.ndarray, polynomial: int = 0x1021,
                   init_value: int = 0xFFFF) -> np.ndarray:
        """
        CRC编码 (CRC-16/CCITT)
        
        Args:
            data: 输入比特流
            polynomial: CRC多项式
            init_value: 初始值
        Returns:
            含CRC的编码数据
        """
        crc = init_value
        
        for byte_idx in range(0, len(data), 8):
            byte = 0
            for bit_idx in range(8):
                if byte_idx + bit_idx < len(data):
                    byte = (byte << 1) | data[byte_idx + bit_idx]
                else:
                    byte = byte << 1
            
            for _ in range(8):
                if crc & 0x8000:
                    crc = ((crc << 1) ^ polynomial) & 0xFFFF
                else:
                    crc = (crc << 1) & 0xFFFF
                if byte & 0x80:
                    crc ^= 1
                byte = (byte << 1) & 0xFF
        
        # CRC转比特
        crc_bits = np.array([(crc >> i) & 1 for i in range(15, -1, -1)], dtype=int)
        
        return np.concatenate([data, crc_bits])
    
    @staticmethod
    def interleaver(data: np.ndarray, rows: int, cols: int) -> np.ndarray:
        """
        矩阵交织器 (抗突发错误)
        
        Args:
            data: 输入数据
            rows, cols: 交织矩阵大小
        Returns:
            交织后数据
        """
        # 补零
        padded = np.zeros(rows * cols, dtype=int)
        padded[:len(data)] = data
        
        # 写入矩阵 (按行)
        matrix = padded.reshape(rows, cols)
        
        # 读出 (按列)
        return matrix.flatten(order='F')
    
    @staticmethod
    def deinterleaver(data: np.ndarray, rows: int, cols: int) -> np.ndarray:
        """解交织"""
        matrix = data.reshape(cols, rows, order='F').T
        return matrix.flatten()


# ─────────────────────────────────────────────
# 3. 信道模型
# ─────────────────────────────────────────────

class ChannelModel:
    """通信信道模型"""
    
    @staticmethod
    def awgn(signal: np.ndarray, snr_dB: float, signal_power: float = None) -> np.ndarray:
        """
        加性高斯白噪声信道
        
        Args:
            signal: 输入信号
            snr_dB: 信噪比 (dB)
            signal_power: 信号功率 (None=自动计算)
        Returns:
            含噪信号
        """
        if signal_power is None:
            signal_power = np.mean(signal**2) if len(signal) > 0 else 1
        
        snr_linear = 10**(snr_dB / 10)
        noise_power = signal_power / snr_linear
        noise = np.sqrt(noise_power) * np.random.randn(len(signal))
        
        return signal + noise
    
    @staticmethod
    def rayleigh_fading(n_samples: int, n_taps: int = 6,
                        max_doppler: float = 0) -> np.ndarray:
        """
        瑞利衰落信道
        
        Args:
            n_samples: 样本数
            n_taps: 多径数
            max_doppler: 最大多普勒频移 (Hz)
        Returns:
            衰落系数 (复数)
        """
        # 每条路径的增益
        path_gains = (np.random.randn(n_taps) + 1j * np.random.randn(n_taps)) / np.sqrt(2)
        
        # 路径延迟 (指数衰减)
        delays = np.arange(n_taps)
        tau_rms = n_taps / 4
        path_powers = np.exp(-delays / tau_rms)
        path_powers /= np.sum(path_powers)
        
        path_gains *= np.sqrt(path_powers)
        
        # 生成时变衰落
        fading = np.zeros(n_samples, dtype=complex)
        for i, gain in enumerate(path_gains):
            if max_doppler > 0:
                # Jakes模型 (简化)
                phase = 2 * np.pi * max_doppler * np.random.rand() * np.arange(n_samples) / n_samples
                fading += gain * np.exp(1j * phase)
            else:
                fading += gain * np.ones(n_samples)
        
        return fading
    
    @staticmethod
    def rician_fading(n_samples: int, K_dB: float = 10,
                      n_taps: int = 6) -> np.ndarray:
        """
        莱斯衰落信道
        
        Args:
            K_dB: 莱斯因子 (dB), K = 直射/散射功率比
            n_taps: 散射路径数
        Returns:
            衰落系数
        """
        K = 10**(K_dB / 10)
        
        # 直射分量
        los = np.sqrt(K / (K + 1))
        
        # 散射分量
        scatter = np.sqrt(1 / (K + 1)) * (np.random.randn(n_samples) + 1j * np.random.randn(n_samples)) / np.sqrt(2)
        
        return los + scatter
    
    @staticmethod
    def multipath_channel(signal: np.ndarray, delays: np.ndarray,
                          gains: np.ndarray, fs: float) -> np.ndarray:
        """
        多径信道
        
        Args:
            signal: 输入信号
            delays: 延迟数组 (秒)
            gains: 各径增益 (复数)
            fs: 采样率
        Returns:
            多径输出
        """
        output = np.zeros(len(signal) + int(np.max(delays) * fs) + 1, dtype=complex)
        
        for delay, gain in zip(delays, gains):
            delay_samples = int(delay * fs)
            output[delay_samples:delay_samples + len(signal)] += gain * signal
        
        return output[:len(signal)]


# ─────────────────────────────────────────────
# 4. 误码率分析
# ─────────────────────────────────────────────

class BERAnalyzer:
    """误码率分析器"""
    
    @staticmethod
    def theoretical_ber_bpsk(snr_dB: np.ndarray) -> np.ndarray:
        """BPSK理论BER: Q(sqrt(2*SNR))"""
        snr_linear = 10**(snr_dB / 10)
        return 0.5 * erfc(np.sqrt(snr_linear))
    
    @staticmethod
    def theoretical_ber_qpsk(snr_dB: np.ndarray) -> np.ndarray:
        """QPSK理论BER = BPSK BER"""
        return BERAnalyzer.theoretical_ber_bpsk(snr_dB)
    
    @staticmethod
    def theoretical_ber_mpsk(snr_dB: np.ndarray, M: int) -> np.ndarray:
        """M-PSK理论BER (近似)"""
        snr_linear = 10**(snr_dB / 10)
        k = np.log2(M)
        
        if M == 2:
            return BERAnalyzer.theoretical_ber_bpsk(snr_dB)
        elif M == 4:
            return BERAnalyzer.theoretical_ber_qpsk(snr_dB)
        else:
            # 近似公式
            ber = (2 / k) * 0.5 * erfc(np.sqrt(k * snr_linear) * np.sin(np.pi / M))
            return ber
    
    @staticmethod
    def theoretical_ber_qam(snr_dB: np.ndarray, M: int) -> np.ndarray:
        """M-QAM理论BER (近似)"""
        snr_linear = 10**(snr_dB / 10)
        k = np.log2(M)
        sqrt_M = np.sqrt(M)
        
        ber = (4 / k) * (1 - 1/sqrt_M) * 0.5 * erfc(np.sqrt(3*k*snr_linear / (2*(M-1))))
        return ber
    
    @staticmethod
    def theoretical_ber_fsk(snr_dB: np.ndarray, M: int = 2,
                            coherent: bool = True) -> np.ndarray:
        """M-FSK理论BER"""
        snr_linear = 10**(snr_dB / 10)
        
        if coherent and M == 2:
            return 0.5 * erfc(np.sqrt(snr_linear / 2))
        elif not coherent and M == 2:
            return 0.5 * np.exp(-snr_linear / 2)
        else:
            # M-FSK近似
            k = np.log2(M)
            if coherent:
                ber = 0.5 * erfc(np.sqrt(k * snr_linear / 2)) / k
            else:
                ber = 0.5 * np.exp(-snr_linear / 2) / k
            return ber
    
    @staticmethod
    def simulate_ber(mod_order: int, snr_range_dB: np.ndarray,
                     n_bits: int = 100000, mod_type: str = 'psk',
                     channel: str = 'awgn') -> Tuple[np.ndarray, np.ndarray]:
        """
        仿真BER
        
        Args:
            mod_order: 调制阶数
            snr_range_dB: SNR范围
            n_bits: 仿真比特数
            mod_type: 'psk', 'qam', 'fsk'
            channel: 'awgn', 'rayleigh'
        Returns:
            (SNR数组, BER数组)
        """
        ber_results = []
        
        bits_per_symbol = int(np.log2(mod_order))
        n_symbols = n_bits // bits_per_symbol
        
        for snr_dB in snr_range_dB:
            # 生成随机比特
            bits = np.random.randint(0, 2, n_bits)
            
            # 调制
            if mod_type == 'psk' or mod_type == 'qam':
                I, Q = DigitalModulator.psk_modulate(bits, M=mod_order)
                signal = I + 1j * Q
            elif mod_type == 'qam':
                I, Q = DigitalModulator.qam_modulate(bits, M=mod_order)
                signal = I + 1j * Q
            else:
                I, Q = DigitalModulator.psk_modulate(bits, M=mod_order)
                signal = I + 1j * Q
            
            # 信道
            if channel == 'awgn':
                rx = ChannelModel.awgn(np.real(signal), snr_dB) + \
                     1j * ChannelModel.awgn(np.imag(signal), snr_dB)
            elif channel == 'rayleigh':
                fading = ChannelModel.rayleigh_fading(len(signal), K_dB=100)  # 深衰落
                rx = fading * signal
                rx = ChannelModel.awgn(np.real(rx), snr_dB) + \
                     1j * ChannelModel.awgn(np.imag(rx), snr_dB)
            else:
                rx = signal
            
            # 解调
            if mod_type == 'psk':
                rx_bits = DigitalDemodulator.psk_demodulate(np.real(rx), np.imag(rx), M=mod_order)
            elif mod_type == 'qam':
                rx_bits = DigitalDemodulator.qam_demodulate(np.real(rx), np.imag(rx), M=mod_order)
            else:
                rx_bits = DigitalDemodulator.psk_demodulate(np.real(rx), np.imag(rx), M=mod_order)
            
            # 计算BER
            min_len = min(len(bits), len(rx_bits))
            errors = np.sum(bits[:min_len] != rx_bits[:min_len])
            ber = errors / min_len if min_len > 0 else 1.0
            ber_results.append(ber)
        
        return snr_range_dB, np.array(ber_results)
    
    @staticmethod
    def shannon_capacity(snr_dB: np.ndarray, bandwidth_Hz: float = 1) -> np.ndarray:
        """
        香农容量 C = B * log2(1 + SNR)
        
        Args:
            snr_dB: 信噪比 (dB)
            bandwidth_Hz: 带宽
        Returns:
            容量 (bits/s)
        """
        snr_linear = 10**(snr_dB / 10)
        return bandwidth_Hz * np.log2(1 + snr_linear)


# ─────────────────────────────────────────────
# 综合演示
# ─────────────────────────────────────────────

def demo_modulation():
    """调制解调演示"""
    print("=" * 60)
    print("  数字调制仿真演示")
    print("=" * 60)
    
    np.random.seed(42)
    bits = np.random.randint(0, 2, 100)
    
    # BPSK
    I, Q = DigitalModulator.psk_modulate(bits, M=2)
    print(f"  BPSK调制: {len(bits)}比特 → {len(I)}符号")
    print(f"    I值: {np.unique(I)}")
    
    # QPSK
    I, Q = DigitalModulator.psk_modulate(bits, M=4)
    print(f"\n  QPSK调制: {len(bits)}比特 → {len(I)}符号")
    print(f"    I范围: [{I.min():.3f}, {I.max():.3f}]")
    print(f"    Q范围: [{Q.min():.3f}, {Q.max():.3f}]")
    
    # 16QAM
    bits_qam = np.random.randint(0, 2, 400)
    I, Q = DigitalModulator.qam_modulate(bits_qam, M=16)
    print(f"\n  16QAM调制: {len(bits_qam)}比特 → {len(I)}符号")
    print(f"    星座点数: {len(np.unique(I + 1j*Q))}")
    
    # OFDM
    ofdm_data = np.random.randn(64) + 1j * np.random.randn(64)
    ofdm_sig = DigitalModulator.ofdm_modulate(ofdm_data, n_subcarriers=64, n_cyclic_prefix=16)
    print(f"\n  OFDM: 64子载波 + 16CP → {len(ofdm_sig)}采样")


def demo_coding():
    """信道编码演示"""
    print("\n" + "=" * 60)
    print("  信道编码仿真演示")
    print("=" * 60)
    
    # 汉明码
    data = np.array([1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 1])
    encoded, H = ChannelEncoder.hamming_encode(data, m=3)
    print(f"  汉明码(7,4):")
    print(f"    数据: {data}")
    print(f"    编码: {encoded}")
    
    # 引入错误
    received = encoded.copy()
    received[3] ^= 1  # 翻转1位
    print(f"    接收 (第3位错误): {received}")
    
    decoded, n_corrected = ChannelEncoder.hamming_decode(received, H, m=3)
    print(f"    纠错后: {decoded[:len(data)]}, 纠正{n_corrected}个错误")
    
    # 卷积码
    data_conv = np.array([1, 0, 1, 1, 0])
    encoded_conv = ChannelEncoder.convolutional_encode(data_conv)
    print(f"\n  卷积码 (K=3, R=1/2):")
    print(f"    数据: {data_conv}")
    print(f"    编码: {encoded_conv}")
    
    # 交织
    data_inter = np.arange(12)
    interleaved = ChannelEncoder.interleaver(data_inter, rows=3, cols=4)
    print(f"\n  矩阵交织 (3×4):")
    print(f"    输入: {data_inter}")
    print(f"    交织: {interleaved}")


def demo_ber():
    """BER分析演示"""
    print("\n" + "=" * 60)
    print("  误码率分析演示")
    print("=" * 60)
    
    snr_range = np.linspace(0, 15, 16)
    
    # 理论BER
    ber_bpsk = BERAnalyzer.theoretical_ber_bpsk(snr_range)
    ber_qpsk = BERAnalyzer.theoretical_ber_qpsk(snr_range)
    ber_16qam = BERAnalyzer.theoretical_ber_qam(snr_range, M=16)
    ber_64qam = BERAnalyzer.theoretical_ber_qam(snr_range, M=64)
    
    print(f"  理论BER @ SNR=10dB:")
    print(f"    BPSK:    {ber_bpsk[10]:.2e}")
    print(f"    QPSK:    {ber_qpsk[10]:.2e}")
    print(f"    16QAM:   {ber_16qam[10]:.2e}")
    print(f"    64QAM:   {ber_64qam[10]:.2e}")
    
    # 香农容量
    cap = BERAnalyzer.shannon_capacity(snr_range, bandwidth_Hz=1e6)
    print(f"\n  香农容量 @ SNR=10dB, BW=1MHz:")
    print(f"    C = {cap[10]/1e6:.2f} Mbps")
    
    # BER仿真 (小规模)
    print(f"\n  BPSK BER仿真 (10000比特):")
    snr_test = np.array([0, 3, 6, 9, 12])
    _, ber_sim = BERAnalyzer.simulate_ber(2, snr_test, n_bits=10000, mod_type='psk')
    ber_theory = BERAnalyzer.theoretical_ber_bpsk(snr_test)
    
    for i, snr in enumerate(snr_test):
        print(f"    SNR={snr:2d}dB: 仿真BER={ber_sim[i]:.4f}, 理论BER={ber_theory[i]:.4f}")


def demo_channel():
    """信道模型演示"""
    print("\n" + "=" * 60)
    print("  信道模型演示")
    print("=" * 60)
    
    # 瑞利衰落
    fading = ChannelModel.rayleigh_fading(1000, n_taps=6, max_doppler=100)
    print(f"  瑞利衰落:")
    print(f"    幅度均值: {np.mean(np.abs(fading)):.3f}")
    print(f"    幅度标准差: {np.std(np.abs(fading)):.3f}")
    
    # 莱斯衰落
    fading_rice = ChannelModel.rician_fading(1000, K_dB=10)
    print(f"\n  莱斯衰落 (K=10dB):")
    print(f"    幅度均值: {np.mean(np.abs(fading_rice)):.3f}")
    print(f"    幅度标准差: {np.std(np.abs(fading_rice)):.3f}")
    
    # 多径信道
    signal = np.random.randn(1000)
    delays = np.array([0, 1e-6, 3e-6, 5e-6])
    gains = np.array([1.0, 0.5, 0.2, 0.1])
    rx = ChannelModel.multipath_channel(signal, delays, gains, fs=1e6)
    print(f"\n  多径信道 (4径):")
    print(f"    输出信号长度: {len(rx)}")
    print(f"    输出功率比: {np.mean(rx**2)/np.mean(signal**2):.3f}")


if __name__ == "__main__":
    demo_modulation()
    demo_coding()
    demo_ber()
    demo_channel()
    print("\n✓ 数字通信仿真演示完成")
