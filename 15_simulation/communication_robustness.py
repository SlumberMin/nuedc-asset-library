# -*- coding: utf-8 -*-
"""
通信鲁棒性仿真 — 丢包重传 + CRC校验 + 误码率

仿真内容：
  1. 信道误码率(BER)分析
  2. CRC-16校验仿真
  3. 丢包重传机制（Stop-and-Wait, Go-Back-N）
  4. 吞吐量与可靠性对比

依赖：numpy, matplotlib
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# ============================================================
# 全局设置
# ============================================================


def main():
    plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False

    np.random.seed(42)

    # ============================================================
    # 1. CRC-16 实现
    # ============================================================
    def crc16_ccitt(data_bits):
        """
        CRC-16/CCITT-FALSE 计算
        输入: data_bits — numpy数组 (0/1)
        返回: 16位CRC余数
        """
        crc = 0xFFFF
        # 将bit转为byte处理
        bytes_data = []
        for i in range(0, len(data_bits), 8):
            byte = 0
            for j in range(8):
                if i + j < len(data_bits):
                    byte = (byte << 1) | int(data_bits[i + j])
            bytes_data.append(byte)

        for byte in bytes_data:
            crc ^= byte << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ 0x1021
                else:
                    crc <<= 1
                crc &= 0xFFFF
        return crc

    def add_crc(data_bits):
        """添加CRC校验"""
        crc = crc16_ccitt(data_bits)
        crc_bits = np.array([(crc >> (15 - i)) & 1 for i in range(16)])
        return np.concatenate([data_bits, crc_bits])

    def check_crc(packet_bits):
        """CRC校验，返回是否正确"""
        return crc16_ccitt(packet_bits) == 0

    # ============================================================
    # 2. 信道模型
    # ============================================================
    def bsc_channel(bits, ber):
        """
        二进制对称信道 (BSC)
        bits: 输入比特
        ber: 误码率
        返回: 受干扰的比特
        """
        noise = (np.random.random(len(bits)) < ber).astype(int)
        return np.bitwise_xor(bits.astype(int), noise)

    def burst_error_channel(bits, ber, burst_prob=0.01, burst_len=10):
        """
        突发错误信道
        ber: 随机误码率
        burst_prob: 突发错误概率
        burst_len: 突发长度
        """
        corrupted = bits.copy()
        # 随机误码
        random_noise = (np.random.random(len(bits)) < ber).astype(int)
        corrupted = np.bitwise_xor(corrupted.astype(int), random_noise)
        # 突发错误
        i = 0
        while i < len(corrupted):
            if np.random.random() < burst_prob:
                end = min(i + burst_len, len(corrupted))
                corrupted[i:end] = 1 - corrupted[i:end]
                i = end
            else:
                i += 1
        return corrupted

    # ============================================================
    # 3. ARQ协议仿真
    # ============================================================
    class StopAndWait:
        """停止-等待协议"""
        def __init__(self, ber, packet_size=128, ack_size=16):
            self.ber = ber
            self.packet_size = packet_size
            self.ack_size = ack_size
            self.timeout = 3  # 超时重传次数上限

        def transmit(self, data_bits, n_packets):
            """传输n_packets个包"""
            total_sent = 0
            total_retx = 0
            success = 0
            latencies = []

            for pkt_idx in range(n_packets):
                # 添加CRC
                packet = data_bits[pkt_idx * self.packet_size:(pkt_idx + 1) * self.packet_size]
                if len(packet) < self.packet_size:
                    packet = np.pad(packet, (0, self.packet_size - len(packet)))
                tx_packet = add_crc(packet)

                retries = 0
                while retries < self.timeout:
                    total_sent += 1
                    # 发送
                    rx_packet = bsc_channel(tx_packet, self.ber)
                    # 校验
                    if check_crc(rx_packet):
                        # ACK
                        ack = np.zeros(self.ack_size)
                        rx_ack = bsc_channel(ack, self.ber)
                        if np.sum(rx_ack) < self.ack_size * self.ber * 2:  # 简化ACK判断
                            success += 1
                            break
                    retries += 1
                    total_retx += 1

                latencies.append(retries + 1)

            return {
                'success_rate': success / n_packets,
                'retransmissions': total_retx,
                'avg_latency': np.mean(latencies),
                'throughput': success * self.packet_size / total_sent / self.packet_size
            }

    class GoBackN:
        """回退N步协议"""
        def __init__(self, ber, packet_size=128, window_size=4):
            self.ber = ber
            self.packet_size = packet_size
            self.window_size = window_size

        def transmit(self, data_bits, n_packets):
            """传输n_packets个包"""
            total_sent = 0
            success = 0
            i = 0

            while i < n_packets:
                # 发送窗口
                window_end = min(i + self.window_size, n_packets)
                window_packets = []
                for j in range(i, window_end):
                    pkt = data_bits[j * self.packet_size:(j + 1) * self.packet_size]
                    if len(pkt) < self.packet_size:
                        pkt = np.pad(pkt, (0, self.packet_size - len(pkt)))
                    window_packets.append(add_crc(pkt))
                    total_sent += 1

                # 模拟传输
                first_error = -1
                for idx, pkt in enumerate(window_packets):
                    rx = bsc_channel(pkt, self.ber)
                    total_sent += 1  # ACK开销
                    if not check_crc(rx):
                        first_error = idx
                        break

                if first_error == -1:
                    # 全部成功
                    success += window_end - i
                    i = window_end
                else:
                    # 回退到第一个错误包
                    success += first_error
                    i = i + first_error

            return {
                'success_rate': success / n_packets,
                'retransmissions': total_sent - n_packets,
                'throughput': success * self.packet_size / total_sent / self.packet_size
            }

    # ============================================================
    # 4. 仿真参数
    # ============================================================
    n_packets = 500         # 包数量
    packet_size = 128       # 包大小 (bits)
    data_bits = np.random.randint(0, 2, n_packets * packet_size)

    ber_values = np.logspace(-5, -1, 20)  # 误码率范围

    # ============================================================
    # 5. 仿真运行
    # ============================================================
    print("仿真1: CRC校验性能...")
    crc_detect_rate = []
    for ber in ber_values:
        detected = 0
        total_errors = 0
        n_test = 1000
        for _ in range(n_test):
            original = add_crc(np.random.randint(0, 2, packet_size))
            corrupted = bsc_channel(original, ber)
            has_error = not np.array_equal(original, corrupted)
            if has_error:
                total_errors += 1
                if check_crc(corrupted):  # 漏检
                    pass
                else:
                    detected += 1
        if total_errors > 0:
            crc_detect_rate.append(detected / total_errors * 100)
        else:
            crc_detect_rate.append(100)
    crc_detect_rate = np.array(crc_detect_rate)

    print("仿真2: 停止-等待协议...")
    sw_results = []
    for ber in ber_values:
        sw = StopAndWait(ber, packet_size)
        res = sw.transmit(data_bits, n_packets)
        sw_results.append(res)

    print("仿真3: 回退N步协议...")
    gbn_results = []
    for ber in ber_values:
        gbn = GoBackN(ber, packet_size, window_size=8)
        res = gbn.transmit(data_bits, n_packets)
        gbn_results.append(res)

    # ============================================================
    # 6. 理论BER vs 丢包率
    # ============================================================
    theoretical_per = 1 - (1 - ber_values)**(packet_size + 16)  # 包含CRC

    # ============================================================
    # 7. 绘图
    # ============================================================
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    # (a) CRC检测率 vs BER
    ax1 = axes[0, 0]
    ax1.semilogx(ber_values, crc_detect_rate, 'b-o', linewidth=2, markersize=4)
    ax1.set_xlabel('误码率 (BER)')
    ax1.set_ylabel('CRC错误检测率 (%)')
    ax1.set_title('(a) CRC-16错误检测率')
    ax1.set_xlim([1e-5, 1e-1])
    ax1.grid(True, alpha=0.3)

    # (b) 包成功率对比
    ax2 = axes[0, 1]
    ax2.semilogx(ber_values, [r['success_rate']*100 for r in sw_results], 'b-o',
                 linewidth=2, label='停止-等待')
    ax2.semilogx(ber_values, [r['success_rate']*100 for r in gbn_results], 'r-s',
                 linewidth=2, label='回退N步')
    ax2.semilogx(ber_values, (1 - theoretical_per) * 100, 'k--',
                 linewidth=1, alpha=0.5, label='无重传理论值')
    ax2.set_xlabel('误码率 (BER)')
    ax2.set_ylabel('包成功率 (%)')
    ax2.set_title('(b) ARQ协议包成功率对比')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    # (c) 吞吐量对比
    ax3 = axes[1, 0]
    ax3.semilogx(ber_values, [r['throughput']*100 for r in sw_results], 'b-o',
                 linewidth=2, label='停止-等待')
    ax3.semilogx(ber_values, [r['throughput']*100 for r in gbn_results], 'r-s',
                 linewidth=2, label='回退N步')
    ax3.set_xlabel('误码率 (BER)')
    ax3.set_ylabel('有效吞吐量 (%)')
    ax3.set_title('(c) ARQ协议吞吐量对比')
    ax3.legend()
    ax3.grid(True, alpha=0.3)

    # (d) 重传次数
    ax4 = axes[1, 1]
    ax4.semilogx(ber_values, [r['retransmissions'] for r in sw_results], 'b-o',
                 linewidth=2, label='停止-等待')
    ax4.semilogx(ber_values, [r['retransmissions'] for r in gbn_results], 'r-s',
                 linewidth=2, label='回退N步')
    ax4.set_xlabel('误码率 (BER)')
    ax4.set_ylabel('重传次数')
    ax4.set_title('(d) ARQ协议重传次数对比')
    ax4.legend()
    ax4.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'communication_robustness.png'), dpi=150, bbox_inches='tight')
    print("图表已保存: communication_robustness.png")
    plt.close('all')

    # 总结
    print("\n=== 通信鲁棒性仿真结果 ===")
    print(f"BER=1e-3时:")
    idx = np.argmin(np.abs(ber_values - 1e-3))
    print(f"  CRC检测率: {crc_detect_rate[idx]:.1f}%")
    print(f"  停止-等待 成功率: {sw_results[idx]['success_rate']*100:.1f}%, 吞吐量: {sw_results[idx]['throughput']*100:.1f}%")
    print(f"  回退N步   成功率: {gbn_results[idx]['success_rate']*100:.1f}%, 吞吐量: {gbn_results[idx]['throughput']*100:.1f}%")

    print("\n=== 通信鲁棒性仿真完成 ===")



if __name__ == '__main__':
    main()
