#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
通信协议可靠性仿真 - 丢包+重传+CRC校验
nuedc-asset-library V3
"""

import numpy as np
import matplotlib.pyplot as plt
import random
import struct
import time

# ============================================================
# 1. CRC校验
# ============================================================

def crc16(data: bytes, poly=0xA001, init=0xFFFF) -> int:
    """CRC-16/Modbus"""
    crc = init
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ poly
            else:
                crc >>= 1
    return crc

def crc8(data: bytes, poly=0x07, init=0x00) -> int:
    """CRC-8"""
    crc = init
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ poly) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc

# ============================================================
# 2. 通信协议帧
# ============================================================

class CommFrame:
    HEADER = 0xAA55

    def __init__(self, seq, cmd, payload):
        self.seq = seq
        self.cmd = cmd
        self.payload = payload  # bytes

    def pack(self):
        """打包为字节流"""
        header = struct.pack('<H', self.HEADER)
        seq = struct.pack('<B', self.seq & 0xFF)
        cmd = struct.pack('<B', self.cmd)
        length = struct.pack('<H', len(self.payload))
        body = seq + cmd + length + self.payload
        checksum = struct.pack('<H', crc16(body))
        return header + body + checksum

    @staticmethod
    def unpack(data):
        """解包，返回 (frame, valid)"""
        if len(data) < 9:
            return None, False
        header = struct.unpack('<H', data[:2])[0]
        if header != CommFrame.HEADER:
            return None, False
        seq = data[2]
        cmd = data[3]
        length = struct.unpack('<H', data[4:6])[0]
        payload = data[6:6+length]
        recv_crc = struct.unpack('<H', data[6+length:8+length])[0]
        calc_crc = crc16(data[2:6+length])
        return CommFrame(seq, cmd, payload), recv_crc == calc_crc

# ============================================================
# 3. 通信信道模型
# ============================================================

class Channel:
    def __init__(self, loss_rate=0.05, bit_error_rate=1e-4, jitter_ms=5.0):
        self.loss_rate = loss_rate
        self.ber = bit_error_rate
        self.jitter_ms = jitter_ms

    def transmit(self, data: bytes) -> bytes:
        """模拟信道传输"""
        # 丢包
        if random.random() < self.loss_rate:
            return None
        # 比特翻转
        data = bytearray(data)
        for i in range(len(data)):
            for bit in range(8):
                if random.random() < self.ber:
                    data[i] ^= (1 << bit)
        # 延迟抖动
        jitter = random.gauss(0, self.jitter_ms)
        return bytes(data)

# ============================================================
# 4. 发送/接收协议栈
# ============================================================

class Sender:
    def __init__(self, channel, max_retries=3, timeout_ms=100):
        self.channel = channel
        self.max_retries = max_retries
        self.timeout_ms = timeout_ms
        self.seq = 0
        self.stats = {'sent': 0, 'acked': 0, 'retries': 0, 'failed': 0}

    def send(self, cmd, payload):
        """发送并等待ACK，返回成功与否"""
        frame = CommFrame(self.seq, cmd, payload)
        data = frame.pack()
        self.stats['sent'] += 1

        for retry in range(self.max_retries + 1):
            rx = self.channel.transmit(data)
            if rx is not None:
                resp, valid = CommFrame.unpack(rx)
                if valid and resp and resp.cmd == 0x06:  # ACK
                    self.stats['acked'] += 1
                    self.seq += 1
                    return True
            if retry < self.max_retries:
                self.stats['retries'] += 1

        self.stats['failed'] += 1
        self.seq += 1
        return False

class Receiver:
    def __init__(self, channel, ack_loss_rate=0.02):
        self.channel = channel
        self.ack_loss = ack_loss_rate
        self.received = []
        self.stats = {'recv': 0, 'crc_pass': 0, 'crc_fail': 0, 'duplicates': 0}
        self.last_seq = -1

    def process(self, data):
        """处理接收帧"""
        if data is None:
            return
        self.stats['recv'] += 1
        frame, valid = CommFrame.unpack(data)
        if not valid:
            self.stats['crc_fail'] += 1
            return
        self.stats['crc_pass'] += 1

        # 重复检测
        if frame.seq == self.last_seq:
            self.stats['duplicates'] += 1
        self.last_seq = frame.seq
        self.received.append(frame.payload)

        # 发ACK（可能丢失）
        ack = CommFrame(frame.seq, 0x06, b'')
        return self.channel.transmit(ack.pack())  # ACK也可能丢

# ============================================================
# 5. 仿真
# ============================================================

def run_comm_simulation():
    np.random.seed(42)
    random.seed(42)

    # 测试不同信道质量
    scenarios = [
        ('理想信道',      0.00, 0,       0),
        ('轻微噪声',      0.02, 1e-5,    2),
        ('中等干扰',      0.05, 1e-4,    5),
        ('严重干扰',      0.15, 1e-3,    15),
        ('极端环境',      0.30, 5e-3,    30),
    ]

    results = []
    n_packets = 1000

    for name, loss, ber, jitter in scenarios:
        ch = Channel(loss_rate=loss, bit_error_rate=ber, jitter_ms=jitter)
        sender = Sender(ch, max_retries=3, timeout_ms=100)
        receiver = Receiver(ch)

        for i in range(n_packets):
            payload = struct.pack('<f', np.sin(i * 0.01))  # 模拟传感器数据
            frame = CommFrame(i, 0x01, payload)
            data = frame.pack()

            # 发送
            rx_data = ch.transmit(data)
            if rx_data is not None:
                frame, valid = CommFrame.unpack(rx_data)
                if valid:
                    receiver.stats['crc_pass'] += 1
                else:
                    receiver.stats['crc_fail'] += 1
                receiver.stats['recv'] += 1
            else:
                # 重传
                for retry in range(3):
                    sender.stats['retries'] += 1
                    rx_data = ch.transmit(data)
                    if rx_data is not None:
                        frame, valid = CommFrame.unpack(rx_data)
                        if valid:
                            receiver.stats['crc_pass'] += 1
                            break
                        receiver.stats['crc_fail'] += 1

            sender.stats['sent'] += 1

        recv_rate = receiver.stats['crc_pass'] / n_packets * 100
        results.append({
            'name': name,
            'loss': loss,
            'ber': ber,
            'recv': receiver.stats['recv'],
            'crc_pass': receiver.stats['crc_pass'],
            'crc_fail': receiver.stats['crc_fail'],
            'success_rate': recv_rate,
            'retries': sender.stats['retries'],
        })

        # 重置
        receiver.stats = {'recv': 0, 'crc_pass': 0, 'crc_fail': 0, 'duplicates': 0}
        sender.stats = {'sent': 0, 'acked': 0, 'retries': 0, 'failed': 0}

    # 可视化
    fig, axes = plt.subplots(2, 2, figsize=(16, 10))

    names = [r['name'] for r in results]
    x = np.arange(len(names))

    # 成功率
    ax = axes[0, 0]
    rates = [r['success_rate'] for r in results]
    colors = ['green' if r > 95 else 'orange' if r > 80 else 'red' for r in rates]
    bars = ax.bar(x, rates, color=colors, alpha=0.7)
    ax.set_xticks(x); ax.set_xticklabels(names, rotation=15)
    ax.set_ylabel('成功率 (%)'); ax.set_title('不同信道条件下的传输成功率')
    ax.axhline(y=95, color='r', linestyle='--', alpha=0.5, label='95%阈值')
    ax.legend(); ax.grid(True, alpha=0.3, axis='y')
    for bar, rate in zip(bars, rates):
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.5,
                f'{rate:.1f}%', ha='center', fontsize=9)

    # CRC错误率
    ax = axes[0, 1]
    crc_fails = [r['crc_fail'] for r in results]
    ax.bar(x, crc_fails, color='red', alpha=0.7, label='CRC失败')
    ax.set_xticks(x); ax.set_xticklabels(names, rotation=15)
    ax.set_ylabel('包数'); ax.set_title('CRC校验失败数量')
    ax.legend(); ax.grid(True, alpha=0.3, axis='y')

    # 重传次数
    ax = axes[1, 0]
    retries = [r['retries'] for r in results]
    ax.bar(x, retries, color='orange', alpha=0.7)
    ax.set_xticks(x); ax.set_xticklabels(names, rotation=15)
    ax.set_ylabel('重传次数'); ax.set_title('总重传次数')
    ax.grid(True, alpha=0.3, axis='y')

    # 丢包率vs成功率散点
    ax = axes[1, 1]
    losses = [r['loss']*100 for r in results]
    ax.scatter(losses, rates, s=100, c=colors, zorder=5)
    for l, r, n in zip(losses, rates, names):
        ax.annotate(n, (l, r), textcoords="offset points", xytext=(5, 5), fontsize=8)
    ax.set_xlabel('丢包率 (%)'); ax.set_ylabel('最终成功率 (%)')
    ax.set_title('丢包率 vs 传输成功率'); ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('communication_protocol_test.png', dpi=150)
    plt.show()

    # 打印结果
    print("=" * 60)
    print("通信协议可靠性仿真结果")
    print("=" * 60)
    print(f"{'场景':12s} {'丢包率':>8s} {'BER':>10s} {'成功率':>8s} {'CRC失败':>8s} {'重传':>8s}")
    print("-" * 60)
    for r in results:
        print(f"{r['name']:12s} {r['loss']*100:7.1f}% {r['ber']:10.1e} {r['success_rate']:7.1f}% {r['crc_fail']:8d} {r['retries']:8d}")

if __name__ == '__main__':
    run_comm_simulation()
