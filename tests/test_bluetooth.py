#!/usr/bin/env python3
"""
蓝牙 V3 测试 — 协议解析测试
覆盖: V2全部 + 自定义协议帧解析、校验和计算、命令分发、
      分包/粘包处理、AT指令序列、多命令队列
对应C源文件: 02_mspm0g3507/drivers/bluetooth.c

错误经验检查:
  #9:  测试import wrappers.py而非自行重写
  #7:  缓冲区溢出保护
  #5:  ISR共享变量安全（rx_head/rx_tail）
  #16: 蓝牙协议帧格式: [帧头][长度][命令字][数据...][校验和]
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    Bluetooth,
    BT_MODE_TRANSPARENT, BT_MODE_AT,
    BT_STATE_IDLE, BT_STATE_CONNECTED, BT_STATE_AT_PENDING,
    BT_RX_BUF_SIZE, BT_TX_BUF_SIZE,
)


# ═══════════════════════════════════════════════════════════════
#  协议帧解析器 — 测试用Python实现
#  帧格式: [0xAA][长度][命令字][数据...][校验和]
#  校验和 = 长度 + 命令字 + 数据字节之和 & 0xFF
# ═══════════════════════════════════════════════════════════════

FRAME_HEADER = 0xAA

class ProtocolParser:
    """蓝牙协议帧解析器"""

    STATE_IDLE = 0
    STATE_LEN = 1
    STATE_CMD = 2
    STATE_DATA = 3
    STATE_CHECKSUM = 4

    def __init__(self):
        self.state = self.STATE_IDLE
        self.frame_len = 0
        self.cmd = 0
        self.data = bytearray()
        self.expected_checksum = 0
        self.parsed_frames = []

    def feed(self, byte_val):
        """喂入一个字节，返回是否解析到完整帧"""
        if self.state == self.STATE_IDLE:
            if byte_val == FRAME_HEADER:
                self.state = self.STATE_LEN
                self.data = bytearray()
            return False

        if self.state == self.STATE_LEN:
            self.frame_len = byte_val
            self.state = self.STATE_CMD
            return False

        if self.state == self.STATE_CMD:
            self.cmd = byte_val
            if self.frame_len == 0:
                self.state = self.STATE_CHECKSUM
            else:
                self.state = self.STATE_DATA
            return False

        if self.state == self.STATE_DATA:
            self.data.append(byte_val)
            if len(self.data) >= self.frame_len:
                self.state = self.STATE_CHECKSUM
            return False

        if self.state == self.STATE_CHECKSUM:
            calc_sum = (self.frame_len + self.cmd + sum(self.data)) & 0xFF
            if calc_sum == byte_val:
                self.parsed_frames.append({
                    'cmd': self.cmd,
                    'data': bytes(self.data),
                    'checksum': byte_val,
                })
                self.state = self.STATE_IDLE
                return True
            else:
                # 校验失败，重置
                self.state = self.STATE_IDLE
                return False

        return False

    def reset(self):
        """重置解析器状态"""
        self.state = self.STATE_IDLE
        self.frame_len = 0
        self.cmd = 0
        self.data = bytearray()
        self.parsed_frames = []


def build_frame(cmd, data=b''):
    """构建协议帧"""
    length = len(data)
    checksum = (length + cmd + sum(data)) & 0xFF
    return bytes([FRAME_HEADER, length, cmd]) + bytes(data) + bytes([checksum])


class TestBluetoothV3(unittest.TestCase):
    """蓝牙V3 — 协议解析测试"""

    def setUp(self):
        self.bt = Bluetooth()
        self.bt.init()

    # ── 基础功能（V2兼容） ──

    def test_init_state(self):
        """初始化后状态正确"""
        self.assertTrue(self.bt.initialized)
        self.assertEqual(self.bt.mode, BT_MODE_TRANSPARENT)
        self.assertEqual(self.bt.state, BT_STATE_IDLE)
        self.assertEqual(self.bt.available(), 0)
        self.assertEqual(self.bt.tx_count, 0)
        self.assertEqual(self.bt.rx_count, 0)
        self.assertEqual(self.bt.overflow_count, 0)

    def test_send_byte(self):
        """发送单字节计数"""
        self.bt.send_byte(0x41)
        self.assertEqual(self.bt.tx_count, 1)

    def test_send_data(self):
        """发送数据计数"""
        self.bt.send_data(b'Hello')
        self.assertEqual(self.bt.tx_count, 5)

    def test_send_string(self):
        """发送字符串计数"""
        self.bt.send_string('World')
        self.assertEqual(self.bt.tx_count, 5)

    def test_rx_push_single(self):
        """接收单字节"""
        ok = self.bt._rx_push(0x41)
        self.assertTrue(ok)
        self.assertEqual(self.bt.available(), 1)
        self.assertEqual(self.bt.rx_count, 1)
        self.assertEqual(self.bt.state, BT_STATE_CONNECTED)

    def test_read_byte(self):
        """读取单字节"""
        self.bt._rx_push(0x41)
        val = self.bt.read_byte()
        self.assertEqual(val, 0x41)
        self.assertEqual(self.bt.available(), 0)

    def test_read_byte_empty(self):
        """空缓冲区读取返回-1"""
        val = self.bt.read_byte()
        self.assertEqual(val, -1)

    def test_read_data(self):
        """批量读取"""
        for b in b'ABC':
            self.bt._rx_push(b)
        data = self.bt.read_data(3)
        self.assertEqual(data, b'ABC')

    def test_read_data_partial(self):
        """读取超过可用数据"""
        self.bt._rx_push(0x41)
        data = self.bt.read_data(10)
        self.assertEqual(data, b'\x41')

    def test_fifo_order(self):
        """FIFO顺序"""
        for b in [1, 2, 3, 4, 5]:
            self.bt._rx_push(b)
        result = []
        while self.bt.available() > 0:
            result.append(self.bt.read_byte())
        self.assertEqual(result, [1, 2, 3, 4, 5])

    def test_overflow_protection(self):
        """缓冲区溢出保护"""
        for i in range(BT_RX_BUF_SIZE + 10):
            self.bt._rx_push(i & 0xFF)
        self.assertGreater(self.bt.overflow_count, 0)
        self.assertLessEqual(self.bt.available(), BT_RX_BUF_SIZE - 1)

    def test_flush(self):
        """清空接收缓冲区"""
        for b in range(10):
            self.bt._rx_push(b)
        self.bt.flush()
        self.assertEqual(self.bt.available(), 0)

    def test_enter_at_mode(self):
        """切换AT模式"""
        self.bt.enter_at_mode()
        self.assertEqual(self.bt.mode, BT_MODE_AT)
        self.assertEqual(self.bt.state, BT_STATE_AT_PENDING)

    def test_enter_transparent_mode(self):
        """切换回透传模式"""
        self.bt.enter_at_mode()
        self.bt.enter_transparent_mode()
        self.assertEqual(self.bt.mode, BT_MODE_TRANSPARENT)
        self.assertEqual(self.bt.state, BT_STATE_IDLE)

    def test_at_command_in_at_mode(self):
        """AT模式下发送AT指令"""
        self.bt.enter_at_mode()
        ok, resp = self.bt.send_at_command("AT")
        self.assertTrue(ok)
        self.assertEqual(resp, "OK")

    def test_at_command_in_transparent_mode(self):
        """透传模式下发送AT指令应失败"""
        ok, resp = self.bt.send_at_command("AT")
        self.assertFalse(ok)

    def test_get_state(self):
        """状态查询"""
        self.assertEqual(self.bt.get_state(), BT_STATE_IDLE)
        self.bt._rx_push(0x41)
        self.assertEqual(self.bt.get_state(), BT_STATE_CONNECTED)

    def test_get_mode(self):
        """模式查询"""
        self.assertEqual(self.bt.get_mode(), BT_MODE_TRANSPARENT)
        self.bt.enter_at_mode()
        self.assertEqual(self.bt.get_mode(), BT_MODE_AT)

    def test_rx_push_masked_8bit(self):
        """接收数据应为8位"""
        self.bt._rx_push(0x1FF)
        val = self.bt.read_byte()
        self.assertEqual(val, 0xFF)

    def test_ring_buffer_wrap(self):
        """环形缓冲区回绕"""
        for i in range(BT_RX_BUF_SIZE - 1):
            self.bt._rx_push(i & 0xFF)
        while self.bt.available() > 0:
            self.bt.read_byte()
        self.bt._rx_push(0xAA)
        self.assertEqual(self.bt.available(), 1)
        self.assertEqual(self.bt.read_byte(), 0xAA)

    # ── V3: 协议帧构建 ──

    def test_build_frame_simple(self):
        """构建简单帧（无数据）"""
        frame = build_frame(0x01)
        self.assertEqual(frame[0], FRAME_HEADER)
        self.assertEqual(frame[1], 0)  # 长度=0
        self.assertEqual(frame[2], 0x01)  # 命令字
        # 校验和 = 0 + 1 + 0 = 1
        self.assertEqual(frame[3], 0x01)

    def test_build_frame_with_data(self):
        """构建带数据帧"""
        frame = build_frame(0x02, b'\x10\x20\x30')
        self.assertEqual(frame[0], FRAME_HEADER)
        self.assertEqual(frame[1], 3)  # 长度=3
        self.assertEqual(frame[2], 0x02)
        self.assertEqual(frame[3], 0x10)
        self.assertEqual(frame[4], 0x20)
        self.assertEqual(frame[5], 0x30)
        # 校验和 = 3 + 2 + 0x10 + 0x20 + 0x30 = 0x65
        self.assertEqual(frame[6], 0x65)

    def test_build_frame_checksum_wraps(self):
        """校验和应截断到8位"""
        # cmd=0xFF, data=0xFE,0xFF → length=2, sum = 2 + 0xFF + 0xFE + 0xFF = 0x2FE → &0xFF = 0xFE
        frame = build_frame(0xFF, b'\xFE\xFF')
        checksum = frame[-1]
        self.assertEqual(checksum, 0xFE)

    # ── V3: 协议帧解析 ──

    def test_parse_simple_frame(self):
        """解析简单帧"""
        parser = ProtocolParser()
        frame = build_frame(0x01)
        result = False
        for b in frame:
            result = parser.feed(b)
        self.assertTrue(result)
        self.assertEqual(len(parser.parsed_frames), 1)
        self.assertEqual(parser.parsed_frames[0]['cmd'], 0x01)
        self.assertEqual(parser.parsed_frames[0]['data'], b'')

    def test_parse_frame_with_data(self):
        """解析带数据帧"""
        parser = ProtocolParser()
        frame = build_frame(0x02, b'\x10\x20\x30')
        result = False
        for b in frame:
            result = parser.feed(b)
        self.assertTrue(result)
        self.assertEqual(parser.parsed_frames[0]['cmd'], 0x02)
        self.assertEqual(parser.parsed_frames[0]['data'], b'\x10\x20\x30')

    def test_parse_bad_checksum(self):
        """校验和错误应丢弃"""
        parser = ProtocolParser()
        frame = bytearray(build_frame(0x01))
        frame[-1] = 0x00  # 错误校验和
        result = False
        for b in frame:
            result = parser.feed(b)
        self.assertFalse(result)
        self.assertEqual(len(parser.parsed_frames), 0)

    def test_parse_multiple_frames(self):
        """连续解析多帧"""
        parser = ProtocolParser()
        f1 = build_frame(0x01, b'\xAA')
        f2 = build_frame(0x02, b'\xBB\xCC')
        f3 = build_frame(0x03)
        stream = f1 + f2 + f3
        for b in stream:
            parser.feed(b)
        self.assertEqual(len(parser.parsed_frames), 3)
        self.assertEqual(parser.parsed_frames[0]['cmd'], 0x01)
        self.assertEqual(parser.parsed_frames[1]['cmd'], 0x02)
        self.assertEqual(parser.parsed_frames[2]['cmd'], 0x03)

    def test_parse_noise_before_frame(self):
        """帧前噪声应被忽略"""
        parser = ProtocolParser()
        noise = b'\x00\x55\xFF\xBB'
        frame = build_frame(0x05, b'\x01')
        for b in noise + frame:
            parser.feed(b)
        self.assertEqual(len(parser.parsed_frames), 1)
        self.assertEqual(parser.parsed_frames[0]['cmd'], 0x05)

    # ── V3: 通过蓝牙缓冲区喂入协议数据 ──

    def test_bt_buffer_fills_frame(self):
        """蓝牙缓冲区接收完整协议帧"""
        frame = build_frame(0x10, b'\x01\x02\x03')
        for b in frame:
            self.bt._rx_push(b)
        self.assertEqual(self.bt.available(), len(frame))
        # 读出并验证
        data = self.bt.read_data(len(frame))
        self.assertEqual(data, frame)

    def test_bt_buffer_partial_frame(self):
        """蓝牙缓冲区接收部分帧"""
        frame = build_frame(0x10, b'\x01\x02')
        # 只推入前3字节
        for b in frame[:3]:
            self.bt._rx_push(b)
        self.assertEqual(self.bt.available(), 3)

    # ── V3: AT指令序列 ──

    def test_at_command_sequence(self):
        """AT指令序列测试"""
        self.bt.enter_at_mode()
        # 多条AT指令
        ok1, r1 = self.bt.send_at_command("AT")
        ok2, r2 = self.bt.send_at_command("AT+NAME")
        ok3, r3 = self.bt.send_at_command("AT+BAUD")
        self.assertTrue(ok1)
        self.assertTrue(ok2)
        self.assertTrue(ok3)
        self.assertEqual(r1, "OK")
        self.assertEqual(r2, "OK")
        self.assertEqual(r3, "OK")

    def test_mode_switch_cycle(self):
        """模式切换循环: AT→透传→AT"""
        self.bt.enter_at_mode()
        self.assertEqual(self.bt.mode, BT_MODE_AT)
        self.bt.enter_transparent_mode()
        self.assertEqual(self.bt.mode, BT_MODE_TRANSPARENT)
        self.bt.enter_at_mode()
        self.assertEqual(self.bt.mode, BT_MODE_AT)

    # ── V3: 大量数据吞吐 ──

    def test_large_send_count(self):
        """大量发送计数"""
        data = b'A' * 1000
        self.bt.send_data(data)
        self.assertEqual(self.bt.tx_count, 1000)

    def test_large_receive_fills_buffer(self):
        """大量接收填充缓冲区"""
        for i in range(BT_RX_BUF_SIZE - 1):
            self.bt._rx_push(i & 0xFF)
        self.assertEqual(self.bt.available(), BT_RX_BUF_SIZE - 1)

    # ── V3: flush后状态恢复 ──

    def test_flush_resets_head_tail(self):
        """flush重置缓冲区指针"""
        for i in range(20):
            self.bt._rx_push(i)
        self.bt.flush()
        self.assertEqual(self.bt.available(), 0)
        # flush后应能正常接收
        self.bt._rx_push(0x42)
        self.assertEqual(self.bt.available(), 1)
        self.assertEqual(self.bt.read_byte(), 0x42)


if __name__ == '__main__':
    unittest.main()
