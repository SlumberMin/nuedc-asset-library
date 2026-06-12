# -*- coding: utf-8 -*-
"""
test_fm24cl64_v2.py - FM24CL64铁电存储器测试 V2
==================================================
测试内容：
  1. I2C初始化
  2. 单字节读写(FRAM无擦除需求)
  3. 连续读写
  4. 页写入(兼容EEPROM接口)
  5. 数据填充
  6. 数据比较
  7. 边界检查(地址越界)
  8. FRAM特性(无需擦除、直接写入)
  9. 统计计数器
  10. 全地址范围读写

使用 wrappers.py 的 FM24CL64 类
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    FM24CL64,
    FM24CL64_I2C_ADDR, FM24CL64_SIZE, FM24CL64_PAGE_SIZE,
)


class TestFM24CL64V2(unittest.TestCase):
    """FM24CL64铁电存储器V2 — 基于wrappers.py包装层"""

    def setUp(self):
        """每个测试前初始化FRAM"""
        self.fram = FM24CL64()
        self.fram.init()

    # ── 初始化 ──────────────────────────────────

    def test_init_state(self):
        """初始化状态正确"""
        self.assertTrue(self.fram.is_initialized())
        self.assertEqual(self.fram.get_size(), FM24CL64_SIZE)
        self.assertEqual(self.fram.addr, FM24CL64_I2C_ADDR)

    def test_init_counters_zero(self):
        """初始化后计数器为0"""
        self.assertEqual(self.fram.get_write_count(), 0)
        self.assertEqual(self.fram.get_read_count(), 0)
        self.assertEqual(self.fram.get_byte_writes(), 0)

    def test_size_is_8kb(self):
        """存储器大小8KB"""
        self.assertEqual(self.fram.get_size(), 8192)

    # ── 单字节读写 ──────────────────────────────────

    def test_write_read_byte(self):
        """单字节写读"""
        self.assertTrue(self.fram.write_byte(0x100, 0xAA))
        self.assertEqual(self.fram.read_byte(0x100), 0xAA)

    def test_write_read_byte_various(self):
        """多种值写读"""
        test_vals = [0x00, 0x01, 0x55, 0xAA, 0xFF]
        for i, val in enumerate(test_vals):
            addr = i * 100
            self.fram.write_byte(addr, val)
            self.assertEqual(self.fram.read_byte(addr), val)

    def test_write_byte_masked_to_8bit(self):
        """写入值截断到8位"""
        self.fram.write_byte(0, 0x1FF)  # 超过8位
        self.assertEqual(self.fram.read_byte(0), 0xFF)

    # ── 连续读写 ──────────────────────────────────

    def test_continuous_write_read(self):
        """连续写读"""
        data = bytes(range(32))
        self.assertTrue(self.fram.write(0x500, data))
        readback = self.fram.read(0x500, 32)
        self.assertEqual(bytes(readback), data)

    def test_write_read_256_bytes(self):
        """256字节连续写读"""
        data = bytes([i & 0xFF for i in range(256)])
        self.fram.write(0x1000, data)
        readback = self.fram.read(0x1000, 256)
        self.assertEqual(bytes(readback), data)

    def test_read_returns_bytearray(self):
        """read返回bytearray"""
        self.fram.write(0, b'\x01\x02')
        result = self.fram.read(0, 2)
        self.assertIsInstance(result, bytearray)

    # ── 页写入 ──────────────────────────────────

    def test_write_page(self):
        """页写入(兼容EEPROM接口)"""
        data = bytes(range(64))  # 一页
        self.assertTrue(self.fram.write_page(0, data))
        readback = self.fram.read(0, 64)
        self.assertEqual(bytes(readback), data)

    def test_write_page_no_boundary_limit(self):
        """FRAM无页边界限制(不同于EEPROM)"""
        # 跨越页边界写入(FRAM特性)
        data = bytes(range(128))
        addr = FM24CL64_PAGE_SIZE - 32  # 跨越页边界
        self.assertTrue(self.fram.write_page(addr, data))
        readback = self.fram.read(addr, 128)
        self.assertEqual(bytes(readback), data)

    # ── 填充 ──────────────────────────────────

    def test_fill(self):
        """填充"""
        self.fram.fill(0x55, 0x1000, 100)
        for i in range(100):
            self.assertEqual(self.fram.read_byte(0x1000 + i), 0x55)

    def test_fill_entire_memory(self):
        """全存储器填充"""
        self.fram.fill(0xAA)
        # 检查几个位置
        for addr in [0, 4095, 4096, 8191]:
            self.assertEqual(self.fram.read_byte(addr), 0xAA)

    def test_fill_default_full(self):
        """默认fill填充全部"""
        self.fram.fill(0xFF)
        self.assertEqual(self.fram.read_byte(0), 0xFF)
        self.assertEqual(self.fram.read_byte(FM24CL64_SIZE - 1), 0xFF)

    # ── 比较 ──────────────────────────────────

    def test_compare_equal(self):
        """比较相等数据"""
        data = b'\x01\x02\x03\x04'
        self.fram.write(0x200, data)
        self.assertTrue(self.fram.compare(0x200, data))

    def test_compare_not_equal(self):
        """比较不等数据"""
        self.fram.write(0x200, b'\x01\x02\x03\x04')
        self.assertFalse(self.fram.compare(0x200, b'\x01\x02\x03\x05'))

    def test_compare_empty_data(self):
        """空数据比较"""
        self.assertTrue(self.fram.compare(0, b''))

    # ── 边界检查 ──────────────────────────────────

    def test_write_negative_addr(self):
        """负地址写入失败"""
        self.assertFalse(self.fram.write_byte(-1, 0x55))

    def test_read_negative_addr(self):
        """负地址读取返回0"""
        self.assertEqual(self.fram.read_byte(-1), 0x00)

    def test_write_beyond_size(self):
        """超出大小写入失败"""
        self.assertFalse(self.fram.write_byte(FM24CL64_SIZE, 0x55))

    def test_read_beyond_size(self):
        """超出大小读取返回0"""
        self.assertEqual(self.fram.read_byte(FM24CL64_SIZE), 0x00)

    def test_read_clamped_at_end(self):
        """连续读取在末尾截断"""
        self.fram.write(FM24CL64_SIZE - 4, b'\x01\x02\x03\x04')
        readback = self.fram.read(FM24CL64_SIZE - 2, 10)  # 超出2字节
        self.assertEqual(len(readback), 2)

    # ── FRAM特性 ──────────────────────────────────

    def test_no_erase_needed(self):
        """FRAM无需擦除即可直接写入"""
        # 直接写入(不像Flash需要先擦除)
        self.fram.write_byte(0, 0x55)
        self.assertEqual(self.fram.read_byte(0), 0x55)
        # 覆盖写入
        self.fram.write_byte(0, 0xAA)
        self.assertEqual(self.fram.read_byte(0), 0xAA)

    def test_write_readback_immediate(self):
        """FRAM写入后立即可读(无延迟)"""
        data = bytes(range(64))
        self.fram.write(0, data)
        # 立即读取
        readback = self.fram.read(0, 64)
        self.assertEqual(bytes(readback), data)

    def test_overwrite_without_erase(self):
        """反复覆盖写入无需擦除"""
        for val in [0x00, 0xFF, 0x55, 0xAA, 0x12]:
            self.fram.write_byte(0x500, val)
            self.assertEqual(self.fram.read_byte(0x500), val)

    # ── 统计计数 ──────────────────────────────────

    def test_write_count_increments(self):
        """写操作计数"""
        self.fram.write_byte(0, 0x01)
        self.assertEqual(self.fram.get_write_count(), 1)
        self.fram.write(0, b'\x02\x03')
        self.assertEqual(self.fram.get_write_count(), 2)

    def test_read_count_increments(self):
        """读操作计数"""
        self.fram.read_byte(0)
        self.assertEqual(self.fram.get_read_count(), 1)
        self.fram.read(0, 10)
        self.assertEqual(self.fram.get_read_count(), 2)

    def test_byte_writes_increments(self):
        """字节写入计数"""
        self.fram.write_byte(0, 0x01)
        self.assertEqual(self.fram.get_byte_writes(), 1)
        self.fram.write(0, b'\x02\x03\x04')
        self.assertEqual(self.fram.get_byte_writes(), 4)  # 1+3

    # ── 全地址范围 ──────────────────────────────────

    def test_first_byte(self):
        """首字节(地址0)"""
        self.fram.write_byte(0, 0x42)
        self.assertEqual(self.fram.read_byte(0), 0x42)

    def test_last_byte(self):
        """末字节(地址8191)"""
        self.fram.write_byte(FM24CL64_SIZE - 1, 0x42)
        self.assertEqual(self.fram.read_byte(FM24CL64_SIZE - 1), 0x42)

    def test_midpoint(self):
        """中间地址(4096)"""
        self.fram.write_byte(4096, 0x99)
        self.assertEqual(self.fram.read_byte(4096), 0x99)

    def test_stress_write_read(self):
        """压力测试: 多次写读"""
        for i in range(100):
            self.fram.write_byte(i, i & 0xFF)
        for i in range(100):
            self.assertEqual(self.fram.read_byte(i), i & 0xFF)


if __name__ == '__main__':
    unittest.main()
