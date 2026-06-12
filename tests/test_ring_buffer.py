#!/usr/bin/env python3
"""
环形缓冲区单元测试
覆盖: 初始化、读写、满/空状态、溢出、边界容量、性能基准

V2修复: import wrappers.py中的生产代码逻辑，而非自行重写
对应C源: 02_mspm0g3507/drivers/ring_buffer.c
"""

import sys
import os
import unittest
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from tests.wrappers import RingBuffer


class TestRingBuffer(unittest.TestCase):
    """环形缓冲区测试（与C版本RingBuffer_t逻辑一致）"""

    def test_initialization(self):
        """测试初始化"""
        rb = RingBuffer(64)
        self.assertEqual(rb.size, 64)
        self.assertTrue(rb.is_empty())

    def test_power_of_two_rounding(self):
        """测试2的幂对齐（与C版本一致：必须为2的幂）"""
        rb = RingBuffer(100)
        self.assertEqual(rb.size, 128)
        rb2 = RingBuffer(64)
        self.assertEqual(rb2.size, 64)

    def test_mask_correct(self):
        """测试掩码正确（size-1）"""
        rb = RingBuffer(64)
        self.assertEqual(rb.mask, 63)

    def test_write_read_single(self):
        """测试单字节读写"""
        rb = RingBuffer(16)
        self.assertTrue(rb.put_byte(0x42))
        self.assertFalse(rb.is_empty())
        val = rb.get_byte()
        self.assertEqual(val, 0x42)
        self.assertTrue(rb.is_empty())

    def test_write_read_batch(self):
        """测试批量读写"""
        rb = RingBuffer(32)
        data = b"Hello, World!"
        written = rb.write(data)
        self.assertEqual(written, len(data))
        result = rb.read(len(data))
        self.assertEqual(result, data)

    def test_full_buffer(self):
        """测试缓冲区满"""
        rb = RingBuffer(4)
        for i in range(4):
            self.assertTrue(rb.put_byte(i))
        self.assertTrue(rb.is_full())
        self.assertFalse(rb.put_byte(0xFF))

    def test_empty_read(self):
        """测试空缓冲区读取"""
        rb = RingBuffer(16)
        self.assertIsNone(rb.get_byte())
        self.assertEqual(rb.read(10), b'')

    def test_overflow_protection(self):
        """测试溢出保护（写入量超过容量）"""
        rb = RingBuffer(8)
        data = b'\x00' * 100
        written = rb.write(data)
        self.assertEqual(written, 8)

    def test_wraparound(self):
        """测试回绕"""
        rb = RingBuffer(4)
        rb.write(b'\x01\x02\x03\x04')
        rb.read(4)
        # 再次写入
        self.assertTrue(rb.put_byte(0x42))
        self.assertEqual(rb.get_byte(), 0x42)

    def test_peek(self):
        """测试peek操作（不消费数据）"""
        rb = RingBuffer(16)
        rb.write(b'\x01\x02\x03')
        self.assertEqual(rb.peek(), 0x01)
        # peek不消耗数据
        self.assertEqual(rb.used(), 3)

    def test_reset(self):
        """测试重置"""
        rb = RingBuffer(16)
        rb.write(b'\x01\x02\x03')
        rb.reset()
        self.assertTrue(rb.is_empty())
        self.assertEqual(rb.used(), 0)

    def test_free_space(self):
        """测试剩余空间"""
        rb = RingBuffer(8)
        rb.write(b'\x00\x00\x00')
        self.assertEqual(rb.free_space(), 5)

    def test_used_count(self):
        """测试已用计数"""
        rb = RingBuffer(16)
        self.assertEqual(rb.used(), 0)
        rb.put_byte(0x01)
        rb.put_byte(0x02)
        rb.put_byte(0x03)
        self.assertEqual(rb.used(), 3)

    def test_boundary_single_byte_buffer(self):
        """测试最小缓冲区（1字节）"""
        rb = RingBuffer(1)
        self.assertEqual(rb.size, 1)
        self.assertTrue(rb.put_byte(0x42))
        self.assertTrue(rb.is_full())
        self.assertEqual(rb.get_byte(), 0x42)

    def test_interleaved_operations(self):
        """测试交错操作"""
        rb = RingBuffer(8)
        rb.write(b'\x01\x02\x03')
        self.assertEqual(rb.get_byte(), 0x01)
        rb.write(b'\x04\x05')
        self.assertEqual(rb.read(3), b'\x02\x03\x04')

    def test_write_returns_actual_count(self):
        """测试write返回实际写入字节数"""
        rb = RingBuffer(4)
        self.assertEqual(rb.write(b'\x01\x02\x03'), 3)
        self.assertEqual(rb.write(b'\x04\x05'), 1)  # 只能再写1个

    def test_performance_benchmark(self):
        """性能基准: 环形缓冲区读写性能"""
        rb = RingBuffer(1024)
        data = bytes(range(256)) * 4
        iterations = 1000
        start = time.perf_counter()
        for _ in range(iterations):
            rb.write(data[:512])
            rb.read(512)
        elapsed = time.perf_counter() - start
        ops_per_sec = iterations / elapsed
        self.assertGreater(ops_per_sec, 1000, f"吞吐量: {ops_per_sec:.0f} ops/s")


if __name__ == '__main__':
    unittest.main()
