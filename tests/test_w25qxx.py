# -*- coding: utf-8 -*-
"""
test_w25qxx_v2.py - W25Qxx SPI Flash测试 V2
==============================================
测试内容：
  1. Flash初始化与JEDEC ID读取
  2. 写使能/写禁止流程
  3. 页编程(写入)与数据读取
  4. 扇区擦除(4KB)
  5. 块擦除(32KB/64KB)
  6. 全片擦除
  7. Flash NAND特性(只能1→0, 擦除后全FF)
  8. 掉电/唤醒
  9. 状态寄存器读取
  10. 统计计数器

使用 wrappers.py 的 W25Qxx 类
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    W25Qxx,
    W25Q16_JEDEC_ID, W25Q32_JEDEC_ID, W25Q64_JEDEC_ID,
    W25Q16_TOTAL_SIZE, W25Q16_SECTOR_SIZE, W25Q16_PAGE_SIZE,
    W25Q16_BLOCK_SIZE_32K, W25Q16_BLOCK_SIZE_64K,
)


class TestW25QxxV2(unittest.TestCase):
    """W25Qxx SPI Flash V2 — 基于wrappers.py包装层"""

    def setUp(self):
        """每个测试前初始化Flash"""
        self.flash = W25Qxx()
        self.flash.init()

    # ── 初始化 ──────────────────────────────────

    def test_init_flash_size(self):
        """Flash大小2MB"""
        self.assertEqual(self.flash.get_flash_size(), W25Q16_TOTAL_SIZE)

    def test_init_not_busy(self):
        """初始化后不忙"""
        self.assertFalse(self.flash.is_busy())

    def test_init_write_disabled(self):
        """初始化后写禁止"""
        status = self.flash.read_status()
        self.assertEqual(status & 0x02, 0)  # WEL=0

    # ── JEDEC ID ──────────────────────────────────

    def test_read_jedec_id_w25q16(self):
        """W25Q16 JEDEC ID"""
        jedec = self.flash.read_jedec_id()
        self.assertEqual(jedec, W25Q16_JEDEC_ID)
        self.assertEqual(jedec[0], 0xEF)  # Winbond厂商
        self.assertEqual(jedec[1], 0x40)  # 内存类型
        self.assertEqual(jedec[2], 0x15)  # 容量

    def test_read_jedec_id_w25q32(self):
        """W25Q32 JEDEC ID"""
        flash32 = W25Qxx(jedec_id=W25Q32_JEDEC_ID)
        self.assertEqual(flash32.read_jedec_id(), W25Q32_JEDEC_ID)

    def test_read_jedec_id_w25q64(self):
        """W25Q64 JEDEC ID"""
        flash64 = W25Qxx(jedec_id=W25Q64_JEDEC_ID)
        self.assertEqual(flash64.read_jedec_id(), W25Q64_JEDEC_ID)

    # ── 写使能/禁止 ──────────────────────────────────

    def test_write_enable_sets_wel(self):
        """写使能设置WEL位"""
        self.flash.write_enable()
        status = self.flash.read_status()
        self.assertTrue(status & 0x02)

    def test_write_disable_clears_wel(self):
        """写禁止清除WEL位"""
        self.flash.write_enable()
        self.flash.write_disable()
        status = self.flash.read_status()
        self.assertFalse(status & 0x02)

    def test_program_without_write_enable_fails(self):
        """未写使能时编程失败"""
        result = self.flash.page_program(0x0000, b'\xAB\xCD')
        self.assertFalse(result)

    def test_erase_without_write_enable_fails(self):
        """未写使能时擦除失败"""
        result = self.flash.sector_erase(0x0000)
        self.assertFalse(result)

    # ── 页编程(写入) ──────────────────────────────────

    def test_page_program_basic(self):
        """基本页编程"""
        self.flash.write_enable()
        data = b'\x01\x02\x03\x04\x05'
        result = self.flash.page_program(0x1000, data)
        self.assertTrue(result)

    def test_read_after_program(self):
        """编程后读取数据"""
        self.flash.write_enable()
        data = b'\xAA\xBB\xCC\xDD'
        self.flash.page_program(0x2000, data)
        readback = self.flash.read_data(0x2000, 4)
        self.assertEqual(readback, data)

    def test_program_single_byte(self):
        """单字节编程"""
        self.flash.write_enable()
        self.flash.page_program(0x3000, b'\x55')
        self.assertEqual(self.flash.read_byte(0x3000), 0x55)

    def test_program_auto_disables_write(self):
        """编程后自动关闭写使能"""
        self.flash.write_enable()
        self.flash.page_program(0x0000, b'\xFF')
        status = self.flash.read_status()
        self.assertFalse(status & 0x02)  # WEL应被清除

    # ── Flash NAND特性 ──────────────────────────────────

    def test_flash_and_behavior(self):
        """Flash只能把1变成0(NAND特性)"""
        # 先擦除(全FF)
        self.flash.write_enable()
        self.flash.sector_erase(0x0000)
        # 写入0xAA
        self.flash.write_enable()
        self.flash.page_program(0x0000, b'\xAA')
        self.assertEqual(self.flash.read_byte(0x0000), 0xAA)
        # 再写入0x55(AND: 0xAA & 0x55 = 0x00)
        self.flash.write_enable()
        self.flash.page_program(0x0000, b'\x55')
        self.assertEqual(self.flash.read_byte(0x0000), 0x00)

    def test_erased_state_is_ff(self):
        """擦除后全为0xFF"""
        self.flash.write_enable()
        self.flash.sector_erase(0x0000)
        data = self.flash.read_data(0x0000, 16)
        self.assertTrue(all(b == 0xFF for b in data))

    def test_read_unwritten_returns_ff(self):
        """未写入区域读取返回0xFF"""
        data = self.flash.read_data(0xF0000, 8)
        self.assertTrue(all(b == 0xFF for b in data))

    # ── 扇区擦除 ──────────────────────────────────

    def test_sector_erase(self):
        """扇区擦除(4KB)"""
        # 先写入数据
        self.flash.write_enable()
        self.flash.sector_erase(0x5000)  # 先擦除
        self.flash.write_enable()
        self.flash.page_program(0x5000, b'\x12\x34\x56')
        self.assertEqual(self.flash.read_byte(0x5000), 0x12)
        # 擦除扇区
        self.flash.write_enable()
        result = self.flash.sector_erase(0x5000)
        self.assertTrue(result)
        # 擦除后全FF
        self.assertEqual(self.flash.read_byte(0x5000), 0xFF)

    def test_sector_erase_alignment(self):
        """扇区擦除自动对齐"""
        self.flash.write_enable()
        self.flash.sector_erase(0x5001)  # 非对齐地址
        # 应擦除0x5000所在扇区
        data = self.flash.read_data(0x5000, 4)
        self.assertTrue(all(b == 0xFF for b in data))

    # ── 块擦除 ──────────────────────────────────

    def test_block_erase_32k(self):
        """32KB块擦除"""
        self.flash.write_enable()
        self.flash.page_program(0x10000, b'\xAA')
        self.flash.write_enable()
        result = self.flash.block_erase_32k(0x10000)
        self.assertTrue(result)
        self.assertEqual(self.flash.read_byte(0x10000), 0xFF)

    def test_block_erase_64k(self):
        """64KB块擦除"""
        self.flash.write_enable()
        self.flash.page_program(0x20000, b'\xBB')
        self.flash.write_enable()
        result = self.flash.block_erase_64k(0x20000)
        self.assertTrue(result)
        self.assertEqual(self.flash.read_byte(0x20000), 0xFF)

    # ── 全片擦除 ──────────────────────────────────

    def test_chip_erase(self):
        """全片擦除"""
        # 写入多个位置
        for addr in [0x0000, 0x10000, 0x100000]:
            self.flash.write_enable()
            self.flash.page_program(addr, b'\xCC')
        # 全片擦除
        self.flash.write_enable()
        result = self.flash.chip_erase()
        self.assertTrue(result)
        # 验证
        for addr in [0x0000, 0x10000, 0x100000]:
            self.assertEqual(self.flash.read_byte(addr), 0xFF)

    def test_chip_erase_without_wel_fails(self):
        """未写使能全片擦除失败"""
        result = self.flash.chip_erase()
        self.assertFalse(result)

    # ── 掉电/唤醒 ──────────────────────────────────

    def test_power_down(self):
        """进入掉电模式"""
        self.flash.power_down()
        self.assertTrue(self.flash.is_powered_down())

    def test_release_power_down(self):
        """退出掉电模式"""
        self.flash.power_down()
        self.flash.release_power_down()
        self.assertFalse(self.flash.is_powered_down())

    def test_power_down_preserves_data(self):
        """掉电不丢失数据"""
        self.flash.write_enable()
        self.flash.sector_erase(0x0000)
        self.flash.write_enable()
        self.flash.page_program(0x0000, b'\xDE\xAD')
        self.flash.power_down()
        self.flash.release_power_down()
        self.assertEqual(self.flash.read_data(0x0000, 2), b'\xDE\xAD')

    # ── 统计计数 ──────────────────────────────────

    def test_erase_count(self):
        """擦除计数"""
        self.assertEqual(self.flash.get_erase_count(), 0)
        self.flash.write_enable()
        self.flash.sector_erase(0x0000)
        self.assertEqual(self.flash.get_erase_count(), 1)

    def test_program_count(self):
        """编程计数"""
        self.assertEqual(self.flash.get_program_count(), 0)
        self.flash.write_enable()
        self.flash.page_program(0x0000, b'\x01')
        self.assertEqual(self.flash.get_program_count(), 1)

    def test_read_count(self):
        """读取计数"""
        self.assertEqual(self.flash.get_read_count(), 0)
        self.flash.read_byte(0x0000)
        self.assertEqual(self.flash.get_read_count(), 1)
        self.flash.read_data(0x0000, 10)
        self.assertEqual(self.flash.get_read_count(), 2)

    # ── 读取 ──────────────────────────────────

    def test_read_byte(self):
        """单字节读取"""
        self.flash.write_enable()
        self.flash.sector_erase(0x0000)
        self.flash.write_enable()
        self.flash.page_program(0x100, b'\x42')
        self.assertEqual(self.flash.read_byte(0x100), 0x42)

    def test_read_data_multiple_bytes(self):
        """多字节连续读取"""
        self.flash.write_enable()
        self.flash.sector_erase(0x0000)
        self.flash.write_enable()
        data = bytes(range(32))
        self.flash.page_program(0x200, data)
        readback = self.flash.read_data(0x200, 32)
        self.assertEqual(readback, data)

    def test_read_beyond_flash_returns_ff(self):
        """超出Flash范围返回0xFF"""
        self.assertEqual(self.flash.read_byte(W25Q16_TOTAL_SIZE), 0xFF)


if __name__ == '__main__':
    unittest.main()
