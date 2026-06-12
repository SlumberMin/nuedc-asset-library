#!/usr/bin/env python3
"""
AT24C02 EEPROM V2 测试 — 基于wrappers.py包装层
覆盖: 初始化、字节读写、多字节读写、页写入、边界条件
对应C源文件: 02_mspm0g3507/drivers/at24c02.h

错误经验检查:
  #9:  测试import wrappers.py而非自行重写
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    AT24C02, AT24C02_ADDR, AT24C02_SIZE,
    AT24C02_PAGE_SIZE, AT24C02_WRITE_CYCLE_MS,
)


class TestAT24C02V2Init(unittest.TestCase):
    """初始化测试"""

    def test_init_success(self):
        """初始化成功"""
        eep = AT24C02()
        ok = eep.init()
        self.assertTrue(ok)
        self.assertTrue(eep.initialized)

    def test_is_ready(self):
        """初始化后is_ready=True"""
        eep = AT24C02()
        eep.init()
        self.assertTrue(eep.is_ready())

    def test_default_addr(self):
        """默认地址0x50"""
        eep = AT24C02()
        self.assertEqual(eep.addr, AT24C02_ADDR)


class TestAT24C02V2Byte(unittest.TestCase):
    """字节读写测试"""

    def test_write_read_byte(self):
        """写入后读出一致"""
        eep = AT24C02()
        eep.init()
        ok = eep.write_byte(0x00, 0xA5)
        self.assertTrue(ok)
        ok, val = eep.read_byte(0x00)
        self.assertTrue(ok)
        self.assertEqual(val, 0xA5)

    def test_write_read_all_addresses(self):
        """遍历所有地址写读"""
        eep = AT24C02()
        eep.init()
        for addr in range(AT24C02_SIZE):
            eep.write_byte(addr, addr & 0xFF)
        for addr in range(AT24C02_SIZE):
            ok, val = eep.read_byte(addr)
            self.assertTrue(ok)
            self.assertEqual(val, addr & 0xFF)

    def test_write_byte_overflow_mask(self):
        """写入值应被0xFF掩码"""
        eep = AT24C02()
        eep.init()
        eep.write_byte(0x00, 0x1FF)
        _, val = eep.read_byte(0x00)
        self.assertEqual(val, 0xFF)

    def test_read_invalid_addr(self):
        """无效地址读取应失败"""
        eep = AT24C02()
        eep.init()
        ok, _ = eep.read_byte(-1)
        self.assertFalse(ok)
        ok, _ = eep.read_byte(AT24C02_SIZE)
        self.assertFalse(ok)

    def test_write_invalid_addr(self):
        """无效地址写入应失败"""
        eep = AT24C02()
        eep.init()
        self.assertFalse(eep.write_byte(-1, 0x55))
        self.assertFalse(eep.write_byte(AT24C02_SIZE, 0x55))


class TestAT24C02V2Multi(unittest.TestCase):
    """多字节读写测试"""

    def test_write_read_multi(self):
        """多字节写入后读出一致"""
        eep = AT24C02()
        eep.init()
        data = bytes([0x01, 0x02, 0x03, 0x04, 0x05])
        ok = eep.write(0x10, data)
        self.assertTrue(ok)
        ok, read_data = eep.read(0x10, 5)
        self.assertTrue(ok)
        self.assertEqual(read_data, data)

    def test_write_full_size(self):
        """写满256字节"""
        eep = AT24C02()
        eep.init()
        data = bytes(range(256))
        ok = eep.write(0, data)
        self.assertTrue(ok)
        ok, read_data = eep.read(0, 256)
        self.assertTrue(ok)
        self.assertEqual(read_data, data)

    def test_write_overflow(self):
        """超出容量应失败"""
        eep = AT24C02()
        eep.init()
        data = bytes(range(10))
        self.assertFalse(eep.write(250, data))  # 250+10=260 > 256

    def test_read_overflow(self):
        """读取超出容量应失败"""
        eep = AT24C02()
        eep.init()
        ok, _ = eep.read(250, 10)
        self.assertFalse(ok)


class TestAT24C02V2Page(unittest.TestCase):
    """页写入测试"""

    def test_write_page_within_boundary(self):
        """页内写入成功"""
        eep = AT24C02()
        eep.init()
        data = bytes([0xAA, 0xBB, 0xCC])
        ok = eep.write_page(0, data)
        self.assertTrue(ok)
        ok, read_data = eep.read(0, 3)
        self.assertEqual(read_data, data)

    def test_write_page_cross_boundary_fail(self):
        """跨页边界写入应失败"""
        eep = AT24C02()
        eep.init()
        # 页0: 0~7, 从地址6写入4字节 → 6+4=10 > 8 → 跨页
        data = bytes([1, 2, 3, 4])
        ok = eep.write_page(6, data)
        self.assertFalse(ok)

    def test_write_page_max_length(self):
        """页写入最大8字节"""
        eep = AT24C02()
        eep.init()
        data = bytes(range(8))
        ok = eep.write_page(0, data)
        self.assertTrue(ok)
        ok, read_data = eep.read(0, 8)
        self.assertEqual(read_data, data)

    def test_write_page_too_long(self):
        """超过页大小应失败"""
        eep = AT24C02()
        eep.init()
        data = bytes(range(9))
        self.assertFalse(eep.write_page(0, data))


class TestAT24C02V2Constants(unittest.TestCase):
    """常量一致性"""

    def test_constants(self):
        self.assertEqual(AT24C02_ADDR, 0x50)
        self.assertEqual(AT24C02_SIZE, 256)
        self.assertEqual(AT24C02_PAGE_SIZE, 8)
        self.assertEqual(AT24C02_WRITE_CYCLE_MS, 5)


if __name__ == '__main__':
    unittest.main()
