#!/usr/bin/env python3
"""
I2C扫描工具V2测试 — 基于wrappers.py包装层
覆盖: I2C总线扫描器（设备探测、地址映射、设备识别）
模拟场景: 单设备/多设备扫描、已知设备类型识别、地址范围验证
对应C源文件: 02_mspm0g3507/examples/i2c_scanner_tool.c
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    I2CScanner,
    KNOWN_I2C_DEVICES,
    I2C_ADDR_MIN, I2C_ADDR_MAX,
)


class TestI2CScannerInit(unittest.TestCase):
    """I2C扫描器初始化测试"""

    def test_init_success(self):
        """扫描器初始化成功"""
        scanner = I2CScanner()
        self.assertTrue(scanner.init())
        self.assertTrue(scanner.initialized)

    def test_default_state(self):
        """默认状态：无设备"""
        scanner = I2CScanner()
        scanner.init()
        self.assertEqual(scanner.get_device_count(), 0)
        self.assertEqual(len(scanner.get_found_devices()), 0)


class TestI2CScannerProbe(unittest.TestCase):
    """I2C地址探测测试"""

    def setUp(self):
        self.scanner = I2CScanner()
        self.scanner.init()

    def test_probe_existing_device(self):
        """探测存在的设备"""
        self.scanner.set_simulated_devices([0x3C])
        self.assertTrue(self.scanner.probe_address(0x3C))

    def test_probe_nonexistent_device(self):
        """探测不存在的设备"""
        self.scanner.set_simulated_devices([0x3C])
        self.assertFalse(self.scanner.probe_address(0x3D))

    def test_probe_below_min_addr(self):
        """低于最小地址失败"""
        self.scanner.set_simulated_devices([0x01])
        self.assertFalse(self.scanner.probe_address(0x01))

    def test_probe_above_max_addr(self):
        """高于最大地址失败"""
        self.scanner.set_simulated_devices([0x78])
        self.assertFalse(self.scanner.probe_address(0x78))

    def test_probe_boundary_min(self):
        """最小有效地址0x03"""
        self.scanner.set_simulated_devices([I2C_ADDR_MIN])
        self.assertTrue(self.scanner.probe_address(I2C_ADDR_MIN))

    def test_probe_boundary_max(self):
        """最大有效地址0x77"""
        self.scanner.set_simulated_devices([I2C_ADDR_MAX])
        self.assertTrue(self.scanner.probe_address(I2C_ADDR_MAX))

    def test_probe_not_initialized(self):
        """未初始化探测失败"""
        scanner2 = I2CScanner()
        self.assertFalse(scanner2.probe_address(0x3C))


class TestI2CScannerScanBus(unittest.TestCase):
    """I2C总线扫描测试"""

    def setUp(self):
        self.scanner = I2CScanner()
        self.scanner.init()

    def test_scan_empty_bus(self):
        """空总线扫描"""
        results = self.scanner.scan_bus()
        self.assertEqual(len(results), 0)
        self.assertEqual(self.scanner.get_device_count(), 0)

    def test_scan_single_device(self):
        """扫描单个设备"""
        self.scanner.set_simulated_devices([0x3C])
        results = self.scanner.scan_bus()
        self.assertEqual(len(results), 1)
        self.assertEqual(results[0][0], 0x3C)

    def test_scan_multiple_devices(self):
        """扫描多个设备"""
        devices = [0x3C, 0x48, 0x68, 0x76]
        self.scanner.set_simulated_devices(devices)
        results = self.scanner.scan_bus()
        self.assertEqual(len(results), 4)
        found_addrs = [r[0] for r in results]
        for addr in devices:
            self.assertIn(addr, found_addrs)

    def test_scan_response_time(self):
        """扫描结果包含响应时间"""
        self.scanner.set_simulated_devices([0x3C])
        results = self.scanner.scan_bus()
        self.assertEqual(len(results), 1)
        addr, resp_time = results[0]
        self.assertGreater(resp_time, 0)

    def test_scan_result_sorted(self):
        """扫描结果按地址排序"""
        devices = [0x76, 0x3C, 0x48, 0x68]
        self.scanner.set_simulated_devices(devices)
        results = self.scanner.scan_bus()
        addrs = [r[0] for r in results]
        self.assertEqual(addrs, sorted(addrs))

    def test_scan_not_initialized(self):
        """未初始化扫描返回空"""
        scanner2 = I2CScanner()
        scanner2.set_simulated_devices([0x3C])
        results = scanner2.scan_bus()
        self.assertEqual(len(results), 0)


class TestI2CScannerIdentify(unittest.TestCase):
    """I2C设备识别测试"""

    def setUp(self):
        self.scanner = I2CScanner()
        self.scanner.init()

    def test_identify_known_device(self):
        """识别已知设备（OLED）"""
        info = self.scanner.identify_device(0x3C)
        self.assertEqual(info['name'], 'SSD1306')
        self.assertEqual(info['type'], 'Display')

    def test_identify_mpu6050(self):
        """识别MPU6050"""
        info = self.scanner.identify_device(0x68)
        self.assertEqual(info['name'], 'MPU6050')
        self.assertEqual(info['type'], 'IMU')

    def test_identify_ads1115(self):
        """识别ADS1115 ADC"""
        info = self.scanner.identify_device(0x48)
        self.assertEqual(info['name'], 'ADS1115')

    def test_identify_bmp280(self):
        """识别BMP280气压传感器"""
        info = self.scanner.identify_device(0x76)
        self.assertEqual(info['name'], 'BMP280')

    def test_identify_unknown_device(self):
        """识别未知设备"""
        info = self.scanner.identify_device(0x10)
        self.assertEqual(info['name'], 'Unknown')
        self.assertEqual(info['type'], 'Unknown')

    def test_identify_mcp4725(self):
        """识别MCP4725 DAC"""
        info = self.scanner.identify_device(0x60)
        self.assertEqual(info['type'], 'DAC')


class TestI2CScannerAddressMap(unittest.TestCase):
    """I2C地址映射图测试"""

    def setUp(self):
        self.scanner = I2CScanner()
        self.scanner.init()

    def test_address_map_dimensions(self):
        """地址映射图为8行×16列"""
        self.scanner.set_simulated_devices([0x3C])
        self.scanner.scan_bus()
        addr_map = self.scanner.get_address_map()
        self.assertEqual(len(addr_map), 8)
        for row in addr_map:
            self.assertEqual(len(row), 16)

    def test_address_map_found_device(self):
        """发现设备标记为XX"""
        self.scanner.set_simulated_devices([0x3C])
        self.scanner.scan_bus()
        addr_map = self.scanner.get_address_map()
        # 0x3C = row 3, col 12
        self.assertEqual(addr_map[3][12], 'XX')

    def test_address_map_empty_slot(self):
        """空地址标记为.."""
        self.scanner.set_simulated_devices([])
        self.scanner.scan_bus()
        addr_map = self.scanner.get_address_map()
        # 0x3C应为空
        self.assertEqual(addr_map[3][12], '..')

    def test_address_map_reserved(self):
        """保留地址标记为--"""
        self.scanner.scan_bus()
        addr_map = self.scanner.get_address_map()
        # 0x00 = row 0, col 0 (保留)
        self.assertEqual(addr_map[0][0], '--')
        # 0x78 = row 7, col 8 (超出范围)
        self.assertEqual(addr_map[7][8], '--')


class TestI2CScannerKnownDevices(unittest.TestCase):
    """已知I2C设备数据库验证"""

    def test_known_devices_not_empty(self):
        """已知设备数据库非空"""
        self.assertGreater(len(KNOWN_I2C_DEVICES), 0)

    def test_known_device_format(self):
        """已知设备数据格式正确"""
        for addr, (name, dev_type, desc) in KNOWN_I2C_DEVICES.items():
            self.assertIsInstance(addr, int)
            self.assertIsInstance(name, str)
            self.assertIsInstance(dev_type, str)
            self.assertIsInstance(desc, str)

    def test_address_range_valid(self):
        """有效地址范围 0x03~0x77"""
        self.assertEqual(I2C_ADDR_MIN, 0x03)
        self.assertEqual(I2C_ADDR_MAX, 0x77)


if __name__ == '__main__':
    unittest.main()
