#!/usr/bin/env python3
"""
SHT30 温湿度传感器 V3 测试 — 加热器+单次/连续模式深度测试
覆盖: V2全部 + 加热器开关状态、温度/湿度物理转换、单次测量精度、
      状态寄存器、软复位、CRC校验、边界值
对应C源文件: 02_mspm0g3507/drivers/sht30.c

错误经验检查:
  #9:  测试import wrappers.py而非自行重写
  #6:  I2C忙等待/超时
  #13: SHT30用于环境监测，温湿度精度至关重要
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    SHT30, SHT30_I2C_ADDR, SHT30_I2C_ADDR_LOW, SHT30_I2C_ADDR_HIGH,
    SHT30_CMD_SINGLE_HIGH_CS_EN, SHT30_CMD_SOFT_RESET,
    SHT30_CMD_HEATER_ON, SHT30_CMD_HEATER_OFF, SHT30_CMD_READ_STATUS,
    SHT30_CRC_POLYNOMIAL, SHT30_CRC_INIT,
)


class TestSHT30V3Init(unittest.TestCase):
    """SHT30 V3 初始化测试"""

    def test_init_success(self):
        """初始化成功"""
        sht = SHT30()
        ok = sht.init()
        self.assertTrue(ok)
        self.assertTrue(sht.initialized)

    def test_default_addr(self):
        """默认I2C地址0x44"""
        sht = SHT30()
        self.assertEqual(sht.addr, SHT30_I2C_ADDR)
        self.assertEqual(sht.addr, 0x44)

    def test_addr_variants(self):
        """地址变体：0x44和0x45"""
        self.assertEqual(SHT30_I2C_ADDR_LOW, 0x44)
        self.assertEqual(SHT30_I2C_ADDR_HIGH, 0x45)

    def test_custom_addr(self):
        """自定义地址"""
        sht = SHT30(addr=SHT30_I2C_ADDR_HIGH)
        self.assertEqual(sht.addr, 0x45)

    def test_heater_off_after_init(self):
        """初始化后加热器默认关闭"""
        sht = SHT30()
        sht.init()
        self.assertFalse(sht._heater_on)

    def test_status_zero_after_init(self):
        """初始化后状态寄存器为0"""
        sht = SHT30()
        sht.init()
        ok, status = sht.read_status()
        self.assertTrue(ok)
        self.assertEqual(status, 0x0000)

    def test_multiple_init_safe(self):
        """多次初始化应安全"""
        sht = SHT30()
        sht.init()
        sht.heater_on()
        sht.init()  # 重置
        self.assertFalse(sht._heater_on)


class TestSHT30V3Measure(unittest.TestCase):
    """SHT30 V3 单次测量测试"""

    def test_measure_not_init(self):
        """未初始化测量应失败"""
        sht = SHT30()
        ok, data = sht.measure_single()
        self.assertFalse(ok)
        self.assertIsNone(data)

    def test_measure_default_zero(self):
        """默认原始值=0时，温度=-45°C，湿度=0%"""
        sht = SHT30()
        sht.init()
        ok, (temp, humi) = sht.measure_single()
        self.assertTrue(ok)
        # T = -45 + 175 * 0 / 65535 = -45
        self.assertAlmostEqual(temp, -45.0, places=1)
        # RH = 100 * 0 / 65535 = 0
        self.assertAlmostEqual(humi, 0.0, places=1)

    def test_measure_mid_range(self):
        """中间值：raw=32768时温湿度"""
        sht = SHT30()
        sht.init()
        sht.set_raw_values(32768, 32768)
        ok, (temp, humi) = sht.measure_single()
        self.assertTrue(ok)
        # T = -45 + 175 * 32768 / 65535 ≈ 42.5
        self.assertAlmostEqual(temp, -45.0 + 175.0 * 32768 / 65535, places=1)
        # RH = 100 * 32768 / 65535 ≈ 50
        self.assertAlmostEqual(humi, 100.0 * 32768 / 65535, places=1)

    def test_measure_max_raw(self):
        """最大原始值0xFFFF"""
        sht = SHT30()
        sht.init()
        sht.set_raw_values(0xFFFF, 0xFFFF)
        ok, (temp, humi) = sht.measure_single()
        self.assertTrue(ok)
        # T = -45 + 175 * 65535 / 65535 = 130
        self.assertAlmostEqual(temp, 130.0, places=1)
        # RH = 100 * 65535 / 65535 = 100（被裁剪到100）
        self.assertAlmostEqual(humi, 100.0, places=1)

    def test_measure_typical_room(self):
        """典型室温：25°C, 50%RH"""
        sht = SHT30()
        sht.init()
        # 25°C → raw = (25 + 45) / 175 * 65535 ≈ 26214
        raw_temp = int((25.0 + 45.0) / 175.0 * 65535)
        # 50%RH → raw = 50 / 100 * 65535 ≈ 32768
        raw_humi = int(50.0 / 100.0 * 65535)
        sht.set_raw_values(raw_temp, raw_humi)
        ok, (temp, humi) = sht.measure_single()
        self.assertTrue(ok)
        self.assertAlmostEqual(temp, 25.0, places=0)
        self.assertAlmostEqual(humi, 50.0, places=0)

    def test_humidity_clamped_to_100(self):
        """湿度上限裁剪到100%"""
        sht = SHT30()
        sht.init()
        sht.set_raw_values(0xFFFF, 0xFFFF)
        ok, (temp, humi) = sht.measure_single()
        self.assertLessEqual(humi, 100.0)

    def test_humidity_clamped_to_zero(self):
        """湿度下限裁剪到0%"""
        sht = SHT30()
        sht.init()
        sht.set_raw_values(0, 0)
        ok, (temp, humi) = sht.measure_single()
        self.assertGreaterEqual(humi, 0.0)

    def test_repeated_measure_stability(self):
        """连续多次测量结果一致"""
        sht = SHT30()
        sht.init()
        sht.set_raw_values(40000, 50000)
        for _ in range(50):
            ok, (temp, humi) = sht.measure_single()
            self.assertTrue(ok)
            self.assertAlmostEqual(temp, -45.0 + 175.0 * 40000 / 65535, places=2)


class TestSHT30V3Heater(unittest.TestCase):
    """SHT30 V3 加热器控制测试"""

    def test_heater_on_success(self):
        """开启加热器"""
        sht = SHT30()
        sht.init()
        ok = sht.heater_on()
        self.assertTrue(ok)
        self.assertTrue(sht._heater_on)

    def test_heater_off_success(self):
        """关闭加热器"""
        sht = SHT30()
        sht.init()
        sht.heater_on()
        ok = sht.heater_off()
        self.assertTrue(ok)
        self.assertFalse(sht._heater_on)

    def test_heater_on_not_init(self):
        """未初始化开启加热器应失败"""
        sht = SHT30()
        ok = sht.heater_on()
        self.assertFalse(ok)

    def test_heater_off_not_init(self):
        """未初始化关闭加热器应失败"""
        sht = SHT30()
        ok = sht.heater_off()
        self.assertFalse(ok)

    def test_heater_toggle(self):
        """加热器多次切换"""
        sht = SHT30()
        sht.init()
        for _ in range(10):
            sht.heater_on()
            self.assertTrue(sht._heater_on)
            sht.heater_off()
            self.assertFalse(sht._heater_on)

    def test_heater_on_does_not_affect_measure(self):
        """加热器开关不影响测量接口调用"""
        sht = SHT30()
        sht.init()
        sht.heater_on()
        sht.set_raw_values(30000, 40000)
        ok, (temp, humi) = sht.measure_single()
        self.assertTrue(ok)
        self.assertAlmostEqual(temp, -45.0 + 175.0 * 30000 / 65535, places=2)


class TestSHT30V3SoftReset(unittest.TestCase):
    """SHT30 V3 软复位测试"""

    def test_soft_reset_success(self):
        """软复位成功"""
        sht = SHT30()
        sht.init()
        sht.heater_on()
        ok = sht.soft_reset()
        self.assertTrue(ok)
        self.assertFalse(sht._heater_on)
        self.assertEqual(sht._status, 0x0000)

    def test_soft_reset_not_init(self):
        """未初始化软复位应失败"""
        sht = SHT30()
        ok = sht.soft_reset()
        self.assertFalse(ok)

    def test_soft_reset_restores_defaults(self):
        """软复位恢复默认状态"""
        sht = SHT30()
        sht.init()
        sht.heater_on()
        sht._status = 0x1234
        sht.soft_reset()
        self.assertFalse(sht._heater_on)
        self.assertEqual(sht._status, 0x0000)


class TestSHT30V3Status(unittest.TestCase):
    """SHT30 V3 状态寄存器测试"""

    def test_read_status_not_init(self):
        """未初始化读状态应失败"""
        sht = SHT30()
        ok, status = sht.read_status()
        self.assertFalse(ok)

    def test_read_status_after_init(self):
        """初始化后读状态"""
        sht = SHT30()
        sht.init()
        ok, status = sht.read_status()
        self.assertTrue(ok)
        self.assertEqual(status, 0x0000)


class TestSHT30V3CRC(unittest.TestCase):
    """SHT30 V3 CRC-8校验测试"""

    def test_crc_empty(self):
        """空数据CRC为初始值"""
        crc = SHT30.crc8(b'')
        self.assertEqual(crc, SHT30_CRC_INIT)

    def test_crc_deterministic(self):
        """CRC确定性"""
        data = bytes([0x2C, 0x06])
        self.assertEqual(SHT30.crc8(data), SHT30.crc8(data))

    def test_crc_range(self):
        """CRC值范围0~255"""
        for i in range(256):
            crc = SHT30.crc8(bytes([i]))
            self.assertGreaterEqual(crc, 0)
            self.assertLessEqual(crc, 255)

    def test_crc_different_data(self):
        """不同数据不同CRC"""
        self.assertNotEqual(SHT30.crc8(bytes([0x24, 0x00])),
                            SHT30.crc8(bytes([0x24, 0x16])))


class TestSHT30V3Constants(unittest.TestCase):
    """SHT30 V3 常量一致性验证"""

    def test_addr(self):
        self.assertEqual(SHT30_I2C_ADDR, 0x44)
        self.assertEqual(SHT30_I2C_ADDR_LOW, 0x44)
        self.assertEqual(SHT30_I2C_ADDR_HIGH, 0x45)

    def test_commands(self):
        self.assertEqual(SHT30_CMD_SINGLE_HIGH_CS_EN, 0x2C06)
        self.assertEqual(SHT30_CMD_SOFT_RESET, 0x30A2)
        self.assertEqual(SHT30_CMD_HEATER_ON, 0x306D)
        self.assertEqual(SHT30_CMD_HEATER_OFF, 0x3066)
        self.assertEqual(SHT30_CMD_READ_STATUS, 0xF32D)

    def test_crc_params(self):
        self.assertEqual(SHT30_CRC_POLYNOMIAL, 0x31)
        self.assertEqual(SHT30_CRC_INIT, 0xFF)


class TestSHT30V3FullWorkflow(unittest.TestCase):
    """SHT30 V3 完整工作流程"""

    def test_heater_clean_measure_workflow(self):
        """加热器除湿→关闭→测量完整流程"""
        sht = SHT30()
        sht.init()
        # 1. 开启加热器除湿
        sht.heater_on()
        self.assertTrue(sht._heater_on)
        # 2. 模拟加热一段时间后关闭
        sht.heater_off()
        self.assertFalse(sht._heater_on)
        # 3. 设置正常读数并测量
        raw_temp = int((25.0 + 45.0) / 175.0 * 65535)
        raw_humi = int(55.0 / 100.0 * 65535)
        sht.set_raw_values(raw_temp, raw_humi)
        ok, (temp, humi) = sht.measure_single()
        self.assertTrue(ok)
        self.assertAlmostEqual(temp, 25.0, places=0)
        self.assertAlmostEqual(humi, 55.0, places=0)

    def test_soft_reset_and_reinit_workflow(self):
        """软复位→重新测量"""
        sht = SHT30()
        sht.init()
        sht.set_raw_values(40000, 50000)
        ok, _ = sht.measure_single()
        self.assertTrue(ok)
        # 软复位
        sht.soft_reset()
        # 复位后仍可测量
        sht.set_raw_values(20000, 30000)
        ok, (temp, humi) = sht.measure_single()
        self.assertTrue(ok)
        self.assertAlmostEqual(temp, -45.0 + 175.0 * 20000 / 65535, places=1)


if __name__ == '__main__':
    unittest.main()
