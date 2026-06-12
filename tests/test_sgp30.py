#!/usr/bin/env python3
"""
SGP30 空气质量传感器 V3 测试 — 基线管理+湿度补偿深度测试
覆盖: V2全部 + 基线读写一致性、基线边界值、湿度补偿多次设置、
      测量值范围验证、CRC多组数据、初始化→测量→基线完整流程
对应C源文件: 02_mspm0g3507/drivers/sgp30.c

错误经验检查:
  #9:  测试import wrappers.py而非自行重写
  #6:  I2C忙等待/超时
  #13: 基线恢复对传感器长期稳定性至关重要
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    SGP30, SGP30_I2C_ADDR,
    SGP30_CMD_INIT_AIR_QUALITY, SGP30_CMD_MEASURE_AIR_QUALITY,
    SGP30_CMD_GET_FEATURE_SET, SGP30_CMD_GET_TVOC_BASELINE,
    SGP30_CMD_SET_TVOC_BASELINE,
    SGP30_CRC_POLYNOMIAL, SGP30_CRC_INIT,
)


class TestSGP30V3Init(unittest.TestCase):
    """SGP30 V3 初始化测试"""

    def test_init_success(self):
        """初始化成功，状态正确"""
        sgp = SGP30()
        ok = sgp.init()
        self.assertTrue(ok)
        self.assertTrue(sgp.initialized)

    def test_default_addr(self):
        """默认I2C地址0x58"""
        sgp = SGP30()
        self.assertEqual(sgp.addr, SGP30_I2C_ADDR)
        self.assertEqual(sgp.addr, 0x58)

    def test_default_values_after_init(self):
        """初始化后TVOC=0, eCO2=400（SGP30规范默认值）"""
        sgp = SGP30()
        sgp.init()
        ok, (tvoc, eco2) = sgp.measure()
        self.assertTrue(ok)
        self.assertEqual(tvoc, 0)
        self.assertEqual(eco2, 400)

    def test_multiple_init_safe(self):
        """多次初始化应安全"""
        sgp = SGP30()
        sgp.init()
        sgp.set_raw_values(100, 500)
        sgp.init()  # 重新初始化
        ok, (tvoc, eco2) = sgp.measure()
        self.assertTrue(ok)
        # 重新初始化后应恢复默认值
        self.assertEqual(tvoc, 0)
        self.assertEqual(eco2, 400)


class TestSGP30V3Measure(unittest.TestCase):
    """SGP30 V3 测量测试"""

    def test_measure_not_init(self):
        """未初始化测量应失败"""
        sgp = SGP30()
        ok, data = sgp.measure()
        self.assertFalse(ok)
        self.assertIsNone(data)

    def test_measure_tvoc_range(self):
        """TVOC典型范围0~60000 ppb"""
        sgp = SGP30()
        sgp.init()
        for tvoc_val in [0, 100, 1000, 5000, 60000]:
            sgp.set_raw_values(tvoc_val, 400)
            ok, (tvoc, eco2) = sgp.measure()
            self.assertTrue(ok)
            self.assertEqual(tvoc, tvoc_val)

    def test_measure_eco2_range(self):
        """eCO2典型范围400~60000 ppm"""
        sgp = SGP30()
        sgp.init()
        for eco2_val in [400, 500, 1000, 5000, 60000]:
            sgp.set_raw_values(0, eco2_val)
            ok, (tvoc, eco2) = sgp.measure()
            self.assertTrue(ok)
            self.assertEqual(eco2, eco2_val)

    def test_measure_stability(self):
        """连续多次测量结果应一致"""
        sgp = SGP30()
        sgp.init()
        sgp.set_raw_values(200, 800)
        for _ in range(100):
            ok, (tvoc, eco2) = sgp.measure()
            self.assertTrue(ok)
            self.assertEqual(tvoc, 200)
            self.assertEqual(eco2, 800)

    def test_measure_overwrite(self):
        """set_raw_values多次覆盖正确"""
        sgp = SGP30()
        sgp.init()
        sgp.set_raw_values(100, 500)
        sgp.set_raw_values(300, 1200)
        ok, (tvoc, eco2) = sgp.measure()
        self.assertEqual(tvoc, 300)
        self.assertEqual(eco2, 1200)


class TestSGP30V3Baseline(unittest.TestCase):
    """SGP30 V3 基线管理深度测试"""

    def test_baseline_default_zero(self):
        """初始化后基线默认为0"""
        sgp = SGP30()
        sgp.init()
        ok, (tvoc_base, eco2_base) = sgp.get_baseline()
        self.assertTrue(ok)
        self.assertEqual(tvoc_base, 0)
        self.assertEqual(eco2_base, 0)

    def test_set_and_get_baseline(self):
        """设置基线后读回一致"""
        sgp = SGP30()
        sgp.init()
        ok = sgp.set_baseline(0x8000, 0x4000)
        self.assertTrue(ok)
        ok, (tvoc_base, eco2_base) = sgp.get_baseline()
        self.assertTrue(ok)
        self.assertEqual(tvoc_base, 0x8000)
        self.assertEqual(eco2_base, 0x4000)

    def test_baseline_boundary_max(self):
        """基线最大值(0xFFFF)边界"""
        sgp = SGP30()
        sgp.init()
        ok = sgp.set_baseline(0xFFFF, 0xFFFF)
        self.assertTrue(ok)
        ok, (tvoc_base, eco2_base) = sgp.get_baseline()
        self.assertEqual(tvoc_base, 0xFFFF)
        self.assertEqual(eco2_base, 0xFFFF)

    def test_baseline_boundary_zero(self):
        """基线零值"""
        sgp = SGP30()
        sgp.init()
        sgp.set_baseline(0x8000, 0x4000)  # 先设一个非零值
        ok = sgp.set_baseline(0, 0)
        self.assertTrue(ok)
        ok, (tvoc_base, eco2_base) = sgp.get_baseline()
        self.assertEqual(tvoc_base, 0)
        self.assertEqual(eco2_base, 0)

    def test_baseline_overwrite(self):
        """多次设置基线，最后一次生效"""
        sgp = SGP30()
        sgp.init()
        sgp.set_baseline(0x1000, 0x2000)
        sgp.set_baseline(0x3000, 0x4000)
        sgp.set_baseline(0x5000, 0x6000)
        ok, (tvoc_base, eco2_base) = sgp.get_baseline()
        self.assertEqual(tvoc_base, 0x5000)
        self.assertEqual(eco2_base, 0x6000)

    def test_set_baseline_not_init(self):
        """未初始化设置基线应失败"""
        sgp = SGP30()
        ok = sgp.set_baseline(0x8000, 0x4000)
        self.assertFalse(ok)

    def test_get_baseline_not_init(self):
        """未初始化获取基线应失败"""
        sgp = SGP30()
        ok, _ = sgp.get_baseline()
        self.assertFalse(ok)

    def test_baseline_independent_of_measure(self):
        """基线与测量值独立"""
        sgp = SGP30()
        sgp.init()
        sgp.set_baseline(0xABCD, 0x1234)
        sgp.set_raw_values(500, 2000)
        ok, (tvoc, eco2) = sgp.measure()
        self.assertTrue(ok)
        self.assertEqual(tvoc, 500)
        # 基线不应被测量影响
        ok, (tvoc_base, eco2_base) = sgp.get_baseline()
        self.assertEqual(tvoc_base, 0xABCD)
        self.assertEqual(eco2_base, 0x1234)


class TestSGP30V3Humidity(unittest.TestCase):
    """SGP30 V3 湿度补偿深度测试"""

    def test_set_humidity_success(self):
        """设置湿度补偿成功"""
        sgp = SGP30()
        sgp.init()
        ok = sgp.set_humidity(50, 25)
        self.assertTrue(ok)

    def test_set_humidity_not_init(self):
        """未初始化设置湿度应失败"""
        sgp = SGP30()
        ok = sgp.set_humidity(50, 25)
        self.assertFalse(ok)

    def test_set_humidity_zero(self):
        """零湿度补偿"""
        sgp = SGP30()
        sgp.init()
        ok = sgp.set_humidity(0, 0)
        self.assertTrue(ok)

    def test_set_humidity_high(self):
        """高湿度补偿（100%, 60°C）"""
        sgp = SGP30()
        sgp.init()
        ok = sgp.set_humidity(100, 60)
        self.assertTrue(ok)

    def test_set_humidity_multiple_times(self):
        """多次设置湿度补偿"""
        sgp = SGP30()
        sgp.init()
        for rh in [20, 40, 60, 80, 50]:
            ok = sgp.set_humidity(rh, 25)
            self.assertTrue(ok)

    def test_set_humidity_does_not_affect_baseline(self):
        """湿度补偿不影响基线"""
        sgp = SGP30()
        sgp.init()
        sgp.set_baseline(0x1234, 0x5678)
        sgp.set_humidity(60, 30)
        ok, (tvoc_base, eco2_base) = sgp.get_baseline()
        self.assertEqual(tvoc_base, 0x1234)
        self.assertEqual(eco2_base, 0x5678)


class TestSGP30V3Selftest(unittest.TestCase):
    """SGP30 V3 自检测试"""

    def test_selftest_pass(self):
        """初始化后自检通过"""
        sgp = SGP30()
        sgp.init()
        self.assertTrue(sgp.selftest())

    def test_selftest_fail_not_init(self):
        """未初始化自检失败"""
        sgp = SGP30()
        self.assertFalse(sgp.selftest())


class TestSGP30V3CRC(unittest.TestCase):
    """SGP30 V3 CRC-8校验测试"""

    def test_crc_empty(self):
        """空数据CRC为初始值0xFF"""
        crc = SGP30.crc8(b'')
        self.assertEqual(crc, 0xFF)

    def test_crc_single_byte(self):
        """单字节CRC"""
        crc = SGP30.crc8(bytes([0x00]))
        self.assertIsInstance(crc, int)
        self.assertNotEqual(crc, 0xFF)

    def test_crc_deterministic(self):
        """CRC确定性：相同输入相同输出"""
        data = bytes([0x20, 0x03])
        self.assertEqual(SGP30.crc8(data), SGP30.crc8(data))

    def test_crc_different_data(self):
        """不同数据CRC不同"""
        crc1 = SGP30.crc8(bytes([0x20, 0x03]))
        crc2 = SGP30.crc8(bytes([0x20, 0x08]))
        self.assertNotEqual(crc1, crc2)

    def test_crc_range(self):
        """CRC值在0~255范围"""
        for i in range(256):
            crc = SGP30.crc8(bytes([i]))
            self.assertGreaterEqual(crc, 0)
            self.assertLessEqual(crc, 255)


class TestSGP30V3Constants(unittest.TestCase):
    """SGP30 V3 常量一致性验证"""

    def test_addr(self):
        self.assertEqual(SGP30_I2C_ADDR, 0x58)

    def test_commands(self):
        self.assertEqual(SGP30_CMD_INIT_AIR_QUALITY, 0x2003)
        self.assertEqual(SGP30_CMD_MEASURE_AIR_QUALITY, 0x2008)
        self.assertEqual(SGP30_CMD_GET_FEATURE_SET, 0x202F)
        self.assertEqual(SGP30_CMD_GET_TVOC_BASELINE, 0x20B3)
        self.assertEqual(SGP30_CMD_SET_TVOC_BASELINE, 0x2077)

    def test_crc_params(self):
        self.assertEqual(SGP30_CRC_POLYNOMIAL, 0x31)
        self.assertEqual(SGP30_CRC_INIT, 0xFF)


class TestSGP30V3FullWorkflow(unittest.TestCase):
    """SGP30 V3 完整工作流程测试"""

    def test_init_measure_baseline_workflow(self):
        """初始化→测量→保存基线→恢复基线完整流程"""
        sgp = SGP30()
        # 1. 初始化
        self.assertTrue(sgp.init())
        # 2. 模拟运行一段时间后测量
        sgp.set_raw_values(300, 1200)
        ok, (tvoc, eco2) = sgp.measure()
        self.assertTrue(ok)
        self.assertEqual(tvoc, 300)
        # 3. 读取基线并保存
        ok, (tvoc_base, eco2_base) = sgp.get_baseline()
        self.assertTrue(ok)
        saved_tvoc, saved_eco2 = tvoc_base, eco2_base
        # 4. 模拟掉电重启
        sgp2 = SGP30()
        sgp2.init()
        # 5. 恢复基线
        ok = sgp2.set_baseline(saved_tvoc, saved_eco2)
        self.assertTrue(ok)
        # 6. 验证恢复
        ok, (tvoc_base2, eco2_base2) = sgp2.get_baseline()
        self.assertEqual(tvoc_base2, saved_tvoc)
        self.assertEqual(eco2_base2, saved_eco2)


if __name__ == '__main__':
    unittest.main()
