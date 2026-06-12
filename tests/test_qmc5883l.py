#!/usr/bin/env python3
"""
QMC5883L 三轴电子罗盘 V3 测试 — 椭圆拟合校准+航向计算深度测试
覆盖: V2全部 + 航向角多象限验证、磁场单位转换、校准偏移/缩放、
      芯片ID、软复位、温度读取、边界值
对应C源文件: 02_mspm0g3507/drivers/qmc5883l.c

错误经验检查:
  #9:  测试import wrappers.py而非自行重写
  #6:  I2C忙等待/超时
  #13: 磁力计校准精度直接影响航向角准确性
"""

import sys
import os
import math
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    QMC5883L, QMC5883L_Data,
    QMC5883L_I2C_ADDR, QMC5883L_CHIP_ID_VALUE,
    QMC5883L_RANGE_2G, QMC5883L_RANGE_8G,
)


def apply_calibration(raw_x, raw_y, raw_z, offset_x=0, offset_y=0, offset_z=0,
                      scale_x=1.0, scale_y=1.0, scale_z=1.0):
    """
    椭圆拟合校准: 减去偏移，乘以缩放因子
    raw: 原始LSB值
    offset: 零偏（椭圆中心）
    scale: 缩放因子（使椭圆变圆）
    返回: (cal_x, cal_y, cal_z) 单位LSB
    """
    cal_x = (raw_x - offset_x) * scale_x
    cal_y = (raw_y - offset_y) * scale_y
    cal_z = (raw_z - offset_z) * scale_z
    return cal_x, cal_y, cal_z


def calc_heading_from_calibrated(cal_x, cal_y):
    """
    从校准后的磁场计算航向角（0~360°）
    """
    heading = math.atan2(cal_y, cal_x) * 180.0 / math.pi
    if heading < 0:
        heading += 360.0
    return heading


class TestQMC5883LV3Init(unittest.TestCase):
    """QMC5883L V3 初始化测试"""

    def test_init_success(self):
        """初始化成功"""
        qmc = QMC5883L()
        ok = qmc.init()
        self.assertTrue(ok)
        self.assertTrue(qmc.initialized)

    def test_default_addr(self):
        """默认I2C地址0x0D"""
        qmc = QMC5883L()
        self.assertEqual(qmc.addr, QMC5883L_I2C_ADDR)
        self.assertEqual(qmc.addr, 0x0D)

    def test_default_range(self):
        """默认量程±2G"""
        qmc = QMC5883L()
        self.assertEqual(qmc._range, QMC5883L_RANGE_2G)

    def test_init_with_range_8g(self):
        """初始化时设置±8G量程"""
        qmc = QMC5883L()
        ok = qmc.init(range_cfg=QMC5883L_RANGE_8G)
        self.assertTrue(ok)
        self.assertEqual(qmc._range, QMC5883L_RANGE_8G)

    def test_read_chip_id(self):
        """芯片ID应为0xFF"""
        qmc = QMC5883L()
        qmc.init()
        ok, chip_id = qmc.read_chip_id()
        self.assertTrue(ok)
        self.assertEqual(chip_id, QMC5883L_CHIP_ID_VALUE)
        self.assertEqual(chip_id, 0xFF)

    def test_multiple_init_safe(self):
        """多次初始化安全"""
        qmc = QMC5883L()
        qmc.init()
        qmc.set_raw_values(1000, 2000, 3000)
        qmc.init()
        # init不重置raw值（与C版本行为一致），但initialized应为True
        self.assertTrue(qmc.initialized)


class TestQMC5883LV3ReadData(unittest.TestCase):
    """QMC5883L V3 数据读取测试"""

    def test_read_not_init(self):
        """未初始化读取应失败"""
        qmc = QMC5883L()
        ok, data = qmc.read_data()
        self.assertFalse(ok)
        self.assertIsNone(data)

    def test_read_default_zero(self):
        """默认原始值全零"""
        qmc = QMC5883L()
        qmc.init()
        ok, data = qmc.read_data()
        self.assertTrue(ok)
        self.assertEqual(data.x, 0)
        self.assertEqual(data.y, 0)
        self.assertEqual(data.z, 0)

    def test_set_and_read_raw(self):
        """设置并读取原始值"""
        qmc = QMC5883L()
        qmc.init()
        qmc.set_raw_values(5000, -3000, 8000)
        ok, data = qmc.read_data()
        self.assertTrue(ok)
        self.assertEqual(data.x, 5000)
        self.assertEqual(data.y, -3000)
        self.assertEqual(data.z, 8000)


class TestQMC5883LV3Conversion(unittest.TestCase):
    """QMC5883L V3 磁场单位转换测试"""

    def test_gauss_2g_range(self):
        """±2G量程: 12000 LSB/Gauss"""
        qmc = QMC5883L()
        qmc.init(range_cfg=QMC5883L_RANGE_2G)
        qmc.set_raw_values(12000, 0, 0)
        ok, data = qmc.read_data()
        self.assertTrue(ok)
        self.assertAlmostEqual(data.x_gauss, 1.0, places=2)

    def test_gauss_8g_range(self):
        """±8G量程: 3000 LSB/Gauss"""
        qmc = QMC5883L()
        qmc.init(range_cfg=QMC5883L_RANGE_8G)
        qmc.set_raw_values(3000, 0, 0)
        ok, data = qmc.read_data()
        self.assertTrue(ok)
        self.assertAlmostEqual(data.x_gauss, 1.0, places=2)

    def test_gauss_negative(self):
        """负磁场值"""
        qmc = QMC5883L()
        qmc.init()
        qmc.set_raw_values(-12000, 0, 0)
        ok, data = qmc.read_data()
        self.assertAlmostEqual(data.x_gauss, -1.0, places=2)


class TestQMC5883LV3Heading(unittest.TestCase):
    """QMC5883L V3 航向角计算测试"""

    def test_heading_north(self):
        """北方: y=0, x>0 → 0°"""
        qmc = QMC5883L()
        qmc.init()
        qmc.set_raw_values(10000, 0, 0)
        ok, data = qmc.read_data()
        self.assertTrue(ok)
        self.assertAlmostEqual(data.heading_deg, 0.0, places=0)

    def test_heading_east(self):
        """东方: x=0, y>0 → 90°"""
        qmc = QMC5883L()
        qmc.init()
        qmc.set_raw_values(0, 10000, 0)
        ok, data = qmc.read_data()
        self.assertTrue(ok)
        self.assertAlmostEqual(data.heading_deg, 90.0, places=0)

    def test_heading_south(self):
        """南方: y=0, x<0 → 180°"""
        qmc = QMC5883L()
        qmc.init()
        qmc.set_raw_values(-10000, 0, 0)
        ok, data = qmc.read_data()
        self.assertTrue(ok)
        self.assertAlmostEqual(data.heading_deg, 180.0, places=0)

    def test_heading_west(self):
        """西方: x=0, y<0 → 270°"""
        qmc = QMC5883L()
        qmc.init()
        qmc.set_raw_values(0, -10000, 0)
        ok, data = qmc.read_data()
        self.assertTrue(ok)
        self.assertAlmostEqual(data.heading_deg, 270.0, places=0)

    def test_heading_ne_45(self):
        """东北45°"""
        qmc = QMC5883L()
        qmc.init()
        val = 7071  # 10000/√2 ≈ 7071
        qmc.set_raw_values(val, val, 0)
        ok, data = qmc.read_data()
        self.assertTrue(ok)
        self.assertAlmostEqual(data.heading_deg, 45.0, places=0)

    def test_heading_range_0_to_360(self):
        """航向角范围0~360°"""
        qmc = QMC5883L()
        qmc.init()
        for angle_deg in range(0, 360, 30):
            rad = math.radians(angle_deg)
            x = int(10000 * math.cos(rad))
            y = int(10000 * math.sin(rad))
            qmc.set_raw_values(x, y, 0)
            ok, data = qmc.read_data()
            self.assertTrue(ok)
            self.assertGreaterEqual(data.heading_deg, 0.0)
            self.assertLess(data.heading_deg, 360.1)


class TestQMC5883LV3Calibration(unittest.TestCase):
    """QMC5883L V3 椭圆拟合校准测试"""

    def test_calibration_no_offset(self):
        """无偏移时校准后不变"""
        cal_x, cal_y, cal_z = apply_calibration(5000, 3000, 8000)
        self.assertEqual(cal_x, 5000)
        self.assertEqual(cal_y, 3000)
        self.assertEqual(cal_z, 8000)

    def test_calibration_with_offset(self):
        """减去偏移"""
        cal_x, cal_y, cal_z = apply_calibration(
            5000, 3000, 8000,
            offset_x=1000, offset_y=500, offset_z=2000
        )
        self.assertEqual(cal_x, 4000)
        self.assertEqual(cal_y, 2500)
        self.assertEqual(cal_z, 6000)

    def test_calibration_with_scale(self):
        """缩放因子"""
        cal_x, cal_y, cal_z = apply_calibration(
            10000, 10000, 10000,
            scale_x=1.0, scale_y=1.2, scale_z=0.8
        )
        self.assertEqual(cal_x, 10000)
        self.assertEqual(cal_y, 12000)
        self.assertEqual(cal_z, 8000)

    def test_calibration_offset_and_scale(self):
        """偏移+缩放组合"""
        cal_x, cal_y, cal_z = apply_calibration(
            10000, 10000, 10000,
            offset_x=1000, offset_y=2000, offset_z=3000,
            scale_x=1.1, scale_y=1.0, scale_z=0.9
        )
        self.assertAlmostEqual(cal_x, (10000 - 1000) * 1.1)
        self.assertAlmostEqual(cal_y, (10000 - 2000) * 1.0)
        self.assertAlmostEqual(cal_z, (10000 - 3000) * 0.9)

    def test_calibration_heading_improvement(self):
        """校准后航向角更准确（模拟椭圆→圆变换）"""
        # 模拟未校准数据: 椭圆压缩，y轴偏移500
        raw_x = 10000
        raw_y = 500  # 应该是0但有偏移
        # 未校准航向
        raw_heading = math.atan2(raw_y, raw_x) * 180 / math.pi
        # 校准后
        cal_x, cal_y, _ = apply_calibration(raw_x, raw_y, 0, offset_y=500)
        cal_heading = math.atan2(cal_y, cal_x) * 180 / math.pi
        # 校准后应更接近0°
        self.assertLess(abs(cal_heading), abs(raw_heading))


class TestQMC5883LV3Temperature(unittest.TestCase):
    """QMC5883L V3 温度读取测试"""

    def test_temperature_default(self):
        """默认温度"""
        qmc = QMC5883L()
        qmc.init()
        ok, data = qmc.read_data()
        self.assertTrue(ok)
        self.assertEqual(data.temperature, 0.0)

    def test_temperature_conversion(self):
        """温度转换: raw/100"""
        qmc = QMC5883L()
        qmc.init()
        qmc.set_raw_values(0, 0, 0, temp_raw=2500)
        ok, data = qmc.read_data()
        self.assertTrue(ok)
        self.assertAlmostEqual(data.temperature, 25.0, places=1)


class TestQMC5883LV3SoftReset(unittest.TestCase):
    """QMC5883L V3 软复位测试"""

    def test_soft_reset_success(self):
        """软复位清除数据"""
        qmc = QMC5883L()
        qmc.init()
        qmc.set_raw_values(5000, 3000, 8000, 2500)
        ok = qmc.soft_reset()
        self.assertTrue(ok)
        self.assertEqual(qmc._raw_x, 0)
        self.assertEqual(qmc._raw_y, 0)
        self.assertEqual(qmc._raw_z, 0)
        self.assertEqual(qmc._raw_temp, 0)

    def test_soft_reset_not_init(self):
        """未初始化软复位应失败"""
        qmc = QMC5883L()
        ok = qmc.soft_reset()
        self.assertFalse(ok)


class TestQMC5883LV3Standby(unittest.TestCase):
    """QMC5883L V3 待机模式测试"""

    def test_standby_success(self):
        """待机模式"""
        qmc = QMC5883L()
        qmc.init()
        self.assertTrue(qmc.set_standby())

    def test_standby_not_init(self):
        """未初始化待机应失败"""
        qmc = QMC5883L()
        self.assertFalse(qmc.set_standby())


class TestQMC5883LV3DataStructure(unittest.TestCase):
    """QMC5883L V3 数据结构测试"""

    def test_data_defaults(self):
        """默认数据全零"""
        data = QMC5883L_Data()
        self.assertEqual(data.x, 0)
        self.assertEqual(data.x_gauss, 0.0)
        self.assertEqual(data.heading_deg, 0.0)
        self.assertEqual(data.temperature, 0.0)


class TestQMC5883LV3FullWorkflow(unittest.TestCase):
    """QMC5883L V3 完整工作流程"""

    def test_init_calibrate_heading_workflow(self):
        """初始化→设置数据→校准→航向计算"""
        qmc = QMC5883L()
        qmc.init()
        # 模拟磁场数据（含偏移）
        qmc.set_raw_values(8500, 500, 12000)
        ok, data = qmc.read_data()
        self.assertTrue(ok)
        # 手动校准
        offset_x, offset_y = 500, 500
        cal_x = data.x - offset_x
        cal_y = data.y - offset_y
        heading = calc_heading_from_calibrated(cal_x, cal_y)
        self.assertGreaterEqual(heading, 0)
        self.assertLess(heading, 360)


if __name__ == '__main__':
    unittest.main()
