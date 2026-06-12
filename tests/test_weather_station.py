#!/usr/bin/env python3
"""
气象站V2测试 — SHT30温湿度 + BMP280气压 + SGP30空气质量
覆盖: 多传感器融合、温度/湿度/气压/海拔/空气质量采集、
      气象数据统计、舒适度指数计算、极端天气报警、
      数据格式化输出
对应C源文件: 02_mspm0g3507/drivers/sht30.c + bmp280.c + sgp30.c

错误经验检查:
  #9:  测试import wrappers.py而非自行重写
"""

import sys
import os
import math
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    SHT30, BMP280, SGP30,
    BMP280_MODE_SLEEP, BMP280_MODE_NORMAL, BMP280_MODE_FORCED,
    BMP280_OS_1X, BMP280_OS_16X,
    BMP280_FILTER_OFF, BMP280_FILTER_16,
)


# ═══════════════════════════════════════════════════════════════
#  舒适度等级定义
# ═══════════════════════════════════════════════════════════════

COMFORT_LEVEL_COLD = 0       # 寒冷
COMFORT_LEVEL_COOL = 1       # 凉爽
COMFORT_LEVEL_COMFORTABLE = 2  # 舒适
COMFORT_LEVEL_WARM = 3       # 温暖
COMFORT_LEVEL_HOT = 4        # 炎热
COMFORT_LEVEL_MISERABLE = 5  # 闷热难耐

# 天气趋势
TREND_STABLE = 0    # 稳定
TREND_RISING = 1    # 上升
TREND_FALLING = 2   # 下降

# 海平面标准气压
SEA_LEVEL_PRESSURE = 101325.0  # Pa


class WeatherReading:
    """单次气象读数"""

    def __init__(self):
        self.temperature = 0.0    # °C
        self.humidity = 0.0       # %RH
        self.pressure = 0.0       # Pa
        self.altitude = 0.0       # m
        self.tvoc = 0             # ppb
        self.eco2 = 400           # ppm
        self.timestamp = 0        # 采集时间戳(模拟)


class WeatherStationV2:
    """气象站V2 — 多传感器融合

    功能:
    - SHT30采集温湿度
    - BMP280采集气压和海拔
    - SGP30采集空气质量(TVOC/eCO2)
    - 舒适度指数计算
    - 气压趋势分析
    - 数据历史记录
    - 极端天气报警
    """

    def __init__(self):
        self.sensor_th = SHT30()        # 温湿度
        self.sensor_bp = BMP280()       # 气压
        self.sensor_air = SGP30()       # 空气质量

        # 当前数据
        self.temperature = 0.0
        self.humidity = 0.0
        self.pressure = 0.0
        self.altitude = 0.0
        self.tvoc = 0
        self.eco2 = 400

        # 历史数据（最近N次读数）
        self.history = []
        self.max_history = 100

        # 气压趋势
        self.pressure_trend = TREND_STABLE
        self._prev_pressure = 0.0

    def init(self):
        """初始化所有传感器"""
        self.sensor_th.init()
        self.sensor_bp.init()
        self.sensor_bp.set_mode(BMP280_MODE_NORMAL)
        self.sensor_bp.set_oversampling(BMP280_OS_16X, BMP280_OS_16X)
        self.sensor_bp.set_filter(BMP280_FILTER_16)
        self.sensor_air.init()
        self.history.clear()

    def read_all(self):
        """读取所有传感器

        返回: (success, WeatherReading)
        """
        # 温湿度
        ok_th, th_data = self.sensor_th.measure_single()
        if not ok_th:
            return False, None
        self.temperature, self.humidity = th_data

        # 气压
        temp_bp = self.sensor_bp.read_temperature()
        press_bp = self.sensor_bp.read_pressure()
        alt_bp = self.sensor_bp.read_altitude()
        if temp_bp is None or press_bp is None:
            return False, None
        self.pressure = press_bp
        self.altitude = alt_bp if alt_bp is not None else 0.0

        # 空气质量
        ok_air, air_data = self.sensor_air.measure()
        if not ok_air:
            return False, None
        self.tvoc, self.eco2 = air_data

        # 构建读数
        reading = WeatherReading()
        reading.temperature = self.temperature
        reading.humidity = self.humidity
        reading.pressure = self.pressure
        reading.altitude = self.altitude
        reading.tvoc = self.tvoc
        reading.eco2 = self.eco2

        # 更新历史
        self._update_history(reading)
        # 更新气压趋势
        self._update_pressure_trend()

        return True, reading

    def _update_history(self, reading):
        """更新历史记录"""
        self.history.append(reading)
        if len(self.history) > self.max_history:
            self.history.pop(0)

    def _update_pressure_trend(self):
        """更新气压趋势"""
        if len(self.history) < 2:
            self.pressure_trend = TREND_STABLE
            return

        recent = self.history[-5:] if len(self.history) >= 5 else self.history
        if len(recent) < 2:
            self.pressure_trend = TREND_STABLE
            return

        pressures = [r.pressure for r in recent]
        avg_first = sum(pressures[:len(pressures) // 2]) / (len(pressures) // 2)
        avg_last = sum(pressures[len(pressures) // 2:]) / (len(pressures) - len(pressures) // 2)

        diff = avg_last - avg_first
        if diff > 50:     # 气压上升超过50Pa
            self.pressure_trend = TREND_RISING
        elif diff < -50:  # 气压下降超过50Pa
            self.pressure_trend = TREND_FALLING
        else:
            self.pressure_trend = TREND_STABLE

    def get_comfort_level(self):
        """计算舒适度等级

        基于温湿度组合:
        - 寒冷: T < 5°C
        - 凉爽: 5°C ≤ T < 18°C
        - 舒适: 18°C ≤ T < 26°C 且 30% ≤ H ≤ 70%
        - 温暖: 26°C ≤ T < 32°C
        - 炎热: T ≥ 32°C
        - 闷热: T ≥ 28°C 且 H ≥ 80%
        """
        t = self.temperature
        h = self.humidity

        # 闷热优先判断
        if t >= 28 and h >= 80:
            return COMFORT_LEVEL_MISERABLE

        if t < 5:
            return COMFORT_LEVEL_COLD
        elif t < 18:
            return COMFORT_LEVEL_COOL
        elif t < 26:
            if 30 <= h <= 70:
                return COMFORT_LEVEL_COMFORTABLE
            elif h > 70:
                return COMFORT_LEVEL_WARM
            else:
                return COMFORT_LEVEL_COOL
        elif t < 32:
            return COMFORT_LEVEL_WARM
        else:
            return COMFORT_LEVEL_HOT

    def get_dew_point(self):
        """计算露点温度(°C)

        Magnus公式
        """
        t = self.temperature
        h = self.humidity
        if h <= 0:
            return t
        a = 17.27
        b = 237.7
        gamma = (a * t) / (b + t) + math.log(h / 100.0)
        return (b * gamma) / (a - gamma)

    def get_heat_index(self):
        """计算体感温度(°C)

        简化热指数公式
        """
        t = self.temperature
        h = self.humidity
        if t < 27:
            return t
        hi = (-8.7847 + 1.6114 * t + 2.3385 * h
              - 0.1461 * t * h - 0.0123 * t * t
              - 0.0164 * h * h + 0.0022 * t * t * h
              + 0.0007 * t * h * h - 0.0000036 * t * t * h * h)
        return hi

    def is_extreme_weather(self):
        """判断是否为极端天气

        返回: (is_extreme, reason)
        """
        reasons = []

        if self.temperature > 40:
            reasons.append(f"极端高温: {self.temperature:.1f}°C")
        elif self.temperature < -10:
            reasons.append(f"极端低温: {self.temperature:.1f}°C")

        if self.humidity > 95:
            reasons.append(f"极端高湿: {self.humidity:.0f}%")
        elif self.humidity < 10:
            reasons.append(f"极端低湿: {self.humidity:.0f}%")

        if self.pressure > 105000:
            reasons.append(f"极高气压: {self.pressure/100:.0f}hPa")
        elif self.pressure < 98000:
            reasons.append(f"极低气压: {self.pressure/100:.0f}hPa")

        is_extreme = len(reasons) > 0
        return is_extreme, "; ".join(reasons)

    def format_display(self):
        """格式化显示数据（两行LCD格式）

        返回: (line1, line2)
        """
        line1 = f"T:{self.temperature:.1f}C H:{self.humidity:.0f}%"
        line2 = f"P:{self.pressure/100:.0f}hPa A:{self.altitude:.0f}m"
        return line1[:16], line2[:16]

    def format_bluetooth(self):
        """格式化蓝牙数据

        返回: CSV格式字符串
        """
        return (f"{self.temperature:.1f},{self.humidity:.0f},"
                f"{self.pressure/100:.1f},{self.altitude:.1f},"
                f"{self.tvoc},{self.eco2}\n")


class TestSHT30Sensor(unittest.TestCase):
    """SHT30温湿度传感器测试"""

    def test_init(self):
        """初始化"""
        ws = WeatherStationV2()
        ws.sensor_th.init()
        self.assertTrue(ws.sensor_th.initialized)

    def test_normal_reading(self):
        """正常读取"""
        ws = WeatherStationV2()
        ws.sensor_th.init()
        # 25°C, 50%RH
        ws.sensor_th.set_raw_values(26214, 32768)
        ok, (temp, humi) = ws.sensor_th.measure_single()
        self.assertTrue(ok)
        self.assertAlmostEqual(temp, 25.0, delta=0.5)
        self.assertAlmostEqual(humi, 50.0, delta=1.0)

    def test_before_init(self):
        """初始化前读取失败"""
        ws = WeatherStationV2()
        ok, _ = ws.sensor_th.measure_single()
        self.assertFalse(ok)


class TestBMP280Sensor(unittest.TestCase):
    """BMP280气压传感器测试"""

    def test_init(self):
        """初始化"""
        bp = BMP280()
        bp.init()
        self.assertTrue(bp.initialized)

    def test_mode_setting(self):
        """模式设置"""
        bp = BMP280()
        bp.init()
        self.assertTrue(bp.set_mode(BMP280_MODE_NORMAL))
        self.assertEqual(bp.mode, BMP280_MODE_NORMAL)
        self.assertTrue(bp.set_mode(BMP280_MODE_SLEEP))
        self.assertFalse(bp.set_mode(99))  # 无效模式

    def test_read_temperature(self):
        """读取温度"""
        bp = BMP280()
        bp.init()
        bp.set_simulated_raw(519888, 415148)  # 约25°C的标准值
        temp = bp.read_temperature()
        self.assertIsNotNone(temp)
        # 温度应在合理范围内
        self.assertGreater(temp, -40)
        self.assertLess(temp, 85)

    def test_read_pressure(self):
        """读取气压"""
        bp = BMP280()
        bp.init()
        bp.set_simulated_raw(519888, 415148)
        press = bp.read_pressure()
        self.assertIsNotNone(press)
        # 气压应在合理范围内 (300hPa ~ 1100hPa)
        self.assertGreater(press, 30000)
        self.assertLess(press, 110000)

    def test_read_altitude(self):
        """计算海拔"""
        bp = BMP280()
        bp.init()
        bp.set_simulated_raw(519888, 415148)
        alt = bp.read_altitude()
        self.assertIsNotNone(alt)
        # 海拔应在合理范围内
        self.assertGreater(alt, -500)
        self.assertLess(alt, 9000)

    def test_before_init(self):
        """初始化前读取返回None"""
        bp = BMP280()
        self.assertIsNone(bp.read_temperature())
        self.assertIsNone(bp.read_pressure())
        self.assertIsNone(bp.read_altitude())

    def test_oversampling(self):
        """过采样设置"""
        bp = BMP280()
        bp.init()
        self.assertTrue(bp.set_oversampling(BMP280_OS_16X, BMP280_OS_16X))

    def test_filter(self):
        """IIR滤波设置"""
        bp = BMP280()
        bp.init()
        self.assertTrue(bp.set_filter(BMP280_FILTER_16))
        self.assertFalse(bp.set_filter(99))


class TestSGP30Sensor(unittest.TestCase):
    """SGP30空气质量传感器测试"""

    def test_init(self):
        """初始化"""
        sgp = SGP30()
        sgp.init()
        self.assertTrue(sgp.initialized)

    def test_normal_reading(self):
        """正常读取"""
        sgp = SGP30()
        sgp.init()
        sgp.set_raw_values(150, 600)
        ok, (tvoc, eco2) = sgp.measure()
        self.assertTrue(ok)
        self.assertEqual(tvoc, 150)
        self.assertEqual(eco2, 600)

    def test_before_init(self):
        """初始化前读取失败"""
        sgp = SGP30()
        ok, _ = sgp.measure()
        self.assertFalse(ok)


class TestWeatherStationInit(unittest.TestCase):
    """气象站初始化测试"""

    def test_init_success(self):
        """初始化成功"""
        ws = WeatherStationV2()
        ws.init()
        self.assertTrue(ws.sensor_th.initialized)
        self.assertTrue(ws.sensor_bp.initialized)
        self.assertTrue(ws.sensor_air.initialized)

    def test_initial_values(self):
        """初始值"""
        ws = WeatherStationV2()
        ws.init()
        self.assertEqual(ws.temperature, 0.0)
        self.assertEqual(ws.humidity, 0.0)
        self.assertEqual(len(ws.history), 0)

    def test_bmp280_normal_mode(self):
        """BMP280初始化后为正常模式"""
        ws = WeatherStationV2()
        ws.init()
        self.assertEqual(ws.sensor_bp.mode, BMP280_MODE_NORMAL)


class TestWeatherReading(unittest.TestCase):
    """气象数据读取测试"""

    def setUp(self):
        self.ws = WeatherStationV2()
        self.ws.init()
        # 设置模拟值: 25°C, 50%RH
        self.ws.sensor_th.set_raw_values(26214, 32768)
        # BMP280: ~25°C, ~101325Pa
        self.ws.sensor_bp.set_simulated_raw(519888, 415148)
        # SGP30: TVOC=100, eCO2=500
        self.ws.sensor_air.set_raw_values(100, 500)

    def test_read_all_success(self):
        """完整读取成功"""
        ok, reading = self.ws.read_all()
        self.assertTrue(ok)
        self.assertIsNotNone(reading)

    def test_temperature_range(self):
        """温度范围合理"""
        ok, reading = self.ws.read_all()
        self.assertTrue(ok)
        self.assertGreater(reading.temperature, -40)
        self.assertLess(reading.temperature, 85)

    def test_humidity_range(self):
        """湿度范围合理"""
        ok, reading = self.ws.read_all()
        self.assertTrue(ok)
        self.assertGreaterEqual(reading.humidity, 0)
        self.assertLessEqual(reading.humidity, 100)

    def test_pressure_range(self):
        """气压范围合理"""
        ok, reading = self.ws.read_all()
        self.assertTrue(ok)
        self.assertGreater(reading.pressure, 30000)
        self.assertLess(reading.pressure, 110000)

    def test_air_quality(self):
        """空气质量数据"""
        ok, reading = self.ws.read_all()
        self.assertTrue(ok)
        self.assertEqual(reading.tvoc, 100)
        self.assertEqual(reading.eco2, 500)

    def test_read_failure_sht30(self):
        """SHT30读取失败"""
        ws = WeatherStationV2()
        ws.init()
        # 不设置SHT30模拟值，初始化前读取
        ws.sensor_th = SHT30()  # 未初始化
        ok, reading = ws.read_all()
        self.assertFalse(ok)


class TestComfortLevel(unittest.TestCase):
    """舒适度测试"""

    def setUp(self):
        self.ws = WeatherStationV2()
        self.ws.init()

    def test_comfortable(self):
        """舒适: 22°C, 50%RH"""
        self.ws.temperature = 22.0
        self.ws.humidity = 50.0
        self.assertEqual(self.ws.get_comfort_level(), COMFORT_LEVEL_COMFORTABLE)

    def test_cold(self):
        """寒冷: 0°C"""
        self.ws.temperature = 0.0
        self.ws.humidity = 60.0
        self.assertEqual(self.ws.get_comfort_level(), COMFORT_LEVEL_COLD)

    def test_cool(self):
        """凉爽: 15°C"""
        self.ws.temperature = 15.0
        self.ws.humidity = 50.0
        self.assertEqual(self.ws.get_comfort_level(), COMFORT_LEVEL_COOL)

    def test_warm(self):
        """温暖: 28°C"""
        self.ws.temperature = 28.0
        self.ws.humidity = 50.0
        self.assertEqual(self.ws.get_comfort_level(), COMFORT_LEVEL_WARM)

    def test_hot(self):
        """炎热: 35°C"""
        self.ws.temperature = 35.0
        self.ws.humidity = 40.0
        self.assertEqual(self.ws.get_comfort_level(), COMFORT_LEVEL_HOT)

    def test_miserable(self):
        """闷热: 30°C, 85%RH"""
        self.ws.temperature = 30.0
        self.ws.humidity = 85.0
        self.assertEqual(self.ws.get_comfort_level(), COMFORT_LEVEL_MISERABLE)

    def test_comfortable_low_humidity(self):
        """舒适区低湿度→凉爽"""
        self.ws.temperature = 22.0
        self.ws.humidity = 20.0
        self.assertEqual(self.ws.get_comfort_level(), COMFORT_LEVEL_COOL)

    def test_comfortable_high_humidity(self):
        """舒适区高湿度→温暖"""
        self.ws.temperature = 22.0
        self.ws.humidity = 80.0
        self.assertEqual(self.ws.get_comfort_level(), COMFORT_LEVEL_WARM)


class TestDerivedMetrics(unittest.TestCase):
    """衍生指标测试"""

    def setUp(self):
        self.ws = WeatherStationV2()
        self.ws.init()

    def test_dew_point(self):
        """露点温度"""
        self.ws.temperature = 25.0
        self.ws.humidity = 50.0
        dp = self.ws.get_dew_point()
        # 露点应低于当前温度
        self.assertLess(dp, self.ws.temperature)
        # 25°C/50%RH的露点约13.9°C
        self.assertAlmostEqual(dp, 13.9, delta=1.0)

    def test_dew_point_high_humidity(self):
        """高湿度露点接近温度"""
        self.ws.temperature = 25.0
        self.ws.humidity = 95.0
        dp = self.ws.get_dew_point()
        self.assertAlmostEqual(dp, 25.0, delta=2.0)

    def test_dew_point_zero_humidity(self):
        """零湿度保护"""
        self.ws.temperature = 25.0
        self.ws.humidity = 0.0
        dp = self.ws.get_dew_point()
        self.assertEqual(dp, 25.0)  # 返回原温度

    def test_heat_index_low_temp(self):
        """低温时体感温度等于实际温度"""
        self.ws.temperature = 20.0
        self.ws.humidity = 50.0
        hi = self.ws.get_heat_index()
        self.assertEqual(hi, 20.0)

    def test_heat_index_high_temp(self):
        """高温时体感温度高于实际"""
        self.ws.temperature = 35.0
        self.ws.humidity = 70.0
        hi = self.ws.get_heat_index()
        self.assertGreater(hi, self.ws.temperature)


class TestExtremeWeather(unittest.TestCase):
    """极端天气测试"""

    def setUp(self):
        self.ws = WeatherStationV2()
        self.ws.init()

    def test_normal_not_extreme(self):
        """正常天气"""
        self.ws.temperature = 25.0
        self.ws.humidity = 50.0
        self.ws.pressure = 101325.0
        extreme, reason = self.ws.is_extreme_weather()
        self.assertFalse(extreme)
        self.assertEqual(reason, "")

    def test_extreme_heat(self):
        """极端高温"""
        self.ws.temperature = 45.0
        self.ws.humidity = 50.0
        self.ws.pressure = 101325.0
        extreme, reason = self.ws.is_extreme_weather()
        self.assertTrue(extreme)
        self.assertIn("极端高温", reason)

    def test_extreme_cold(self):
        """极端低温"""
        self.ws.temperature = -15.0
        self.ws.humidity = 50.0
        self.ws.pressure = 101325.0
        extreme, reason = self.ws.is_extreme_weather()
        self.assertTrue(extreme)
        self.assertIn("极端低温", reason)

    def test_extreme_high_humidity(self):
        """极端高湿"""
        self.ws.temperature = 25.0
        self.ws.humidity = 98.0
        self.ws.pressure = 101325.0
        extreme, reason = self.ws.is_extreme_weather()
        self.assertTrue(extreme)
        self.assertIn("极端高湿", reason)

    def test_extreme_low_pressure(self):
        """极端低气压（台风级别）"""
        self.ws.temperature = 25.0
        self.ws.humidity = 50.0
        self.ws.pressure = 95000.0
        extreme, reason = self.ws.is_extreme_weather()
        self.assertTrue(extreme)
        self.assertIn("极低气压", reason)


class TestHistoryAndTrend(unittest.TestCase):
    """历史记录和趋势测试"""

    def setUp(self):
        self.ws = WeatherStationV2()
        self.ws.init()
        self.ws.sensor_th.set_raw_values(26214, 32768)
        self.ws.sensor_bp.set_simulated_raw(519888, 415148)
        self.ws.sensor_air.set_raw_values(100, 500)

    def test_history_growth(self):
        """历史记录增长"""
        self.ws.read_all()
        self.ws.read_all()
        self.ws.read_all()
        self.assertEqual(len(self.ws.history), 3)

    def test_history_max_limit(self):
        """历史记录上限"""
        self.ws.max_history = 5
        for _ in range(10):
            self.ws.read_all()
        self.assertEqual(len(self.ws.history), 5)

    def test_pressure_trend_stable(self):
        """气压稳定"""
        for _ in range(5):
            self.ws.read_all()
        self.assertEqual(self.ws.pressure_trend, TREND_STABLE)

    def test_clear_history(self):
        """清空历史"""
        self.ws.read_all()
        self.ws.history.clear()
        self.assertEqual(len(self.ws.history), 0)


class TestDisplayFormat(unittest.TestCase):
    """显示格式测试"""

    def test_format_display(self):
        """LCD格式化"""
        ws = WeatherStationV2()
        ws.temperature = 25.5
        ws.humidity = 60.0
        ws.pressure = 101325.0
        ws.altitude = 100.0
        line1, line2 = ws.format_display()
        self.assertIn("25.5", line1)
        self.assertIn("60", line1)
        self.assertLessEqual(len(line1), 16)
        self.assertLessEqual(len(line2), 16)

    def test_format_bluetooth(self):
        """蓝牙数据格式"""
        ws = WeatherStationV2()
        ws.temperature = 25.0
        ws.humidity = 50.0
        ws.pressure = 101325.0
        ws.altitude = 50.0
        ws.tvoc = 100
        ws.eco2 = 500
        msg = ws.format_bluetooth()
        self.assertIn("25.0", msg)
        self.assertIn("100", msg)
        self.assertTrue(msg.endswith("\n"))
        # CSV格式应有5个逗号
        self.assertEqual(msg.count(","), 5)


if __name__ == '__main__':
    unittest.main()
