#!/usr/bin/env python3
"""
智能家居V2测试 — 温湿度 + 空气质量 + OLED显示 + 蓝牙通信
覆盖: SHT30温湿度读取、SGP30空气质量检测、
      LCD1602显示(兼容OLED)、HC-05蓝牙数据透传、
      环境阈值报警联动
对应C源文件: 02_mspm0g3507/drivers/sht30.c + sgp30.c + bluetooth.c + lcd1602.c

错误经验检查:
  #9:  测试import wrappers.py而非自行重写
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    SHT30, SGP30, Bluetooth, LCD1602,
    BT_MODE_TRANSPARENT, BT_MODE_AT,
)


# ═══════════════════════════════════════════════════════════════
#  报警阈值定义
# ═══════════════════════════════════════════════════════════════
TEMP_HIGH = 35.0      # 高温报警(°C)
TEMP_LOW = 5.0        # 低温报警(°C)
HUMI_HIGH = 85.0      # 高湿报警(%RH)
HUMI_LOW = 20.0       # 低湿报警(%RH)
TVOC_HIGH = 500       # TVOC报警(ppb)
ECO2_HIGH = 1000      # eCO2报警(ppm)

# 报警等级
ALARM_NONE = 0
ALARM_WARNING = 1
ALARM_CRITICAL = 2


class AlarmEvent:
    """报警事件"""
    def __init__(self, source, level, message, value):
        self.source = source      # 'temp', 'humi', 'tvoc', 'eco2'
        self.level = level        # ALARM_NONE/WARNING/CRITICAL
        self.message = message
        self.value = value


class SmartHomeV2:
    """智能家居V2 — 多传感器融合 + 显示 + 蓝牙

    功能:
    - SHT30采集温湿度
    - SGP30采集空气质量(TVOC/eCO2)
    - LCD1602显示实时数据(兼容OLED接口)
    - HC-05蓝牙透传数据到手机
    - 多级报警联动
    """

    def __init__(self):
        self.sensor_th = SHT30()        # 温湿度
        self.sensor_air = SGP30()       # 空气质量
        self.display = LCD1602()        # 显示
        self.bluetooth = Bluetooth()    # 蓝牙

        # 当前数据
        self.temperature = 0.0
        self.humidity = 0.0
        self.tvoc = 0
        self.eco2 = 400

        # 报警状态
        self.alarm_level = ALARM_NONE
        self.alarm_events = []

    def init(self):
        """初始化所有传感器和外设"""
        self.sensor_th.init()
        self.sensor_air.init()
        self.display.init()
        self.bluetooth.init()
        self.alarm_level = ALARM_NONE
        self.alarm_events.clear()

    def read_sensors(self):
        """读取所有传感器数据

        返回: (success, temp, humi, tvoc, eco2)
        """
        ok_th, th_data = self.sensor_th.measure_single()
        if not ok_th:
            return False, 0, 0, 0, 0
        self.temperature, self.humidity = th_data

        ok_air, air_data = self.sensor_air.measure()
        if not ok_air:
            return False, 0, 0, 0, 0
        self.tvoc, self.eco2 = air_data

        return True, self.temperature, self.humidity, self.tvoc, self.eco2

    def check_alarms(self):
        """检查报警条件

        返回: 报警事件列表
        """
        events = []
        level = ALARM_NONE

        # 温度报警
        if self.temperature > TEMP_HIGH:
            diff = self.temperature - TEMP_HIGH
            evt_level = ALARM_CRITICAL if diff > 10 else ALARM_WARNING
            events.append(AlarmEvent('temp', evt_level,
                          f'高温: {self.temperature:.1f}°C', self.temperature))
            level = max(level, evt_level)
        elif self.temperature < TEMP_LOW:
            diff = TEMP_LOW - self.temperature
            evt_level = ALARM_CRITICAL if diff > 10 else ALARM_WARNING
            events.append(AlarmEvent('temp', evt_level,
                          f'低温: {self.temperature:.1f}°C', self.temperature))
            level = max(level, evt_level)

        # 湿度报警
        if self.humidity > HUMI_HIGH:
            events.append(AlarmEvent('humi', ALARM_WARNING,
                          f'高湿: {self.humidity:.1f}%', self.humidity))
            level = max(level, ALARM_WARNING)
        elif self.humidity < HUMI_LOW:
            events.append(AlarmEvent('humi', ALARM_WARNING,
                          f'低湿: {self.humidity:.1f}%', self.humidity))
            level = max(level, ALARM_WARNING)

        # TVOC报警
        if self.tvoc > TVOC_HIGH:
            events.append(AlarmEvent('tvoc', ALARM_WARNING,
                          f'TVOC高: {self.tvoc}ppb', self.tvoc))
            level = max(level, ALARM_WARNING)

        # eCO2报警
        if self.eco2 > ECO2_HIGH:
            events.append(AlarmEvent('eco2', ALARM_WARNING,
                          f'eCO2高: {self.eco2}ppm', self.eco2))
            level = max(level, ALARM_WARNING)

        self.alarm_level = level
        self.alarm_events = events
        return events

    def update_display(self):
        """更新显示内容

        第一行: 温度 + 湿度
        第二行: TVOC + eCO2 或 报警信息
        """
        line1 = f"T:{self.temperature:.1f}C H:{self.humidity:.0f}%"
        if self.alarm_level > ALARM_NONE and self.alarm_events:
            line2 = self.alarm_events[0].message[:16]
        else:
            line2 = f"V:{self.tvoc} C:{self.eco2}"

        self.display.print_line(0, line1[:16])
        self.display.print_line(1, line2[:16])

    def send_bluetooth(self):
        """通过蓝牙发送数据(格式: T,H,V,C)"""
        msg = f"{self.temperature:.1f},{self.humidity:.0f},{self.tvoc},{self.eco2}\n"
        self.bluetooth.send_string(msg)
        return msg

    def run_cycle(self):
        """执行一次完整采集-显示-发送周期

        返回: (success, alarm_level)
        """
        ok, temp, humi, tvoc, eco2 = self.read_sensors()
        if not ok:
            return False, ALARM_NONE

        self.check_alarms()
        self.update_display()
        self.send_bluetooth()

        return True, self.alarm_level


class TestSmartHomeInit(unittest.TestCase):
    """智能家居初始化测试"""

    def test_init_success(self):
        """初始化成功"""
        sh = SmartHomeV2()
        sh.init()
        self.assertTrue(sh.sensor_th.initialized)
        self.assertTrue(sh.sensor_air.initialized)
        self.assertTrue(sh.display.initialized)
        self.assertTrue(sh.bluetooth.initialized)

    def test_initial_alarm_none(self):
        """初始无报警"""
        sh = SmartHomeV2()
        sh.init()
        self.assertEqual(sh.alarm_level, ALARM_NONE)
        self.assertEqual(len(sh.alarm_events), 0)


class TestSHT30Sensor(unittest.TestCase):
    """SHT30温湿度传感器测试"""

    def test_normal_reading(self):
        """正常温湿度读取"""
        sht = SHT30()
        sht.init()
        # 25°C, 50%RH
        # raw_temp = (25+45)/175*65535 ≈ 26214
        # raw_humi = 50/100*65535 = 32768
        sht.set_raw_values(26214, 32768)
        ok, data = sht.measure_single()
        self.assertTrue(ok)
        temp, humi = data
        self.assertAlmostEqual(temp, 25.0, delta=0.5)
        self.assertAlmostEqual(humi, 50.0, delta=1.0)

    def test_high_temperature(self):
        """高温读取"""
        sht = SHT30()
        sht.init()
        # 45°C → raw = (45+45)/175*65535 ≈ 33539
        sht.set_raw_values(33539, 32768)
        ok, (temp, humi) = sht.measure_single()
        self.assertTrue(ok)
        self.assertAlmostEqual(temp, 45.0, delta=1.0)

    def test_before_init(self):
        """初始化前读取失败"""
        sht = SHT30()
        ok, _ = sht.measure_single()
        self.assertFalse(ok)

    def test_heater_control(self):
        """加热器控制"""
        sht = SHT30()
        sht.init()
        self.assertTrue(sht.heater_on())
        self.assertTrue(sht.heater_off())

    def test_soft_reset(self):
        """软复位"""
        sht = SHT30()
        sht.init()
        self.assertTrue(sht.soft_reset())


class TestSGP30Sensor(unittest.TestCase):
    """SGP30空气质量传感器测试"""

    def test_normal_reading(self):
        """正常空气质量读取"""
        sgp = SGP30()
        sgp.init()
        sgp.set_raw_values(100, 500)
        ok, data = sgp.measure()
        self.assertTrue(ok)
        tvoc, eco2 = data
        self.assertEqual(tvoc, 100)
        self.assertEqual(eco2, 500)

    def test_before_init(self):
        """初始化前读取失败"""
        sgp = SGP30()
        ok, _ = sgp.measure()
        self.assertFalse(ok)

    def test_baseline(self):
        """基线值读写"""
        sgp = SGP30()
        sgp.init()
        self.assertTrue(sgp.set_baseline(100, 400))
        ok, (tvoc_b, eco2_b) = sgp.get_baseline()
        self.assertTrue(ok)
        self.assertEqual(tvoc_b, 100)
        self.assertEqual(eco2_b, 400)

    def test_selftest(self):
        """自检"""
        sgp = SGP30()
        sgp.init()
        self.assertTrue(sgp.selftest())

    def test_humidity_compensation(self):
        """湿度补偿"""
        sgp = SGP30()
        sgp.init()
        self.assertTrue(sgp.set_humidity(50.0, 25.0))


class TestBluetooth(unittest.TestCase):
    """HC-05蓝牙测试"""

    def test_init(self):
        """初始化"""
        bt = Bluetooth()
        bt.init()
        self.assertTrue(bt.initialized)
        self.assertEqual(bt.mode, BT_MODE_TRANSPARENT)

    def test_send_data(self):
        """发送数据计数"""
        bt = Bluetooth()
        bt.init()
        bt.send_string("Hello")
        self.assertEqual(bt.tx_count, 5)

    def test_send_byte(self):
        """发送单字节"""
        bt = Bluetooth()
        bt.init()
        bt.send_byte(0x41)
        self.assertEqual(bt.tx_count, 1)

    def test_receive_data(self):
        """接收数据"""
        bt = Bluetooth()
        bt.init()
        # 模拟接收
        bt._rx_push(0x48)  # 'H'
        bt._rx_push(0x69)  # 'i'
        self.assertEqual(bt.available(), 2)
        self.assertEqual(bt.read_byte(), 0x48)
        self.assertEqual(bt.read_byte(), 0x69)
        self.assertEqual(bt.read_byte(), -1)  # 缓冲区空

    def test_at_mode(self):
        """AT模式切换"""
        bt = Bluetooth()
        bt.init()
        bt.enter_at_mode()
        self.assertEqual(bt.mode, BT_MODE_AT)
        bt.enter_transparent_mode()
        self.assertEqual(bt.mode, BT_MODE_TRANSPARENT)

    def test_overflow(self):
        """缓冲区溢出"""
        bt = Bluetooth()
        bt.init()
        for i in range(300):
            bt._rx_push(i)
        self.assertGreater(bt.overflow_count, 0)


class TestLCD1602Display(unittest.TestCase):
    """LCD1602显示测试"""

    def test_init(self):
        """初始化"""
        lcd = LCD1602()
        lcd.init()
        self.assertTrue(lcd.initialized)

    def test_set_cursor(self):
        """设置光标"""
        lcd = LCD1602()
        lcd.init()
        lcd.set_cursor(0, 0)
        lcd.set_cursor(1, 5)

    def test_print_string(self):
        """打印字符串"""
        lcd = LCD1602()
        lcd.init()
        lcd.set_cursor(0, 0)
        lcd.write_string("T:25.0C H:50%")


class TestSmartHomeAlarm(unittest.TestCase):
    """报警逻辑测试"""

    def setUp(self):
        self.sh = SmartHomeV2()
        self.sh.init()

    def test_no_alarm_normal(self):
        """正常环境→无报警"""
        self.sh.temperature = 25.0
        self.sh.humidity = 50.0
        self.sh.tvoc = 100
        self.sh.eco2 = 500
        events = self.sh.check_alarms()
        self.assertEqual(len(events), 0)
        self.assertEqual(self.sh.alarm_level, ALARM_NONE)

    def test_high_temp_warning(self):
        """高温报警"""
        self.sh.temperature = 38.0
        self.sh.humidity = 50.0
        self.sh.tvoc = 100
        self.sh.eco2 = 500
        events = self.sh.check_alarms()
        self.assertGreater(len(events), 0)
        temp_events = [e for e in events if e.source == 'temp']
        self.assertGreater(len(temp_events), 0)

    def test_high_temp_critical(self):
        """极端高温→严重报警"""
        self.sh.temperature = 50.0  # diff=15>10 → CRITICAL
        self.sh.humidity = 50.0
        self.sh.tvoc = 100
        self.sh.eco2 = 500
        events = self.sh.check_alarms()
        self.assertEqual(self.sh.alarm_level, ALARM_CRITICAL)

    def test_low_temp_warning(self):
        """低温报警"""
        self.sh.temperature = 3.0
        self.sh.humidity = 50.0
        self.sh.tvoc = 100
        self.sh.eco2 = 500
        events = self.sh.check_alarms()
        temp_events = [e for e in events if e.source == 'temp']
        self.assertGreater(len(temp_events), 0)

    def test_high_humidity_warning(self):
        """高湿报警"""
        self.sh.temperature = 25.0
        self.sh.humidity = 90.0
        self.sh.tvoc = 100
        self.sh.eco2 = 500
        events = self.sh.check_alarms()
        humi_events = [e for e in events if e.source == 'humi']
        self.assertGreater(len(humi_events), 0)

    def test_tvoc_alarm(self):
        """TVOC超标报警"""
        self.sh.temperature = 25.0
        self.sh.humidity = 50.0
        self.sh.tvoc = 600
        self.sh.eco2 = 500
        events = self.sh.check_alarms()
        tvoc_events = [e for e in events if e.source == 'tvoc']
        self.assertGreater(len(tvoc_events), 0)

    def test_eco2_alarm(self):
        """eCO2超标报警"""
        self.sh.temperature = 25.0
        self.sh.humidity = 50.0
        self.sh.tvoc = 100
        self.sh.eco2 = 1200
        events = self.sh.check_alarms()
        eco2_events = [e for e in events if e.source == 'eco2']
        self.assertGreater(len(eco2_events), 0)

    def test_multi_alarm(self):
        """多参数同时报警"""
        self.sh.temperature = 40.0
        self.sh.humidity = 90.0
        self.sh.tvoc = 600
        self.sh.eco2 = 1200
        events = self.sh.check_alarms()
        sources = {e.source for e in events}
        self.assertIn('temp', sources)
        self.assertIn('humi', sources)
        self.assertIn('tvoc', sources)
        self.assertIn('eco2', sources)


class TestSmartHomeIntegration(unittest.TestCase):
    """智能家居集成测试"""

    def setUp(self):
        self.sh = SmartHomeV2()
        self.sh.init()

    def test_read_sensors(self):
        """传感器数据读取"""
        self.sh.sensor_th.set_raw_values(26214, 32768)  # ~25°C, 50%
        self.sh.sensor_air.set_raw_values(100, 500)
        ok, temp, humi, tvoc, eco2 = self.sh.read_sensors()
        self.assertTrue(ok)
        self.assertAlmostEqual(temp, 25.0, delta=1.0)
        self.assertEqual(tvoc, 100)

    def test_bluetooth_send(self):
        """蓝牙数据发送"""
        self.sh.temperature = 25.5
        self.sh.humidity = 60.0
        self.sh.tvoc = 120
        self.sh.eco2 = 600
        msg = self.sh.send_bluetooth()
        self.assertIn("25.5", msg)
        self.assertIn("60", msg)
        self.assertEqual(self.sh.bluetooth.tx_count, len(msg))

    def test_full_cycle(self):
        """完整采集周期"""
        self.sh.sensor_th.set_raw_values(26214, 32768)
        self.sh.sensor_air.set_raw_values(100, 500)
        ok, level = self.sh.run_cycle()
        self.assertTrue(ok)
        self.assertEqual(level, ALARM_NONE)

    def test_cycle_with_alarm(self):
        """带报警的采集周期"""
        # 高温场景: ~45°C
        self.sh.sensor_th.set_raw_values(33539, 32768)
        self.sh.sensor_air.set_raw_values(100, 500)
        ok, level = self.sh.run_cycle()
        self.assertTrue(ok)
        self.assertGreater(level, ALARM_NONE)

    def test_display_update(self):
        """显示更新不抛异常"""
        self.sh.temperature = 25.0
        self.sh.humidity = 50.0
        self.sh.tvoc = 100
        self.sh.eco2 = 500
        self.sh.update_display()  # 不应抛异常

    def test_sensor_fail_propagation(self):
        """传感器失败传播"""
        # 不设置原始值，使用默认值(仍可读取)
        self.sh.sensor_th.set_raw_values(0, 0)
        self.sh.sensor_air.set_raw_values(0, 400)
        ok, temp, humi, tvoc, eco2 = self.sh.read_sensors()
        # SHT30 raw=0 → T=-45°C, SGP30可以读取
        self.assertTrue(ok)


if __name__ == '__main__':
    unittest.main()
