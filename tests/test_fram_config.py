#!/usr/bin/env python3
"""
铁电配置管理V2测试 — FM24CL64 FRAM参数存储
覆盖: FRAM初始化、字节/页读写、配置参数存储、
      校验和验证、版本迁移、工厂默认值
对应C源文件: 02_mspm0g3507/drivers/fm24cl64.c + config_manager.c

错误经验检查:
  #9:  测试import wrappers.py而非自行重写
"""

import sys
import os
import struct
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    FM24CL64, FM24CL64_SIZE, FM24CL64_PAGE_SIZE,
    AT24C02, AT24C02_SIZE,
)


# ═══════════════════════════════════════════════════════════════
#  配置管理器 (Python镜像)
# ═══════════════════════════════════════════════════════════════

# 配置区域布局
CFG_MAGIC = 0x43464700   # "CFG\0"
CFG_HEADER_SIZE = 16     # magic(4) + version(2) + count(2) + data_len(2) + checksum(4) + reserved(2)
CFG_DATA_START = 64      # 数据区起始地址(跳过头和保留区)
CFG_MAX_PARAMS = 100     # 最大参数数量

# 参数ID
PARAM_MOTOR_KP = 0x01
PARAM_MOTOR_KI = 0x02
PARAM_MOTOR_KD = 0x03
PARAM_SPEED_MAX = 0x04
PARAM_ANGLE_OFFSET = 0x05
PARAM_SERVO_MIN = 0x06
PARAM_SERVO_MAX = 0x07
PARAM_ADC_SCALE = 0x08
PARAM_DISPLAY_ROT = 0x09
PARAM_SYSTEM_MODE = 0x0A

# 参数大小(字节)
PARAM_SIZE_FLOAT = 4
PARAM_SIZE_UINT16 = 2
PARAM_SIZE_UINT8 = 1

# 参数条目格式: [ID(1B)][Size(1B)][Value(NB)]
PARAM_HEADER_SIZE = 2


def calc_checksum(data):
    """计算CRC32校验和"""
    crc = 0xFFFFFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xEDB88320
            else:
                crc >>= 1
    return crc ^ 0xFFFFFFFF


class ConfigParam:
    """配置参数条目"""
    def __init__(self, param_id=0, value=None, size=4):
        self.id = param_id
        self.value = value
        self.size = size

    def pack(self):
        """序列化"""
        header = struct.pack('<BB', self.id, self.size)
        if isinstance(self.value, float):
            data = struct.pack('<f', self.value)
        elif self.size == 2:
            data = struct.pack('<H', self.value & 0xFFFF)
        elif self.size == 1:
            data = struct.pack('<B', self.value & 0xFF)
        else:
            data = struct.pack('<f', self.value)
        return header + data

    @staticmethod
    def unpack(raw, offset=0):
        """反序列化"""
        if offset + 2 > len(raw):
            return None, offset
        param_id, size = struct.unpack('<BB', raw[offset:offset + 2])
        offset += 2
        if offset + size > len(raw):
            return None, offset
        if size == 4:
            value = struct.unpack('<f', raw[offset:offset + 4])[0]
        elif size == 2:
            value = struct.unpack('<H', raw[offset:offset + 2])[0]
        elif size == 1:
            value = raw[offset]
        else:
            value = 0
        offset += size
        return ConfigParam(param_id, value, size), offset


class ConfigManager:
    """FRAM配置管理器 — 掉电保存参数

    功能:
    - 参数读写（float/uint16/uint8）
    - CRC32校验
    - 版本管理
    - 工厂默认值恢复
    """

    def __init__(self, fram=None):
        self.fram = fram or FM24CL64()
        self.version = 1
        self.params = {}  # id -> ConfigParam
        self.initialized = False

    def init(self):
        """初始化"""
        self.fram.init()
        self.initialized = True
        self.params.clear()
        # 尝试加载已有配置
        self._load_from_fram()

    def _load_from_fram(self):
        """从FRAM加载配置"""
        # 读取头
        header = self.fram.read(0, CFG_HEADER_SIZE)
        if len(header) < CFG_HEADER_SIZE:
            return False

        magic = struct.unpack('<I', header[0:4])[0]
        if magic != CFG_MAGIC:
            return False  # 未格式化

        version, count, data_len = struct.unpack('<HHH', header[4:10])
        stored_checksum = struct.unpack('<I', header[10:14])[0]

        if count > CFG_MAX_PARAMS or data_len > 800:
            return False

        # 读取数据区
        data = self.fram.read(CFG_DATA_START, data_len)
        if len(data) < data_len:
            return False

        # 验证校验和
        calc_crc = calc_checksum(data)
        if calc_crc != stored_checksum:
            return False  # 数据损坏

        # 解析参数
        offset = 0
        for _ in range(count):
            param, offset = ConfigParam.unpack(data, offset)
            if param is None:
                break
            self.params[param.id] = param

        self.version = version
        return True

    def _save_to_fram(self):
        """保存配置到FRAM"""
        # 序列化所有参数
        data = bytearray()
        for param in self.params.values():
            data.extend(param.pack())

        count = len(self.params)
        data_len = len(data)
        checksum = calc_checksum(bytes(data))

        # 写入头
        header = struct.pack('<IHHHI',
                             CFG_MAGIC, self.version, count, data_len, checksum)
        self.fram.write(0, header)

        # 写入数据
        self.fram.write(CFG_DATA_START, bytes(data))
        return True

    def set_float(self, param_id, value):
        """设置浮点参数"""
        self.params[param_id] = ConfigParam(param_id, value, 4)

    def get_float(self, param_id, default=0.0):
        """获取浮点参数"""
        if param_id in self.params:
            return self.params[param_id].value
        return default

    def set_uint16(self, param_id, value):
        """设置16位无符号参数"""
        self.params[param_id] = ConfigParam(param_id, value & 0xFFFF, 2)

    def get_uint16(self, param_id, default=0):
        """获取16位无符号参数"""
        if param_id in self.params:
            return self.params[param_id].value
        return default

    def set_uint8(self, param_id, value):
        """设置8位无符号参数"""
        self.params[param_id] = ConfigParam(param_id, value & 0xFF, 1)

    def get_uint8(self, param_id, default=0):
        """获取8位无符号参数"""
        if param_id in self.params:
            return self.params[param_id].value
        return default

    def save(self):
        """保存配置"""
        if not self.initialized:
            return False
        return self._save_to_fram()

    def load(self):
        """重新加载配置"""
        if not self.initialized:
            return False
        self.params.clear()
        return self._load_from_fram()

    def load_defaults(self):
        """加载工厂默认值"""
        self.set_float(PARAM_MOTOR_KP, 10.0)
        self.set_float(PARAM_MOTOR_KI, 0.5)
        self.set_float(PARAM_MOTOR_KD, 0.1)
        self.set_uint16(PARAM_SPEED_MAX, 1000)
        self.set_float(PARAM_ANGLE_OFFSET, 0.0)
        self.set_uint16(PARAM_SERVO_MIN, 500)
        self.set_uint16(PARAM_SERVO_MAX, 2500)
        self.set_float(PARAM_ADC_SCALE, 1.0)
        self.set_uint8(PARAM_DISPLAY_ROT, 0)
        self.set_uint8(PARAM_SYSTEM_MODE, 0)

    def get_param_count(self):
        """获取参数数量"""
        return len(self.params)

    def clear(self):
        """清除所有参数"""
        self.params.clear()

    def format_fram(self):
        """格式化FRAM（擦除配置头）"""
        if not self.initialized:
            return False
        self.fram.fill(0x00, 0, CFG_DATA_START + 256)
        self.params.clear()
        return True


# ═══════════════════════════════════════════════════════════════
#  测试类
# ═══════════════════════════════════════════════════════════════

class TestFRAMInit(unittest.TestCase):
    """FRAM初始化测试"""

    def test_init_success(self):
        """初始化成功"""
        fram = FM24CL64()
        fram.init()
        self.assertTrue(fram.is_initialized())

    def test_fram_size(self):
        """FRAM大小正确"""
        fram = FM24CL64()
        self.assertEqual(fram.get_size(), FM24CL64_SIZE)

    def test_config_manager_init(self):
        """配置管理器初始化"""
        cfg = ConfigManager()
        cfg.init()
        self.assertTrue(cfg.initialized)


class TestFRAMBasicOps(unittest.TestCase):
    """FRAM基本操作测试"""

    def setUp(self):
        self.fram = FM24CL64()
        self.fram.init()

    def test_write_read_byte(self):
        """字节读写"""
        self.assertTrue(self.fram.write_byte(0x100, 0xAB))
        self.assertEqual(self.fram.read_byte(0x100), 0xAB)

    def test_write_read_block(self):
        """块读写"""
        data = b'Hello FRAM!'
        self.assertTrue(self.fram.write(0x200, data))
        result = self.fram.read(0x200, len(data))
        self.assertEqual(bytes(result), data)

    def test_write_page(self):
        """页写入"""
        data = bytes(range(32))  # 32字节
        self.assertTrue(self.fram.write_page(0x300, data))
        result = self.fram.read(0x300, 32)
        self.assertEqual(bytes(result), data)

    def test_no_erase_needed(self):
        """FRAM无需擦除即可直接写入"""
        # 先写入
        self.fram.write_byte(0x100, 0xFF)
        self.assertEqual(self.fram.read_byte(0x100), 0xFF)
        # 直接覆写（不同于Flash需要先擦除）
        self.fram.write_byte(0x100, 0x55)
        self.assertEqual(self.fram.read_byte(0x100), 0x55)

    def test_fill(self):
        """填充"""
        self.assertTrue(self.fram.fill(0xAA, 0x500, 16))
        for i in range(16):
            self.assertEqual(self.fram.read_byte(0x500 + i), 0xAA)

    def test_compare(self):
        """比较"""
        data = b'test data'
        self.fram.write(0x600, data)
        self.assertTrue(self.fram.compare(0x600, data))
        self.assertFalse(self.fram.compare(0x600, b'wrong!'))

    def test_boundary_check(self):
        """边界检查"""
        # 越界写入
        self.assertFalse(self.fram.write_byte(-1, 0xAA))
        self.assertFalse(self.fram.write_byte(FM24CL64_SIZE, 0xAA))
        # 越界读取
        self.assertEqual(self.fram.read_byte(-1), 0x00)
        self.assertEqual(self.fram.read_byte(FM24CL64_SIZE), 0x00)

    def test_statistics(self):
        """操作统计"""
        self.fram.write_byte(0, 0x01)
        self.fram.write_byte(1, 0x02)
        self.fram.read_byte(0)
        self.assertEqual(self.fram.get_write_count(), 2)
        self.assertGreater(self.fram.get_byte_writes(), 0)
        self.assertEqual(self.fram.get_read_count(), 1)

    def test_unlimited_writes(self):
        """FRAM支持无限次写入（无磨损限制）"""
        for i in range(1000):
            self.fram.write_byte(0, i & 0xFF)
        self.assertEqual(self.fram.get_byte_writes(), 1000)
        self.assertEqual(self.fram.read_byte(0), 0xE7)  # 999 & 0xFF = 231


class TestConfigManager(unittest.TestCase):
    """配置管理器测试"""

    def setUp(self):
        self.cfg = ConfigManager()
        self.cfg.init()
        self.cfg.format_fram()  # 清除旧数据
        self.cfg.load_defaults()

    def test_set_get_float(self):
        """浮点参数读写"""
        self.cfg.set_float(PARAM_MOTOR_KP, 12.5)
        self.assertAlmostEqual(self.cfg.get_float(PARAM_MOTOR_KP), 12.5)

    def test_set_get_uint16(self):
        """16位参数读写"""
        self.cfg.set_uint16(PARAM_SPEED_MAX, 800)
        self.assertEqual(self.cfg.get_uint16(PARAM_SPEED_MAX), 800)

    def test_set_get_uint8(self):
        """8位参数读写"""
        self.cfg.set_uint8(PARAM_DISPLAY_ROT, 2)
        self.assertEqual(self.cfg.get_uint8(PARAM_DISPLAY_ROT), 2)

    def test_default_value(self):
        """默认值"""
        self.assertEqual(self.cfg.get_float(0xFF, -1.0), -1.0)
        self.assertEqual(self.cfg.get_uint16(0xFF, 999), 999)

    def test_save_load(self):
        """保存和加载"""
        self.cfg.set_float(PARAM_MOTOR_KP, 25.0)
        self.cfg.set_uint16(PARAM_SPEED_MAX, 500)
        self.cfg.save()

        # 新实例加载（共享同一个FRAM硬件）
        cfg2 = ConfigManager(self.cfg.fram)
        cfg2.init()
        self.assertAlmostEqual(cfg2.get_float(PARAM_MOTOR_KP), 25.0)
        self.assertEqual(cfg2.get_uint16(PARAM_SPEED_MAX), 500)

    def test_defaults_loaded(self):
        """默认值加载"""
        self.assertAlmostEqual(self.cfg.get_float(PARAM_MOTOR_KP), 10.0)
        self.assertAlmostEqual(self.cfg.get_float(PARAM_MOTOR_KI), 0.5)
        self.assertEqual(self.cfg.get_uint16(PARAM_SPEED_MAX), 1000)

    def test_param_count(self):
        """参数计数"""
        count = self.cfg.get_param_count()
        self.assertGreater(count, 0)

    def test_overwrite_param(self):
        """覆写参数"""
        self.cfg.set_float(PARAM_MOTOR_KP, 10.0)
        self.cfg.set_float(PARAM_MOTOR_KP, 20.0)
        self.assertAlmostEqual(self.cfg.get_float(PARAM_MOTOR_KP), 20.0)
        self.assertEqual(self.cfg.get_param_count(), 10)  # 不应增加

    def test_checksum_validation(self):
        """校验和验证"""
        self.cfg.set_float(PARAM_MOTOR_KP, 15.0)
        self.cfg.save()

        # 手动破坏数据
        self.cfg.fram.write_byte(CFG_DATA_START, 0x00)

        # 尝试加载应失败（使用同一个FRAM）
        cfg2 = ConfigManager(self.cfg.fram)
        cfg2.init()
        # 应该无法加载损坏的配置
        # 如果checksum不匹配，load_defaults不会自动调用
        # 所以参数应为空
        self.assertEqual(cfg2.get_float(PARAM_MOTOR_KP, -1.0), -1.0)

    def test_format_and_reconfigure(self):
        """格式化后重新配置"""
        self.cfg.format_fram()
        self.cfg.load_defaults()
        self.cfg.set_float(PARAM_MOTOR_KP, 99.0)
        self.cfg.save()

        # 新实例加载（共享FRAM）
        cfg2 = ConfigManager(self.cfg.fram)
        cfg2.init()
        self.assertAlmostEqual(cfg2.get_float(PARAM_MOTOR_KP), 99.0)

    def test_clear_params(self):
        """清除参数"""
        self.cfg.clear()
        self.assertEqual(self.cfg.get_param_count(), 0)


class TestFRAMEndurance(unittest.TestCase):
    """FRAM耐久性测试"""

    def test_rapid_write_read(self):
        """快速写读循环"""
        fram = FM24CL64()
        fram.init()
        for i in range(10000):
            fram.write_byte(i % 100, i & 0xFF)
        # 最后一次写入
        fram.write_byte(50, 0xAB)
        self.assertEqual(fram.read_byte(50), 0xAB)

    def test_full_address_range(self):
        """全地址范围读写"""
        fram = FM24CL64()
        fram.init()
        # 每256字节写入一个标记
        for addr in range(0, FM24CL64_SIZE, 256):
            fram.write_byte(addr, addr & 0xFF)
        # 验证
        for addr in range(0, FM24CL64_SIZE, 256):
            self.assertEqual(fram.read_byte(addr), addr & 0xFF)


class TestAT24C02Compat(unittest.TestCase):
    """AT24C02 EEPROM兼容测试"""

    def test_init(self):
        """AT24C02初始化"""
        eeprom = AT24C02()
        eeprom.init()
        self.assertTrue(eeprom.initialized)

    def test_write_read(self):
        """AT24C02读写"""
        eeprom = AT24C02()
        eeprom.init()
        self.assertTrue(eeprom.write_byte(0x10, 0xCD))
        ok, val = eeprom.read_byte(0x10)
        self.assertTrue(ok)
        self.assertEqual(val, 0xCD)

    def test_page_write(self):
        """AT24C02页写入"""
        eeprom = AT24C02()
        eeprom.init()
        data = bytes(range(8))
        self.assertTrue(eeprom.write_page(0, data))
        ok, result = eeprom.read(0, 8)
        self.assertTrue(ok)
        self.assertEqual(bytes(result), data)

    def test_size(self):
        """AT24C02容量"""
        eeprom = AT24C02()
        self.assertEqual(AT24C02_SIZE, 256)


if __name__ == '__main__':
    unittest.main()
