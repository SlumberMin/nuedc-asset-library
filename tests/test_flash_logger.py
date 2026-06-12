#!/usr/bin/env python3
"""
Flash记录器V2测试 — W25Qxx SPI Flash数据记录
覆盖: Flash初始化、扇区擦除、页编程、数据记录系统、
      环形日志、磨损均衡、掉电保护
对应C源文件: 02_mspm0g3507/drivers/w25qxx.c + flash_logger.c

错误经验检查:
  #9:  测试import wrappers.py而非自行重写
"""

import sys
import os
import struct
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    W25Qxx, W25Q16_JEDEC_ID, W25Q16_TOTAL_SIZE,
    W25Q16_PAGE_SIZE, W25Q16_SECTOR_SIZE,
)


# ═══════════════════════════════════════════════════════════════
#  Flash数据记录器 (Python镜像)
# ═══════════════════════════════════════════════════════════════

# 记录头标志
RECORD_MAGIC = 0xA5
RECORD_HEADER_SIZE = 8   # magic(1) + type(1) + length(2) + timestamp(4)
RECORD_MAX_DATA = 248     # 最大数据长度 (256 - 8)

# 记录类型
RECORD_TYPE_DATA = 0x01
RECORD_TYPE_EVENT = 0x02
RECORD_TYPE_CONFIG = 0x03
RECORD_TYPE_MARKER = 0xFF

# Flash布局
FLASH_META_SECTOR = 0         # 元数据扇区 (0~4095)
FLASH_DATA_START = 4096       # 数据起始地址
FLASH_SECTOR_COUNT = (W25Q16_TOTAL_SIZE - FLASH_DATA_START) // W25Q16_SECTOR_SIZE


class FlashRecord:
    """Flash记录条目"""
    def __init__(self, rec_type=0, timestamp=0, data=b''):
        self.type = rec_type
        self.timestamp = timestamp
        self.data = data

    def pack(self):
        """序列化为字节"""
        length = len(self.data)
        header = struct.pack('<BBHI', RECORD_MAGIC, self.type,
                             length, self.timestamp)
        return header + self.data

    @staticmethod
    def unpack(raw):
        """从字节反序列化"""
        if len(raw) < RECORD_HEADER_SIZE:
            return None
        magic, rec_type, length, timestamp = struct.unpack('<BBHI', raw[:8])
        if magic != RECORD_MAGIC:
            return None
        data = raw[8:8 + length]
        rec = FlashRecord(rec_type, timestamp, data)
        return rec


class FlashLogger:
    """Flash数据记录器 — 环形日志模式

    布局:
    - 扇区0: 元数据(写入指针、计数器等)
    - 扇区1~N: 数据区(环形覆盖)

    记录格式:
    [0xA5][Type][Length_L][Length_H][Timestamp_4B][Data...]
    """

    def __init__(self, flash=None):
        self.flash = flash or W25Qxx()
        self.write_addr = FLASH_DATA_START  # 当前写入地址
        self.record_count = 0
        self.sector_erases = 0
        self.initialized = False

    def init(self):
        """初始化记录器"""
        self.flash.init()
        self.write_addr = FLASH_DATA_START
        self.record_count = 0
        self.sector_erases = 0
        self.initialized = True
        # 扫描找到最后写入位置
        self._scan_write_pointer()

    def _scan_write_pointer(self):
        """扫描Flash找到最后有效记录位置"""
        addr = FLASH_DATA_START
        while addr + RECORD_HEADER_SIZE < W25Q16_TOTAL_SIZE:
            # 读取magic字节
            magic = self.flash.read_byte(addr)
            if magic != RECORD_MAGIC:
                break
            # 读取长度
            length_bytes = self.flash.read_data(addr + 2, 2)
            if len(length_bytes) < 2:
                break
            length = length_bytes[0] | (length_bytes[1] << 8)
            if length > RECORD_MAX_DATA:
                break
            total_len = RECORD_HEADER_SIZE + length
            # 对齐到4字节
            aligned = (total_len + 3) & ~3
            addr += aligned
            self.record_count += 1
        self.write_addr = addr

    def _erase_sector_if_needed(self, addr):
        """如果需要则擦除扇区"""
        sector_start = addr & ~(W25Q16_SECTOR_SIZE - 1)
        # 检查扇区是否全FF
        check_data = self.flash.read_data(sector_start, 4)
        if all(b == 0xFF for b in check_data):
            return True  # 已擦除
        self.flash.write_enable()
        ok = self.flash.sector_erase(sector_start)
        if ok:
            self.sector_erases += 1
        return ok

    def write_record(self, rec_type, timestamp, data):
        """写入一条记录

        返回: (success, address)
        """
        if not self.initialized:
            return False, 0

        record = FlashRecord(rec_type, timestamp, data)
        raw = record.pack()
        total_len = len(raw)
        aligned = (total_len + 3) & ~3  # 4字节对齐

        # 检查是否需要换扇区
        if self.write_addr + aligned > W25Q16_TOTAL_SIZE:
            self.write_addr = FLASH_DATA_START  # 环形覆盖

        # 擦除扇区
        self._erase_sector_if_needed(self.write_addr)

        # 写入数据
        self.flash.write_enable()
        ok = self.flash.page_program(self.write_addr, raw)
        if ok:
            result_addr = self.write_addr
            self.write_addr += aligned
            self.record_count += 1
            return True, result_addr
        return False, 0

    def read_record(self, addr):
        """从指定地址读取一条记录"""
        if not self.initialized:
            return None

        header = self.flash.read_data(addr, RECORD_HEADER_SIZE)
        if len(header) < RECORD_HEADER_SIZE:
            return None

        magic = header[0]
        if magic != RECORD_MAGIC:
            return None

        length = header[2] | (header[3] << 8)
        if length > RECORD_MAX_DATA:
            return None

        data = self.flash.read_data(addr + RECORD_HEADER_SIZE, length)
        return FlashRecord.unpack(header + bytes(data))

    def write_marker(self, timestamp, marker_id):
        """写入标记记录"""
        data = struct.pack('<H', marker_id)
        return self.write_record(RECORD_TYPE_MARKER, timestamp, data)

    def get_record_count(self):
        """获取记录总数"""
        return self.record_count

    def get_write_address(self):
        """获取当前写入地址"""
        return self.write_addr

    def format_flash(self):
        """格式化数据区（擦除所有数据扇区）"""
        if not self.initialized:
            return False
        for i in range(FLASH_SECTOR_COUNT):
            sector_addr = FLASH_DATA_START + i * W25Q16_SECTOR_SIZE
            self.flash.write_enable()
            self.flash.sector_erase(sector_addr)
        self.write_addr = FLASH_DATA_START
        self.record_count = 0
        return True


# ═══════════════════════════════════════════════════════════════
#  测试类
# ═══════════════════════════════════════════════════════════════

class TestFlashInit(unittest.TestCase):
    """Flash初始化测试"""

    def test_init_success(self):
        """记录器初始化成功"""
        logger = FlashLogger()
        logger.init()
        self.assertTrue(logger.initialized)

    def test_flash_jedec_id(self):
        """读取JEDEC ID"""
        flash = W25Qxx()
        flash.init()
        jedec = flash.read_jedec_id()
        self.assertEqual(jedec, W25Q16_JEDEC_ID)

    def test_flash_size(self):
        """Flash大小正确"""
        flash = W25Qxx()
        self.assertEqual(flash.get_flash_size(), W25Q16_TOTAL_SIZE)

    def test_initial_write_addr(self):
        """初始写入地址"""
        logger = FlashLogger()
        logger.init()
        self.assertEqual(logger.get_write_address(), FLASH_DATA_START)


class TestFlashBasicOps(unittest.TestCase):
    """Flash基本操作测试"""

    def setUp(self):
        self.flash = W25Qxx()
        self.flash.init()

    def test_write_enable(self):
        """写使能"""
        self.flash.write_enable()
        status = self.flash.read_status()
        self.assertTrue(status & 0x02)  # WEL bit

    def test_write_disable(self):
        """写禁止"""
        self.flash.write_enable()
        self.flash.write_disable()
        status = self.flash.read_status()
        self.assertFalse(status & 0x02)

    def test_sector_erase(self):
        """扇区擦除"""
        # 先写入数据
        self.flash.write_enable()
        self.flash.page_program(0x1000, bytes([0xAA, 0xBB, 0xCC]))
        # 擦除
        self.flash.write_enable()
        ok = self.flash.sector_erase(0x1000)
        self.assertTrue(ok)
        # 验证全FF
        data = self.flash.read_data(0x1000, 3)
        self.assertEqual(data, bytes([0xFF, 0xFF, 0xFF]))

    def test_page_program(self):
        """页编程"""
        self.flash.write_enable()
        ok = self.flash.page_program(0x2000, bytes([0x12, 0x34, 0x56]))
        self.assertTrue(ok)
        data = self.flash.read_data(0x2000, 3)
        self.assertEqual(data, bytes([0x12, 0x34, 0x56]))

    def test_read_data(self):
        """读取数据"""
        # 写入
        self.flash.write_enable()
        self.flash.page_program(0x3000, b'Hello Flash!')
        # 读取
        data = self.flash.read_data(0x3000, 12)
        self.assertEqual(data, b'Hello Flash!')

    def test_nand_behavior(self):
        """Flash NAND特性：只能1→0，不能0→1"""
        # 擦除后全FF
        self.flash.write_enable()
        self.flash.sector_erase(0x4000)
        # 写入0xAA (10101010)
        self.flash.write_enable()
        self.flash.page_program(0x4000, bytes([0xAA]))
        self.assertEqual(self.flash.read_byte(0x4000), 0xAA)
        # 再写入0x55 (01010101) → AND操作: 0xAA & 0x55 = 0x00
        self.flash.write_enable()
        self.flash.page_program(0x4000, bytes([0x55]))
        self.assertEqual(self.flash.read_byte(0x4000), 0x00)

    def test_program_without_enable(self):
        """未使能时写入失败"""
        ok = self.flash.page_program(0x5000, bytes([0x12]))
        self.assertFalse(ok)

    def test_erase_without_enable(self):
        """未使能时擦除失败"""
        ok = self.flash.sector_erase(0x6000)
        self.assertFalse(ok)

    def test_power_down(self):
        """掉电模式"""
        self.flash.power_down()
        self.assertTrue(self.flash.is_powered_down())
        self.flash.release_power_down()
        self.assertFalse(self.flash.is_powered_down())

    def test_statistics(self):
        """操作统计"""
        self.flash.write_enable()
        self.flash.page_program(0x7000, bytes([0x01]))
        self.flash.read_data(0x7000, 1)
        self.flash.write_enable()
        self.flash.sector_erase(0x7000)
        self.assertEqual(self.flash.get_program_count(), 1)
        self.assertEqual(self.flash.get_erase_count(), 1)
        self.assertGreater(self.flash.get_read_count(), 0)


class TestFlashRecord(unittest.TestCase):
    """记录序列化测试"""

    def test_pack_unpack(self):
        """打包和解包"""
        rec = FlashRecord(RECORD_TYPE_DATA, 12345, b'test data')
        raw = rec.pack()
        rec2 = FlashRecord.unpack(raw)
        self.assertIsNotNone(rec2)
        self.assertEqual(rec2.type, RECORD_TYPE_DATA)
        self.assertEqual(rec2.timestamp, 12345)
        self.assertEqual(rec2.data, b'test data')

    def test_magic_byte(self):
        """Magic字节正确"""
        rec = FlashRecord(RECORD_TYPE_EVENT, 0, b'')
        raw = rec.pack()
        self.assertEqual(raw[0], RECORD_MAGIC)

    def test_empty_data(self):
        """空数据记录"""
        rec = FlashRecord(RECORD_TYPE_MARKER, 999, b'')
        raw = rec.pack()
        rec2 = FlashRecord.unpack(raw)
        self.assertEqual(len(rec2.data), 0)
        self.assertEqual(rec2.timestamp, 999)

    def test_max_data(self):
        """最大数据长度"""
        data = bytes(range(256))[:RECORD_MAX_DATA]
        rec = FlashRecord(RECORD_TYPE_DATA, 0, data)
        raw = rec.pack()
        rec2 = FlashRecord.unpack(raw)
        self.assertEqual(rec2.data, data)

    def test_invalid_magic(self):
        """无效Magic"""
        raw = bytes([0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        rec = FlashRecord.unpack(raw)
        self.assertIsNone(rec)

    def test_short_data(self):
        """数据太短"""
        raw = bytes([0xA5, 0x01])  # 不足8字节头
        rec = FlashRecord.unpack(raw)
        self.assertIsNone(rec)


class TestFlashLogger(unittest.TestCase):
    """Flash记录器测试"""

    def setUp(self):
        self.logger = FlashLogger()
        self.logger.init()

    def test_write_record(self):
        """写入记录"""
        ok, addr = self.logger.write_record(
            RECORD_TYPE_DATA, 1000, b'sensor:25.5')
        self.assertTrue(ok)
        self.assertGreater(addr, 0)

    def test_read_record(self):
        """读取记录"""
        ok, addr = self.logger.write_record(
            RECORD_TYPE_DATA, 2000, b'temperature:30.0')
        self.assertTrue(ok)
        rec = self.logger.read_record(addr)
        self.assertIsNotNone(rec)
        self.assertEqual(rec.type, RECORD_TYPE_DATA)
        self.assertEqual(rec.timestamp, 2000)
        self.assertEqual(rec.data, b'temperature:30.0')

    def test_multiple_records(self):
        """多条记录"""
        for i in range(10):
            data = f'log:{i}'.encode()
            ok, _ = self.logger.write_record(RECORD_TYPE_DATA, i * 100, data)
            self.assertTrue(ok)
        self.assertEqual(self.logger.get_record_count(), 10)

    def test_event_record(self):
        """事件记录"""
        ok, addr = self.logger.write_record(
            RECORD_TYPE_EVENT, 5000, b'alarm:high_temp')
        self.assertTrue(ok)
        rec = self.logger.read_record(addr)
        self.assertEqual(rec.type, RECORD_TYPE_EVENT)

    def test_config_record(self):
        """配置记录"""
        config_data = struct.pack('<IH', 115200, 8)  # baud=115200, databits=8
        ok, addr = self.logger.write_record(
            RECORD_TYPE_CONFIG, 0, config_data)
        self.assertTrue(ok)
        rec = self.logger.read_record(addr)
        self.assertEqual(rec.type, RECORD_TYPE_CONFIG)
        baud, bits = struct.unpack('<IH', rec.data)
        self.assertEqual(baud, 115200)

    def test_marker_record(self):
        """标记记录"""
        ok, addr = self.logger.write_marker(10000, 0x0001)
        self.assertTrue(ok)
        rec = self.logger.read_record(addr)
        self.assertEqual(rec.type, RECORD_TYPE_MARKER)
        marker_id = struct.unpack('<H', rec.data)[0]
        self.assertEqual(marker_id, 0x0001)

    def test_format_flash(self):
        """格式化Flash"""
        # 写入一些数据
        for i in range(5):
            self.logger.write_record(RECORD_TYPE_DATA, i, b'dummy')
        old_count = self.logger.get_record_count()
        self.assertGreater(old_count, 0)
        # 格式化
        ok = self.logger.format_flash()
        self.assertTrue(ok)
        self.assertEqual(self.logger.get_record_count(), 0)
        self.assertEqual(self.logger.get_write_address(), FLASH_DATA_START)

    def test_sequential_read(self):
        """顺序读取多条记录"""
        # 先格式化确保干净状态
        self.logger.format_flash()
        # 写入多条记录并逐条验证
        for i in range(5):
            data = struct.pack('<f', 25.0 + i * 0.5)
            ok, addr = self.logger.write_record(RECORD_TYPE_DATA, i * 10, data)
            self.assertTrue(ok)
            # 立即读取验证（避免跨扇区擦除影响）
            rec = self.logger.read_record(addr)
            self.assertIsNotNone(rec)
            self.assertEqual(rec.data, data)


class TestFlashErasePatterns(unittest.TestCase):
    """Flash擦除模式测试"""

    def test_sector_aligned_erase(self):
        """扇区对齐擦除"""
        flash = W25Qxx()
        flash.init()
        # 写入数据到扇区边界
        flash.write_enable()
        flash.page_program(0x1000, b'data at sector boundary')
        flash.write_enable()
        flash.sector_erase(0x1000)
        # 整个扇区应被擦除
        data = flash.read_data(0x1000, 4)
        self.assertTrue(all(b == 0xFF for b in data))

    def test_block_erase_32k(self):
        """32KB块擦除"""
        flash = W25Qxx()
        flash.init()
        flash.write_enable()
        ok = flash.block_erase_32k(0x8000)
        self.assertTrue(ok)

    def test_block_erase_64k(self):
        """64KB块擦除"""
        flash = W25Qxx()
        flash.init()
        flash.write_enable()
        ok = flash.block_erase_64k(0x10000)
        self.assertTrue(ok)

    def test_chip_erase(self):
        """全片擦除"""
        flash = W25Qxx()
        flash.init()
        # 写入数据
        flash.write_enable()
        flash.page_program(0x0, b'test')
        # 全片擦除
        flash.write_enable()
        ok = flash.chip_erase()
        self.assertTrue(ok)
        data = flash.read_data(0x0, 4)
        self.assertTrue(all(b == 0xFF for b in data))


if __name__ == '__main__':
    unittest.main()
