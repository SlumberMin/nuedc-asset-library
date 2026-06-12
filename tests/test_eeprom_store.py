# -*- coding: utf-8 -*-
"""
test_eeprom_store_v2.py - EEPROM存储测试 V2
=============================================
测试内容：
  1. AT24C02 EEPROM初始化
  2. 单字节读写（返回元组）
  3. 页写入
  4. 多字节读写
  5. 参数存储（整型/浮点/字符串）
  6. 数据持久化验证
  7. I2C总线通信
  8. 就绪检测
  9. 环形缓冲区日志
  10. 地址边界

使用 wrappers.py 封装的 AT24C02、I2CBus、RingBuffer
"""

import sys, os, struct
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from wrappers import AT24C02, I2CBus, RingBuffer

# ---- 测试辅助函数 ----
def assert_close(actual, expected, tolerance=0.01, msg=""):
    if abs(actual - expected) > tolerance:
        raise AssertionError(f"{msg}: 期望 {expected}±{tolerance}, 实际 {actual}")

def assert_in_range(value, min_val, max_val, msg=""):
    if value < min_val or value > max_val:
        raise AssertionError(f"{msg}: {value} 不在 [{min_val}, {max_val}] 范围内")

def run_test(test_func, test_name=""):
    try:
        test_func()
        print(f"  [通过] {test_name}")
        return True
    except AssertionError as e:
        print(f"  [失败] {test_name}: {e}")
        return False
    except Exception as e:
        print(f"  [错误] {test_name}: {type(e).__name__}: {e}")
        return False


class EEPROMStore:
    """EEPROM存储系统"""
    TYPE_UINT8 = 0; TYPE_UINT16 = 1; TYPE_UINT32 = 2
    TYPE_INT32 = 3; TYPE_FLOAT = 4; TYPE_STRING = 5

    def __init__(self):
        self.eeprom = AT24C02(); self.eeprom.init()
        self.i2c = I2CBus(); self.i2c.init()
        self.log_buf = RingBuffer(512)
        self.params = {}; self.next_addr = 0
        self.write_count = 0; self.read_count = 0

    def read_byte(self, addr):
        self.read_count += 1
        ok, val = self.eeprom.read_byte(addr)
        return val if ok else 0

    def write_byte(self, addr, val):
        self.eeprom.write_byte(addr, val & 0xFF)
        self.write_count += 1

    def write_bytes(self, addr, data):
        self.eeprom.write(addr, data)
        self.write_count += 1

    def read_bytes(self, addr, length):
        self.read_count += 1
        ok, data = self.eeprom.read(addr, length)
        return data if ok else b'\x00' * length

    def is_ready(self):
        return self.eeprom.is_ready()

    def register_param(self, name, dtype, default=None):
        sizes = {0:1, 1:2, 2:4, 3:4, 4:4, 5:16}
        size = sizes.get(dtype, 4)
        addr = self.next_addr
        self.params[name] = {'addr': addr, 'type': dtype, 'size': size}
        self.next_addr += size
        if default is not None:
            self.write_param(name, default)
        return addr

    def write_param(self, name, value):
        p = self.params[name]
        addr, dtype = p['addr'], p['type']
        if dtype == self.TYPE_UINT8:
            self.write_byte(addr, value)
        elif dtype == self.TYPE_UINT16:
            self.write_bytes(addr, struct.pack('<H', value & 0xFFFF))
        elif dtype == self.TYPE_UINT32:
            self.write_bytes(addr, struct.pack('<I', value & 0xFFFFFFFF))
        elif dtype == self.TYPE_INT32:
            self.write_bytes(addr, struct.pack('<i', value))
        elif dtype == self.TYPE_FLOAT:
            self.write_bytes(addr, struct.pack('<f', value))
        elif dtype == self.TYPE_STRING:
            s = str(value).encode('utf-8')[:15] + b'\x00'
            self.write_bytes(addr, s)

    def read_param(self, name):
        p = self.params[name]
        addr, dtype = p['addr'], p['type']
        if dtype == self.TYPE_UINT8:
            return self.read_byte(addr)
        elif dtype == self.TYPE_UINT16:
            return struct.unpack('<H', self.read_bytes(addr, 2))[0]
        elif dtype == self.TYPE_UINT32:
            return struct.unpack('<I', self.read_bytes(addr, 4))[0]
        elif dtype == self.TYPE_INT32:
            return struct.unpack('<i', self.read_bytes(addr, 4))[0]
        elif dtype == self.TYPE_FLOAT:
            return struct.unpack('<f', self.read_bytes(addr, 4))[0]
        elif dtype == self.TYPE_STRING:
            data = self.read_bytes(addr, 16)
            null_pos = data.find(b'\x00')
            if null_pos >= 0: data = data[:null_pos]
            return data.decode('utf-8', errors='replace')

    def get_stats(self):
        return {'writes': self.write_count, 'reads': self.read_count, 'params': len(self.params)}


def test_eeprom_init():
    """AT24C02初始化"""
    e = AT24C02(); e.init()
    assert e.initialized

def test_write_read_byte():
    """单字节读写"""
    s = EEPROMStore()
    s.write_byte(0, 0xAA)
    assert s.read_byte(0) == 0xAA
    s.write_byte(10, 0x55)
    assert s.read_byte(10) == 0x55

def test_write_read_bytes():
    """多字节读写"""
    s = EEPROMStore()
    data = b'\x01\x02\x03\x04\x05'
    s.write_bytes(0, data)
    assert s.read_bytes(0, 5) == data

def test_page_write():
    """页写入"""
    s = EEPROMStore()
    s.eeprom.write_page(0, bytes(range(8)))
    ok, result = s.eeprom.read(0, 8)
    assert ok and result == bytes(range(8))

def test_is_ready():
    """就绪检测"""
    s = EEPROMStore()
    assert isinstance(s.is_ready(), bool)

def test_param_uint8():
    """uint8参数"""
    s = EEPROMStore()
    s.register_param("speed", EEPROMStore.TYPE_UINT8, 100)
    assert s.read_param("speed") == 100
    s.write_param("speed", 200)
    assert s.read_param("speed") == 200

def test_param_uint16():
    """uint16参数"""
    s = EEPROMStore()
    s.register_param("target", EEPROMStore.TYPE_UINT16, 1000)
    assert s.read_param("target") == 1000
    s.write_param("target", 50000)
    assert s.read_param("target") == 50000

def test_param_uint32():
    """uint32参数"""
    s = EEPROMStore()
    s.register_param("count", EEPROMStore.TYPE_UINT32, 100000)
    assert s.read_param("count") == 100000

def test_param_int32():
    """int32参数"""
    s = EEPROMStore()
    s.register_param("offset", EEPROMStore.TYPE_INT32, -500)
    assert s.read_param("offset") == -500
    s.write_param("offset", 12345)
    assert s.read_param("offset") == 12345

def test_param_float():
    """float参数"""
    s = EEPROMStore()
    s.register_param("kp", EEPROMStore.TYPE_FLOAT, 1.5)
    assert_close(s.read_param("kp"), 1.5, tolerance=0.01, msg="float")

def test_param_string():
    """string参数"""
    s = EEPROMStore()
    s.register_param("name", EEPROMStore.TYPE_STRING, "STM32")
    assert s.read_param("name") == "STM32"
    s.write_param("name", "ESP32")
    assert s.read_param("name") == "ESP32"

def test_multiple_params():
    """多参数"""
    s = EEPROMStore()
    s.register_param("p1", EEPROMStore.TYPE_UINT8, 10)
    s.register_param("p2", EEPROMStore.TYPE_UINT16, 1000)
    s.register_param("p3", EEPROMStore.TYPE_FLOAT, 3.14)
    assert s.read_param("p1") == 10
    assert s.read_param("p2") == 1000
    assert_close(s.read_param("p3"), 3.14, tolerance=0.01, msg="多参数")

def test_persistence():
    """数据持久化"""
    s = EEPROMStore()
    s.register_param("val", EEPROMStore.TYPE_UINT32, 0xDEADBEEF)
    for _ in range(10):
        assert s.read_param("val") == 0xDEADBEEF

def test_i2c_bus():
    """I2C总线"""
    i2c = I2CBus(); i2c.init()
    assert i2c.tx_count == 0

def test_log_buffer():
    """日志缓冲"""
    s = EEPROMStore()
    s.log_buf.put_byte(1); s.log_buf.put_byte(2)
    assert s.log_buf.used() == 2

def test_stats():
    """统计"""
    s = EEPROMStore()
    s.register_param("a", EEPROMStore.TYPE_UINT8, 1)
    s.write_byte(0, 42); s.read_byte(0)
    stats = s.get_stats()
    assert stats['writes'] >= 2
    assert stats['reads'] >= 1
    assert stats['params'] == 1

def test_address_boundary():
    """地址边界"""
    s = EEPROMStore()
    s.write_byte(0, 11); assert s.read_byte(0) == 11
    s.write_byte(255, 99); assert s.read_byte(255) == 99


def main():
    print("=" * 60)
    print("  EEPROM存储系统测试 V2")
    print("=" * 60)
    tests = [
        (test_eeprom_init, "EEPROM初始化"),
        (test_write_read_byte, "单字节读写"), (test_write_read_bytes, "多字节读写"),
        (test_page_write, "页写入"), (test_is_ready, "就绪检测"),
        (test_param_uint8, "uint8参数"), (test_param_uint16, "uint16参数"),
        (test_param_uint32, "uint32参数"), (test_param_int32, "int32参数"),
        (test_param_float, "float参数"), (test_param_string, "string参数"),
        (test_multiple_params, "多参数"), (test_persistence, "数据持久化"),
        (test_i2c_bus, "I2C总线"), (test_log_buffer, "日志缓冲"),
        (test_stats, "统计"), (test_address_boundary, "地址边界"),
    ]
    passed = failed = 0
    for func, name in tests:
        if run_test(func, name): passed += 1
        else: failed += 1
    print("-" * 60)
    print(f"  结果: {passed} 通过, {failed} 失败, 共 {passed + failed} 项")
    print("=" * 60)
    return failed == 0

if __name__ == "__main__":
    sys.exit(0 if main() else 1)
