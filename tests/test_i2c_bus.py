#!/usr/bin/env python3
"""
I2C总线管理层单元测试
覆盖: 互斥锁、超时保护、自动重试、错误码、统计计数
测试: 正常操作、边界条件、异常场景、多设备共享总线
参考: 02_mspm0g3507/drivers/i2c_bus.h/.c
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# ── 错误码 (与 i2c_bus.h 一致) ──
I2C_BUS_OK          = 0
I2C_BUS_ERR_BUSY    = -1
I2C_BUS_ERR_TIMEOUT = -2
I2C_BUS_ERR_NACK    = -3
I2C_BUS_ERR_RETRY   = -4

# ── 默认配置 ──
I2C_BUS_TIMEOUT     = 100000
I2C_BUS_MAX_RETRY   = 3
I2C_BUS_RETRY_DELAY = 3200


class I2CBusHandle:
    """I2C总线管理层句柄 (Python镜像)"""

    def __init__(self):
        self.locked = False
        self.timeout = I2C_BUS_TIMEOUT
        self.max_retry = I2C_BUS_MAX_RETRY
        self.tx_count = 0
        self.rx_count = 0
        self.err_count = 0
        self.retry_count = 0
        # 模拟: 是否模拟传输失败
        self._simulate_fail = False
        self._fail_count = 0
        self._fail_times = 0
        # 模拟: 锁行为
        self._simulate_locked = False
        # 存储写入的数据用于验证
        self.last_tx_addr = None
        self.last_tx_data = None
        self.last_rx_addr = None
        self.last_rx_len = None

    def lock(self):
        """获取总线锁"""
        if self._simulate_locked or self.locked:
            return False
        self.locked = True
        return True

    def unlock(self):
        """释放总线锁"""
        self.locked = False

    def _raw_write(self, addr, data):
        """底层写操作 (模拟)"""
        if self._simulate_fail:
            self._fail_count += 1
            if self._fail_count <= self._fail_times:
                return I2C_BUS_ERR_NACK
        self.last_tx_addr = addr
        self.last_tx_data = data
        return I2C_BUS_OK

    def _raw_read(self, addr, length):
        """底层读操作 (模拟)"""
        if self._simulate_fail:
            self._fail_count += 1
            if self._fail_count <= self._fail_times:
                return I2C_BUS_ERR_NACK, None
        self.last_rx_addr = addr
        self.last_rx_len = length
        # 模拟返回数据
        return I2C_BUS_OK, bytes([0x00] * length)

    def write(self, addr, data):
        """I2C写操作 (带锁+重试)"""
        if not self.lock():
            return I2C_BUS_ERR_BUSY

        err = I2C_BUS_ERR_NACK
        retry = 0
        while retry <= self.max_retry:
            err = self._raw_write(addr, data)
            if err == I2C_BUS_OK:
                self.tx_count += 1
                self.unlock()
                return I2C_BUS_OK
            retry += 1
            self.retry_count += 1

        self.err_count += 1
        self.unlock()
        return err

    def read(self, addr, length):
        """I2C读操作 (带锁+重试)"""
        if not self.lock():
            return I2C_BUS_ERR_BUSY, None

        err = I2C_BUS_ERR_NACK
        result = None
        retry = 0
        while retry <= self.max_retry:
            err, result = self._raw_read(addr, length)
            if err == I2C_BUS_OK:
                self.rx_count += 1
                self.unlock()
                return I2C_BUS_OK, result
            retry += 1
            self.retry_count += 1

        self.err_count += 1
        self.unlock()
        return err, None

    def write_reg(self, addr, reg, val):
        """写单个寄存器"""
        return self.write(addr, bytes([reg, val]))

    def read_reg(self, addr, reg):
        """读单个寄存器"""
        err, result = self.write_read(addr, bytes([reg]), 1)
        if err == I2C_BUS_OK and result:
            return I2C_BUS_OK, result[0]
        return err, None

    def write_read(self, addr, tx_data, rx_len):
        """写后读操作"""
        if not self.lock():
            return I2C_BUS_ERR_BUSY, None

        err = I2C_BUS_ERR_NACK
        result = None
        retry = 0
        while retry <= self.max_retry:
            # 写阶段
            err = self._raw_write(addr, tx_data)
            if err != I2C_BUS_OK:
                retry += 1
                self.retry_count += 1
                continue
            # 读阶段
            err, result = self._raw_read(addr, rx_len)
            if err == I2C_BUS_OK:
                self.tx_count += 1
                self.rx_count += 1
                self.unlock()
                return I2C_BUS_OK, result
            retry += 1
            self.retry_count += 1

        self.err_count += 1
        self.unlock()
        return err, None

    def get_stats(self):
        """获取统计信息"""
        return {
            'tx': self.tx_count,
            'rx': self.rx_count,
            'err': self.err_count,
            'retry': self.retry_count,
        }

    @staticmethod
    def error_str(err):
        """错误码转字符串"""
        return {
            I2C_BUS_OK: "OK",
            I2C_BUS_ERR_BUSY: "BUSY (locked)",
            I2C_BUS_ERR_TIMEOUT: "TIMEOUT",
            I2C_BUS_ERR_NACK: "NACK (no response)",
            I2C_BUS_ERR_RETRY: "RETRY exhausted",
        }.get(err, "UNKNOWN")


# ═══════════════════════════════════════════════════════════
#  测试用例
# ═══════════════════════════════════════════════════════════

class TestI2CBusInit(unittest.TestCase):
    """I2C总线初始化测试"""

    def test_default_state(self):
        """测试默认状态"""
        bus = I2CBusHandle()
        self.assertFalse(bus.locked)
        self.assertEqual(bus.timeout, I2C_BUS_TIMEOUT)
        self.assertEqual(bus.max_retry, I2C_BUS_MAX_RETRY)

    def test_stats_zero(self):
        """测试初始统计计数为0"""
        bus = I2CBusHandle()
        stats = bus.get_stats()
        self.assertEqual(stats['tx'], 0)
        self.assertEqual(stats['rx'], 0)
        self.assertEqual(stats['err'], 0)
        self.assertEqual(stats['retry'], 0)


class TestI2CBusLock(unittest.TestCase):
    """互斥锁测试"""

    def test_lock_success(self):
        """测试获取锁成功"""
        bus = I2CBusHandle()
        self.assertTrue(bus.lock())
        self.assertTrue(bus.locked)

    def test_lock_already_locked(self):
        """测试重复获取锁失败"""
        bus = I2CBusHandle()
        bus.lock()
        self.assertFalse(bus.lock())

    def test_unlock(self):
        """测试释放锁"""
        bus = I2CBusHandle()
        bus.lock()
        bus.unlock()
        self.assertFalse(bus.locked)

    def test_lock_after_unlock(self):
        """测试释放后重新获取锁"""
        bus = I2CBusHandle()
        bus.lock()
        bus.unlock()
        self.assertTrue(bus.lock())


class TestI2CBusWrite(unittest.TestCase):
    """I2C写操作测试"""

    def test_write_success(self):
        """测试写操作成功"""
        bus = I2CBusHandle()
        err = bus.write(0x3C, bytes([0x01, 0x02]))
        self.assertEqual(err, I2C_BUS_OK)
        self.assertEqual(bus.last_tx_addr, 0x3C)
        self.assertEqual(bus.last_tx_data, bytes([0x01, 0x02]))

    def test_write_increments_tx_count(self):
        """测试写操作递增发送计数"""
        bus = I2CBusHandle()
        bus.write(0x3C, bytes([0x01]))
        bus.write(0x3D, bytes([0x02]))
        self.assertEqual(bus.tx_count, 2)

    def test_write_retries_on_failure(self):
        """测试写失败自动重试"""
        bus = I2CBusHandle()
        bus._simulate_fail = True
        bus._fail_times = 2  # 前2次失败，第3次成功
        err = bus.write(0x3C, bytes([0x01]))
        self.assertEqual(err, I2C_BUS_OK)
        self.assertEqual(bus.retry_count, 2)

    def test_write_exhausts_retries(self):
        """测试重试次数耗尽返回错误"""
        bus = I2CBusHandle()
        bus._simulate_fail = True
        bus._fail_times = 10  # 始终失败
        err = bus.write(0x3C, bytes([0x01]))
        self.assertEqual(err, I2C_BUS_ERR_NACK)
        self.assertEqual(bus.err_count, 1)

    def test_write_releases_lock_on_success(self):
        """测试写成功后释放锁"""
        bus = I2CBusHandle()
        bus.write(0x3C, bytes([0x01]))
        self.assertFalse(bus.locked)

    def test_write_releases_lock_on_failure(self):
        """测试写失败后释放锁"""
        bus = I2CBusHandle()
        bus._simulate_fail = True
        bus._fail_times = 10
        bus.write(0x3C, bytes([0x01]))
        self.assertFalse(bus.locked)


class TestI2CBusRead(unittest.TestCase):
    """I2C读操作测试"""

    def test_read_success(self):
        """测试读操作成功"""
        bus = I2CBusHandle()
        err, data = bus.read(0x3C, 2)
        self.assertEqual(err, I2C_BUS_OK)
        self.assertIsNotNone(data)
        self.assertEqual(len(data), 2)

    def test_read_increments_rx_count(self):
        """测试读操作递增接收计数"""
        bus = I2CBusHandle()
        bus.read(0x3C, 1)
        bus.read(0x3D, 1)
        self.assertEqual(bus.rx_count, 2)

    def test_read_retries_on_failure(self):
        """测试读失败自动重试"""
        bus = I2CBusHandle()
        bus._simulate_fail = True
        bus._fail_times = 1
        err, data = bus.read(0x3C, 4)
        self.assertEqual(err, I2C_BUS_OK)
        self.assertEqual(bus.retry_count, 1)


class TestI2CBusWriteRead(unittest.TestCase):
    """I2C写后读操作测试"""

    def test_write_read_success(self):
        """测试写后读成功"""
        bus = I2CBusHandle()
        err, data = bus.write_read(0x3C, bytes([0x00]), 2)
        self.assertEqual(err, I2C_BUS_OK)
        self.assertEqual(len(data), 2)

    def test_write_read_increments_counts(self):
        """测试写后读递增收发计数"""
        bus = I2CBusHandle()
        bus.write_read(0x3C, bytes([0x00]), 2)
        self.assertEqual(bus.tx_count, 1)
        self.assertEqual(bus.rx_count, 1)

    def test_write_read_failure_in_write_phase(self):
        """测试写阶段失败"""
        bus = I2CBusHandle()
        bus._simulate_fail = True
        bus._fail_times = 10
        err, data = bus.write_read(0x3C, bytes([0x00]), 2)
        self.assertNotEqual(err, I2C_BUS_OK)


class TestI2CBusRegAccess(unittest.TestCase):
    """寄存器访问测试"""

    def test_write_reg(self):
        """测试写寄存器"""
        bus = I2CBusHandle()
        err = bus.write_reg(0x3C, 0x0A, 0xFF)
        self.assertEqual(err, I2C_BUS_OK)
        self.assertEqual(bus.last_tx_data, bytes([0x0A, 0xFF]))

    def test_read_reg(self):
        """测试读寄存器"""
        bus = I2CBusHandle()
        err, val = bus.read_reg(0x3C, 0x0A)
        self.assertEqual(err, I2C_BUS_OK)
        self.assertIsNotNone(val)


class TestI2CBusBusy(unittest.TestCase):
    """总线忙测试"""

    def test_write_when_busy(self):
        """测试总线忙时写操作返回错误"""
        bus = I2CBusHandle()
        bus._simulate_locked = True
        err = bus.write(0x3C, bytes([0x01]))
        self.assertEqual(err, I2C_BUS_ERR_BUSY)

    def test_read_when_busy(self):
        """测试总线忙时读操作返回错误"""
        bus = I2CBusHandle()
        bus._simulate_locked = True
        err, data = bus.read(0x3C, 2)
        self.assertEqual(err, I2C_BUS_ERR_BUSY)
        self.assertIsNone(data)

    def test_write_read_when_busy(self):
        """测试总线忙时写后读返回错误"""
        bus = I2CBusHandle()
        bus._simulate_locked = True
        err, data = bus.write_read(0x3C, bytes([0x00]), 2)
        self.assertEqual(err, I2C_BUS_ERR_BUSY)

    def test_busy_returns_busy_error(self):
        """测试总线忙返回BUSY错误码"""
        bus = I2CBusHandle()
        bus._simulate_locked = True
        err = bus.write(0x3C, bytes([0x01]))
        self.assertEqual(err, I2C_BUS_ERR_BUSY)
        # BUSY是锁层面的错误，不计入err_count
        self.assertEqual(bus.err_count, 0)


class TestI2CBusErrorStr(unittest.TestCase):
    """错误码转字符串测试"""

    def test_ok(self):
        """测试OK错误码"""
        self.assertEqual(I2CBusHandle.error_str(I2C_BUS_OK), "OK")

    def test_busy(self):
        """测试BUSY错误码"""
        self.assertEqual(I2CBusHandle.error_str(I2C_BUS_ERR_BUSY), "BUSY (locked)")

    def test_timeout(self):
        """测试TIMEOUT错误码"""
        self.assertEqual(I2CBusHandle.error_str(I2C_BUS_ERR_TIMEOUT), "TIMEOUT")

    def test_nack(self):
        """测试NACK错误码"""
        self.assertEqual(I2CBusHandle.error_str(I2C_BUS_ERR_NACK), "NACK (no response)")

    def test_retry(self):
        """测试RETRY错误码"""
        self.assertEqual(I2CBusHandle.error_str(I2C_BUS_ERR_RETRY), "RETRY exhausted")

    def test_unknown(self):
        """测试未知错误码"""
        self.assertEqual(I2CBusHandle.error_str(-99), "UNKNOWN")


class TestI2CBusStats(unittest.TestCase):
    """统计计数测试"""

    def test_stats_after_multiple_ops(self):
        """测试多次操作后统计正确"""
        bus = I2CBusHandle()
        bus.write(0x3C, bytes([0x01]))
        bus.write(0x3D, bytes([0x02]))
        bus.read(0x3C, 2)
        stats = bus.get_stats()
        self.assertEqual(stats['tx'], 2)
        self.assertEqual(stats['rx'], 1)
        self.assertEqual(stats['err'], 0)

    def test_stats_after_failures(self):
        """测试失败后统计正确"""
        bus = I2CBusHandle()
        bus._simulate_fail = True
        bus._fail_times = 10
        bus.write(0x3C, bytes([0x01]))
        stats = bus.get_stats()
        self.assertEqual(stats['tx'], 0)
        self.assertEqual(stats['err'], 1)
        self.assertEqual(stats['retry'], 4)  # max_retry=3, 循环4次

    def test_stats_partial_failure(self):
        """测试部分失败后统计正确"""
        bus = I2CBusHandle()
        bus._simulate_fail = True
        bus._fail_times = 2  # 前2次失败，第3次成功
        bus.write(0x3C, bytes([0x01]))
        stats = bus.get_stats()
        self.assertEqual(stats['tx'], 1)
        self.assertEqual(stats['err'], 0)
        self.assertEqual(stats['retry'], 2)


class TestI2CBusMultiDevice(unittest.TestCase):
    """多设备共享总线测试"""

    def test_different_addresses(self):
        """测试不同地址设备交替访问"""
        bus = I2CBusHandle()
        bus.write(0x3C, bytes([0x01]))  # 设备A
        bus.write(0x68, bytes([0x6B]))  # 设备B
        bus.read(0x3C, 1)               # 设备A
        bus.read(0x68, 6)               # 设备B
        self.assertEqual(bus.last_rx_addr, 0x68)
        self.assertEqual(bus.last_rx_len, 6)

    def test_sequential_operations(self):
        """测试顺序操作不冲突"""
        bus = I2CBusHandle()
        for addr in [0x3C, 0x68, 0x76, 0x48]:
            err = bus.write(addr, bytes([0x00, 0x01]))
            self.assertEqual(err, I2C_BUS_OK)


class TestI2CBusBoundary(unittest.TestCase):
    """边界条件测试"""

    def test_write_empty_data(self):
        """测试写空数据"""
        bus = I2CBusHandle()
        err = bus.write(0x3C, b'')
        self.assertEqual(err, I2C_BUS_OK)

    def test_write_max_data(self):
        """测试写最大长度数据"""
        bus = I2CBusHandle()
        err = bus.write(0x3C, bytes([0x00] * 255))
        self.assertEqual(err, I2C_BUS_OK)

    def test_read_zero_length(self):
        """测试读零长度"""
        bus = I2CBusHandle()
        err, data = bus.read(0x3C, 0)
        self.assertEqual(err, I2C_BUS_OK)

    def test_boundary_address_low(self):
        """测试低地址"""
        bus = I2CBusHandle()
        err = bus.write(0x08, bytes([0x00]))
        self.assertEqual(err, I2C_BUS_OK)

    def test_boundary_address_high(self):
        """测试高地址"""
        bus = I2CBusHandle()
        err = bus.write(0x77, bytes([0x00]))
        self.assertEqual(err, I2C_BUS_OK)


if __name__ == '__main__':
    unittest.main()
