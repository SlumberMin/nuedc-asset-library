#!/usr/bin/env python3
"""
红外接收器 V2 测试 — 基于wrappers.py包装层
覆盖: 初始化、命令注入/读取、缓冲区管理、溢出保护、重复码
对应C源文件: 02_mspm0g3507/drivers/ir_receiver.c

错误经验检查:
  #9:  测试import wrappers.py而非自行重写
  #7:  缓冲区溢出保护
  #5:  FIFO读取顺序
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    IRReceiver, IRCommand,
    IR_PROTO_NEC, IR_PROTO_RC5,
    IR_BUF_SIZE,
)


class TestIRReceiverV2(unittest.TestCase):
    """红外接收器V2 — 基于wrappers.py包装层"""

    def setUp(self):
        self.ir = IRReceiver()
        self.ir.init()

    def test_init_state(self):
        """初始化后状态正确"""
        self.assertTrue(self.ir.initialized)
        self.assertEqual(self.ir.protocol, IR_PROTO_NEC)
        self.assertEqual(self.ir.available(), 0)
        self.assertEqual(self.ir.cmd_count, 0)
        self.assertEqual(self.ir.error_count, 0)
        self.assertFalse(self.ir.repeat_code)

    def test_init_with_rc5(self):
        """RC5协议初始化"""
        ir2 = IRReceiver()
        ir2.init(IR_PROTO_RC5)
        self.assertEqual(ir2.protocol, IR_PROTO_RC5)

    def test_inject_single_command(self):
        """注入单条命令"""
        self.assertTrue(self.ir._inject_command(0x00, 0x0A))
        self.assertEqual(self.ir.available(), 1)
        self.assertEqual(self.ir.cmd_count, 1)

    def test_read_command(self):
        """读取命令"""
        self.ir._inject_command(0x00, 0x0A)
        cmd = self.ir.read()
        self.assertIsNotNone(cmd)
        self.assertEqual(cmd.address, 0x00)
        self.assertEqual(cmd.command, 0x0A)
        self.assertEqual(self.ir.available(), 0)

    def test_read_empty(self):
        """空缓冲区读取返回None"""
        cmd = self.ir.read()
        self.assertIsNone(cmd)

    def test_fifo_order(self):
        """FIFO顺序读取"""
        self.ir._inject_command(0x00, 0x01)
        self.ir._inject_command(0x00, 0x02)
        self.ir._inject_command(0x00, 0x03)
        cmd1 = self.ir.read()
        cmd2 = self.ir.read()
        cmd3 = self.ir.read()
        self.assertEqual(cmd1.command, 0x01)
        self.assertEqual(cmd2.command, 0x02)
        self.assertEqual(cmd3.command, 0x03)

    def test_peek(self):
        """查看但不取出"""
        self.ir._inject_command(0x00, 0x0A)
        cmd = self.ir.peek()
        self.assertIsNotNone(cmd)
        self.assertEqual(cmd.command, 0x0A)
        # peek不消耗，available应仍为1
        self.assertEqual(self.ir.available(), 1)

    def test_peek_empty(self):
        """空缓冲区peek返回None"""
        cmd = self.ir.peek()
        self.assertIsNone(cmd)

    def test_flush(self):
        """清空缓冲区"""
        for i in range(10):
            self.ir._inject_command(0x00, i)
        self.ir.flush()
        self.assertEqual(self.ir.available(), 0)

    def test_overflow_protection(self):
        """缓冲区溢出保护"""
        for i in range(IR_BUF_SIZE + 5):
            self.ir._inject_command(0x00, i)
        # 缓冲区不应超过IR_BUF_SIZE
        self.assertLessEqual(self.ir.available(), IR_BUF_SIZE)
        # 溢出应计入错误数
        self.assertGreater(self.ir.error_count, 0)

    def test_error_count(self):
        """溢出错误计数"""
        for i in range(IR_BUF_SIZE + 3):
            self.ir._inject_command(0x00, i)
        self.assertEqual(self.ir.error_count, 3)

    def test_command_count_total(self):
        """总接收命令计数（含溢出）"""
        for i in range(IR_BUF_SIZE + 5):
            self.ir._inject_command(0x00, i)
        # 只有成功入缓冲区的才计入cmd_count
        self.assertEqual(self.ir.cmd_count, IR_BUF_SIZE)

    def test_multiple_addresses(self):
        """不同地址的命令"""
        self.ir._inject_command(0x01, 0x0A)
        self.ir._inject_command(0x02, 0x0B)
        cmd1 = self.ir.read()
        cmd2 = self.ir.read()
        self.assertEqual(cmd1.address, 0x01)
        self.assertEqual(cmd2.address, 0x02)

    def test_repeat_code(self):
        """重复码检测"""
        self.ir._inject_repeat()
        self.assertTrue(self.ir.is_repeat())

    def test_ir_command_equality(self):
        """IRCommand相等性"""
        cmd1 = IRCommand(0x00, 0x0A, IR_PROTO_NEC)
        cmd2 = IRCommand(0x00, 0x0A, IR_PROTO_NEC)
        cmd3 = IRCommand(0x00, 0x0B, IR_PROTO_NEC)
        self.assertEqual(cmd1, cmd2)
        self.assertNotEqual(cmd1, cmd3)

    def test_ir_command_repr(self):
        """IRCommand字符串表示"""
        cmd = IRCommand(0x12, 0xAB)
        self.assertIn("0x12", repr(cmd))
        self.assertIn("0xAB", repr(cmd))

    def test_inject_returns_false_on_full(self):
        """缓冲区满时注入返回False"""
        for i in range(IR_BUF_SIZE):
            self.ir._inject_command(0x00, i)
        self.assertFalse(self.ir._inject_command(0x00, 0xFF))

    def test_inject_returns_true_when_space(self):
        """缓冲区有空间时注入返回True"""
        self.assertTrue(self.ir._inject_command(0x00, 0x0A))

    def test_nec_typical_command(self):
        """NEC典型遥控器命令"""
        # 模拟常见遥控器: 地址0x00, 命令码
        commands = [
            (0x00, 0x45),  # CH-
            (0x00, 0x46),  # CH
            (0x00, 0x47),  # CH+
            (0x00, 0x44),  # PREV
            (0x00, 0x40),  # NEXT
            (0x00, 0x43),  # PLAY/PAUSE
        ]
        for addr, cmd in commands:
            self.ir._inject_command(addr, cmd)
        self.assertEqual(self.ir.available(), 6)
        for addr, cmd in commands:
            r = self.ir.read()
            self.assertEqual(r.address, addr)
            self.assertEqual(r.command, cmd)

    def test_read_all_then_reuse(self):
        """全部读取后可继续注入"""
        self.ir._inject_command(0x00, 0x01)
        self.ir.read()
        self.assertEqual(self.ir.available(), 0)
        self.ir._inject_command(0x00, 0x02)
        self.assertEqual(self.ir.available(), 1)
        cmd = self.ir.read()
        self.assertEqual(cmd.command, 0x02)


if __name__ == '__main__':
    unittest.main()
