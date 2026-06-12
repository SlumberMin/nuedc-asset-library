#!/usr/bin/env python3
"""
电机保护模块单元测试
覆盖: 过流检测、过温检测、堵转检测、冷却恢复、状态机转换
测试: 正常运行、边界条件、异常输入、多次故障累计
参考: 02_mspm0g3507/drivers/motor_protect.h/.c
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# ── 保护标志位 (与 motor_protect.h 一致) ──
MOTOR_PROT_NONE         = 0x00
MOTOR_PROT_OVERCURRENT  = 0x01
MOTOR_PROT_OVERTEMP     = 0x02
MOTOR_PROT_STALL        = 0x04
MOTOR_PROT_ALL          = 0x07

# ── 电机状态枚举 ──
MOTOR_STATE_IDLE       = 0
MOTOR_STATE_RUNNING    = 1
MOTOR_STATE_PROTECTED  = 2
MOTOR_STATE_COOLDOWN   = 3

STATE_NAMES = {
    MOTOR_STATE_IDLE: "IDLE",
    MOTOR_STATE_RUNNING: "RUNNING",
    MOTOR_STATE_PROTECTED: "PROTECTED",
    MOTOR_STATE_COOLDOWN: "COOLDOWN",
}


class MotorProtectParam:
    """保护参数"""
    def __init__(self):
        self.current_limit = 5.0        # 过流阈值 (A)
        self.temp_limit = 80.0          # 过温阈值 (°C)
        self.stall_speed_thresh = 50.0  # 堵转速度阈值 (RPM)
        self.stall_current_thresh = 3.0 # 堵转电流阈值 (A)
        self.stall_time_ms = 500        # 堵转持续时间阈值 (ms)
        self.cooldown_ms = 3000         # 冷却时间 (ms)


class MotorProtectHandle:
    """电机保护句柄 (Python镜像)"""

    def __init__(self):
        self.state = MOTOR_STATE_IDLE
        self.fault_flag = MOTOR_PROT_NONE
        self.current_a = 0.0
        self.temperature_c = 25.0
        self.speed_rpm = 0.0
        self.stall_counter_ms = 0
        self.cooldown_counter = 0
        self.fault_count = 0
        self.param = MotorProtectParam()

    def set_param(self, current_limit, temp_limit, stall_speed_thresh,
                  stall_current_thresh, stall_time_ms, cooldown_ms):
        """设置保护参数"""
        self.param.current_limit = current_limit
        self.param.temp_limit = temp_limit
        self.param.stall_speed_thresh = stall_speed_thresh
        self.param.stall_current_thresh = stall_current_thresh
        self.param.stall_time_ms = stall_time_ms
        self.param.cooldown_ms = cooldown_ms

    def update(self, current_a, temperature_c, speed_rpm):
        """周期性更新 (10ms)"""
        self.current_a = current_a
        self.temperature_c = temperature_c
        self.speed_rpm = speed_rpm

        # 冷却状态处理
        if self.state == MOTOR_STATE_COOLDOWN:
            self.cooldown_counter += 10
            if self.cooldown_counter >= self.param.cooldown_ms:
                self.cooldown_counter = 0
                self.fault_flag = MOTOR_PROT_NONE
                self.state = MOTOR_STATE_IDLE
            return

        # 已保护状态不重复检测
        if self.state == MOTOR_STATE_PROTECTED:
            return

        # 过流检测
        abs_current = abs(current_a)
        if abs_current > self.param.current_limit:
            self.fault_flag |= MOTOR_PROT_OVERCURRENT
            self.fault_count += 1
            self.cooldown_counter = 0
            self.state = MOTOR_STATE_COOLDOWN
            return

        # 过温检测
        if temperature_c > self.param.temp_limit:
            self.fault_flag |= MOTOR_PROT_OVERTEMP
            self.fault_count += 1
            self.cooldown_counter = 0
            self.state = MOTOR_STATE_COOLDOWN
            return

        # 堵转检测: 低速 + 高电流持续一段时间
        abs_speed = abs(speed_rpm)
        if abs_speed < self.param.stall_speed_thresh and \
           abs_current > self.param.stall_current_thresh:
            self.stall_counter_ms += 10
            if self.stall_counter_ms >= self.param.stall_time_ms:
                self.fault_flag |= MOTOR_PROT_STALL
                self.fault_count += 1
                self.stall_counter_ms = 0
                self.cooldown_counter = 0
                self.state = MOTOR_STATE_COOLDOWN
                return
        else:
            self.stall_counter_ms = 0

        # 正常运行
        self.state = MOTOR_STATE_RUNNING

    def get_state(self):
        return self.state

    def get_fault(self):
        return self.fault_flag

    def clear_fault(self):
        self.fault_flag = MOTOR_PROT_NONE
        self.stall_counter_ms = 0
        self.cooldown_counter = 0
        self.state = MOTOR_STATE_IDLE

    def is_output_enabled(self):
        return self.state in (MOTOR_STATE_RUNNING, MOTOR_STATE_IDLE)

    def get_fault_str(self):
        if self.fault_flag & MOTOR_PROT_OVERCURRENT:
            return "OVERCURRENT"
        if self.fault_flag & MOTOR_PROT_OVERTEMP:
            return "OVERTEMP"
        if self.fault_flag & MOTOR_PROT_STALL:
            return "STALL"
        return "NONE"


# ═══════════════════════════════════════════════════════════
#  测试用例
# ═══════════════════════════════════════════════════════════

class TestMotorProtInit(unittest.TestCase):
    """电机保护初始化测试"""

    def test_default_state(self):
        """测试默认状态为IDLE"""
        hmp = MotorProtectHandle()
        self.assertEqual(hmp.state, MOTOR_STATE_IDLE)
        self.assertEqual(hmp.fault_flag, MOTOR_PROT_NONE)

    def test_default_params(self):
        """测试默认保护参数"""
        hmp = MotorProtectHandle()
        self.assertEqual(hmp.param.current_limit, 5.0)
        self.assertEqual(hmp.param.temp_limit, 80.0)
        self.assertEqual(hmp.param.stall_speed_thresh, 50.0)
        self.assertEqual(hmp.param.stall_current_thresh, 3.0)
        self.assertEqual(hmp.param.stall_time_ms, 500)
        self.assertEqual(hmp.param.cooldown_ms, 3000)

    def test_set_param(self):
        """测试设置自定义保护参数"""
        hmp = MotorProtectHandle()
        hmp.set_param(10.0, 100.0, 30.0, 5.0, 1000, 5000)
        self.assertEqual(hmp.param.current_limit, 10.0)
        self.assertEqual(hmp.param.temp_limit, 100.0)
        self.assertEqual(hmp.param.stall_speed_thresh, 30.0)
        self.assertEqual(hmp.param.stall_current_thresh, 5.0)
        self.assertEqual(hmp.param.stall_time_ms, 1000)
        self.assertEqual(hmp.param.cooldown_ms, 5000)

    def test_fault_count_zero(self):
        """测试初始故障计数为0"""
        hmp = MotorProtectHandle()
        self.assertEqual(hmp.fault_count, 0)


class TestMotorProtNormal(unittest.TestCase):
    """正常运行测试"""

    def test_idle_to_running(self):
        """测试IDLE到RUNNING转换"""
        hmp = MotorProtectHandle()
        hmp.update(1.0, 25.0, 300.0)
        self.assertEqual(hmp.state, MOTOR_STATE_RUNNING)

    def test_running_stable(self):
        """测试稳定运行状态"""
        hmp = MotorProtectHandle()
        for _ in range(100):
            hmp.update(2.0, 40.0, 500.0)
        self.assertEqual(hmp.state, MOTOR_STATE_RUNNING)
        self.assertEqual(hmp.fault_flag, MOTOR_PROT_NONE)

    def test_output_enabled_running(self):
        """测试运行时允许输出"""
        hmp = MotorProtectHandle()
        hmp.update(1.0, 25.0, 300.0)
        self.assertTrue(hmp.is_output_enabled())

    def test_output_enabled_idle(self):
        """测试空闲时允许输出"""
        hmp = MotorProtectHandle()
        self.assertTrue(hmp.is_output_enabled())

    def test_zero_current_zero_speed(self):
        """测试零电流零速度(静止状态)"""
        hmp = MotorProtectHandle()
        hmp.update(0.0, 25.0, 0.0)
        # 零速+零电流，不触发堵转(电流未超堵转阈值)
        self.assertEqual(hmp.state, MOTOR_STATE_RUNNING)

    def test_fault_str_none(self):
        """测试无故障时描述为NONE"""
        hmp = MotorProtectHandle()
        self.assertEqual(hmp.get_fault_str(), "NONE")


class TestMotorProtOvercurrent(unittest.TestCase):
    """过流保护测试"""

    def test_overcurrent_triggers(self):
        """测试过流触发保护"""
        hmp = MotorProtectHandle()
        # 电流超过5A阈值
        hmp.update(6.0, 25.0, 500.0)
        self.assertEqual(hmp.state, MOTOR_STATE_COOLDOWN)
        self.assertTrue(hmp.fault_flag & MOTOR_PROT_OVERCURRENT)

    def test_overcurrent_negative(self):
        """测试负方向过流"""
        hmp = MotorProtectHandle()
        hmp.update(-6.0, 25.0, 500.0)
        self.assertTrue(hmp.fault_flag & MOTOR_PROT_OVERCURRENT)

    def test_overcurrent_exact_limit(self):
        """测试恰好等于电流阈值(不触发)"""
        hmp = MotorProtectHandle()
        hmp.update(5.0, 25.0, 500.0)
        # 等于阈值不触发 (> 而非 >=)
        self.assertEqual(hmp.state, MOTOR_STATE_RUNNING)

    def test_overcurrent_just_over(self):
        """测试略超电流阈值"""
        hmp = MotorProtectHandle()
        hmp.update(5.01, 25.0, 500.0)
        self.assertTrue(hmp.fault_flag & MOTOR_PROT_OVERCURRENT)

    def test_overcurrent_fault_count(self):
        """测试过流故障计数递增"""
        hmp = MotorProtectHandle()
        hmp.update(6.0, 25.0, 500.0)
        self.assertEqual(hmp.fault_count, 1)

    def test_overcurrent_output_disabled(self):
        """测试过流后禁止输出"""
        hmp = MotorProtectHandle()
        hmp.update(6.0, 25.0, 500.0)
        self.assertFalse(hmp.is_output_enabled())

    def test_overcurrent_fault_str(self):
        """测试过流故障描述"""
        hmp = MotorProtectHandle()
        hmp.update(6.0, 25.0, 500.0)
        self.assertEqual(hmp.get_fault_str(), "OVERCURRENT")


class TestMotorProtOvertemp(unittest.TestCase):
    """过温保护测试"""

    def test_overtemp_triggers(self):
        """测试过温触发保护"""
        hmp = MotorProtectHandle()
        hmp.update(1.0, 85.0, 500.0)
        self.assertTrue(hmp.fault_flag & MOTOR_PROT_OVERTEMP)

    def test_overtemp_exact_limit(self):
        """测试恰好等于温度阈值(不触发)"""
        hmp = MotorProtectHandle()
        hmp.update(1.0, 80.0, 500.0)
        self.assertEqual(hmp.state, MOTOR_STATE_RUNNING)

    def test_overtemp_just_over(self):
        """测试略超温度阈值"""
        hmp = MotorProtectHandle()
        hmp.update(1.0, 80.1, 500.0)
        self.assertTrue(hmp.fault_flag & MOTOR_PROT_OVERTEMP)

    def test_overtemp_fault_str(self):
        """测试过温故障描述"""
        hmp = MotorProtectHandle()
        hmp.update(1.0, 85.0, 500.0)
        self.assertEqual(hmp.get_fault_str(), "OVERTEMP")


class TestMotorProtStall(unittest.TestCase):
    """堵转保护测试"""

    def test_stall_detection(self):
        """测试堵转检测: 低速+高电流持续500ms"""
        hmp = MotorProtectHandle()
        # 低速(30 RPM < 50阈值) + 高电流(4A > 3A阈值)
        for _ in range(50):  # 50 * 10ms = 500ms
            hmp.update(4.0, 25.0, 30.0)
        self.assertTrue(hmp.fault_flag & MOTOR_PROT_STALL)
        self.assertEqual(hmp.state, MOTOR_STATE_COOLDOWN)

    def test_stall_not_triggered_too_short(self):
        """测试堵转未达持续时间不触发"""
        hmp = MotorProtectHandle()
        # 只持续490ms (不足500ms)
        for _ in range(49):
            hmp.update(4.0, 25.0, 30.0)
        self.assertFalse(hmp.fault_flag & MOTOR_PROT_STALL)

    def test_stall_interrupted(self):
        """测试堵转检测被打断(电流恢复正常)"""
        hmp = MotorProtectHandle()
        # 先累积400ms
        for _ in range(40):
            hmp.update(4.0, 25.0, 30.0)
        self.assertEqual(hmp.stall_counter_ms, 400)
        # 电流恢复正常，计数器应重置
        hmp.update(1.0, 25.0, 30.0)
        self.assertEqual(hmp.stall_counter_ms, 0)

    def test_stall_speed_too_high(self):
        """测试速度高于阈值不触发堵转"""
        hmp = MotorProtectHandle()
        for _ in range(100):
            hmp.update(4.0, 25.0, 100.0)  # 100 > 50阈值
        self.assertFalse(hmp.fault_flag & MOTOR_PROT_STALL)

    def test_stall_current_too_low(self):
        """测试电流低于阈值不触发堵转"""
        hmp = MotorProtectHandle()
        for _ in range(100):
            hmp.update(2.0, 25.0, 30.0)  # 2A < 3A阈值
        self.assertFalse(hmp.fault_flag & MOTOR_PROT_STALL)

    def test_stall_negative_speed(self):
        """测试负速度(反转)堵转检测"""
        hmp = MotorProtectHandle()
        for _ in range(50):
            hmp.update(4.0, 25.0, -30.0)
        self.assertTrue(hmp.fault_flag & MOTOR_PROT_STALL)

    def test_stall_fault_str(self):
        """测试堵转故障描述"""
        hmp = MotorProtectHandle()
        for _ in range(50):
            hmp.update(4.0, 25.0, 30.0)
        self.assertEqual(hmp.get_fault_str(), "STALL")


class TestMotorProtCooldown(unittest.TestCase):
    """冷却恢复测试"""

    def test_cooldown_recovery(self):
        """测试冷却后自动恢复到IDLE"""
        hmp = MotorProtectHandle()
        # 触发过流保护
        hmp.update(6.0, 25.0, 500.0)
        self.assertEqual(hmp.state, MOTOR_STATE_COOLDOWN)
        # 冷却3000ms (300次 * 10ms)
        for _ in range(300):
            hmp.update(1.0, 25.0, 500.0)
        self.assertEqual(hmp.state, MOTOR_STATE_IDLE)
        self.assertEqual(hmp.fault_flag, MOTOR_PROT_NONE)

    def test_cooldown_no_detection(self):
        """测试冷却期间不进行新的检测"""
        hmp = MotorProtectHandle()
        hmp.update(6.0, 25.0, 500.0)  # 触发过流
        self.assertEqual(hmp.state, MOTOR_STATE_COOLDOWN)
        # 冷却期间输入过温，不应改变状态
        hmp.update(1.0, 100.0, 500.0)
        self.assertEqual(hmp.state, MOTOR_STATE_COOLDOWN)

    def test_cooldown_partial(self):
        """测试冷却未完成不恢复"""
        hmp = MotorProtectHandle()
        hmp.update(6.0, 25.0, 500.0)
        # 只冷却2000ms (不足3000ms)
        for _ in range(200):
            hmp.update(1.0, 25.0, 500.0)
        self.assertEqual(hmp.state, MOTOR_STATE_COOLDOWN)

    def test_cooldown_custom_time(self):
        """测试自定义冷却时间"""
        hmp = MotorProtectHandle()
        hmp.set_param(5.0, 80.0, 50.0, 3.0, 500, 1000)  # 1秒冷却
        hmp.update(6.0, 25.0, 500.0)
        # 冷却1000ms (100次 * 10ms)
        for _ in range(100):
            hmp.update(1.0, 25.0, 500.0)
        self.assertEqual(hmp.state, MOTOR_STATE_IDLE)


class TestMotorProtClearFault(unittest.TestCase):
    """手动清除故障测试"""

    def test_clear_fault_resets_state(self):
        """测试清除故障重置状态"""
        hmp = MotorProtectHandle()
        hmp.update(6.0, 25.0, 500.0)
        hmp.clear_fault()
        self.assertEqual(hmp.state, MOTOR_STATE_IDLE)
        self.assertEqual(hmp.fault_flag, MOTOR_PROT_NONE)
        self.assertEqual(hmp.stall_counter_ms, 0)
        self.assertEqual(hmp.cooldown_counter, 0)

    def test_clear_fault_reenables_output(self):
        """测试清除故障后重新允许输出"""
        hmp = MotorProtectHandle()
        hmp.update(6.0, 25.0, 500.0)
        self.assertFalse(hmp.is_output_enabled())
        hmp.clear_fault()
        self.assertTrue(hmp.is_output_enabled())


class TestMotorProtBoundary(unittest.TestCase):
    """边界条件测试"""

    def test_very_large_current(self):
        """测试极大电流值"""
        hmp = MotorProtectHandle()
        hmp.update(1000.0, 25.0, 500.0)
        self.assertTrue(hmp.fault_flag & MOTOR_PROT_OVERCURRENT)

    def test_very_high_temperature(self):
        """测试极高温度"""
        hmp = MotorProtectHandle()
        hmp.update(1.0, 200.0, 500.0)
        self.assertTrue(hmp.fault_flag & MOTOR_PROT_OVERTEMP)

    def test_custom_thresholds(self):
        """测试自定义阈值"""
        hmp = MotorProtectHandle()
        hmp.set_param(10.0, 120.0, 20.0, 5.0, 200, 1000)
        # 8A < 10A，不触发过流
        hmp.update(8.0, 50.0, 500.0)
        self.assertEqual(hmp.state, MOTOR_STATE_RUNNING)
        # 11A > 10A，触发过流
        hmp.clear_fault()
        hmp.update(11.0, 50.0, 500.0)
        self.assertTrue(hmp.fault_flag & MOTOR_PROT_OVERCURRENT)

    def test_rapid_fault_recovery_cycle(self):
        """测试快速故障-恢复循环"""
        hmp = MotorProtectHandle()
        hmp.set_param(5.0, 80.0, 50.0, 3.0, 500, 100)  # 100ms冷却
        for cycle in range(10):
            hmp.update(6.0, 25.0, 500.0)  # 触发过流
            for _ in range(10):  # 冷却100ms
                hmp.update(1.0, 25.0, 500.0)
        self.assertEqual(hmp.fault_count, 10)


class TestMotorProtFaultPriority(unittest.TestCase):
    """故障优先级测试"""

    def test_overcurrent_before_overtemp(self):
        """测试过流优先于过温检测"""
        hmp = MotorProtectHandle()
        # 同时过流和过温
        hmp.update(6.0, 85.0, 500.0)
        # 过流先被检测到
        self.assertTrue(hmp.fault_flag & MOTOR_PROT_OVERCURRENT)
        self.assertEqual(hmp.get_fault_str(), "OVERCURRENT")

    def test_multiple_faults_accumulate(self):
        """测试故障标志位累积"""
        hmp = MotorProtectHandle()
        hmp.update(6.0, 25.0, 500.0)  # 过流
        self.assertTrue(hmp.fault_flag & MOTOR_PROT_OVERCURRENT)
        # 清除后再触发过温
        hmp.clear_fault()
        hmp.update(1.0, 85.0, 500.0)  # 过温
        self.assertTrue(hmp.fault_flag & MOTOR_PROT_OVERTEMP)


if __name__ == '__main__':
    unittest.main()
