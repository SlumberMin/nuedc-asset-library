#!/usr/bin/env python3
"""
输入整形器单元测试
覆盖: ZV/ZVD/ZVDD/EI脉冲计算、初始化、延迟线、
      整形输出、复位、延迟时间、脉冲归一化
注意: 使用纯 Python 模拟 C InputShaper 逻辑
"""

import sys
import os
import math
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

IS_MAX_IMPULSES = 4
PI = 3.141592653589793


def _calc_params(freq_hz, zeta):
    """计算衰减比K和半周期T_half"""
    wn = 2.0 * PI * freq_hz
    wd = wn * math.sqrt(1.0 - zeta * zeta)
    K = math.exp(-zeta * PI / math.sqrt(1.0 - zeta * zeta))
    T_half = PI / wd
    return K, T_half, wd


def compute_zv(freq_hz, zeta):
    """ZV整形器: 两脉冲"""
    K, T_half, _ = _calc_params(freq_hz, zeta)
    impulses = [
        (1.0 / (1.0 + K), 0.0),
        (K / (1.0 + K), T_half)
    ]
    return impulses


def compute_zvd(freq_hz, zeta):
    """ZVD整形器: 三脉冲"""
    K, T_half, _ = _calc_params(freq_hz, zeta)
    K2 = K * K
    denom = 1.0 + 2.0 * K + K2
    impulses = [
        (1.0 / denom, 0.0),
        (2.0 * K / denom, T_half),
        (K2 / denom, 2.0 * T_half)
    ]
    return impulses


def compute_zvdd(freq_hz, zeta):
    """ZVDD整形器: 四脉冲"""
    K, T_half, _ = _calc_params(freq_hz, zeta)
    K2 = K * K
    K3 = K2 * K
    denom = 1.0 + 3.0 * K + 3.0 * K2 + K3
    impulses = [
        (1.0 / denom, 0.0),
        (3.0 * K / denom, T_half),
        (3.0 * K2 / denom, 2.0 * T_half),
        (K3 / denom, 3.0 * T_half)
    ]
    return impulses


def compute_ei(freq_hz, zeta, allowed_vib=0.05):
    """EI整形器: 三脉冲"""
    K, T_half, _ = _calc_params(freq_hz, zeta)
    A = max(0.001, min(0.5, allowed_vib))
    K2 = K * K
    denom = 1.0 + 2.0 * K * A + K2
    impulses = [
        (1.0 / denom, 0.0),
        (2.0 * K * A / denom, T_half),
        (K2 / denom, 2.0 * T_half)
    ]
    return impulses


class InputShaperSimulator:
    """输入整形器模拟"""

    def __init__(self, shaper_type, freq_hz, zeta, dt, buffer_size=1024):
        self.type = shaper_type
        self.freq = freq_hz
        self.zeta = zeta
        self.dt = dt

        # 计算脉冲序列
        if shaper_type == 'ZV':
            imp = compute_zv(freq_hz, zeta)
        elif shaper_type == 'ZVD':
            imp = compute_zvd(freq_hz, zeta)
        elif shaper_type == 'ZVDD':
            imp = compute_zvdd(freq_hz, zeta)
        elif shaper_type == 'EI':
            imp = compute_ei(freq_hz, zeta)
        else:
            imp = compute_zv(freq_hz, zeta)

        self.impulses = imp
        self.num_impulses = len(imp)

        # 延迟缓冲区
        max_time = max(t for _, t in imp)
        self.delay_samples = int(max_time / dt) + 1
        self.buffer = [0.0] * buffer_size
        self.buffer_size = buffer_size
        self.write_idx = 0
        self.output = 0.0
        self.initialized = True

    def update(self, reference):
        if not self.initialized:
            return reference
        self.buffer[self.write_idx] = reference
        shaped = 0.0
        for amp, t in self.impulses:
            delay = int(t / self.dt + 0.5)
            read_idx = self.write_idx - delay
            if read_idx < 0:
                read_idx += self.buffer_size
            shaped += amp * self.buffer[read_idx]
        self.write_idx += 1
        if self.write_idx >= self.buffer_size:
            self.write_idx = 0
        self.output = shaped
        return shaped

    def get_output(self):
        return self.output

    def get_delay(self):
        if not self.initialized or self.num_impulses == 0:
            return 0.0
        return max(t for _, t in self.impulses)

    def reset(self):
        self.buffer = [0.0] * self.buffer_size
        self.write_idx = 0
        self.output = 0.0


# ── 脉冲计算测试 ──

class TestZVImpulses(unittest.TestCase):
    """ZV脉冲计算测试"""

    def test_zv_two_impulses(self):
        imp = compute_zv(2.0, 0.0)
        self.assertEqual(len(imp), 2)

    def test_zv_amplitudes_sum_to_one(self):
        """脉冲幅值之和应为1"""
        imp = compute_zv(2.0, 0.0)
        total = sum(a for a, _ in imp)
        self.assertAlmostEqual(total, 1.0, places=5)

    def test_zv_first_impulse_at_zero(self):
        imp = compute_zv(2.0, 0.0)
        self.assertAlmostEqual(imp[0][1], 0.0, places=5)

    def test_zv_amplitudes_positive(self):
        imp = compute_zv(2.0, 0.1)
        for amp, _ in imp:
            self.assertGreater(amp, 0.0)

    def test_zv_second_at_half_period(self):
        """第二个脉冲应在半周期处"""
        imp = compute_zv(2.0, 0.0)  # freq=2Hz, zeta=0
        # T_half = pi/(2*pi*2) = 0.25
        self.assertAlmostEqual(imp[1][1], 0.25, delta=0.01)


class TestZVDImpulses(unittest.TestCase):
    """ZVD脉冲计算测试"""

    def test_zvd_three_impulses(self):
        imp = compute_zvd(2.0, 0.0)
        self.assertEqual(len(imp), 3)

    def test_zvd_amplitudes_sum_to_one(self):
        imp = compute_zvd(2.0, 0.0)
        total = sum(a for a, _ in imp)
        self.assertAlmostEqual(total, 1.0, places=5)

    def test_zvd_amplitudes_positive(self):
        imp = compute_zvd(2.0, 0.1)
        for amp, _ in imp:
            self.assertGreater(amp, 0.0)


class TestZVDDImpulses(unittest.TestCase):
    """ZVDD脉冲计算测试"""

    def test_zvdd_four_impulses(self):
        imp = compute_zvdd(2.0, 0.0)
        self.assertEqual(len(imp), 4)

    def test_zvdd_amplitudes_sum_to_one(self):
        imp = compute_zvdd(2.0, 0.0)
        total = sum(a for a, _ in imp)
        self.assertAlmostEqual(total, 1.0, places=5)


class TestEIImpulses(unittest.TestCase):
    """EI脉冲计算测试"""

    def test_ei_three_impulses(self):
        imp = compute_ei(2.0, 0.0)
        self.assertEqual(len(imp), 3)

    def test_ei_amplitudes_sum_to_one(self):
        imp = compute_ei(2.0, 0.0)
        total = sum(a for a, _ in imp)
        self.assertAlmostEqual(total, 1.0, places=5)

    def test_ei_allowed_vib_clamped(self):
        """allowed_vib应被限制"""
        imp1 = compute_ei(2.0, 0.0, allowed_vib=0.0)
        imp2 = compute_ei(2.0, 0.0, allowed_vib=1.0)
        # 都应返回有效结果
        self.assertEqual(len(imp1), 3)
        self.assertEqual(len(imp2), 3)


# ── 整形器实例测试 ──

class TestInputShaperInit(unittest.TestCase):
    """初始化测试"""

    def test_zv_init(self):
        shaper = InputShaperSimulator('ZV', 2.0, 0.0, 0.001)
        self.assertTrue(shaper.initialized)
        self.assertEqual(shaper.num_impulses, 2)

    def test_zvd_init(self):
        shaper = InputShaperSimulator('ZVD', 2.0, 0.0, 0.001)
        self.assertEqual(shaper.num_impulses, 3)

    def test_zvdd_init(self):
        shaper = InputShaperSimulator('ZVDD', 2.0, 0.0, 0.001)
        self.assertEqual(shaper.num_impulses, 4)

    def test_ei_init(self):
        shaper = InputShaperSimulator('EI', 2.0, 0.0, 0.001)
        self.assertEqual(shaper.num_impulses, 3)

    def test_delay_samples_computed(self):
        shaper = InputShaperSimulator('ZV', 2.0, 0.0, 0.001)
        self.assertGreater(shaper.delay_samples, 0)


class TestInputShaperUpdate(unittest.TestCase):
    """整形计算测试"""

    def test_returns_float(self):
        shaper = InputShaperSimulator('ZV', 2.0, 0.0, 0.001)
        result = shaper.update(10.0)
        self.assertIsInstance(result, float)

    def test_step_input_shaped(self):
        """阶跃输入应被整形(产生延迟)"""
        shaper = InputShaperSimulator('ZV', 2.0, 0.0, 0.01)
        # 阶跃输入
        outputs = []
        for _ in range(100):
            outputs.append(shaper.update(10.0))
        # 输出应逐渐增加到10.0
        self.assertAlmostEqual(outputs[-1], 10.0, delta=0.5)

    def test_shaping_introduces_delay(self):
        """整形应引入延迟"""
        shaper = InputShaperSimulator('ZV', 2.0, 0.0, 0.01)
        # 阶跃输入
        outputs = []
        for _ in range(100):
            outputs.append(shaper.update(10.0))
        # 在初始阶段输出应小于10(有延迟)
        self.assertLess(outputs[0], 10.0)


class TestInputShaperGetDelay(unittest.TestCase):
    """延迟时间测试"""

    def test_zv_delay(self):
        shaper = InputShaperSimulator('ZV', 2.0, 0.0, 0.001)
        delay = shaper.get_delay()
        # ZV延迟 = T_half = pi/(2*pi*2) = 0.25s
        self.assertAlmostEqual(delay, 0.25, delta=0.01)

    def test_zvd_longer_than_zv(self):
        """ZVD延迟应大于ZV"""
        shaper_zv = InputShaperSimulator('ZV', 2.0, 0.0, 0.001)
        shaper_zvd = InputShaperSimulator('ZVD', 2.0, 0.0, 0.001)
        self.assertGreater(shaper_zvd.get_delay(), shaper_zv.get_delay())

    def test_zvdd_longest(self):
        """ZVDD延迟应最长"""
        shaper_zvd = InputShaperSimulator('ZVD', 2.0, 0.0, 0.001)
        shaper_zvdd = InputShaperSimulator('ZVDD', 2.0, 0.0, 0.001)
        self.assertGreater(shaper_zvdd.get_delay(), shaper_zvd.get_delay())


class TestInputShaperReset(unittest.TestCase):
    """复位测试"""

    def test_reset_clears_buffer(self):
        shaper = InputShaperSimulator('ZV', 2.0, 0.0, 0.001)
        for _ in range(10):
            shaper.update(10.0)
        shaper.reset()
        for v in shaper.buffer:
            self.assertEqual(v, 0.0)

    def test_reset_clears_output(self):
        shaper = InputShaperSimulator('ZV', 2.0, 0.0, 0.001)
        shaper.update(10.0)
        shaper.reset()
        self.assertEqual(shaper.output, 0.0)

    def test_reset_resets_index(self):
        shaper = InputShaperSimulator('ZV', 2.0, 0.0, 0.001)
        shaper.update(10.0)
        shaper.reset()
        self.assertEqual(shaper.write_idx, 0)


class TestInputShaperGetOutput(unittest.TestCase):
    """输出获取测试"""

    def test_get_output(self):
        shaper = InputShaperSimulator('ZV', 2.0, 0.0, 0.001)
        shaper.update(10.0)
        self.assertEqual(shaper.get_output(), shaper.output)


class TestInputShaperDamping(unittest.TestCase):
    """阻尼比测试"""

    def test_nonzero_damping(self):
        """非零阻尼应正常工作"""
        shaper = InputShaperSimulator('ZV', 2.0, 0.1, 0.001)
        result = shaper.update(10.0)
        self.assertIsNotNone(result)

    def test_higher_damping_changes_impulses(self):
        """高阻尼应改变脉冲参数"""
        imp1 = compute_zv(2.0, 0.0)
        imp2 = compute_zv(2.0, 0.3)
        # 幅值比例应不同
        ratio1 = imp1[0][0] / imp1[1][0]
        ratio2 = imp2[0][0] / imp2[1][0]
        self.assertNotAlmostEqual(ratio1, ratio2, places=2)


class TestInputShaperFrequency(unittest.TestCase):
    """频率测试"""

    def test_higher_freq_shorter_delay(self):
        """高频应有更短延迟"""
        shaper_low = InputShaperSimulator('ZV', 1.0, 0.0, 0.001)
        shaper_high = InputShaperSimulator('ZV', 5.0, 0.0, 0.001)
        self.assertGreater(shaper_low.get_delay(), shaper_high.get_delay())


if __name__ == '__main__':
    unittest.main()
