#!/usr/bin/env python3
"""
PID自动整定单元测试
覆盖: 继电反馈法初始化/步进/完成/复位、阶跃响应法初始化/步进/完成/复位、
      ZN规则计算、Cohen-Coon计算、超时错误、状态转换
注意: 使用纯 Python 模拟 C AutoTune 逻辑
"""

import sys
import os
import math
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# 枚举
AT_METHOD_RELAY = 0
AT_METHOD_ZN_STEP = 1
AT_METHOD_COHEN_COON = 2

AT_RULE_CLASSIC = 0
AT_RULE_PESSEN = 1
AT_RULE_SOME_OVERSHOOT = 2
AT_RULE_NO_OVERSHOOT = 3

AT_STATE_IDLE = 0
AT_STATE_RELAY_WAIT = 1
AT_STATE_RELAY_MEASURE = 2
AT_STATE_STEP_WAIT = 3
AT_STATE_STEP_MEASURE = 4
AT_STATE_DONE = 5
AT_STATE_ERROR = 6


def auto_tune_compute_zn(Ku, Tu, rule):
    """ZN规则查表"""
    if rule == AT_RULE_PESSEN:
        kp = 0.70 * Ku
        ti = 0.40 * Tu
        td = 0.15 * Tu
    elif rule == AT_RULE_SOME_OVERSHOOT:
        kp = 0.33 * Ku
        ti = 0.50 * Tu
        td = 0.33 * Tu
    elif rule == AT_RULE_NO_OVERSHOOT:
        kp = 0.20 * Ku
        ti = 0.50 * Tu
        td = 0.33 * Tu
    else:  # CLASSIC
        kp = 0.60 * Ku
        ti = 0.50 * Tu
        td = 0.125 * Tu

    ki = kp / ti if ti > 0 else 0.0
    kd = kp * td
    return kp, ki, kd


def auto_tune_compute_cohen_coon(K, L, T):
    """Cohen-Coon公式"""
    if K <= 0 or L <= 0 or T <= 0:
        return 0.0, 0.0, 0.0
    tau = L / T
    kp = (1.0 / K) * (1.0 + (1.0 / (3.0 * tau))) * (0.9316 / tau)
    ti = L * (3.33 * tau + 0.347) / (1.0 + 2.22 * tau)
    td = L * 0.473 * tau / (1.0 + 2.22 * tau)
    ki = kp / ti if ti > 0 else 0.0
    kd = kp * td
    return kp, ki, kd


class RelayAutoTuneSimulator:
    """继电反馈法模拟"""

    def __init__(self, relay_amp=1.0, hysteresis=0.5, offset=0.0,
                 dt=0.01, timeout=30.0, rule=AT_RULE_CLASSIC):
        self.relay_amplitude = abs(relay_amp)
        self.relay_hysteresis = abs(hysteresis)
        self.output_offset = offset
        self.dt = dt
        self.timeout = timeout
        self.rule = rule
        self.state = AT_STATE_RELAY_WAIT
        self.timer = 0.0
        self.relay_output = offset + relay_amp
        self.peak_max = -1e30
        self.peak_min = 1e30
        self.prev_input = 0.0
        self.zero_cross_time = 0.0
        self.rising_edge = False
        self.amplitude_sum = 0.0
        self.period_sum = 0.0
        self.period_count = 0
        self.result = {'kp': 0.0, 'ki': 0.0, 'kd': 0.0, 'Ku': 0.0, 'Tu': 0.0}

    def step(self, measurement):
        if self.state in (AT_STATE_DONE, AT_STATE_ERROR):
            return self.output_offset

        self.timer += self.dt

        if self.timer > self.timeout:
            self.state = AT_STATE_ERROR
            return self.output_offset

        # 继电滞环
        if measurement > self.relay_hysteresis:
            self.relay_output = self.output_offset - self.relay_amplitude
        elif measurement < -self.relay_hysteresis:
            self.relay_output = self.output_offset + self.relay_amplitude

        # 过零检测
        if self.prev_input < 0.0 and measurement >= 0.0:
            if self.rising_edge and self.zero_cross_time > 0.0:
                period = self.timer - self.zero_cross_time
                amplitude = (self.peak_max - self.peak_min) / 2.0
                if period > 0.0 and amplitude > 0.0:
                    self.period_sum += period
                    self.amplitude_sum += amplitude
                    self.period_count += 1
            self.zero_cross_time = self.timer
            self.rising_edge = True
            self.peak_max = measurement
            self.peak_min = measurement
        else:
            if measurement > self.peak_max:
                self.peak_max = measurement
            if measurement < self.peak_min:
                self.peak_min = measurement

        self.prev_input = measurement

        if self.period_count >= 4:
            Tu = self.period_sum / self.period_count
            a = self.amplitude_sum / self.period_count
            d = self.relay_amplitude
            Ku = (4.0 * d) / (math.pi * a)
            kp, ki, kd = auto_tune_compute_zn(Ku, Tu, self.rule)
            self.result = {'kp': kp, 'ki': ki, 'kd': kd, 'Ku': Ku, 'Tu': Tu}
            self.state = AT_STATE_DONE

        return self.relay_output

    def is_done(self):
        return self.state == AT_STATE_DONE

    def get_result(self):
        return self.result

    def reset(self):
        self.__init__(self.relay_amplitude, self.relay_hysteresis,
                      self.output_offset, self.dt, self.timeout, self.rule)


class StepAutoTuneSimulator:
    """阶跃响应法模拟"""

    def __init__(self, step_amp=10.0, dt=0.01, timeout=60.0,
                 method=AT_METHOD_ZN_STEP, rule=AT_RULE_CLASSIC):
        self.step_amplitude = step_amp
        self.dt = dt
        self.timeout = timeout
        self.method = method
        self.rule = rule
        self.state = AT_STATE_STEP_WAIT
        self.timer = 0.0
        self.baseline = 0.0
        self.steady_state = 0.0
        self.t632 = 0.0
        self.t283 = 0.0
        self.threshold_632 = 0.0
        self.threshold_283 = 0.0
        self.found_283 = False
        self.found_632 = False
        self.start_time = 0.0
        self.result = {'kp': 0.0, 'ki': 0.0, 'kd': 0.0, 'K_proc': 0.0, 'L': 0.0, 'T': 0.0}

    def step(self, measurement):
        if self.state in (AT_STATE_DONE, AT_STATE_ERROR):
            return self.baseline

        self.timer += self.dt

        if self.timer > self.timeout:
            self.state = AT_STATE_ERROR
            return self.baseline

        if self.state == AT_STATE_STEP_WAIT:
            self.baseline = measurement
            self.start_time = self.timer
            self.threshold_283 = self.baseline + 0.283 * self.step_amplitude
            self.threshold_632 = self.baseline + 0.632 * self.step_amplitude
            self.state = AT_STATE_STEP_MEASURE
            return self.baseline + self.step_amplitude

        if self.state == AT_STATE_STEP_MEASURE:
            if not self.found_283 and measurement >= self.threshold_283:
                self.t283 = self.timer - self.start_time
                self.found_283 = True
            if not self.found_632 and measurement >= self.threshold_632:
                self.t632 = self.timer - self.start_time
                self.found_632 = True

            if self.found_632:
                y_norm = (measurement - self.baseline) / self.step_amplitude
                if y_norm >= 0.99:
                    self.steady_state = measurement
                    K_proc = (self.steady_state - self.baseline) / self.step_amplitude
                    T_est = 1.5 * (self.t632 - self.t283)
                    L_est = max(0.0, self.t632 - T_est)
                    self.result['K_proc'] = K_proc
                    self.result['L'] = L_est
                    self.result['T'] = T_est
                    if self.method == AT_METHOD_COHEN_COON:
                        kp, ki, kd = auto_tune_compute_cohen_coon(K_proc, L_est, T_est)
                    else:
                        Ku = T_est / (K_proc * L_est) if (K_proc * L_est) > 0 else 0
                        Tu = 3.33 * L_est
                        kp, ki, kd = auto_tune_compute_zn(Ku, Tu, self.rule)
                        self.result['Ku'] = Ku
                        self.result['Tu'] = Tu
                    self.result['kp'] = kp
                    self.result['ki'] = ki
                    self.result['kd'] = kd
                    self.state = AT_STATE_DONE

            return self.baseline + self.step_amplitude

        return self.baseline

    def is_done(self):
        return self.state == AT_STATE_DONE

    def get_result(self):
        return self.result

    def reset(self):
        self.__init__(self.step_amplitude, self.dt, self.timeout,
                      self.method, self.rule)


# ── ZN规则测试 ──

class TestAutoTuneZN(unittest.TestCase):
    """ZN规则计算测试"""

    def test_classic_rule(self):
        kp, ki, kd = auto_tune_compute_zn(10.0, 1.0, AT_RULE_CLASSIC)
        self.assertAlmostEqual(kp, 6.0, places=3)   # 0.6*10
        self.assertAlmostEqual(ki, 12.0, places=3)   # 6.0/0.5
        self.assertAlmostEqual(kd, 0.75, places=3)   # 6.0*0.125

    def test_pessen_rule(self):
        kp, ki, kd = auto_tune_compute_zn(10.0, 1.0, AT_RULE_PESSEN)
        self.assertAlmostEqual(kp, 7.0, places=3)
        self.assertAlmostEqual(ki, 17.5, places=3)   # 7.0/0.4

    def test_some_overshoot_rule(self):
        kp, ki, kd = auto_tune_compute_zn(10.0, 1.0, AT_RULE_SOME_OVERSHOOT)
        self.assertAlmostEqual(kp, 3.3, places=3)

    def test_no_overshoot_rule(self):
        kp, ki, kd = auto_tune_compute_zn(10.0, 1.0, AT_RULE_NO_OVERSHOOT)
        self.assertAlmostEqual(kp, 2.0, places=3)

    def test_all_positive(self):
        for rule in [AT_RULE_CLASSIC, AT_RULE_PESSEN, AT_RULE_SOME_OVERSHOOT, AT_RULE_NO_OVERSHOOT]:
            kp, ki, kd = auto_tune_compute_zn(5.0, 0.5, rule)
            self.assertGreater(kp, 0)
            self.assertGreaterEqual(ki, 0)
            self.assertGreaterEqual(kd, 0)


class TestAutoTuneCohenCoon(unittest.TestCase):
    """Cohen-Coon计算测试"""

    def test_valid_params(self):
        kp, ki, kd = auto_tune_compute_cohen_coon(2.0, 0.5, 1.0)
        self.assertGreater(kp, 0)
        self.assertGreater(ki, 0)
        self.assertGreater(kd, 0)

    def test_zero_params_returns_zero(self):
        kp, ki, kd = auto_tune_compute_cohen_coon(0.0, 0.5, 1.0)
        self.assertEqual(kp, 0.0)
        kp, ki, kd = auto_tune_compute_cohen_coon(2.0, 0.0, 1.0)
        self.assertEqual(kp, 0.0)


# ── 继电反馈法测试 ──

class TestRelayAutoTuneInit(unittest.TestCase):
    """继电反馈法初始化测试"""

    def test_default_params(self):
        at = RelayAutoTuneSimulator(relay_amp=1.0, hysteresis=0.5)
        self.assertEqual(at.relay_amplitude, 1.0)
        self.assertEqual(at.relay_hysteresis, 0.5)
        self.assertEqual(at.state, AT_STATE_RELAY_WAIT)

    def test_negative_amp_absolute(self):
        """负幅值应取绝对值"""
        at = RelayAutoTuneSimulator(relay_amp=-2.0)
        self.assertEqual(at.relay_amplitude, 2.0)


class TestRelayAutoTuneStep(unittest.TestCase):
    """继电反馈法步进测试"""

    def test_returns_float(self):
        at = RelayAutoTuneSimulator()
        output = at.step(measurement=0.0)
        self.assertIsInstance(output, float)

    def test_timeout_error(self):
        """超时应进入错误状态"""
        at = RelayAutoTuneSimulator(dt=1.0, timeout=5.0)
        for _ in range(10):
            at.step(measurement=0.0)
        self.assertEqual(at.state, AT_STATE_ERROR)

    def test_done_state_after_oscillation(self):
        """经过足够振荡后应完成"""
        at = RelayAutoTuneSimulator(relay_amp=1.0, hysteresis=0.1, dt=0.001, timeout=100.0)
        for i in range(50000):
            t = i * 0.001
            y = math.sin(2 * math.pi * 1.0 * t)  # 1Hz振荡
            at.step(y)
            if at.is_done():
                break
        self.assertTrue(at.is_done())


class TestRelayAutoTuneResult(unittest.TestCase):
    """继电反馈法结果测试"""

    def test_result_contains_keys(self):
        at = RelayAutoTuneSimulator(relay_amp=1.0, hysteresis=0.1, dt=0.001, timeout=100.0)
        for i in range(50000):
            t = i * 0.001
            y = math.sin(2 * math.pi * 1.0 * t)
            at.step(y)
            if at.is_done():
                break
        if at.is_done():
            result = at.get_result()
            self.assertIn('kp', result)
            self.assertIn('ki', result)
            self.assertIn('kd', result)
            self.assertGreater(result['kp'], 0)


class TestRelayAutoTuneReset(unittest.TestCase):
    """继电反馈法复位测试"""

    def test_reset_clears_state(self):
        at = RelayAutoTuneSimulator()
        at.step(measurement=1.0)
        at.reset()
        self.assertEqual(at.state, AT_STATE_RELAY_WAIT)
        self.assertEqual(at.timer, 0.0)
        self.assertEqual(at.period_count, 0)


# ── 阶跃响应法测试 ──

class TestStepAutoTuneInit(unittest.TestCase):
    """阶跃响应法初始化测试"""

    def test_default_params(self):
        at = StepAutoTuneSimulator(step_amp=10.0, dt=0.01)
        self.assertEqual(at.step_amplitude, 10.0)
        self.assertEqual(at.state, AT_STATE_STEP_WAIT)


class TestStepAutoTuneStep(unittest.TestCase):
    """阶跃响应法步进测试"""

    def test_returns_float(self):
        at = StepAutoTuneSimulator(step_amp=10.0, dt=0.01)
        output = at.step(measurement=0.0)
        self.assertIsInstance(output, float)

    def test_first_step_sets_baseline(self):
        at = StepAutoTuneSimulator(step_amp=10.0, dt=0.01)
        at.step(measurement=5.0)
        self.assertEqual(at.baseline, 5.0)
        self.assertEqual(at.state, AT_STATE_STEP_MEASURE)

    def test_timeout_error(self):
        at = StepAutoTuneSimulator(step_amp=10.0, dt=1.0, timeout=5.0)
        at.step(measurement=0.0)  # 进入STEP_MEASURE
        for _ in range(10):
            at.step(measurement=0.0)
        self.assertEqual(at.state, AT_STATE_ERROR)

    def test_done_on_steady_state(self):
        """达到稳态后应完成"""
        at = StepAutoTuneSimulator(step_amp=10.0, dt=0.01, timeout=100.0)
        baseline = 0.0
        at.step(measurement=baseline)  # STEP_WAIT -> STEP_MEASURE
        # 模拟一阶响应
        for i in range(5000):
            t = i * 0.01
            y = baseline + 10.0 * (1.0 - math.exp(-t / 1.0))
            at.step(y)
            if at.is_done():
                break
        self.assertTrue(at.is_done())


class TestStepAutoTuneResult(unittest.TestCase):
    """阶跃响应法结果测试"""

    def test_result_params(self):
        at = StepAutoTuneSimulator(step_amp=10.0, dt=0.01, timeout=100.0)
        at.step(measurement=0.0)
        for i in range(5000):
            t = i * 0.01
            y = 10.0 * (1.0 - math.exp(-t / 1.0))
            at.step(y)
            if at.is_done():
                break
        if at.is_done():
            result = at.get_result()
            self.assertGreater(result['kp'], 0)
            self.assertGreater(result['L'], 0)
            self.assertGreater(result['T'], 0)


class TestStepAutoTuneReset(unittest.TestCase):
    """阶跃响应法复位测试"""

    def test_reset(self):
        at = StepAutoTuneSimulator()
        at.step(measurement=5.0)
        at.reset()
        self.assertEqual(at.state, AT_STATE_STEP_WAIT)
        self.assertEqual(at.timer, 0.0)


if __name__ == '__main__':
    unittest.main()
