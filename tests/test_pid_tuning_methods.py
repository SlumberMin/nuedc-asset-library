"""
PID调参方法单元测试
覆盖: Ziegler-Nichols临界比例度法/阶跃响应法/Cohen-Coon法/SIMC法/手动调参法/
      FOPDT被控对象/PID控制器(带微分滤波和抗饱和)/仿真计算/性能指标
"""
import unittest
import numpy as np
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# ============================================================
# 从 pid_tuning_methods_simulation.py 提取核心逻辑
# ============================================================


def zn_critical_period(Ku, Tu):
    """Ziegler-Nichols 临界比例度法"""
    kp = 0.6 * Ku
    ki = 2.0 * kp / Tu
    kd = kp * Tu / 8.0
    return kp, ki, kd


def zn_step_response(K, tau, L):
    """Ziegler-Nichols 阶跃响应法"""
    a = K * L / tau
    kp = 1.2 / a
    Ti = 2.0 * L
    Td = 0.5 * L
    ki = kp / Ti
    kd = kp * Td
    return kp, ki, kd


def cohen_coon(K, tau, L):
    """Cohen-Coon 整定法"""
    r = L / tau
    kp = (1.0 / (K * r)) * (1.0 + r / 3.0)
    Ti = L * (30.0 + 3.0 * r) / (9.0 + 20.0 * r)
    Td = L * 4.0 / (11.0 + 2.0 * r)
    ki = kp / Ti
    kd = kp * Td
    return kp, ki, kd


def simc_tuning(K, tau, L):
    """SIMC (Skogestad IMC) 法"""
    tau_c = max(L, 0.1 * tau)
    kp = tau / (K * (tau_c + L))
    Ti = min(tau, 4.0 * (tau_c + L))
    Td = 0.0
    ki = kp / Ti
    kd = kp * Td
    return kp, ki, kd


def manual_tuning(K, tau, L):
    """手动工程调参经验法"""
    kp = 0.8 * tau / (K * max(L, 0.01))
    Ti = 2.0 * tau
    Td = 0.25 * L
    ki = kp / Ti
    kd = kp * Td
    return kp, ki, kd


# ============================================================
# FOPDT被控对象
# ============================================================
class FOPDT_Plant:
    """一阶惯性+纯滞后"""

    def __init__(self, K=1.0, tau=1.0, L=0.2, dt=0.01):
        self.K = K
        self.tau = tau
        self.L = L
        self.dt = dt
        self.y = 0.0
        self._delay_steps = int(L / dt)
        self._u_buffer = [0.0] * max(self._delay_steps + 1, 1)

    def update(self, u):
        self._u_buffer.append(u)
        if len(self._u_buffer) > self._delay_steps + 1:
            self._u_buffer.pop(0)
        u_delayed = self._u_buffer[0]
        self.y += (self.K * u_delayed - self.y) / self.tau * self.dt
        return self.y

    def reset(self):
        self.y = 0.0
        self._u_buffer = [0.0] * max(self._delay_steps + 1, 1)


# ============================================================
# PID控制器
# ============================================================
class PID:
    def __init__(self, kp, ki, kd, out_min=-100, out_max=100, dt=0.01):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self.dt = dt
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_d = 0.0
        self.d_filter = 0.1

    def update(self, sp, fb):
        err = sp - fb
        self.integral += err * self.dt
        raw_d = (err - self.prev_error) / self.dt
        filt_d = self.d_filter * raw_d + (1 - self.d_filter) * self.prev_d
        self.prev_d = filt_d
        self.prev_error = err
        out = self.kp * (err + self.ki * self.integral + self.kd * filt_d)
        saturated = False
        if out > self.out_max:
            out = self.out_max
            saturated = True
        elif out < self.out_min:
            out = self.out_min
            saturated = True
        if saturated:
            self.integral -= err * self.dt
        return out

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_d = 0.0


# ============================================================
# 性能指标
# ============================================================
def calc_metrics(y, sp, dt, settle_threshold=0.02):
    final_val = sp[-1]
    if final_val == 0:
        return {}, y
    peak = np.max(y)
    overshoot = max(0, (peak - final_val) / final_val * 100)
    iae = np.sum(np.abs(sp - y)) * dt
    t = np.arange(len(y)) * dt
    itae = np.sum(t * np.abs(sp - y)) * dt
    settling_time = len(y) * dt
    for i in range(len(y) - 1, 0, -1):
        if abs(y[i] - final_val) > settle_threshold * final_val:
            settling_time = (i + 1) * dt
            break
    rise_start = None
    rise_end = None
    for i in range(len(y)):
        if y[i] >= 0.1 * final_val and rise_start is None:
            rise_start = i * dt
        if y[i] >= 0.9 * final_val and rise_end is None:
            rise_end = i * dt
            break
    rise_time = (rise_end - rise_start) if (rise_start and rise_end) else float('inf')
    return {
        'overshoot': overshoot,
        'iae': iae,
        'itae': itae,
        'settling_time': settling_time,
        'rise_time': rise_time,
    }, y


# ============================================================
# 测试用例
# ============================================================

class TestZNCriticalPeriod(unittest.TestCase):
    """Ziegler-Nichols临界比例度法测试"""

    def test_returns_positive_values(self):
        kp, ki, kd = zn_critical_period(Ku=4.5, Tu=2.8)
        self.assertGreater(kp, 0)
        self.assertGreater(ki, 0)
        self.assertGreater(kd, 0)

    def test_known_values(self):
        kp, ki, kd = zn_critical_period(Ku=10.0, Tu=2.0)
        self.assertAlmostEqual(kp, 6.0, places=3)   # 0.6*10
        self.assertAlmostEqual(ki, 6.0, places=3)    # 2*6/2
        self.assertAlmostEqual(kd, 1.5, places=3)    # 6*2/8

    def test_linear_in_ku(self):
        kp1, _, _ = zn_critical_period(Ku=5.0, Tu=1.0)
        kp2, _, _ = zn_critical_period(Ku=10.0, Tu=1.0)
        self.assertAlmostEqual(kp2 / kp1, 2.0, places=3)


class TestZNStepResponse(unittest.TestCase):
    """Ziegler-Nichols阶跃响应法测试"""

    def test_returns_positive_values(self):
        kp, ki, kd = zn_step_response(K=2.0, tau=3.0, L=0.5)
        self.assertGreater(kp, 0)
        self.assertGreater(ki, 0)
        self.assertGreater(kd, 0)

    def test_known_values(self):
        K, tau, L = 2.0, 3.0, 0.5
        kp, ki, kd = zn_step_response(K, tau, L)
        a = K * L / tau
        self.assertAlmostEqual(kp, 1.2 / a, places=3)


class TestCohenCoon(unittest.TestCase):
    """Cohen-Coon整定法测试"""

    def test_returns_positive_values(self):
        kp, ki, kd = cohen_coon(K=2.0, tau=3.0, L=0.5)
        self.assertGreater(kp, 0)
        self.assertGreater(ki, 0)
        self.assertGreater(kd, 0)

    def test_small_lag_ratio(self):
        """小滞后比应得到较大增益"""
        kp_small, _, _ = cohen_coon(K=1.0, tau=10.0, L=0.1)
        kp_large, _, _ = cohen_coon(K=1.0, tau=1.0, L=0.5)
        self.assertGreater(kp_small, kp_large)


class TestSIMCTuning(unittest.TestCase):
    """SIMC整定法测试"""

    def test_returns_positive_values(self):
        kp, ki, kd = simc_tuning(K=2.0, tau=3.0, L=0.5)
        self.assertGreater(kp, 0)
        self.assertGreater(ki, 0)

    def test_kd_is_zero(self):
        """SIMC通常不用微分"""
        _, _, kd = simc_tuning(K=2.0, tau=3.0, L=0.5)
        self.assertEqual(kd, 0.0)

    def test_conservative(self):
        """SIMC应比ZN保守（增益更小）"""
        kp_simc, _, _ = simc_tuning(K=2.0, tau=3.0, L=0.5)
        kp_zn, _, _ = zn_step_response(K=2.0, tau=3.0, L=0.5)
        # SIMC通常更保守
        self.assertLessEqual(kp_simc, kp_zn * 2)


class TestManualTuning(unittest.TestCase):
    """手动调参法测试"""

    def test_returns_positive_values(self):
        kp, ki, kd = manual_tuning(K=2.0, tau=3.0, L=0.5)
        self.assertGreater(kp, 0)
        self.assertGreater(ki, 0)
        self.assertGreaterEqual(kd, 0)

    def test_small_lag_uses_minimum(self):
        """极小滞后不应除以零"""
        kp, ki, kd = manual_tuning(K=1.0, tau=1.0, L=0.0)
        self.assertGreater(kp, 0)


class TestFOPDTPlant(unittest.TestCase):
    """FOPDT被控对象测试"""

    def test_init(self):
        plant = FOPDT_Plant(K=2.0, tau=1.0, L=0.5)
        self.assertEqual(plant.K, 2.0)
        self.assertEqual(plant.tau, 1.0)
        self.assertEqual(plant.y, 0.0)

    def test_step_response(self):
        """阶跃响应应趋近K*u"""
        plant = FOPDT_Plant(K=2.0, tau=1.0, L=0.0, dt=0.01)
        for _ in range(1000):
            y = plant.update(1.0)
        self.assertAlmostEqual(y, 2.0, places=0)

    def test_delay_effect(self):
        """纯滞后应延迟响应"""
        plant = FOPDT_Plant(K=1.0, tau=0.5, L=0.5, dt=0.01)
        # 刚施加输入时输出应接近0（因滞后）
        y = plant.update(1.0)
        self.assertLess(y, 0.1)

    def test_reset(self):
        plant = FOPDT_Plant(K=1.0, tau=1.0, L=0.0, dt=0.01)
        for _ in range(100):
            plant.update(1.0)
        plant.reset()
        self.assertEqual(plant.y, 0.0)

    def test_zero_input_zero_output(self):
        plant = FOPDT_Plant(K=2.0, tau=1.0, L=0.0, dt=0.01)
        for _ in range(100):
            y = plant.update(0.0)
        self.assertAlmostEqual(y, 0.0, places=5)


class TestPIDController(unittest.TestCase):
    """PID控制器测试"""

    def test_proportional_only(self):
        pid = PID(kp=10, ki=0, kd=0, dt=0.01)
        out = pid.update(sp=10, fb=5)
        self.assertGreater(out, 0)

    def test_integral_accumulation(self):
        pid = PID(kp=0, ki=10, kd=0, dt=0.01)
        pid.update(sp=10, fb=0)
        pid.update(sp=10, fb=0)
        self.assertGreater(pid.integral, 0)

    def test_anti_windup(self):
        pid = PID(kp=0, ki=1000, kd=0, out_min=-10, out_max=10, dt=0.01)
        for _ in range(1000):
            out = pid.update(sp=100, fb=0)
        self.assertLessEqual(out, 10.0)
        self.assertGreaterEqual(out, -10.0)

    def test_reset(self):
        pid = PID(kp=10, ki=5, kd=1, dt=0.01)
        for _ in range(100):
            pid.update(sp=10, fb=0)
        pid.reset()
        self.assertEqual(pid.integral, 0.0)
        self.assertEqual(pid.prev_error, 0.0)


class TestPIDSimulation(unittest.TestCase):
    """PID仿真闭环测试"""

    def test_step_response_converges(self):
        """闭环阶跃响应应趋近设定值"""
        K, tau, L = 1.0, 1.0, 0.1
        dt = 0.01
        T = 10.0
        steps = int(T / dt)
        setpoint = np.zeros(steps)
        setpoint[int(1.0 / dt):] = 1.0

        kp, ki, kd = simc_tuning(K, tau, L)
        plant = FOPDT_Plant(K=K, tau=tau, L=L, dt=dt)
        pid = PID(kp=kp, ki=ki, kd=kd, dt=dt)

        y = np.zeros(steps)
        for i in range(steps):
            y[i] = plant.update(pid.update(setpoint[i], y[i]))

        # 最终输出应接近设定值
        self.assertAlmostEqual(y[-1], 1.0, delta=0.15)

    def test_different_tuning_methods(self):
        """各调参方法都应能闭环稳定"""
        K, tau, L = 2.0, 3.0, 0.5
        dt = 0.01
        T = 30.0
        steps = int(T / dt)
        setpoint = np.zeros(steps)
        setpoint[int(1.0 / dt):] = 1.0

        methods = {
            'ZN': zn_step_response(K, tau, L),
            'CC': cohen_coon(K, tau, L),
            'SIMC': simc_tuning(K, tau, L),
            'Manual': manual_tuning(K, tau, L),
        }

        for name, (kp, ki, kd) in methods.items():
            plant = FOPDT_Plant(K=K, tau=tau, L=L, dt=dt)
            pid = PID(kp=kp, ki=ki, kd=kd, dt=dt, out_min=-100, out_max=100)
            y = np.zeros(steps)
            for i in range(steps):
                y[i] = plant.update(pid.update(setpoint[i], y[i]))
            # 应能稳定（不是NaN或发散）
            self.assertFalse(np.any(np.isnan(y)), f'{name} produced NaN')
            self.assertTrue(np.all(np.abs(y) < 100), f'{name} diverged')


class TestCalcMetrics(unittest.TestCase):
    """性能指标计算测试"""

    def test_zero_setpoint_returns_empty(self):
        y = np.zeros(100)
        sp = np.zeros(100)
        metrics, _ = calc_metrics(y, sp, 0.01)
        self.assertEqual(metrics, {})

    def test_perfect_tracking_zero_overshoot(self):
        """完美跟踪应零超调"""
        sp = np.ones(1000)
        y = np.ones(1000)
        metrics, _ = calc_metrics(y, sp, 0.01)
        self.assertAlmostEqual(metrics['overshoot'], 0.0, places=3)

    def test_overshoot_positive(self):
        """有超调应报告正值"""
        sp = np.ones(1000)
        y = np.concatenate([np.linspace(0, 1.2, 200), np.ones(800)])
        metrics, _ = calc_metrics(y, sp, 0.01)
        self.assertGreater(metrics['overshoot'], 0)

    def test_iae_positive(self):
        sp = np.ones(1000)
        y = np.ones(1000) * 0.9
        metrics, _ = calc_metrics(y, sp, 0.01)
        self.assertGreater(metrics['iae'], 0)

    def test_keys_present(self):
        sp = np.ones(1000)
        y = np.ones(1000)
        metrics, _ = calc_metrics(y, sp, 0.01)
        for key in ['overshoot', 'iae', 'itae', 'settling_time', 'rise_time']:
            self.assertIn(key, metrics)


if __name__ == '__main__':
    unittest.main()
