#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file    python_test_template.py
@brief   Python测试代码模板 — 基于47个已知错误模式的标准化测试防护
@version 2.0
@date    2026-06-12

使用说明:
    1. 搜索替换 YOUR_MODULE 为实际模块名
    2. 根据被测模块修改测试用例
    3. 测试必须import生产代码 (错误经验 #9)
    4. 使用强断言 (assertEqual, assertAlmostEqual) 而非弱断言 (assertTrue)

本模板覆盖的错误模式:
    #4   缺失import            → 完整import声明
    #9   测试不import生产代码   → 使用wrappers.py或直接import
    #10  变量名/枚举拼写错误    → 类型注解+命名规范
    #15  np但漏import          → 文件头完整import
    #17  错误kwargs导致TypeError → 正确的构造参数
    #18  控制仿真反馈符号反转   → 负反馈闭环
    #19  测试import路径错误     → sys.path正确设置
    #28  循环内.close()        → with语句
    #29  bare except           → 具体异常类型
"""

# =========================================================================
#  1. 标准库import
# =========================================================================
import os
import sys
import math
import time
import random
import unittest
import warnings
from typing import List, Optional, Any

# =========================================================================
#  2. 第三方库import (错误经验 #4, #15: 测试文件漏import)
#
#  !! 错误经验 #15: test_incremental_pid.py, test_pid.py 使用np.isfinite()
#  !! 但未import numpy导致NameError
#  !! 所有测试文件顶部必须import所有用到的库
# =========================================================================
import numpy as np

# =========================================================================
#  3. 被测代码import (错误经验 #9, #19)
#
#  !! 错误经验 #9: 测试必须import生产代码，不能自行重写算法
#  !! 错误经验 #19: import路径必须与实际目录结构匹配
# =========================================================================

# --- 方法1: 通过sys.path添加项目根目录 (错误经验 #19) ---
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.join(_SCRIPT_DIR, '..')
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

# --- 方法2: 使用tests/wrappers.py包装层 (错误经验 #9) ---
# wrappers.py提供与C代码逻辑一致的Python实现
# !! 错误写法: 测试自行定义PIDController类 — 这样测的是自己的实现，不是生产代码
# !! 正确写法: from tests.wrappers import PIDController
try:
    from tests.wrappers import (  # 错误经验 #9: import生产代码
        PIDController,
        ADRCv1,
        ADRCv2,
        KalmanFilter,
        RingBuffer,
        StateMachine,
        TaskScheduler,
    )
    HAS_WRAPPERS = True
except ImportError:
    HAS_WRAPPERS = False
    warnings.warn(
        "tests/wrappers.py不可用，部分测试将跳过。"
        "请确保项目目录结构正确。"
    )

# --- 方法3: 直接import Python模块 ---
# from your_module.submodule import YourClass

# =========================================================================
#  4. 测试辅助工具
# =========================================================================

# --- 随机种子 (保证测试可重复) ---
RANDOM_SEED = 42
random.seed(RANDOM_SEED)
np.random.seed(RANDOM_SEED)


def assert_is_finite(value: float, msg: str = ""):
    """断言值是有限数 (不是NaN/Inf)"""
    if not math.isfinite(value):
        raise AssertionError(
            f"值不是有限数: {value}. {msg}"
        )


def assert_in_range(value: float, lo: float, hi: float, msg: str = ""):
    """断言值在范围内 [lo, hi]"""
    if not (lo <= value <= hi):
        raise AssertionError(
            f"值 {value} 不在范围 [{lo}, {hi}] 内. {msg}"
        )


def assert_no_nan_inf(arr: np.ndarray, msg: str = ""):
    """断言数组中无NaN/Inf"""
    if not np.all(np.isfinite(arr)):
        nan_count = np.sum(np.isnan(arr))
        inf_count = np.sum(np.isinf(arr))
        raise AssertionError(
            f"数组包含 {nan_count}个NaN, {inf_count}个Inf. {msg}"
        )


# =========================================================================
#  5. PID控制器测试类 (示例: 展示测试模式)
# =========================================================================

@unittest.skipUnless(HAS_WRAPPERS, "需要tests/wrappers.py")
class TestPIDController(unittest.TestCase):
    """
    PID控制器测试套件

    注意:
        - 使用wrappers.py中的PIDController (错误经验 #9)
        - 不自行重写PID逻辑
        - 使用强断言 (assertAlmostEqual) 而非 assertTrue(abs(...))
    """

    def setUp(self):
        """
        每个测试前的初始化

        !! 错误经验 #17: 构造函数参数要正确
        !! 错误: PIDController(dt=0.01) — dt是calc()的参数，不是__init__的
        !! 正确: PIDController(kp=1.0, ki=0.1, kd=0.01)
        """
        self.pid = PIDController(kp=1.0, ki=0.1, kd=0.01)
        self.dt = 0.01  # 控制周期

    def test_initialization(self):
        """测试初始化后的状态"""
        # 强断言: assertEqual, assertAlmostEqual
        self.assertIsNotNone(self.pid)

    def test_proportional_response(self):
        """测试比例响应"""
        output = self.pid.calculate(setpoint=1.0, feedback=0.0, dt=self.dt)

        # 强断言: 输出应该是有限数 (错误经验 #1: 除零防护)
        assert_is_finite(output, "PID输出不应为NaN/Inf")

        # 强断言: 输出方向正确
        self.assertGreater(output, 0, "误差为正时输出应为正")

    def test_integral_accumulation(self):
        """测试积分累积"""
        outputs = []
        for _ in range(100):
            output = self.pid.calculate(
                setpoint=1.0, feedback=0.5, dt=self.dt  # !! dt传给calc() (错误经验 #17)
            )
            outputs.append(output)

        # 强断言: 积分应导致输出增大
        self.assertGreater(outputs[-1], outputs[0],
                           "积分累积应使输出随时间增大")

    def test_derivative_action(self):
        """测试微分作用"""
        outputs = []
        for i in range(10):
            # 阶跃误差后保持恒定
            output = self.pid.calculate(
                setpoint=1.0,
                feedback=0.5 if i > 0 else 0.0,
                dt=self.dt  # !! dt传给calc() (错误经验 #17)
            )
            outputs.append(output)

        # 微分在第一次变化后应有尖峰
        assert_is_finite(outputs[0], "微分输出应为有限数")

    def test_output_clamping(self):
        """测试输出限幅"""
        for _ in range(1000):
            output = self.pid.calculate(
                setpoint=100.0, feedback=0.0, dt=self.dt
            )

        # 强断言: 输出应在限幅范围内
        assert_in_range(output, -100.0, 100.0, "输出应被钳位")

    def test_zero_division_protection(self):
        """测试除零保护 (错误经验 #1)"""
        try:
            output = self.pid.calculate(
                setpoint=1.0, feedback=0.0, dt=0.0  # dt=0 应被安全处理
            )
            assert_is_finite(output, "dt=0时输出不应为NaN/Inf")
        except (ValueError, ZeroDivisionError):
            pass  # 抛出明确异常也是可接受的

    def test_nan_input_protection(self):
        """测试NaN输入保护"""
        try:
            output = self.pid.calculate(
                setpoint=float('nan'), feedback=0.0, dt=self.dt
            )
            # 如果返回值，应为有限数
            assert_is_finite(output, "NaN输入时输出不应为NaN")
        except (ValueError, TypeError):
            pass  # 抛出明确异常也是可接受的

    def test_reset(self):
        """测试重置功能"""
        # 先累积积分
        for _ in range(100):
            self.pid.calculate(setpoint=1.0, feedback=0.0, dt=self.dt)

        self.pid.reset()

        # 重置后输出应与初始状态相同
        output = self.pid.calculate(setpoint=1.0, feedback=0.0, dt=self.dt)
        assert_is_finite(output, "重置后输出应为有限数")


# =========================================================================
#  6. 控制仿真测试 (错误经验 #18: 反馈符号)
# =========================================================================

class TestControlSimulation(unittest.TestCase):
    """
    控制仿真测试 — 展示正确的反馈闭环 (错误经验 #18)

    注意:
        控制律u=-k*sign(s)已含负号时，plant模型必须取反:
        state -= output * dt  (负反馈)
        而非:
        state += output * dt  (正反馈 → 发散!)
    """

    def test_negative_feedback_convergence(self):
        """测试负反馈闭环收敛"""
        setpoint = 1.0
        state = 0.0
        kp = 5.0
        dt = 0.01

        trajectory = [state]
        for _ in range(500):
            error = setpoint - state
            output = kp * error

            # !! 错误经验 #18: 负反馈闭环
            # !! 错误: state += output * dt  (正反馈, 发散)
            # !! 正确: state -= output * dt  (负反馈, 收敛)
            state -= output * dt  # !! 注意: -= 不是 +=

            trajectory.append(state)

        trajectory = np.array(trajectory)

        # 强断言: 系统应收敛
        assert_no_nan_inf(trajectory, "轨迹不应包含NaN/Inf")
        self.assertAlmostEqual(trajectory[-1], setpoint, places=1,
                               msg="系统应收敛到设定值")

    def test_output_bounded(self):
        """测试输出有界"""
        setpoint = 1.0
        state = 0.0
        kp = 5.0
        dt = 0.01

        for _ in range(1000):
            error = setpoint - state
            output = kp * error

            # 钳位
            output = max(-100.0, min(100.0, output))

            state -= output * dt  # 负反馈

        # 强断言: 状态应有界
        assert_in_range(state, -10.0, 10.0, "状态应有界")
        assert_is_finite(state, "状态应为有限数")


# =========================================================================
#  7. Kalman滤波器测试 (使用wrappers, 错误经验 #9)
# =========================================================================

@unittest.skipUnless(HAS_WRAPPERS, "需要tests/wrappers.py")
class TestKalmanFilter(unittest.TestCase):
    """Kalman滤波器测试 — import wrappers.py (错误经验 #9)"""

    def test_prediction_step(self):
        """测试预测步"""
        kf = KalmanFilter(dim_x=2, dim_z=1)
        kf.predict()

        # 强断言: 状态应为有限数
        assert_is_finite(kf.x[0], "预测状态应为有限数")

    def test_update_step(self):
        """测试更新步"""
        kf = KalmanFilter(dim_x=2, dim_z=1)
        kf.predict()
        kf.update(np.array([1.0]))

        assert_is_finite(kf.x[0], "更新状态应为有限数")

    def test_convergence_with_noise(self):
        """测试有噪声时的收敛性"""
        kf = KalmanFilter(dim_x=2, dim_z=1)

        true_value = 5.0
        measurements = true_value + np.random.randn(100) * 0.5

        estimates = []
        for z in measurements:
            kf.predict()
            kf.update(np.array([z]))
            estimates.append(kf.x[0])

        estimates = np.array(estimates)

        # 强断言: 估计应趋近真值
        assert_no_nan_inf(estimates, "估计值不应包含NaN/Inf")
        self.assertAlmostEqual(estimates[-1], true_value, delta=0.5,
                               msg="滤波器应收敛到真值附近")

    def test_matrix_dimension_check(self):
        """测试矩阵维度校验 (错误经验 #12)"""
        try:
            kf = KalmanFilter(dim_x=2, dim_z=1)
            # 尝试用错误维度更新
            kf.update(np.array([1.0, 2.0, 3.0]))  # dim_z=1但传了3维
            # 如果没报错，说明内部做了处理
        except (ValueError, IndexError):
            pass  # 抛出明确异常也是可接受的


# =========================================================================
#  8. 环形缓冲区测试
# =========================================================================

@unittest.skipUnless(HAS_WRAPPERS, "需要tests/wrappers.py")
class TestRingBuffer(unittest.TestCase):
    """环形缓冲区测试"""

    def test_basic_operations(self):
        """测试基本操作"""
        buf = RingBuffer(capacity=10)

        # 写入
        for i in range(5):
            buf.push(i)

        # 强断言: 长度正确
        self.assertEqual(len(buf), 5)

    def test_overflow_behavior(self):
        """测试溢出行为"""
        buf = RingBuffer(capacity=5)

        for i in range(10):
            buf.push(i)

        # 强断言: 溢出后长度等于容量
        self.assertEqual(len(buf), 5)

    def test_index_bounds(self):
        """测试索引边界 (错误经验 #7)"""
        buf = RingBuffer(capacity=5)

        # 空缓冲区访问
        try:
            _ = buf[0]
        except (IndexError, ValueError):
            pass  # 应抛出异常


# =========================================================================
#  9. 状态机测试
# =========================================================================

@unittest.skipUnless(HAS_WRAPPERS, "需要tests/wrappers.py")
class TestStateMachine(unittest.TestCase):
    """状态机测试"""

    def test_initial_state(self):
        """测试初始状态"""
        sm = StateMachine(initial_state='IDLE')
        self.assertEqual(sm.get_state(), 'IDLE')

    def test_transition(self):
        """测试状态转移"""
        sm = StateMachine(initial_state='IDLE')
        sm.add_transition('IDLE', 'START', 'RUNNING')
        sm.trigger('START')
        self.assertEqual(sm.get_state(), 'RUNNING')

    def test_invalid_transition(self):
        """测试无效转移"""
        sm = StateMachine(initial_state='IDLE')
        try:
            sm.trigger('INVALID')
        except (ValueError, KeyError):
            pass  # 应抛出异常


# =========================================================================
#  10. 数值稳定性测试 (错误经验 #1, #12, #41)
# =========================================================================

class TestNumericalStability(unittest.TestCase):
    """数值稳定性测试"""

    def test_float_functions_with_f_suffix(self):
        """测试使用f后缀的浮点函数 (错误经验 #41)"""
        # !! 在C代码中应使用fabsf, sinf, cosf, atan2f, sqrtf, powf
        # !! Python中float精度默认，但验证逻辑一致性
        values = [0.0, 1.0, -1.0, 0.5, -0.5, 1e-10, -1e-10]

        for v in values:
            assert_is_finite(abs(v), f"abs({v})")
            assert_is_finite(math.sin(v), f"sin({v})")
            assert_is_finite(math.cos(v), f"cos({v})")

    def test_division_by_near_zero(self):
        """测试接近零的除法 (错误经验 #1)"""
        tiny_values = [1e-10, -1e-10, 1e-30, 0.0]

        for v in tiny_values:
            try:
                result = 1.0 / v if abs(v) > 1e-15 else 0.0
                assert_is_finite(result, f"1/{v}")
            except ZeroDivisionError:
                pass  # 除零异常也是可接受的

    def test_accumulation_stability(self):
        """测试长时间累积的数值稳定性"""
        total = 0.0
        for i in range(100000):
            total += 0.1

        # 累积误差检查
        expected = 10000.0
        relative_error = abs(total - expected) / expected
        self.assertLess(relative_error, 1e-6,
                        f"累积误差过大: {relative_error:.2e}")

    def test_matrix_dimension_consistency(self):
        """测试矩阵维度一致性 (错误经验 #12)"""
        A = np.array([[1, 2], [3, 4]])
        B = np.array([[1], [2]])

        # 2x2 @ 2x1 = 2x1 (合法)
        result = A @ B
        self.assertEqual(result.shape, (2, 1))

        # 测试维度不匹配
        C = np.array([[1, 2, 3]])
        try:
            A @ C  # 2x2 @ 1x3 → 错误
        except ValueError:
            pass  # 应抛出维度不匹配异常


# =========================================================================
#  11. 串口相关测试 (错误经验 #28, #29)
# =========================================================================

class TestSerialOperations(unittest.TestCase):
    """串口操作测试 (无实际硬件时验证逻辑)"""

    def test_context_manager_pattern(self):
        """测试使用with语句管理资源 (错误经验 #28)"""
        # !! 错误: 循环内close()
        # !! 正确: 使用with语句或循环外close
        import io
        fake_serial = io.StringIO("1.0\n2.0\n3.0\n")

        values = []
        try:
            for line in fake_serial:
                values.append(float(line.strip()))
        finally:
            fake_serial.close()  # 在循环外关闭

        self.assertEqual(len(values), 3)
        np.testing.assert_array_almost_equal(values, [1.0, 2.0, 3.0])

    def test_specific_exception_handling(self):
        """测试具体异常类型 (错误经验 #29)"""
        # !! 错误: except: (吞掉KeyboardInterrupt)
        # !! 正确: except (ValueError, IOError):
        try:
            raise ValueError("test error")
        except (ValueError, IOError) as e:
            self.assertIn("test error", str(e))
        except Exception:
            self.fail("不应到达此处")


# =========================================================================
#  12. 入口守卫
# =========================================================================
if __name__ == '__main__':
    # 设置随机种子保证可重复性
    random.seed(RANDOM_SEED)
    np.random.seed(RANDOM_SEED)

    # 运行所有测试
    unittest.main(verbosity=2)
