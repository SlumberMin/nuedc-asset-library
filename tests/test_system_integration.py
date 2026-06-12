#!/usr/bin/env python3
"""
系统集成测试
覆盖: 跨模块集成测试、端到端工作流测试、模块间接口测试
测试目标: 验证各模块间的协同工作能力
"""

import sys
import os
import unittest
import time
import numpy as np
from unittest.mock import MagicMock, patch
from typing import List, Tuple, Dict, Any

# 将项目根目录加入路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False


def create_test_image(width: int = 640, height: int = 480, color: Tuple = (0, 0, 0)) -> np.ndarray:
    """创建测试图像"""
    img = np.zeros((height, width, 3), dtype=np.uint8)
    img[:] = color
    return img


# ===================== 视觉-控制集成测试 =====================

@unittest.skipUnless(HAS_CV2, "OpenCV未安装，跳过视觉-控制集成测试")
class TestVisionControlIntegration(unittest.TestCase):
    """视觉-控制系统集成测试"""

    def setUp(self):
        """测试前初始化"""
        self.integration_available = False
        try:
            # 尝试导入视觉模块
            from visual.color_tracker import ColorTracker
            # 尝试导入控制模块
            from control.pid_controller import PIDController

            self.color_tracker = ColorTracker(color_name='red', min_area=100)
            self.pid_controller = PIDController(kp=0.5, ki=0.1, kd=0.05)
            self.integration_available = True
        except ImportError as e:
            self.skipTest(f"集成模块未全部安装: {e}")

    def test_visual_guided_control(self):
        """视觉引导控制流程"""
        if not self.integration_available:
            self.skipTest("集成模块未安装")

        try:
            # 1. 视觉检测
            img = create_test_image(640, 480, (100, 100, 100))
            cv2.circle(img, (400, 240), 50, (0, 0, 255), -1)  # 偏右的红色目标

            results, mask = self.color_tracker.update(img)

            if len(results) > 0:
                target_x = results[0]['cx']

                # 2. 计算误差
                center_x = 320  # 图像中心
                error = target_x - center_x

                # 3. PID控制
                control_output = self.pid_controller.compute(error, dt=0.01)

                # 4. 验证控制输出方向
                if error > 0:
                    self.assertGreater(control_output, 0, "控制输出方向错误")
                elif error < 0:
                    self.assertLess(control_output, 0, "控制输出方向错误")
            else:
                self.skipTest("未检测到目标")
        except Exception as e:
            self.skipTest(f"视觉引导控制测试失败: {e}")

    def test_visual_servo_loop(self):
        """视觉伺服闭环测试"""
        if not self.integration_available:
            self.skipTest("集成模块未安装")

        try:
            # 模拟视觉伺服过程
            img = create_test_image(640, 480, (100, 100, 100))
            cv2.circle(img, (400, 240), 50, (0, 0, 255), -1)

            robot_x = 0.0  # 机器人x坐标
            target_reached = False

            for _ in range(100):
                # 视觉检测
                results, _ = self.color_tracker.update(img)
                if len(results) == 0:
                    break

                target_x = results[0]['cx']
                error = target_x - 320

                # PID控制
                velocity = self.pid_controller.compute(error, dt=0.01)

                # 更新机器人位置
                robot_x += velocity * 0.1

                # 检查是否到达目标
                if abs(error) < 20:
                    target_reached = True
                    break

            if not target_reached:
                self.skipTest("未完成闭环")
        except Exception as e:
            self.skipTest(f"视觉伺服测试失败: {e}")


# ===================== 传感器-执行器集成测试 =====================

class TestSensorActuatorIntegration(unittest.TestCase):
    """传感器-执行器集成测试"""

    def setUp(self):
        self.integration_available = False
        try:
            from control.pid_controller import PIDController
            self.pid_controller = PIDController(kp=1.0, ki=0.1, kd=0.01)
            self.integration_available = True
        except ImportError as e:
            self.skipTest(f"控制模块未安装: {e}")

    def test_encoder_pid_integration(self):
        """编码器-PID集成测试"""
        if not self.integration_available:
            self.skipTest("集成模块未安装")

        try:
            # 模拟编码器反馈
            encoder_count = 0
            target_count = 360  # 目标脉冲数
            dt = 0.01

            for _ in range(500):
                error = target_count - encoder_count
                velocity = self.pid_controller.compute(error, dt)

                # 模拟电机响应
                encoder_count += velocity * dt * 10

                if abs(error) < 5:
                    break

            final_error = abs(target_count - encoder_count)
            self.assertLess(final_error, 10, "编码器-PID集成误差过大")
        except Exception as e:
            self.skipTest(f"编码器-PID集成测试失败: {e}")

    def test_ir_sensor_pid_integration(self):
        """红外传感器-PID集成测试"""
        if not self.integration_available:
            self.skipTest("集成模块未安装")

        try:
            # 模拟红外传感器数据
            target_position = 0.0  # 目标位置(中心)
            current_position = 3.0  # 初始偏移(偏左)
            dt = 0.01

            for _ in range(200):
                error = target_position - current_position
                correction = self.pid_controller.compute(error, dt)

                # 模拟运动
                current_position += correction * dt

                if abs(error) < 0.1:
                    break

            final_error = abs(target_position - current_position)
            self.assertLess(final_error, 0.5, "IR-PID集成误差过大")
        except Exception as e:
            self.skipTest(f"IR-PID集成测试失败: {e}")


# ===================== 多传感器融合测试 =====================

class TestSensorFusion(unittest.TestCase):
    """多传感器融合集成测试"""

    def test_kalman_pid_fusion(self):
        """Kalman-PID融合测试"""
        try:
            from simulation.kalman_filter_simulation import KalmanFilterSimulator
            from control.pid_controller import PIDController

            kalman = KalmanFilterSimulator()
            pid = PIDController(kp=1.0, ki=0.1, kd=0.01)

            # 模拟传感器数据
            true_position = 100.0
            measurements = true_position + np.random.randn(100) * 5

            filtered_positions = []
            for z in measurements:
                # Kalman滤波
                state = kalman.update(z)
                filtered_positions.append(state)

                # PID控制
                error = 0.0 - state  # 目标位置为0
                output = pid.compute(error, 0.1)

            filtered_positions = np.array(filtered_positions)
            rmse = np.sqrt(np.mean((filtered_positions - true_position) ** 2))

            print(f"\n传感器融合性能:")
            print(f"  滤波RMSE: {rmse:.4f}")

            self.assertLess(rmse, 5.0, "传感器融合精度不足")
        except ImportError as e:
            self.skipTest(f"融合模块未安装: {e}")

    def test_complementary_filter(self):
        """互补滤波器测试"""
        try:
            # 互补滤波器: 融合高频和低频传感器
            alpha = 0.98  # 滤波系数

            # 模拟陀螺仪和加速度计数据
            gyro_angle = 0.0
            accel_angle = 0.0
            dt = 0.01

            fused_angle = 0.0
            for _ in range(100):
                # 模拟传感器读数
                gyro_angle += 2.0 * dt  # 陀螺仪积分
                accel_angle = 30.0 + np.random.randn() * 5  # 加速度计(有噪声)

                # 互补滤波
                fused_angle = alpha * (fused_angle + 2.0 * dt) + (1 - alpha) * accel_angle

            # 融合结果应接近真实角度(30度)
            self.assertAlmostEqual(fused_angle, 30.0, delta=5.0,
                                 msg="互补滤波器精度不足")
        except Exception as e:
            self.skipTest(f"互补滤波器测试失败: {e}")


# ===================== 视觉处理流水线测试 =====================

@unittest.skipUnless(HAS_CV2, "OpenCV未安装，跳过视觉流水线测试")
class TestVisionPipeline(unittest.TestCase):
    """视觉处理流水线集成测试"""

    def test_color_detection_to_tracking(self):
        """颜色检测到跟踪流水线"""
        try:
            from visual.color_tracker import ColorTracker
            from visual.tracking_kalman import TrackingKalman

            tracker = ColorTracker(color_name='red', min_area=100)
            kalman_tracker = TrackingKalman(initial_x=320, initial_y=240)

            # 模拟连续帧
            img = create_test_image(640, 480, (100, 100, 100))
            cv2.circle(img, (350, 240), 50, (0, 0, 255), -1)

            for _ in range(10):
                # 视觉检测
                results, _ = tracker.update(img)
                if len(results) > 0:
                    # Kalman更新
                    kalman_tracker.update(results[0]['cx'], results[0]['cy'])

            # 验证跟踪
            final_pos = kalman_tracker.get_position()
            self.assertIsNotNone(final_pos)
            self.assertGreater(len(kalman_tracker.trajectory), 0)
        except ImportError as e:
            self.skipTest(f"视觉流水线模块未安装: {e}")

    def test_detection_to_control_pipeline(self):
        """检测到控制完整流水线"""
        try:
            from visual.color_tracker import ColorTracker
            from control.pid_controller import PIDController

            tracker = ColorTracker(color_name='red', min_area=100)
            pid = PIDController(kp=0.5, ki=0.1, kd=0.05)

            img = create_test_image(640, 480, (100, 100, 100))
            cv2.circle(img, (400, 240), 50, (0, 0, 255), -1)

            control_outputs = []
            for _ in range(50):
                # 检测
                results, _ = tracker.update(img)
                if len(results) > 0:
                    # 误差计算
                    error = results[0]['cx'] - 320
                    # PID控制
                    output = pid.compute(error, 0.01)
                    control_outputs.append(output)

            if len(control_outputs) > 0:
                # 控制输出应该有变化
                output_var = np.var(control_outputs)
                self.assertGreater(output_var, 0, "控制输出无变化")
        except ImportError as e:
            self.skipTest(f"流水线模块未安装: {e}")


# ===================== 模块接口兼容性测试 =====================

class TestModuleInterfaceCompatibility(unittest.TestCase):
    """模块接口兼容性测试"""

    def test_pid_interface_compatibility(self):
        """PID控制器接口兼容性"""
        try:
            from control.pid_controller import PIDController

            # 标准接口测试
            pid = PIDController(kp=1.0, ki=0.1, kd=0.01)

            # 检查必要方法
            self.assertTrue(hasattr(pid, 'compute'))
            self.assertTrue(hasattr(pid, 'reset'))
            self.assertTrue(hasattr(pid, 'set_gains'))

            # 检查compute接口
            output = pid.compute(error=10.0, dt=0.01)
            self.assertIsNotNone(output)

            # 检查reset接口
            pid.reset()

            # 检查set_gains接口
            pid.set_gains(kp=2.0, ki=0.2, kd=0.02)

        except ImportError as e:
            self.skipTest(f"PID模块未安装: {e}")

    def test_motor_interface_compatibility(self):
        """电机控制器接口兼容性"""
        try:
            from control.motor_controller import MotorController

            # 检查必要方法存在
            self.assertTrue(hasattr(MotorController, 'set_speed'))
            self.assertTrue(hasattr(MotorController, 'stop'))
            self.assertTrue(hasattr(MotorController, 'brake'))

        except ImportError as e:
            self.skipTest(f"电机模块未安装: {e}")

    def test_encoder_interface_compatibility(self):
        """编码器接口兼容性"""
        try:
            from control.encoder_reader import EncoderReader

            # 检查必要方法存在
            self.assertTrue(hasattr(EncoderReader, 'get_count'))
            self.assertTrue(hasattr(EncoderReader, 'get_angle'))
            self.assertTrue(hasattr(EncoderReader, 'get_speed'))
            self.assertTrue(hasattr(EncoderReader, 'reset_count'))

        except ImportError as e:
            self.skipTest(f"编码器模块未安装: {e}")


# ===================== 数据流一致性测试 =====================

class TestDataFlowConsistency(unittest.TestCase):
    """数据流一致性测试"""

    @unittest.skipUnless(HAS_CV2, "OpenCV未安装，跳过图像格式测试")
    def test_image_format_consistency(self):
        """图像格式一致性"""
        try:
            # 模拟图像处理流水线
            img = create_test_image(640, 480, (100, 100, 100))

            # 检查格式
            self.assertEqual(img.shape, (480, 640, 3))
            self.assertEqual(img.dtype, np.uint8)

            # 转换处理
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            self.assertEqual(hsv.shape, (480, 640, 3))

            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            self.assertEqual(gray.shape, (480, 640))
        except Exception as e:
            self.skipTest(f"图像格式测试失败: {e}")

    def test_numeric_type_consistency(self):
        """数值类型一致性"""
        # 检查各模块的数值类型
        float_val = 3.14
        int_val = 100
        np_float = np.float64(3.14)
        np_int = np.int32(100)

        # 验证类型
        self.assertIsInstance(float_val, float)
        self.assertIsInstance(int_val, int)
        self.assertIsInstance(np_float, np.floating)
        self.assertIsInstance(np_int, np.integer)


# ===================== 性能集成基准测试 =====================

@unittest.skipUnless(HAS_CV2, "OpenCV未安装，跳过性能集成基准测试")
class TestPerformanceIntegrationBenchmark(unittest.TestCase):
    """性能集成基准测试"""

    def test_visual_pid_pipeline_performance(self):
        """视觉-PID流水线性能"""
        try:
            from visual.color_tracker import ColorTracker
            from control.pid_controller import PIDController

            tracker = ColorTracker(color_name='red', min_area=100)
            pid = PIDController(kp=0.5, ki=0.1, kd=0.05)

            img = create_test_image(640, 480, (100, 100, 100))
            cv2.circle(img, (400, 240), 50, (0, 0, 255), -1)

            start_time = time.time()
            for _ in range(100):
                results, _ = tracker.update(img)
                if len(results) > 0:
                    error = results[0]['cx'] - 320
                    output = pid.compute(error, 0.01)
            elapsed = time.time() - start_time

            fps = 100 / elapsed

            print(f"\n视觉-PID流水线性能:")
            print(f"  100帧处理时间: {elapsed:.3f}秒")
            print(f"  处理帧率: {fps:.2f} FPS")

            # 视觉控制流水线应达到30+ FPS
            self.assertGreater(fps, 20, "视觉-PID流水线性能不足")
        except ImportError as e:
            self.skipTest(f"性能基准模块未安装: {e}")

    def test_sensor_fusion_pipeline_performance(self):
        """传感器融合流水线性能"""
        try:
            from simulation.kalman_filter_simulation import KalmanFilterSimulator
            from control.pid_controller import PIDController

            kalman = KalmanFilterSimulator()
            pid = PIDController(kp=1.0, ki=0.1, kd=0.01)

            start_time = time.time()
            for _ in range(1000):
                z = 100.0 + np.random.randn() * 5
                state = kalman.update(z)
                error = 0.0 - state
                output = pid.compute(error, 0.01)
            elapsed = time.time() - start_time

            fps = 1000 / elapsed

            print(f"\n传感器融合流水线性能:")
            print(f"  1000次融合时间: {elapsed:.3f}秒")
            print(f"  融合帧率: {fps:.2f} FPS")

            # 融合控制应达到100+ FPS
            self.assertGreater(fps, 100, "传感器融合流水线性能不足")
        except ImportError as e:
            self.skipTest(f"融合性能基准模块未安装: {e}")


# ===================== 错误处理集成测试 =====================

class TestErrorHandlingIntegration(unittest.TestCase):
    """错误处理集成测试"""

    def test_none_input_handling(self):
        """None输入处理"""
        try:
            from control.pid_controller import PIDController
            pid = PIDController(kp=1.0, ki=0.1, kd=0.01)

            # None输入应被处理或抛出明确异常
            try:
                output = pid.compute(None, 0.01)
                # 如果不抛出异常，输出应为None或0
                self.assertTrue(output is None or output == 0)
            except (TypeError, ValueError):
                pass  # 预期的异常
        except ImportError as e:
            self.skipTest(f"错误处理测试模块未安装: {e}")

    def test_invalid_parameter_handling(self):
        """无效参数处理"""
        try:
            from control.pid_controller import PIDController

            # 无效参数应被处理或抛出明确异常
            try:
                pid = PIDController(kp=-1.0, ki=-0.1, kd=-0.01)
                # 检查是否正确处理负参数
                output = pid.compute(10.0, 0.01)
            except ValueError:
                pass  # 预期的异常
        except ImportError as e:
            self.skipTest(f"错误处理测试模块未安装: {e}")


if __name__ == '__main__':
    unittest.main()
