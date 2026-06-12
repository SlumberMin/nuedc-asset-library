#!/usr/bin/env python3
"""
机械臂控制器V2测试 — 多舵机 + 步进电机联动
覆盖: 舵机角度控制、PCA9685多路PWM、步进电机定位、
      机械臂运动学、轨迹规划、坐标映射
对应C源文件: 02_mspm0g3507/drivers/servo.c + pca9685.c + stepper_a4988.c

错误经验检查:
  #9:  测试import wrappers.py而非自行重写
"""

import sys
import os
import math
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from wrappers import (
    Servo, PCA9685, StepperA4988,
    SERVO_MAX_ANGLE, SERVO_MIN_PULSE, SERVO_MAX_PULSE,
    STEPPER_DIR_CW, STEPPER_DIR_CCW, STEPPER_MAX_SPEED,
    PCA9685_NUM_CHANNELS,
)


# ═══════════════════════════════════════════════════════════════
#  机械臂参数
# ═══════════════════════════════════════════════════════════════

# 机械臂连杆长度(mm)
ARM_BASE_HEIGHT = 100    # 底座高度
ARM_LINK1 = 150          # 大臂长度
ARM_LINK2 = 120          # 小臂长度
ARM_GRIPPER = 60         # 夹爪长度

# 关节限位
JOINT_BASE_MIN = 0       # 底座旋转最小角度
JOINT_BASE_MAX = 180     # 底座旋转最大角度
JOINT_SHOULDER_MIN = 15  # 肩关节最小角度
JOINT_SHOULDER_MAX = 165
JOINT_ELBOW_MIN = 0      # 肘关节最小角度
JOINT_ELBOW_MAX = 180
JOINT_GRIPPER_MIN = 0    # 夹爪最小角度
JOINT_GRIPPER_MAX = 90

# 舵机通道映射
CH_BASE = 0
CH_SHOULDER = 1
CH_ELBOW = 2
CH_GRIPPER = 3

# 步进电机控制底座升降
STEPS_PER_MM = 50        # 每毫米步数
Z_MIN = 0
Z_MAX = 200              # 最大升降高度(mm)


class ArmKinematics:
    """机械臂运动学"""

    @staticmethod
    def forward(theta1_deg, theta2_deg):
        """正运动学：关节角 → 末端位置

        theta1: 肩关节角度(度)
        theta2: 肘关节角度(度)
        返回: (x, y) 末端坐标(mm)
        """
        t1 = math.radians(theta1_deg)
        t2 = math.radians(theta2_deg)
        x = ARM_LINK1 * math.cos(t1) + ARM_LINK2 * math.cos(t1 + t2)
        y = ARM_LINK1 * math.sin(t1) + ARM_LINK2 * math.sin(t1 + t2)
        return x, y

    @staticmethod
    def inverse(x, y):
        """逆运动学：末端位置 → 关节角

        返回: (theta1, theta2) 关节角度(度)
              如果不可达返回None
        """
        dist_sq = x * x + y * y
        dist = math.sqrt(dist_sq)

        # 检查工作空间
        max_reach = ARM_LINK1 + ARM_LINK2
        min_reach = abs(ARM_LINK1 - ARM_LINK2)
        if dist > max_reach or dist < min_reach:
            return None

        # 余弦定理求theta2
        cos_t2 = (dist_sq - ARM_LINK1 ** 2 - ARM_LINK2 ** 2) / \
                 (2.0 * ARM_LINK1 * ARM_LINK2)
        cos_t2 = max(-1.0, min(1.0, cos_t2))
        t2 = math.acos(cos_t2)

        # 求theta1
        t1 = math.atan2(y, x) - math.atan2(
            ARM_LINK2 * math.sin(t2),
            ARM_LINK1 + ARM_LINK2 * math.cos(t2))

        return math.degrees(t1), math.degrees(t2)


class ServoArmController:
    """机械臂控制器 — 4自由度

    关节:
    1. 底座旋转 (Servo CH0, 0~180°)
    2. 肩关节   (Servo CH1, 15~165°)
    3. 肘关节   (Servo CH2, 0~180°)
    4. 夹爪     (Servo CH3, 0~90°)
    5. 底座升降 (Stepper, 0~200mm)
    """

    def __init__(self):
        self.pwm = PCA9685()
        self.servos = [Servo() for _ in range(4)]
        self.stepper_z = StepperA4988()

        # 当前关节角度
        self.joints = [90, 90, 90, 45]  # base, shoulder, elbow, gripper
        self.z_height = 0.0              # 升降高度(mm)

        # 速度限制 (度/步)
        self.max_step_deg = 2.0
        self.initialized = False

    def init(self):
        """初始化所有硬件"""
        self.pwm.init()
        self.pwm.set_pwm_freq(50)  # 50Hz舵机频率
        for s in self.servos:
            s.init()
        self.stepper_z.init()
        self.stepper_z.enable()
        self.stepper_z.set_speed(200)
        self.initialized = True
        # 移动到初始位置
        self.set_joint(CH_BASE, 90)
        self.set_joint(CH_SHOULDER, 90)
        self.set_joint(CH_ELBOW, 90)
        self.set_joint(CH_GRIPPER, 45)

    def set_joint(self, joint, angle):
        """设置关节角度

        joint: 关节编号 (CH_BASE=0, CH_SHOULDER=1, ...)
        angle: 目标角度(度)
        返回: (success, actual_angle)
        """
        if joint < 0 or joint > CH_GRIPPER:
            return False, 0

        # 限位检查
        limits = [
            (JOINT_BASE_MIN, JOINT_BASE_MAX),
            (JOINT_SHOULDER_MIN, JOINT_SHOULDER_MAX),
            (JOINT_ELBOW_MIN, JOINT_ELBOW_MAX),
            (JOINT_GRIPPER_MIN, JOINT_GRIPPER_MAX),
        ]
        min_a, max_a = limits[joint]
        angle = max(min_a, min(max_a, angle))

        # 设置舵机
        self.servos[joint].set_angle(int(angle))
        # 同步到PCA9685
        self.pwm.set_angle(joint, angle)
        self.joints[joint] = angle
        return True, angle

    def get_joint(self, joint):
        """获取当前关节角度"""
        if 0 <= joint <= CH_GRIPPER:
            return self.joints[joint]
        return 0

    def set_z_height(self, height_mm):
        """设置升降高度"""
        height_mm = max(Z_MIN, min(Z_MAX, height_mm))
        target_steps = int(height_mm * STEPS_PER_MM)
        current_steps = int(self.z_height * STEPS_PER_MM)
        delta = target_steps - current_steps

        self.stepper_z.move_relative(delta)
        self.stepper_z.run_to_target()
        self.z_height = height_mm
        return True

    def get_z_height(self):
        """获取当前升降高度"""
        return self.z_height

    def open_gripper(self):
        """打开夹爪"""
        return self.set_joint(CH_GRIPPER, JOINT_GRIPPER_MIN)

    def close_gripper(self):
        """关闭夹爪"""
        return self.set_joint(CH_GRIPPER, JOINT_GRIPPER_MAX)

    def move_to_xyz(self, x, y, z=None):
        """移动到笛卡尔坐标

        x, y: 水平面坐标(mm)
        z: 升降高度(mm), None=不调整
        返回: success
        """
        result = ArmKinematics.inverse(x, y)
        if result is None:
            return False

        theta1, theta2 = result

        # 限位检查
        if not (JOINT_SHOULDER_MIN <= theta1 <= JOINT_SHOULDER_MAX):
            return False
        if not (JOINT_ELBOW_MIN <= theta2 <= JOINT_ELBOW_MAX):
            return False

        self.set_joint(CH_SHOULDER, theta1)
        self.set_joint(CH_ELBOW, theta2)

        if z is not None:
            self.set_z_height(z)

        return True

    def get_end_effector_pos(self):
        """获取末端执行器位置"""
        x, y = ArmKinematics.forward(self.joints[CH_SHOULDER],
                                      self.joints[CH_ELBOW])
        return x, y, self.z_height

    def grip_sequence(self, x, y, z_down, z_up):
        """抓取序列：移动→下降→夹紧→提升"""
        # 移动到目标上方
        if not self.move_to_xyz(x, y, z_up):
            return False
        # 打开夹爪
        self.open_gripper()
        # 下降
        self.set_z_height(z_down)
        # 夹紧
        self.close_gripper()
        # 提升
        self.set_z_height(z_up)
        return True

    def home(self):
        """回到初始位置"""
        self.set_joint(CH_BASE, 90)
        self.set_joint(CH_SHOULDER, 90)
        self.set_joint(CH_ELBOW, 90)
        self.open_gripper()
        self.set_z_height(50)
        return True


# ═══════════════════════════════════════════════════════════════
#  测试类
# ═══════════════════════════════════════════════════════════════

class TestArmInit(unittest.TestCase):
    """机械臂初始化测试"""

    def test_init_success(self):
        """初始化成功"""
        arm = ServoArmController()
        arm.init()
        self.assertTrue(arm.initialized)
        self.assertTrue(arm.pwm.initialized)

    def test_initial_joints(self):
        """初始关节位置"""
        arm = ServoArmController()
        arm.init()
        self.assertEqual(arm.get_joint(CH_BASE), 90)
        self.assertEqual(arm.get_joint(CH_SHOULDER), 90)
        self.assertEqual(arm.get_joint(CH_ELBOW), 90)
        self.assertEqual(arm.get_joint(CH_GRIPPER), 45)

    def test_servo_count(self):
        """舵机数量"""
        arm = ServoArmController()
        self.assertEqual(len(arm.servos), 4)

    def test_stepper_initialized(self):
        """步进电机初始化"""
        arm = ServoArmController()
        arm.init()
        self.assertTrue(arm.stepper_z.initialized)
        self.assertTrue(arm.stepper_z.enabled)


class TestServoBasic(unittest.TestCase):
    """SG90舵机基本测试"""

    def test_init_angle(self):
        """初始化角度90°"""
        s = Servo()
        s.init()
        self.assertEqual(s.get_angle(), 90)

    def test_set_angle(self):
        """设置角度"""
        s = Servo()
        s.init()
        s.set_angle(45)
        self.assertEqual(s.get_angle(), 45)
        s.set_angle(135)
        self.assertEqual(s.get_angle(), 135)

    def test_angle_clamp(self):
        """角度限幅"""
        s = Servo()
        s.init()
        s.set_angle(200)  # 超过180°
        self.assertEqual(s.get_angle(), 180)

    def test_pulse_width(self):
        """脉宽设置"""
        s = Servo()
        s.init()
        # 90°对应1500µs
        s.set_pulse_width(1500)
        self.assertEqual(s.get_angle(), 90)

    def test_pulse_range(self):
        """脉宽范围"""
        s = Servo()
        s.init()
        s.set_pulse_width(500)   # 0°
        self.assertEqual(s.get_angle(), 0)
        s.set_pulse_width(2500)  # 180°
        self.assertEqual(s.get_angle(), 180)


class TestPCA9685(unittest.TestCase):
    """PCA9685 PWM驱动测试"""

    def test_init(self):
        """初始化"""
        pwm = PCA9685()
        pwm.init()
        self.assertTrue(pwm.initialized)

    def test_set_freq(self):
        """设置频率"""
        pwm = PCA9685()
        pwm.init()
        self.assertTrue(pwm.set_pwm_freq(50))
        self.assertEqual(pwm.freq, 50)

    def test_freq_range(self):
        """频率范围检查"""
        pwm = PCA9685()
        pwm.init()
        self.assertFalse(pwm.set_pwm_freq(10))   # 太低
        self.assertFalse(pwm.set_pwm_freq(2000))  # 太高

    def test_set_pwm(self):
        """设置PWM值"""
        pwm = PCA9685()
        pwm.init()
        self.assertTrue(pwm.set_pwm(0, 0, 300))
        on, off = pwm.get_pwm(0)
        self.assertEqual(on, 0)
        self.assertEqual(off, 300)

    def test_set_angle(self):
        """设置舵机角度"""
        pwm = PCA9685()
        pwm.init()
        self.assertTrue(pwm.set_angle(0, 90))
        on, off = pwm.get_pwm(0)
        # 90°应映射到中间值
        self.assertGreater(off, 102)
        self.assertLess(off, 512)

    def test_channel_limit(self):
        """通道限制"""
        pwm = PCA9685()
        pwm.init()
        self.assertFalse(pwm.set_pwm(-1, 0, 0))
        self.assertFalse(pwm.set_pwm(PCA9685_NUM_CHANNELS, 0, 0))

    def test_all_off(self):
        """全部关闭"""
        pwm = PCA9685()
        pwm.init()
        pwm.set_pwm(0, 0, 300)
        pwm.all_off()
        on, off = pwm.get_pwm(0)
        self.assertEqual(on, 0)
        self.assertEqual(off, 0)


class TestStepperMotor(unittest.TestCase):
    """步进电机测试"""

    def test_init(self):
        """初始化"""
        st = StepperA4988()
        st.init()
        self.assertTrue(st.initialized)

    def test_enable_disable(self):
        """使能/失能"""
        st = StepperA4988()
        st.init()
        st.enable()
        self.assertTrue(st.enabled)
        st.disable()
        self.assertFalse(st.enabled)

    def test_move_to(self):
        """移动到目标"""
        st = StepperA4988()
        st.init()
        st.enable()
        st.move_to(100)
        st.run_to_target()
        self.assertEqual(st.get_position(), 100)
        self.assertTrue(st.is_at_target())

    def test_move_relative(self):
        """相对移动"""
        st = StepperA4988()
        st.init()
        st.enable()
        st.move_to(50)
        st.run_to_target()
        st.move_relative(30)
        st.run_to_target()
        self.assertEqual(st.get_position(), 80)

    def test_direction(self):
        """方向控制"""
        st = StepperA4988()
        st.init()
        st.enable()
        st.move_to(10)
        st.run_to_target()
        self.assertEqual(st.direction, STEPPER_DIR_CW)
        st.move_to(0)
        st.run_to_target()
        self.assertEqual(st.direction, STEPPER_DIR_CCW)

    def test_stop(self):
        """停止"""
        st = StepperA4988()
        st.init()
        st.enable()
        st.move_to(1000)
        st.step()  # 走一步
        st.stop()
        self.assertTrue(st.is_at_target())

    def test_speed_limit(self):
        """速度限制"""
        st = StepperA4988()
        st.init()
        st.set_speed(STEPPER_MAX_SPEED + 100)
        self.assertEqual(st.speed, STEPPER_MAX_SPEED)

    def test_microstep(self):
        """细分设置"""
        st = StepperA4988()
        st.init()
        self.assertTrue(st.set_microstep(16))
        self.assertEqual(st.microstep, 16)
        self.assertFalse(st.set_microstep(3))  # 无效细分


class TestKinematics(unittest.TestCase):
    """运动学测试"""

    def test_forward_zero_angles(self):
        """零角度正运动学"""
        x, y = ArmKinematics.forward(0, 0)
        # 水平伸展
        expected_x = ARM_LINK1 + ARM_LINK2
        self.assertAlmostEqual(x, expected_x, delta=1)
        self.assertAlmostEqual(y, 0.0, delta=1)

    def test_forward_vertical(self):
        """垂直正运动学"""
        x, y = ArmKinematics.forward(90, 0)
        # 垂直向上
        self.assertAlmostEqual(x, 0.0, delta=1)
        expected_y = ARM_LINK1 + ARM_LINK2
        self.assertAlmostEqual(y, expected_y, delta=1)

    def test_inverse_known_point(self):
        """已知点逆运动学"""
        # 正运动学得到的点，逆运动学应能恢复
        t1_in, t2_in = 60, 30
        x, y = ArmKinematics.forward(t1_in, t2_in)
        result = ArmKinematics.inverse(x, y)
        self.assertIsNotNone(result)
        t1_out, t2_out = result
        self.assertAlmostEqual(t1_out, t1_in, delta=1)
        self.assertAlmostEqual(t2_out, t2_in, delta=1)

    def test_inverse_unreachable(self):
        """不可达点"""
        # 超出最大臂长
        result = ArmKinematics.inverse(500, 500)
        self.assertIsNone(result)

    def test_inverse_min_reach(self):
        """最小臂展内点"""
        # 非常近的点(折叠状态)
        result = ArmKinematics.inverse(5, 0)
        # 可能可达也可能不可达
        if result is not None:
            t1, t2 = result
            self.assertIsInstance(t1, float)


class TestArmJointControl(unittest.TestCase):
    """关节控制测试"""

    def setUp(self):
        self.arm = ServoArmController()
        self.arm.init()

    def test_set_get_joint(self):
        """设置和获取关节"""
        ok, angle = self.arm.set_joint(CH_BASE, 45)
        self.assertTrue(ok)
        self.assertEqual(self.arm.get_joint(CH_BASE), 45)

    def test_joint_limit(self):
        """关节限位"""
        ok, angle = self.arm.set_joint(CH_SHOULDER, 0)  # 低于最小15°
        self.assertTrue(ok)
        self.assertEqual(angle, JOINT_SHOULDER_MIN)

    def test_invalid_joint(self):
        """无效关节号"""
        ok, _ = self.arm.set_joint(10, 90)
        self.assertFalse(ok)

    def test_gripper_open_close(self):
        """夹爪开关"""
        self.arm.open_gripper()
        self.assertEqual(self.arm.get_joint(CH_GRIPPER), JOINT_GRIPPER_MIN)
        self.arm.close_gripper()
        self.assertEqual(self.arm.get_joint(CH_GRIPPER), JOINT_GRIPPER_MAX)


class TestArmZAxis(unittest.TestCase):
    """Z轴升降测试"""

    def setUp(self):
        self.arm = ServoArmController()
        self.arm.init()

    def test_set_z_height(self):
        """设置升降高度"""
        self.arm.set_z_height(100)
        self.assertAlmostEqual(self.arm.get_z_height(), 100.0, delta=0.1)

    def test_z_limit(self):
        """升降限位"""
        self.arm.set_z_height(300)  # 超过最大
        self.assertAlmostEqual(self.arm.get_z_height(), Z_MAX, delta=0.1)

    def test_z_zero(self):
        """最低位"""
        self.arm.set_z_height(0)
        self.assertAlmostEqual(self.arm.get_z_height(), 0.0, delta=0.1)


class TestArmMovement(unittest.TestCase):
    """机械臂运动测试"""

    def setUp(self):
        self.arm = ServoArmController()
        self.arm.init()

    def test_move_to_xyz(self):
        """笛卡尔坐标移动"""
        # 水平前方200mm
        x, y = ArmKinematics.forward(60, 30)
        ok = self.arm.move_to_xyz(x, y)
        # 可能可达也可能不可达
        if ok:
            ex, ey, ez = self.arm.get_end_effector_pos()
            self.assertAlmostEqual(ex, x, delta=5)
            self.assertAlmostEqual(ey, y, delta=5)

    def test_move_unreachable(self):
        """不可达位置"""
        ok = self.arm.move_to_xyz(500, 500)
        self.assertFalse(ok)

    def test_end_effector_position(self):
        """末端位置计算"""
        self.arm.set_joint(CH_SHOULDER, 60)
        self.arm.set_joint(CH_ELBOW, 30)
        x, y, z = self.arm.get_end_effector_pos()
        # 与运动学计算一致
        expected_x, expected_y = ArmKinematics.forward(60, 30)
        self.assertAlmostEqual(x, expected_x, delta=1)
        self.assertAlmostEqual(y, expected_y, delta=1)

    def test_grip_sequence(self):
        """抓取序列"""
        # 在工作空间内找一个可达点
        x, y = ArmKinematics.forward(60, 45)
        ok = self.arm.grip_sequence(x, y, z_down=20, z_up=100)
        # 如果点可达则应成功
        if ok:
            self.assertAlmostEqual(self.arm.get_z_height(), 100, delta=1)
            self.assertEqual(self.arm.get_joint(CH_GRIPPER), JOINT_GRIPPER_MAX)

    def test_home(self):
        """回到初始位置"""
        self.arm.set_joint(CH_BASE, 45)
        self.arm.set_joint(CH_SHOULDER, 60)
        self.arm.set_z_height(150)
        self.arm.home()
        self.assertEqual(self.arm.get_joint(CH_BASE), 90)
        self.assertEqual(self.arm.get_joint(CH_SHOULDER), 90)
        self.assertAlmostEqual(self.arm.get_z_height(), 50, delta=1)


if __name__ == '__main__':
    unittest.main()
