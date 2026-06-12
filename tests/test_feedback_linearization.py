"""
反馈线性化单元测试
"""
import unittest
import numpy as np


class TestFeedbackLinearization(unittest.TestCase):
    """反馈线性化控制算法测试"""

    def setUp(self):
        self.dt = 0.01
        self.t_span = np.arange(0, 2.0, self.dt)

    def test_siso_linear_system(self):
        """单输入单输出线性系统反馈线性化"""
        # x_dot = -x + u, 选择 u = v + x 使 x_dot = v
        x = 5.0
        trajectory = [x]
        for _ in self.t_span:
            v = -2.0 * x  # 期望 v = -2x
            u = v + x  # 反馈线性化
            x = x + (-x + u) * self.dt
            trajectory.append(x)
        self.assertAlmostEqual(trajectory[-1], 0.0, places=1)

    def test_relative_degree(self):
        """测试相对度计算"""
        # y = x1, x1_dot = x2, x2_dot = -x1 + u -> relative degree = 2
        x1, x2 = 1.0, 0.0
        trajectory = [x1]
        for _ in self.t_span:
            # 反馈线性化: u = x1 + v, v = -k1*x1 - k2*x2
            v = -3.0 * x1 - 2.0 * x2
            u = x1 + v
            x2_new = x2 + (-x1 + u) * self.dt
            x1_new = x1 + x2 * self.dt
            x1, x2 = x1_new, x2_new
            trajectory.append(x1)
        self.assertAlmostEqual(trajectory[-1], 0.0, places=1)

    def test_nonlinear_system_stabilization(self):
        """非线性系统稳定化"""
        # x_dot = -x^3 + u, u = v + x^3, v = -kx
        x = 2.0
        k = 5.0
        trajectory = [x]
        for _ in self.t_span:
            v = -k * x
            u = v + x ** 3
            x = x + (-x ** 3 + u) * self.dt
            trajectory.append(x)
        self.assertAlmostEqual(trajectory[-1], 0.0, places=2)

    def test_state_space_transformation(self):
        """状态空间变换"""
        # 坐标变换 z = phi(x)
        x = np.array([1.0, 2.0])
        # phi = [x1, x1 + x2]
        z = np.array([x[0], x[0] + x[1]])
        self.assertAlmostEqual(z[0], 1.0)
        self.assertAlmostEqual(z[1], 3.0)

    def test_input_output_linearization(self):
        """输入输出线性化"""
        # y = h(x) = x1, 输出的n阶导数应等于v
        x1, x2 = 3.0, 0.0
        history = [x1]
        for _ in range(200):
            # Lfh = x2, Lf2h = -x1, LgLfh = 1
            # y'' = Lf2h + LgLfh * u = -x1 + u = v
            v = -4.0 * x1 - 4.0 * x2
            u = v + x1
            x2 = x2 + (-x1 + u) * self.dt
            x1 = x1 + x2 * self.dt
            history.append(x1)
        self.assertAlmostEqual(history[-1], 0.0, places=1)

    def test_tracking_error_convergence(self):
        """跟踪误差收敛"""
        x = 0.0
        x_ref = lambda t: np.sin(t)
        errors = []
        for i, t in enumerate(self.t_span):
            xd = x_ref(t)
            e = x - xd
            errors.append(abs(e))
            u = -3.0 * e - 2.0 * (x - x_ref(max(0, t - self.dt))) / self.dt + np.cos(t)
            x = x + u * self.dt
        self.assertLess(errors[-1], errors[0])

    def test_zero_dynamics_stability(self):
        """零动态稳定性检测"""
        # 内动态应稳定
        eta = 1.0
        z = 0.0
        history = [eta]
        for _ in range(500):
            eta_dot = -eta + z
            eta = eta + eta_dot * self.dt
            z = 0.0  # 零动态下 z=0
            history.append(eta)
        self.assertAlmostEqual(history[-1], 0.0, places=2)

    def test_control_bounds(self):
        """控制输入有界性"""
        x = 10.0
        u_max = 100.0
        for _ in self.t_span:
            u = -5.0 * x
            u = np.clip(u, -u_max, u_max)
            x = x + (-x + u) * self.dt
            self.assertLessEqual(abs(u), u_max)

    def test_robutness_to_disturbance(self):
        """抗扰动能力"""
        x = 1.0
        for _ in range(200):
            d = np.random.normal(0, 0.1)
            u = -3.0 * x
            x = x + (-x + u + d) * self.dt
        self.assertLess(abs(x), 2.0)  # 应保持有界

    def test_lyapunov_stability(self):
        """Lyapunov稳定性验证"""
        x = np.array([2.0, 1.0])
        V_history = []
        for _ in range(300):
            V = 0.5 * (x[0] ** 2 + x[1] ** 2)
            V_history.append(V)
            u = -3.0 * x[0] - 2.0 * x[1]
            x1_dot = x[1]
            x2_dot = -x[0] + u
            x[0] += x1_dot * self.dt
            x[1] += x2_dot * self.dt
        self.assertLess(V_history[-1], V_history[0])


if __name__ == '__main__':
    unittest.main()
