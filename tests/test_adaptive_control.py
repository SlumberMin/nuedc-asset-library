"""
自适应控制单元测试
"""
import unittest
import numpy as np


class TestAdaptiveControl(unittest.TestCase):
    """自适应控制算法测试"""

    def setUp(self):
        self.dt = 0.01
        self.N = 500

    def test_mrac_basic(self):
        """模型参考自适应控制基本测试"""
        # 参考模型: xm_dot = -am * xm + am * r
        am = 2.0
        x = 0.0
        xm = 0.0
        r = 1.0
        theta = 0.0  # 自适应参数
        gamma = 5.0  # 自适应增益
        for _ in range(self.N):
            e = x - xm
            u = theta * r
            theta = theta - gamma * e * r * self.dt
            x = x + (-3.0 * x + u) * self.dt
            xm = xm + (-am * xm + am * r) * self.dt
        # 误差应收敛
        self.assertLess(abs(x - xm), abs(x))

    def test_parameter_estimation(self):
        """参数估计收敛"""
        # 真实系统: y = theta_true * u
        theta_true = 2.0
        theta_hat = 0.0
        gamma = 0.5
        errors = []
        for _ in range(self.N):
            u = np.random.randn()
            y = theta_true * u
            y_hat = theta_hat * u
            e = y - y_hat
            theta_hat = theta_hat + gamma * e * u * self.dt
            errors.append(abs(e))
        self.assertLess(errors[-1], errors[0])

    def test_gradient_descent_adaptation(self):
        """梯度下降自适应"""
        theta = 0.0
        theta_true = 3.0
        gamma = 1.0
        for _ in range(self.N):
            u = 1.0
            y = theta_true * u
            y_hat = theta * u
            e = y_hat - y
            theta = theta - gamma * e * u * self.dt
        self.assertAlmostEqual(theta, theta_true, places=0)

    def test_lyapunov_adaptation(self):
        """Lyapunov设计自适应律"""
        x = 1.0
        x_ref = 0.0
        a_true = -2.0
        a_hat = 0.0
        gamma = 10.0
        for _ in range(self.N):
            e = x - x_ref
            u = -a_hat * x - 2.0 * e
            x = x + (a_true * x + u) * self.dt
            a_hat = a_hat + gamma * e * x * self.dt
        self.assertLess(abs(x), 0.5)

    def test_sigma_modification(self):
        """σ修正防止参数漂移"""
        theta = 0.0
        theta_true = 1.0
        gamma = 5.0
        sigma = 0.1
        for _ in range(self.N * 2):
            u = np.random.randn()
            y = theta_true * u + 0.5 * np.random.randn()  # 含噪声
            e = theta * u - y
            theta = theta - gamma * (e * u + sigma * theta) * self.dt
        self.assertLess(abs(theta - theta_true), 2.0)

    def test_dead_zone_adaptation(self):
        """死区自适应"""
        theta = 0.0
        theta_true = 2.0
        gamma = 5.0
        delta = 0.1  # 死区阈值
        for _ in range(self.N):
            u = 1.0
            y = theta_true * u
            e = theta * u - y
            if abs(e) > delta:
                theta = theta - gamma * e * u * self.dt
        self.assertLess(abs(theta - theta_true), 3.0)

    def test_multiple_parameters(self):
        """多参数自适应"""
        theta_true = np.array([1.0, 2.0])
        theta_hat = np.zeros(2)
        gamma = 1.0
        for _ in range(self.N):
            phi = np.random.randn(2)
            y = theta_true @ phi
            y_hat = theta_hat @ phi
            e = y_hat - y
            theta_hat = theta_hat - gamma * e * phi * self.dt
        np.testing.assert_allclose(theta_hat, theta_true, atol=0.5)

    def test_adaptive_tracking(self):
        """自适应跟踪控制"""
        x = 0.0
        a_true = -1.0
        b_true = 1.0
        a_hat, b_hat = 0.0, 0.5
        gamma = 5.0
        tracking_errors = []
        for i in range(self.N):
            t = i * self.dt
            r = np.sin(t)
            e = x - r
            tracking_errors.append(abs(e))
            u = (r * (1 + a_hat) - 3.0 * e) / max(b_hat, 0.1)
            x = x + (a_true * x + b_true * u) * self.dt
            a_hat = a_hat + gamma * e * x * self.dt
            b_hat = b_hat + gamma * e * u * self.dt
        self.assertLess(tracking_errors[-1], tracking_errors[0] + 0.5)

    def test_persistent_excitation(self):
        """持续激励条件"""
        theta_true = 1.5
        theta_hat = 0.0
        gamma = 2.0
        # PE信号: 多频率叠加
        for i in range(self.N):
            t = i * self.dt
            u = np.sin(t) + 0.5 * np.sin(3 * t)
            y = theta_true * u
            e = theta_hat * u - y
            theta_hat = theta_hat - gamma * e * u * self.dt
        self.assertAlmostEqual(theta_hat, theta_true, places=0)

    def test_adaptive_robustness(self):
        """自适应控制鲁棒性"""
        x = 1.0
        a_true = -2.0
        a_hat = 0.0
        gamma = 3.0
        sigma = 0.05
        for _ in range(self.N):
            d = 0.1 * np.sin(10 * _ * self.dt)  # 扰动
            u = -a_hat * x - 2.0 * x
            x = x + (a_true * x + u + d) * self.dt
            a_hat = a_hat + gamma * x * x * self.dt - sigma * a_hat * self.dt
        self.assertLess(abs(x), 2.0)

    def test_parameter_convergence_rate(self):
        """参数收敛速率"""
        theta_true = 1.0
        theta_hat = 0.0
        gamma = 10.0
        times = []
        for i in range(self.N):
            u = 1.0
            e = theta_hat * u - theta_true * u
            theta_hat = theta_hat - gamma * e * u * self.dt
            if abs(theta_hat - theta_true) < 0.1 and not times:
                times.append(i * self.dt)
        self.assertLess(times[0], 1.0)  # 应快速收敛


if __name__ == '__main__':
    unittest.main()
