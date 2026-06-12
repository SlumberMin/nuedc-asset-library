"""
最优控制单元测试
"""
import unittest
import numpy as np


class TestOptimalControl(unittest.TestCase):
    """最优控制算法测试"""

    def setUp(self):
        self.dt = 0.01
        self.N = 200  # 时间步数

    def test_lqr_stabilization(self):
        """LQR稳定化"""
        A = np.array([[0, 1], [0, -1]])
        B = np.array([[0], [1]])
        Q = np.diag([10.0, 1.0])
        R = np.array([[0.1]])
        # 简化LQR: 解Riccati方程
        P = np.eye(2)
        for _ in range(100):
            K = np.linalg.inv(R + B.T @ P @ B) @ (B.T @ P @ A)
            P_new = Q + A.T @ P @ A - A.T @ P @ B @ K
            if np.allclose(P_new, P, atol=1e-8):
                break
            P = P_new
        x = np.array([1.0, 0.0])
        for _ in range(self.N):
            u = -K @ x
            x = x + (A @ x + B @ u.flatten()) * self.dt
        self.assertAlmostEqual(np.linalg.norm(x), 0.0, places=1)

    def test_mpc_basic(self):
        """基本MPC控制器"""
        # 简单一维系统: x_dot = -x + u
        x = 5.0
        N_mpc = 10
        trajectory = [x]
        for _ in range(self.N):
            # 简化MPC: 前向模拟选最优u
            best_u = 0
            best_cost = float('inf')
            for u_try in np.linspace(-10, 10, 21):
                cost = 0
                x_sim = x
                for j in range(N_mpc):
                    x_sim = x_sim + (-x_sim + u_try) * self.dt
                    cost += x_sim ** 2 + 0.01 * u_try ** 2
                if cost < best_cost:
                    best_cost = cost
                    best_u = u_try
            x = x + (-x + best_u) * self.dt
            trajectory.append(x)
        self.assertAlmostEqual(trajectory[-1], 0.0, places=0)

    def test_bang_bang_control(self):
        """Bang-bang最优控制"""
        # 最短时间控制: x_dot = u, |u| <= 1
        x = 5.0
        u_max = 1.0
        trajectory = [x]
        t_final = 0
        for i in range(1000):
            u = -u_max if x > 0 else u_max
            x = x + u * self.dt
            trajectory.append(x)
            t_final += self.dt
            if abs(x) < 0.01:
                break
        self.assertAlmostEqual(x, 0.0, places=1)
        self.assertLess(t_final, abs(5.0 / u_max) + 0.5)

    def test_dynamic_programming_1d(self):
        """一维动态规划"""
        # 最小代价到达目标
        N = 50
        x_grid = np.linspace(-5, 5, 21)
        V = np.zeros((N + 1, len(x_grid)))
        V[N, :] = x_grid ** 2  # 终端代价
        for n in range(N - 1, -1, -1):
            for i, x in enumerate(x_grid):
                best = float('inf')
                for u in [-1, 0, 1]:
                    x_next = x + u * 0.1
                    j = np.argmin(np.abs(x_grid - x_next))
                    cost = V[n + 1, j] + x ** 2 + 0.01 * u ** 2
                    best = min(best, cost)
                V[n, i] = best
        self.assertLess(V[0, len(x_grid) // 2], V[N, len(x_grid) // 2])

    def test_pontryagin_minimum_principle(self):
        """Pontryagin最小值原理"""
        # x_dot = u, min integral(x^2 + u^2)dt
        x = 1.0
        lam = 1.0  # 协态变量
        trajectory = [x]
        for _ in range(self.N):
            u = -lam / 2  # 最优条件
            x = x + u * self.dt
            lam = lam + (-2 * x) * self.dt  # 协态方程
            trajectory.append(x)
        self.assertLess(abs(trajectory[-1]), abs(trajectory[0]))

    def test_linear_quadratic_regulator_terminal(self):
        """LQR终端代价"""
        A = np.array([[1, 0.01], [0, 1]])
        B = np.array([[0], [0.01]])
        x = np.array([1.0, 0.0])
        P = np.eye(2)
        Q = np.eye(2)
        R = np.array([[0.1]])
        for _ in range(50):
            K = np.linalg.inv(R + B.T @ P @ B) @ (B.T @ P @ A)
            P = Q + A.T @ P @ A - A.T @ P @ B @ K
        for _ in range(self.N):
            u = -K @ x
            x = A @ x + B @ u.flatten()
        self.assertLess(np.linalg.norm(x), 1.0)

    def test_cost_function_monotonicity(self):
        """代价函数单调性"""
        costs = []
        for u_val in np.linspace(-5, 5, 11):
            x = 3.0
            cost = 0
            for _ in range(100):
                u = u_val
                x = x + (-x + u) * self.dt
                cost += x ** 2 + 0.01 * u ** 2
            costs.append(cost)
        # u接近3时代价最小
        best_idx = np.argmin(costs)
        self.assertTrue(0 < best_idx < len(costs) - 1)

    def test_time_optimal_control(self):
        """时间最优控制"""
        x = 10.0
        u_max = 5.0
        t = 0
        while abs(x) > 0.1 and t < 10:
            u = -u_max * np.sign(x)
            x = x + u * self.dt
            t += self.dt
        self.assertAlmostEqual(x, 0.0, places=0)

    def test_energy_optimal_control(self):
        """能量最优控制"""
        x = 5.0
        # u = -kx, 选择k使能量最小
        k = 2.0
        energy = 0
        for _ in range(500):
            u = -k * x
            energy += u ** 2 * self.dt
            x = x + (-x + u) * self.dt
        self.assertGreater(energy, 0)
        self.assertLess(abs(x), 1.0)

    def test_state_constraints(self):
        """状态约束下的最优控制"""
        x = 3.0
        x_max = 5.0
        for _ in range(self.N):
            u = -2.0 * x
            x = x + u * self.dt
            x = np.clip(x, -x_max, x_max)
            self.assertLessEqual(abs(x), x_max + 0.01)


if __name__ == '__main__':
    unittest.main()
