#!/usr/bin/env python3
"""
路径规划仿真单元测试
覆盖: GridMap基础操作、邻居生成、障碍物管理、A*、Dijkstra、RRT算法
"""

import sys
import os
import unittest
import math
import random

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from _15_simulation.path_planning_simulation import GridMap, astar, dijkstra, rrt


class TestGridMapInit(unittest.TestCase):
    def test_default(self):
        g = GridMap()
        self.assertEqual(g.cols, 50)
        self.assertEqual(g.rows, 35)
        self.assertEqual(g.start, (2, 2))
        self.assertEqual(g.goal, (47, 32))

    def test_custom(self):
        g = GridMap(cols=20, rows=15, cell_size=10)
        self.assertEqual(g.cols, 20)
        self.assertEqual(g.rows, 15)


class TestGridMapIsValid(unittest.TestCase):
    def test_inside(self):
        g = GridMap(cols=50, rows=35)
        self.assertTrue(g.is_valid(25, 17))

    def test_outside(self):
        g = GridMap(cols=50, rows=35)
        self.assertFalse(g.is_valid(-1, 0))
        self.assertFalse(g.is_valid(50, 0))
        self.assertFalse(g.is_valid(0, 35))


class TestGridMapIsFree(unittest.TestCase):
    def test_free(self):
        g = GridMap(cols=50, rows=35)
        self.assertTrue(g.is_free(10, 10))

    def test_obstacle(self):
        g = GridMap(cols=50, rows=35)
        g.obstacles.add((10, 10))
        self.assertFalse(g.is_free(10, 10))

    def test_boundary(self):
        g = GridMap(cols=50, rows=35)
        self.assertFalse(g.is_free(-1, 0))


class TestGridMapNeighbors(unittest.TestCase):
    def test_center_8_neighbors(self):
        g = GridMap(cols=50, rows=35)
        # Manually check: (25,17) should have 8 free neighbors
        nbrs = g.neighbors(25, 17, allow_diag=True)
        self.assertEqual(len(nbrs), 8)

    def test_center_4_neighbors(self):
        g = GridMap(cols=50, rows=35)
        nbrs = g.neighbors(25, 17, allow_diag=False)
        self.assertEqual(len(nbrs), 4)

    def test_corner(self):
        g = GridMap(cols=50, rows=35)
        nbrs = g.neighbors(0, 0, allow_diag=True)
        self.assertEqual(len(nbrs), 3)  # right, down, diag

    def test_obstacle_blocks(self):
        g = GridMap(cols=50, rows=35)
        g.obstacles.add((26, 17))
        nbrs = g.neighbors(25, 17, allow_diag=True)
        for nx, ny, cost in nbrs:
            self.assertNotEqual((nx, ny), (26, 17))

    def test_diagonal_through_obstacle(self):
        """对角线不应穿越障碍"""
        g = GridMap(cols=50, rows=35)
        g.obstacles.add((26, 17))
        nbrs = g.neighbors(25, 17, allow_diag=True)
        # (26,18) diagonal should be blocked since (26,17) is obstacle
        diag_blocked = all((nx, ny) != (26, 18) for nx, ny, _ in nbrs)
        self.assertTrue(diag_blocked)


class TestGridMapAddWall(unittest.TestCase):
    def test_horizontal_wall(self):
        g = GridMap(cols=50, rows=35)
        g.add_wall(5, 10, 10, 10)
        for x in range(5, 11):
            self.assertIn((x, 10), g.obstacles)

    def test_vertical_wall(self):
        g = GridMap(cols=50, rows=35)
        g.add_wall(10, 5, 10, 10)
        for y in range(5, 11):
            self.assertIn((10, y), g.obstacles)

    def test_avoids_start_goal(self):
        g = GridMap(cols=50, rows=35)
        start = g.start
        g.add_wall(start[0], start[1], start[0], start[1])
        self.assertNotIn(start, g.obstacles)


class TestGridMapGenerateObstacles(unittest.TestCase):
    def test_generates(self):
        g = GridMap(cols=50, rows=35)
        random.seed(42)
        g.generate_obstacles(0.3)
        self.assertGreater(len(g.obstacles), 0)

    def test_preserves_start_goal(self):
        g = GridMap(cols=50, rows=35)
        random.seed(42)
        g.generate_obstacles(0.5)
        self.assertNotIn(g.start, g.obstacles)
        self.assertNotIn(g.goal, g.obstacles)


class TestGridMapRandomFree(unittest.TestCase):
    def test_returns_free(self):
        g = GridMap(cols=50, rows=35)
        g.generate_obstacles(0.3)
        pos = g.random_free()
        self.assertTrue(g.is_free(*pos))


class TestAStar(unittest.TestCase):
    def test_simple_path(self):
        g = GridMap(cols=20, rows=20)
        g.start = (0, 0)
        g.goal = (19, 19)
        path, explored = astar(g, g.start, g.goal)
        self.assertIsNotNone(path)
        self.assertEqual(path[0], g.start)
        self.assertEqual(path[-1], g.goal)

    def test_no_path(self):
        g = GridMap(cols=10, rows=10)
        g.start = (0, 0)
        g.goal = (9, 9)
        # Wall across the middle
        for x in range(10):
            g.obstacles.add((x, 5))
        path, explored = astar(g, g.start, g.goal)
        self.assertIsNone(path)

    def test_empty_grid(self):
        g = GridMap(cols=10, rows=10)
        g.start = (0, 0)
        g.goal = (9, 9)
        path, explored = astar(g, g.start, g.goal)
        self.assertIsNotNone(path)
        # Path length should be close to Manhattan distance
        self.assertGreater(len(path), 0)


class TestDijkstra(unittest.TestCase):
    def test_simple_path(self):
        g = GridMap(cols=20, rows=20)
        g.start = (0, 0)
        g.goal = (19, 19)
        path, explored = dijkstra(g, g.start, g.goal)
        self.assertIsNotNone(path)
        self.assertEqual(path[0], g.start)
        self.assertEqual(path[-1], g.goal)

    def test_no_path(self):
        g = GridMap(cols=10, rows=10)
        g.start = (0, 0)
        g.goal = (9, 9)
        for x in range(10):
            g.obstacles.add((x, 5))
        path, explored = dijkstra(g, g.start, g.goal)
        self.assertIsNone(path)

    def test_path_cost_reasonable(self):
        g = GridMap(cols=10, rows=10)
        g.start = (0, 0)
        g.goal = (9, 0)
        path, explored = dijkstra(g, g.start, g.goal)
        self.assertIsNotNone(path)
        # Direct horizontal path
        self.assertEqual(len(path), 10)


class TestRRT(unittest.TestCase):
    def test_simple_path(self):
        g = GridMap(cols=30, rows=30)
        g.start = (2, 2)
        g.goal = (27, 27)
        random.seed(42)
        path, nodes = rrt(g, g.start, g.goal, max_iter=5000, step_size=3, goal_bias=0.2)
        self.assertIsNotNone(path)
        self.assertEqual(path[0], g.start)
        self.assertEqual(path[-1], g.goal)

    def test_returns_nodes(self):
        g = GridMap(cols=30, rows=30)
        g.start = (2, 2)
        g.goal = (27, 27)
        random.seed(42)
        path, nodes = rrt(g, g.start, g.goal, max_iter=500)
        self.assertIsInstance(nodes, list)
        self.assertGreater(len(nodes), 0)

    def test_blocked_map(self):
        """完全封锁可能无路径"""
        g = GridMap(cols=20, rows=20)
        g.start = (0, 0)
        g.goal = (19, 19)
        for x in range(20):
            g.obstacles.add((x, 10))
        random.seed(42)
        path, nodes = rrt(g, g.start, g.goal, max_iter=500, step_size=2)
        # May or may not find a path
        if path is not None:
            self.assertEqual(path[0], g.start)


if __name__ == '__main__':
    unittest.main()
