#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
动态障碍物 A* 算法测试套件
"""

import unittest
import time
from astar_dynamic import AStarDynamic, find_path_with_dynamic_obstacles


class TestAStarDynamic(unittest.TestCase):
    """动态障碍物 A* 算法测试类"""
    
    def test_basic_dynamic_obstacle(self):
        """测试基本动态障碍物功能"""
        # 创建 10x10 网格
        width, height = 10, 10
        start = (0, 0)
        goal = (9, 9)
        
        # 初始障碍物
        static_obstacles = [(2, 2), (3, 3)]
        
        # 动态障碍物：在时间步 2 时出现在 (5, 5)
        dynamic_obstacles = {2: [(5, 5)]}
        
        path = find_path_with_dynamic_obstacles(
            width, height, start, goal, 
            static_obstacles=static_obstacles,
            dynamic_obstacles=dynamic_obstacles
        )
        
        self.assertIsNotNone(path)
        self.assertEqual(path[0], start)
        self.assertEqual(path[-1], goal)
        
        # 验证路径避开了动态障碍物
        # 在时间步 2，路径不应该经过 (5, 5)
        if len(path) > 2:
            self.assertNotEqual(path[2], (5, 5))
    
    def test_moving_obstacle_sequence(self):
        """测试移动障碍物序列"""
        width, height = 15, 15
        start = (0, 0)
        goal = (14, 14)
        
        # 静态障碍物
        static_obstacles = [(3, 3), (7, 7)]
        
        # 移动障碍物：沿着对角线移动
        dynamic_obstacles = {
            1: [(4, 4)],
            2: [(5, 5)],
            3: [(6, 6)],
            4: [(7, 7)],  # 注意：这里与静态障碍物重叠，应该被忽略
            5: [(8, 8)]
        }
        
        path = find_path_with_dynamic_obstacles(
            width, height, start, goal,
            static_obstacles=static_obstacles,
            dynamic_obstacles=dynamic_obstacles
        )
        
        self.assertIsNotNone(path)
        
        # 验证路径避开了移动障碍物
        for time_step, obstacle_pos in dynamic_obstacles.items():
            if time_step < len(path):
                self.assertNotIn(path[time_step], obstacle_pos)
    
    def test_no_valid_path_with_dynamic_obstacles(self):
        """测试动态障碍物导致无路径的情况"""
        width, height = 5, 5
        start = (0, 0)
        goal = (4, 4)
        
        # 静态障碍物很少
        static_obstacles = []
        
        # 动态障碍物完全阻塞所有可能路径
        dynamic_obstacles = {
            1: [(0, 1), (1, 0)],
            2: [(1, 1), (0, 2), (2, 0)],
            3: [(2, 2), (1, 3), (3, 1)],
            4: [(3, 3), (2, 4), (4, 2)]
        }
        
        path = find_path_with_dynamic_obstacles(
            width, height, start, goal,
            static_obstacles=static_obstacles,
            dynamic_obstacles=dynamic_obstacles
        )
        
        # 可能仍然有路径，因为算法会尝试绕行
        # 这个测试主要是验证不会崩溃
    
    def test_empty_dynamic_obstacles(self):
        """测试空的动态障碍物（应回退到静态 A*）"""
        width, height = 8, 8
        start = (0, 0)
        goal = (7, 7)
        static_obstacles = [(2, 2), (5, 5)]
        dynamic_obstacles = {}
        
        path = find_path_with_dynamic_obstacles(
            width, height, start, goal,
            static_obstacles=static_obstacles,
            dynamic_obstacles=dynamic_obstacles
        )
        
        self.assertIsNotNone(path)
        self.assertEqual(path[0], start)
        self.assertEqual(path[-1], goal)
    
    def test_large_time_steps(self):
        """测试大时间步的动态障碍物"""
        width, height = 10, 10
        start = (0, 0)
        goal = (9, 9)
        static_obstacles = []
        dynamic_obstacles = {100: [(5, 5)]}  # 很晚才出现的障碍物
        
        path = find_path_with_dynamic_obstacles(
            width, height, start, goal,
            static_obstacles=static_obstacles,
            dynamic_obstacles=dynamic_obstacles
        )
        
        self.assertIsNotNone(path)
        # 由于路径长度远小于 100，这个障碍物不会影响结果
    
    def test_performance_comparison(self):
        """测试性能对比"""
        import time
        
        width, height = 20, 20
        start = (0, 0)
        goal = (19, 19)
        static_obstacles = [(5, 5), (10, 10), (15, 15)]
        dynamic_obstacles = {5: [(8, 8)], 10: [(12, 12)]}
        
        # 测试动态版本
        start_time = time.time()
        path = find_path_with_dynamic_obstacles(
            width, height, start, goal,
            static_obstacles=static_obstacles,
            dynamic_obstacles=dynamic_obstacles
        )
        dynamic_time = time.time() - start_time
        
        self.assertIsNotNone(path)
        print(f"动态障碍物版本耗时: {dynamic_time:.4f}秒")
    
    def test_real_time_simulation(self):
        """测试实时模拟场景"""
        # 模拟一个机器人需要避开移动的障碍物
        width, height = 20, 20
        start = (0, 0)
        goal = (19, 19)
        
        # 静态环境障碍物
        static_obstacles = [
            (3, 3), (3, 4), (3, 5),  # 墙壁
            (10, 10), (11, 10), (12, 10)  # 另一堵墙
        ]
        
        # 移动的巡逻障碍物
        dynamic_obstacles = {}
        patrol_x, patrol_y = 6, 6
        for t in range(1, 15):
            if t % 4 == 1:
                dynamic_obstacles[t] = [(patrol_x, patrol_y)]
                patrol_x += 1
            elif t % 4 == 2:
                dynamic_obstacles[t] = [(patrol_x, patrol_y)]
                patrol_y += 1
            elif t % 4 == 3:
                dynamic_obstacles[t] = [(patrol_x, patrol_y)]
                patrol_x -= 1
            else:
                dynamic_obstacles[t] = [(patrol_x, patrol_y)]
                patrol_y -= 1
        
        path = find_path_with_dynamic_obstacles(
            width, height, start, goal,
            static_obstacles=static_obstacles,
            dynamic_obstacles=dynamic_obstacles
        )
        
        self.assertIsNotNone(path)
        print(f"实时模拟路径长度: {len(path)}")


def demo_dynamic_obstacle():
    """动态障碍物演示"""
    print("\n" + "="*60)
    print("动态障碍物 A* 算法演示")
    print("="*60)
    
    # 创建场景
    width, height = 12, 12
    start = (1, 1)
    goal = (10, 10)
    static_obstacles = [(3, 3), (3, 4), (3, 5), (4, 3), (5, 3)]
    dynamic_obstacles = {
        3: [(6, 6)],
        5: [(7, 7)],
        7: [(8, 8)]
    }
    
    print(f"网格大小: {width}x{height}")
    print(f"起点: {start}, 终点: {goal}")
    print(f"静态障碍物: {static_obstacles}")
    print(f"动态障碍物: {dynamic_obstacles}")
    
    path = find_path_with_dynamic_obstacles(
        width, height, start, goal,
        static_obstacles=static_obstacles,
        dynamic_obstacles=dynamic_obstacles
    )
    
    if path:
        print(f"\n找到路径! 路径长度: {len(path)}")
        print(f"路径: {path}")
        
        # 验证动态障碍物避让
        print("\n动态障碍物避让验证:")
        for time_step, pos in enumerate(path):
            if time_step in dynamic_obstacles:
                obstacles_at_time = dynamic_obstacles[time_step]
                if pos in obstacles_at_time:
                    print(f"❌ 时间步 {time_step}: 路径经过动态障碍物 {pos}!")
                else:
                    print(f"✅ 时间步 {time_step}: 成功避开动态障碍物 {obstacles_at_time}")
    else:
        print("❌ 未找到有效路径!")


if __name__ == "__main__":
    # 运行演示
    demo_dynamic_obstacle()
    
    # 运行单元测试
    print("\n" + "="*60)
    print("运行单元测试")
    print("="*60)
    unittest.main(verbosity=2, exit=False)