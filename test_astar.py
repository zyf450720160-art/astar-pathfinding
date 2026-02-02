"""
A* 算法测试套件
"""

import unittest
from astar import AStar, find_path_in_grid


class TestAStar(unittest.TestCase):
    """A* 算法测试类"""
    
    def test_simple_path(self):
        """测试简单路径规划"""
        # 创建 5x5 网格，无障碍物
        astar = AStar(5, 5)
        path = astar.find_path((0, 0), (4, 4))
        
        self.assertIsNotNone(path)
        self.assertEqual(path[0], (0, 0))
        self.assertEqual(path[-1], (4, 4))
        print(f"简单路径测试通过: 路径长度 {len(path)}")
    
    def test_with_obstacles(self):
        """测试有障碍物的情况"""
        # 创建障碍物
        obstacles = [(2, 2), (2, 3), (3, 2)]
        astar = AStar(5, 5, obstacles)
        path = astar.find_path((0, 0), (4, 4))
        
        self.assertIsNotNone(path)
        # 检查路径是否避开障碍物
        for x, y in path:
            self.assertNotIn((x, y), obstacles)
        print(f"障碍物测试通过: 路径长度 {len(path)}")
    
    def test_no_path(self):
        """测试无路径情况"""
        # 创建完全阻塞的障碍物墙
        obstacles = [(2, y) for y in range(5)]  # 垂直墙壁
        astar = AStar(5, 5, obstacles)
        path = astar.find_path((0, 0), (4, 4))
        
        self.assertIsNone(path)
        print("无路径测试通过")
    
    def test_same_start_goal(self):
        """测试起点等于终点"""
        astar = AStar(5, 5)
        path = astar.find_path((2, 2), (2, 2))
        
        self.assertIsNotNone(path)
        self.assertEqual(len(path), 1)
        self.assertEqual(path[0], (2, 2))
        print("起点终点相同测试通过")
    
    def test_edge_cases(self):
        """测试边界情况"""
        astar = AStar(10, 10)
        
        # 测试角落到角落
        path1 = astar.find_path((0, 0), (9, 9))
        self.assertIsNotNone(path1)
        
        # 测试边缘点
        path2 = astar.find_path((0, 5), (9, 5))
        self.assertIsNotNone(path2)
        
        print("边界情况测试通过")
    
    def test_invalid_positions(self):
        """测试无效位置"""
        astar = AStar(5, 5)
        
        # 测试超出边界的起点
        with self.assertRaises(ValueError):
            astar.find_path((-1, 0), (2, 2))
        
        # 测试超出边界的终点
        with self.assertRaises(ValueError):
            astar.find_path((2, 2), (10, 10))
        
        print("无效位置测试通过")
    
    def test_obstacle_at_start_or_goal(self):
        """测试起点或终点为障碍物"""
        obstacles = [(0, 0)]
        astar = AStar(5, 5, obstacles)
        
        # 起点为障碍物
        with self.assertRaises(ValueError):
            astar.find_path((0, 0), (4, 4))
        
        # 终点为障碍物
        obstacles2 = [(4, 4)]
        astar2 = AStar(5, 5, obstacles2)
        with self.assertRaises(ValueError):
            astar2.find_path((0, 0), (4, 4))
        
        print("障碍物在起点/终点测试通过")
    
    def test_convenience_function(self):
        """测试便捷函数"""
        path = find_path_in_grid(
            width=5, 
            height=5, 
            start=(0, 0), 
            goal=(4, 4),
            obstacles=[(2, 2), (2, 3)]
        )
        
        self.assertIsNotNone(path)
        self.assertEqual(path[0], (0, 0))
        self.assertEqual(path[-1], (4, 4))
        print("便捷函数测试通过")
    
    def test_complex_maze(self):
        """测试复杂迷宫"""
        # 创建复杂障碍物布局
        obstacles = [
            (1, 1), (1, 2), (1, 3),
            (2, 1), (2, 3),
            (3, 1), (3, 2), (3, 3),
            (5, 2), (5, 3), (5, 4),
            (6, 2), (6, 4),
            (7, 2), (7, 3), (7, 4)
        ]
        astar = AStar(10, 10, obstacles)
        path = astar.find_path((0, 5), (9, 5))
        
        self.assertIsNotNone(path)
        # 验证路径避开所有障碍物
        for x, y in path:
            self.assertNotIn((x, y), obstacles)
        print(f"复杂迷宫测试通过: 路径长度 {len(path)}")


def demo():
    """演示 A* 算法的使用"""
    print("\n" + "="*50)
    print("A* 算法演示")
    print("="*50)
    
    # 创建一个带障碍物的网格
    obstacles = [
        (2, 2), (2, 3), (2, 4),
        (3, 2),
        (4, 2), (4, 3), (4, 4)
    ]
    
    astar = AStar(8, 8, obstacles)
    
    # 查找路径
    start = (1, 1)
    goal = (6, 6)
    path = astar.find_path(start, goal)
    
    if path:
        print(f"从 {start} 到 {goal} 找到路径:")
        print(f"路径: {path}")
        print(f"路径长度: {len(path)} 步")
        
        # 可视化网格
        print("\n网格可视化 (S=起点, G=终点, X=障碍物, *=路径):")
        grid = [['.' for _ in range(8)] for _ in range(8)]
        
        # 标记障碍物
        for x, y in obstacles:
            grid[x][y] = 'X'
        
        # 标记路径
        for x, y in path:
            if (x, y) != start and (x, y) != goal:
                grid[x][y] = '*'
        
        # 标记起点和终点
        sx, sy = start
        gx, gy = goal
        grid[sx][sy] = 'S'
        grid[gx][gy] = 'G'
        
        # 打印网格
        for row in grid:
            print(' '.join(row))
    else:
        print(f"从 {start} 到 {goal} 未找到路径")


if __name__ == "__main__":
    # 运行演示
    demo()
    
    # 运行单元测试
    print("\n" + "="*50)
    print("运行单元测试")
    print("="*50)
    
    unittest.main(verbosity=2)