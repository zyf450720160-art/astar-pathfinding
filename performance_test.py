#!/usr/bin/env python3
"""
A* 算法性能测试对比
比较原始版本和优化版本的性能差异
"""

import time
import random
from typing import List, Tuple
import sys
import os

# 添加当前目录到 Python 路径
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from astar import find_path_in_grid as astar_original
from astar_optimized import AStarOptimized


def generate_random_obstacles(width: int, height: int, obstacle_ratio: float = 0.3) -> List[Tuple[int, int]]:
    """生成随机障碍物"""
    obstacles = []
    total_cells = width * height
    num_obstacles = int(total_cells * obstacle_ratio)
    
    for _ in range(num_obstacles):
        x = random.randint(0, width - 1)
        y = random.randint(0, height - 1)
        if (x, y) not in obstacles:
            obstacles.append((x, y))
    
    return obstacles


def benchmark_astar(width: int, height: int, start: Tuple[int, int], goal: Tuple[int, int], 
                   obstacles: List[Tuple[int, int]], iterations: int = 10) -> float:
    """基准测试函数"""
    total_time = 0.0
    
    # 测试原始版本
    print(f"测试网格大小: {width}x{height}, 障碍物比例: {len(obstacles)/(width*height):.2%}")
    
    for i in range(iterations):
        start_time = time.perf_counter()
        try:
            path = astar_original(width, height, start, goal, obstacles)
            end_time = time.perf_counter()
            total_time += (end_time - start_time)
        except Exception as e:
            print(f"原始版本测试失败: {e}")
            return float('inf')
    
    avg_time_original = total_time / iterations
    
    # 测试优化版本
    total_time = 0.0
    for i in range(iterations):
        start_time = time.perf_counter()
        try:
            astar = AStarOptimized(width, height, obstacles)
            path = astar.find_path(start, goal)
            end_time = time.perf_counter()
            total_time += (end_time - start_time)
        except Exception as e:
            print(f"优化版本测试失败: {e}")
            return float('inf')
    
    avg_time_optimized = total_time / iterations
    
    speedup = avg_time_original / avg_time_optimized if avg_time_optimized > 0 else float('inf')
    
    print(f"原始版本平均时间: {avg_time_original:.6f} 秒")
    print(f"优化版本平均时间: {avg_time_optimized:.6f} 秒")
    print(f"性能提升: {speedup:.2f}x")
    print("-" * 50)
    
    return speedup


def run_comprehensive_benchmark():
    """运行全面的性能基准测试"""
    print("=" * 60)
    print("A* 算法性能优化基准测试")
    print("=" * 60)
    
    test_cases = [
        # (width, height, start, goal, obstacle_ratio, iterations)
        (20, 20, (0, 0), (19, 19), 0.2, 50),
        (50, 50, (5, 5), (45, 45), 0.3, 20),
        (100, 100, (10, 10), (90, 90), 0.25, 10),
        (20, 20, (0, 0), (19, 19), 0.4, 50),  # 高障碍物密度
    ]
    
    total_speedup = 0
    valid_tests = 0
    
    for width, height, start, goal, obstacle_ratio, iterations in test_cases:
        obstacles = generate_random_obstacles(width, height, obstacle_ratio)
        speedup = benchmark_astar(width, height, start, goal, obstacles, iterations)
        if speedup != float('inf'):
            total_speedup += speedup
            valid_tests += 1
    
    if valid_tests > 0:
        avg_speedup = total_speedup / valid_tests
        print(f"\n总体平均性能提升: {avg_speedup:.2f}x")
    
    print("\n✅ 性能测试完成！")


if __name__ == "__main__":
    run_comprehensive_benchmark()