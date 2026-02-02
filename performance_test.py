#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
A* 算法性能基准测试
对比原始版本和优化版本的性能差异
"""

import time
import random
from astar import find_path_in_grid as astar_original
from astar_optimized import find_path_in_grid as astar_optimized


def generate_obstacles(width, height, obstacle_ratio=0.2):
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


def safe_find_path(func, width, height, start, goal, obstacles, max_retries=5):
    """安全地查找路径，避免起点/终点为障碍物"""
    for _ in range(max_retries):
        # 确保起点和终点不是障碍物
        obstacles_set = set(obstacles)
        if start in obstacles_set:
            obstacles_set.remove(start)
        if goal in obstacles_set:
            obstacles_set.remove(goal)
        
        try:
            start_time = time.time()
            path = func(width, height, start, goal, list(obstacles_set))
            end_time = time.time()
            return end_time - start_time, path
        except ValueError as e:
            # 如果起点或终点是障碍物，重新生成
            if "起点" in str(e) or "终点" in str(e):
                # 尝试移动起点或终点
                start = (random.randint(0, width-1), random.randint(0, height-1))
                goal = (random.randint(0, width-1), random.randint(0, height-1))
                continue
            else:
                raise e
    
    return None, None


def benchmark_astar_versions():
    """基准测试函数"""
    print("=" * 60)
    print("A* 算法性能优化基准测试")
    print("=" * 60)
    
    test_cases = [
        (20, 20, 0.2),
        (50, 50, 0.25),
        # (100, 100, 0.2)  # 大网格测试可能较慢
    ]
    
    total_speedup = 0
    valid_tests = 0
    
    for width, height, obstacle_ratio in test_cases:
        print(f"\n测试网格大小: {width}x{height}, 障碍物比例: {obstacle_ratio:.2%}")
        
        # 生成测试数据
        obstacles = generate_obstacles(width, height, obstacle_ratio)
        start = (0, 0)
        goal = (width - 1, height - 1)
        
        # 测试原始版本
        original_times = []
        for _ in range(3):  # 运行3次取平均
            elapsed, path = safe_find_path(
                astar_original, width, height, start, goal, obstacles
            )
            if elapsed is not None:
                original_times.append(elapsed)
        
        if not original_times:
            print("原始版本测试失败: 起点不能是障碍物")
            continue
        
        avg_original = sum(original_times) / len(original_times)
        
        # 测试优化版本
        optimized_times = []
        for _ in range(3):  # 运行3次取平均
            elapsed, path = safe_find_path(
                astar_optimized, width, height, start, goal, obstacles
            )
            if elapsed is not None:
                optimized_times.append(elapsed)
        
        if not optimized_times:
            print("优化版本测试失败: 终点不能是障碍物")
            continue
        
        avg_optimized = sum(optimized_times) / len(optimized_times)
        
        speedup = avg_original / avg_optimized
        total_speedup += speedup
        valid_tests += 1
        
        print(f"原始版本平均时间: {avg_original:.6f} 秒")
        print(f"优化版本平均时间: {avg_optimized:.6f} 秒")
        print(f"性能提升: {speedup:.2f}x")
        print("-" * 50)
    
    if valid_tests > 0:
        overall_speedup = total_speedup / valid_tests
        print(f"\n总体平均性能提升: {overall_speedup:.2f}x")
    
    print("\n✅ 性能测试完成！")


if __name__ == "__main__":
    benchmark_astar_versions()