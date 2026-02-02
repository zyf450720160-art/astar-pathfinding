"""
高性能 A* 路径规划算法实现

优化要点：
- 使用元组代替 Node 对象，减少内存开销
- 使用字典存储 g_score 和 f_score，提高查找效率  
- 避免预分配整个网格，按需创建节点
- 内联关键操作，减少函数调用开销
- 使用集合进行 O(1) 的开放集查找
"""

import heapq
from typing import List, Tuple, Optional, Set


def heuristic(a: Tuple[int, int], b: Tuple[int, int]) -> int:
    """曼哈顿距离启发式函数"""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def get_neighbors(pos: Tuple[int, int], width: int, height: int, obstacles: Set[Tuple[int, int]]) -> List[Tuple[int, int]]:
    """获取有效邻居节点（4方向）"""
    x, y = pos
    neighbors = []
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
    
    for dx, dy in directions:
        nx, ny = x + dx, y + dy
        if 0 <= nx < width and 0 <= ny < height and (nx, ny) not in obstacles:
            neighbors.append((nx, ny))
    
    return neighbors


def find_path_in_grid(
    width: int, 
    height: int, 
    start: Tuple[int, int], 
    goal: Tuple[int, int], 
    obstacles: List[Tuple[int, int]] = None
) -> Optional[List[Tuple[int, int]]]:
    """
    在网格中查找路径的高性能实现
    
    Args:
        width: 网格宽度
        height: 网格高度
        start: 起点坐标 (x, y)
        goal: 终点坐标 (x, y)  
        obstacles: 障碍物坐标列表
        
    Returns:
        路径坐标列表，如果无路径则返回 None
    """
    # 输入验证
    if not (0 <= start[0] < width and 0 <= start[1] < height):
        raise ValueError(f"起点 {start} 超出网格范围")
    if not (0 <= goal[0] < width and 0 <= goal[1] < height):
        raise ValueError(f"终点 {goal} 超出网格范围")
    
    obstacle_set = set(obstacles) if obstacles else set()
    
    if start in obstacle_set:
        raise ValueError("起点不能是障碍物")
    if goal in obstacle_set:
        raise ValueError("终点不能是障碍物")
    
    if start == goal:
        return [start]
    
    # 初始化数据结构
    open_set = []  # 优先队列：(f_score, position)
    open_set_hash = {start}  # 用于 O(1) 查找
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}
    came_from = {}  # 用于重建路径
    
    heapq.heappush(open_set, (f_score[start], start))
    
    while open_set:
        current = heapq.heappop(open_set)[1]
        open_set_hash.remove(current)
        
        if current == goal:
            # 重建路径
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]
        
        for neighbor in get_neighbors(current, width, height, obstacle_set):
            tentative_g_score = g_score[current] + 1
            
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                
                if neighbor not in open_set_hash:
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
                    open_set_hash.add(neighbor)
    
    return None


class AStar:
    """A* 路径规划器（兼容原始接口）"""
    
    def __init__(self, width: int, height: int, obstacles: List[Tuple[int, int]] = None):
        self.width = width
        self.height = height
        self.obstacles = obstacles or []
    
    def find_path(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        return find_path_in_grid(self.width, self.height, start, goal, self.obstacles)