"""
优化版 A* 路径规划算法实现

主要优化点：
1. 使用坐标元组代替 Node 对象，减少内存开销
2. 使用字典存储 g_cost 和 f_cost，提高查找效率  
3. 预计算启发式距离，避免重复计算
4. 使用更高效的开放列表管理
5. 早期终止条件优化
"""

import heapq
from typing import List, Tuple, Optional, Dict, Set


class AStarOptimized:
    """优化版 A* 路径规划器"""
    
    def __init__(self, width: int, height: int, obstacles: List[Tuple[int, int]] = None):
        """
        初始化优化版 A* 规划器
        
        Args:
            width: 网格宽度
            height: 网格高度  
            obstacles: 障碍物坐标集合
        """
        self.width = width
        self.height = height
        # 使用 frozenset 提高查找效率
        self.obstacles = frozenset(obstacles) if obstacles else frozenset()
    
    def is_valid_position(self, x: int, y: int) -> bool:
        """检查位置是否有效（在边界内且不是障碍物）"""
        return (0 <= x < self.width and 
                0 <= y < self.height and 
                (x, y) not in self.obstacles)
    
    def get_neighbors(self, x: int, y: int) -> List[Tuple[int, int]]:
        """
        获取节点的邻居（4方向）
        直接返回坐标元组，避免对象创建开销
        """
        neighbors = []
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if self.is_valid_position(nx, ny):
                neighbors.append((nx, ny))
        
        return neighbors
    
    def manhattan_distance(self, pos1: Tuple[int, int], pos2: Tuple[int, int]) -> int:
        """曼哈顿距离计算（内联优化）"""
        return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])
    
    def find_path(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        """
        优化版路径查找
        
        主要优化：
        - 使用坐标元组代替对象
        - 字典存储成本值
        - 更高效的开放列表
        - 早期终止
        """
        # 边界检查
        if not self.is_valid_position(*start):
            raise ValueError(f"起点 {start} 无效（超出边界或为障碍物）")
        if not self.is_valid_position(*goal):
            raise ValueError(f"终点 {goal} 无效（超出边界或为障碍物）")
        
        # 起点等于终点
        if start == goal:
            return [start]
        
        # 使用字典存储 g_cost 和 parent 信息
        g_costs: Dict[Tuple[int, int], float] = {start: 0.0}
        parents: Dict[Tuple[int, int], Tuple[int, int]] = {}
        
        # 开放列表：(f_cost, h_cost, x, y)
        # 注意：包含 h_cost 作为第二优先级，避免相同 f_cost 时比较元组
        open_heap = [(self.manhattan_distance(start, goal), 0, start[0], start[1])]
        
        # 已访问集合
        closed_set: Set[Tuple[int, int]] = set()
        
        goal_h = 0  # 到目标的启发式距离
        
        while open_heap:
            f_cost, h_cost, x, y = heapq.heappop(open_heap)
            current = (x, y)
            
            # 如果已经处理过这个节点，跳过
            if current in closed_set:
                continue
            
            # 到达目标
            if current == goal:
                return self._reconstruct_path(parents, start, goal)
            
            closed_set.add(current)
            
            # 检查邻居
            for neighbor in self.get_neighbors(x, y):
                if neighbor in closed_set:
                    continue
                
                # 计算新的 g_cost（移动代价为1）
                tentative_g = g_costs[current] + 1.0
                
                # 如果找到更好的路径或这是第一次访问
                if neighbor not in g_costs or tentative_g < g_costs[neighbor]:
                    g_costs[neighbor] = tentative_g
                    parents[neighbor] = current
                    
                    # 计算启发式距离和 f_cost
                    goal_h = self.manhattan_distance(neighbor, goal)
                    f_cost = tentative_g + goal_h
                    
                    heapq.heappush(open_heap, (f_cost, goal_h, neighbor[0], neighbor[1]))
        
        return None
    
    def _reconstruct_path(self, parents: Dict[Tuple[int, int], Tuple[int, int]], 
                         start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        """重建路径（优化版）"""
        path = []
        current = goal
        
        while current != start:
            path.append(current)
            current = parents[current]
        
        path.append(start)
        path.reverse()
        return path


# 极简版本 - 适用于简单场景
def astar_simple(
    width: int, 
    height: int, 
    start: Tuple[int, int], 
    goal: Tuple[int, int], 
    obstacles: Set[Tuple[int, int]]
) -> Optional[List[Tuple[int, int]]]:
    """
    极简 A* 实现，适用于性能要求极高的场景
    
    注意：此版本不包含完整的错误检查，假设输入都是有效的
    """
    if start == goal:
        return [start]
    
    g_costs = {start: 0}
    parents = {}
    open_heap = [(abs(start[0] - goal[0]) + abs(start[1] - goal[1]), start)]
    closed = set()
    
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
    
    while open_heap:
        _, current = heapq.heappop(open_heap)
        
        if current in closed:
            continue
            
        if current == goal:
            # 重建路径
            path = []
            c = goal
            while c != start:
                path.append(c)
                c = parents[c]
            path.append(start)
            return path[::-1]
        
        closed.add(current)
        
        for dx, dy in directions:
            nx, ny = current[0] + dx, current[1] + dy
            neighbor = (nx, ny)
            
            if (0 <= nx < width and 0 <= ny < height and 
                neighbor not in obstacles and neighbor not in closed):
                
                tentative_g = g_costs[current] + 1
                
                if neighbor not in g_costs or tentative_g < g_costs[neighbor]:
                    g_costs[neighbor] = tentative_g
                    parents[neighbor] = current
                    h = abs(nx - goal[0]) + abs(ny - goal[1])
                    heapq.heappush(open_heap, (tentative_g + h, neighbor))
    
    return None


# 便捷函数
def find_path_optimized(
    width: int, 
    height: int, 
    start: Tuple[int, int], 
    goal: Tuple[int, int], 
    obstacles: List[Tuple[int, int]] = None
) -> Optional[List[Tuple[int, int]]]:
    """
    优化版便捷函数
    """
    astar = AStarOptimized(width, height, obstacles)
    return astar.find_path(start, goal)