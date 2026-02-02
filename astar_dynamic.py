"""
A* 路径规划算法 - 动态障碍物支持版本

在基础 A* 算法上添加动态障碍物避障能力，支持：
- 实时更新障碍物位置
- 动态环境中的路径重规划
- 高效的障碍物检测和处理
"""

import heapq
from typing import List, Tuple, Optional, Set, Callable


class DynamicAStar:
    """支持动态障碍物的 A* 路径规划器"""
    
    def __init__(self, width: int, height: int):
        """
        初始化动态 A* 规划器
        
        Args:
            width: 网格宽度
            height: 网格高度
        """
        self.width = width
        self.height = height
        self.obstacles: Set[Tuple[int, int]] = set()  # 动态障碍物集合
    
    def add_obstacle(self, x: int, y: int):
        """添加障碍物"""
        if 0 <= x < self.width and 0 <= y < self.height:
            self.obstacles.add((x, y))
    
    def remove_obstacle(self, x: int, y: int):
        """移除障碍物"""
        self.obstacles.discard((x, y))
    
    def clear_obstacles(self):
        """清除所有障碍物"""
        self.obstacles.clear()
    
    def update_obstacles(self, new_obstacles: List[Tuple[int, int]]):
        """批量更新障碍物"""
        self.obstacles = set()
        for x, y in new_obstacles:
            if 0 <= x < self.width and 0 <= y < self.height:
                self.obstacles.add((x, y))
    
    def is_obstacle(self, x: int, y: int) -> bool:
        """检查位置是否为障碍物"""
        return (x, y) in self.obstacles
    
    def get_neighbors(self, x: int, y: int) -> List[Tuple[int, int]]:
        """获取有效邻居节点（4方向）"""
        neighbors = []
        directions = [(0, 1), (1, 甬), (0, -1), (-1, 0)]
        
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            
            # 检查边界和障碍物
            if (0 <= nx < self.width and 0 <= ny < self.height and 
                not self.is_obstacle(nx, ny)):
                neighbors.append((nx, ny))
        
        return neighbors
    
    def heuristic(self, x1: int, y1: int, x2: int, y2: int) -> int:
        """曼哈顿距离启发式函数"""
        return abs(x1 - x2) + abs(y1 - y2)
    
    def find_path(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        """
        在动态障碍物环境中找到从起点到终点的最短路径
        
        Args:
            start: 起点坐标 (x, y)
            goal: 终点坐标 (x, y)
            
        Returns:
            路径坐标列表 [(x1, y1), (x2, y2), ...]，如果无路径则返回 None
        """
        start_x, start_y = start
        goal_x, goal_y = goal
        
        # 边界检查
        if not (0 <= start_x < self.width and 0 <= start_y < self.height):
            raise ValueError(f"起点 {start} 超出网格范围")
        if not (0 <= goal_x < self.width and 0 <= goal_y < self.height):
            raise ValueError(f"终点 {goal} 超出网格范围")
        
        # 检查起点或终点是否为障碍物
        if self.is_obstacle(start_x, start_y):
            raise ValueError("起点不能是障碍物")
        if self.is_obstacle(goal_x, goal_y):
            raise ValueError("终点不能是障碍物")
        
        # 如果起点和终点相同
        if start == goal:
            return [start]
        
        # 初始化开放列表（优先队列）
        open_set = []
        # 使用字典存储 g_score 和 f_score
        g_score = {start: 0}
        f_score = {start: self.heuristic(start_x, start_y, goal_x, goal_y)}
        
        # 将起点加入开放列表
        heapq.heappush(open_set, (f_score[start], start))
        
        # 记录父节点用于重建路径
        came_from = {}
        
        while open_set:
            # 获取 f_score 最小的节点
            current_f, current = heapq.heappop(open_set)
            
            # 如果到达目标
            if current == goal:
                return self._reconstruct_path(came_from, current)
            
            current_x, current_y = current
            
            # 检查所有邻居
            for neighbor in self.get_neighbors(current_x, current_y):
                # 计算新的 g_score（每个移动代价为1）
                tentative_g_score = g_score[current] + 1
                
                # 如果找到更好的路径
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(
                        neighbor[0], neighbor[1], goal_x, goal_y
                    )
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        # 没有找到路径
        return None
    
    def _reconstruct_path(self, came_from: dict, current: Tuple[int, int]) -> List[Tuple[int, int]]:
        """重建路径"""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]  # 反转路径，使其从起点到终点
    
    def find_path_with_dynamic_updates(
        self, 
        start: Tuple[int, int], 
        goal: Tuple[int, int],
        obstacle_callback: Callable[[], List[Tuple[int, int]]] = None,
        max_replans: int = 3
    ) -> Optional[List[Tuple[int, int]]]:
        """
        支持动态障碍物更新的路径规划
        
        Args:
            start: 起点坐标
            goal: 终点坐标  
            obstacle_callback: 回调函数，返回当前障碍物列表
            max_replans: 最大重规划次数
            
        Returns:
            路径或 None
        """
        current_start = start
        
        for attempt in range(max_replans + 1):
            # 更新障碍物（如果提供了回调函数）
            if obstacle_callback:
                current_obstacles = obstacle_callback()
                self.update_obstacles(current_obstacles)
            
            try:
                path = self.find_path(current_start, goal)
                if path:
                    return path
            except ValueError:
                # 起点或终点被障碍物阻挡，尝试下一步
                pass
            
            # 如果是最后一次尝试，返回 None
            if attempt == max_replans:
                return None
            
            # 等待下一帧或状态更新（这里简单地继续循环）
            # 在实际应用中，这里可能会等待传感器数据更新等
        
        return None


# 便捷函数
def find_path_with_dynamic_obstacles(
    width: int,
    height: int,
    start: Tuple[int, int],
    goal: Tuple[int, int],
    obstacles: List[Tuple[int, int]] = None
) -> Optional[List[Tuple[int, int]]]:
    """
    动态障碍物环境中的路径查找便捷函数
    """
    astar = DynamicAStar(width, height)
    if obstacles:
        astar.update_obstacles(obstacles)
    return astar.find_path(start, goal)