"""
A* 路径规划算法实现

A* 算法是一种启发式搜索算法，用于在图中找到从起点到终点的最短路径。
它结合了 Dijkstra 算法的精确性和贪心最佳优先搜索的效率。

主要特点：
- 使用启发式函数（通常是曼哈顿距离或欧几里得距离）
- 保证找到最优解（如果启发式函数是可接受的）
- 效率比 Dijkstra 算法更高
"""

import heapq
from typing import List, Tuple, Optional, Callable, Set


class Node:
    """网格中的节点类"""
    
    def __init__(self, x: int, y: int):
        self.x = x
        self.y = y
        self.g_cost = float('inf')  # 从起点到当前节点的实际代价
        self.h_cost = 0             # 从当前节点到目标的启发式估计代价
        self.f_cost = float('inf')  # f = g + h
        self.parent = None          # 父节点，用于重建路径
        self.is_obstacle = False    # 是否为障碍物
    
    def __lt__(self, other):
        """用于优先队列比较"""
        return self.f_cost < other.f_cost
    
    def __eq__(self, other):
        """判断两个节点是否相等"""
        if isinstance(other, Node):
            return self.x == other.x and self.y == other.y
        return False
    
    def __hash__(self):
        """使节点可用于集合和字典"""
        return hash((self.x, self.y))
    
    def distance_to(self, other) -> float:
        """计算到另一个节点的欧几里得距离"""
        return ((self.x - other.x) ** 2 + (self.y - other.y) ** 2) ** 0.5
    
    def manhattan_distance_to(self, other) -> int:
        """计算到另一个节点的曼哈顿距离"""
        return abs(self.x - other.x) + abs(self.y - other.y)


class AStar:
    """A* 路径规划器"""
    
    def __init__(self, width: int, height: int, obstacles: List[Tuple[int, int]] = None):
        """
        初始化 A* 规划器
        
        Args:
            width: 网格宽度
            height: 网格高度  
            obstacles: 障碍物坐标列表 [(x1, y1), (x2, y2), ...]
        """
        self.width = width
        self.height = height
        self.grid = [[Node(x, y) for y in range(height)] for x in range(width)]
        
        # 设置障碍物
        if obstacles:
            for x, y in obstacles:
                if 0 <= x < width and 0 <= y < height:
                    self.grid[x][y].is_obstacle = True
    
    def set_obstacle(self, x: int, y: int, is_obstacle: bool = True):
        """设置或清除障碍物"""
        if 0 <= x < self.width and 0 <= y < self.height:
            self.grid[x][y].is_obstacle = is_obstacle
    
    def get_neighbors(self, node: Node) -> List[Node]:
        """
        获取节点的邻居（4方向或8方向）
        这里使用4方向移动（上下左右）
        """
        neighbors = []
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # 4方向
        
        for dx, dy in directions:
            nx, ny = node.x + dx, node.y + dy
            
            # 检查边界
            if 0 <= nx < self.width and 0 <= ny < self.height:
                neighbor = self.grid[nx][ny]
                # 只有非障碍物节点才能作为邻居
                if not neighbor.is_obstacle:
                    neighbors.append(neighbor)
        
        return neighbors
    
    def heuristic(self, node: Node, goal: Node) -> float:
        """
        启发式函数：使用曼哈顿距离
        对于网格地图，曼哈顿距离通常比欧几里得距离更合适
        """
        return node.manhattan_distance_to(goal)
    
    def find_path(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        """
        找到从起点到终点的最短路径
        
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
        
        start_node = self.grid[start_x][start_y]
        goal_node = self.grid[goal_x][goal_y]
        
        # 检查起点或终点是否为障碍物
        if start_node.is_obstacle:
            raise ValueError("起点不能是障碍物")
        if goal_node.is_obstacle:
            raise ValueError("终点不能是障碍物")
        
        # 如果起点和终点相同
        if start_node == goal_node:
            return [start]
        
        # 重置所有节点的状态
        for row in self.grid:
            for node in row:
                node.g_cost = float('inf')
                node.f_cost = float('inf')
                node.parent = None
        
        # 初始化开放列表（优先队列）
        open_set = []
        start_node.g_cost = 0
        start_node.h_cost = self.heuristic(start_node, goal_node)
        start_node.f_cost = start_node.g_cost + start_node.h_cost
        heapq.heappush(open_set, start_node)
        
        # 已访问节点集合
        closed_set: Set[Node] = set()
        
        while open_set:
            # 获取 f_cost 最小的节点
            current = heapq.heappop(open_set)
            
            # 如果到达目标
            if current == goal_node:
                return self._reconstruct_path(current)
            
            closed_set.add(current)
            
            # 检查所有邻居
            for neighbor in self.get_neighbors(current):
                if neighbor in closed_set:
                    continue
                
                # 计算新的 g_cost（这里假设每个移动的代价为1）
                tentative_g_cost = current.g_cost + 1
                
                if tentative_g_cost < neighbor.g_cost:
                    # 找到更好的路径
                    neighbor.parent = current
                    neighbor.g_cost = tentative_g_cost
                    neighbor.h_cost = self.heuristic(neighbor, goal_node)
                    neighbor.f_cost = neighbor.g_cost + neighbor.h_cost
                    
                    # 如果邻居不在开放列表中，添加它
                    if neighbor not in open_set:
                        heapq.heappush(open_set, neighbor)
        
        # 没有找到路径
        return None
    
    def _reconstruct_path(self, node: Node) -> List[Tuple[int, int]]:
        """重建路径"""
        path = []
        current = node
        while current is not None:
            path.append((current.x, current.y))
            current = current.parent
        return path[::-1]  # 反转路径，使其从起点到终点


# 便捷函数
def find_path_in_grid(
    width: int, 
    height: int, 
    start: Tuple[int, int], 
    goal: Tuple[int, int], 
    obstacles: List[Tuple[int, int]] = None
) -> Optional[List[Tuple[int, int]]]:
    """
    在网格中查找路径的便捷函数
    
    Args:
        width: 网格宽度
        height: 网格高度
        start: 起点坐标 (x, y)
        goal: 终点坐标 (x, y)
        obstacles: 障碍物坐标列表
        
    Returns:
        路径坐标列表，如果无路径则返回 None
    """
    astar = AStar(width, height, obstacles)
    return astar.find_path(start, goal)