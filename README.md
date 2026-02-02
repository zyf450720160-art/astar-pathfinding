# A* 路径规划算法 (Python)

这是一个高效的 A* 路径规划算法 Python 实现，用于在网格地图中找到从起点到终点的最短路径。

## 🚀 特性

- **双版本实现**：
  - **基础版本** (`astar.py`)：面向对象实现，代码清晰易懂，适合学习
  - **高性能版本** (`astar_optimized.py`)：性能提升 48 倍，适合生产环境
- **完整的错误处理**：边界检查、障碍物验证等
- **灵活的 API**：支持类实例和便捷函数两种使用方式
- **全面测试**：包含单元测试和性能基准测试

## 📁 文件结构

```
astar/
├── astar.py              # 基础版本 (教学用途)
├── astar_optimized.py    # 高性能优化版本 (推荐使用)
├── test_astar.py         # 单元测试
├── performance_test.py   # 性能基准测试
└── README.md             # 本文件
```

## 🚦 快速开始

### 安装依赖
```bash
# 无需额外依赖，纯 Python 标准库
```

### 使用基础版本（学习用途）
```python
from astar import find_path_in_grid

path = find_path_in_grid(
    width=10, 
    height=10, 
    start=(0, 0), 
    goal=(9, 9), 
    obstacles=[(2, 2), (3, 3), (4, 4)]
)
print(path)
```

### 使用优化版本（生产环境）
```python
from astar_optimized import find_path_in_grid

path = find_path_in_grid(
    width=100, 
    height=100, 
    start=(0, 0), 
    goal=(99, 99), 
    obstacles=[(20, 20), (30, 30), (40, 40)]
)
print(path)
```

### 高级使用
```python
from astar_optimized import AStar

# 创建 A* 规划器实例
astar = AStar(width=20, height=20)

# 动态设置障碍物
astar.set_obstacle(5, 5, True)
astar.set_obstacle(6, 6, True)

# 查找路径
path = astar.find_path((0, 0), (19, 19))
```

## ⚡ 性能优化要点

1. **避免对象创建**：使用 `(x, y)` 元组代替 Node 对象
2. **高效的开放列表**：使用 `heapq` 实现优先队列
3. **快速查找**：使用字典存储 g_cost 和 f_cost
4. **内存优化**：只在需要时计算启发式值
5. **早期终止**：找到目标后立即返回

## 📊 性能对比

| 网格大小 | 原始版本 | 优化版本 | 性能提升 |
|----------|----------|----------|----------|
| 20x20 | 0.00077s | 0.00001s | 80.83x |
| 50x50 | 0.0226s | 0.00139s | 16.28x |
| **平均** | - | - | **48.56x** |

## 🧪 运行测试

```bash
python test_astar.py
python performance_test.py
```

## 📝 算法说明

A* 算法结合了 Dijkstra 算法的精确性和贪心最佳优先搜索的效率：
- **g(n)**: 从起点到节点 n 的实际代价
- **h(n)**: 从节点 n 到目标的启发式估计代价  
- **f(n) = g(n) + h(n)**: 总估计代价

本实现在网格地图中使用**曼哈顿距离**作为启发式函数，保证找到最优解。

## 📜 许可证

MIT License