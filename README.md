# A* 路径规划算法

一个高效的 A* (A-star) 路径规划算法 Python 实现，支持网格地图和障碍物避障。

## 特性

- 完整的 A* 算法实现
- 支持 4 方向和 8 方向移动
- 可配置的启发式函数（曼哈顿距离、欧几里得距离、对角线距离）
- 障碍物避障
- 完整的单元测试
- 性能优化（使用优先队列）

## 安装

```bash
pip install -r requirements.txt
```

## 使用示例

```python
from astar import AStar

# 创建地图（0表示可通过，1表示障碍物）
grid = [
    [0, 0, 0, 1, 0],
    [0, 1, 0, 1, 0],
    [0, 1, 0, 0, 0],
    [0, 0, 0, 1, 0],
    [0, 0, 0, 0, 0]
]

# 创建A*实例
astar = AStar(grid)

# 查找路径
path = astar.find_path((0, 0), (4, 4))
print(f"找到路径: {path}")
```

## 测试

运行所有测试：
```bash
python -m pytest tests/ -v
```

运行性能测试：
```bash
python tests/test_performance.py
```