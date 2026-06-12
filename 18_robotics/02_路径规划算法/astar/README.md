# A*路径规划算法使用文档

## 1. 算法原理

A*算法是一种启发式搜索算法，用于在加权图中找到从起点到终点的最短路径。

### 1.1 核心公式

```
f(n) = g(n) + h(n)
```

- **f(n)**: 节点n的总代价
- **g(n)**: 从起点到节点n的实际代价
- **h(n)**: 从节点n到终点的估计代价（启发式函数）

### 1.2 启发式函数选择

**曼哈顿距离**（适用于4方向移动）：
```
h(n) = |x1 - x2| + |y1 - y2|
```

**欧几里得距离**（适用于8方向移动）：
```
h(n) = sqrt((x1 - x2)² + (y1 - y2)²)
```

**切比雪夫距离**（适用于8方向移动，对角线代价为1）：
```
h(n) = max(|x1 - x2|, |y1 - y2|)
```

### 1.3 算法流程

```
1. 将起点加入开放列表
2. 循环：
   a. 从开放列表中选择f值最小的节点
   b. 如果该节点是终点，回溯路径并返回
   c. 将该节点移到关闭列表
   d. 遍历该节点的所有邻居：
      - 如果邻居在关闭列表中，跳过
      - 如果邻居不可通行，跳过
      - 计算新的g值
      - 如果邻居不在开放列表中，或者新的g值更小：
        * 更新邻居的g、h、f值
        * 设置邻居的父节点
        * 将邻居加入开放列表
3. 如果开放列表为空，返回失败
```

## 2. 数据结构

### 2.1 地图表示
```c
uint8_t map[MAP_WIDTH][MAP_HEIGHT];
// 0: 可通行
// 1: 障碍物
// 2: 起点
// 3: 终点
```

### 2.2 节点结构
```c
typedef struct {
    int16_t x, y;           // 坐标
    float g, h, f;          // 代价
    int16_t parent_x, parent_y;  // 父节点
    bool closed, opened;    // 状态
} AStarNode;
```

### 2.3 路径结构
```c
typedef struct {
    int16_t x, y;
} PathPoint;

typedef struct {
    PathPoint points[MAX_PATH_LENGTH];
    uint16_t length;
} Path;
```

## 3. 使用示例

### 3.1 基本使用
```c
#include "astar.h"

int main()
{
    AStar_HandleTypeDef hastar;
    uint8_t map[MAP_WIDTH][MAP_HEIGHT] = {0};
    
    // 初始化
    AStar_Init(&hastar);
    
    // 设置地图
    // 添加障碍物
    map[5][5] = NODE_OBSTACLE;
    map[5][6] = NODE_OBSTACLE;
    map[5][7] = NODE_OBSTACLE;
    
    AStar_SetMap(&hastar, map);
    
    // 设置起点和终点
    AStar_SetStart(&hastar, 0, 0);
    AStar_SetEnd(&hastar, 10, 10);
    
    // 查找路径
    if (AStar_FindPath(&hastar)) {
        Path *path = AStar_GetPath(&hastar);
        
        // 执行路径
        for (uint16_t i = 0; i < path->length; i++) {
            // 移动到 path->points[i].x, path->points[i].y
        }
    }
    
    return 0;
}
```

### 3.2 动态障碍物处理
```c
void UpdateMap(AStar_HandleTypeDef *hastar, int16_t obs_x, int16_t obs_y)
{
    // 添加新障碍物
    hastar->map[obs_x][obs_y] = NODE_OBSTACLE;
    
    // 重新规划路径
    AStar_FindPath(hastar);
}
```

## 4. 性能优化

### 4.1 使用优先队列
当前实现使用线性搜索，时间复杂度为O(n)。可以使用最小堆优化到O(log n)。

### 4.2 路径平滑
原始A*路径可能不平滑，可以使用以下方法平滑：
- **样条曲线插值**
- **贝塞尔曲线**
- **梯度下降优化**

### 4.3 内存优化
- 使用位图存储地图
- 动态分配节点内存
- 限制搜索区域大小

## 5. 常见问题

### 5.1 找不到路径
- 检查起点和终点是否可通行
- 检查地图是否有连通路径
- 增加地图分辨率

### 5.2 路径不是最优
- 检查启发式函数是否满足可采纳性
- 确保移动代价设置正确
- 检查对角线移动代价

### 5.3 内存不足
- 减小地图尺寸
- 使用更高效的数据结构
- 限制最大路径长度

## 6. 扩展功能

### 6.1 路径平滑算法
```c
void SmoothPath(Path *path, float smooth_factor)
{
    // 使用梯度下降平滑路径
    for (int iter = 0; iter < 100; iter++) {
        for (uint16_t i = 1; i < path->length - 1; i++) {
            float new_x = path->points[i].x + 
                         smooth_factor * (path->points[i-1].x + path->points[i+1].x - 2*path->points[i].x);
            float new_y = path->points[i].y + 
                         smooth_factor * (path->points[i-1].y + path->points[i+1].y - 2*path->points[i].y);
            path->points[i].x = (int16_t)new_x;
            path->points[i].y = (int16_t)new_y;
        }
    }
}
```

### 6.2 路径跟踪控制器
```c
void FollowPath(Path *path, Robot_HandleTypeDef *robot)
{
    for (uint16_t i = 0; i < path->length; i++) {
        float target_x = path->points[i].x;
        float target_y = path->points[i].y;
        
        // 使用PID控制移动到目标点
        while (DistanceTo(robot, target_x, target_y) > 0.1f) {
            float error_x = target_x - robot->x;
            float error_y = target_y - robot->y;
            
            float speed = PID_Compute(&robot->pid, 0, DistanceTo(robot, target_x, target_y));
            float angle = atan2f(error_y, error_x);
            
            robot->vx = speed * cosf(angle);
            robot->vy = speed * sinf(angle);
        }
    }
}
```
