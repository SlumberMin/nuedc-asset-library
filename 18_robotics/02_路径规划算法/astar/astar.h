/*
 * A*路径规划算法
 * 来源：RoboMaster 导航系统
 * 适配平台：MSPM0G3507
 * 
 * 设计思路：
 * 1. 使用优先队列（最小堆）管理开放列表
 * 2. 启发式函数：曼哈顿距离或欧几里得距离
 * 3. 支持8方向移动
 * 4. 返回最优路径
 */

#ifndef ASTAR_H
#define ASTAR_H

#include <stdint.h>
#include <stdbool.h>

// 地图参数
#define MAP_WIDTH       100     // 地图宽度（格子数）
#define MAP_HEIGHT      100     // 地图高度（格子数）
#define MAX_PATH_LENGTH 200     // 最大路径长度

// 节点状态
typedef enum {
    NODE_FREE = 0,      // 可通行
    NODE_OBSTACLE = 1,  // 障碍物
    NODE_START = 2,     // 起点
    NODE_END = 3        // 终点
} NodeState;

// 节点结构体
typedef struct {
    int16_t x;          // x坐标
    int16_t y;          // y坐标
    float g;            // 从起点到当前节点的实际代价
    float h;            // 从当前节点到终点的估计代价
    float f;            // 总代价 (f = g + h)
    int16_t parent_x;   // 父节点x坐标
    int16_t parent_y;   // 父节点y坐标
    bool closed;        // 是否在关闭列表中
    bool opened;        // 是否在开放列表中
} AStarNode;

// 路径点结构体
typedef struct {
    int16_t x;
    int16_t y;
} PathPoint;

// 路径结构体
typedef struct {
    PathPoint points[MAX_PATH_LENGTH];
    uint16_t length;
} Path;

// A*算法结构体
typedef struct {
    AStarNode nodes[MAP_WIDTH][MAP_HEIGHT];
    uint8_t map[MAP_WIDTH][MAP_HEIGHT];
    int16_t start_x, start_y;
    int16_t end_x, end_y;
    Path path;
} AStar_HandleTypeDef;

// 函数声明
void AStar_Init(AStar_HandleTypeDef *hastar);
void AStar_SetMap(AStar_HandleTypeDef *hastar, uint8_t map[MAP_WIDTH][MAP_HEIGHT]);
void AStar_SetStart(AStar_HandleTypeDef *hastar, int16_t x, int16_t y);
void AStar_SetEnd(AStar_HandleTypeDef *hastar, int16_t x, int16_t y);
bool AStar_FindPath(AStar_HandleTypeDef *hastar);
Path* AStar_GetPath(AStar_HandleTypeDef *hastar);
float AStar_Heuristic(int16_t x1, int16_t y1, int16_t x2, int16_t y2);

#endif // ASTAR_H
