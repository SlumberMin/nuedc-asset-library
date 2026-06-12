/*
 * A*路径规划算法实现
 * 来源：RoboMaster 导航系统
 * 适配平台：MSPM0G3507
 */

#include "astar.h"
#include <math.h>
#include <string.h>

// 8方向移动偏移量
static const int8_t dx[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
static const int8_t dy[8] = {-1, -1, -1, 0, 0, 1, 1, 1};

// 8方向移动代价
static const float cost[8] = {1.414f, 1.0f, 1.414f, 1.0f, 1.0f, 1.414f, 1.0f, 1.414f};

/**
 * @brief 初始化A*算法
 * @param hastar A*句柄
 */
void AStar_Init(AStar_HandleTypeDef *hastar)
{
    memset(hastar, 0, sizeof(AStar_HandleTypeDef));
}

/**
 * @brief 设置地图
 * @param hastar A*句柄
 * @param map 地图数据
 */
void AStar_SetMap(AStar_HandleTypeDef *hastar, uint8_t map[MAP_WIDTH][MAP_HEIGHT])
{
    memcpy(hastar->map, map, sizeof(uint8_t) * MAP_WIDTH * MAP_HEIGHT);
}

/**
 * @brief 设置起点
 * @param hastar A*句柄
 * @param x 起点x坐标
 * @param y 起点y坐标
 */
void AStar_SetStart(AStar_HandleTypeDef *hastar, int16_t x, int16_t y)
{
    hastar->start_x = x;
    hastar->start_y = y;
    hastar->map[x][y] = NODE_START;
}

/**
 * @brief 设置终点
 * @param hastar A*句柄
 * @param x 终点x坐标
 * @param y 终点y坐标
 */
void AStar_SetEnd(AStar_HandleTypeDef *hastar, int16_t x, int16_t y)
{
    hastar->end_x = x;
    hastar->end_y = y;
    hastar->map[x][y] = NODE_END;
}

/**
 * @brief 启发式函数（曼哈顿距离）
 * @param x1 起点x坐标
 * @param y1 起点y坐标
 * @param x2 终点x坐标
 * @param y2 终点y坐标
 * @return 估计代价
 */
float AStar_Heuristic(int16_t x1, int16_t y1, int16_t x2, int16_t y2)
{
    // 使用欧几里得距离
    float dx = abs(x2 - x1);
    float dy = abs(y2 - y1);
    return sqrtf(dx*dx + dy*dy);
}

/**
 * @brief 检查坐标是否有效
 * @param x x坐标
 * @param y y坐标
 * @return 是否有效
 */
static bool IsValid(int16_t x, int16_t y)
{
    return (x >= 0 && x < MAP_WIDTH && y >= 0 && y < MAP_HEIGHT);
}

/**
 * @brief 检查节点是否可通行
 * @param hastar A*句柄
 * @param x x坐标
 * @param y y坐标
 * @return 是否可通行
 */
static bool IsWalkable(AStar_HandleTypeDef *hastar, int16_t x, int16_t y)
{
    return (hastar->map[x][y] != NODE_OBSTACLE);
}

/**
 * @brief 查找f值最小的节点（简化版本，使用线性搜索）
 * @param hastar A*句柄
 * @return 最小f值节点的坐标
 */
static PathPoint FindMinFNode(AStar_HandleTypeDef *hastar)
{
    PathPoint min_point = {-1, -1};
    float min_f = 1e9f;
    
    for (int16_t x = 0; x < MAP_WIDTH; x++) {
        for (int16_t y = 0; y < MAP_HEIGHT; y++) {
            if (hastar->nodes[x][y].opened && !hastar->nodes[x][y].closed) {
                if (hastar->nodes[x][y].f < min_f) {
                    min_f = hastar->nodes[x][y].f;
                    min_point.x = x;
                    min_point.y = y;
                }
            }
        }
    }
    
    return min_point;
}

/**
 * @brief 查找路径
 * @param hastar A*句柄
 * @return 是否找到路径
 */
bool AStar_FindPath(AStar_HandleTypeDef *hastar)
{
    // 初始化起点
    hastar->nodes[hastar->start_x][hastar->start_y].x = hastar->start_x;
    hastar->nodes[hastar->start_x][hastar->start_y].y = hastar->start_y;
    hastar->nodes[hastar->start_x][hastar->start_y].g = 0.0f;
    hastar->nodes[hastar->start_x][hastar->start_y].h = AStar_Heuristic(hastar->start_x, hastar->start_y, 
                                                                          hastar->end_x, hastar->end_y);
    hastar->nodes[hastar->start_x][hastar->start_y].f = hastar->nodes[hastar->start_x][hastar->start_y].g + 
                                                          hastar->nodes[hastar->start_x][hastar->start_y].h;
    hastar->nodes[hastar->start_x][hastar->start_y].opened = true;
    
    while (1) {
        // 查找f值最小的节点
        PathPoint current = FindMinFNode(hastar);
        
        // 没有找到可达节点
        if (current.x == -1 || current.y == -1) {
            return false;
        }
        
        // 到达终点
        if (current.x == hastar->end_x && current.y == hastar->end_y) {
            // 回溯路径
            hastar->path.length = 0;
            int16_t x = current.x;
            int16_t y = current.y;
            
            while (x != hastar->start_x || y != hastar->start_y) {
                hastar->path.points[hastar->path.length].x = x;
                hastar->path.points[hastar->path.length].y = y;
                hastar->path.length++;
                
                // 获取父节点
                int16_t parent_x = hastar->nodes[x][y].parent_x;
                int16_t parent_y = hastar->nodes[x][y].parent_y;
                x = parent_x;
                y = parent_y;
            }
            
            // 添加起点
            hastar->path.points[hastar->path.length].x = hastar->start_x;
            hastar->path.points[hastar->path.length].y = hastar->start_y;
            hastar->path.length++;
            
            // 反转路径（从起点到终点）
            for (uint16_t i = 0; i < hastar->path.length / 2; i++) {
                PathPoint temp = hastar->path.points[i];
                hastar->path.points[i] = hastar->path.points[hastar->path.length - 1 - i];
                hastar->path.points[hastar->path.length - 1 - i] = temp;
            }
            
            return true;
        }
        
        // 标记当前节点为已关闭
        hastar->nodes[current.x][current.y].closed = true;
        
        // 遍历8个方向
        for (uint8_t i = 0; i < 8; i++) {
            int16_t nx = current.x + dx[i];
            int16_t ny = current.y + dy[i];
            
            // 检查坐标有效性
            if (!IsValid(nx, ny)) {
                continue;
            }
            
            // 检查是否可通行
            if (!IsWalkable(hastar, nx, ny)) {
                continue;
            }
            
            // 检查是否在关闭列表中
            if (hastar->nodes[nx][ny].closed) {
                continue;
            }
            
            // 计算新的g值
            float new_g = hastar->nodes[current.x][current.y].g + cost[i];
            
            // 如果节点不在开放列表中，或者新的g值更小
            if (!hastar->nodes[nx][ny].opened || new_g < hastar->nodes[nx][ny].g) {
                hastar->nodes[nx][ny].x = nx;
                hastar->nodes[nx][ny].y = ny;
                hastar->nodes[nx][ny].g = new_g;
                hastar->nodes[nx][ny].h = AStar_Heuristic(nx, ny, hastar->end_x, hastar->end_y);
                hastar->nodes[nx][ny].f = hastar->nodes[nx][ny].g + hastar->nodes[nx][ny].h;
                hastar->nodes[nx][ny].parent_x = current.x;
                hastar->nodes[nx][ny].parent_y = current.y;
                hastar->nodes[nx][ny].opened = true;
            }
        }
    }
}

/**
 * @brief 获取规划路径
 * @param hastar A*句柄
 * @return 路径指针
 */
Path* AStar_GetPath(AStar_HandleTypeDef *hastar)
{
    return &hastar->path;
}
