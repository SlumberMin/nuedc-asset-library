/*
 * 车道线检测算法
 * 来源：飞思卡尔智能车竞赛
 * 适配平台：MSPM0G3507
 * 
 * 设计思路：
 * 1. 图像二值化
 * 2. 边缘检测（Sobel算子）
 * 3. 霍夫变换检测直线
 * 4. 车道线拟合
 * 5. 计算车辆偏移量
 */

#ifndef LANE_DETECTION_H
#define LANE_DETECTION_H

#include <stdint.h>
#include <stdbool.h>

// 图像参数
#define IMAGE_WIDTH     320     // 图像宽度
#define IMAGE_HEIGHT    240     // 图像高度
#define ROI_TOP         120     // 感兴趣区域顶部
#define ROI_BOTTOM      240     // 感兴趣区域底部

// 车道线参数
#define LANE_WIDTH      200     // 车道宽度（像素）
#define MAX_LANES       2       // 最大车道线数量

// 车道线结构体
typedef struct {
    float slope;        // 斜率
    float intercept;    // 截距
    bool valid;         // 是否有效
    uint16_t points;    // 拟合点数
} LaneLine;

// 车道线检测结果
typedef struct {
    LaneLine left_lane;     // 左车道线
    LaneLine right_lane;    // 右车道线
    float offset;           // 车辆偏移量（像素）
    float angle;            // 车辆偏转角（度）
    bool valid;             // 检测结果是否有效
} LaneDetectionResult;

// 车道线检测器
typedef struct {
    uint8_t image[IMAGE_HEIGHT][IMAGE_WIDTH];       // 原始图像
    uint8_t binary[IMAGE_HEIGHT][IMAGE_WIDTH];      // 二值化图像
    uint8_t edge[IMAGE_HEIGHT][IMAGE_WIDTH];        // 边缘图像
    
    uint8_t threshold;      // 二值化阈值
    float min_slope;        // 最小斜率
    float max_slope;        // 最大斜率
    
    LaneDetectionResult result;
} LaneDetector_HandleTypeDef;

// 函数声明
void LaneDetector_Init(LaneDetector_HandleTypeDef *hdetector);
void LaneDetector_SetImage(LaneDetector_HandleTypeDef *hdetector, uint8_t *image);
void LaneDetector_Binarize(LaneDetector_HandleTypeDef *hdetector);
void LaneDetector_EdgeDetect(LaneDetector_HandleTypeDef *hdetector);
void LaneDetector_FindLaneLines(LaneDetector_HandleTypeDef *hdetector);
void LaneDetector_FitLaneLine(LaneDetector_HandleTypeDef *hdetector, LaneLine *line, uint16_t *points_x, uint16_t *points_y, uint16_t num_points);
LaneDetectionResult* LaneDetector_GetResult(LaneDetector_HandleTypeDef *hdetector);

#endif // LANE_DETECTION_H
