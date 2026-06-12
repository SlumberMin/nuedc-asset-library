/*
 * 车道线检测算法实现
 * 来源：飞思卡尔智能车竞赛
 * 适配平台：MSPM0G3507
 */

#include "lane_detection.h"
#include <math.h>
#include <string.h>

// Sobel算子
static const int8_t sobel_x[3][3] = {
    {-1, 0, 1},
    {-2, 0, 2},
    {-1, 0, 1}
};

static const int8_t sobel_y[3][3] = {
    {-1, -2, -1},
    { 0,  0,  0},
    { 1,  2,  1}
};

/**
 * @brief 初始化车道线检测器
 * @param hdetector 检测器句柄
 */
void LaneDetector_Init(LaneDetector_HandleTypeDef *hdetector)
{
    memset(hdetector, 0, sizeof(LaneDetector_HandleTypeDef));
    
    // 设置默认参数
    hdetector->threshold = 128;     // 二值化阈值
    hdetector->min_slope = -2.0f;   // 最小斜率
    hdetector->max_slope = 2.0f;    // 最大斜率
}

/**
 * @brief 设置图像数据
 * @param hdetector 检测器句柄
 * @param image 图像数据（灰度图）
 */
void LaneDetector_SetImage(LaneDetector_HandleTypeDef *hdetector, uint8_t *image)
{
    memcpy(hdetector->image, image, IMAGE_WIDTH * IMAGE_HEIGHT);
}

/**
 * @brief 图像二值化
 * @param hdetector 检测器句柄
 */
void LaneDetector_Binarize(LaneDetector_HandleTypeDef *hdetector)
{
    // 只处理感兴趣区域（ROI）
    for (uint16_t y = ROI_TOP; y < ROI_BOTTOM; y++) {
        for (uint16_t x = 0; x < IMAGE_WIDTH; x++) {
            hdetector->binary[y][x] = (hdetector->image[y][x] > hdetector->threshold) ? 255 : 0;
        }
    }
}

/**
 * @brief 边缘检测（Sobel算子）
 * @param hdetector 检测器句柄
 */
void LaneDetector_EdgeDetect(LaneDetector_HandleTypeDef *hdetector)
{
    // 只处理感兴趣区域（ROI）
    for (uint16_t y = ROI_TOP + 1; y < ROI_BOTTOM - 1; y++) {
        for (uint16_t x = 1; x < IMAGE_WIDTH - 1; x++) {
            int16_t gx = 0, gy = 0;
            
            // 计算Sobel梯度
            for (int8_t ky = -1; ky <= 1; ky++) {
                for (int8_t kx = -1; kx <= 1; kx++) {
                    gx += hdetector->binary[y+ky][x+kx] * sobel_x[ky+1][kx+1];
                    gy += hdetector->binary[y+ky][x+kx] * sobel_y[ky+1][kx+1];
                }
            }
            
            // 计算梯度幅值
            uint16_t magnitude = abs(gx) + abs(gy);
            if (magnitude > 255) magnitude = 255;
            
            hdetector->edge[y][x] = (uint8_t)magnitude;
        }
    }
}

/**
 * @brief 查找车道线点
 * @param hdetector 检测器句柄
 * @param points_x 左车道线x坐标数组
 * @param points_y 左车道线y坐标数组
 * @param left_count 左车道线点数
 * @param right_points_x 右车道线x坐标数组
 * @param right_points_y 右车道线y坐标数组
 * @param right_count 右车道线点数
 */
static void FindLanePoints(LaneDetector_HandleTypeDef *hdetector,
                           uint16_t *left_points_x, uint16_t *left_points_y, uint16_t *left_count,
                           uint16_t *right_points_x, uint16_t *right_points_y, uint16_t *right_count)
{
    *left_count = 0;
    *right_count = 0;
    
    // 从下往上扫描
    for (uint16_t y = ROI_BOTTOM - 1; y >= ROI_TOP; y--) {
        // 查找左车道线（从中间向左扫描）
        for (uint16_t x = IMAGE_WIDTH / 2; x > 0; x--) {
            if (hdetector->edge[y][x] > 100) {
                left_points_x[*left_count] = x;
                left_points_y[*left_count] = y;
                (*left_count)++;
                break;
            }
        }
        
        // 查找右车道线（从中间向右扫描）
        for (uint16_t x = IMAGE_WIDTH / 2; x < IMAGE_WIDTH; x++) {
            if (hdetector->edge[y][x] > 100) {
                right_points_x[*right_count] = x;
                right_points_y[*right_count] = y;
                (*right_count)++;
                break;
            }
        }
    }
}

/**
 * @brief 最小二乘法拟合直线
 * @param points_x x坐标数组
 * @param points_y y坐标数组
 * @param num_points 点数
 * @param slope 斜率输出
 * @param intercept 截距输出
 */
static void LinearRegression(uint16_t *points_x, uint16_t *points_y, uint16_t num_points,
                             float *slope, float *intercept)
{
    if (num_points < 2) {
        *slope = 0.0f;
        *intercept = 0.0f;
        return;
    }
    
    float sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
    
    for (uint16_t i = 0; i < num_points; i++) {
        sum_x += points_x[i];
        sum_y += points_y[i];
        sum_xy += points_x[i] * points_y[i];
        sum_x2 += points_x[i] * points_x[i];
    }
    
    float n = (float)num_points;
    float denominator = n * sum_x2 - sum_x * sum_x;
    
    if (fabsf(denominator) < 1e-6f) {
        *slope = 0.0f;
        *intercept = sum_y / n;
        return;
    }
    
    *slope = (n * sum_xy - sum_x * sum_y) / denominator;
    *intercept = (sum_y - *slope * sum_x) / n;
}

/**
 * @brief 查找并拟合车道线
 * @param hdetector 检测器句柄
 */
void LaneDetector_FindLaneLines(LaneDetector_HandleTypeDef *hdetector)
{
    uint16_t left_points_x[IMAGE_HEIGHT], left_points_y[IMAGE_HEIGHT];
    uint16_t right_points_x[IMAGE_HEIGHT], right_points_y[IMAGE_HEIGHT];
    uint16_t left_count, right_count;
    
    // 查找车道线点
    FindLanePoints(hdetector, 
                   left_points_x, left_points_y, &left_count,
                   right_points_x, right_points_y, &right_count);
    
    // 拟合左车道线
    if (left_count >= 10) {
        float slope, intercept;
        LinearRegression(left_points_x, left_points_y, left_count, &slope, &intercept);
        
        hdetector->result.left_lane.slope = slope;
        hdetector->result.left_lane.intercept = intercept;
        hdetector->result.left_lane.points = left_count;
        hdetector->result.left_lane.valid = (slope >= hdetector->min_slope && slope <= hdetector->max_slope);
    } else {
        hdetector->result.left_lane.valid = false;
    }
    
    // 拟合右车道线
    if (right_count >= 10) {
        float slope, intercept;
        LinearRegression(right_points_x, right_points_y, right_count, &slope, &intercept);
        
        hdetector->result.right_lane.slope = slope;
        hdetector->result.right_lane.intercept = intercept;
        hdetector->result.right_lane.points = right_count;
        hdetector->result.right_lane.valid = (slope >= hdetector->min_slope && slope <= hdetector->max_slope);
    } else {
        hdetector->result.right_lane.valid = false;
    }
    
    // 计算偏移量和偏转角
    if (hdetector->result.left_lane.valid && hdetector->result.right_lane.valid) {
        // 计算车道中心
        float left_x = (ROI_BOTTOM - hdetector->result.left_lane.intercept) / hdetector->result.left_lane.slope;
        float right_x = (ROI_BOTTOM - hdetector->result.right_lane.intercept) / hdetector->result.right_lane.slope;
        float lane_center = (left_x + right_x) / 2.0f;
        
        // 计算车辆偏移量（相对于图像中心）
        hdetector->result.offset = lane_center - IMAGE_WIDTH / 2.0f;
        
        // 计算偏转角
        float left_angle = atanf(hdetector->result.left_lane.slope) * 180.0f / 3.14159f;
        float right_angle = atanf(hdetector->result.right_lane.slope) * 180.0f / 3.14159f;
        hdetector->result.angle = (left_angle + right_angle) / 2.0f;
        
        hdetector->result.valid = true;
    } else {
        hdetector->result.offset = 0.0f;
        hdetector->result.angle = 0.0f;
        hdetector->result.valid = false;
    }
}

/**
 * @brief 拟合车道线（单条）
 * @param hdetector 检测器句柄
 * @param line 车道线输出
 * @param points_x x坐标数组
 * @param points_y y坐标数组
 * @param num_points 点数
 */
void LaneDetector_FitLaneLine(LaneDetector_HandleTypeDef *hdetector, LaneLine *line, 
                               uint16_t *points_x, uint16_t *points_y, uint16_t num_points)
{
    LinearRegression(points_x, points_y, num_points, &line->slope, &line->intercept);
    line->points = num_points;
    line->valid = (num_points >= 10 && line->slope >= hdetector->min_slope && line->slope <= hdetector->max_slope);
}

/**
 * @brief 获取检测结果
 * @param hdetector 检测器句柄
 * @return 检测结果指针
 */
LaneDetectionResult* LaneDetector_GetResult(LaneDetector_HandleTypeDef *hdetector)
{
    return &hdetector->result;
}
