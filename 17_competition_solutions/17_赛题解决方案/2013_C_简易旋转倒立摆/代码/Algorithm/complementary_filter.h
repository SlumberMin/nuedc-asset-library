/**
 * @file    complementary_filter.h
 * @brief   互补滤波器
 * 
 * 互补滤波器优势：
 * 1. 计算量极小（比卡尔曼小10倍）
 * 2. 无需矩阵运算
 * 3. 实时性好
 * 
 * 原理：
 * angle = α * (angle + gyro * dt) + (1-α) * accel_angle
 * 
 * α通常取0.96~0.98
 * α越大越信任陀螺仪（响应快但有漂移）
 * α越小越信任加速度计（响应慢但无漂移）
 */

#ifndef __COMPLEMENTARY_FILTER_H
#define __COMPLEMENTARY_FILTER_H

#include <stdint.h>

typedef struct {
    float alpha;        // 滤波系数(0~1)
    float angle;        // 滤波后的角度
    float dt;           // 采样周期(s)
} ComplementaryFilter_t;

void ComplementaryFilter_Init(ComplementaryFilter_t *cf, float alpha, float dt);
float ComplementaryFilter_Update(ComplementaryFilter_t *cf, float gyro, float accel_angle);
void ComplementaryFilter_Reset(ComplementaryFilter_t *cf);

#endif
