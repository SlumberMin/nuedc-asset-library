/**
 * @file    vision.h
 * @brief   OpenMV视觉通信模块头文件
 */
#ifndef __VISION_H
#define __VISION_H

#include <stdint.h>

/* 视觉数据结构体 */
typedef struct {
    float dx;           // 水平偏差(像素)
    float dy;           // 垂直偏差(像素)
    uint8_t valid;      // 数据有效标志
    uint16_t area;      // 目标面积(用于判断距离)
} VisionData_t;

void Vision_Init(void);
uint8_t Vision_GetData(VisionData_t *data);
void Vision_RxCallback(uint8_t ch);

#endif /* __VISION_H */
