/**
 * @file    height.h
 * @brief   高度检测模块头文件
 * @version 1.0
 */

#ifndef __HEIGHT_H
#define __HEIGHT_H

#include "stm32f1xx_hal.h"

/* 高度范围定义 */
#define HEIGHT_MIN          0.0f    // 最小高度(cm)
#define HEIGHT_MAX          10.0f   // 最大高度(cm)
#define HEIGHT_VALID_MIN    0.5f    // 有效最小高度(cm)
#define HEIGHT_VALID_MAX    8.0f    // 有效最大高度(cm)

/* 标定点数量 */
#define CALIB_POINTS        7

/* 函数声明 */
void Height_Init(void);
float Height_Calculate(uint16_t *adc_values);
float Height_GetAverage(void);
float Height_GetTilt(void);

#endif /* __HEIGHT_H */
