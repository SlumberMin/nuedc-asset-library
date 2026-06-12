/**
 * @file    tracking.h
 * @brief   红外循迹传感器模块头文件
 */
#ifndef __TRACKING_H
#define __TRACKING_H

#include <stdint.h>

/* 传感器数量 */
#define TRACK_SENSOR_NUM    8

/* 函数声明 */
void Tracking_Init(void);
int16_t Tracking_GetPosition(void);
uint8_t Tracking_GetRawData(void);
uint8_t Tracking_CheckCrossLine(void);
uint8_t Tracking_IsOnLine(void);

#endif /* __TRACKING_H */
