/**
 * @file    sensor.h
 * @brief   红外循迹传感器模块头文件
 * @author  电赛团队
 * @date    2024
 * @note    7路TCRT5000红外对管，数字输出
 */

#ifndef __SENSOR_H
#define __SENSOR_H

#include "stm32f1xx_hal.h"
#include "user_config.h"

/* ========================================================================== */
/*                              数据结构                                       */
/* ========================================================================== */

/**
 * @brief  传感器原始数据结构
 */
typedef struct {
    uint8_t s[7];           /* 各路传感器状态: 1=检测到黑线, 0=白底 */
    uint8_t count;          /* 检测到黑线的传感器数量 */
    int8_t  error;          /* 循迹偏差值 (-100 ~ +100) */
    uint8_t all_white;      /* 1=全部在白底上（脱轨标志） */
    uint8_t all_black;      /* 1=全部在黑线上 */
} SensorData_t;

/* ========================================================================== */
/*                              函数声明                                       */
/* ========================================================================== */

/**
 * @brief  传感器模块初始化
 * @note   配置传感器引脚为输入模式
 * @retval None
 */
void Sensor_Init(void);

/**
 * @brief  读取所有传感器状态
 * @retval SensorData_t* 指向传感器数据结构体
 */
SensorData_t* Sensor_Read(void);

/**
 * @brief  获取循迹偏差值
 * @retval int8_t 偏差值 (-100 ~ +100)
 *         负值=偏左，正值=偏右，0=居中
 */
int8_t Sensor_GetError(void);

/**
 * @brief  判断是否脱轨
 * @retval uint8_t 1=脱轨, 0=正常
 */
uint8_t Sensor_IsOffTrack(void);

/**
 * @brief  获取原始传感器位图
 * @retval uint8_t 位图（bit0=S1, bit6=S7）
 */
uint8_t Sensor_GetBitmap(void);

#endif /* __SENSOR_H */
