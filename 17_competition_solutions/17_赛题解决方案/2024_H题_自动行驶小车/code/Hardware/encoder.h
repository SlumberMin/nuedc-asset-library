/**
 * @file    encoder.h
 * @brief   编码器模块头文件
 * @author  电赛团队
 * @date    2024
 * @note    左编码器: TIM2 (PA0/PA1), 右编码器: TIM3 (PA6/PA7)
 */

#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f1xx_hal.h"
#include "user_config.h"

/* ========================================================================== */
/*                              编码器数据结构                                  */
/* ========================================================================== */

/**
 * @brief  编码器数据结构体
 */
typedef struct {
    int32_t  count;             /* 当前编码器计数值 */
    int32_t  last_count;        /* 上次编码器计数值 */
    int32_t  delta;             /* 本次增量 */
    float    speed;             /* 瞬时速度 (cm/s) */
    float    total_distance;    /* 累积行驶距离 (cm) */
    uint8_t  direction;         /* 行驶方向: 1=前进, 0=后退 */
} Encoder_t;

/* ========================================================================== */
/*                              函数声明                                       */
/* ========================================================================== */

/**
 * @brief  编码器模块初始化
 * @note   配置TIM2和TIM3为编码器模式（四倍频）
 * @retval None
 */
void Encoder_Init(void);

/**
 * @brief  更新编码器数据（在定时中断中调用，建议10ms周期）
 * @retval None
 */
void Encoder_Update(void);

/**
 * @brief  获取左轮编码器数据
 * @retval Encoder_t* 指向左编码器数据结构体
 */
Encoder_t* Encoder_GetLeft(void);

/**
 * @brief  获取右轮编码器数据
 * @retval Encoder_t* 指向右编码器数据结构体
 */
Encoder_t* Encoder_GetRight(void);

/**
 * @brief  获取左右轮平均行驶距离(cm)
 * @retval float 平均距离
 */
float Encoder_GetAvgDistance(void);

/**
 * @brief  重置编码器累积距离
 * @retval None
 */
void Encoder_ResetDistance(void);

/**
 * @brief  获取瞬时速度(cm/s)
 * @retval float 左右轮平均速度
 */
float Encoder_GetSpeed(void);

#endif /* __ENCODER_H */
