/**
 * @file    encoder_stm32.h
 * @brief   编码器驱动模块 — STM32 HAL库版本（TIM硬件编码器模式）
 * @details 使用STM32定时器硬件编码器模式，四倍频计数。
 *          可计算实时速度(cm/s)和累计距离(cm)。
 * @author  nuedc-asset-library
 * @version 1.0
 * @date    2026-06
 */

#ifndef __ENCODER_STM32_H
#define __ENCODER_STM32_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ========================================================================== */
/*                              类型定义                                       */
/* ========================================================================== */

/** @brief 编码器配置结构体 */
typedef struct {
    TIM_HandleTypeDef *htim;             /**< 编码器定时器句柄 */

    /* 物理参数 */
    uint16_t ppr;                        /**< 编码器线数（每转脉冲数，单相） */
    float    wheel_diameter_cm;          /**< 轮子直径(cm) */
    float    gear_ratio;                 /**< 减速比，默认1.0 */

    /* 运行时计算常量 */
    float    cm_per_pulse;               /**< 每个脉冲对应距离(cm) */

    /* 状态变量 */
    int32_t  total_count;                /**< 累计编码器脉冲总数 */
    int32_t  last_count;                 /**< 上次读取的计数值 */
    float    total_distance_cm;          /**< 累计行驶距离(cm) */
    float    speed_cm_s;                 /**< 当前速度(cm/s) */
    uint32_t last_update_tick;           /**< 上次更新时刻(ms) */
    bool     initialized;
} Encoder_t;

/* ========================================================================== */
/*                              接口函数                                       */
/* ========================================================================== */

/**
 * @brief 初始化编码器
 * @param encoder           编码器结构体指针
 * @param htim              编码器定时器句柄（需已配置为Encoder Mode, TI1+TI2）
 * @param ppr               编码器线数（如13线编码器填13）
 * @param wheel_diameter_cm  轮子直径(cm)
 * @param gear_ratio        减速比
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef Encoder_Init(Encoder_t *encoder, TIM_HandleTypeDef *htim,
                               uint16_t ppr, float wheel_diameter_cm,
                               float gear_ratio);

/**
 * @brief 更新编码器数据（需周期性调用，建议10~20ms一次）
 * @param encoder  编码器结构体指针
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef Encoder_Update(Encoder_t *encoder);

/**
 * @brief 获取当前速度
 * @param encoder  编码器结构体指针
 * @return 速度(cm/s)
 */
float Encoder_GetSpeed(const Encoder_t *encoder);

/**
 * @brief 获取累计行驶距离
 * @param encoder  编码器结构体指针
 * @return 累计距离(cm)
 */
float Encoder_GetDistance(const Encoder_t *encoder);

/**
 * @brief 重置累计距离和计数
 * @param encoder  编码器结构体指针
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef Encoder_Reset(Encoder_t *encoder);

/**
 * @brief 获取原始编码器累计计数值
 * @param encoder  编码器结构体指针
 * @return 累计原始脉冲数
 */
int32_t Encoder_GetCount(const Encoder_t *encoder);

#endif /* __ENCODER_STM32_H */
