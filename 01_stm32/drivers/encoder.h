/**
 * @file    encoder.h
 * @brief   编码器驱动模块 — STM32电赛通用代码库
 * @details 使用STM32定时器硬件编码器模式，支持四倍频计数。
 *          可计算实时速度(cm/s)和累计距离(cm)。
 *          需要在CubeMX中将定时器配置为Encoder Mode。
 * @author  电赛通用代码库
 * @version 1.0
 * @date    2026-06
 */

#ifndef __ENCODER_H
#define __ENCODER_H

#include "platform/hal_stm32.h"

/* ========================================================================== */
/*                              类型定义                                       */
/* ========================================================================== */

/**
 * @brief 编码器配置结构体
 */
typedef struct {
    /* 硬件配置 */
    TIM_HandleTypeDef *htim;        /**< 编码器定时器句柄（需配置为Encoder Mode） */

    /* 物理参数 */
    uint16_t ppr;                   /**< 编码器线数（每转脉冲数，单相） */
    float    wheel_diameter_cm;     /**< 轮子直径(cm) */
    float    gear_ratio;            /**< 齿轮减速比（电机转数:轮子转数），默认1.0 */

    /* 计算用常量（Init时自动计算） */
    float    cm_per_pulse;          /**< 每个脉冲对应的距离(cm) */

    /* 状态变量 */
    int32_t  total_count;           /**< 累计编码器脉冲总数 */
    int32_t  last_count;            /**< 上次读取的计数值 */
    float    total_distance_cm;     /**< 累计行驶距离(cm) */
    float    speed_cm_s;            /**< 当前速度(cm/s)，正值前进 */
    uint32_t last_update_tick;      /**< 上次更新时刻(ms) */
    bool     initialized;           /**< 是否已初始化 */
} Encoder_t;

/* ========================================================================== */
/*                              接口函数                                       */
/* ========================================================================== */

/**
 * @brief 初始化编码器
 * @param encoder          编码器结构体指针
 * @param htim             编码器定时器句柄（需已配置为Encoder Mode, TI1+TI2）
 * @param ppr              编码器线数（如13线编码器填13）
 * @param wheel_diameter_cm 轮子直径(cm)，如6.5cm
 * @param gear_ratio       减速比，如30:1填30.0f，无减速填1.0f
 * @return ErrorCode_t: HAL_OK_CODE=成功
 * @note   四倍频已在硬件中完成，ppr为编码器原始线数
 *         实际每转脉冲数 = ppr * 4 * gear_ratio
 */
ErrorCode_t Encoder_Init(Encoder_t *encoder, TIM_HandleTypeDef *htim,
                         uint16_t ppr, float wheel_diameter_cm, float gear_ratio);

/**
 * @brief 更新编码器数据（需周期性调用，建议10~20ms一次）
 * @param encoder  编码器结构体指针
 * @return ErrorCode_t: HAL_OK_CODE=成功
 * @details 读取定时器计数值，计算速度和累计距离。
 *          速度计算：delta_distance / delta_time
 *          处理定时器计数溢出（16位定时器 ±32767）
 */
ErrorCode_t Encoder_Update(Encoder_t *encoder);

/**
 * @brief 获取当前速度
 * @param encoder  编码器结构体指针
 * @return float: 速度(cm/s)，正值前进，负值后退
 */
float Encoder_GetSpeed(const Encoder_t *encoder);

/**
 * @brief 获取累计行驶距离
 * @param encoder  编码器结构体指针
 * @return float: 累计距离(cm)，始终为正
 */
float Encoder_GetDistance(const Encoder_t *encoder);

/**
 * @brief 重置累计距离和计数
 * @param encoder  编码器结构体指针
 * @return ErrorCode_t
 * @note   不影响速度计算
 */
ErrorCode_t Encoder_Reset(Encoder_t *encoder);

/**
 * @brief 获取原始编码器计数值
 * @param encoder  编码器结构体指针
 * @return int32_t: 累计原始脉冲数
 */
int32_t Encoder_GetCount(const Encoder_t *encoder);

#endif /* __ENCODER_H */
