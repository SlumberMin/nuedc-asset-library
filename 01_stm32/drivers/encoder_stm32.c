/**
 * @file    encoder_stm32.c
 * @brief   编码器驱动模块实现 — STM32 HAL库版本（TIM硬件编码器模式）
 */

#include "drivers/encoder_stm32.h"

#define ENCODER_PI  3.14159265f
#define SPEED_SMOOTH_ALPHA  0.3f   /* 速度低通滤波系数 (0~1, 越小越平滑) */

/* ========================================================================== */
/*                              接口函数实现                                   */
/* ========================================================================== */

/**
 * @brief 初始化编码器
 * @param encoder           编码器结构体指针
 * @param htim              HAL定时器句柄（已配置为编码器模式）
 * @param ppr               每转脉冲数（未四倍频前）
 * @param wheel_diameter_cm 轮子直径(cm)
 * @param gear_ratio        减速比（轮子转速/电机转速）
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef Encoder_Init(Encoder_t *encoder, TIM_HandleTypeDef *htim,
                               uint16_t ppr, float wheel_diameter_cm,
                               float gear_ratio)
{
    if (encoder == NULL || htim == NULL) return HAL_ERROR;
    if (ppr == 0 || wheel_diameter_cm <= 0 || gear_ratio <= 0) return HAL_ERROR;

    /* 保存配置 */
    encoder->htim              = htim;
    encoder->ppr               = ppr;
    encoder->wheel_diameter_cm = wheel_diameter_cm;
    encoder->gear_ratio        = gear_ratio;

    /* 四倍频后每转脉冲数 = ppr * 4
     * 轮子每转距离 = π * d
     * cm_per_pulse = π * d / (ppr * 4 * gear_ratio)
     */
    float circumference  = ENCODER_PI * wheel_diameter_cm;
    float pulses_per_rev = (float)ppr * 4.0f * gear_ratio;
    encoder->cm_per_pulse = circumference / pulses_per_rev;

    /* 初始化状态 */
    encoder->total_count       = 0;
    encoder->last_count        = 0;
    encoder->total_distance_cm = 0.0f;
    encoder->speed_cm_s        = 0.0f;
    encoder->speed_filtered    = 0.0f;
    encoder->last_update_tick  = HAL_GetTick();

    /* 启动编码器模式 */
    HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);

    /* 重置定时器计数 */
    __HAL_TIM_SET_COUNTER(htim, 0);

    encoder->initialized = true;
    return HAL_OK;
}

HAL_StatusTypeDef Encoder_Update(Encoder_t *encoder)
{
    if (encoder == NULL || !encoder->initialized) return HAL_ERROR;

    uint32_t now = HAL_GetTick();
    uint32_t dt  = now - encoder->last_update_tick;

    /* 防止dt为0导致除零 */
    if (dt == 0) return HAL_OK;

    /* 读取当前计数值（16位有符号处理溢出） */
    int16_t current_raw = (int16_t)__HAL_TIM_GET_COUNTER(encoder->htim);

    /* 计算增量（正确处理16位溢出） */
    int16_t delta = (int16_t)(current_raw - (int16_t)(encoder->last_count & 0xFFFF));

    /* 更新累计计数 */
    encoder->total_count += delta;
    encoder->last_count   = current_raw;

    /* 计算距离增量(cm) */
    float delta_dist = (float)delta * encoder->cm_per_pulse;
    encoder->total_distance_cm += delta_dist;

    /* 计算瞬时速度(cm/s) */
    float speed_raw = delta_dist / ((float)dt / 1000.0f);
    encoder->speed_cm_s = speed_raw;

    /* 一阶低通滤波: y = alpha * x + (1-alpha) * y_prev */
    encoder->speed_filtered = SPEED_SMOOTH_ALPHA * speed_raw
                            + (1.0f - SPEED_SMOOTH_ALPHA) * encoder->speed_filtered;

    encoder->last_update_tick = now;
    return HAL_OK;
}

/**
 * @brief 获取滤波后的速度 (cm/s)
 */
float Encoder_GetSpeed(const Encoder_t *encoder)
{
    if (encoder == NULL) return 0.0f;
    return encoder->speed_filtered;
}

/**
 * @brief 获取瞬时速度（未滤波, cm/s）
 */
float Encoder_GetSpeedRaw(const Encoder_t *encoder)
{
    if (encoder == NULL) return 0.0f;
    return encoder->speed_cm_s;
}

/**
 * @brief 获取累计行驶距离 (cm)
 */
float Encoder_GetDistance(const Encoder_t *encoder)
{
    if (encoder == NULL) return 0.0f;
    return encoder->total_distance_cm;
}

/**
 * @brief 重置编码器（清零累计距离和速度）
 */
HAL_StatusTypeDef Encoder_Reset(Encoder_t *encoder)
{
    if (encoder == NULL || !encoder->initialized) return HAL_ERROR;

    encoder->total_count       = 0;
    encoder->total_distance_cm = 0.0f;
    encoder->speed_cm_s        = 0.0f;
    encoder->speed_filtered    = 0.0f;
    __HAL_TIM_SET_COUNTER(encoder->htim, 0);
    encoder->last_count        = 0;
    encoder->last_update_tick  = HAL_GetTick();

    return HAL_OK;
}

/**
 * @brief 获取累计脉冲计数
 */
int32_t Encoder_GetCount(const Encoder_t *encoder)
{
    if (encoder == NULL) return 0;
    return encoder->total_count;
}
