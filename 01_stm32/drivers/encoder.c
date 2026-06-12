/**
 * @file    encoder.c
 * @brief   编码器驱动模块实现
 * @details 使用STM32 TIM硬件编码器模式，四倍频计数。
 *          通过定时读取CNT计算速度和累计距离。
 */

#include "drivers/encoder.h"

/* ========================================================================== */
/*                              接口函数实现                                   */
/* ========================================================================== */

ErrorCode_t Encoder_Init(Encoder_t *encoder, TIM_HandleTypeDef *htim,
                         uint16_t ppr, float wheel_diameter_cm, float gear_ratio)
{
    if (encoder == NULL || htim == NULL) {
        return HAL_ERR_PARAM;
    }
    if (ppr == 0 || wheel_diameter_cm <= 0 || gear_ratio <= 0) {
        return HAL_ERR_PARAM;
    }

    /* 保存配置 */
    encoder->htim              = htim;
    encoder->ppr               = ppr;
    encoder->wheel_diameter_cm = wheel_diameter_cm;
    encoder->gear_ratio        = gear_ratio;

    /*
     * 四倍频后的每转脉冲数 = ppr * 4
     * 轮子每转一圈的距离 = π * diameter
     * 每个脉冲对应距离 = 轮子每转距离 / (ppr * 4 * gear_ratio)
     */
    float wheel_circumference = 3.14159265f * wheel_diameter_cm;
    float pulses_per_rev      = (float)ppr * 4.0f * gear_ratio;
    encoder->cm_per_pulse     = wheel_circumference / pulses_per_rev;

    /* 初始化状态 */
    encoder->total_count       = 0;
    encoder->last_count        = 0;
    encoder->total_distance_cm = 0.0f;
    encoder->speed_cm_s        = 0.0f;
    encoder->last_update_tick  = HAL_GetTick();

    /* 启动编码器模式 */
    HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);

    /* 重置定时器计数 */
    __HAL_TIM_SET_COUNTER(htim, 0);

    encoder->initialized = true;

    DBG_PRINTF("Encoder init OK: ppr=%d, dia=%.1fcm, ratio=%.1f, cm/pulse=%.4f",
               ppr, wheel_diameter_cm, gear_ratio, encoder->cm_per_pulse);

    return HAL_OK_CODE;
}

ErrorCode_t Encoder_Update(Encoder_t *encoder)
{
    if (encoder == NULL || !encoder->initialized) {
        return HAL_ERR_NOT_INIT;
    }

    uint32_t now = HAL_GetTick();
    uint32_t dt  = now - encoder->last_update_tick;

    /* 防止dt为0导致除零 */
    if (dt == 0) {
        return HAL_OK_CODE;
    }

    /* 读取当前计数值（16位，有符号处理） */
    int16_t current_count_raw = (int16_t)__HAL_TIM_GET_COUNTER(encoder->htim);

    /*
     * 计算增量：处理16位定时器的溢出
     * current_count_raw 是16位有符号值
     * last_count 存储的是上一次的16位有符号值
     * 直接做有符号减法即可正确处理溢出
     */
    int16_t delta = (int16_t)(current_count_raw - (int16_t)(encoder->last_count & 0xFFFF));

    /* 更新累计计数 */
    encoder->total_count += delta;
    encoder->last_count   = current_count_raw;

    /* 计算这段时间内的距离增量(cm) */
    float delta_distance = (float)delta * encoder->cm_per_pulse;
    encoder->total_distance_cm += delta_distance;

    /* 计算速度(cm/s)：delta_distance / dt(s) */
    encoder->speed_cm_s = delta_distance / ((float)dt / 1000.0f);

    encoder->last_update_tick = now;

    return HAL_OK_CODE;
}

float Encoder_GetSpeed(const Encoder_t *encoder)
{
    if (encoder == NULL) {
        return 0.0f;
    }
    return encoder->speed_cm_s;
}

float Encoder_GetDistance(const Encoder_t *encoder)
{
    if (encoder == NULL) {
        return 0.0f;
    }
    return encoder->total_distance_cm;
}

ErrorCode_t Encoder_Reset(Encoder_t *encoder)
{
    if (encoder == NULL || !encoder->initialized) {
        return HAL_ERR_NOT_INIT;
    }

    encoder->total_count       = 0;
    encoder->total_distance_cm = 0.0f;
    encoder->speed_cm_s        = 0.0f;
    __HAL_TIM_SET_COUNTER(encoder->htim, 0);
    encoder->last_count        = 0;
    encoder->last_update_tick  = HAL_GetTick();

    return HAL_OK_CODE;
}

int32_t Encoder_GetCount(const Encoder_t *encoder)
{
    if (encoder == NULL) {
        return 0;
    }
    return encoder->total_count;
}
