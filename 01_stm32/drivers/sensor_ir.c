/**
 * @file    sensor_ir.c
 * @brief   红外循迹传感器模块实现
 * @details ADC模式下读取各路ADC值，阈值比较后加权平均计算位置。
 *          GPIO模式下直接读取数字电平。
 */

#include "drivers/sensor_ir.h"

/* ========================================================================== */
/*                              内部函数                                       */
/* ========================================================================== */

/**
 * @brief 设置默认权重（内部函数）
 * @param ir  传感器结构体指针
 * @note  对称分布，如5路: {-4, -2, 0, 2, 4}
 */
static void SensorIR_SetDefaultWeights(SensorIR_t *ir)
{
    /* 等间距对称权重：从 -(n-1) 到 +(n-1) */
    int8_t max_w = (int8_t)(ir->channel_count - 1);
    for (uint8_t i = 0; i < ir->channel_count; i++) {
        ir->weights[i] = (int8_t)(-max_w + 2 * i);
    }
}

/**
 * @brief ADC模式读取单通道（内部函数）
 * @param hadc    ADC句柄
 * @param channel ADC通道号
 * @return uint16_t: ADC值(0~4095)
 */
static uint16_t SensorIR_ReadADC(ADC_HandleTypeDef *hadc, uint16_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel      = channel;
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
    HAL_ADC_ConfigChannel(hadc, &sConfig);

    HAL_ADC_Start(hadc);
    HAL_ADC_PollForConversion(hadc, 10);
    return (uint16_t)HAL_ADC_GetValue(hadc);
}

/* ========================================================================== */
/*                              接口函数实现                                   */
/* ========================================================================== */

ErrorCode_t SensorIR_InitADC(SensorIR_t *ir, ADC_HandleTypeDef *hadc,
                             const uint16_t *channels, uint8_t count,
                             uint16_t threshold)
{
    if (ir == NULL || hadc == NULL || channels == NULL) {
        return HAL_ERR_PARAM;
    }
    if (count == 0 || count > IR_SENSOR_MAX_CH) {
        return HAL_ERR_PARAM;
    }

    ir->mode          = IR_MODE_ADC;
    ir->hadc          = hadc;
    ir->channel_count = count;
    ir->threshold     = threshold;

    for (uint8_t i = 0; i < count; i++) {
        ir->adc_channel[i] = channels[i];
        ir->raw_value[i]   = 0;
        ir->digital[i]     = false;
    }

    /* 设置默认权重 */
    SensorIR_SetDefaultWeights(ir);

    ir->position        = 0.0f;
    ir->on_line         = false;
    ir->cross_detected  = false;
    ir->initialized     = true;

    DBG_PRINTF("IR sensor init ADC mode, %d channels, threshold=%d", count, threshold);

    return HAL_OK_CODE;
}

ErrorCode_t SensorIR_InitGPIO(SensorIR_t *ir, GPIO_TypeDef **ports,
                              const uint16_t *pins, uint8_t count)
{
    if (ir == NULL || ports == NULL || pins == NULL) {
        return HAL_ERR_PARAM;
    }
    if (count == 0 || count > IR_SENSOR_MAX_CH) {
        return HAL_ERR_PARAM;
    }

    ir->mode          = IR_MODE_GPIO;
    ir->channel_count = count;
    ir->hadc          = NULL;

    for (uint8_t i = 0; i < count; i++) {
        ir->gpio_port[i] = ports[i];
        ir->gpio_pin[i]  = pins[i];
        ir->digital[i]   = false;
    }

    SensorIR_SetDefaultWeights(ir);

    ir->position        = 0.0f;
    ir->on_line         = false;
    ir->cross_detected  = false;
    ir->initialized     = true;

    DBG_PRINTF("IR sensor init GPIO mode, %d channels", count);

    return HAL_OK_CODE;
}

ErrorCode_t SensorIR_SetWeights(SensorIR_t *ir, const int8_t *weights)
{
    if (ir == NULL || weights == NULL) {
        return HAL_ERR_PARAM;
    }

    for (uint8_t i = 0; i < ir->channel_count; i++) {
        ir->weights[i] = weights[i];
    }

    return HAL_OK_CODE;
}

ErrorCode_t SensorIR_Update(SensorIR_t *ir)
{
    if (ir == NULL || !ir->initialized) {
        return HAL_ERR_NOT_INIT;
    }

    uint8_t on_line_count = 0;
    float   weighted_sum  = 0.0f;

    for (uint8_t i = 0; i < ir->channel_count; i++) {
        if (ir->mode == IR_MODE_ADC) {
            /* ADC模式：读取ADC值 */
            ir->raw_value[i] = SensorIR_ReadADC(ir->hadc, ir->adc_channel[i]);
            /*
             * TCRT5000在黑线上：反射弱 → 输出低 → ADC值小
             * 在白地上：反射强 → 输出高 → ADC值大
             * 因此：ADC < threshold → 在黑线上 → digital=false
             */
            ir->digital[i] = (ir->raw_value[i] >= ir->threshold);
        } else {
            /* GPIO模式 */
            GPIO_PinState state = GPIO_READ(ir->gpio_port[i], ir->gpio_pin[i]);
            ir->digital[i] = (state == GPIO_PIN_SET);
            ir->raw_value[i] = (state == GPIO_PIN_SET) ? 1 : 0;
        }

        /*
         * digital[i] = false 表示在黑线上
         * 计算位置时，对在线上(digital=false)的传感器加权
         */
        if (!ir->digital[i]) {
            weighted_sum += (float)ir->weights[i];
            on_line_count++;
        }
    }

    /* 计算位置和状态 */
    if (on_line_count > 0) {
        ir->on_line  = true;
        ir->position = weighted_sum / (float)on_line_count;

        /* 十字路口检测：所有传感器都在黑线上 */
        ir->cross_detected = (on_line_count == ir->channel_count);
    } else {
        /* 所有传感器都不在黑线上 → 脱线 */
        ir->on_line        = false;
        ir->cross_detected = false;
        /* 保持上一次的位置方向（惯性） */
        /* position不变 */
    }

    return HAL_OK_CODE;
}

float SensorIR_GetPosition(const SensorIR_t *ir)
{
    if (ir == NULL) return 0.0f;
    return ir->position;
}

bool SensorIR_IsOnLine(const SensorIR_t *ir)
{
    if (ir == NULL) return false;
    return ir->on_line;
}

bool SensorIR_IsCrossDetected(const SensorIR_t *ir)
{
    if (ir == NULL) return false;
    return ir->cross_detected;
}

uint16_t SensorIR_GetRaw(const SensorIR_t *ir, uint8_t index)
{
    if (ir == NULL || index >= ir->channel_count) return 0;
    return ir->raw_value[index];
}

bool SensorIR_GetDigital(const SensorIR_t *ir, uint8_t index)
{
    if (ir == NULL || index >= ir->channel_count) return false;
    return ir->digital[index];
}
