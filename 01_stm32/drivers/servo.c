/**
 * @file    servo.c
 * @brief   舵机驱动模块实现
 * @details 角度→脉宽→CCR值映射链：
 *          angle → pulse_us → ccr = pulse_us / period_us * (ARR+1)
 */

#include "drivers/servo.h"

/* ========================================================================== */
/*                              内部函数                                       */
/* ========================================================================== */

/**
 * @brief 根据舵机类型设置默认参数（内部函数）
 */
static void Servo_SetDefaultParams(Servo_t *servo, ServoType_t type)
{
    switch (type) {
        case SERVO_TYPE_SG90:
            servo->min_angle    = 0.0f;
            servo->max_angle    = 180.0f;
            servo->min_pulse_us = 500.0f;    /* 0.5ms → 0° */
            servo->max_pulse_us = 2500.0f;   /* 2.5ms → 180° */
            break;
        case SERVO_TYPE_MG996R:
            servo->min_angle    = 0.0f;
            servo->max_angle    = 180.0f;
            servo->min_pulse_us = 500.0f;
            servo->max_pulse_us = 2500.0f;
            break;
        default:
            /* CUSTOM类型在InitCustom中已设置 */
            break;
    }
}

/**
 * @brief 角度转脉宽(μs)（内部函数）
 * @param servo  舵机结构体指针
 * @param angle  角度(°)
 * @return float: 脉宽(μs)
 */
static float Servo_AngleToPulse(const Servo_t *servo, float angle)
{
    /* 防止除零: 若角度范围为0，返回中位脉宽 */
    float angle_range = servo->max_angle - servo->min_angle;
    if (angle_range <= 0.0f) {
        return (servo->min_pulse_us + servo->max_pulse_us) / 2.0f;
    }
    /* 线性映射：angle ∈ [min_angle, max_angle] → pulse ∈ [min_pulse, max_pulse] */
    float ratio = (angle - servo->min_angle) / angle_range;
    float pulse = servo->min_pulse_us + ratio * (servo->max_pulse_us - servo->min_pulse_us);
    return pulse;
}

/**
 * @brief 脉宽转CCR值（内部函数）
 * @param servo     舵机结构体指针
 * @param pulse_us  脉宽(μs)
 * @return uint32_t: CCR比较值
 */
static uint32_t Servo_PulseToCCR(const Servo_t *servo, float pulse_us)
{
    uint32_t arr = servo->htim->Init.Period;
    /* ccr = (pulse_us / period_us) * (ARR + 1) */
    uint32_t ccr = (uint32_t)((pulse_us / servo->pwm_period_us) * (arr + 1));
    return ccr;
}

/* ========================================================================== */
/*                              接口函数实现                                   */
/* ========================================================================== */

ErrorCode_t Servo_Init(Servo_t *servo, TIM_HandleTypeDef *htim,
                       uint32_t channel, ServoType_t type)
{
    if (servo == NULL || htim == NULL) {
        return HAL_ERR_PARAM;
    }
    if (type == SERVO_TYPE_CUSTOM) {
        return HAL_ERR_PARAM;  /* 自定义类型请使用Servo_InitCustom */
    }

    servo->htim         = htim;
    servo->pwm_channel  = channel;
    servo->type         = type;
    servo->pwm_period_us = 20000;  /* 50Hz → 20ms = 20000μs */

    /* 根据类型设置默认参数 */
    Servo_SetDefaultParams(servo, type);

    /* 归中 */
    servo->current_angle = (servo->min_angle + servo->max_angle) / 2.0f;

    /* 启动PWM */
    PWM_START(htim, channel);

    /* 设置到中间位置 */
    Servo_SetAngle(servo, servo->current_angle);

    servo->initialized = true;

    DBG_PRINTF("Servo init OK: type=%d, TIM%p CH%lu, range[%.0f~%.0f]",
               type, (void *)htim, channel, servo->min_angle, servo->max_angle);

    return HAL_OK_CODE;
}

ErrorCode_t Servo_InitCustom(Servo_t *servo, TIM_HandleTypeDef *htim,
                             uint32_t channel, float min_angle, float max_angle,
                             float min_pulse_us, float max_pulse_us)
{
    if (servo == NULL || htim == NULL) {
        return HAL_ERR_PARAM;
    }

    servo->htim          = htim;
    servo->pwm_channel   = channel;
    servo->type          = SERVO_TYPE_CUSTOM;
    servo->pwm_period_us = 20000;
    servo->min_angle     = min_angle;
    servo->max_angle     = max_angle;
    servo->min_pulse_us  = min_pulse_us;
    servo->max_pulse_us  = max_pulse_us;

    servo->current_angle = (min_angle + max_angle) / 2.0f;

    PWM_START(htim, channel);
    Servo_SetAngle(servo, servo->current_angle);

    servo->initialized = true;

    return HAL_OK_CODE;
}

ErrorCode_t Servo_SetAngle(Servo_t *servo, float angle)
{
    if (servo == NULL || !servo->initialized) {
        return HAL_ERR_NOT_INIT;
    }

    /* 限幅 */
    angle = CLAMP(angle, servo->min_angle, servo->max_angle);

    /* 角度→脉宽→CCR */
    float pulse_us = Servo_AngleToPulse(servo, angle);
    uint32_t ccr   = Servo_PulseToCCR(servo, pulse_us);

    PWM_SET(servo->htim, servo->pwm_channel, ccr);
    servo->current_angle = angle;

    return HAL_OK_CODE;
}

float Servo_GetAngle(const Servo_t *servo)
{
    if (servo == NULL) {
        return 0.0f;
    }
    return servo->current_angle;
}

ErrorCode_t Servo_SetPulse(Servo_t *servo, float pulse_us)
{
    if (servo == NULL || !servo->initialized) {
        return HAL_ERR_NOT_INIT;
    }

    /* 限幅脉宽 */
    pulse_us = CLAMP(pulse_us, servo->min_pulse_us, servo->max_pulse_us);

    uint32_t ccr = Servo_PulseToCCR(servo, pulse_us);
    PWM_SET(servo->htim, servo->pwm_channel, ccr);

    /* 反算当前角度 */
    /* [修复#1] 防止max_pulse==min_pulse导致除零 */
    float pulse_range = servo->max_pulse_us - servo->min_pulse_us;
    float ratio = (pulse_range > 0.0f) ? (pulse_us - servo->min_pulse_us) / pulse_range : 0.5f;
    servo->current_angle = servo->min_angle + ratio * (servo->max_angle - servo->min_angle);

    return HAL_OK_CODE;
}

ErrorCode_t Servo_Center(Servo_t *servo)
{
    if (servo == NULL || !servo->initialized) {
        return HAL_ERR_NOT_INIT;
    }

    float center = (servo->min_angle + servo->max_angle) / 2.0f;
    return Servo_SetAngle(servo, center);
}
