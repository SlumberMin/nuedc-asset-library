/**
 * @file    servo_stm32.c
 * @brief   SG90舵机驱动实现 — STM32 HAL库版本
 *
 * 工作原理:
 *   TIM1: PSC=71 → 72MHz/72=1MHz, ARR=19999 → 50Hz
 *   TIM1_CH1: PWM模式1
 *   比较值: 500(0.5ms/0°) ~ 2500(2.5ms/180°)
 *   1 tick = 1µs
 *
 * 硬件:
 *   PA8: TIM1_CH1 → Servo Signal
 */

#include "drivers/servo_stm32.h"

/* ── 内部变量 ─────────────────────────────────────────────── */
static TIM_HandleTypeDef *g_servo_htim = NULL;

/* ── 内部: 设置比较值 ─────────────────────────────────────── */
static void Servo_SetCompare(uint32_t compare)
{
    if (g_servo_htim == NULL) return;
    __HAL_TIM_SET_COMPARE(g_servo_htim, TIM_CHANNEL_1, compare);
}

/* ── 公开API ──────────────────────────────────────────────── */

void Servo_Init(TIM_HandleTypeDef *htim)
{
    g_servo_htim = htim;

    /* 配置PA8为复用推挽输出 (TIM1_CH1) */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();

    GPIO_InitStruct.Pin   = GPIO_PIN_8;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* 设置初始位置90° (1.5ms) */
    Servo_SetCompare(SERVO_CENTER_PULSE);

    /* 启动PWM输出 */
    HAL_TIM_PWM_Start(g_servo_htim, TIM_CHANNEL_1);

    /* TIM1是高级定时器，需要手动使能MOE（主输出使能）位 */
    __HAL_TIM_MOE_ENABLE(g_servo_htim);
}

void Servo_SetAngle(uint16_t angle)
{
    if (angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;

    /* 线性映射: angle [0,180] → pulse [500,2500] µs */
    uint32_t pulse = SERVO_MIN_PULSE_US +
                     (uint32_t)(SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) * angle
                     / SERVO_MAX_ANGLE;

    Servo_SetCompare(pulse);
}

void Servo_SetPulseWidth(uint16_t pulse_us)
{
    if (pulse_us < SERVO_MIN_PULSE_US)  pulse_us = SERVO_MIN_PULSE_US;
    if (pulse_us > SERVO_MAX_PULSE_US)  pulse_us = SERVO_MAX_PULSE_US;

    /* TIM时钟1MHz，直接用µs作为比较值 */
    Servo_SetCompare((uint32_t)pulse_us);
}

void Servo_Stop(void)
{
    HAL_TIM_PWM_Stop(g_servo_htim, TIM_CHANNEL_1);
    Servo_SetCompare(0);
}
