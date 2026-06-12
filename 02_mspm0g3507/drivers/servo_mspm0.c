/**
 * @file    servo_mspm0.c
 * @brief   舵机驱动实现 — MSPM0G3507
 */

#include "servo_mspm0.h"

/* ── 私有变量 ────────────────────────────────────────────── */
static TIMER_Regs *g_servo_timer   = NULL;
static uint32_t    g_servo_channel = 0;
static uint16_t    g_pulse_min_us  = SERVO_PULSE_MIN_US;
static uint16_t    g_pulse_max_us  = SERVO_PULSE_MAX_US;

/* ── API ─────────────────────────────────────────────────── */

void Servo_Init(TIMER_Regs *timer, uint32_t channel)
{
    g_servo_timer   = timer;
    g_servo_channel = channel;

    /* 配置 PWM 周期 20ms (50Hz) */
    DL_Timer_setLoadValue(timer, SERVO_PWM_PERIOD);

    /* 初始居中 */
    Servo_SetAngle(90);
}

void Servo_SetAngle(uint8_t angle)
{
    if (angle < SERVO_ANGLE_MIN) angle = SERVO_ANGLE_MIN;
    if (angle > SERVO_ANGLE_MAX) angle = SERVO_ANGLE_MAX;

    /* 线性映射角度到脉宽 */
    uint16_t pulse_us = g_pulse_min_us +
        (uint32_t)(g_pulse_max_us - g_pulse_min_us) * angle / SERVO_ANGLE_MAX;

    Servo_SetPulse_us(pulse_us);
}

void Servo_SetPulse_us(uint16_t pulse_us)
{
    if (g_servo_timer == NULL) return;

    /* us → 计数值: count = pulse_us * (freq_periph / 1_000_000) */
    uint32_t count = (uint32_t)pulse_us * (MSPM0_PERIPH_CLK_HZ / 1000000UL);
    if (count > SERVO_PWM_PERIOD) count = SERVO_PWM_PERIOD;

    PWM_SET_DUTY(g_servo_timer, g_servo_channel, count);
}

void Servo_SetRange(uint16_t min_us, uint16_t max_us)
{
    g_pulse_min_us = min_us;
    g_pulse_max_us = max_us;
}
