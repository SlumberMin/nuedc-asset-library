/**
 * @file    servo.c
 * @brief   SG90舵机驱动实现 — MSPM0G3507
 *
 * 工作原理:
 *   SysConfig配置TIMA0: clockDivider=8 → 4MHz, period=40000
 *   Servo_Init()修改prescale=1 → 有效时钟2MHz
 *   period=40000 / 2MHz = 20ms = 50Hz (SG90要求)
 *   compare值: 1000(0.5ms/0°) ~ 5000(2.5ms/180°)
 *
 * SysConfig生成的宏:
 *   SERVO_INST, GPIO_SERVO_C0_IDX
 */

#include "drivers/servo.h"

/* ── 内部: 设置比较值 ─────────────────────────────────────── */
static void Servo_SetCompare(uint32_t compare)
{
    DL_TimerA_setCaptureCompareValue(SERVO_INST, compare, GPIO_SERVO_C0_IDX);
}

/* ── 公开API ─────────────────────────────────────────────── */

void Servo_Init(void)
{
    /* 重新配置时钟: prescale=1 → 4MHz/(0+1)/8/(1+1) = 2MHz */
    DL_TimerA_ClockConfig clockCfg = {
        .clockSel    = DL_TIMER_CLOCK_BUSCLK,
        .divideRatio = DL_TIMER_CLOCK_DIVIDE_8,
        .prescale    = 1U
    };
    DL_TimerA_setClockConfig(SERVO_INST, &clockCfg);

    /* 设置初始位置90° (1.5ms → 3000) */
    Servo_SetCompare(3000);

    /* 启动定时器 */
    DL_TimerA_startCounter(SERVO_INST);
}

void Servo_SetAngle(uint16_t angle)
{
    if (angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;

    /* 线性映射: angle [0,180] → pulse [1000,5000] */
    uint32_t pulse = SERVO_MIN_PULSE +
                     (uint32_t)(SERVO_MAX_PULSE - SERVO_MIN_PULSE) * angle
                     / SERVO_MAX_ANGLE;

    Servo_SetCompare(pulse);
}

void Servo_SetPulseWidth(uint16_t pulse_us)
{
    if (pulse_us < 500)  pulse_us = 500;
    if (pulse_us > 2500) pulse_us = 2500;

    /* 微秒转tick: ticks = pulse_us * 2MHz / 1000000 = pulse_us * 2 */
    uint32_t compare = (uint32_t)pulse_us * 2U;
    Servo_SetCompare(compare);
}

void Servo_Stop(void)
{
    DL_TimerA_stopCounter(SERVO_INST);
    Servo_SetCompare(0);
}
