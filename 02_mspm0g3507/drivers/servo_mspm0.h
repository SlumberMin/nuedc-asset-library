/**
 * @file    servo_mspm0.h
 * @brief   舵机驱动 — MSPM0G3507
 * @note    标准 50Hz PWM (20ms 周期)
 *          0.5ms~2.5ms 脉宽对应 0°~180°
 */

#ifndef __SERVO_MSPM0_H
#define __SERVO_MSPM0_H

#include <stdint.h>
#include "platform/driverlib_mspm0.h"

/* 舵机 PWM 参数 (80MHz 外设时钟, 分频可调) */
#define SERVO_PWM_FREQ_HZ       50UL     /* 50 Hz */
#define SERVO_PWM_PERIOD         (MSPM0_PERIPH_CLK_HZ / SERVO_PWM_FREQ_HZ)  /* 1600000 */

#define SERVO_PULSE_MIN_US       500     /* 0° 脉宽 (us) */
#define SERVO_PULSE_MAX_US       2500    /* 180° 脉宽 (us) */
#define SERVO_ANGLE_MIN          0
#define SERVO_ANGLE_MAX          180

/* ── API ──────────────────────────────────────────────────── */

/**
 * @brief 初始化舵机 PWM
 * @param timer    TIMER 实例
 * @param channel  PWM 通道
 */
void Servo_Init(TIMER_Regs *timer, uint32_t channel);

/**
 * @brief 设置舵机角度
 * @param angle  0 ~ 180 度
 */
void Servo_SetAngle(uint8_t angle);

/**
 * @brief 直接设置脉宽 (微秒)
 */
void Servo_SetPulse_us(uint16_t pulse_us);

/**
 * @brief 设置舵机脉宽范围 (校准用)
 */
void Servo_SetRange(uint16_t min_us, uint16_t max_us);

#endif /* __SERVO_MSPM0_H */
