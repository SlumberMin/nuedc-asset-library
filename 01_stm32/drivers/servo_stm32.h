/**
 * @file    servo_stm32.h
 * @brief   SG90舵机驱动 — STM32 HAL库版本
 *
 * 硬件连接:
 *   PA8 → Servo Signal (TIM1_CH1 PWM)
 *
 * SG90参数:
 *   PWM周期: 20ms (50Hz)
 *   脉宽: 0.5ms(0°) ~ 2.5ms(180°)
 *
 * 定时器配置:
 *   TIM1: PSC=71 → 72MHz/72=1MHz, ARR=19999 → 1MHz/20000=50Hz
 *   TIM1_CH1: PWM模式1
 *   1µs分辨率，脉宽范围 500~2500 (µs)
 */

#ifndef __SERVO_STM32_H
#define __SERVO_STM32_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* ── 舵机参数 ─────────────────────────────────────────────── */
#define SERVO_MIN_PULSE_US  500U    /* 0.5ms → 0°   */
#define SERVO_MAX_PULSE_US  2500U   /* 2.5ms → 180°  */
#define SERVO_CENTER_PULSE  1500U   /* 1.5ms → 90°   */
#define SERVO_MAX_ANGLE     180U

/* ── API ──────────────────────────────────────────────────── */

/**
 * @brief 初始化SG90舵机
 *        配置TIM1_CH1 PWM输出，设置初始角度90°，启动PWM
 * @param htim TIM1句柄指针
 */
void Servo_Init(TIM_HandleTypeDef *htim);

/**
 * @brief 设置舵机角度
 * @param angle  角度 0~180
 */
void Servo_SetAngle(uint16_t angle);

/**
 * @brief 直接设置脉宽
 * @param pulse_us  脉宽(微秒) 500~2500
 */
void Servo_SetPulseWidth(uint16_t pulse_us);

/**
 * @brief 停止舵机PWM输出
 */
void Servo_Stop(void);

#endif /* __SERVO_STM32_H */
