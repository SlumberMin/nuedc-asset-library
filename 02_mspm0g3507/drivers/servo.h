/**
 * @file    servo.h
 * @brief   SG90舵机驱动 — MSPM0G3507
 *
 * 硬件连接:
 *   MSPM0 PA8 → Servo Signal (TIMA0 CH0 PWM)
 *
 * SG90参数:
 *   PWM周期: 20ms (50Hz)
 *   脉宽: 0.5ms(0°) ~ 2.5ms(180°)
 *
 * SysConfig配置:
 *   TIMA0, clockDivider=8 → 4MHz, prescale=1 → 2MHz
 *   period=40000 → 40000/2MHz = 20ms = 50Hz
 *
 * 依赖: ti_msp_dl_config.h (SysConfig生成)
 */

#ifndef __SERVO_H
#define __SERVO_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

/* ── 舵机参数 (2MHz定时器时钟) ────────────────────────────── */
#define SERVO_TIM_CLK       2000000U    /* 4MHz / prescale(1+1) = 2MHz */
#define SERVO_PERIOD        40000U      /* 2MHz / 50Hz = 40000 ticks */
#define SERVO_MIN_PULSE     1000U       /* 0.5ms × 2MHz = 1000 (0°) */
#define SERVO_MAX_PULSE     5000U       /* 2.5ms × 2MHz = 5000 (180°) */
#define SERVO_MAX_ANGLE     180U

/* ── API ──────────────────────────────────────────────────── */

/**
 * @brief 初始化SG90舵机
 *        重新配置TIMA0预分频为2MHz, 设置初始角度90°, 启动PWM输出
 */
void Servo_Init(void);

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

#endif /* __SERVO_H */
