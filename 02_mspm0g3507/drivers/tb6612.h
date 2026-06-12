/**
 * @file    tb6612.h
 * @brief   TB6612FNG 双路直流电机驱动 — MSPM0G3507
 *
 * 硬件连接:
 *   MSPM0 PA0 → AIN1    PA1 → AIN2    PA12(PWM) → PWMA
 *   MSPM0 PA2 → BIN1    PA3 → BIN2    PA13(PWM) → PWMB
 *
 * 依赖: ti_msp_dl_config.h (SysConfig生成)
 */

#ifndef __TB6612_H
#define __TB6612_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

/* ── 电机编号 ────────────────────────────────────────────── */
typedef enum {
    MOTOR_CH_A = 0,   /* A通道 */
    MOTOR_CH_B = 1,   /* B通道 */
} MotorChannel;

/* ── 旋转方向 ────────────────────────────────────────────── */
typedef enum {
    MOTOR_DIR_FORWARD = 0,  /* 正转 */
    MOTOR_DIR_REVERSE = 1,  /* 反转 */
    MOTOR_DIR_BRAKE   = 2,  /* 制动 */
} MotorDirection;

/* ── API ──────────────────────────────────────────────────── */

/**
 * @brief 初始化TB6612电机驱动
 *        配置GPIO输出和PWM输出，电机默认停止
 */
void TB6612_Init(void);

/**
 * @brief 设置电机速度和方向
 * @param ch    电机通道 MOTOR_CH_A / MOTOR_CH_B
 * @param dir   旋转方向
 * @param speed 速度值 0~3999 (对应PWM占空比)
 */
void TB6612_SetMotor(MotorChannel ch, MotorDirection dir, uint32_t speed);

/**
 * @brief 电机刹车（两个方向引脚都拉高）
 * @param ch 电机通道
 */
void TB6612_Brake(MotorChannel ch);

/**
 * @brief 电机停止（PWM输出0）
 * @param ch 电机通道
 */
void TB6612_Stop(MotorChannel ch);

/**
 * @brief 所有电机停止
 */
void TB6612_StopAll(void);

#endif /* __TB6612_H */
