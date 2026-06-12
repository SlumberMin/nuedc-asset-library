/**
 * @file    motor_mspm0.h
 * @brief   TB6612FNG 电机驱动 — MSPM0G3507 版
 * @note    每个 TB6612 控制 2 路直流电机
 *          AIN1/AIN2 → GPIO 控制方向
 *          PWMA      → TIMG PWM 输出控制速度
 *
 * 接线示例:
 *   MSPM0 PA0 → AIN1   PA1 → AIN2   PA12(PWM) → PWMA
 *   MSPM0 PA2 → BIN1   PA3 → BIN2   PA13(PWM) → PWMB
 */

#ifndef __MOTOR_MSPM0_H
#define __MOTOR_MSPM0_H

#include <stdint.h>
#include "platform/driverlib_mspm0.h"

/* ── 电机标识 ────────────────────────────────────────────── */
typedef enum {
    MOTOR_A = 0,
    MOTOR_B = 1,
    MOTOR_MAX
} MotorId;

/* ── 旋转方向 ────────────────────────────────────────────── */
typedef enum {
    MOTOR_DIR_FORWARD = 0,
    MOTOR_DIR_REVERSE = 1,
    MOTOR_DIR_BRAKE   = 2,
    MOTOR_DIR_STOP    = 3
} MotorDir;

/* ── 电机配置结构 ────────────────────────────────────────── */
typedef struct {
    GPIO_Regs *port_in1;
    uint32_t   pin_in1;
    GPIO_Regs *port_in2;
    uint32_t   pin_in2;
    TIMER_Regs *pwm_timer;
    uint32_t    pwm_channel;   /* DL_TIMER_CC_0_INDEX 等 */
    uint16_t    pwm_period;    /* 对应 100% 占空比的计数值 */
} MotorConfig;

/* ── API ──────────────────────────────────────────────────── */

/**
 * @brief 初始化电机驱动
 * @param cfg  指向 MotorConfig 数组 (长度 MOTOR_MAX)
 */
void Motor_Init(const MotorConfig *cfg);

/**
 * @brief 设置电机速度和方向
 * @param id     电机编号 MOTOR_A / MOTOR_B
 * @param speed  速度值 -1000 ~ +1000 (负值反转, 0停车)
 */
void Motor_SetSpeed(MotorId id, int16_t speed);

/** 电机刹车 */
void Motor_Brake(MotorId id);

/** 停止 (滑行) */
void Motor_Stop(MotorId id);

/**
 * @brief 获取当前电机PWM占空比
 * @param id  电机编号 MOTOR_A / MOTOR_B
 * @return 当前PWM值 (0 ~ pwm_period)
 */
int16_t Motor_GetPWM(MotorId id);

#endif /* __MOTOR_MSPM0_H */
