/**
 * @file    l298n.h
 * @brief   L298N 双路直流电机驱动 — MSPM0G3507
 *
 * 硬件连接:
 *   MSPM0 PA4 → IN1    PA5 → IN2    PA8(PWM) → ENA
 *   MSPM0 PA6 → IN3    PA7 → IN4    PA9(PWM) → ENB
 *
 * 真值表:
 *   IN1=H, IN2=L → 正转    IN1=L, IN2=H → 反转
 *   IN1=L, IN2=L → 停止    IN1=H, IN2=H → 制动
 *
 * 依赖: ti_msp_dl_config.h (SysConfig生成)
 */

#ifndef __L298N_H
#define __L298N_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

/* ── 电机编号 ────────────────────────────────────────────── */
typedef enum {
    L298N_CH_A = 0,   /* A通道 (IN1/IN2, ENA) */
    L298N_CH_B = 1,   /* B通道 (IN3/IN4, ENB) */
} L298N_Channel;

/* ── 旋转方向 ────────────────────────────────────────────── */
typedef enum {
    L298N_DIR_FORWARD = 0,  /* 正转 */
    L298N_DIR_REVERSE = 1,  /* 反转 */
    L298N_DIR_BRAKE   = 2,  /* 制动 (两IN同高) */
    L298N_DIR_STOP    = 3,  /* 停止 (两IN同低) */
} L298N_Direction;

/* ── API ──────────────────────────────────────────────────── */

/**
 * @brief 初始化L298N电机驱动，电机默认停止
 */
void L298N_Init(void);

/**
 * @brief 设置电机速度和方向
 * @param ch    电机通道 L298N_CH_A / L298N_CH_B
 * @param dir   旋转方向
 * @param speed 速度值 0~3999 (对应PWM占空比)
 */
void L298N_SetMotor(L298N_Channel ch, L298N_Direction dir, uint32_t speed);

/**
 * @brief 电机刹车（两个方向引脚都拉高）
 * @param ch 电机通道
 */
void L298N_Brake(L298N_Channel ch);

/**
 * @brief 电机停止（两个方向引脚拉低，PWM输出0）
 * @param ch 电机通道
 */
void L298N_Stop(L298N_Channel ch);

/**
 * @brief 所有电机停止
 */
void L298N_StopAll(void);

#endif /* __L298N_H */
