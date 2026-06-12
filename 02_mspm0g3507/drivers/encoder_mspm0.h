/**
 * @file    encoder_mspm0.h
 * @brief   编码器驱动 — MSPM0G3507 (定时器编码器模式)
 * @note    MSPM0G3507 TIMG 支持正交编码器输入模式
 *          QEI 模式自动解码 A/B 相，无需外部中断
 */

#ifndef __ENCODER_MSPM0_H
#define __ENCODER_MSPM0_H

#include <stdint.h>
#include "platform/driverlib_mspm0.h"

typedef enum {
    ENCODER_LEFT  = 0,
    ENCODER_RIGHT = 1,
    ENCODER_CH_MAX
} EncoderId;

/* ── API ──────────────────────────────────────────────────── */

/**
 * @brief 初始化编码器
 * @param timer    TIMG 实例 (如 TIMG12)
 * @param period   自动重装载值 (计数溢出值)
 */
void Encoder_Init(TIMER_Regs *timer, uint16_t period);

/**
 * @brief 读取并清零编码器累计值
 * @param id  编码器编号
 * @return 累计脉冲数 (有符号)
 */
int32_t Encoder_Read(EncoderId id);

/**
 * @brief 获取当前计数器原始值 (不重置)
 */
int16_t Encoder_GetCount(EncoderId id);

/**
 * @brief 设置编码器方向修正 (接线反接时翻转)
 */
void Encoder_SetInverted(EncoderId id, uint8_t inv);

#endif /* __ENCODER_MSPM0_H */
