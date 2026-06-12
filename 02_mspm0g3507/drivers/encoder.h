/**
 * @file    encoder.h
 * @brief   N20霍尔编码器驱动 — MSPM0G3507
 *
 * 使用GPIO外部中断+定时器采样实现编码器计数
 *
 * 硬件连接:
 *   PB0 → E1A (左轮A相, 上升沿中断)
 *   PB1 → E1B (左轮B相, GPIO输入)
 *   PB4 → E2A (右轮A相, 上升沿中断)
 *   PB5 → E2B (右轮B相, GPIO输入)
 */

#ifndef __ENCODER_H
#define __ENCODER_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

/* ── 编码器通道 ───────────────────────────────────────────── */
typedef enum {
    ENC_LEFT  = 0,   /* 左轮 */
    ENC_RIGHT = 1,   /* 右轮 */
} EncoderChannel;

/* ── 编码器数据结构 ───────────────────────────────────────── */
typedef struct {
    volatile int32_t count;        /* 累计脉冲数 */
    volatile int32_t speed;        /* 采样周期内的脉冲数(速度) */
    volatile int32_t last_count;   /* 上次采样时的累计值 */
} EncoderData;

/* ── API ──────────────────────────────────────────────────── */

/**
 * @brief 初始化编码器
 *        使能GPIO中断和定时器中断
 */
void Encoder_Init(void);

/**
 * @brief 获取编码器累计脉冲数
 * @param ch  编码器通道
 * @return 累计脉冲数（可正可负）
 */
int32_t Encoder_GetCount(EncoderChannel ch);

/**
 * @brief 获取编码器速度（最近一个采样周期的脉冲数）
 * @param ch  编码器通道
 * @return 速度值（脉冲/采样周期）
 */
int32_t Encoder_GetSpeed(EncoderChannel ch);

/**
 * @brief 清零编码器累计值
 * @param ch  编码器通道
 */
void Encoder_Reset(EncoderChannel ch);

/**
 * @brief 编码器采样回调（在定时器中断中调用）
 *        计算速度并清零周期计数
 */
void Encoder_SampleCallback(void);

/**
 * @brief GPIO中断处理（在GROUP1_IRQHandler中调用）
 */
void Encoder_GPIO_IRQHandler(void);

#endif /* __ENCODER_H */
