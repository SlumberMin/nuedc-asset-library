/**
 * @file    ws2812_stm32.h
 * @brief   WS2812 彩灯驱动 — STM32 HAL库版本（PWM+DMA）
 * @details 使用TIM PWM + DMA方式驱动WS2812灯带。
 *          每个WS2812 bit对应一个PWM周期（800kHz, 1.25us/bit）。
 *          通过改变占空比区分0码和1码。
 * @version 1.0
 * @date    2026-06
 */

#ifndef __WS2812_STM32_H
#define __WS2812_STM32_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ── WS2812 参数 ─────────────────────────────────────────── */
/* PWM参数 (假设TIM时钟为72MHz, PSC=0):
 *   ARR = 89 → PWM频率 = 72MHz / 90 = 800kHz (1.25us/周期)
 *   T0H = 28  → 高电平 28/90 ≈ 0.31us (数据0)
 *   T1H = 58  → 高电平 58/90 ≈ 0.64us (数据1)
 *   Reset = 50个零周期 (>50us)
 */
#define WS2812_TIM_ARR          (89)
#define WS2812_T0H              (28)
#define WS2812_T1H              (58)
#define WS2812_BITS_PER_LED     (24)    /* GRB, 每通道8bit */
#define WS2812_RESET_LEN        (50)    /* 复位码长度(个PWM周期) */

#ifndef WS2812_MAX_LEDS
#define WS2812_MAX_LEDS         (64)    /* 最大LED数量(可覆盖) */
#endif

/* ── RGB 颜色结构 ────────────────────────────────────────── */
typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} WS2812_Color_t;

/* ── API ──────────────────────────────────────────────────── */

/**
 * @brief 初始化WS2812驱动
 * @param htim    TIM句柄指针（已配置PWM模式+DMA）
 * @param channel TIM通道 (TIM_CHANNEL_1 ~ TIM_CHANNEL_4)
 * @param num_leds LED灯珠数量
 * @return true=成功
 */
bool WS2812_Init(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t num_leds);

/**
 * @brief 设置单个LED颜色
 * @param index LED索引 (从0开始)
 * @param color RGB颜色
 */
void WS2812_SetPixel(uint16_t index, WS2812_Color_t color);

/**
 * @brief 设置单个LED颜色(RGB分量)
 * @param index LED索引
 * @param r 红 (0~255)
 * @param g 绿 (0~255)
 * @param b 蓝 (0~255)
 */
void WS2812_SetPixelRGB(uint16_t index, uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief 全部设置为同一颜色
 * @param color RGB颜色
 */
void WS2812_FillAll(WS2812_Color_t color);

/**
 * @brief 清除所有LED（全部熄灭）
 */
void WS2812_Clear(void);

/**
 * @brief 通过DMA发送数据到灯带（阻塞等待完成）
 * @return true=成功
 */
bool WS2812_Show(void);

/**
 * @brief DMA传输完成回调（在HAL_TIM_PWM_PulseFinishedCallback中调用）
 */
void WS2812_DMACallback(void);

/**
 * @brief 获取当前忙碌状态
 * @return true=DMA正在传输中
 */
bool WS2812_IsBusy(void);

#endif /* __WS2812_STM32_H */
