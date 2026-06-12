/**
 * @file    ws2812_stm32.c
 * @brief   WS2812 彩灯驱动实现 — STM32 HAL库版本 (PWM+DMA)
 */

#include "drivers/ws2812_stm32.h"

/* ── PWM DMA 缓冲区: 每个LED 24bit + 复位码 ─────────────── */
static uint16_t g_ws2812_buf[WS2812_MAX_LEDS * WS2812_BITS_PER_LED + WS2812_RESET_LEN];

static TIM_HandleTypeDef *g_ws2812_htim = NULL;
static uint32_t g_ws2812_channel = TIM_CHANNEL_1;
static uint16_t g_ws2812_num_leds = 0;
static volatile bool g_ws2812_busy = false;

/* ── 初始化 ─────────────────────────────────────────────── */
bool WS2812_Init(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t num_leds)
{
    if (num_leds > WS2812_MAX_LEDS) return false;
    g_ws2812_htim = htim;
    g_ws2812_channel = channel;
    g_ws2812_num_leds = num_leds;

    WS2812_Clear();
    return true;
}

/* ── 设置单个LED（GRB顺序）─────────────────────────────── */
void WS2812_SetPixel(uint16_t index, WS2812_Color_t color)
{
    if (index >= g_ws2812_num_leds) return;

    /* WS2812数据格式: GRB, MSB first */
    uint32_t grb = ((uint32_t)color.g << 16) | ((uint32_t)color.r << 8) | color.b;
    uint16_t base = index * WS2812_BITS_PER_LED;

    for (int i = 0; i < WS2812_BITS_PER_LED; i++) {
        if (grb & (1UL << (23 - i)))
            g_ws2812_buf[base + i] = WS2812_T1H;
        else
            g_ws2812_buf[base + i] = WS2812_T0H;
    }
}

/* ── 设置单个LED(RGB分量) ──────────────────────────────── */
void WS2812_SetPixelRGB(uint16_t index, uint8_t r, uint8_t g, uint8_t b)
{
    WS2812_Color_t c = {r, g, b};
    WS2812_SetPixel(index, c);
}

/* ── 全部填充 ───────────────────────────────────────────── */
void WS2812_FillAll(WS2812_Color_t color)
{
    for (uint16_t i = 0; i < g_ws2812_num_leds; i++) {
        WS2812_SetPixel(i, color);
    }
}

/* ── 清除 ───────────────────────────────────────────────── */
void WS2812_Clear(void)
{
    for (uint16_t i = 0; i < g_ws2812_num_leds * WS2812_BITS_PER_LED; i++) {
        g_ws2812_buf[i] = 0;
    }
    /* 复位码: 低电平 */
    for (uint16_t i = 0; i < WS2812_RESET_LEN; i++) {
        g_ws2812_buf[g_ws2812_num_leds * WS2812_BITS_PER_LED + i] = 0;
    }
}

/* ── DMA发送 ────────────────────────────────────────────── */
bool WS2812_Show(void)
{
    if (g_ws2812_busy) return false;
    g_ws2812_busy = true;

    uint16_t total_len = g_ws2812_num_leds * WS2812_BITS_PER_LED + WS2812_RESET_LEN;

    if (HAL_TIM_PWM_Start_DMA(g_ws2812_htim, g_ws2812_channel,
                              (uint32_t *)g_ws2812_buf, total_len) != HAL_OK) {
        g_ws2812_busy = false;
        return false;
    }
    return true;
}

/* ── DMA完成回调 ────────────────────────────────────────── */
void WS2812_DMACallback(void)
{
    HAL_TIM_PWM_Stop_DMA(g_ws2812_htim, g_ws2812_channel);
    g_ws2812_busy = false;
}

/* ── 忙碌状态 ───────────────────────────────────────────── */
bool WS2812_IsBusy(void)
{
    return g_ws2812_busy;
}
