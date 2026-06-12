/**
 * @file    grayscale_mspm0.c
 * @brief   感为8路灰度传感器驱动实现 — MSPM0G3507
 * @note    使用ADC读取模拟值，通过地址线切换通道
 *          基于感为无MCU灰度传感器官方例程优化
 */

#include "grayscale_mspm0.h"
#include <string.h>

/* ── 私有变量 ────────────────────────────────────────────── */
static GrayscaleConfig g_gray_cfg;
static GrayscaleData   g_gray_data;

/* ── ADC超时计数 ─────────────────────────────────────────── */
#ifndef ADC_TIMEOUT_COUNT
#define ADC_TIMEOUT_COUNT   100000
#endif

/* ── 内部函数 ────────────────────────────────────────────── */

/**
 * @brief 切换传感器通道
 * @param channel  通道号 (0~7)
 */
static void Grayscale_SwitchChannel(uint8_t channel)
{
    /* 通过地址线切换通道 (注意取反逻辑) */
    GPIO_WRITE(g_gray_cfg.addr_port, g_gray_cfg.addr_pin_0, !(channel & 0x01));
    GPIO_WRITE(g_gray_cfg.addr_port, g_gray_cfg.addr_pin_1, !(channel & 0x02));
    GPIO_WRITE(g_gray_cfg.addr_port, g_gray_cfg.addr_pin_2, !(channel & 0x04));
    
    /* 等待传感器切换地址 (IO电平信号翻转有延迟) */
    DELAY_US(10);
}

/**
 * @brief 读取ADC值
 * @return ADC值 (12位)
 */
static uint16_t Grayscale_ReadADC(void)
{
    /* 启动ADC转换 */
    DL_ADC12_startConversion(g_gray_cfg.adc);
    
    /* 等待转换完成 (带超时) */
    uint32_t timeout = ADC_TIMEOUT_COUNT;
    while (!DL_ADC12_getRawInterruptStatus(g_gray_cfg.adc, 
           DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED) && --timeout) {}
    if (timeout == 0) return 0;
    
    /* 读取ADC值 */
    return DL_ADC12_getMemResult(g_gray_cfg.adc, DL_ADC12_MEM_IDX_0);
}

/* ── 公开API ─────────────────────────────────────────────── */

void Grayscale_Init(const GrayscaleConfig *cfg)
{
    /* 保存配置 */
    g_gray_cfg = *cfg;
    
    /* 清空数据 */
    memset(&g_gray_data, 0, sizeof(GrayscaleData));
    
    /* 初始化默认校准值 */
    for (int i = 0; i < GRAYSENSOR_NUM; i++) {
        g_gray_data.white_cal[i] = 3000;  /* 默认白校准值 */
        g_gray_data.black_cal[i] = 1000;  /* 默认黑校准值 */
    }
    
    g_gray_data.initialized = 1;
}

void Grayscale_Calibrate(uint16_t *white, uint16_t *black)
{
    /* 保存校准值 */
    for (int i = 0; i < GRAYSENSOR_NUM; i++) {
        g_gray_data.white_cal[i] = white[i];
        g_gray_data.black_cal[i] = black[i];
        
        /* 计算阈值 */
        g_gray_data.threshold_high[i] = (white[i] * 2 + black[i]) / 3;
        g_gray_data.threshold_low[i] = (white[i] + black[i] * 2) / 3;
        
        /* 计算归一化系数 */
        int16_t diff = white[i] - black[i];
        if (diff > 0) {
            g_gray_data.normal_factor[i] = 4096.0f / diff;  /* 12位ADC */
        } else {
            g_gray_data.normal_factor[i] = 0.0f;
        }
    }
}

void Grayscale_Read(void)
{
    if (!g_gray_data.initialized) return;
    
    /* 读取8路传感器数据 */
    for (int i = 0; i < GRAYSENSOR_NUM; i++) {
        /* 切换通道 */
        Grayscale_SwitchChannel(i);
        
        /* 读取ADC值 */
        uint16_t adc_val = Grayscale_ReadADC();
        
        /* 根据方向存储 */
        uint8_t index = g_gray_cfg.direction ? (GRAYSENSOR_NUM - 1 - i) : i;
        g_gray_data.analog[index] = adc_val;
        
        /* 归一化处理 */
        if (g_gray_data.normal_factor[index] > 0) {
            int16_t normalized = (int16_t)((adc_val - g_gray_data.black_cal[index]) * 
                                  g_gray_data.normal_factor[index]);
            if (normalized < 0) normalized = 0;
            if (normalized > 4095) normalized = 4095;
            g_gray_data.normalized[index] = normalized;
        }
        
        /* 数字量处理 */
        if (adc_val > g_gray_data.threshold_high[index]) {
            g_gray_data.digital |= (1 << index);   /* 白色 */
        } else if (adc_val < g_gray_data.threshold_low[index]) {
            g_gray_data.digital &= ~(1 << index);  /* 黑色 */
        }
    }
}

uint8_t Grayscale_GetDigital(void)
{
    return g_gray_data.digital;
}

void Grayscale_GetAnalog(uint16_t *buf)
{
    memcpy(buf, g_gray_data.analog, sizeof(g_gray_data.analog));
}

void Grayscale_GetNormalized(uint16_t *buf)
{
    memcpy(buf, g_gray_data.normalized, sizeof(g_gray_data.normalized));
}

int16_t Grayscale_GetTrackError(void)
{
    /* 加权平均法计算偏差 */
    int32_t sum = 0;
    int32_t weight_sum = 0;
    
    /* 权重表: 左负右正 */
    const int8_t weights[GRAYSENSOR_NUM] = {-4, -3, -2, -1, 1, 2, 3, 4};
    
    for (int i = 0; i < GRAYSENSOR_NUM; i++) {
        if (g_gray_data.digital & (1 << i)) {
            /* 白色区域 */
            sum += weights[i] * 100;
            weight_sum += 100;
        } else {
            /* 黑色区域 */
            sum += weights[i] * 0;
            weight_sum += 0;
        }
    }
    
    if (weight_sum == 0) {
        /* 脱线 */
        return 0;
    }
    
    /* 计算偏差 */
    int16_t error = (int16_t)(sum * 1000 / weight_sum);
    
    /* 限幅 */
    if (error > 1000) error = 1000;
    if (error < -1000) error = -1000;
    
    return error;
}

uint8_t Grayscale_IsOffTrack(void)
{
    /* 所有传感器都检测到黑色 */
    return (g_gray_data.digital == 0x00);
}

uint8_t Grayscale_DetectCross(void)
{
    /* 所有传感器都检测到白色 */
    return (g_gray_data.digital == 0xFF);
}

uint8_t Grayscale_DetectTJunction(void)
{
    /* 检测丁字路口 (中间传感器检测到黑色，两侧检测到白色) */
    uint8_t left_white = (g_gray_data.digital & 0x07) == 0x07;   /* 左3个白 */
    uint8_t right_white = (g_gray_data.digital & 0xE0) == 0xE0;  /* 右3个白 */
    uint8_t center_black = (g_gray_data.digital & 0x18) == 0x00;  /* 中2个黑 */
    
    return (left_white && right_white && center_black);
}

uint8_t Grayscale_DetectStartLine(void)
{
    /* 检测起跑线 (特定模式: 10011001) */
    uint8_t pattern = g_gray_data.digital;
    return (pattern == 0x99);  /* 10011001 */
}