/**
 * @file    sensor_ir_mspm0.c
 * @brief   红外循迹传感器实现 — MSPM0G3507
 */

#include "sensor_ir_mspm0.h"

/* ── 私有变量 ────────────────────────────────────────────── */
static IRConfig g_ir_cfg;
static uint8_t  g_ir_state[IR_MAX_CHANNELS] = {0};

/* ── API ─────────────────────────────────────────────────── */

void IR_Init(const IRConfig *cfg)
{
    g_ir_cfg = *cfg;
}

void IR_ReadDigital(uint8_t *buf, uint8_t len)
{
    if (len > g_ir_cfg.num_sensors) len = g_ir_cfg.num_sensors;

    for (uint8_t i = 0; i < len; i++) {
        uint8_t val = GPIO_READ(g_ir_cfg.port[i], g_ir_cfg.pin[i]) ? 1 : 0;
        if (g_ir_cfg.inverted) val ^= 1;
        buf[i] = val;
        g_ir_state[i] = val;
    }
}

/* ── ADC超时计数 ─────────────────────────────────────────── */
#ifndef ADC_TIMEOUT_COUNT
#define ADC_TIMEOUT_COUNT   100000
#endif

void IR_ReadAnalog(uint16_t *buf, uint8_t len)
{
    if (len > g_ir_cfg.num_sensors) len = g_ir_cfg.num_sensors;

    for (uint8_t i = 0; i < len; i++) {
        /* 启动 ADC 转换 */
        DL_ADC12_startConversion(g_ir_cfg.adc);
        /* 等待完成 (带超时) */
        uint32_t timeout = ADC_TIMEOUT_COUNT;
        while (!DL_ADC12_getRawInterruptStatus(g_ir_cfg.adc,
                DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED) && --timeout) {}
        if (timeout == 0) {
            buf[i] = 0;
            continue;
        }
        buf[i] = DL_ADC12_getMemResult(g_ir_cfg.adc,
                    (DL_ADC12_MemIdx)g_ir_cfg.adc_channels[i]);

        if (g_ir_cfg.inverted) {
            buf[i] = 4095 - buf[i];  /* 12-bit 反转 */
        }

        g_ir_state[i] = (buf[i] > g_ir_cfg.threshold) ? 1 : 0;
    }
}

int16_t IR_GetTrackError(void)
{
    /*
     * 加权平均法:
     *   权重: -4, -3, -2, -1, 0, 1, 2, 3 (8路示例)
     *   error = Σ(state[i] * weight[i]) / Σ(state[i])
     */
    int32_t sum_weight = 0;
    int32_t sum_state  = 0;
    uint8_t n = g_ir_cfg.num_sensors;

    for (uint8_t i = 0; i < n; i++) {
        int16_t weight = (int16_t)i - (int16_t)(n / 2);  /* 对称权重 */
        sum_weight += (int32_t)g_ir_state[i] * weight;
        sum_state  += g_ir_state[i];
    }

    if (sum_state == 0) return 0;  /* 脱线返回 0 */
    return (int16_t)(sum_weight / sum_state);
}

uint8_t IR_IsOffTrack(void)
{
    /* 检查是否所有传感器状态相同 (全白或全黑) */
    uint8_t first = g_ir_state[0];
    for (uint8_t i = 1; i < g_ir_cfg.num_sensors; i++) {
        if (g_ir_state[i] != first) return 0;  /* 至少有差异, 在线上 */
    }
    return 1;  /* 全部相同 = 脱线 */
}
