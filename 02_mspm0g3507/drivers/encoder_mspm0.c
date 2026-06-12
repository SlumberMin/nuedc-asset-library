/**
 * @file    encoder_mspm0.c
 * @brief   编码器驱动实现 — MSPM0G3507
 * @note    利用 TIMG 的 QEI (正交解码) 模式
 */

#include "encoder_mspm0.h"

/* ── 私有变量 ────────────────────────────────────────────── */
static TIMER_Regs *g_enc_timer[ENCODER_CH_MAX] = {NULL};
static int32_t     g_enc_accum[ENCODER_CH_MAX] = {0};
static uint16_t    g_enc_period = 65535;
static int8_t      g_enc_dir[ENCODER_CH_MAX] = {1, 1};

/* ── API ─────────────────────────────────────────────────── */

void Encoder_Init(TIMER_Regs *timer, uint16_t period)
{
    g_enc_timer[ENCODER_LEFT]  = timer;
    g_enc_timer[ENCODER_RIGHT] = timer;  /* 同一定时器可分 A/B 相 */
    g_enc_period = period;

    /*
     * TIMG QEI 模式初始化:
     *   DL_TimerG_setCounterMode(timer, DL_TIMERG_COUNTER_MODE_QEI)
     *   DL_TimerG_setLoadValue(timer, period)
     *
     * 实际引脚映射通过 SYSCFG_DL_init() 完成
     *
     * 注: MSPM0G3507 的 QEI 需要通过 Timer 配置为计数模式
     *     DL_TIMER_COUNT_MODE_QEI 是正交编码器模式的正确枚举值
     */
    DL_Timer_setCounterMode(timer, DL_TIMER_COUNT_MODE_QEI);

    DL_Timer_setLoadValue(timer, period);
    DL_Timer_enableCounter(timer);
}

int32_t Encoder_Read(EncoderId id)
{
    if (id >= ENCODER_CH_MAX || g_enc_timer[id] == NULL) return 0;

    /* 读取当前计数值 */
    int16_t cnt = (int16_t)DL_Timer_getTimerCount(g_enc_timer[id]);

    /* 重置计数器 */
    DL_Timer_setTimerCount(g_enc_timer[id], 0);

    /* 方向修正 */
    int32_t val = (int32_t)cnt * g_enc_dir[id];

    g_enc_accum[id] += val;
    return val;
}

int16_t Encoder_GetCount(EncoderId id)
{
    if (id >= ENCODER_CH_MAX || g_enc_timer[id] == NULL) return 0;
    int16_t cnt = (int16_t)DL_Timer_getTimerCount(g_enc_timer[id]);
    return cnt * g_enc_dir[id];
}

void Encoder_SetInverted(EncoderId id, uint8_t inv)
{
    if (id >= ENCODER_CH_MAX) return;
    g_enc_dir[id] = inv ? -1 : 1;
}
