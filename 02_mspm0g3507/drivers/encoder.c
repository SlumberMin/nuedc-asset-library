/**
 * @file    encoder.c
 * @brief   N20霍尔编码器驱动实现 — MSPM0G3507
 *
 * SysConfig生成的宏:
 *   ENCODER_PORT, ENCODER_E1A_PIN, ENCODER_E1B_PIN, ENCODER_E2A_PIN, ENCODER_E2B_PIN
 *   ENCODER_INT_IRQN
 *   TIMER_0_INST, TIMER_0_INST_INT_IRQN
 */

#include "drivers/encoder.h"

/* ── 编码器数据 ───────────────────────────────────────────── */
static EncoderData g_enc[2] = {
    {0, 0, 0},
    {0, 0, 0},
};

/* ── 内部: 处理A相中断 ───────────────────────────────────── */
static void HandlePhaseA(EncoderChannel ch, uint32_t pinB)
{
    if (ch == ENC_LEFT) {
        if (DL_GPIO_readPins(ENCODER_PORT, pinB)) {
            g_enc[ch].count++;
        } else {
            g_enc[ch].count--;
        }
    } else {
        /* 右轮方向相反 */
        if (DL_GPIO_readPins(ENCODER_PORT, pinB)) {
            g_enc[ch].count--;
        } else {
            g_enc[ch].count++;
        }
    }
}

/* ── 公开API ─────────────────────────────────────────────── */

void Encoder_Init(void)
{
    NVIC_ClearPendingIRQ(ENCODER_INT_IRQN);
    NVIC_EnableIRQ(ENCODER_INT_IRQN);

    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
}

int32_t Encoder_GetCount(EncoderChannel ch)
{
    if (ch > ENC_RIGHT) return 0;
    return g_enc[ch].count;
}

int32_t Encoder_GetSpeed(EncoderChannel ch)
{
    if (ch > ENC_RIGHT) return 0;
    return g_enc[ch].speed;
}

void Encoder_Reset(EncoderChannel ch)
{
    if (ch > ENC_RIGHT) return;
    g_enc[ch].count = 0;
    g_enc[ch].speed = 0;
    g_enc[ch].last_count = 0;
}

void Encoder_SampleCallback(void)
{
    g_enc[ENC_LEFT].speed  = g_enc[ENC_LEFT].count  - g_enc[ENC_LEFT].last_count;
    g_enc[ENC_RIGHT].speed = g_enc[ENC_RIGHT].count - g_enc[ENC_RIGHT].last_count;
    g_enc[ENC_LEFT].last_count  = g_enc[ENC_LEFT].count;
    g_enc[ENC_RIGHT].last_count = g_enc[ENC_RIGHT].count;
}

void Encoder_GPIO_IRQHandler(void)
{
    uint32_t flags = DL_GPIO_getEnabledInterruptStatus(
        ENCODER_PORT,
        ENCODER_E1A_PIN | ENCODER_E2A_PIN
    );

    if (flags & ENCODER_E1A_PIN) {
        HandlePhaseA(ENC_LEFT, ENCODER_E1B_PIN);
    }

    if (flags & ENCODER_E2A_PIN) {
        HandlePhaseA(ENC_RIGHT, ENCODER_E2B_PIN);
    }

    DL_GPIO_clearInterruptStatus(ENCODER_PORT,
        ENCODER_E1A_PIN | ENCODER_E2A_PIN);
}
