/**
 * @file    ultrasonic.c
 * @brief   SR04/US-016 超声波测距驱动实现 — MSPM0G3507
 *
 * 测量方法:
 *   1. Trig输出10µs脉冲触发模块 (delay_cycles校准@32MHz)
 *   2. 使用Timer中断维护的全局µs计数器测量Echo脉宽
 *   3. 距离(cm) = 脉宽(µs) / 58
 *
 * SysConfig生成的宏:
 *   ENCODER_PORT, ENCODER_TRIG_PIN, ENCODER_ECHO_PIN
 *   ENCODER_INT_IRQN
 *   TIMER_0_INST (TIMG6, 1ms周期@32MHz)
 *
 * @note 系统时钟: 32MHz (默认)
 *       Timer中断周期: 1ms (32000 ticks)
 *       g_timer_us 在 TIMER_0_INST_IRQHandler 中每1ms累加1000
 */

#include "drivers/ultrasonic.h"

/* ── 外部变量 (在main.c中定义) ─────────────────────────────── */
extern volatile uint32_t g_timer_us;

/* ── 内部变量 ─────────────────────────────────────────────── */
static volatile uint32_t g_echo_start_us = 0;

/* ── 系统时钟相关 ─────────────────────────────────────────── */
/** 系统时钟频率 (Hz), 用于delay_cycles校准 */
#define SYSTEM_CLK_HZ   32000000UL

/** 10µs触发脉冲所需delay_cycles数: 32MHz * 10µs = 320 */
#define TRIG_PULSE_CYCLES   320

/** 保持低电平的短延时: ~1µs = 32 cycles */
#define TRIG_IDLE_CYCLES    32

/* ── 公开API ──────────────────────────────────────────────── */

void Ultrasonic_Init(void)
{
    /* Trig默认低电平 */
    DL_GPIO_clearPins(ENCODER_PORT, ENCODER_TRIG_PIN);

    /* 使能Timer中断 */
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    /* 使能GPIO中断（Echo） */
    NVIC_ClearPendingIRQ(ENCODER_INT_IRQN);
    NVIC_EnableIRQ(ENCODER_INT_IRQN);
}

bool Ultrasonic_MeasureRaw(uint32_t *pulse_us)
{
    uint32_t start_us, end_us;

    /* ① Trig: 10µs高电平触发 */
    DL_GPIO_clearPins(ENCODER_PORT, ENCODER_TRIG_PIN);
    delay_cycles(TRIG_IDLE_CYCLES);     /* 保持低电平 ~1µs */
    DL_GPIO_setPins(ENCODER_PORT, ENCODER_TRIG_PIN);
    delay_cycles(TRIG_PULSE_CYCLES);    /* 10µs高电平 @32MHz */
    DL_GPIO_clearPins(ENCODER_PORT, ENCODER_TRIG_PIN);

    /* ② 等待Echo上升沿（模块开始回波） */
    start_us = g_timer_us;  /* BugFix: 在等待循环之前记录起始时间，
                             * 原代码在循环内使用g_echo_start_us(始终为0)
                             * 导致首次调用时超时判断基于错误的基准 */
    while (!DL_GPIO_readPins(ENCODER_PORT, ENCODER_ECHO_PIN)) {
        if ((g_timer_us - start_us) > ULTRASONIC_TIMEOUT_US) {
            return false;   /* 超时：无回波 */
        }
    }

    /* ③ 等待Echo下降沿（收到回波） */
    while (DL_GPIO_readPins(ENCODER_PORT, ENCODER_ECHO_PIN)) {
        if ((g_timer_us - start_us) > ULTRASONIC_TIMEOUT_US) {
            return false;   /* 超时：脉冲过长 */
        }
    }
    end_us = g_timer_us;

    /* ④ 计算脉宽 */
    *pulse_us = end_us - start_us;
    return true;
}

bool Ultrasonic_Measure(float *distance_cm)
{
    uint32_t pulse_us;

    if (!Ultrasonic_MeasureRaw(&pulse_us)) {
        return false;
    }

    /* 距离(cm) = 脉宽(µs) / 58 */
    *distance_cm = (float)pulse_us / ULTRASONIC_US_PER_CM;

    /* 范围过滤 */
    if (*distance_cm < ULTRASONIC_MIN_CM ||
        *distance_cm > ULTRASONIC_MAX_CM) {
        return false;
    }

    return true;
}

/* ── GPIO中断回调 ─────────────────────────────────────────── */

void Ultrasonic_EchoIRQHandler(void)
{
    /* 清除Echo中断标志 */
    DL_GPIO_clearInterruptStatus(ENCODER_PORT, ENCODER_ECHO_PIN);
}
