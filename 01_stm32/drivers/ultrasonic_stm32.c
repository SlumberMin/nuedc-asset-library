/**
 * @file    ultrasonic_stm32.c
 * @brief   SR04/US-016 超声波测距驱动实现 — STM32 HAL库版本
 *
 * 测量方法 (TIM输入捕获):
 *   1. Trig输出10µs脉冲触发模块
 *   2. 使用TIM4_CH2输入捕获检测Echo上升沿/下降沿
 *   3. 两次捕获值之差 = Echo脉宽(µs)
 *   4. 距离(cm) = 脉宽(µs) / 58
 *
 * 硬件连接:
 *   PB6 → Trig (GPIO输出)
 *   PB7 → Echo (TIM4_CH2 输入捕获)
 *
 * 定时器配置:
 *   TIM4: PSC=71 → 72MHz/72=1MHz, ARR=0xFFFF
 *   TIM4_CH2: 输入捕获，双边沿检测
 */

#include "drivers/ultrasonic_stm32.h"

/* ── 引脚定义 ─────────────────────────────────────────────── */
#define TRIG_PORT   GPIOB
#define TRIG_PIN    GPIO_PIN_6
#define ECHO_PORT   GPIOB
#define ECHO_PIN    GPIO_PIN_7

/* ── 内部变量 ─────────────────────────────────────────────── */
static TIM_HandleTypeDef *g_htim = NULL;

/* ── 系统时钟相关 ─────────────────────────────────────────── */
/** 72MHz 时钟下 10µs = 720 cycles */
#define TRIG_PULSE_CYCLES   720
/** 1µs = 72 cycles */
#define TRIG_IDLE_CYCLES    72

/* ── 内部延时函数 (基于CPU周期) ──────────────────────────── */
static inline void delay_cycles(uint32_t cycles)
{
    volatile uint32_t i;
    for (i = 0; i < cycles / 4; i++) {
        __NOP();
    }
}

/* ── 公开API ──────────────────────────────────────────────── */

void Ultrasonic_Init(TIM_HandleTypeDef *htim)
{
    g_htim = htim;

    /* 配置Trig引脚为推挽输出 */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin   = TRIG_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(TRIG_PORT, &GPIO_InitStruct);

    /* Trig默认低电平 */
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);

    /* 配置Echo引脚为浮空输入 (TIM4_CH2) */
    GPIO_InitStruct.Pin   = ECHO_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(ECHO_PORT, &GPIO_InitStruct);
}

bool Ultrasonic_MeasureRaw(uint32_t *pulse_us)
{
    uint32_t start_cnt, end_cnt, width;
    uint32_t timeout;

    if (g_htim == NULL) return false;

    /* ① Trig: 10µs高电平触发 */
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
    delay_cycles(TRIG_IDLE_CYCLES);     /* 保持低电平 ~1µs */
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
    delay_cycles(TRIG_PULSE_CYCLES);    /* 10µs高电平 */
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);

    /* ② 等待Echo上升沿（模块开始回波） */
    timeout = ULTRASONIC_TIMEOUT_US * 100; /* 粗略超时计数 */
    while (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_RESET) {
        if (--timeout == 0) return false;
    }
    start_cnt = __HAL_TIM_GET_COUNTER(g_htim);

    /* ③ 等待Echo下降沿（收到回波） */
    timeout = ULTRASONIC_TIMEOUT_US * 100;
    while (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_SET) {
        if (--timeout == 0) return false;
    }
    end_cnt = __HAL_TIM_GET_COUNTER(g_htim);

    /* ④ 计算脉宽 (处理定时器溢出) */
    if (end_cnt >= start_cnt) {
        width = end_cnt - start_cnt;
    } else {
        width = (0xFFFF - start_cnt) + end_cnt + 1;
    }

    *pulse_us = width; /* TIM时钟1MHz，1 tick = 1µs */
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
