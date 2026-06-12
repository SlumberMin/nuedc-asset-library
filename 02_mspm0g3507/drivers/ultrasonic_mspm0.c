/**
 * @file    ultrasonic_mspm0.c
 * @brief   SR04/US-016 超声波传感器驱动实现 — MSPM0G3507
 * @note    使用GPIO触发和Echo中断检测，通过定时器计时
 *          基于天猛星MSPM0G3507模块移植代码优化
 */

#include "ultrasonic_mspm0.h"
#include <string.h>

/* ── 私有变量 ────────────────────────────────────────────── */
static UltrasonicConfig g_ultra_cfg;
static UltrasonicData   g_ultra_data;
static volatile uint32_t g_echo_start = 0;   /* Echo上升沿时间 */
static volatile uint32_t g_echo_end = 0;     /* Echo下降沿时间 */
static volatile uint8_t  g_echo_flag = 0;    /* Echo完成标志 */
static volatile uint32_t g_timer_count = 0;  /* 定时器计数 */

/* ── 滤波器 (简单移动平均) ─────────────────────────────────── */
#ifndef ULTRASONIC_FILTER_SIZE
#define ULTRASONIC_FILTER_SIZE  5
#endif
static float  g_filter_buf[ULTRASONIC_FILTER_SIZE];
static uint8_t g_filter_idx = 0;
static uint8_t g_filter_count = 0;

/* ── 内部函数 ────────────────────────────────────────────── */

/**
 * @brief 计算SysTick递减计数器经过的tick数 (安全处理回绕)
 * @param prev  之前的计数值
 * @param now   当前计数值
 * @return 经过的tick数
 */
static uint32_t Ultrasonic_GetElapsed(uint32_t prev, uint32_t now)
{
    if (prev >= now) return prev - now;
    return (prev + SysTick->LOAD + 1) - now;
}

/**
 * @brief 微秒延时 (安全处理SysTick回绕)
 * @param us  微秒数
 */
static void Ultrasonic_DelayUs(uint32_t us)
{
    uint32_t ticks = us * (MSPM0_SYS_CLK_HZ / 1000000);
    uint32_t prev = SysTick->VAL;
    uint32_t elapsed = 0;
    while (elapsed < ticks) {
        uint32_t now = SysTick->VAL;
        elapsed += Ultrasonic_GetElapsed(prev, now);
        prev = now;
    }
}

/**
 * @brief 发送触发脉冲
 */
static void Ultrasonic_Trigger(void)
{
    /* 拉低Trig引脚 */
    GPIO_CLR(g_ultra_cfg.port, g_ultra_cfg.trig_pin);
    Ultrasonic_DelayUs(2);
    
    /* 拉高Trig引脚 (>10us) */
    GPIO_SET(g_ultra_cfg.port, g_ultra_cfg.trig_pin);
    Ultrasonic_DelayUs(15);
    
    /* 拉低Trig引脚 */
    GPIO_CLR(g_ultra_cfg.port, g_ultra_cfg.trig_pin);
}

/**
 * @brief 等待Echo引脚状态变化 (安全处理SysTick回绕)
 * @param state  期望状态 (0=低电平, 1=高电平)
 * @param timeout_us  超时时间 (微秒)
 * @return 1=成功, 0=超时
 */
static uint8_t Ultrasonic_WaitEcho(uint8_t state, uint32_t timeout_us)
{
    uint32_t ticks = timeout_us * (MSPM0_SYS_CLK_HZ / 1000000);
    uint32_t prev = SysTick->VAL;
    uint32_t elapsed = 0;
    
    while (elapsed < ticks) {
        uint8_t current = GPIO_READ(g_ultra_cfg.port, g_ultra_cfg.echo_pin) ? 1 : 0;
        if (current == state) {
            return 1;
        }
        uint32_t now = SysTick->VAL;
        elapsed += Ultrasonic_GetElapsed(prev, now);
        prev = now;
    }
    return 0;  /* 超时 */
}

/* ── 公开API ─────────────────────────────────────────────── */

void Ultrasonic_Init(const UltrasonicConfig *cfg)
{
    /* 保存配置 */
    g_ultra_cfg = *cfg;
    
    /* 清空数据 */
    memset(&g_ultra_data, 0, sizeof(UltrasonicData));
    
    /* 清空滤波器 */
    memset(g_filter_buf, 0, sizeof(g_filter_buf));
    g_filter_idx = 0;
    g_filter_count = 0;
    
    /* 初始化默认滤波窗口 */
    if (g_ultra_cfg.filter_size == 0) {
        g_ultra_cfg.filter_size = 5;
    }
    
    /* 配置GPIO */
    /* Trig引脚设为输出 */
    /* Echo引脚设为输入 */
    /* 注意：实际GPIO配置需在SysConfig中完成 */
}

float Ultrasonic_Measure(void)
{
    uint32_t echo_time;
    float distance;
    
    /* 发送触发脉冲 */
    Ultrasonic_Trigger();
    
    /* 等待Echo高电平 (超时10ms) */
    if (!Ultrasonic_WaitEcho(1, 10000)) {
        return 0.0f;  /* 超时 */
    }
    
    /* 记录开始时间 */
    uint32_t start_time = SysTick->VAL;
    
    /* 等待Echo低电平 (超时30ms，对应约5m) */
    if (!Ultrasonic_WaitEcho(0, 30000)) {
        return 0.0f;  /* 超时 */
    }
    
    /* 记录结束时间 */
    uint32_t end_time = SysTick->VAL;
    
    /* 计算Echo高电平时间 (微秒) — 使用安全的回绕处理 */
    echo_time = Ultrasonic_GetElapsed(start_time, end_time) / (MSPM0_SYS_CLK_HZ / 1000000);
    
    /* 计算距离 (cm) */
    /* 声速：340m/s = 0.034cm/us */
    /* 距离 = 时间 * 声速 / 2 */
    distance = (float)echo_time * 0.034f / 2.0f;
    
    /* 限幅 (2cm ~ 400cm) */
    if (distance < 2.0f) distance = 2.0f;
    if (distance > 400.0f) distance = 400.0f;
    
    /* 更新数据 */
    g_ultra_data.distance = distance;
    g_ultra_data.valid = 1;
    g_ultra_data.measure_count++;
    
    /* 更新滤波器 */
    g_filter_buf[g_filter_idx] = distance;
    g_filter_idx = (g_filter_idx + 1) % ULTRASONIC_FILTER_SIZE;
    if (g_filter_count < ULTRASONIC_FILTER_SIZE) g_filter_count++;
    
    /* 计算滤波结果 (移动平均) */
    float sum = 0;
    for (uint8_t i = 0; i < g_filter_count; i++) {
        sum += g_filter_buf[i];
    }
    g_ultra_data.filtered = sum / g_filter_count;
    
    return distance;
}

float Ultrasonic_GetDistance(void)
{
    return g_ultra_data.distance;
}

float Ultrasonic_GetFilteredDistance(void)
{
    return g_ultra_data.filtered;
}

uint8_t Ultrasonic_IsValid(void)
{
    return g_ultra_data.valid;
}

UltrasonicData* Ultrasonic_GetData(void)
{
    return &g_ultra_data;
}