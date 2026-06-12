/**
 * @file    timer.c
 * @brief   系统定时器模块实现
 * 
 * 使用TIM2作为1ms系统节拍定时器
 */

#include "timer.h"
#include "microcontroller_hal.h"

static volatile uint32_t g_tick_ms = 0;
static volatile uint32_t g_start_tick = 0;

/**
 * @brief  定时器初始化（1ms中断）
 */
void Timer_Init(void)
{
    g_tick_ms = 0;
    /* TIM2配置：72MHz / 72 = 1MHz, 1MHz / 1000 = 1kHz(1ms) */
    /* 具体寄存器配置由CubeMX或手动完成 */
}

/**
 * @brief  获取系统毫秒计数
 * @retval uint32_t: 毫秒数
 */
uint32_t Timer_GetTick_ms(void)
{
    return g_tick_ms;
}

/**
 * @brief  毫秒延时（非阻塞建议用Timer_Elapsed_ms）
 */
void Timer_Delay_ms(uint32_t ms)
{
    uint32_t start = g_tick_ms;
    while((g_tick_ms - start) < ms);
}

/**
 * @brief  开始计时
 */
void Timer_Start(void)
{
    g_start_tick = g_tick_ms;
}

/**
 * @brief  获取经过时间
 * @retval uint32_t: 自Timer_Start()以来的毫秒数
 */
uint32_t Timer_Elapsed_ms(void)
{
    return g_tick_ms - g_start_tick;
}

/**
 * @brief  TIM2中断回调（需要在中断服务函数中调用）
 */
void Timer_IRQ_Callback(void)
{
    g_tick_ms++;
}
