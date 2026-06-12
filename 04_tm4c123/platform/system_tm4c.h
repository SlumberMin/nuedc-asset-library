/**
 * @file system_tm4c.h
 * @brief TM4C123GH6PZT7 系统初始化模块
 *
 * 负责:
 * - 80MHz系统时钟配置 (16MHz晶振 + PLL)
 * - FPU使能
 * - SysTick定时器(1ms tick)
 * - GPIO端口使能
 * - 中断优先级分组
 */
#ifndef __SYSTEM_TM4C_H
#define __SYSTEM_TM4C_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== 系统节拍 ======================== */
#define SYSTICK_FREQ_HZ     1000    /* 1ms 系统节拍 */
#define TICK_MS             1       /* 每tick毫秒数 */

/* ======================== API ======================== */

/**
 * @brief 系统总初始化
 *
 * 依次执行:
 * 1. FPU使能
 * 2. 80MHz时钟配置
 * 3. SysTick 1ms中断
 * 4. 全局中断使能
 */
void system_init(void);

/**
 * @brief 获取系统运行时间(ms)
 */
uint32_t system_get_tick(void);

/**
 * @brief 毫秒级延时(基于SysTick)
 */
void delay_ms(uint32_t ms);

/**
 * @brief 微秒级延时(基于循环计数, 粗略)
 */
void delay_us(uint32_t us);

/**
 * @brief 获取系统时钟频率(Hz)
 */
uint32_t system_get_clk_freq(void);

#ifdef __cplusplus
}
#endif

#endif /* __SYSTEM_TM4C_H */
