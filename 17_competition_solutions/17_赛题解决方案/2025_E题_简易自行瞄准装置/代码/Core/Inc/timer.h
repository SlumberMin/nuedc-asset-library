/**
 * @file    timer.h
 * @brief   系统定时器模块头文件
 */
#ifndef __TIMER_H
#define __TIMER_H

#include <stdint.h>

void Timer_Init(void);
uint32_t Timer_GetTick_ms(void);
void Timer_Delay_ms(uint32_t ms);
void Timer_Start(void);
uint32_t Timer_Elapsed_ms(void);

#endif /* __TIMER_H */
