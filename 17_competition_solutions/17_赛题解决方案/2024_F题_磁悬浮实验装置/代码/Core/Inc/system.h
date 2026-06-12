/**
 * @file    system.h
 * @brief   系统时钟配置
 */
#ifndef __SYSTEM_H
#define __SYSTEM_H

#include "stm32f1xx_hal.h"

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_USART1_Init(void);
void MX_ADC1_Init(void);
void MX_TIM2_Init(void);
void MX_TIM3_Init(void);

extern UART_HandleTypeDef huart1;

#endif
