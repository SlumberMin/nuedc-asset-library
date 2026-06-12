/**
 * @file    usart.h
 * @brief   串口通信模块
 */
#ifndef __USART_H
#define __USART_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

void USART1_SendString(const char *str);
void USART1_SendFloat(float val, uint8_t decimals);
void USART1_Printf(const char *fmt, ...);

#endif
