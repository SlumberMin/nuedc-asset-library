/**
 * @file    usart.c
 * @brief   串口通信模块实现
 */

#include "usart.h"
#include "system.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

void USART1_SendString(const char *str)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 100);
}

void USART1_SendFloat(float val, uint8_t decimals)
{
    char buf[32];
    if(decimals == 1) sprintf(buf, "%.1f", val);
    else if(decimals == 2) sprintf(buf, "%.2f", val);
    else sprintf(buf, "%.0f", val);
    USART1_SendString(buf);
}

void USART1_Printf(const char *fmt, ...)
{
    char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    USART1_SendString(buf);
}
