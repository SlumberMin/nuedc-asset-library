/**
 * @file    microcontroller_hal.h
 * @brief   通用MCU硬件抽象层 - 统一STM32/MSPM0接口
 * @version 2.0
 * 
 * 本文件提供统一的硬件抽象接口，使代码可以在STM32和MSPM0之间无缝切换。
 * 通过宏定义选择目标平台，所有底层操作通过统一接口调用。
 */

#ifndef __MICROCONTROLLER_HAL_H
#define __MICROCONTROLLER_HAL_H

#include <stdint.h>
#include <stdbool.h>

/* ============================================================
 * 平台选择宏（编译时通过-D选项定义）
 * #define PLATFORM_STM32    // 使用STM32
 * #define PLATFORM_MSPM0    // 使用TI MSPM0
 * ============================================================ */

/* 默认选择STM32 */
#if !defined(PLATFORM_STM32) && !defined(PLATFORM_MSPM0)
#define PLATFORM_STM32
#endif

/* ============================================================
 * GPIO操作抽象
 * ============================================================ */
#ifdef PLATFORM_STM32
#include "stm32f1xx_hal.h"
typedef GPIO_TypeDef* GPIO_Port_t;
typedef uint16_t GPIO_Pin_t;
#define GPIO_HIGH(port, pin)    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET)
#define GPIO_LOW(port, pin)     HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET)
#define GPIO_READ(port, pin)    HAL_GPIO_ReadPin(port, pin)
#define GPIO_TOGGLE(port, pin)  HAL_GPIO_TogglePin(port, pin)
#endif

#ifdef PLATFORM_MSPM0
#include "ti_msp_dl_config.h"
typedef GPIO_Regs* GPIO_Port_t;
typedef uint32_t GPIO_Pin_t;
#define GPIO_HIGH(port, pin)    DL_GPIO_setPins(port, pin)
#define GPIO_LOW(port, pin)     DL_GPIO_clearPins(port, pin)
#define GPIO_READ(port, pin)    DL_GPIO_readPins(port, pin)
#define GPIO_TOGGLE(port, pin)  DL_GPIO_togglePins(port, pin)
#endif

/* ============================================================
 * 定时器操作抽象
 * ============================================================ */
#ifdef PLATFORM_STM32
#define TIM_SET_COMPARE(htim, ch, val)  __HAL_TIM_SET_COMPARE(htim, ch, val)
#define TIM_GET_COUNTER(htim)            __HAL_TIM_GET_COUNTER(htim)
#endif

#ifdef PLATFORM_MSPM0
#define TIM_SET_COMPARE(tim, ch, val)   DL_Timer_setCaptureCompareValue(tim, val, ch)
#define TIM_GET_COUNTER(tim)             DL_Timer_getTimerCount(tim)
#endif

/* ============================================================
 * UART操作抽象
 * ============================================================ */
#ifdef PLATFORM_STM32
#define UART_SEND(huart, data, len, timeout)  HAL_UART_Transmit(huart, data, len, timeout)
#define UART_RECV(huart, buf, len, timeout)   HAL_UART_Receive(huart, buf, len, timeout)
#endif

#ifdef PLATFORM_MSPM0
#define UART_SEND(uart, data, len, timeout)   DL_UART_main_fillTXFIFO(uart, data, len)
#define UART_RECV(uart, buf, len, timeout)    // MSPM0 uses interrupt-based receive
#endif

/* ============================================================
 * ADC操作抽象
 * ============================================================ */
#ifdef PLATFORM_STM32
#define ADC_READ(hadc, channel)  HAL_ADC_GetValue(hadc)
#endif

#ifdef PLATFORM_MSPM0
#define ADC_READ(adc, channel)   DL_ADC12_getMemResult(adc, channel)
#endif

/* ============================================================
 * 系统控制抽象
 * ============================================================ */
#ifdef PLATFORM_STM32
#define SYSTEM_DELAY_MS(ms)      HAL_Delay(ms)
#define SYSTEM_GET_TICK()        HAL_GetTick()
#define SYSTEM_DISABLE_IRQ()     __disable_irq()
#define SYSTEM_ENABLE_IRQ()      __enable_irq()
#endif

#ifdef PLATFORM_MSPM0
#define SYSTEM_DELAY_MS(ms)      delay_cycles(ms * 80000)  // 80MHz
#define SYSTEM_GET_TICK()        DL_Timer_getTimerCount(TIMG0)
#define SYSTEM_DISABLE_IRQ()     __disable_irq()
#define SYSTEM_ENABLE_IRQ()      __enable_irq()
#endif

/* ============================================================
 * 数学工具宏
 * ============================================================ */
#define CONSTRAIN(val, min, max) \
    do { \
        if((val) < (min)) (val) = (min); \
        if((val) > (max)) (val) = (max); \
    } while(0)

#define ABS(x)     ((x) >= 0 ? (x) : -(x))
#define SIGN(x)    ((x) >= 0 ? 1.0f : -1.0f)
#define MAP(x, in_min, in_max, out_min, out_max) \
    ((x) - (in_min)) * ((out_max) - (out_min)) / ((in_max) - (in_min)) + (out_min)

#endif /* __MICROCONTROLLER_HAL_H */
