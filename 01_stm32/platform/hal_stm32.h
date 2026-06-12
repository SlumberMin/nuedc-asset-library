/**
 * @file    hal_stm32.h
 * @brief   HAL统一抽象层 — STM32电赛通用代码库
 * @details 封装STM32 HAL库常用操作为统一宏，降低移植成本。
 *          包含：GPIO/PWM/ADC/UART操作宏、数学工具宏、错误码定义。
 * @author  电赛通用代码库
 * @version 1.0
 * @date    2026-06
 *
 * 使用方法:
 *   1. 在工程中包含此头文件即可使用所有宏
 *   2. 需要先在CubeMX中配置好对应外设
 *   3. PWM通道统一使用句柄 htimX, ADC使用 hadcX, UART使用 huartX
 */

#ifndef __HAL_STM32_H
#define __HAL_STM32_H

#include "stm32f1xx_hal.h"  /* 根据实际芯片系列修改，如 stm32f4xx_hal.h */
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* ========================================================================== */
/*                              错误码定义                                     */
/* ========================================================================== */

/**
 * @brief 统一错误码枚举
 * @note  所有模块函数统一返回此类型，0=成功，负值=错误
 */
typedef enum {
    HAL_OK_CODE         =  0,   /**< 操作成功 */
    HAL_ERR_PARAM       = -1,   /**< 参数错误 */
    HAL_ERR_TIMEOUT     = -2,   /**< 超时 */
    HAL_ERR_BUSY        = -3,   /**< 忙碌/资源占用 */
    HAL_ERR_NOT_INIT    = -4,   /**< 未初始化 */
    HAL_ERR_OVERFLOW    = -5,   /**< 溢出 */
    HAL_ERR_EMPTY       = -6,   /**< 缓冲区空 */
    HAL_ERR_FULL        = -7,   /**< 缓冲区满 */
    HAL_ERR_IO          = -8,   /**< IO错误 */
    HAL_ERR_NOMEM       = -9,   /**< 内存不足 */
    HAL_ERR_UNKNOWN     = -99,  /**< 未知错误 */
} ErrorCode_t;

/* ========================================================================== */
/*                            数学工具宏                                       */
/* ========================================================================== */

/** @brief 返回两值中的较小值 */
#define MIN(a, b)               ((a) < (b) ? (a) : (b))

/** @brief 返回两值中的较大值 */
#define MAX(a, b)               ((a) > (b) ? (a) : (b))

/** @brief 将val限制在[lo, hi]范围内 */
#define CLAMP(val, lo, hi)      MIN(MAX((val), (lo)), (hi))

/** @brief 求绝对值 */
#define ABS(x)                  ((x) >= 0 ? (x) : -(x))

/** @brief 交换两个变量 */
#define SWAP(a, b, type)        do { type t = (a); (a) = (b); (b) = t; } while(0)

/** @brief 符号函数，返回 -1/0/+1 */
#define SIGN(x)                 ((x) > 0 ? 1 : ((x) < 0 ? -1 : 0))

/** @brief 角度转弧度 */
#define DEG2RAD(deg)            ((deg) * 0.01745329251994f)

/** @brief 弧度转角度 */
#define RAD2DEG(rad)            ((rad) * 57.2957795130823f)

/** @brief 数组元素个数 */
#define ARRAY_SIZE(arr)         (sizeof(arr) / sizeof((arr)[0]))

/** @brief 将值从[old_min,old_max]映射到[new_min,new_max] */
#define MAP_RANGE(val, old_min, old_max, new_min, new_max) \
    ((new_min) + ((float)(val) - (old_min)) * ((new_max) - (new_min)) / ((old_max) - (old_min)))

/* ========================================================================== */
/*                          GPIO 操作宏                                       */
/* ========================================================================== */

/**
 * @brief 设置GPIO引脚为高电平
 * @param port  GPIO端口，如 GPIOA, GPIOB
 * @param pin   引脚号，如 GPIO_PIN_0
 */
#define GPIO_SET(port, pin)         HAL_GPIO_WritePin((port), (pin), GPIO_PIN_SET)

/**
 * @brief 设置GPIO引脚为低电平
 * @param port  GPIO端口
 * @param pin   引脚号
 */
#define GPIO_CLR(port, pin)         HAL_GPIO_WritePin((port), (pin), GPIO_PIN_RESET)

/**
 * @brief 翻转GPIO引脚电平
 * @param port  GPIO端口
 * @param pin   引脚号
 */
#define GPIO_TOGGLE(port, pin)      HAL_GPIO_TogglePin((port), (pin))

/**
 * @brief 读取GPIO引脚电平
 * @param port  GPIO端口
 * @param pin   引脚号
 * @return GPIO_PinState: GPIO_PIN_SET 或 GPIO_PIN_RESET
 */
#define GPIO_READ(port, pin)        HAL_GPIO_ReadPin((port), (pin))

/**
 * @brief 写入GPIO引脚指定电平
 * @param port  GPIO端口
 * @param pin   引脚号
 * @param state 电平状态 GPIO_PIN_SET / GPIO_PIN_RESET
 */
#define GPIO_WRITE(port, pin, state) HAL_GPIO_WritePin((port), (pin), (state))

/* ========================================================================== */
/*                          PWM 操作宏                                        */
/* ========================================================================== */

/**
 * @brief 启动PWM输出（单通道）
 * @param htim     定时器句柄指针，如 &htim2
 * @param channel  定时器通道，如 TIM_CHANNEL_1
 */
#define PWM_START(htim, channel)    HAL_TIM_PWM_Start((htim), (channel))

/**
 * @brief 停止PWM输出
 * @param htim     定时器句柄指针
 * @param channel  定时器通道
 */
#define PWM_STOP(htim, channel)     HAL_TIM_PWM_Stop((htim), (channel))

/**
 * @brief 设置PWM占空比（直接设置CCR值）
 * @param htim     定时器句柄指针
 * @param channel  定时器通道，如 TIM_CHANNEL_1
 * @param ccr      比较值(0 ~ ARR)
 * @note  占空比 = ccr / (ARR+1)，ARR即Period值
 *        例: ARR=999时, ccr=500 → 占空比50%
 */
#define PWM_SET(htim, channel, ccr) \
    do { \
        switch (channel) { \
            case TIM_CHANNEL_1: (htim)->Instance->CCR1 = (ccr); break; \
            case TIM_CHANNEL_2: (htim)->Instance->CCR2 = (ccr); break; \
            case TIM_CHANNEL_3: (htim)->Instance->CCR3 = (ccr); break; \
            case TIM_CHANNEL_4: (htim)->Instance->CCR4 = (ccr); break; \
        } \
    } while(0)

/**
 * @brief 设置PWM占空比（百分比形式）
 * @param htim     定时器句柄指针
 * @param channel  定时器通道
 * @param pct      占空比百分比 0.0~100.0
 */
#define PWM_SET_PCT(htim, channel, pct) \
    PWM_SET((htim), (channel), (uint32_t)((pct) * ((htim)->Init.Period + 1) / 100.0f))

/* ========================================================================== */
/*                          ADC 操作宏                                        */
/* ========================================================================== */

/**
 * @brief 启动ADC转换并读取单次值（阻塞式）
 * @param hadc   ADC句柄指针
 * @return uint32_t: ADC原始值(12位: 0~4095)
 */
#define ADC_READ(hadc) \
    ({ \
        HAL_ADC_Start((hadc)); \
        HAL_ADC_PollForConversion((hadc), 100); \
        HAL_ADC_GetValue((hadc)); \
    })

/**
 * @brief ADC原始值转电压
 * @param adc_val   ADC原始值
 * @param vref      参考电压，通常3.3V
 * @param resolution ADC分辨率，12位=4096
 * @return float: 电压值(V)
 */
#define ADC_TO_VOLTAGE(adc_val, vref, resolution) \
    ((float)(adc_val) * (vref) / (resolution))

/* ========================================================================== */
/*                          UART 操作宏                                       */
/* ========================================================================== */

/**
 * @brief 通过UART发送字符串（阻塞式）
 * @param huart  UART句柄指针
 * @param str    字符串指针
 * @param timeout 超时时间(ms)
 */
#define UART_SEND_STR(huart, str, timeout) \
    HAL_UART_Transmit((huart), (uint8_t *)(str), strlen((str)), (timeout))

/**
 * @brief 通过UART发送数据（阻塞式）
 * @param huart   UART句柄指针
 * @param data    数据指针
 * @param len     数据长度
 * @param timeout 超时时间(ms)
 */
#define UART_SEND(huart, data, len, timeout) \
    HAL_UART_Transmit((huart), (uint8_t *)(data), (len), (timeout))

/**
 * @brief 通过UART接收数据（阻塞式）
 * @param huart   UART句柄指针
 * @param buf     接收缓冲区指针
 * @param len     期望接收长度
 * @param timeout 超时时间(ms)
 */
#define UART_RECV(huart, buf, len, timeout) \
    HAL_UART_Receive((huart), (uint8_t *)(buf), (len), (timeout))

/* ========================================================================== */
/*                          延时/时间宏                                       */
/* ========================================================================== */

/** @brief 毫秒延时 */
#define DELAY_MS(ms)                HAL_Delay(ms)

/** @brief 获取系统tick(毫秒) */
#define GET_TICK()                  HAL_GetTick()

/**
 * @brief 超时检测宏
 * @param start_tick 起始tick值
 * @param timeout_ms 超时时间(ms)
 * @return bool: true=已超时, false=未超时
 */
#define IS_TIMEOUT(start_tick, timeout_ms) \
    ((HAL_GetTick() - (start_tick)) >= (timeout_ms))

/* ========================================================================== */
/*                          调试辅助宏                                        */
/* ========================================================================== */

#ifdef DEBUG
    #include <stdio.h>
    /** @brief 调试打印宏，仅在DEBUG模式下输出 */
    #define DBG_PRINTF(fmt, ...)    printf("[DBG] " fmt "\r\n", ##__VA_ARGS__)
#else
    #define DBG_PRINTF(fmt, ...)    ((void)0)
#endif

/** @brief 断言宏（仅DEBUG模式生效） */
#ifdef DEBUG
    #define ASSERT(cond) \
        do { if (!(cond)) { DBG_PRINTF("ASSERT failed: %s @ %s:%d", #cond, __FILE__, __LINE__); while(1); } } while(0)
#else
    #define ASSERT(cond) ((void)0)
#endif

#endif /* __HAL_STM32_H */
