/**
 * @file    tb6612_stm32.h
 * @brief   TB6612FNG 电机驱动模块 — STM32 HAL库版本
 * @details 驱动 TB6612FNG 双H桥电机驱动芯片。
 *          速度范围 -1000 ~ +1000（对应PWM占空比0~100%）。
 *          支持正反转、制动、滑行。
 * @author  nuedc-asset-library
 * @version 1.0
 * @date    2026-06
 */

#ifndef __TB6612_STM32_H
#define __TB6612_STM32_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ========================================================================== */
/*                              类型定义                                       */
/* ========================================================================== */

/** @brief 电机方向 */
typedef enum {
    TB6612_DIR_FORWARD  = 0,   /**< 正转 */
    TB6612_DIR_BACKWARD = 1,   /**< 反转 */
    TB6612_DIR_BRAKE    = 2,   /**< 制动 */
    TB6612_DIR_COAST    = 3,   /**< 滑行 */
} TB6612_Dir_t;

/** @brief TB6612 电机通道配置 */
typedef struct {
    GPIO_TypeDef *in1_port;        /**< AIN1/BIN1 端口 */
    uint16_t      in1_pin;         /**< AIN1/BIN1 引脚 */
    GPIO_TypeDef *in2_port;        /**< AIN2/BIN2 端口 */
    uint16_t      in2_pin;         /**< AIN2/BIN2 引脚 */
    TIM_HandleTypeDef *pwm_htim;   /**< PWM定时器句柄指针 */
    uint32_t           pwm_channel;/**< PWM通道 TIM_CHANNEL_x */
    int16_t  speed;                /**< 当前速度 -1000~+1000 */
    bool     initialized;          /**< 是否已初始化 */
} TB6612_Motor_t;

/** @brief TB6612 整体配置（双通道） */
typedef struct {
    TB6612_Motor_t motor_a;        /**< 电机A */
    TB6612_Motor_t motor_b;        /**< 电机B */
    GPIO_TypeDef *stby_port;       /**< STBY引脚端口（可选，NULL则不控制） */
    uint16_t      stby_pin;        /**< STBY引脚 */
    bool     initialized;
} TB6612_Dev_t;

/* ========================================================================== */
/*                              接口函数                                       */
/* ========================================================================== */

/**
 * @brief 初始化 TB6612 设备（含双通道电机）
 * @param dev        TB6612设备结构体指针
 * @param stby_port  STBY端口（使能端），可为NULL
 * @param stby_pin   STBY引脚
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef TB6612_Init(TB6612_Dev_t *dev,
                              GPIO_TypeDef *stby_port, uint16_t stby_pin);

/**
 * @brief 初始化单个电机通道
 * @param motor       电机结构体指针
 * @param in1_port    IN1端口
 * @param in1_pin     IN1引脚
 * @param in2_port    IN2端口
 * @param in2_pin     IN2引脚
 * @param htim        PWM定时器句柄
 * @param channel     PWM通道
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef TB6612_MotorInit(TB6612_Motor_t *motor,
                                   GPIO_TypeDef *in1_port, uint16_t in1_pin,
                                   GPIO_TypeDef *in2_port, uint16_t in2_pin,
                                   TIM_HandleTypeDef *htim, uint32_t channel);

/**
 * @brief 设置电机速度
 * @param motor  电机结构体指针
 * @param speed  速度 -1000 ~ +1000
 */
void TB6612_SetSpeed(TB6612_Motor_t *motor, int16_t speed);

/**
 * @brief 获取电机当前速度
 * @param motor  电机结构体指针
 * @return 当前速度值
 */
int16_t TB6612_GetSpeed(const TB6612_Motor_t *motor);

/**
 * @brief 电机制动（快速停止）
 * @param motor  电机结构体指针
 */
void TB6612_Brake(TB6612_Motor_t *motor);

/**
 * @brief 电机滑行（自由停止）
 * @param motor  电机结构体指针
 */
void TB6612_Coast(TB6612_Motor_t *motor);

/**
 * @brief 使能/禁用 TB6612（STBY引脚）
 * @param dev    TB6612设备结构体指针
 * @param enable true=使能, false=禁用
 */
void TB6612_Enable(TB6612_Dev_t *dev, bool enable);

#endif /* __TB6612_STM32_H */
