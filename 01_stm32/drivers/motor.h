/**
 * @file    motor.h
 * @brief   电机驱动模块 — STM32电赛通用代码库
 * @details 支持L298N / TB6612FNG等H桥驱动芯片。
 *          速度范围 -1000 ~ +1000（对应PWM占空比0~100%）。
 *          支持正反转、制动、滑行。
 * @author  电赛通用代码库
 * @version 1.0
 * @date    2026-06
 */

#ifndef __MOTOR_H
#define __MOTOR_H

#include "platform/hal_stm32.h"

/* ========================================================================== */
/*                              类型定义                                       */
/* ========================================================================== */

/**
 * @brief 电机驱动芯片类型
 */
typedef enum {
    MOTOR_DRV_L298N = 0,   /**< L298N双H桥驱动 */
    MOTOR_DRV_TB6612,       /**< TB6612FNG驱动 */
} MotorDriver_t;

/**
 * @brief 电机转动方向
 */
typedef enum {
    MOTOR_DIR_FORWARD = 0,  /**< 正转 */
    MOTOR_DIR_BACKWARD,     /**< 反转 */
    MOTOR_DIR_BRAKE,        /**< 制动（两线同电平） */
    MOTOR_DIR_COAST,        /**< 滑行（两线低电平/使能关闭） */
} MotorDir_t;

/**
 * @brief 电机配置结构体
 * @note  根据驱动芯片类型填写不同字段：
 *        - L298N: in1_port/pin, in2_port/pin, en_pwm
 *        - TB6612: in1_port/pin, in2_port/pin, pwm_htim/pwm_channel
 */
typedef struct {
    MotorDriver_t driver;       /**< 驱动芯片类型 */

    /* 方向控制GPIO */
    GPIO_TypeDef *in1_port;     /**< IN1端口 */
    uint16_t      in1_pin;      /**< IN1引脚 */
    GPIO_TypeDef *in2_port;     /**< IN2端口 */
    uint16_t      in2_pin;      /**< IN2引脚 */

    /* PWM控制 */
    TIM_HandleTypeDef *pwm_htim;    /**< PWM定时器句柄 */
    uint32_t           pwm_channel; /**< PWM定时器通道 */

    /* 状态 */
    int16_t  speed;             /**< 当前速度 -1000~+1000 */
    bool     initialized;       /**< 是否已初始化 */
} Motor_t;

/* ========================================================================== */
/*                              接口函数                                       */
/* ========================================================================== */

/**
 * @brief 初始化电机
 * @param motor     电机结构体指针（需先填写配置字段）
 * @param driver    驱动芯片类型
 * @param in1_port  IN1 GPIO端口
 * @param in1_pin   IN1 GPIO引脚
 * @param in2_port  IN2 GPIO端口
 * @param in2_pin   IN2 GPIO引脚
 * @param htim      PWM定时器句柄
 * @param channel   PWM定时器通道
 * @return ErrorCode_t: HAL_OK_CODE=成功
 */
ErrorCode_t Motor_Init(Motor_t *motor, MotorDriver_t driver,
                       GPIO_TypeDef *in1_port, uint16_t in1_pin,
                       GPIO_TypeDef *in2_port, uint16_t in2_pin,
                       TIM_HandleTypeDef *htim, uint32_t channel);

/**
 * @brief 设置电机速度
 * @param motor  电机结构体指针
 * @param speed  目标速度 -1000 ~ +1000
 *               正值正转，负值反转，0停止
 * @return ErrorCode_t: HAL_OK_CODE=成功
 * @note   速度绝对值100以下时自动归零（可选的死区处理）
 */
ErrorCode_t Motor_SetSpeed(Motor_t *motor, int16_t speed);

/**
 * @brief 获取当前电机速度
 * @param motor  电机结构体指针
 * @return int16_t: 当前速度 -1000~+1000
 */
int16_t Motor_GetSpeed(const Motor_t *motor);

/**
 * @brief 电机制动（快速停止）
 * @param motor  电机结构体指针
 * @return ErrorCode_t: HAL_OK_CODE=成功
 * @note   L298N: IN1=HIGH, IN2=HIGH; TB6612: IN1=HIGH, IN2=HIGH
 */
ErrorCode_t Motor_Brake(Motor_t *motor);

/**
 * @brief 电机滑行（自由停止）
 * @param motor  电机结构体指针
 * @return ErrorCode_t: HAL_OK_CODE=成功
 */
ErrorCode_t Motor_Coast(Motor_t *motor);

/**
 * @brief 反初始化电机（停止输出、释放资源）
 * @param motor  电机结构体指针
 * @return ErrorCode_t
 */
ErrorCode_t Motor_DeInit(Motor_t *motor);

#endif /* __MOTOR_H */
