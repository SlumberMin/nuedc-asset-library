/**
 * @file    servo.h
 * @brief   舵机驱动模块 — STM32电赛通用代码库
 * @details 支持SG90(0~180°)和MG996R(0~180°)舵机。
 *          角度→PWM映射，支持多通道舵机控制。
 *          PWM参数：50Hz(周期20ms)，脉宽0.5ms~2.5ms对应0°~180°。
 * @author  电赛通用代码库
 * @version 1.0
 * @date    2026-06
 */

#ifndef __SERVO_H
#define __SERVO_H

#include "platform/hal_stm32.h"

/* ========================================================================== */
/*                              类型定义                                       */
/* ========================================================================== */

/** @brief 最大支持舵机数量 */
#define SERVO_MAX_CHANNELS  8

/**
 * @brief 舵机类型
 */
typedef enum {
    SERVO_TYPE_SG90 = 0,    /**< SG90微型舵机(0~180°) */
    SERVO_TYPE_MG996R,      /**< MG996R金属齿轮舵机(0~180°) */
    SERVO_TYPE_CUSTOM,      /**< 自定义参数舵机 */
} ServoType_t;

/**
 * @brief 单个舵机实例
 */
typedef struct {
    TIM_HandleTypeDef *htim;        /**< PWM定时器句柄 */
    uint32_t           pwm_channel; /**< PWM定时器通道 */

    ServoType_t type;               /**< 舵机类型 */
    float       min_angle;          /**< 最小角度(°) */
    float       max_angle;          /**< 最大角度(°) */
    float       min_pulse_us;       /**< 最小角度对应的脉宽(μs) */
    float       max_pulse_us;       /**< 最大角度对应的脉宽(μs) */

    float       current_angle;      /**< 当前角度(°) */
    uint32_t    pwm_period_us;      /**< PWM周期(μs)，50Hz=20000μs */
    bool        initialized;        /**< 是否已初始化 */
} Servo_t;

/**
 * @brief 舵机管理器（可选，用于统一管理多个舵机）
 */
typedef struct {
    Servo_t  servos[SERVO_MAX_CHANNELS];
    uint8_t  count;
} ServoMgr_t;

/* ========================================================================== */
/*                              接口函数                                       */
/* ========================================================================== */

/**
 * @brief 初始化单个舵机
 * @param servo     舵机结构体指针
 * @param htim      PWM定时器句柄
 * @param channel   PWM定时器通道
 * @param type      舵机类型(SG90/MG996R/自定义)
 * @return ErrorCode_t: HAL_OK_CODE=成功
 * @note   定时器需配置为50Hz PWM输出（ARR=19999, 预分频根据时钟计算）
 *         例如72MHz时钟: PSC=71, ARR=19999 → 72MHz/(72*20000)=50Hz
 */
ErrorCode_t Servo_Init(Servo_t *servo, TIM_HandleTypeDef *htim,
                       uint32_t channel, ServoType_t type);

/**
 * @brief 初始化自定义参数舵机
 * @param servo         舵机结构体指针
 * @param htim          PWM定时器句柄
 * @param channel       PWM定时器通道
 * @param min_angle     最小角度(°)
 * @param max_angle     最大角度(°)
 * @param min_pulse_us  最小角度脉宽(μs)
 * @param max_pulse_us  最大角度脉宽(μs)
 * @return ErrorCode_t
 */
ErrorCode_t Servo_InitCustom(Servo_t *servo, TIM_HandleTypeDef *htim,
                             uint32_t channel, float min_angle, float max_angle,
                             float min_pulse_us, float max_pulse_us);

/**
 * @brief 设置舵机角度
 * @param servo   舵机结构体指针
 * @param angle   目标角度(°)
 * @return ErrorCode_t: HAL_OK_CODE=成功
 * @note   角度会自动限幅到有效范围
 */
ErrorCode_t Servo_SetAngle(Servo_t *servo, float angle);

/**
 * @brief 获取舵机当前角度
 * @param servo  舵机结构体指针
 * @return float: 当前角度(°)
 */
float Servo_GetAngle(const Servo_t *servo);

/**
 * @brief 设置舵机脉宽（直接控制，高级用法）
 * @param servo      舵机结构体指针
 * @param pulse_us   脉宽(μs)
 * @return ErrorCode_t
 */
ErrorCode_t Servo_SetPulse(Servo_t *servo, float pulse_us);

/**
 * @brief 舵机归中（回到90°位置）
 * @param servo  舵机结构体指针
 * @return ErrorCode_t
 */
ErrorCode_t Servo_Center(Servo_t *servo);

#endif /* __SERVO_H */
