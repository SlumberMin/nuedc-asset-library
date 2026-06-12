/**
 * @file    gimbal.c
 * @brief   二维舵机云台模块实现
 * 
 * 硬件连接：
 * MSPM0 TIM1_CH1(PB6) → 水平舵机SG90（左右）
 * MSPM0 TIM1_CH2(PB7) → 垂直舵机SG90（上下）
 * MSPM0 PA5 → MOSFET → 激光笔
 * 
 * 舵机控制：
 * PWM频率50Hz，周期20ms
 * 脉宽0.5ms(0°) ~ 2.5ms(180°)
 * 对应CCR值：250(0°) ~ 1250(180°)（假设预分频后计数频率50kHz）
 */

#include "gimbal.h"
#include "msp.h"

/* 舵机PWM参数（50kHz计数频率） */
#define SERVO_MIN_PULSE    250     // 0.5ms → 0°
#define SERVO_MAX_PULSE    1250    // 2.5ms → 180°
#define SERVO_CENTER_PULSE 750     // 1.5ms → 90°

/* 激光笔控制引脚 */
#define LASER_PORT  GPIOA
#define LASER_PIN   BIT5

/**
 * @brief  云台初始化，舵机居中
 * @param  无
 * @retval 无
 */
void Gimbal_Init(void)
{
    /* TIM1 PWM初始化（50Hz） */
    /* 配置PB6为TIM1_CH1输出，PB7为TIM1_CH2输出 */
    
    /* 舵机初始居中 */
    Gimbal_Center();
    
    /* 激光笔初始关闭 */
    Laser_SetState(0);
}

/**
 * @brief  设置云台角度
 * @param  angle_h: 水平角度(0~180°)
 * @param  angle_v: 垂直角度(0~180°)
 * @retval 无
 */
void Gimbal_SetAngle(float angle_h, float angle_v)
{
    uint16_t pulse_h, pulse_v;
    
    /* 角度限幅 */
    if(angle_h < 0.0f) angle_h = 0.0f;
    if(angle_h > 180.0f) angle_h = 180.0f;
    if(angle_v < 0.0f) angle_v = 0.0f;
    if(angle_v > 180.0f) angle_v = 180.0f;
    
    /* 角度映射到脉宽 */
    pulse_h = SERVO_MIN_PULSE + (uint16_t)((angle_h / 180.0f) * (SERVO_MAX_PULSE - SERVO_MIN_PULSE));
    pulse_v = SERVO_MIN_PULSE + (uint16_t)((angle_v / 180.0f) * (SERVO_MAX_PULSE - SERVO_MIN_PULSE));
    
    /* 设置PWM占空比 */
    TIM1->CCR[0] = pulse_h;    // 通道1：水平
    TIM1->CCR[1] = pulse_v;    // 通道2：垂直
}

/**
 * @brief  云台归中
 * @param  无
 * @retval 无
 */
void Gimbal_Center(void)
{
    Gimbal_SetAngle(90.0f, 90.0f);
}

/**
 * @brief  设置激光笔状态
 * @param  state: 0=关闭，非0=开启
 * @retval 无
 */
void Laser_SetState(uint8_t state)
{
    if(state)
        LASER_PORT->OUT |= LASER_PIN;  // 开启激光
    else
        LASER_PORT->OUT &= ~LASER_PIN; // 关闭激光
}
