/**
 * @file    motor.c
 * @brief   电机驱动模块实现
 * 
 * 硬件连接：
 * MSPM0 TIM3_CH1(PA6) → TB6612FNG PWMA → 左电机
 * MSPM0 TIM3_CH2(PA7) → TB6612FNG PWMB → 右电机
 * MSPM0 PB0/PB1 → TB6612FNG AIN1/AIN2 → 左电机方向
 * MSPM0 PB2/PB3 → TB6612FNG BIN1/BIN2 → 右电机方向
 * MSPM0 PB4 → TB6612FNG STBY → 使能
 */

#include "motor.h"
#include "msp.h"

#define MOTOR_PWM_MAX   999
#define MOTOR_PWM_MIN   0

/**
 * @brief  电机驱动初始化
 * @param  无
 * @retval 无
 */
void Motor_Init(void)
{
    /* TIM3 PWM输出初始化（20kHz） */
    /* GPIO配置：PB0-PB4为输出（方向控制+使能） */
    
    /* 使能TB6612FNG */
    GPIOB->OUT |= BIT4;    // STBY = HIGH
    
    /* 初始停止 */
    Motor_Stop();
}

/**
 * @brief  设置左右电机速度
 * @param  left_speed: 左电机速度(-999~+999)，正=前进，负=后退
 * @param  right_speed: 右电机速度(-999~+999)
 * @retval 无
 */
void Motor_SetSpeed(int16_t left_speed, int16_t right_speed)
{
    /* 限幅 */
    if(left_speed > MOTOR_PWM_MAX) left_speed = MOTOR_PWM_MAX;
    if(left_speed < -MOTOR_PWM_MAX) left_speed = -MOTOR_PWM_MAX;
    if(right_speed > MOTOR_PWM_MAX) right_speed = MOTOR_PWM_MAX;
    if(right_speed < -MOTOR_PWM_MAX) right_speed = -MOTOR_PWM_MAX;
    
    /* 左电机方向控制 */
    if(left_speed >= 0)
    {
        GPIOB->OUT |= BIT0;    // AIN1 = 1
        GPIOB->OUT &= ~BIT1;   // AIN2 = 0 (前进)
    }
    else
    {
        GPIOB->OUT &= ~BIT0;   // AIN1 = 0
        GPIOB->OUT |= BIT1;    // AIN2 = 1 (后退)
        left_speed = -left_speed;
    }
    
    /* 右电机方向控制 */
    if(right_speed >= 0)
    {
        GPIOB->OUT |= BIT2;    // BIN1 = 1
        GPIOB->OUT &= ~BIT3;   // BIN2 = 0 (前进)
    }
    else
    {
        GPIOB->OUT &= ~BIT2;   // BIN1 = 0
        GPIOB->OUT |= BIT3;    // BIN2 = 1 (后退)
        right_speed = -right_speed;
    }
    
    /* 设置PWM占空比 */
    TIM3->CCR[0] = (uint32_t)left_speed;   // 通道1：左电机
    TIM3->CCR[1] = (uint32_t)right_speed;  // 通道2：右电机
}

/**
 * @brief  电机紧急停止
 * @param  无
 * @retval 无
 */
void Motor_Stop(void)
{
    TIM3->CCR[0] = 0;
    TIM3->CCR[1] = 0;
    GPIOB->OUT &= ~(BIT0|BIT1|BIT2|BIT3);  // 所有方向引脚拉低
}
