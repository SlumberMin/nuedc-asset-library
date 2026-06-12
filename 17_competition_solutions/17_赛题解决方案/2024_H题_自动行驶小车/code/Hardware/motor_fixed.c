/**
 * @file    motor_fixed.c
 * @brief   电机驱动模块实现（修复版）
 * 
 * 修复：Motor_SetSpeed支持负值（差速转向需要）
 */

#include "motor.h"
#include "stm32f1xx_hal.h"

#define MOTOR_PWM_MAX   999

/**
 * @brief  设置左右电机速度（支持负值=反转）
 * @param  left_speed: 左电机速度(-999~+999)
 * @param  right_speed: 右电机速度(-999~+999)
 * 
 * 修复：允许负值，差速转向时内侧电机可能需要减速或反转
 */
void Motor_SetSpeed(int16_t left_speed, int16_t right_speed)
{
    /* 限幅 */
    if(left_speed > MOTOR_PWM_MAX) left_speed = MOTOR_PWM_MAX;
    if(left_speed < -MOTOR_PWM_MAX) left_speed = -MOTOR_PWM_MAX;
    if(right_speed > MOTOR_PWM_MAX) right_speed = MOTOR_PWM_MAX;
    if(right_speed < -MOTOR_PWM_MAX) right_speed = -MOTOR_PWM_MAX;
    
    /* 左电机 */
    if(left_speed >= 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);    // AIN1
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);  // AIN2
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
        left_speed = -left_speed;
    }
    
    /* 右电机 */
    if(right_speed >= 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);    // BIN1
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);  // BIN2
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
        right_speed = -right_speed;
    }
    
    /* 设置PWM */
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, left_speed);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, right_speed);
}

/**
 * @brief  电机紧急停止
 */
void Motor_Stop(void)
{
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);
}
