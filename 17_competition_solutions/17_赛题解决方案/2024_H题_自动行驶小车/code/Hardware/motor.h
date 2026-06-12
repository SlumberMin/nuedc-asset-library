/**
 * @file    motor.h
 * @brief   电机驱动模块头文件
 * @author  电赛团队
 * @date    2024
 * @note    基于TB6612FNG双H桥驱动器
 */

#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f1xx_hal.h"
#include "user_config.h"

/* ========================================================================== */
/*                              函数声明                                       */
/* ========================================================================== */

/**
 * @brief  电机模块初始化
 * @note   配置GPIO和TIM4 PWM输出
 * @retval None
 */
void Motor_Init(void);

/**
 * @brief  设置左电机速度和方向
 * @param  speed: PWM值 (0 ~ PWM_MAX)
 * @param  forward: 1=正转, 0=反转
 * @retval None
 */
void Motor_SetLeft(uint16_t speed, uint8_t forward);

/**
 * @brief  设置右电机速度和方向
 * @param  speed: PWM值 (0 ~ PWM_MAX)
 * @param  forward: 1=正转, 0=反转
 * @retval None
 */
void Motor_SetRight(uint16_t speed, uint8_t forward);

/**
 * @brief  设置左右电机速度（带符号表示方向）
 * @param  left_speed:  左轮速度 (正值=前进, 负值=后退)
 * @param  right_speed: 右轮速度 (正值=前进, 负值=后退)
 * @note   题目要求禁止后退，负值会被限制为0
 * @retval None
 */
void Motor_SetSpeed(int16_t left_speed, int16_t right_speed);

/**
 * @brief  停止所有电机
 * @retval None
 */
void Motor_Stop(void);

/**
 * @brief  制动（短路制动）
 * @retval None
 */
void Motor_Brake(void);

/**
 * @brief  使能电机驱动
 * @retval None
 */
void Motor_Enable(void);

/**
 * @brief  禁用电机驱动（STBY拉低）
 * @retval None
 */
void Motor_Disable(void);

#endif /* __MOTOR_H */
