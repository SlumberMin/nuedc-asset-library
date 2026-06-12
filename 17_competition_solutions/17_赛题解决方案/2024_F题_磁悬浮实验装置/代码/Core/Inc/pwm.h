/**
 * @file    pwm.h
 * @brief   PWM输出模块头文件
 * @version 1.0
 */

#ifndef __PWM_H
#define __PWM_H

#include "stm32f1xx_hal.h"

/* PWM配置参数 */
#define PWM_TIM                 TIM3
#define PWM_FREQUENCY           20000   // 20kHz
#define PWM_PRESCALER           0       // 预分频器
#define PWM_PERIOD              3599    // 自动重装载值 (72MHz / (0+1) / (3599+1) = 20kHz)
#define PWM_MAX_DUTY            4095    // 最大占空比(12位分辨率)

/* PWM通道定义 */
#define PWM_CHANNEL_COIL_AB     TIM_CHANNEL_1   // PA6 - 电磁铁1&2
#define PWM_CHANNEL_COIL_CD     TIM_CHANNEL_2   // PA7 - 电磁铁3&4

/* 外部变量声明 */
extern TIM_HandleTypeDef htim3;

/* 函数声明 */
void MX_TIM3_Init(void);
void PWM_SetDuty(uint16_t duty);
void PWM_SetDutyChannel(uint32_t channel, uint16_t duty);
void PWM_Start(void);
void PWM_Stop(void);

#endif /* __PWM_H */
