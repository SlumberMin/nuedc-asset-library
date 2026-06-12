/**
 * @file    pwm.c
 * @brief   PWM输出模块 - 电磁铁驱动
 * @version 1.0
 * 
 * 功能：
 * - 20kHz PWM输出（人耳不可闻）
 * - 12位分辨率（0~4095）
 * - 双通道输出（电磁铁组1&2、组3&4）
 */

#include "pwm.h"

/* TIM3句柄 */
TIM_HandleTypeDef htim3;

/**
 * @brief  TIM3初始化函数
 * @note   配置PWM输出，频率20kHz
 */
void MX_TIM3_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};
    
    /* TIM3基本配置 */
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = PWM_PRESCALER;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = PWM_PERIOD;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim3);
    
    /* 通道1配置 - 电磁铁1&2 */
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;                    // 初始占空比0
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, PWM_CHANNEL_COIL_AB);
    
    /* 通道2配置 - 电磁铁3&4 */
    sConfigOC.Pulse = 0;
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, PWM_CHANNEL_COIL_CD);
}

/**
 * @brief  设置PWM占空比（所有通道）
 * @param  duty: 占空比值(0~4095)
 * @retval 无
 * 
 * duty = 0: 占空比0%（电磁铁断电）
 * duty = 4095: 占空比100%（电磁铁满功率）
 */
void PWM_SetDuty(uint16_t duty)
{
    /* 限制范围 */
    if (duty > PWM_MAX_DUTY)
    {
        duty = PWM_MAX_DUTY;
    }
    
    /* 设置两个通道的占空比 */
    __HAL_TIM_SET_COMPARE(&htim3, PWM_CHANNEL_COIL_AB, duty);
    __HAL_TIM_SET_COMPARE(&htim3, PWM_CHANNEL_COIL_CD, duty);
}

/**
 * @brief  设置指定通道的PWM占空比
 * @param  channel: PWM通道（TIM_CHANNEL_1 或 TIM_CHANNEL_2）
 * @param  duty: 占空比值(0~4095)
 * @retval 无
 */
void PWM_SetDutyChannel(uint32_t channel, uint16_t duty)
{
    if (duty > PWM_MAX_DUTY)
    {
        duty = PWM_MAX_DUTY;
    }
    
    __HAL_TIM_SET_COMPARE(&htim3, channel, duty);
}

/**
 * @brief  启动PWM输出
 * @retval 无
 */
void PWM_Start(void)
{
    HAL_TIM_PWM_Start(&htim3, PWM_CHANNEL_COIL_AB);
    HAL_TIM_PWM_Start(&htim3, PWM_CHANNEL_COIL_CD);
}

/**
 * @brief  停止PWM输出
 * @retval 无
 */
void PWM_Stop(void)
{
    HAL_TIM_PWM_Stop(&htim3, PWM_CHANNEL_COIL_AB);
    HAL_TIM_PWM_Stop(&htim3, PWM_CHANNEL_COIL_CD);
}
