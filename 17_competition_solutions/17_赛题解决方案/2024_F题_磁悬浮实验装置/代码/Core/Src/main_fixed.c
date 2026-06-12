/**
 * @file    main_fixed.c
 * @brief   2024年F题 磁悬浮实验装置 - 主程序（修复版）
 * @version 2.1
 * 
 * 修复内容：
 * 1. 删除StateMachine_Run中的PID计算，PID只在中断中执行
 * 2. 集成PID_V2改进版算法
 * 3. 集成ADRC自抗扰控制（可选）
 * 4. 修复PWM_MAX_DUTY与PWM_PERIOD不一致
 */

#include "stm32f1xx_hal.h"
#include "pid_v2.h"
#include "adrc.h"
#include "key.h"
#include "oled.h"
#include "system.h"
#include <stdio.h>
#include <math.h>

/* 修复：PWM_MAX_DUTY应等于PWM_PERIOD */
#define PWM_PERIOD      3599
#define PWM_MAX_DUTY    PWM_PERIOD  // 修复：之前是4095，超过PWM_PERIOD

/* 系统状态 */
typedef enum {
    STATE_IDLE = 0,
    STATE_SEARCHING,
    STATE_LIFTING,
    STATE_SUSPENDING,
    STATE_ADJUSTING,
    STATE_ERROR
} SystemState_t;

/* 全局变量 */
volatile SystemState_t g_state = STATE_IDLE;
volatile float g_set_height = 2.5f;
volatile float g_actual_height = 0.0f;
volatile uint16_t g_pwm_output = 0;
volatile uint32_t g_suspend_time = 0;
volatile uint8_t g_key_event = 0;

/* PID控制器实例（使用改进版PID_V2） */
PID_V2_t pid_controller;

/* 可选：ADRC控制器 */
ADRC_t adrc_controller;
volatile uint8_t g_use_adrc = 0;    // 0=使用PID，1=使用ADRC

/* 函数声明 */
void System_Process(void);
void StateMachine_Run(void);
void Display_Update(void);
void Debug_Output(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    
    MX_GPIO_Init();
    MX_USART1_Init();
    MX_ADC1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    
    OLED_Init();
    Key_Init();
    Height_Init();
    
    /* 初始化PID_V2 */
    PID_V2_Init(&pid_controller, 50.0f, 0.5f, 20.0f, 0.0f, PWM_MAX_DUTY, PWM_MAX_DUTY * 0.6f);
    PID_V2_SetFilterAlpha(&pid_controller, 0.3f);
    
    /* 初始化ADRC */
    ADRC_Init(&adrc_controller, 0.001f, 0.5f, 25.0f, 10.0f, 0.0f, PWM_MAX_DUTY);
    ADRC_SetBandwidth(&adrc_controller, 20.0f, 5.0f);
    
    OLED_Clear();
    OLED_ShowString(20, 0, "MagLev v2.1");
    OLED_ShowString(30, 2, "PID Mode");
    OLED_ShowString(20, 4, "Initializing...");
    
    while(1)
    {
        /* 按键处理 */
        uint8_t key = Key_Scan();
        if(key == KEY1_PRESS)
        {
            g_state = STATE_LIFTING;
            g_pwm_output = 1500;
            PWM_SetDuty(g_pwm_output);
        }
        else if(key == KEY2_PRESS)
        {
            g_set_height += 0.2f;
            if(g_set_height > 4.0f) g_set_height = 1.0f;
        }
        else if(key == KEY3_PRESS)
        {
            g_use_adrc = !g_use_adrc;
            if(g_use_adrc)
                OLED_ShowString(30, 2, "ADRC Mode");
            else
                OLED_ShowString(30, 2, "PID Mode ");
        }
        
        /* 状态机（只做状态转换，不执行PID） */
        StateMachine_Run();
        
        /* 显示更新 */
        Display_Update();
        
        /* 短延时 */
        HAL_Delay(50);
    }
}

/**
 * @brief  状态机运行（修复：不在这里执行PID）
 */
void StateMachine_Run(void)
{
    switch(g_state)
    {
        case STATE_IDLE:
            PWM_SetDuty(0);
            break;
            
        case STATE_LIFTING:
            /* 起浮阶段：线性增加PWM */
            if(g_actual_height > 0.5f)
            {
                g_state = STATE_SUSPENDING;
                /* 继承当前PWM作为PID初始输出 */
                pid_controller.output = (float)g_pwm_output;
            }
            break;
            
        case STATE_SUSPENDING:
            /* 悬浮状态：PID已在中断中执行，此处只做监控 */
            g_suspend_time++;
            
            /* 检查是否掉落 */
            if(g_actual_height < 0.5f)
            {
                g_state = STATE_IDLE;
                PWM_SetDuty(0);
                PID_V2_Reset(&pid_controller);
                ADRC_Reset(&adrc_controller);
            }
            break;
            
        case STATE_ADJUSTING:
            /* 高度调节：逐渐逼近新设定值 */
            if(fabsf(g_set_height - g_actual_height) < 0.3f)
            {
                g_state = STATE_SUSPENDING;
            }
            break;
            
        case STATE_ERROR:
            PWM_SetDuty(0);
            break;
    }
}

/**
 * @brief  TIM2中断回调（1kHz）- PID只在这里执行
 * 
 * 修复：删除StateMachine_Run中的PID计算，只保留此处
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)  // 1kHz
    {
        /* 读取霍尔传感器 */
        uint16_t adc_values[4];
        ADC_ReadAll(adc_values);
        g_actual_height = Height_Calculate(adc_values);
        
        /* 只在悬浮状态下执行PID控制 */
        if (g_state == STATE_SUSPENDING || g_state == STATE_ADJUSTING)
        {
            float pid_output;
            
            if(g_use_adrc)
            {
                /* ADRC控制 */
                pid_output = ADRC_Update(&adrc_controller, g_actual_height, g_set_height);
            }
            else
            {
                /* PID_V2控制 */
                pid_output = PID_V2_Calculate(&pid_controller, g_set_height, g_actual_height);
            }
            
            /* 输出限幅 */
            if (pid_output < 0) pid_output = 0;
            if (pid_output > PWM_MAX_DUTY) pid_output = PWM_MAX_DUTY;
            
            g_pwm_output = (uint16_t)pid_output;
            PWM_SetDuty(g_pwm_output);
        }
    }
}

void Display_Update(void)
{
    char buf[32];
    sprintf(buf, "H:%.1f S:%.1f", g_actual_height, g_set_height);
    OLED_ShowString(0, 4, buf);
    sprintf(buf, "PWM:%d T:%lu", g_pwm_output, g_suspend_time);
    OLED_ShowString(0, 6, buf);
}
