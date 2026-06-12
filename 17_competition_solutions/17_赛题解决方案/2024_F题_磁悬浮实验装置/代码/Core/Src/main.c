/**
 * @file    main.c
 * @brief   磁悬浮实验装置 - 主程序
 * @author  电赛F题
 * @version 1.0
 * @date    2024
 * 
 * 系统功能：
 * 1. 霍尔传感器采集悬浮盘高度
 * 2. PID控制算法调节电磁铁电流
 * 3. PWM输出驱动电磁铁
 * 4. OLED显示系统状态
 * 5. 按键设定悬浮高度
 */

#include "stm32f1xx_hal.h"
#include "system.h"
#include "adc.h"
#include "pwm.h"
#include "pid.h"
#include "height.h"
#include "oled.h"
#include "key.h"
#include "usart.h"

/* 系统状态定义 */
typedef enum {
    STATE_IDLE = 0,         // 空闲状态（电磁铁断电）
    STATE_SEARCHING,        // 搜索状态（检测悬浮盘）
    STATE_LIFTING,          // 起浮状态（逐渐增加电流）
    STATE_SUSPENDING,       // 悬浮状态（PID控制）
    STATE_ADJUSTING,        // 调节状态（高度变化过渡）
    STATE_ERROR             // 错误状态
} SystemState;

/* 全局变量 */
volatile SystemState g_state = STATE_IDLE;    // 系统状态
volatile float g_set_height = 2.5f;           // 设定悬浮高度(cm)
volatile float g_actual_height = 0.0f;        // 实际悬浮高度(cm)
volatile uint16_t g_pwm_output = 0;           // PWM输出值
volatile uint32_t g_suspend_time = 0;         // 悬浮持续时间(ms)
volatile uint8_t g_key_event = 0;             // 按键事件标志

/* PID控制器实例 */
PID_Controller_t pid_controller;

/* 函数声明 */
void System_Process(void);
void StateMachine_Run(void);
void Display_Update(void);
void Debug_Output(void);

/**
 * @brief  主函数
 * @retval 无
 */
int main(void)
{
    /* 系统初始化 */
    HAL_Init();
    SystemClock_Config();       // 配置系统时钟72MHz
    
    /* 外设初始化 */
    MX_GPIO_Init();             // GPIO初始化
    MX_USART1_Init();           // 串口1初始化(115200)
    MX_ADC1_Init();             // ADC1初始化(4通道)
    MX_TIM2_Init();             // TIM2初始化(1kHz中断)
    MX_TIM3_Init();             // TIM3初始化(PWM输出)
    
    /* 模块初始化 */
    OLED_Init();                // OLED显示屏初始化
    Key_Init();                 // 按键初始化
    Height_Init();              // 高度检测模块初始化
    PID_Init(&pid_controller);  // PID控制器初始化
    
    /* 显示开机画面 */
    OLED_Clear();
    OLED_ShowString(20, 0, "Magnetic Levitation");
    OLED_ShowString(30, 2, "System v1.0");
    OLED_ShowString(20, 4, "Initializing...");
    HAL_Delay(1000);
    
    /* 启动定时器中断 */
    HAL_TIM_Base_Start_IT(&htim2);  // 启动1kHz定时器中断
    
    /* 发送启动信息 */
    USART1_SendString("System Started!\r\n");
    USART1_SendString("Set Height: ");
    USART1_SendFloat(g_set_height);
    USART1_SendString(" cm\r\n");
    
    /* 主循环 */
    while (1)
    {
        /* 扫描按键 */
        g_key_event = Key_Scan();
        
        /* 处理按键事件 */
        if (g_key_event != 0)
        {
            switch (g_key_event)
            {
                case KEY1_PRESS:    // 高度+
                    g_set_height += 0.2f;
                    if (g_set_height > 5.0f) g_set_height = 5.0f;
                    g_state = STATE_ADJUSTING;
                    break;
                    
                case KEY2_PRESS:    // 高度-
                    g_set_height -= 0.2f;
                    if (g_set_height < 1.0f) g_set_height = 1.0f;
                    g_state = STATE_ADJUSTING;
                    break;
                    
                case KEY3_PRESS:    // 启动/停止
                    if (g_state == STATE_IDLE)
                    {
                        g_state = STATE_SEARCHING;
                    }
                    else
                    {
                        g_state = STATE_IDLE;
                        g_pwm_output = 0;
                        PWM_SetDuty(0);
                    }
                    break;
            }
            g_key_event = 0;
        }
        
        /* 状态机处理 */
        StateMachine_Run();
        
        /* 更新显示 */
        Display_Update();
        
        /* 调试输出 */
        Debug_Output();
        
        /* 主循环延时 */
        HAL_Delay(50);  // 20Hz刷新率
    }
}

/**
 * @brief  状态机运行函数
 * @note   处理系统状态转换和控制逻辑
 */
void StateMachine_Run(void)
{
    static uint32_t search_timer = 0;
    static uint32_t lift_timer = 0;
    
    switch (g_state)
    {
        case STATE_IDLE:
            /* 空闲状态：电磁铁断电 */
            PWM_SetDuty(0);
            g_suspend_time = 0;
            break;
            
        case STATE_SEARCHING:
            /* 搜索状态：低电流检测悬浮盘 */
            PWM_SetDuty(200);  // 约8%占空比
            search_timer++;
            
            /* 检测到悬浮盘（高度<10cm表示有物体） */
            if (g_actual_height < 10.0f && g_actual_height > 0.5f)
            {
                g_state = STATE_LIFTING;
                lift_timer = 0;
                search_timer = 0;
            }
            
            /* 超时未检测到 */
            if (search_timer > 100)  // 5秒超时
            {
                g_state = STATE_ERROR;
                search_timer = 0;
            }
            break;
            
        case STATE_LIFTING:
            /* 起浮状态：逐渐增加电流 */
            lift_timer++;
            
            /* 缓慢增加PWM占空比 */
            if (lift_timer < 50)
            {
                PWM_SetDuty(lift_timer * 20);  // 逐渐增加
            }
            else
            {
                /* 切换到PID控制 */
                g_state = STATE_SUSPENDING;
                PID_Reset(&pid_controller);
            }
            
            /* 检测是否起浮成功 */
            if (g_actual_height > 1.0f)
            {
                g_state = STATE_SUSPENDING;
                PID_Reset(&pid_controller);
            }
            break;
            
        case STATE_SUSPENDING:
            /* 悬浮状态：PID闭环控制 */
            g_suspend_time++;
            
            /* PID计算 */
            float pid_output = PID_Calculate(&pid_controller, 
                                              g_set_height, 
                                              g_actual_height);
            
            /* 限制输出范围 */
            if (pid_output < 0) pid_output = 0;
            if (pid_output > 4095) pid_output = 4095;
            
            g_pwm_output = (uint16_t)pid_output;
            PWM_SetDuty(g_pwm_output);
            
            /* 检测是否掉落 */
            if (g_actual_height < 0.5f)
            {
                g_state = STATE_IDLE;
            }
            break;
            
        case STATE_ADJUSTING:
            /* 调节状态：高度设定变化时的过渡 */
            g_state = STATE_SUSPENDING;
            break;
            
        case STATE_ERROR:
            /* 错误状态：断电保护 */
            PWM_SetDuty(0);
            
            /* 显示错误信息 */
            OLED_ShowString(0, 6, "ERROR!         ");
            break;
    }
}

/**
 * @brief  显示更新函数
 * @note   更新OLED显示内容
 */
void Display_Update(void)
{
    char buf[20];
    
    /* 第1行：系统状态 */
    switch (g_state)
    {
        case STATE_IDLE:
            OLED_ShowString(0, 0, "Status: IDLE   ");
            break;
        case STATE_SEARCHING:
            OLED_ShowString(0, 0, "Status: SEARCH ");
            break;
        case STATE_LIFTING:
            OLED_ShowString(0, 0, "Status: LIFT   ");
            break;
        case STATE_SUSPENDING:
            OLED_ShowString(0, 0, "Status: SUSPEND");
            break;
        case STATE_ERROR:
            OLED_ShowString(0, 0, "Status: ERROR  ");
            break;
        default:
            OLED_ShowString(0, 0, "Status: ???    ");
            break;
    }
    
    /* 第2行：设定高度 */
    sprintf(buf, "Set: %.1f cm   ", g_set_height);
    OLED_ShowString(0, 2, buf);
    
    /* 第3行：实际高度 */
    sprintf(buf, "Now: %.1f cm   ", g_actual_height);
    OLED_ShowString(0, 4, buf);
    
    /* 第4行：PWM输出 */
    sprintf(buf, "PWM: %d        ", g_pwm_output);
    OLED_ShowString(0, 6, buf);
}

/**
 * @brief  调试输出函数
 * @note   通过串口输出调试信息
 */
void Debug_Output(void)
{
    char debug_buf[50];
    
    sprintf(debug_buf, "$DATA,%.2f,%.2f,%d,%lu,%d\r\n",
            g_actual_height,
            g_set_height,
            g_pwm_output,
            g_suspend_time,
            g_state);
    
    USART1_SendString(debug_buf);
}

/**
 * @brief  TIM2定时器中断回调函数
 * @note   1kHz采样率，执行ADC采集和PID计算
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        /* 读取霍尔传感器ADC值 */
        uint16_t adc_values[4];
        ADC_ReadAll(adc_values);
        
        /* 计算悬浮盘高度 */
        g_actual_height = Height_Calculate(adc_values);
        
        /* 在悬浮状态下执行PID控制 */
        if (g_state == STATE_SUSPENDING)
        {
            float pid_output = PID_Calculate(&pid_controller, 
                                              g_set_height, 
                                              g_actual_height);
            
            if (pid_output < 0) pid_output = 0;
            if (pid_output > 4095) pid_output = 4095;
            
            g_pwm_output = (uint16_t)pid_output;
            PWM_SetDuty(g_pwm_output);
        }
    }
}
