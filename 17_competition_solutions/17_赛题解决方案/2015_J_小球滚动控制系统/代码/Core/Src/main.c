/**
 * @file    main.c
 * @brief   2015年J题 小球滚动控制系统 - 主程序
 * 
 * 系统结构：门形支架+U型导轨+电机控制导轨倾斜
 * 小球在导轨上滚动，通过控制导轨角度控制小球位置
 * 
 * 功能：
 * 1. 导轨两端触发检测（声光指示）
 * 2. 自动调平（±15°→水平）
 * 3. 小球往复运动（5-55cm区间）
 * 4. 定点停止（30±2cm位置）
 * 5. 可设置往复周期和幅度
 */

#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <math.h>

/* 系统参数 */
#define RAIL_LENGTH     60      // 导轨长度(cm)
#define RAIL_CENTER     30      // 导轨中心位置(cm)
#define ADC_CENTER      2048    // 水平时ADC中值

/* 运动模式 */
typedef enum {
    MODE_IDLE = 0,
    MODE_LEVELING,          // 调平
    MODE_OSCILLATING,       // 往复运动
    MODE_POSITIONING,       // 定点停止
    MODE_SET_PERIOD,        // 设定周期往复
    MODE_SET_AMPLITUDE      // 设定幅度往复
} Mode_t;

/* 全局变量 */
volatile Mode_t g_mode = MODE_IDLE;
volatile float g_ball_pos = 0;          // 小球位置(cm)
volatile float g_ball_velocity = 0;     // 小球速度(cm/s)
volatile float g_target_pos = 30;       // 目标位置(cm)
volatile float g_rail_angle = 0;        // 导轨倾斜角(°)
volatile float g_target_angle = 0;      // 目标角度(°)
volatile uint32_t g_timer_ms = 0;
volatile uint16_t g_oscillate_count = 0; // 往复次数
volatile float g_oscillate_period = 5.0f; // 往复周期(s)
volatile float g_oscillate_amplitude = 20.0f; // 往复幅度(cm)

/* PID控制器 */
typedef struct {
    float Kp, Ki, Kd;
    float integral, prev_error;
} PID_t;

PID_t pid_pos;      // 位置PID
PID_t pid_angle;    // 角度PID

/* 函数声明 */
void System_Init(void);
float PID_Calc(PID_t *pid, float target, float actual);
float Ball_GetPosition(void);
void Motor_SetAngle(float angle);
void Control_Loop(void);

/**
 * @brief  主函数
 */
int main(void)
{
    System_Init();
    
    /* PID参数初始化 */
    pid_pos = (PID_t){0.5f, 0.02f, 0.8f, 0, 0};
    pid_angle = (PID_t){2.0f, 0.1f, 1.0f, 0, 0};
    
    OLED_Clear();
    OLED_ShowString(0, 0, "Ball Roll Control");
    OLED_ShowString(0, 2, "Pos: 0.0cm");
    OLED_ShowString(0, 4, "Angle: 0.0deg");
    OLED_ShowString(0, 6, "Mode: IDLE");
    
    while(1)
    {
        /* 按键处理 */
        uint8_t key = Key_Scan();
        switch(key)
        {
            case KEY1: g_mode = MODE_LEVELING; g_timer_ms = 0; break;
            case KEY2: g_mode = MODE_OSCILLATING; g_timer_ms = 0; g_oscillate_count = 0; break;
            case KEY3: g_mode = MODE_POSITIONING; g_target_pos = 30; break;
            case KEY4: 
                g_oscillate_period += 1.0f;
                if(g_oscillate_period > 8.0f) g_oscillate_period = 3.0f;
                break;
            case KEY5:
                g_oscillate_amplitude += 5.0f;
                if(g_oscillate_amplitude > 25.0f) g_oscillate_amplitude = 15.0f;
                break;
        }
        
        /* 控制循环 */
        Control_Loop();
        
        /* 显示 */
        char buf[32];
        sprintf(buf, "Pos: %.1fcm", g_ball_pos);
        OLED_ShowString(0, 2, buf);
        sprintf(buf, "Ang: %.1fdeg", g_rail_angle);
        OLED_ShowString(0, 4, buf);
        
        HAL_Delay(10);
    }
}

/**
 * @brief  控制主循环
 */
void Control_Loop(void)
{
    /* 1. 读取小球位置 */
    g_ball_pos = Ball_GetPosition();
    
    /* 2. 检测端点触发 */
    if(g_ball_pos < 2.0f || g_ball_pos > 58.0f)
    {
        Alert_Beep(50);     // 端点声光提示
    }
    
    /* 3. 模式处理 */
    switch(g_mode)
    {
        case MODE_IDLE:
            Motor_SetAngle(0);
            break;
            
        case MODE_LEVELING:
        {
            /* 自动调平：读取当前角度，PID控制到水平 */
            float current_angle = MPU6050_GetAngle();
            float angle_cmd = PID_Calc(&pid_angle, 0.0f, current_angle);
            Motor_SetAngle(angle_cmd);
            
            if(fabsf(current_angle) < 0.5f && g_timer_ms > 3000)
            {
                g_mode = MODE_IDLE;
                Alert_Beep(200);    // 调平完成
            }
            g_timer_ms += 10;
            break;
        }
            
        case MODE_OSCILLATING:
        {
            /* 往复运动：小球在5-55cm区间做往复运动 */
            float t = (float)g_timer_ms / 1000.0f;
            float omega = 2.0f * 3.14159f / g_oscillate_period;
            float target = RAIL_CENTER + g_oscillate_amplitude * sinf(omega * t);
            
            /* 位置PID控制 */
            float pos_error = target - g_ball_pos;
            float angle_cmd = PID_Calc(&pid_pos, target, g_ball_pos);
            
            /* 角度限幅 */
            if(angle_cmd > 15.0f) angle_cmd = 15.0f;
            if(angle_cmd < -15.0f) angle_cmd = -15.0f;
            
            Motor_SetAngle(angle_cmd);
            
            /* 计数往复次数 */
            static float last_target = 0;
            if((last_target < RAIL_CENTER && target >= RAIL_CENTER) ||
               (last_target > RAIL_CENTER && target <= RAIL_CENTER))
            {
                g_oscillate_count++;
            }
            last_target = target;
            
            g_timer_ms += 10;
            break;
        }
            
        case MODE_POSITIONING:
        {
            /* 定点停止：控制小球到指定位置 */
            float angle_cmd = PID_Calc(&pid_pos, g_target_pos, g_ball_pos);
            if(angle_cmd > 15.0f) angle_cmd = 15.0f;
            if(angle_cmd < -15.0f) angle_cmd = -15.0f;
            Motor_SetAngle(angle_cmd);
            
            /* 检查是否到达 */
            if(fabsf(g_ball_pos - g_target_pos) < 2.0f)
            {
                Alert_Beep(100);    // 到达提示
            }
            break;
        }
    }
}

/**
 * @brief  PID计算
 */
float PID_Calc(PID_t *pid, float target, float actual)
{
    float error = target - actual;
    pid->integral += error;
    if(pid->integral > 50.0f) pid->integral = 50.0f;
    if(pid->integral < -50.0f) pid->integral = -50.0f;
    
    float derivative = error - pid->prev_error;
    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
    
    pid->prev_error = error;
    return output;
}

/**
 * @brief  读取小球位置
 * @retval float: 位置(cm)
 * 
 * 位置检测方案：
 * 1. 红外对射传感器阵列（离散点检测）
 * 2. 电位器+导轨角度推算
 * 3. 视觉检测
 * 
 * 本实现采用红外对射传感器阵列
 */
float Ball_GetPosition(void)
{
    /* 读取多个红外传感器的状态 */
    /* 通过检测小球遮挡的传感器位置推算位置 */
    /* 简化实现：读取电位器反馈导轨角度，结合加速度推算 */
    uint16_t adc_val = ADC_Read(0);  // 读取位置传感器ADC
    
    /* ADC值映射到位置 */
    float pos = (float)adc_val / 4096.0f * RAIL_LENGTH;
    return pos;
}

/**
 * @brief  设置导轨倾斜角度
 * @param  angle: 目标角度(-15°~+15°)
 */
void Motor_SetAngle(float angle)
{
    if(angle > 15.0f) angle = 15.0f;
    if(angle < -15.0f) angle = -15.0f;
    
    /* 角度映射到电机PWM */
    int16_t pwm = (int16_t)(angle / 15.0f * 999);
    
    /* 设置电机PWM */
    if(pwm >= 0)
    {
        TIM3->CCR1 = pwm;
        TIM3->CCR2 = 0;
    }
    else
    {
        TIM3->CCR1 = 0;
        TIM3->CCR2 = -pwm;
    }
    
    g_rail_angle = angle;
}
