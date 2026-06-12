/**
 * @file    main.c
 * @brief   2023年E题 红色光斑位置控制系统 - 主程序
 * @note    与绿色系统完全独立，无任何通信
 * 
 * 功能：
 * 1. OpenMV识别屏幕边线和靶纸位置
 * 2. 二维云台PID控制红色激光笔位置
 * 3. 支持位置复位、边线运动、靶纸运动
 */

#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* 系统状态 */
typedef enum {
    MODE_IDLE = 0,      // 空闲
    MODE_RESET,         // 复位到原点
    MODE_BORDER,        // 沿边线运动
    MODE_TARGET,        // 沿靶纸运动
    MODE_MANUAL         // 手动控制
} Mode_t;

/* 运动路径点 */
typedef struct {
    float x;            // X坐标(-1~+1归一化)
    float y;            // Y坐标(-1~+1归一化)
} Point_t;

/* 全局变量 */
volatile Mode_t g_mode = MODE_IDLE;
volatile float g_angle_h = 90.0f;   // 水平舵机角度
volatile float g_angle_v = 90.0f;   // 垂直舵机角度
volatile uint32_t g_timer_ms = 0;   // 计时器

/* PID控制器 */
typedef struct {
    float Kp, Ki, Kd;
    float integral, prev_error;
    float out_min, out_max;
} PID_t;

PID_t pid_x, pid_y;

/* 路径点（屏幕边线，归一化坐标） */
static const Point_t border_path[] = {
    {0.5f, 0.5f},   // 右上
    {-0.5f, 0.5f},  // 左上
    {-0.5f, -0.5f}, // 左下
    {0.5f, -0.5f},  // 右下
    {0.5f, 0.5f},   // 回到右上
};

/* 函数声明 */
float PID_Calc(PID_t *pid, float target, float actual);
void Servo_SetAngle(float h, float v);
void Laser_On(void);
void Laser_Off(void);
void MoveToPoint(float target_x, float target_y);
void ProcessMode(void);

/**
 * @brief  主函数
 */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    
    /* 外设初始化 */
    GPIO_Init();
    TIM_Init();     // PWM for servos
    UART_Init();    // OpenMV communication
    
    /* PID初始化 */
    pid_x = (PID_t){1.5f, 0.2f, 0.8f, 0, 0, -90, 90};
    pid_y = (PID_t){1.5f, 0.2f, 0.8f, 0, 0, -90, 90};
    
    /* 云台居中 */
    Servo_SetAngle(90, 90);
    Laser_Off();
    
    while(1)
    {
        /* 按键扫描 */
        uint8_t key = Key_Scan();
        switch(key)
        {
            case KEY1: g_mode = MODE_RESET; break;
            case KEY2: g_mode = MODE_BORDER; g_timer_ms = 0; break;
            case KEY3: g_mode = MODE_TARGET; g_timer_ms = 0; break;
            case KEY4: Laser_On(); break;
        }
        
        /* 模式处理 */
        ProcessMode();
        
        /* 显示 */
        Display_Update();
        
        HAL_Delay(10);
    }
}

/**
 * @brief  PID计算
 */
float PID_Calc(PID_t *pid, float target, float actual)
{
    float error = target - actual;
    pid->integral += error;
    if(pid->integral > pid->out_max) pid->integral = pid->out_max;
    if(pid->integral < pid->out_min) pid->integral = pid->out_min;
    
    float derivative = error - pid->prev_error;
    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
    
    if(output > pid->out_max) output = pid->out_max;
    if(output < pid->out_min) output = pid->out_min;
    
    pid->prev_error = error;
    return output;
}

/**
 * @brief  模式处理状态机
 */
void ProcessMode(void)
{
    static uint8_t path_index = 0;
    static uint32_t path_timer = 0;
    float vision_x, vision_y;
    
    switch(g_mode)
    {
        case MODE_IDLE:
            break;
            
        case MODE_RESET:
            /* 复位到原点 */
            Vision_GetTarget(&vision_x, &vision_y);
            float dx = PID_Calc(&pid_x, 0.0f, vision_x);
            float dy = PID_Calc(&pid_y, 0.0f, vision_y);
            g_angle_h -= dx * 0.1f;
            g_angle_v += dy * 0.1f;
            Servo_SetAngle(g_angle_h, g_angle_v);
            Laser_On();
            
            if(fabsf(vision_x) < 0.05f && fabsf(vision_y) < 0.05f)
            {
                g_mode = MODE_IDLE; // 复位完成
            }
            break;
            
        case MODE_BORDER:
            /* 沿边线运动 */
            path_timer += 10;
            /* 30s完成一圈，每段边约7.5s */
            if(path_timer > 7500)
            {
                path_timer = 0;
                path_index++;
                if(path_index > 4) { path_index = 0; g_mode = MODE_IDLE; }
            }
            
            /* 在当前路径段内插值运动 */
            float t = (float)path_timer / 7500.0f;
            Point_t start = border_path[path_index];
            Point_t end = border_path[path_index + 1];
            float target_x = start.x + t * (end.x - start.x);
            float target_y = start.y + t * (end.y - start.y);
            
            MoveToPoint(target_x, target_y);
            Laser_On();
            break;
            
        case MODE_TARGET:
            /* 沿靶纸运动（由OpenMV识别靶纸边框） */
            Vision_GetTarget(&vision_x, &vision_y);
            dx = PID_Calc(&pid_x, 0.0f, vision_x);
            dy = PID_Calc(&pid_y, 0.0f, vision_y);
            g_angle_h -= dx * 0.1f;
            g_angle_v += dy * 0.1f;
            Servo_SetAngle(g_angle_h, g_angle_v);
            Laser_On();
            break;
            
        case MODE_MANUAL:
            break;
    }
}

/**
 * @brief  移动到指定归一化坐标
 */
void MoveToPoint(float target_x, float target_y)
{
    float vision_x, vision_y;
    Vision_GetTarget(&vision_x, &vision_y);
    
    float dx = PID_Calc(&pid_x, target_x, vision_x);
    float dy = PID_Calc(&pid_y, target_y, vision_y);
    
    g_angle_h -= dx * 0.05f;
    g_angle_v += dy * 0.05f;
    
    /* 限幅 */
    if(g_angle_h < 0) g_angle_h = 0;
    if(g_angle_h > 180) g_angle_h = 180;
    if(g_angle_v < 0) g_angle_v = 0;
    if(g_angle_v > 180) g_angle_v = 180;
    
    Servo_SetAngle(g_angle_h, g_angle_v);
}

/**
 * @brief  设置舵机角度
 */
void Servo_SetAngle(float h, float v)
{
    /* 角度映射到PWM脉宽 */
    uint16_t pulse_h = 500 + (uint16_t)(h / 180.0f * 2000);
    uint16_t pulse_v = 500 + (uint16_t)(v / 180.0f * 2000);
    TIM3->CCR1 = pulse_h;
    TIM3->CCR2 = pulse_v;
}

void Laser_On(void)  { HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); }
void Laser_Off(void) { HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); }
