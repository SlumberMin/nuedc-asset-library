/**
 * @file    main.c
 * @brief   2023年I题 气垫悬浮车 - 主程序
 * 
 * 功能：
 * 1. 轴流风机产生气垫悬浮
 * 2. 红外循迹传感器引导行驶
 * 3. 超声波避障
 * 4. 语音播报运行状态
 */

#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <string.h>

/* 系统状态 */
typedef enum {
    STATE_IDLE = 0,         // 空闲
    STATE_HOVERING,         // 悬浮中
    STATE_RUNNING,          // 行驶中
    STATE_AVOIDING,         // 避障中
    STATE_PARKING,          // 停车中
    STATE_STOP              // 停止
} State_t;

/* 全局变量 */
volatile State_t g_state = STATE_IDLE;
volatile uint32_t g_timer_ms = 0;
volatile uint16_t g_hover_time = 0;    // 悬停时间(s)
volatile uint16_t g_run_time = 0;      // 运行时间(s)
volatile uint8_t g_obstacle_count = 0; // 避障次数

/* PID控制器 */
typedef struct {
    float Kp, Ki, Kd;
    float integral, prev_error;
} PID_t;

PID_t g_track_pid;

/* 风机控制 */
#define FAN_HOVER_TIM    TIM3    // 悬浮风机PWM
#define FAN_PUSH_TIM     TIM2    // 推进风机PWM
#define FAN_HOVER_CH     1       // 通道1
#define FAN_PUSH_CH      2       // 通道2

/* 函数声明 */
void System_Init(void);
float PID_Calc(PID_t *pid, float target, float actual);
void Fan_Hover_SetSpeed(uint16_t speed);
void Fan_Push_SetSpeed(uint16_t speed);
void Servo_SetAngle(uint16_t angle);
int16_t Track_GetPosition(void);
uint16_t Ultrasonic_GetDist(uint8_t ch);
void Voice_Speak(const char *text);
void StateMachine_Run(void);

/**
 * @brief  主函数
 */
int main(void)
{
    System_Init();
    
    OLED_Clear();
    OLED_ShowString(0, 0, "Hover Car v1.0");
    OLED_ShowString(0, 2, "State: IDLE");
    
    while(1)
    {
        /* 按键处理 */
        uint8_t key = Key_Scan();
        if(key == KEY1_PRESS)
        {
            if(g_state == STATE_IDLE)
            {
                g_state = STATE_HOVERING;
                Fan_Hover_SetSpeed(800);    // 启动悬浮风机
                Voice_Speak("开始悬浮");
                g_hover_time = 0;
            }
            else
            {
                g_state = STATE_STOP;
                Fan_Hover_SetSpeed(0);
                Fan_Push_SetSpeed(0);
            }
        }
        else if(key == KEY2_PRESS)
        {
            if(g_state == STATE_HOVERING)
            {
                g_state = STATE_RUNNING;
                Fan_Push_SetSpeed(500);     // 启动推进风机
                Voice_Speak("开始行驶");
                g_run_time = 0;
            }
        }
        
        /* 状态机 */
        StateMachine_Run();
        
        /* 显示 */
        char buf[32];
        sprintf(buf, "Time: %ds", g_run_time);
        OLED_ShowString(0, 4, buf);
        sprintf(buf, "Avoid: %d", g_obstacle_count);
        OLED_ShowString(0, 6, buf);
        
        HAL_Delay(10);
    }
}

/**
 * @brief  状态机运行
 */
void StateMachine_Run(void)
{
    static uint32_t avoid_timer = 0;
    int16_t position;
    uint16_t dist_front, dist_left, dist_right;
    
    switch(g_state)
    {
        case STATE_IDLE:
            break;
            
        case STATE_HOVERING:
            /* 悬浮状态：保持风机运行，等待启动行驶 */
            g_hover_time++;
            if(g_hover_time >= 10)  // 悬停10s后自动提示
            {
                Voice_Speak("悬浮稳定，可以行驶");
            }
            break;
            
        case STATE_RUNNING:
            /* 行驶状态：循迹+避障 */
            g_run_time++;
            
            /* 1. 超声波避障检测 */
            dist_front = Ultrasonic_GetDist(0);
            dist_left = Ultrasonic_GetDist(1);
            dist_right = Ultrasonic_GetDist(2);
            
            if(dist_front < 30)  // 前方30cm内有障碍
            {
                g_state = STATE_AVOIDING;
                avoid_timer = 0;
                g_obstacle_count++;
                Voice_Speak("检测到障碍物");
                
                /* 判断绕行方向 */
                if(dist_left > dist_right)
                {
                    Servo_SetAngle(120);    // 向左转
                }
                else
                {
                    Servo_SetAngle(60);     // 向右转
                }
                Fan_Push_SetSpeed(400);     // 减速
                break;
            }
            
            /* 2. 循迹PID控制 */
            position = Track_GetPosition();
            float pid_out = PID_Calc(&g_track_pid, 0.0f, (float)position);
            
            /* 舵机转向 */
            uint16_t servo_angle = 90 + (int16_t)pid_out;
            if(servo_angle < 30) servo_angle = 30;
            if(servo_angle > 150) servo_angle = 150;
            Servo_SetAngle(servo_angle);
            
            /* 推进风机速度 */
            Fan_Push_SetSpeed(600);
            
            /* 超时检查 */
            if(g_run_time > 180)    // 180s超时
            {
                g_state = STATE_STOP;
                Voice_Speak("运行超时");
            }
            break;
            
        case STATE_AVOIDING:
            /* 避障状态 */
            avoid_timer++;
            
            /* 避障动作：转向+前进 */
            if(avoid_timer > 200)   // 避障2s
            {
                /* 检查是否回到寻迹线 */
                position = Track_GetPosition();
                if(abs(position) < 20)
                {
                    g_state = STATE_RUNNING;
                    Servo_SetAngle(90);     // 回正
                    Voice_Speak("避障完成");
                }
            }
            break;
            
        case STATE_PARKING:
            /* 停车状态 */
            Fan_Push_SetSpeed(0);
            Servo_SetAngle(90);
            g_state = STATE_STOP;
            Voice_Speak("到达终点，停车完成");
            break;
            
        case STATE_STOP:
            /* 停止状态 */
            Fan_Push_SetSpeed(0);
            break;
    }
}

/**
 * @brief  PID计算
 */
float PID_Calc(PID_t *pid, float target, float actual)
{
    float error = target - actual;
    pid->integral += error;
    if(pid->integral > 500) pid->integral = 500;
    if(pid->integral < -500) pid->integral = -500;
    
    float derivative = error - pid->prev_error;
    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
    
    if(output > 500) output = 500;
    if(output < -500) output = -500;
    
    pid->prev_error = error;
    return output;
}

/**
 * @brief  悬浮风机速度设置
 */
void Fan_Hover_SetSpeed(uint16_t speed)
{
    if(speed > 999) speed = 999;
    FAN_HOVER_TIM->CCR1 = speed;
}

/**
 * @brief  推进风机速度设置
 */
void Fan_Push_SetSpeed(uint16_t speed)
{
    if(speed > 999) speed = 999;
    FAN_PUSH_TIM->CCR2 = speed;
}

/**
 * @brief  舵机角度设置
 */
void Servo_SetAngle(uint16_t angle)
{
    if(angle > 180) angle = 180;
    uint16_t pulse = 500 + (angle * 2000 / 180);
    TIM1->CCR1 = pulse;
}

/**
 * @brief  语音播报
 */
void Voice_Speak(const char *text)
{
    /* SYN6288语音合成模块UART命令 */
    uint8_t cmd[64];
    uint8_t len = strlen(text);
    cmd[0] = 0xFD;
    cmd[1] = 0x00;
    cmd[2] = len + 2;
    cmd[3] = 0x01;
    cmd[4] = 0x00;
    memcpy(&cmd[5], text, len);
    HAL_UART_Transmit(&huart2, cmd, len + 5, 100);
}
