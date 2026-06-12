/**
 * @file    main.c
 * @brief   2016年C题 自动循迹小车 - 主程序
 * 
 * 功能：沿黑色引导线自动行驶
 * 传感器：红外循迹传感器阵列
 * 驱动：直流减速电机+L298N
 */

#include "stm32f1xx_hal.h"
#include <stdio.h>

/* PID控制器 */
typedef struct {
    float Kp, Ki, Kd;
    float integral, prev_error;
} PID_t;

PID_t g_track_pid;
volatile uint32_t g_timer_ms = 0;

void System_Init(void);
float PID_Calc(PID_t *pid, float target, float actual);
int16_t Track_GetPosition(void);
void Motor_SetSpeed(int16_t left, int16_t right);

int main(void)
{
    System_Init();
    g_track_pid = (PID_t){2.0f, 0.3f, 1.0f, 0, 0};
    
    OLED_Clear();
    OLED_ShowString(0, 0, "Auto Tracking Car");
    OLED_ShowString(0, 2, "Running...");
    
    while(1)
    {
        /* 循迹PID控制 */
        int16_t position = Track_GetPosition();
        float pid_out = PID_Calc(&g_track_pid, 0, position);
        
        int16_t base_speed = 400;
        int16_t speed_l = base_speed + (int16_t)pid_out;
        int16_t speed_r = base_speed - (int16_t)pid_out;
        
        /* 限幅 */
        if(speed_l > 999) speed_l = 999;
        if(speed_l < -999) speed_l = -999;
        if(speed_r > 999) speed_r = 999;
        if(speed_r < -999) speed_r = -999;
        
        Motor_SetSpeed(speed_l, speed_r);
        
        /* 计时 */
        g_timer_ms++;
        
        /* 显示 */
        char buf[32];
        sprintf(buf, "Pos: %d", position);
        OLED_ShowString(0, 4, buf);
        sprintf(buf, "Time: %lus", g_timer_ms / 1000);
        OLED_ShowString(0, 6, buf);
        
        HAL_Delay(10);
    }
}

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
