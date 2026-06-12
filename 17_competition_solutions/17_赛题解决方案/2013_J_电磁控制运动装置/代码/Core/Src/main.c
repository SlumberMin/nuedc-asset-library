/**
 * @file    main.c
 * @brief   2013_J_电磁控制运动装置 - 主程序
 * @author  电赛控制题自动处理系统
 * 
 * 系统采用STM32F103C8T6主控
 * 传感器：红外循迹+编码器+超声波
 * 执行器：直流减速电机+舵机
 * 算法：PID控制
 */

#include "stm32f1xx_hal.h"
#include <stdio.h>

typedef struct { float Kp, Ki, Kd, integral, prev_error; } PID_t;

float PID_Calc(PID_t *p, float t, float a) {
    float e = t - a; p->integral += e;
    if(p->integral>500)p->integral=500; if(p->integral<-500)p->integral=-500;
    float d = e - p->prev_error; p->prev_error = e;
    float o = p->Kp*e + p->Ki*p->integral + p->Kd*d;
    if(o>500)o=500; if(o<-500)o=-500; return o;
}

PID_t g_track_pid, g_speed_pid;

int main(void) {
    HAL_Init(); SystemClock_Config();
    GPIO_Init(); TIM_Init(); UART_Init();
    OLED_Init(); Motor_Init(); Sensor_Init();
    
    g_track_pid = (PID_t){2.0f, 0.3f, 1.0f, 0, 0};
    g_speed_pid = (PID_t){3.0f, 0.5f, 0.5f, 0, 0};
    
    OLED_Clear();
    OLED_ShowString(0, 0, "2013_J_电磁控制运动装置");
    OLED_ShowString(0, 2, "Running...");
    
    while(1) {
        int16_t pos = Sensor_GetPosition();
        float pid_out = PID_Calc(&g_track_pid, 0, pos);
        
        int16_t base = 400;
        Motor_SetSpeed(base + pid_out, base - pid_out);
        
        char buf[32];
        sprintf(buf, "Pos: %d", pos);
        OLED_ShowString(0, 4, buf);
        
        HAL_Delay(10);
    }
}
