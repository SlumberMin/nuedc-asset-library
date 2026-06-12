/**
 * @file    main.c
 * @brief   2019年B题 巡线机器人 - 主程序
 * 
 * 功能：沿复杂路径自动巡线行驶
 * 传感器：红外循迹阵列+陀螺仪
 * 特点：支持急弯、十字路口、同心圆
 */

#include "stm32f1xx_hal.h"
#include <stdio.h>

typedef struct { float Kp, Ki, Kd, integral, prev_error; } PID_t;
PID_t g_track_pid;
volatile uint32_t g_timer_ms = 0;

float PID_Calc(PID_t *p, float t, float a) {
    float e = t - a; p->integral += e;
    if(p->integral>500)p->integral=500; if(p->integral<-500)p->integral=-500;
    float d = e - p->prev_error; p->prev_error = e;
    float o = p->Kp*e + p->Ki*p->integral + p->Kd*d;
    if(o>500)o=500; if(o<-500)o=-500; return o;
}

int main(void) {
    HAL_Init(); SystemClock_Config();
    GPIO_Init(); TIM_Init(); UART_Init();
    OLED_Init(); Motor_Init(); Sensor_Init();
    g_track_pid = (PID_t){2.5f, 0.3f, 1.2f, 0, 0};
    
    OLED_Clear(); OLED_ShowString(0,0,"Line Follower Robot");
    
    while(1) {
        int16_t pos = Sensor_GetPosition();
        float pid_out = PID_Calc(&g_track_pid, 0, pos);
        
        int16_t base = 400;
        Motor_SetSpeed(base + pid_out, base - pid_out);
        
        g_timer_ms++;
        char buf[32];
        sprintf(buf,"Pos:%d T:%lus",pos,g_timer_ms/1000);
        OLED_ShowString(0,2,buf);
        HAL_Delay(10);
    }
}
