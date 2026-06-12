/**
 * @file    main.c
 * @brief   2025_H题_野生动物巡查系统 - 主程序
 * @author  电赛控制题自动处理系统
 * 
 * 传感器+控制系统
 * 核心算法：PID控制+信号处理
 */

#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <math.h>

typedef struct { float Kp, Ki, Kd, integral, prev_error; } PID_t;

float PID_Calc(PID_t *p, float t, float a) {
    float e = t - a; p->integral += e;
    if(p->integral>500)p->integral=500; if(p->integral<-500)p->integral=-500;
    float d = e - p->prev_error; p->prev_error = e;
    float o = p->Kp*e + p->Ki*p->integral + p->Kd*d;
    if(o>500)o=500; if(o<-500)o=-500; return o;
}

PID_t g_pid;

int main(void) {
    HAL_Init(); SystemClock_Config();
    GPIO_Init(); TIM_Init(); ADC_Init();
    UART_Init(); OLED_Init();
    
    g_pid = (PID_t){2.0f, 0.3f, 1.0f, 0, 0};
    
    OLED_Clear();
    OLED_ShowString(0, 0, "2025_H题_野生动物巡查系统");
    
    while(1) {
        float sensor_val = Sensor_Read();
        float target = Target_Get();
        float pid_out = PID_Calc(&g_pid, target, sensor_val);
        
        Actuator_Set(pid_out);
        
        char buf[32];
        sprintf(buf, "S:%.1f T:%.1f", sensor_val, target);
        OLED_ShowString(0, 2, buf);
        
        HAL_Delay(10);
    }
}
