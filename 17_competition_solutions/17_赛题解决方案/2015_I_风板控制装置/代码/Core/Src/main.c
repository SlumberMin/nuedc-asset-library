/**
 * @file    main.c
 * @brief   2015年I题 风板控制装置 - 主程序
 * 
 * 功能：控制风扇转速调节风板角度
 * 传感器：角度传感器(电位器/MPU6050)
 * 执行器：直流风扇(PWM调速)
 */

#include "stm32f1xx_hal.h"
#include <stdio.h>

typedef struct { float Kp, Ki, Kd, integral, prev_error; } PID_t;
PID_t pid;
volatile float g_target_angle = 30.0f;
volatile float g_actual_angle = 0.0f;

float PID_Calc(PID_t *p, float t, float a) {
    float e = t - a; p->integral += e;
    if(p->integral>999)p->integral=999; if(p->integral<-999)p->integral=-999;
    float d = e - p->prev_error; p->prev_error = e;
    return p->Kp*e + p->Ki*p->integral + p->Kd*d;
}

int main(void) {
    HAL_Init(); SystemClock_Config();
    GPIO_Init(); TIM_Init(); ADC_Init(); OLED_Init();
    pid = (PID_t){3.0f, 0.5f, 1.0f, 0, 0};
    
    OLED_Clear(); OLED_ShowString(0,0,"Wind Plate Control");
    
    while(1) {
        uint8_t key = Key_Scan();
        if(key==KEY1) { g_target_angle += 5; if(g_target_angle>60) g_target_angle=0; }
        if(key==KEY2) { g_target_angle = 45; }
        
        g_actual_angle = ADC_ReadAngle();  // 读取角度传感器
        float pwm = PID_Calc(&pid, g_target_angle, g_actual_angle);
        if(pwm<0) pwm=0; if(pwm>999) pwm=999;
        TIM3->CCR1 = (uint16_t)pwm;  // 风扇PWM
        
        char buf[32];
        sprintf(buf,"T:%.0f A:%.0f",g_target_angle,g_actual_angle);
        OLED_ShowString(0,2,buf);
        HAL_Delay(20);
    }
}
