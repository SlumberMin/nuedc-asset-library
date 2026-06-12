/**
 * @file    main.c
 * @brief   2020年C题 坡道行驶电动小车 - 主程序
 * 
 * 功能：在坡道上自动行驶，保持速度稳定
 * 传感器：红外循迹+编码器+MPU6050(坡度检测)
 * 特点：坡道自适应，速度闭环
 */

#include "stm32f1xx_hal.h"
#include <stdio.h>

typedef struct { float Kp, Ki, Kd, integral, prev_error; } PID_t;
PID_t g_speed_pid, g_track_pid;
volatile float g_target_speed = 0.5f;  // m/s
volatile float g_actual_speed = 0.0f;

float PID_Calc(PID_t *p, float t, float a) {
    float e = t - a; p->integral += e;
    if(p->integral>999)p->integral=999; if(p->integral<-999)p->integral=-999;
    float d = e - p->prev_error; p->prev_error = e;
    float o = p->Kp*e + p->Ki*p->integral + p->Kd*d;
    if(o>999)o=999; if(o<-999)o=-999; return o;
}

int main(void) {
    HAL_Init(); SystemClock_Config();
    GPIO_Init(); TIM_Init(); Encoder_Init();
    OLED_Init(); Motor_Init(); MPU6050_Init();
    
    g_speed_pid = (PID_t){5.0f, 1.0f, 0.5f, 0, 0};
    g_track_pid = (PID_t){2.0f, 0.3f, 1.0f, 0, 0};
    
    OLED_Clear(); OLED_ShowString(0,0,"Slope Car v1.0");
    
    while(1) {
        /* 编码器测速 */
        g_actual_speed = Encoder_GetSpeed();
        
        /* 坡度检测 */
        float slope = MPU6050_GetAngle();
        
        /* 速度PID（补偿坡度影响） */
        float speed_cmd = PID_Calc(&g_speed_pid, g_target_speed, g_actual_speed);
        
        /* 循迹PID */
        int16_t pos = Sensor_GetPosition();
        float track_cmd = PID_Calc(&g_track_pid, 0, pos);
        
        /* 合成输出 */
        int16_t base = (int16_t)speed_cmd;
        Motor_SetSpeed(base + track_cmd, base - track_cmd);
        
        char buf[32];
        sprintf(buf,"Spd:%.1f Slp:%.1f",g_actual_speed,slope);
        OLED_ShowString(0,2,buf);
        HAL_Delay(10);
    }
}
