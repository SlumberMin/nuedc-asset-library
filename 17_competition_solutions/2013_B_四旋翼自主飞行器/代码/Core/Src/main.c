/**
 * @file    main.c
 * @brief   2013_B_四旋翼自主飞行器 - 主程序
 * @author  电赛控制题自动处理系统
 * 
 * 四旋翼飞行器控制系统
 * 传感器：MPU6050(姿态)+气压计(高度)+GPS(位置)
 * 执行器：4个无刷电机(ESC控制)
 * 算法：PID姿态控制+位置控制
 */

#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <math.h>

typedef struct { float Kp, Ki, Kd, integral, prev_error; } PID_t;

PID_t pid_roll, pid_pitch, pid_yaw, pid_alt;
volatile float roll=0, pitch=0, yaw=0, altitude=0;
volatile float target_alt = 1.0f;  // 目标高度(m)
volatile uint16_t motor[4] = {0,0,0,0};

float PID_Calc(PID_t *p, float t, float a) {
    float e = t - a; p->integral += e;
    if(p->integral>400)p->integral=400; if(p->integral<-400)p->integral=-400;
    float d = e - p->prev_error; p->prev_error = e;
    float o = p->Kp*e + p->Ki*p->integral + p->Kd*d;
    if(o>400)o=400; if(o<-400)o=-400; return o;
}

int main(void) {
    HAL_Init(); SystemClock_Config();
    GPIO_Init(); TIM_Init(); UART_Init();
    MPU6050_Init(); BMP280_Init(); Motor_Init();
    
    pid_roll = (PID_t){1.5f, 0.1f, 0.5f, 0, 0};
    pid_pitch = (PID_t){1.5f, 0.1f, 0.5f, 0, 0};
    pid_yaw = (PID_t){2.0f, 0.05f, 0.3f, 0, 0};
    pid_alt = (PID_t){3.0f, 0.5f, 1.0f, 0, 0};
    
    OLED_Clear();
    OLED_ShowString(0, 0, "2013_B_四旋翼自主飞行器");
    
    while(1) {
        MPU6050_GetEuler(&roll, &pitch, &yaw);
        altitude = BMP280_GetAltitude();
        
        float r = PID_Calc(&pid_roll, 0, roll);
        float p = PID_Calc(&pid_pitch, 0, pitch);
        float y = PID_Calc(&pid_yaw, 0, yaw);
        float a = PID_Calc(&pid_alt, target_alt, altitude);
        
        uint16_t base = 500 + (uint16_t)a;
        motor[0] = base + r + p + y;
        motor[1] = base - r + p - y;
        motor[2] = base - r - p + y;
        motor[3] = base + r - p - y;
        
        for(int i=0;i<4;i++) Motor_SetPWM(i+1, motor[i]);
        
        char buf[32];
        sprintf(buf,"R:%.0f P:%.0f A:%.1f",roll,pitch,altitude);
        OLED_ShowString(0, 2, buf);
        HAL_Delay(5);
    }
}
