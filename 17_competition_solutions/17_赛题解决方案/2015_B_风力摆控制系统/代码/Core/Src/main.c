/**
 * @file    main.c
 * @brief   2015年B题 风力摆控制系统 - 主程序
 * 
 * 功能：
 * 1. MPU6050检测摆角和角速度
 * 2. 双环PID控制4台风机
 * 3. 实现直线摆动、画圆、制动等功能
 * 4. 激光笔在地面画出轨迹
 */

#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <math.h>

/* 风机通道定义 */
#define FAN_0   0   // 0°方向
#define FAN_90  1   // 90°方向
#define FAN_180 2   // 180°方向
#define FAN_270 3   // 270°方向

/* 运动模式 */
typedef enum {
    MODE_IDLE = 0,
    MODE_LINE,          // 直线摆动
    MODE_LINE_SET_LEN,  // 设定长度直线
    MODE_LINE_SET_DIR,  // 设定方向直线
    MODE_STOP,          // 制动
    MODE_CIRCLE,        // 画圆
    MODE_CIRCLE_RECOVER // 干扰恢复
} Mode_t;

/* 全局变量 */
volatile Mode_t g_mode = MODE_IDLE;
volatile float g_angle_x = 0, g_angle_y = 0;       // 摆角(rad)
volatile float g_omega_x = 0, g_omega_y = 0;       // 角速度(rad/s)
volatile float g_target_angle = 0.5f;               // 目标摆角(rad)
volatile float g_target_direction = 0.0f;           // 目标方向(rad)
volatile float g_line_length = 50.0f;               // 直线长度(cm)
volatile float g_circle_radius = 25.0f;             // 圆半径(cm)
volatile uint32_t g_timer_ms = 0;

/* PID控制器 */
typedef struct {
    float Kp, Ki, Kd;
    float integral, prev_error;
} PID_t;

PID_t pid_angle_x, pid_angle_y;
PID_t pid_omega_x, pid_omega_y;

/* 风机PWM */
volatile uint16_t g_fan_pwm[4] = {0, 0, 0, 0};

/* 函数声明 */
void System_Init(void);
float PID_Calc(PID_t *pid, float target, float actual);
void MPU6050_Read(void);
void Fan_SetPWM(uint8_t ch, uint16_t pwm);
void Control_Loop(void);
void Mode_Process(void);
void ApplyForce(float fx, float fy);

/**
 * @brief  主函数
 */
int main(void)
{
    System_Init();
    
    OLED_Clear();
    OLED_ShowString(0, 0, "Wind Pendulum");
    OLED_ShowString(0, 2, "Mode: IDLE");
    OLED_ShowString(0, 4, "Angle: 0.0");
    OLED_ShowString(0, 6, "Ready...");
    
    while(1)
    {
        /* 按键处理 */
        uint8_t key = Key_Scan();
        switch(key)
        {
            case KEY1: g_mode = MODE_LINE; g_timer_ms = 0; break;
            case KEY2: g_mode = MODE_LINE_SET_LEN; g_timer_ms = 0; break;
            case KEY3: g_mode = MODE_LINE_SET_DIR; g_timer_ms = 0; break;
            case KEY4: g_mode = MODE_STOP; break;
            case KEY5: g_mode = MODE_CIRCLE; g_timer_ms = 0; break;
        }
        
        /* 模式处理 */
        Mode_Process();
        
        /* 控制循环 */
        Control_Loop();
        
        /* 显示 */
        char buf[32];
        sprintf(buf, "Ang: %.1f", g_angle_x * 57.3f);
        OLED_ShowString(0, 4, buf);
        
        HAL_Delay(5);   // 5ms控制周期(200Hz)
    }
}

/**
 * @brief  模式处理
 */
void Mode_Process(void)
{
    switch(g_mode)
    {
        case MODE_IDLE:
            ApplyForce(0, 0);  // 停止所有风机
            break;
            
        case MODE_LINE:
            /* 直线摆动：沿X轴方向摆动 */
            /* 目标：正弦摆动，幅度g_target_angle */
            {
                float target = g_target_angle * sinf(g_timer_ms * 0.003f);
                float fx = PID_Calc(&pid_angle_x, target, g_angle_x);
                ApplyForce(fx, 0);
            }
            if(g_timer_ms > 15000) g_mode = MODE_IDLE;
            break;
            
        case MODE_LINE_SET_LEN:
            /* 设定长度直线摆动 */
            {
                float amplitude = g_line_length / 200.0f;  // 长度映射到角度
                float target = amplitude * sinf(g_timer_ms * 0.003f);
                float fx = PID_Calc(&pid_angle_x, target, g_angle_x);
                ApplyForce(fx, 0);
            }
            if(g_timer_ms > 15000) g_mode = MODE_IDLE;
            break;
            
        case MODE_LINE_SET_DIR:
            /* 设定方向直线摆动 */
            {
                float target_x = g_target_angle * cosf(g_target_direction) * sinf(g_timer_ms * 0.003f);
                float target_y = g_target_angle * sinf(g_target_direction) * sinf(g_timer_ms * 0.003f);
                float fx = PID_Calc(&pid_angle_x, target_x, g_angle_x);
                float fy = PID_Calc(&pid_angle_y, target_y, g_angle_y);
                ApplyForce(fx, fy);
            }
            if(g_timer_ms > 15000) g_mode = MODE_IDLE;
            break;
            
        case MODE_STOP:
            /* 制动：施加反向力 */
            {
                float fx = -PID_Calc(&pid_omega_x, 0, g_omega_x);
                float fy = -PID_Calc(&pid_omega_y, 0, g_omega_y);
                ApplyForce(fx, fy);
                
                /* 检查是否静止 */
                if(fabsf(g_angle_x) < 0.02f && fabsf(g_angle_y) < 0.02f)
                {
                    ApplyForce(0, 0);
                    g_mode = MODE_IDLE;
                }
            }
            if(g_timer_ms > 5000) g_mode = MODE_IDLE;
            break;
            
        case MODE_CIRCLE:
            /* 画圆运动 */
            {
                float theta = g_timer_ms * 0.002f;  // 画圆角速度
                float target_x = g_circle_radius / 100.0f * cosf(theta);
                float target_y = g_circle_radius / 100.0f * sinf(theta);
                
                float fx = PID_Calc(&pid_angle_x, target_x, g_angle_x);
                float fy = PID_Calc(&pid_angle_y, target_y, g_angle_y);
                ApplyForce(fx, fy);
            }
            if(g_timer_ms > 30000) g_mode = MODE_IDLE;
            break;
    }
}

/**
 * @brief  控制主循环
 */
void Control_Loop(void)
{
    /* 读取传感器 */
    MPU6050_Read();
    
    /* 更新计时 */
    g_timer_ms += 5;
}

/**
 * @brief  施加力（4风机控制）
 * @param  fx: X方向力(-1~+1)
 * @param  fy: Y方向力(-1~+1)
 */
void ApplyForce(float fx, float fy)
{
    /* 4风机十字布局：
     *     风机0(0°)
     *      │
     * 风机3(270°)──┼──风机1(90°)
     *      │
     *     风机2(180°)
     */
    
    /* 分解到各风机 */
    int16_t pwm_0  = (int16_t)(fx * 999);          // X正方向
    int16_t pwm_90 = (int16_t)(fy * 999);          // Y正方向
    int16_t pwm_180 = -(int16_t)(fx * 999);        // X负方向
    int16_t pwm_270 = -(int16_t)(fy * 999);        // Y负方向
    
    /* 限幅 */
    if(pwm_0 < 0) pwm_0 = 0;
    if(pwm_90 < 0) pwm_90 = 0;
    if(pwm_180 < 0) pwm_180 = 0;
    if(pwm_270 < 0) pwm_270 = 0;
    if(pwm_0 > 999) pwm_0 = 999;
    if(pwm_90 > 999) pwm_90 = 999;
    if(pwm_180 > 999) pwm_180 = 999;
    if(pwm_270 > 999) pwm_270 = 999;
    
    Fan_SetPWM(FAN_0, pwm_0);
    Fan_SetPWM(FAN_90, pwm_90);
    Fan_SetPWM(FAN_180, pwm_180);
    Fan_SetPWM(FAN_270, pwm_270);
}

/**
 * @brief  PID计算
 */
float PID_Calc(PID_t *pid, float target, float actual)
{
    float error = target - actual;
    pid->integral += error;
    if(pid->integral > 1.0f) pid->integral = 1.0f;
    if(pid->integral < -1.0f) pid->integral = -1.0f;
    
    float derivative = error - pid->prev_error;
    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
    
    if(output > 1.0f) output = 1.0f;
    if(output < -1.0f) output = -1.0f;
    
    pid->prev_error = error;
    return output;
}
