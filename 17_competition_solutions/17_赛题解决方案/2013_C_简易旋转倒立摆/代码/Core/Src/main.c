/**
 * @file    main.c
 * @brief   2013年C题 简易旋转倒立摆 - 主程序
 * 
 * 系统结构：电机驱动旋转臂，摆杆通过转轴连接在旋转臂边缘
 * 控制目标：使摆杆从下垂状态摆起并保持倒立
 * 
 * 功能：
 * 1. 摆杆摆动（±60°以上）
 * 2. 完成圆周运动
 * 3. 倒立保持（≥5s）
 * 4. 自动摆起倒立（≥10s）
 * 5. 抗干扰恢复
 * 6. 旋转臂圆周运动（倒立状态下≥360°）
 */

#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <math.h>

/* 系统参数 */
#define ARM_LENGTH      0.20f   // 旋转臂长度(m)
#define PEND_LENGTH     0.15f   // 摆杆长度(m)
#define G               9.8f    // 重力加速度

/* 摆杆状态 */
typedef struct {
    float theta;        // 摆杆角度(rad)，0=下垂，π=倒立
    float omega;        // 摆杆角速度(rad/s)
    float alpha;        // 旋转臂角度(rad)
    float alpha_dot;    // 旋转臂角速度(rad/s)
} State_t;

/* 运动模式 */
typedef enum {
    MODE_IDLE = 0,
    MODE_SWING_UP,      // 摆起
    MODE_SWING_CIRCLE,  // 圆周运动
    MODE_BALANCE,       // 倒立平衡
    MODE_RECOVER,       // 干扰恢复
    MODE_ARM_CIRCLE     // 旋转臂圆周运动
} Mode_t;

/* 全局变量 */
volatile Mode_t g_mode = MODE_IDLE;
volatile State_t g_state = {0, 0, 0, 0};
volatile uint32_t g_timer_ms = 0;
volatile uint32_t g_balance_time = 0;  // 倒立保持时间(ms)

/* 能量控制参数 */
volatile float g_energy_target = 0;    // 目标能量
volatile float g_energy_kp = 0.5f;     // 能量增益

/* 函数声明 */
void System_Init(void);
void Encoder_Read(void);
void Motor_SetPWM(int16_t pwm);
float Energy_Get(void);
void SwingUp_Control(void);
void Balance_Control(void);
void Circle_Control(void);

/**
 * @brief  主函数
 */
int main(void)
{
    System_Init();
    
    OLED_Clear();
    OLED_ShowString(0, 0, "Rotary Pendulum");
    OLED_ShowString(0, 2, "Theta: 0.0");
    OLED_ShowString(0, 4, "Mode: IDLE");
    OLED_ShowString(0, 6, "Time: 0.0s");
    
    while(1)
    {
        /* 按键处理 */
        uint8_t key = Key_Scan();
        switch(key)
        {
            case KEY1: g_mode = MODE_SWING_UP; g_timer_ms = 0; break;
            case KEY2: g_mode = MODE_SWING_CIRCLE; g_timer_ms = 0; break;
            case KEY3: g_mode = MODE_BALANCE; g_timer_ms = 0; g_balance_time = 0; break;
            case KEY4: g_mode = MODE_ARM_CIRCLE; g_timer_ms = 0; break;
        }
        
        /* 读取编码器 */
        Encoder_Read();
        
        /* 模式处理 */
        switch(g_mode)
        {
            case MODE_IDLE:
                Motor_SetPWM(0);
                break;
                
            case MODE_SWING_UP:
                SwingUp_Control();
                g_timer_ms += 5;
                if(g_timer_ms > 30000) g_mode = MODE_IDLE; // 30s超时
                break;
                
            case MODE_SWING_CIRCLE:
                Circle_Control();
                g_timer_ms += 5;
                if(g_timer_ms > 30000) g_mode = MODE_IDLE;
                break;
                
            case MODE_BALANCE:
                Balance_Control();
                g_timer_ms += 5;
                
                /* 检查是否保持倒立 */
                if(fabsf(g_state.theta - 3.14159f) < 0.2f)  // ±11.5°内
                {
                    g_balance_time += 5;
                }
                else
                {
                    g_balance_time = 0;
                }
                
                if(g_timer_ms > 90000) g_mode = MODE_IDLE; // 90s超时
                break;
                
            case MODE_RECOVER:
                Balance_Control();
                if(fabsf(g_state.theta - 3.14159f) < 0.1f)
                {
                    g_balance_time += 5;
                    if(g_balance_time > 2000) g_mode = MODE_BALANCE;
                }
                break;
                
            case MODE_ARM_CIRCLE:
                Balance_Control();  // 保持倒立
                /* 同时驱动旋转臂做圆周运动 */
                static float arm_angle = 0;
                arm_angle += 0.05f;
                if(arm_angle > 6.28f) arm_angle -= 6.28f;
                
                if(g_timer_ms > 180000) g_mode = MODE_IDLE; // 3min超时
                g_timer_ms += 5;
                break;
        }
        
        /* 显示 */
        char buf[32];
        sprintf(buf, "Th: %.1f", g_state.theta * 57.3f);
        OLED_ShowString(0, 2, buf);
        sprintf(buf, "Bal: %.1fs", g_balance_time / 1000.0f);
        OLED_ShowString(0, 6, buf);
        
        HAL_Delay(5);   // 5ms控制周期
    }
}

/**
 * @brief  摆起控制（能量注入法）
 * 
 * 原理：通过旋转臂的往复运动，逐渐增加摆杆的能量
 * 当摆杆能量达到倒立所需能量时，切换到平衡控制
 */
void SwingUp_Control(void)
{
    /* 1. 计算摆杆能量 */
    float energy = Energy_Get();
    float energy_up = G * PEND_LENGTH;  // 倒立所需能量
    
    /* 2. 能量误差 */
    float energy_error = energy_up - energy;
    
    /* 3. 能量注入控制律 */
    /* 当摆杆向下摆时，旋转臂同方向加速 */
    /* 当摆杆向上摆时，旋转臂反方向减速 */
    float sign = (g_state.omega > 0) ? 1.0f : -1.0f;
    float pwm = g_energy_kp * energy_error * sign * g_state.omega;
    
    /* 4. PWM限幅 */
    if(pwm > 999) pwm = 999;
    if(pwm < -999) pwm = -999;
    
    Motor_SetPWM((int16_t)pwm);
    
    /* 5. 检查是否可以切换到平衡模式 */
    if(fabsf(g_state.theta - 3.14159f) < 0.3f && fabsf(g_state.omega) < 2.0f)
    {
        g_mode = MODE_BALANCE;
        g_balance_time = 0;
    }
}

/**
 * @brief  倒立平衡控制（LQR/极点配置）
 * 
 * 状态变量：x = [theta_error, omega, alpha, alpha_dot]^T
 * 控制输出：u = 电机PWM
 */
void Balance_Control(void)
{
    /* 状态误差 */
    float theta_err = g_state.theta - 3.14159f;  // 倒立点误差
    float omega = g_state.omega;
    float alpha = g_state.alpha;
    float alpha_dot = g_state.alpha_dot;
    
    /* LQR增益（离线计算） */
    float K1 = -30.0f;  // 角度增益
    float K2 = -5.0f;   // 角速度增益
    float K3 = -2.0f;   // 旋转臂角度增益
    float K4 = -1.0f;   // 旋转臂角速度增益
    
    /* 控制律 */
    float pwm = K1 * theta_err + K2 * omega + K3 * alpha + K4 * alpha_dot;
    
    /* PWM限幅 */
    if(pwm > 999) pwm = 999;
    if(pwm < -999) pwm = -999;
    
    Motor_SetPWM((int16_t)pwm);
}

/**
 * @brief  计算摆杆能量
 * @retval float: 摆杆能量(J)
 */
float Energy_Get(void)
{
    /* 动能 + 势能 */
    float v = PEND_LENGTH * g_state.omega;
    float KE = 0.5f * v * v;                    // 动能
    float PE = G * PEND_LENGTH * (1.0f - cosf(g_state.theta));  // 势能(以倒立点为参考)
    return KE + PE;
}

/**
 * @brief  圆周运动控制
 */
void Circle_Control(void)
{
    /* 类似摆起控制，但目标能量更高（完成圆周运动） */
    float energy = Energy_Get();
    float energy_circle = 3.0f * G * PEND_LENGTH;  // 圆周运动所需能量
    
    float energy_error = energy_circle - energy;
    float sign = (g_state.omega > 0) ? 1.0f : -1.0f;
    float pwm = g_energy_kp * 1.5f * energy_error * sign * g_state.omega;
    
    if(pwm > 999) pwm = 999;
    if(pwm < -999) pwm = -999;
    
    Motor_SetPWM((int16_t)pwm);
}
