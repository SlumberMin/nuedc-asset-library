/**
 * @file    main.c
 * @brief   2019年H题 模拟电磁曲射炮 - 主程序
 * 
 * 功能：
 * 1. 电磁炮发射弹丸
 * 2. 控制发射角度（舵机）
 * 3. 调节充电电压（控制射程）
 * 4. 自动瞄准目标
 * 
 * 系统组成：电磁线圈+高压电容+角度舵机+距离传感器
 */

#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <math.h>

/* 系统参数 */
#define CHARGE_VOLTAGE_MAX  300     // 最大充电电压(V)
#define ANGLE_MIN           20      // 最小发射角度(°)
#define ANGLE_MAX           70      // 最大发射角度(°)

/* 全局变量 */
volatile float g_target_distance = 150;    // 目标距离(cm)
volatile float g_launch_angle = 45;        // 发射角度(°)
volatile uint16_t g_charge_voltage = 200;  // 充电电压(V)
volatile uint8_t g_ready = 0;              // 充电就绪标志

typedef enum {
    MODE_IDLE = 0,
    MODE_AIM,           // 瞄准
    MODE_CHARGE,        // 充电
    MODE_FIRE,          // 发射
    MODE_AUTO           // 自动模式
} Mode_t;

volatile Mode_t g_mode = MODE_IDLE;

/* 函数声明 */
void System_Init(void);
void Charge_SetVoltage(uint16_t voltage);
void Angle_Set(float angle);
void Fire(void);
float Distance_Measure(void);
float Calc_Angle(float distance, float voltage);

int main(void)
{
    System_Init();
    
    OLED_Clear();
    OLED_ShowString(0, 0, "EM Cannon v1.0");
    OLED_ShowString(0, 2, "Dist: 150cm");
    OLED_ShowString(0, 4, "Angle: 45.0");
    OLED_ShowString(0, 6, "Volt: 200V");
    
    while(1)
    {
        /* 按键处理 */
        uint8_t key = Key_Scan();
        switch(key)
        {
            case KEY1: g_mode = MODE_AIM; break;
            case KEY2: g_mode = MODE_CHARGE; break;
            case KEY3: g_mode = MODE_FIRE; break;
            case KEY4: g_mode = MODE_AUTO; break;
        }
        
        /* 状态机 */
        switch(g_mode)
        {
            case MODE_IDLE:
                break;
                
            case MODE_AIM:
                /* 测距 */
                g_target_distance = Distance_Measure();
                /* 计算最优角度和电压 */
                g_launch_angle = Calc_Angle(g_target_distance, g_charge_voltage);
                Angle_Set(g_launch_angle);
                g_mode = MODE_IDLE;
                break;
                
            case MODE_CHARGE:
                /* 高压电容充电 */
                Charge_SetVoltage(g_charge_voltage);
                /* 等待充电完成 */
                while(!g_ready) { HAL_Delay(100); }
                Alert_Beep(200);
                g_mode = MODE_IDLE;
                break;
                
            case MODE_FIRE:
                /* 发射！ */
                if(g_ready)
                {
                    Fire();
                    g_ready = 0;
                    Alert_Beep(500);
                }
                g_mode = MODE_IDLE;
                break;
                
            case MODE_AUTO:
                /* 自动瞄准+充电+发射 */
                g_target_distance = Distance_Measure();
                g_launch_angle = Calc_Angle(g_target_distance, g_charge_voltage);
                Angle_Set(g_launch_angle);
                Charge_SetVoltage(g_charge_voltage);
                HAL_Delay(2000);    // 等待充电
                Fire();
                Alert_Beep(500);
                g_mode = MODE_IDLE;
                break;
        }
        
        /* 显示 */
        char buf[32];
        sprintf(buf, "Dist: %.0fcm", g_target_distance);
        OLED_ShowString(0, 2, buf);
        sprintf(buf, "Angle: %.1f", g_launch_angle);
        OLED_ShowString(0, 4, buf);
        
        HAL_Delay(100);
    }
}

/**
 * @brief  计算最优发射角度
 * @param  distance: 目标距离(cm)
 * @param  voltage: 充电电压(V)
 * @retval float: 发射角度(°)
 * 
 * 抛物线运动方程：
 * x = v0*cos(θ)*t
 * y = v0*sin(θ)*t - 0.5*g*t²
 * 消去t得：R = v0²*sin(2θ)/g
 * 
 * v0与充电电压的关系需要标定
 */
float Calc_Angle(float distance, float voltage)
{
    /* 简化计算：假设v0与voltage成正比 */
    float v0 = voltage * 0.1f;  // m/s，需要标定
    float g = 9.8f;
    float R = distance / 100.0f;  // 转换为m
    
    /* θ = 0.5 * arcsin(g*R/v0²) */
    float sin2theta = g * R / (v0 * v0);
    if(sin2theta > 1.0f) sin2theta = 1.0f;
    if(sin2theta < -1.0f) sin2theta = -1.0f;
    
    float angle = 0.5f * asinf(sin2theta) * 57.3f;  // 转换为度
    
    /* 角度限幅 */
    if(angle < ANGLE_MIN) angle = ANGLE_MIN;
    if(angle > ANGLE_MAX) angle = ANGLE_MAX;
    
    return angle;
}

void Charge_SetVoltage(uint16_t v) { /* PWM控制升压模块 */ }
void Angle_Set(float a) { /* 舵机设置角度 */ }
void Fire(void) { /* 触发可控硅导通，放电发射 */ }
float Distance_Measure(void) { /* 超声波测距 */ return 150.0f; }
