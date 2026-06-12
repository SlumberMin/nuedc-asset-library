/**
 * @file    main.c
 * @brief   2017年L题 自动泊车系统 - 主程序
 * 
 * 功能：
 * 1. 键盘设定空车位
 * 2. 自动驶入垂直式/平行式停车位
 * 3. 计时计费功能
 * 4. 碰撞检测
 * 5. 自动驶出停车场
 */

#include "stm32f1xx_hal.h"
#include <stdio.h>

/* 停车位定义 */
typedef struct {
    uint8_t type;       // 0=垂直式，1=平行式
    uint16_t x;         // 停车位X坐标
    uint16_t y;         // 停车位Y坐标
    uint16_t angle;     // 停车位角度
} ParkingSpot_t;

/* 6个停车位 */
static const ParkingSpot_t spots[7] = {
    {0, 0, 0, 0},       // 占位
    {0, 26, 26, 0},     // 01 垂直式
    {0, 26, 64, 0},     // 02 垂直式
    {0, 26, 102, 0},    // 03 垂直式
    {0, 26, 140, 0},    // 04 垂直式
    {1, 60, 120, 90},   // 05 平行式
    {1, 60, 156, 90},   // 06 平行式
};

/* 全局变量 */
volatile uint8_t g_target_spot = 0;     // 目标车位号
volatile uint32_t g_timer_ms = 0;       // 计时(ms)
volatile uint8_t g_fee = 0;             // 停车费(元)
volatile uint8_t g_collision_count = 0; // 碰撞次数
volatile uint8_t g_is_parking = 0;      // 是否在泊车中

/* 系统状态 */
typedef enum {
    STATE_IDLE = 0,
    STATE_NAVIGATING,   // 导航到车位
    STATE_PARKING_IN,   // 泊入
    STATE_WAITING,      // 停车等待
    STATE_PARKING_OUT,  // 泊出
    STATE_DONE          // 完成
} State_t;

volatile State_t g_state = STATE_IDLE;

/* PID控制器 */
typedef struct {
    float Kp, Ki, Kd;
    float integral, prev_error;
} PID_t;

PID_t g_track_pid, g_speed_pid;

/* 函数声明 */
void System_Init(void);
float PID_Calc(PID_t *pid, float target, float actual);
void Motor_SetSpeed(int16_t left, int16_t right);
void Parking_Navigate(void);
void Parking_In_Vertical(void);
void Parking_In_Parallel(void);
void Parking_Out(void);
void Fee_Calculate(void);

/**
 * @brief  主函数
 */
int main(void)
{
    System_Init();
    
    /* PID初始化 */
    g_track_pid = (PID_t){2.0f, 0.3f, 1.0f, 0, 0};
    g_speed_pid = (PID_t){3.0f, 0.5f, 0.5f, 0, 0};
    
    OLED_Clear();
    OLED_ShowString(0, 0, "Auto Parking");
    OLED_ShowString(0, 2, "Spot: --");
    OLED_ShowString(0, 4, "Time: 0s");
    OLED_ShowString(0, 6, "Fee: 0 yuan");
    
    while(1)
    {
        /* 键盘输入车位号 */
        uint8_t key = Key_Scan();
        if(key >= '1' && key <= '6')
        {
            g_target_spot = key - '0';
            g_state = STATE_NAVIGATING;
            g_timer_ms = 0;
            g_collision_count = 0;
            
            /* 点亮对应LED */
            LED_On(g_target_spot);
            
            char buf[16];
            sprintf(buf, "Spot: %02d", g_target_spot);
            OLED_ShowString(0, 2, buf);
        }
        
        /* 状态机 */
        switch(g_state)
        {
            case STATE_IDLE:
                break;
                
            case STATE_NAVIGATING:
                Parking_Navigate();
                break;
                
            case STATE_PARKING_IN:
                if(spots[g_target_spot].type == 0)
                    Parking_In_Vertical();
                else
                    Parking_In_Parallel();
                break;
                
            case STATE_WAITING:
                /* 停车5秒 */
                Alert_Beep(100);
                HAL_Delay(5000);
                g_state = STATE_PARKING_OUT;
                break;
                
            case STATE_PARKING_OUT:
                Parking_Out();
                break;
                
            case STATE_DONE:
                Fee_Calculate();
                Motor_SetSpeed(0, 0);
                g_state = STATE_IDLE;
                break;
        }
        
        /* 计时 */
        if(g_state != STATE_IDLE)
        {
            g_timer_ms++;
            Fee_Calculate();
        }
        
        /* 显示 */
        char buf[32];
        sprintf(buf, "Time: %lus", g_timer_ms / 1000);
        OLED_ShowString(0, 4, buf);
        sprintf(buf, "Fee: %d yuan", g_fee);
        OLED_ShowString(0, 6, buf);
        
        HAL_Delay(1);
    }
}

/**
 * @brief  导航到目标车位
 */
void Parking_Navigate(void)
{
    /* 简化：沿车道行驶到目标车位前方 */
    /* 实际需要根据停车场布局规划路径 */
    
    static uint32_t nav_timer = 0;
    nav_timer++;
    
    /* 循迹行驶 */
    int16_t position = Sensor_GetPosition();
    float pid_out = PID_Calc(&g_track_pid, 0, position);
    
    int16_t base_speed = 300;
    Motor_SetSpeed(base_speed + pid_out, base_speed - pid_out);
    
    /* 检测到达车位（通过编码器里程或视觉） */
    if(nav_timer > 5000)   // 简化：5秒后认为到达
    {
        Motor_SetSpeed(0, 0);
        g_state = STATE_PARKING_IN;
        nav_timer = 0;
    }
}

/**
 * @brief  垂直式泊车
 */
void Parking_In_Vertical(void)
{
    /* 垂直泊车策略：前进→转向→倒车入位 */
    
    /* 第1步：前进到车位旁 */
    Motor_SetSpeed(400, 400);
    HAL_Delay(2000);
    
    /* 第2步：转向 */
    Motor_SetSpeed(500, -200);
    HAL_Delay(1500);
    
    /* 第3步：倒车入位 */
    Motor_SetSpeed(-300, -300);
    HAL_Delay(1500);
    
    /* 第4步：调整 */
    Motor_SetSpeed(200, -200);
    HAL_Delay(500);
    
    Motor_SetSpeed(0, 0);
    g_state = STATE_WAITING;
}

/**
 * @brief  平行式泊车
 */
void Parking_In_Parallel(void)
{
    /* 平行泊车策略：前进→大角度转向→倒车→回正 */
    
    /* 第1步：前进超过车位 */
    Motor_SetSpeed(400, 400);
    HAL_Delay(2500);
    
    /* 第2步：右转倒车 */
    Motor_SetSpeed(-500, -200);
    HAL_Delay(2000);
    
    /* 第3步：左转倒车 */
    Motor_SetSpeed(-200, -500);
    HAL_Delay(1500);
    
    /* 第4步：前进调整 */
    Motor_SetSpeed(300, 300);
    HAL_Delay(500);
    
    Motor_SetSpeed(0, 0);
    g_state = STATE_WAITING;
}

/**
 * @brief  泊出
 */
void Parking_Out(void)
{
    /* 泊出策略：转向→驶出 */
    Motor_SetSpeed(400, -200);
    HAL_Delay(1500);
    Motor_SetSpeed(400, 400);
    HAL_Delay(3000);
    Motor_SetSpeed(0, 0);
    g_state = STATE_DONE;
}

/**
 * @brief  计费计算
 */
void Fee_Calculate(void)
{
    /* 每30秒5元，未满30秒按5元 */
    uint32_t seconds = g_timer_ms / 1000;
    g_fee = ((seconds + 29) / 30) * 5;
}

/**
 * @brief  PID计算
 */
float PID_Calc(PID_t *pid, float target, float actual)
{
    float error = target - actual;
    pid->integral += error;
    float derivative = error - pid->prev_error;
    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
    pid->prev_error = error;
    return output;
}

/**
 * @brief  碰撞检测中断
 */
void EXTI_IRQHandler(void)
{
    /* 碰撞传感器触发 */
    g_collision_count++;
    Alert_Beep(50);
}
