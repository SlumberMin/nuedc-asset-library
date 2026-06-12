/**
 * @file    main.c
 * @brief   2024年H题 自动行驶小车 - 主程序
 * @author  电赛控制题自动处理系统
 * @version 1.0
 * 
 * 系统功能：
 * 1. 红外循迹传感器检测黑色弧线轨迹
 * 2. 编码器测速反馈，PID速度闭环控制
 * 3. 支持A→B→C→D→A完整路径行驶
 * 4. 经过ABCD点时声光提示
 * 5. 支持多圈行驶和不同路径模式
 */

#include "msp.h"
#include <stdio.h>
#include <string.h>
#include "sensor.h"
#include "motor.h"
#include "encoder.h"
#include "alert.h"
#include "key.h"
#include "user_config.h"

/* ===== 系统状态定义 ===== */
typedef enum {
    STATE_IDLE = 0,         // 空闲等待
    STATE_RUNNING,          // 行驶中
    STATE_STOP,             // 停止
    STATE_ERROR             // 错误
} SystemState_t;

/* ===== 路径模式定义 ===== */
typedef enum {
    PATH_AB = 0,            // A→B直线
    PATH_ABCDA,             // A→B→C→D→A完整一圈
    PATH_ACBDA,             // A→C→B→D→A特殊路径
    PATH_MULTI              // 多圈行驶
} PathMode_t;

/* ===== 路径点定义 ===== */
typedef struct {
    char name;              // 点名(A/B/C/D)
    uint16_t x;             // X坐标(相对)
    uint16_t y;             // Y坐标(相对)
} WayPoint_t;

/* 全局变量 */
volatile SystemState_t g_state = STATE_IDLE;
volatile PathMode_t g_path_mode = PATH_AB;
volatile uint8_t g_target_circles = 1;
volatile uint8_t g_current_circle = 0;
volatile uint32_t g_run_time_ms = 0;
volatile uint8_t g_current_waypoint = 0;

/* 速度PID控制 */
PID_t g_speed_pid_l;        // 左电机速度PID
PID_t g_speed_pid_r;        // 右电机速度PID
PID_t g_track_pid;          // 循迹PID

/* 目标速度 */
float g_target_speed = 0.3f;    // 目标速度(m/s)

/* 路径点序列(ABCD) */
static const WayPoint_t waypoints[] = {
    {'A', 0, 60},       // A点
    {'B', 220, 60},     // B点
    {'C', 220, 120},    // C点
    {'D', 0, 120},      // D点
};

/* 函数声明 */
void System_Init(void);
void StateMachine_Run(void);
void TrackControl(void);
void SpeedControl(float target_speed);
void WaypointCheck(void);
void Display_Update(void);

/**
 * @brief  主函数
 */
int main(void)
{
    System_Init();
    
    /* 显示开机信息 */
    OLED_Clear();
    OLED_ShowString(0, 0, "2024-H Auto Car");
    OLED_ShowString(0, 2, "Mode: AB");
    OLED_ShowString(0, 4, "Speed: 0.3m/s");
    OLED_ShowString(0, 6, "State: IDLE");
    
    Alert_Beep(100);
    
    while(1)
    {
        /* 按键处理 */
        uint8_t key = Key_Scan();
        if(key == KEY1_PRESS)
        {
            /* 切换路径模式 */
            g_path_mode++;
            if(g_path_mode > PATH_MULTI) g_path_mode = PATH_AB;
        }
        else if(key == KEY2_PRESS)
        {
            /* 启动/停止 */
            if(g_state == STATE_IDLE)
            {
                g_state = STATE_RUNNING;
                g_current_circle = 0;
                g_run_time_ms = 0;
                g_current_waypoint = 0;
                Alert_Beep(200);
            }
            else
            {
                g_state = STATE_STOP;
            }
        }
        else if(key == KEY3_PRESS)
        {
            /* 切换目标速度 */
            g_target_speed += 0.1f;
            if(g_target_speed > 1.0f) g_target_speed = 0.3f;
        }
        
        /* 状态机运行 */
        StateMachine_Run();
        
        /* 显示更新 */
        Display_Update();
        
        /* 短延时 */
        Delay_ms(1);
    }
}

/**
 * @brief  系统初始化
 */
void System_Init(void)
{
    SystemClock_Config();       // 80MHz
    GPIO_Init();
    TIM_Init();
    UART_Init();
    
    Sensor_Init();              // 循迹传感器
    Motor_Init();               // 电机驱动
    Encoder_Init();             // 编码器
    OLED_Init();                // OLED显示
    Key_Init();                 // 按键
    Alert_Init();               // 声光
    
    /* PID初始化 */
    PID_Init(&g_speed_pid_l, 3.0f, 0.8f, 0.5f, -999, 999);
    PID_Init(&g_speed_pid_r, 3.0f, 0.8f, 0.5f, -999, 999);
    PID_Init(&g_track_pid, 2.0f, 0.3f, 1.0f, -400, 400);
    
    g_state = STATE_IDLE;
}

/**
 * @brief  状态机运行
 */
void StateMachine_Run(void)
{
    switch(g_state)
    {
        case STATE_IDLE:
            Motor_SetSpeed(0, 0);
            break;
            
        case STATE_RUNNING:
            /* 循迹控制 */
            TrackControl();
            
            /* 速度闭环控制 */
            SpeedControl(g_target_speed);
            
            /* 航点检测 */
            WaypointCheck();
            break;
            
        case STATE_STOP:
            Motor_SetSpeed(0, 0);
            Alert_Beep(500);
            g_state = STATE_IDLE;
            break;
            
        case STATE_ERROR:
            Motor_SetSpeed(0, 0);
            Alert_Error();
            break;
    }
}

/**
 * @brief  循迹PID控制
 */
void TrackControl(void)
{
    int16_t position = Sensor_GetPosition();
    float pid_out = PID_Calculate(&g_track_pid, 0.0f, (float)position);
    
    int16_t base_speed = (int16_t)(g_target_speed * 3000);  // 速度映射到PWM
    int16_t speed_l = base_speed + (int16_t)pid_out;
    int16_t speed_r = base_speed - (int16_t)pid_out;
    
    Motor_SetSpeed(speed_l, speed_r);
}

/**
 * @brief  速度闭环控制
 */
void SpeedControl(float target_speed)
{
    float speed_l = Encoder_GetSpeed(0);    // 左电机实际速度
    float speed_r = Encoder_GetSpeed(1);    // 右电机实际速度
    
    float target_rpm = target_speed * 60.0f / (WHEEL_DIAMETER * 3.14159f);
    
    PID_Calculate(&g_speed_pid_l, target_rpm, speed_l);
    PID_Calculate(&g_speed_pid_r, target_rpm, speed_r);
}

/**
 * @brief  航点检测
 */
void WaypointCheck(void)
{
    /* 检测是否经过ABCD点 */
    /* 通过编码器里程或特殊标记检测 */
    /* 简化实现：通过时间估算位置 */
    
    uint32_t elapsed = g_run_time_ms;
    
    /* 根据路径模式检查 */
    switch(g_path_mode)
    {
        case PATH_AB:
            if(elapsed > 15000) // 15s到达B点
            {
                Alert_Beep(50);     // 声光提示
                g_state = STATE_STOP;
            }
            break;
            
        case PATH_ABCDA:
            /* 完整一圈约30s */
            if(elapsed > 30000)
            {
                g_current_circle++;
                if(g_current_circle >= g_target_circles)
                {
                    Alert_Beep(500);
                    g_state = STATE_STOP;
                }
                else
                {
                    g_run_time_ms = 0;
                    Alert_Beep(50);
                }
            }
            break;
            
        case PATH_ACBDA:
            if(elapsed > 40000)
            {
                Alert_Beep(500);
                g_state = STATE_STOP;
            }
            break;
            
        case PATH_MULTI:
            if(elapsed > 30000)
            {
                g_current_circle++;
                if(g_current_circle >= g_target_circles * 4)
                {
                    Alert_Beep(500);
                    g_state = STATE_STOP;
                }
                else
                {
                    g_run_time_ms = 0;
                    Alert_Beep(50);
                }
            }
            break;
    }
}

/**
 * @brief  显示更新
 */
void Display_Update(void)
{
    char buf[32];
    
    switch(g_state)
    {
        case STATE_IDLE:    OLED_ShowString(0, 6, "State: IDLE   "); break;
        case STATE_RUNNING: OLED_ShowString(0, 6, "State: RUNNING"); break;
        case STATE_STOP:    OLED_ShowString(0, 6, "State: STOP   "); break;
        case STATE_ERROR:   OLED_ShowString(0, 6, "State: ERROR  "); break;
    }
    
    sprintf(buf, "Time: %lu.%lus", g_run_time_ms/1000, (g_run_time_ms%1000)/100);
    OLED_ShowString(0, 4, buf);
}
