/**
 * @file    main.c
 * @brief   2025年E题 简易自行瞄准装置 - 主程序
 * @author  电赛控制题自动处理系统
 * @version 1.0
 * 
 * 系统功能：
 * 1. 红外循迹传感器阵列检测黑线轨迹，PID差速控制小车行驶
 * 2. OpenMV视觉模块识别靶心坐标，通过UART传送给MSPM0
 * 3. 二维舵机云台PID闭环控制激光笔瞄准靶心
 * 4. 支持行驶中连续瞄准、画圆等高级功能
 * 5. OLED显示系统状态，按键设定圈数和模式
 */

#include "msp.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* ===== 模块头文件 ===== */
#include "tracking.h"       // 循迹模块
#include "motor.h"          // 电机驱动模块
#include "gimbal.h"         // 二维云台模块
#include "vision.h"         // OpenMV视觉通信模块
#include "pid.h"            // PID控制算法
#include "oled.h"           // OLED显示模块
#include "key.h"            // 按键模块
#include "alert.h"          // 声光提示模块
#include "timer.h"          // 定时器模块

/* ===== 系统参数宏定义 ===== */
#define SYS_TICK_MS         1       // 系统节拍周期(ms)
#define TRACK_LINE_NUM      8      // 循迹传感器数量
#define MOTOR_PWM_MAX       999    // 电机PWM最大值
#define MOTOR_PWM_BASE      400    // 电机基础速度PWM
#define GIMBAL_CENTER_H     90     // 水平舵机中心角度
#define GIMBAL_CENTER_V     90     // 垂直舵机中心角度
#define LASER_ON            1
#define LASER_OFF           0

/* ===== 系统状态定义 ===== */
typedef enum {
    STATE_IDLE = 0,         // 空闲状态，等待按键启动
    STATE_TRACKING,         // 循迹行驶状态
    STATE_AIMING,           // 瞄准状态（静止瞄准）
    STATE_TRACK_AIM,        // 行驶中瞄准状态
    STATE_DRAW_CIRCLE,      // 画圆状态
    STATE_STOP,             // 停止状态
    STATE_ERROR             // 错误状态
} SystemState_t;

/* ===== 瞄准模式定义 ===== */
typedef enum {
    AIM_MODE_STATIC = 0,    // 静态瞄准（小车静止）
    AIM_MODE_DYNAMIC,       // 动态瞄准（行驶中瞄准）
    AIM_MODE_CIRCLE         // 画圆模式
} AimMode_t;

/* ===== 全局变量 ===== */
volatile SystemState_t g_sys_state = STATE_IDLE;    // 系统当前状态
volatile AimMode_t g_aim_mode = AIM_MODE_STATIC;    // 瞄准模式
volatile uint8_t g_target_circles = 1;              // 目标圈数(1-5)
volatile uint8_t g_current_circle = 0;              // 当前圈数
volatile uint32_t g_run_time_ms = 0;                // 运行计时(ms)
volatile uint8_t g_laser_state = LASER_OFF;         // 激光笔状态
volatile uint8_t g_cross_count = 0;                 // 跨线计数

/* PID控制器实例 */
PID_Controller_t g_track_pid;       // 循迹PID
PID_Controller_t g_gimbal_pid_h;    // 云台水平PID
PID_Controller_t g_gimbal_pid_v;    // 云台垂直PID

/* 视觉数据 */
VisionData_t g_vision_data;         // OpenMV传回的靶心坐标

/* ===== 函数声明 ===== */
void System_Init(void);
void GPIO_Init(void);
void TIM_Init(void);
void UART_Init(void);
void SystemClock_Config(void);
void StateMachine_Run(void);
void TrackPID_Control(void);
void GimbalAim_Control(void);
void CircleDraw_Control(void);
void Display_Update(void);
void Key_Process(void);

/* ============================================================
 * 函数名：main
 * 功能：主函数，系统入口
 * 参数：无
 * 返回：int（永不返回）
 * ============================================================ */
int main(void)
{
    /* 第一步：系统初始化 */
    System_Init();
    
    /* 第二步：显示开机信息 */
    OLED_Clear();
    OLED_ShowString(0, 0, "E-AutoAim System");
    OLED_ShowString(0, 2, "Circles: 1");
    OLED_ShowString(0, 4, "State: IDLE");
    OLED_ShowString(0, 6, "Ready...");
    
    Alert_Beep(100);    // 开机提示音
    
    /* 第三步：主循环 */
    while(1)
    {
        /* 3.1 按键扫描与处理 */
        Key_Process();
        
        /* 3.2 状态机运行 */
        StateMachine_Run();
        
        /* 3.3 显示更新 */
        Display_Update();
        
        /* 3.4 短延时 */
        Delay_ms(SYS_TICK_MS);
    }
}

/* ============================================================
 * 函数名：System_Init
 * 功能：系统所有模块初始化
 * 参数：无
 * 返回：无
 * ============================================================ */
void System_Init(void)
{
    /* 时钟配置：80MHz系统时钟 */
    SystemClock_Config();
    
    /* GPIO初始化 */
    GPIO_Init();
    
    /* 定时器初始化（PWM输出+编码器输入） */
    TIM_Init();
    
    /* UART初始化（OpenMV通信） */
    UART_Init();
    
    /* 各模块初始化 */
    Tracking_Init();                    // 循迹传感器初始化
    Motor_Init();                       // 电机驱动初始化
    Gimbal_Init();                      // 云台初始化（舵机居中）
    Vision_Init();                      // 视觉模块通信初始化
    OLED_Init();                        // OLED初始化
    Key_Init();                         // 按键初始化
    Alert_Init();                       // 声光提示初始化
    
    /* PID控制器初始化 */
    PID_Init(&g_track_pid, 2.0f, 0.5f, 1.0f, -500.0f, 500.0f);
    PID_Init(&g_gimbal_pid_h, 1.5f, 0.3f, 0.8f, -90.0f, 90.0f);
    PID_Init(&g_gimbal_pid_v, 1.5f, 0.3f, 0.8f, -90.0f, 90.0f);
    
    /* 激光笔初始关闭 */
    Laser_SetState(LASER_OFF);
    
    /* 初始状态 */
    g_sys_state = STATE_IDLE;
    g_current_circle = 0;
    g_run_time_ms = 0;
}

/* ============================================================
 * 函数名：StateMachine_Run
 * 功能：系统主状态机，根据当前状态执行对应控制逻辑
 * 参数：无
 * 返回：无
 * 
 * 状态流转：
 * IDLE → TRACKING（启动循迹行驶）
 * IDLE → AIMING（启动静态瞄准）
 * IDLE → TRACK_AIM（启动行驶中瞄准）
 * TRACKING → STOP（到达目标圈数）
 * AIMING → STOP（瞄准完成）
 * TRACK_AIM → STOP（行驶+瞄准完成）
 * ============================================================ */
void StateMachine_Run(void)
{
    switch(g_sys_state)
    {
        case STATE_IDLE:
            /* 空闲状态：等待按键启动 */
            Motor_SetSpeed(0, 0);       // 电机停止
            break;
            
        case STATE_TRACKING:
            /* 循迹行驶状态 */
            TrackPID_Control();          // 执行循迹PID控制
            break;
            
        case STATE_AIMING:
            /* 静态瞄准状态 */
            GimbalAim_Control();         // 执行瞄准控制
            break;
            
        case STATE_TRACK_AIM:
            /* 行驶中瞄准状态：同时执行循迹和瞄准 */
            TrackPID_Control();          // 循迹控制
            GimbalAim_Control();         // 瞄准控制
            break;
            
        case STATE_DRAW_CIRCLE:
            /* 画圆状态 */
            TrackPID_Control();          // 循迹控制
            CircleDraw_Control();        // 画圆控制
            break;
            
        case STATE_STOP:
            /* 停止状态 */
            Motor_SetSpeed(0, 0);
            Laser_SetState(LASER_OFF);
            Alert_Beep(500);             // 停止提示音
            g_sys_state = STATE_IDLE;
            break;
            
        case STATE_ERROR:
            /* 错误状态 */
            Motor_SetSpeed(0, 0);
            Laser_SetState(LASER_OFF);
            Alert_Error();               // 错误报警
            break;
            
        default:
            g_sys_state = STATE_IDLE;
            break;
    }
}

/* ============================================================
 * 函数名：TrackPID_Control
 * 功能：循迹PID控制算法
 * 参数：无
 * 返回：无
 * 
 * 算法说明：
 * 1. 读取8路循迹传感器，计算位置偏差
 * 2. 偏差 = (加权平均位置 - 中心位置)
 * 3. PID输出 = 左右电机速度差
 * 4. 左电机速度 = 基础速度 + PID输出
 * 5. 右电机速度 = 基础速度 - PID输出
 * ============================================================ */
void TrackPID_Control(void)
{
    int16_t sensor_val;     // 传感器位置值(-100~+100)
    float pid_output;       // PID输出
    int16_t speed_l, speed_r;   // 左右电机速度
    
    /* 1. 读取循迹传感器位置 */
    sensor_val = Tracking_GetPosition();
    
    /* 2. 计算PID输出 */
    pid_output = PID_Calculate(&g_track_pid, 0.0f, (float)sensor_val);
    
    /* 3. 计算左右电机速度 */
    speed_l = MOTOR_PWM_BASE + (int16_t)pid_output;
    speed_r = MOTOR_PWM_BASE - (int16_t)pid_output;
    
    /* 4. 限幅保护 */
    if(speed_l > MOTOR_PWM_MAX) speed_l = MOTOR_PWM_MAX;
    if(speed_l < -MOTOR_PWM_MAX) speed_l = -MOTOR_PWM_MAX;
    if(speed_r > MOTOR_PWM_MAX) speed_r = MOTOR_PWM_MAX;
    if(speed_r < -MOTOR_PWM_MAX) speed_r = -MOTOR_PWM_MAX;
    
    /* 5. 设置电机速度 */
    Motor_SetSpeed(speed_l, speed_r);
    
    /* 6. 圈数检测：通过跨线次数判断 */
    if(Tracking_CheckCrossLine())
    {
        g_cross_count++;
        if(g_cross_count >= 4)  // 正方形有4条边，跨4次线为1圈
        {
            g_cross_count = 0;
            g_current_circle++;
            Alert_Beep(50);     // 过线提示
            
            /* 检查是否到达目标圈数 */
            if(g_current_circle >= g_target_circles)
            {
                Motor_SetSpeed(0, 0);   // 停车
                if(g_aim_mode == AIM_MODE_DYNAMIC)
                {
                    g_sys_state = STATE_STOP;
                }
                else
                {
                    g_sys_state = STATE_STOP;
                }
            }
        }
    }
}

/* ============================================================
 * 函数名：GimbalAim_Control
 * 功能：二维云台瞄准控制算法
 * 参数：无
 * 返回：无
 * 
 * 算法说明：
 * 1. 从OpenMV获取靶心坐标偏移量(dx, dy)
 * 2. PID计算水平和垂直舵机角度调整量
 * 3. 更新舵机PWM输出
 * 4. 当偏差小于阈值时认为瞄准成功
 * ============================================================ */
void GimbalAim_Control(void)
{
    static float aim_angle_h = GIMBAL_CENTER_H;    // 水平角度
    static float aim_angle_v = GIMBAL_CENTER_V;    // 垂直角度
    float pid_h, pid_v;                             // PID输出
    
    /* 1. 读取OpenMV视觉数据 */
    if(Vision_GetData(&g_vision_data))
    {
        /* 2. 计算PID输出 */
        pid_h = PID_Calculate(&g_gimbal_pid_h, 0.0f, g_vision_data.dx);
        pid_v = PID_Calculate(&g_gimbal_pid_v, 0.0f, g_vision_data.dy);
        
        /* 3. 更新舵机角度 */
        aim_angle_h -= pid_h * 0.1f;    // 负反馈：偏差为正则向反方向调整
        aim_angle_v += pid_v * 0.1f;
        
        /* 4. 角度限幅 */
        if(aim_angle_h < 0.0f) aim_angle_h = 0.0f;
        if(aim_angle_h > 180.0f) aim_angle_h = 180.0f;
        if(aim_angle_v < 0.0f) aim_angle_v = 0.0f;
        if(aim_angle_v > 180.0f) aim_angle_v = 180.0f;
        
        /* 5. 设置舵机角度 */
        Gimbal_SetAngle(aim_angle_h, aim_angle_v);
        
        /* 6. 开启激光笔 */
        Laser_SetState(LASER_ON);
        
        /* 7. 瞄准精度检查 */
        float error = sqrtf(g_vision_data.dx * g_vision_data.dx + 
                           g_vision_data.dy * g_vision_data.dy);
        if(error < 5.0f)    // 像素偏差小于5认为瞄准成功
        {
            Alert_ShortBeep();  // 短提示音
        }
    }
}

/* ============================================================
 * 函数名：CircleDraw_Control
 * 功能：激光笔画圆控制算法
 * 参数：无
 * 返回：无
 * 
 * 算法说明：
 * 1. 在循迹行驶的同时，控制云台做圆周运动
 * 2. 水平和垂直舵机分别输出正弦和余弦信号
 * 3. 角速度与小车行驶速度同步，确保画1圈=行驶1圈
 * 4. 画圆半径由题目要求确定(6cm)
 * ============================================================ */
void CircleDraw_Control(void)
{
    static float theta = 0.0f;  // 画圆角度(弧度)
    float angle_h, angle_v;
    
    /* 1. 计算画圆角度增量 */
    /* 正方形周长=400cm，行驶1圈对应theta从0到2π */
    theta += (2.0f * 3.14159f) / (400.0f / 1.0f);  // 每cm行驶对应的角度增量
    
    /* 2. 限制在0~2π */
    if(theta >= 2.0f * 3.14159f)
    {
        theta -= 2.0f * 3.14159f;
    }
    
    /* 3. 计算舵机角度（正弦/余弦叠加在中心角度上） */
    /* 6cm半径对应的角度偏移量需根据实际距离标定 */
    angle_h = GIMBAL_CENTER_H + 15.0f * cosf(theta);   // 15°约为6cm对应的角度
    angle_v = GIMBAL_CENTER_V + 15.0f * sinf(theta);
    
    /* 4. 设置舵机角度 */
    Gimbal_SetAngle(angle_h, angle_v);
}

/* ============================================================
 * 函数名：Key_Process
 * 功能：按键扫描与处理
 * 参数：无
 * 返回：无
 * 
 * 按键功能：
 * KEY1：切换圈数(1-5)
 * KEY2：启动/停止系统
 * KEY3：切换瞄准模式
 * KEY4：手动开关激光
 * ============================================================ */
void Key_Process(void)
{
    uint8_t key = Key_Scan();
    
    switch(key)
    {
        case KEY1_PRESS:
            /* 切换目标圈数 */
            g_target_circles++;
            if(g_target_circles > 5) g_target_circles = 1;
            break;
            
        case KEY2_PRESS:
            /* 启动/停止 */
            if(g_sys_state == STATE_IDLE)
            {
                /* 启动系统 */
                g_current_circle = 0;
                g_cross_count = 0;
                g_run_time_ms = 0;
                
                if(g_aim_mode == AIM_MODE_STATIC)
                {
                    g_sys_state = STATE_AIMING;     // 静态瞄准
                }
                else if(g_aim_mode == AIM_MODE_DYNAMIC)
                {
                    g_sys_state = STATE_TRACK_AIM;  // 行驶中瞄准
                }
                else if(g_aim_mode == AIM_MODE_CIRCLE)
                {
                    g_sys_state = STATE_DRAW_CIRCLE; // 画圆模式
                }
                else
                {
                    g_sys_state = STATE_TRACKING;    // 纯循迹
                }
                
                Alert_Beep(200);    // 启动提示
            }
            else
            {
                /* 停止系统 */
                g_sys_state = STATE_STOP;
            }
            break;
            
        case KEY3_PRESS:
            /* 切换瞄准模式 */
            g_aim_mode++;
            if(g_aim_mode > AIM_MODE_CIRCLE) g_aim_mode = AIM_MODE_STATIC;
            break;
            
        case KEY4_PRESS:
            /* 手动开关激光 */
            g_laser_state = !g_laser_state;
            Laser_SetState(g_laser_state);
            break;
            
        default:
            break;
    }
}

/* ============================================================
 * 函数名：Display_Update
 * 功能：OLED显示更新
 * 参数：无
 * 返回：无
 * ============================================================ */
void Display_Update(void)
{
    char buf[32];
    
    /* 第1行：系统状态 */
    switch(g_sys_state)
    {
        case STATE_IDLE:       OLED_ShowString(0, 0, "State: IDLE     "); break;
        case STATE_TRACKING:   OLED_ShowString(0, 0, "State: TRACKING "); break;
        case STATE_AIMING:     OLED_ShowString(0, 0, "State: AIMING   "); break;
        case STATE_TRACK_AIM:  OLED_ShowString(0, 0, "State: TRK+AIM  "); break;
        case STATE_DRAW_CIRCLE:OLED_ShowString(0, 0, "State: CIRCLE   "); break;
        case STATE_STOP:       OLED_ShowString(0, 0, "State: STOP     "); break;
        case STATE_ERROR:      OLED_ShowString(0, 0, "State: ERROR    "); break;
    }
    
    /* 第2行：圈数信息 */
    sprintf(buf, "Circle: %d/%d", g_current_circle, g_target_circles);
    OLED_ShowString(0, 2, buf);
    
    /* 第3行：运行时间 */
    sprintf(buf, "Time: %lu.%lus", g_run_time_ms/1000, (g_run_time_ms%1000)/100);
    OLED_ShowString(0, 4, buf);
    
    /* 第4行：瞄准偏差 */
    sprintf(buf, "Aim: dx=%d dy=%d", (int)g_vision_data.dx, (int)g_vision_data.dy);
    OLED_ShowString(0, 6, buf);
}

/* ============================================================
 * 函数名：SystemClock_Config
 * 功能：系统时钟配置为80MHz
 * 参数：无
 * 返回：无
 * ============================================================ */
void SystemClock_Config(void)
{
    /* MSPM0G3507内部振荡器配置 */
    /* 具体寄存器配置参考TI MSPM0 SDK */
    SYSCTL->SYSOSCCFG = SYSCTL_SYSOSCCFG_FREQ_80MHZ;
    while(!(SYSCTL->CLKSTATUS & SYSCTL_CLKSTATUS_SYSOSC_GOOD));
}

/* ============================================================
 * 函数名：GPIO_Init
 * 功能：GPIO引脚初始化
 * 参数：无
 * 返回：无
 * ============================================================ */
void GPIO_Init(void)
{
    /* 使能GPIO时钟 */
    __enable_irq();
    
    /* LED指示灯引脚配置 */
    /* PA0-PA1: 电源状态LED */
    
    /* 激光笔控制引脚 */
    /* PA5: 激光笔MOSFET控制 */
}

/* ============================================================
 * 中断服务函数
 * ============================================================ */

/**
 * @brief  TIM2中断服务函数（1ms系统节拍）
 *         用于运行时间计时和周期性任务调度
 */
void TIM2_IRQHandler(void)
{
    if(TIM2->IFG & TIM_IFG_CC0)
    {
        TIM2->IFG &= ~TIM_IFG_CC0;
        
        /* 运行时间计时 */
        if(g_sys_state != STATE_IDLE && g_sys_state != STATE_STOP)
        {
            g_run_time_ms++;
            
            /* 超时保护：20s */
            if(g_run_time_ms > 20000)
            {
                g_sys_state = STATE_STOP;
            }
        }
    }
}

/**
 * @brief  UART1中断服务函数（接收OpenMV数据）
 */
void UART1_IRQHandler(void)
{
    if(UART1->IFG & UART_IFG_RX)
    {
        volatile char ch = UART1->RXDATA;
        Vision_RxCallback(ch);
        UART1->IFG &= ~UART_IFG_RX;
    }
}
