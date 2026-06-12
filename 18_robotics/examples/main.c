/*
 * 电赛平台综合示例 - 主程序
 * 适配平台：MSPM0G3507
 * 
 * 功能：
 * 1. 电机控制（PID算法）
 * 2. 路径规划（A*算法）
 * 3. 视觉处理（车道线检测）
 * 4. 数据可视化调试
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

// 引入各个模块
#include "incremental_pid.h"
#include "position_pid.h"
#include "cascade_pid.h"
#include "astar.h"
#include "lane_detection.h"
#include "state_machine.h"
#include "data_visualization.h"

// 硬件抽象层（需要根据实际平台实现）
#include "hal_gpio.h"
#include "hal_uart.h"
#include "hal_timer.h"
#include "hal_pwm.h"
#include "hal_adc.h"
#include "hal_encoder.h"
#include "hal_camera.h"

// ==================== 全局变量 ====================

// 电机控制
IncrementalPID_HandleTypeDef speed_pid;     // 速度PID
PositionPID_HandleTypeDef steering_pid;     // 转向PID
CascadePID_HandleTypeDef position_pid;      // 位置PID

// 路径规划
AStar_HandleTypeDef astar;
Path *planned_path;

// 视觉处理
LaneDetector_HandleTypeDef lane_detector;
LaneDetectionResult *lane_result;

// 状态机
StateMachine_HandleTypeDef fsm;
DataVisualization_HandleTypeDef vis;

// 系统状态
typedef enum {
    SYS_IDLE,
    SYS_RUNNING,
    SYS_STOPPED,
    SYS_ERROR
} SystemState;

typedef enum {
    EVT_START,
    EVT_STOP,
    EVT_ERROR,
    EVT_RESET
} SystemEvent;

// 传感器数据
float motor_speed = 0.0f;          // 电机转速 (rpm)
float encoder_angle = 0.0f;        // 编码器角度 (度)
float lane_offset = 0.0f;          // 车道偏移量 (像素)
float distance_front = 0.0f;       // 前方距离 (cm)
float battery_voltage = 0.0f;      // 电池电压 (V)

// 控制输出
float motor_pwm = 0.0f;            // 电机PWM
float steering_servo = 0.0f;       // 转向舵机

// ==================== 初始化函数 ====================

/**
 * @brief 系统初始化
 */
void System_Init(void)
{
    // 初始化HAL层
    HAL_Init();
    
    // 配置时钟
    SystemClock_Config();
    
    // 初始化GPIO
    GPIO_Init();
    
    // 初始化UART（调试串口）
    UART_Init(UART_DEBUG, 115200);
    
    // 初始化定时器
    TIM_Init();
    
    // 初始化PWM
    PWM_Init(PWM_MOTOR, 20000);    // 20kHz
    PWM_Init(PWM_SERVO, 50);       // 50Hz
    
    // 初始化ADC
    ADC_Init();
    
    // 初始化编码器
    Encoder_Init();
    
    // 初始化摄像头
    Camera_Init(OV7670, 320, 240);
    
    printf("System initialized.\n");
}

/**
 * @brief 算法初始化
 */
void Algorithm_Init(void)
{
    // 初始化PID控制器
    IncrementalPID_Init(&speed_pid, 0.5f, 0.01f, 0.1f);
    IncrementalPID_SetOutputLimit(&speed_pid, -1000.0f, 1000.0f);
    
    PositionPID_Init(&steering_pid, 0.8f, 0.01f, 0.2f);
    PositionPID_SetOutputLimit(&steering_pid, -45.0f, 45.0f);
    
    CascadePID_Init(&position_pid, 0.1f, 0.001f, 0.05f, 0.5f, 0.01f, 0.1f);
    
    // 初始化A*路径规划
    AStar_Init(&astar);
    
    // 初始化车道线检测
    LaneDetector_Init(&lane_detector);
    lane_detector.threshold = 128;
    
    printf("Algorithms initialized.\n");
}

/**
 * @brief 调试工具初始化
 */
void Debug_Init(void)
{
    // 初始化数据可视化
    DataVis_Init(&vis);
    
    // 注册数据通道
    DataVis_RegisterChannel(&vis, 0, DATA_TYPE_FLOAT, "电机速度");
    DataVis_RegisterChannel(&vis, 1, DATA_TYPE_FLOAT, "编码器角度");
    DataVis_RegisterChannel(&vis, 2, DATA_TYPE_FLOAT, "车道偏移");
    DataVis_RegisterChannel(&vis, 3, DATA_TYPE_FLOAT, "前方距离");
    DataVis_RegisterChannel(&vis, 4, DATA_TYPE_FLOAT, "电池电压");
    DataVis_RegisterChannel(&vis, 5, DATA_TYPE_FLOAT, "电机PWM");
    DataVis_RegisterChannel(&vis, 6, DATA_TYPE_FLOAT, "转向角度");
    DataVis_RegisterChannel(&vis, 7, DATA_TYPE_FLOAT, "系统状态");
    
    // 设置发送回调
    DataVis_SetSendCallback(&vis, UART_SendData);
    
    printf("Debug tools initialized.\n");
}

/**
 * @brief 状态机初始化
 */
void StateMachine_Init_Example(void)
{
    StateMachine_Init(&fsm);
    
    // 注册状态处理函数
    StateMachine_RegisterState(&fsm, SYS_IDLE, NULL);
    StateMachine_RegisterState(&fsm, SYS_RUNNING, NULL);
    StateMachine_RegisterState(&fsm, SYS_STOPPED, NULL);
    StateMachine_RegisterState(&fsm, SYS_ERROR, NULL);
    
    // 注册状态转换
    StateMachine_RegisterTransition(&fsm, SYS_IDLE, EVT_START, SYS_RUNNING, NULL);
    StateMachine_RegisterTransition(&fsm, SYS_RUNNING, EVT_STOP, SYS_STOPPED, NULL);
    StateMachine_RegisterTransition(&fsm, SYS_RUNNING, EVT_ERROR, SYS_ERROR, NULL);
    StateMachine_RegisterTransition(&fsm, SYS_STOPPED, EVT_START, SYS_RUNNING, NULL);
    StateMachine_RegisterTransition(&fsm, SYS_ERROR, EVT_RESET, SYS_IDLE, NULL);
    
    printf("State machine initialized.\n");
}

// ==================== 任务函数 ====================

/**
 * @brief 传感器数据采集任务（1ms周期）
 */
void Sensor_Task(void)
{
    // 读取编码器
    encoder_angle = Encoder_GetAngle();
    motor_speed = Encoder_GetSpeed();
    
    // 读取超声波
    distance_front = Ultrasonic_GetDistance();
    
    // 读取电池电压
    battery_voltage = ADC_ReadBatteryVoltage();
}

/**
 * @brief 电机控制任务（1ms周期）
 */
void MotorControl_Task(void)
{
    if (StateMachine_IsInState(&fsm, SYS_RUNNING)) {
        // 计算速度PID
        float target_speed = 500.0f;  // 目标速度
        motor_pwm = IncrementalPID_Compute(&speed_pid, target_speed, motor_speed);
        
        // 设置PWM
        PWM_SetDuty(PWM_MOTOR, motor_pwm);
    } else {
        // 停止电机
        PWM_SetDuty(PWM_MOTOR, 0.0f);
        IncrementalPID_Reset(&speed_pid);
    }
}

/**
 * @brief 视觉处理任务（10ms周期）
 */
void Vision_Task(void)
{
    if (StateMachine_IsInState(&fsm, SYS_RUNNING)) {
        // 采集图像
        uint8_t image[IMAGE_HEIGHT][IMAGE_WIDTH];
        Camera_Capture((uint8_t*)image);
        
        // 设置图像
        LaneDetector_SetImage(&lane_detector, (uint8_t*)image);
        
        // 车道线检测
        LaneDetector_Binarize(&lane_detector);
        LaneDetector_EdgeDetect(&lane_detector);
        LaneDetector_FindLaneLines(&lane_detector);
        
        // 获取结果
        lane_result = LaneDetector_GetResult(&lane_detector);
        
        if (lane_result->valid) {
            lane_offset = lane_result->offset;
            
            // 转向控制
            steering_servo = PositionPID_Compute(&steering_pid, 0, lane_offset);
            Servo_SetAngle(PWM_SERVO, steering_servo);
        }
    }
}

/**
 * @brief 路径规划任务（100ms周期）
 */
void PathPlanning_Task(void)
{
    if (StateMachine_IsInState(&fsm, SYS_RUNNING)) {
        // 更新地图（添加障碍物）
        uint8_t map[MAP_WIDTH][MAP_HEIGHT] = {0};
        
        // 根据超声波数据更新地图
        if (distance_front < 30.0f) {
            // 在前方添加障碍物
            int16_t obs_x = (int16_t)(encoder_angle / 10.0f) + 5;
            int16_t obs_y = MAP_HEIGHT / 2;
            map[obs_x][obs_y] = NODE_OBSTACLE;
        }
        
        // 设置地图
        AStar_SetMap(&astar, map);
        
        // 设置起点和终点
        AStar_SetStart(&astar, 0, MAP_HEIGHT / 2);
        AStar_SetEnd(&astar, MAP_WIDTH - 1, MAP_HEIGHT / 2);
        
        // 查找路径
        if (AStar_FindPath(&astar)) {
            planned_path = AStar_GetPath(&astar);
            // 执行路径跟踪...
        }
    }
}

/**
 * @brief 调试输出任务（100ms周期）
 */
void Debug_Task(void)
{
    // 更新可视化数据
    DataVis_SetValue(&vis, 0, motor_speed);
    DataVis_SetValue(&vis, 1, encoder_angle);
    DataVis_SetValue(&vis, 2, lane_offset);
    DataVis_SetValue(&vis, 3, distance_front);
    DataVis_SetValue(&vis, 4, battery_voltage);
    DataVis_SetValue(&vis, 5, motor_pwm);
    DataVis_SetValue(&vis, 6, steering_servo);
    DataVis_SetValue(&vis, 7, (float)StateMachine_GetState(&fsm));
    
    // 发送数据
    DataVis_SendData(&vis);
}

// ==================== 主函数 ====================

int main(void)
{
    // 系统初始化
    System_Init();
    
    // 算法初始化
    Algorithm_Init();
    
    // 调试工具初始化
    Debug_Init();
    
    // 状态机初始化
    StateMachine_Init_Example();
    
    printf("System started.\n");
    
    // 设置初始状态
    StateMachine_SetEvent(&fsm, EVT_START);
    
    // 主循环
    while (1) {
        // 运行状态机
        StateMachine_Run(&fsm);
        
        // 运行各个任务
        Sensor_Task();
        MotorControl_Task();
        Vision_Task();
        PathPlanning_Task();
        Debug_Task();
        
        // 检查错误条件
        if (battery_voltage < 10.0f) {
            StateMachine_SetEvent(&fsm, EVT_ERROR);
            printf("Error: Low battery!\n");
        }
        
        // 延时
        HAL_Delay(1);
    }
}
