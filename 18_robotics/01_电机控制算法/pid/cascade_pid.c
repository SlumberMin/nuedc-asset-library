/*
 * 串级PID控制器实现
 * 来源：RoboMaster 步兵机器人
 * 适配平台：MSPM0G3507
 */

#include "cascade_pid.h"

/**
 * @brief 初始化串级PID控制器
 * @param hpid 串级PID句柄
 * @param outer_kp 外环比例系数
 * @param outer_ki 外环积分系数
 * @param outer_kd 外环微分系数
 * @param inner_kp 内环比例系数
 * @param inner_ki 内环积分系数
 * @param inner_kd 内环微分系数
 */
void CascadePID_Init(CascadePID_HandleTypeDef *hpid,
                     float outer_kp, float outer_ki, float outer_kd,
                     float inner_kp, float inner_ki, float inner_kd)
{
    // 初始化外环（位置环）
    PositionPID_Init(&hpid->outer_pid, outer_kp, outer_ki, outer_kd);
    
    // 初始化内环（速度环）
    IncrementalPID_Init(&hpid->inner_pid, inner_kp, inner_ki, inner_kd);
    
    // 设置默认限幅
    PositionPID_SetOutputLimit(&hpid->outer_pid, -1000.0f, 1000.0f);
    PositionPID_SetIntegralLimit(&hpid->outer_pid, -500.0f, 500.0f);
    
    IncrementalPID_SetOutputLimit(&hpid->inner_pid, -1000.0f, 1000.0f);
    
    // 初始化目标值
    hpid->target_position = 0.0f;
    hpid->target_velocity = 0.0f;
    
    // 初始化测量值
    hpid->measured_position = 0.0f;
    hpid->measured_velocity = 0.0f;
    
    // 初始化输出
    hpid->output = 0.0f;
}

/**
 * @brief 设置目标位置
 * @param hpid 串级PID句柄
 * @param position 目标位置
 */
void CascadePID_SetTarget(CascadePID_HandleTypeDef *hpid, float position)
{
    hpid->target_position = position;
}

/**
 * @brief 设置测量值
 * @param hpid 串级PID句柄
 * @param position 测量位置
 * @param velocity 测量速度
 */
void CascadePID_SetMeasurement(CascadePID_HandleTypeDef *hpid, float position, float velocity)
{
    hpid->measured_position = position;
    hpid->measured_velocity = velocity;
}

/**
 * @brief 计算串级PID输出
 * @param hpid 串级PID句柄
 * @return PID输出
 */
float CascadePID_Compute(CascadePID_HandleTypeDef *hpid)
{
    // 外环计算：位置环
    // 输入：目标位置 - 测量位置
    // 输出：目标速度
    hpid->target_velocity = PositionPID_Compute(&hpid->outer_pid,
                                                 hpid->target_position,
                                                 hpid->measured_position);
    
    // 内环计算：速度环
    // 输入：目标速度 - 测量速度
    // 输出：控制量（如PWM占空比）
    hpid->output = IncrementalPID_Compute(&hpid->inner_pid,
                                           hpid->target_velocity,
                                           hpid->measured_velocity);
    
    return hpid->output;
}

/**
 * @brief 重置串级PID控制器
 * @param hpid 串级PID句柄
 */
void CascadePID_Reset(CascadePID_HandleTypeDef *hpid)
{
    PositionPID_Reset(&hpid->outer_pid);
    IncrementalPID_Reset(&hpid->inner_pid);
    
    hpid->target_position = 0.0f;
    hpid->target_velocity = 0.0f;
    hpid->measured_position = 0.0f;
    hpid->measured_velocity = 0.0f;
    hpid->output = 0.0f;
}
