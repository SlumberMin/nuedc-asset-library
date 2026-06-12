/**
 * @file velocity_pid.c
 * @brief 增量式(速度式)PID控制器实现
 * @details 增量式PID只输出控制量的增量Δu, 适用于执行器需要增量输入的场景。
 *          公式: Δu = Kp*(e0-e1) + Ki*e0 + Kd*(e0-2*e1+e2)
 *          支持微分滤波、死区、积分限幅等特性。
 */

#include "velocity_pid.h"
#include <string.h>

/**
 * @brief 初始化增量式PID控制器
 * @param pid PID结构体指针
 * @param Kp 比例增益
 * @param Ki 积分增益
 * @param Kd 微分增益
 * @param out_min 输出下限
 * @param out_max 输出上限
 */
void VelocityPID_Init(VelocityPID_t *pid,
                      float Kp, float Ki, float Kd,
                      float out_min, float out_max)
{
    if (pid == NULL) return;
    memset(pid, 0, sizeof(VelocityPID_t));
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->integral_min = -1e30f;
    pid->integral_max =  1e30f;
    pid->d_filter_alpha = 0.0f;  /* 默认不滤波 */
    pid->dead_zone = 0.0f;
}

/**
 * @brief 设置积分项限幅范围
 * @param pid PID结构体指针
 * @param min 积分下限
 * @param max 积分上限
 */
void VelocityPID_SetIntegralLimit(VelocityPID_t *pid, float min, float max)
{
    if (pid == NULL) return;
    pid->integral_min = min;
    pid->integral_max = max;
}

/**
 * @brief 设置微分项低通滤波系数
 * @param pid PID结构体指针
 * @param alpha 滤波系数(0~0.99), 0=不滤波
 */
void VelocityPID_SetDFilter(VelocityPID_t *pid, float alpha)
{
    if (pid == NULL) return;
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 0.99f) alpha = 0.99f;
    pid->d_filter_alpha = alpha;
}

/**
 * @brief 设置死区大小
 * @param pid PID结构体指针
 * @param dead_zone 死区宽度(绝对值)
 */
void VelocityPID_SetDeadZone(VelocityPID_t *pid, float dead_zone)
{
    if (pid == NULL) return;
    pid->dead_zone = (dead_zone < 0.0f) ? 0.0f : dead_zone;
}

/**
 * @brief 执行一步增量式PID计算
 * @param pid PID结构体指针
 * @param setpoint 设定值(目标)
 * @param feedback 反馈值(当前测量)
 * @return PID控制器输出(累积值)
 *
 * @details 增量式PID计算流程:
 *          1. 计算误差并更新历史
 *          2. 计算增量 Δu = Kp*(e0-e1) + Ki*e0 + Kd*(e0-2*e1+e2)
 *          3. 累加到输出并限幅
 */
float VelocityPID_Compute(VelocityPID_t *pid, float setpoint, float feedback)
{
    if (pid == NULL) return 0.0f;

    /* 计算误差 */
    float error = setpoint - feedback;

    /* 死区处理 */
    if (error > -pid->dead_zone && error < pid->dead_zone) {
        error = 0.0f;
    }

    /* 更新误差历史: shift e2←e1←e0 */
    pid->err[2] = pid->err[1];
    pid->err[1] = pid->err[0];
    pid->err[0] = error;

    /* 增量计算: Δu = Kp*(e0-e1) + Ki*e0 + Kd*(e0-2*e1+e2) */
    float delta_p = pid->Kp * (pid->err[0] - pid->err[1]);
    float delta_i = pid->Ki * pid->err[0];
    float delta_d_raw = pid->Kd * (pid->err[0] - 2.0f * pid->err[1] + pid->err[2]);

    /* 微分项低通滤波 */
    pid->d_filtered = pid->d_filter_alpha * pid->d_filtered
                    + (1.0f - pid->d_filter_alpha) * delta_d_raw;
    float delta_d = pid->d_filtered;

    /* 积分累积并限幅(抗饱和) */
    pid->integral += delta_i;
    if (pid->integral > pid->integral_max) pid->integral = pid->integral_max;
    if (pid->integral < pid->integral_min) pid->integral = pid->integral_min;

    float delta_u = delta_p + delta_i + delta_d;

    /* 累加到输出 */
    pid->output += delta_u;

    /* 输出限幅 */
    if (pid->output > pid->out_max) pid->output = pid->out_max;
    if (pid->output < pid->out_min) pid->output = pid->out_min;

    return pid->output;
}

/**
 * @brief 重置增量式PID控制器内部状态
 * @param pid PID结构体指针
 */
void VelocityPID_Reset(VelocityPID_t *pid)
{
    if (pid == NULL) return;
    pid->err[0] = pid->err[1] = pid->err[2] = 0.0f;
    pid->integral = 0.0f;
    pid->d_filtered = 0.0f;
    pid->output = 0.0f;
}
