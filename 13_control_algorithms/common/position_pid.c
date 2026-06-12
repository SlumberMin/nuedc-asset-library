/**
 * @file position_pid.c
 * @brief 位置式PID控制器实现
 * @details 提供完整的位置式PID控制算法, 支持:
 *          - 积分分离(大误差时停止积分)
 *          - 积分限幅(抗饱和)
 *          - 微分先行(减少设定值突变引起的微分冲击)
 *          - 微分项低通滤波
 *          - 死区控制
 */

#include "position_pid.h"
#include <string.h>
#include <math.h>

/**
 * @brief 初始化位置式PID控制器
 * @param pid PID结构体指针
 * @param Kp 比例增益
 * @param Ki 积分增益
 * @param Kd 微分增益
 * @param out_min 输出下限
 * @param out_max 输出上限
 */
void PositionPID_Init(PositionPID_t *pid,
                      float Kp, float Ki, float Kd,
                      float out_min, float out_max)
{
    if (pid == NULL) return;
    memset(pid, 0, sizeof(PositionPID_t));
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->integral_min = -1e30f;
    pid->integral_max =  1e30f;
    pid->integral_sep_threshold = 1e30f;  /* 默认不启用积分分离 */
    pid->derivative_on_feedback = 0;
    pid->d_filter_alpha = 0.0f;
    pid->dead_zone = 0.0f;
}

/**
 * @brief 设置积分项限幅范围
 * @param pid PID结构体指针
 * @param min 积分下限
 * @param max 积分上限
 */
void PositionPID_SetIntegralLimit(PositionPID_t *pid, float min, float max)
{
    if (pid == NULL) return;
    pid->integral_min = min;
    pid->integral_max = max;
}

/**
 * @brief 设置积分分离阈值
 * @param pid PID结构体指针
 * @param threshold 误差阈值, 超过此值停止积分
 */
void PositionPID_SetIntegralSeparation(PositionPID_t *pid, float threshold)
{
    if (pid == NULL) return;
    pid->integral_sep_threshold = (threshold < 0.0f) ? 0.0f : threshold;
}

/**
 * @brief 启用/禁用微分先行模式
 * @param pid PID结构体指针
 * @param enable 1=启用微分先行, 0=标准微分
 */
void PositionPID_EnableDerivativeOnFeedback(PositionPID_t *pid, uint8_t enable)
{
    if (pid == NULL) return;
    pid->derivative_on_feedback = enable;
}

/**
 * @brief 设置微分项低通滤波系数
 * @param pid PID结构体指针
 * @param alpha 滤波系数(0~0.99), 0=不滤波, 越大滤波越强
 */
void PositionPID_SetDFilter(PositionPID_t *pid, float alpha)
{
    if (pid == NULL) return;
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 0.99f) alpha = 0.99f;
    pid->d_filter_alpha = alpha;
}

/**
 * @brief 设置死区大小
 * @param pid PID结构体指针
 * @param dead_zone 死区宽度(绝对值), 误差在此范围内输出为0
 */
void PositionPID_SetDeadZone(PositionPID_t *pid, float dead_zone)
{
    if (pid == NULL) return;
    pid->dead_zone = (dead_zone < 0.0f) ? 0.0f : dead_zone;
}

/**
 * @brief 执行一步PID计算
 * @param pid PID结构体指针
 * @param setpoint 设定值(目标)
 * @param feedback 反馈值(当前测量)
 * @return PID控制器输出
 *
 * @details 位置式PID公式:
 *          output = Kp*e + Ki*∫e + Kd*de/dt
 *          支持积分分离、微分先行、死区等功能
 */
float PositionPID_Compute(PositionPID_t *pid, float setpoint, float feedback)
{
    if (pid == NULL) return 0.0f;

    /* 计算误差 */
    float error = setpoint - feedback;

    /* 死区处理: 误差在死区内时置零 */
    if (error > -pid->dead_zone && error < pid->dead_zone) {
        error = 0.0f;
    }

    /* 更新误差历史 */
    pid->err_last = pid->err;
    pid->err = error;

    /* P项: 比例输出 */
    float p_out = pid->Kp * error;

    /* I项: 带积分分离的积分输出 */
    float i_out = 0.0f;
    if (fabsf(error) < pid->integral_sep_threshold) {
        pid->err_sum += error;
        /* 积分限幅(抗饱和) */
        if (pid->err_sum > pid->integral_max) pid->err_sum = pid->integral_max;
        if (pid->err_sum < pid->integral_min) pid->err_sum = pid->integral_min;
        i_out = pid->Ki * pid->err_sum;
    }

    /* D项: 微分输出 */
    float d_raw;
    if (pid->derivative_on_feedback) {
        /* 微分先行: -Kd * d(feedback)/dt, 避免设定值突变引起微分冲击 */
        d_raw = -pid->Kd * (feedback - pid->feedback_last);
        pid->feedback_last = feedback;
    } else {
        /* 标准微分: Kd * de/dt */
        d_raw = pid->Kd * (pid->err - pid->err_last);
    }

    /* D项低通滤波 */
    pid->d_filtered = pid->d_filter_alpha * pid->d_filtered
                    + (1.0f - pid->d_filter_alpha) * d_raw;
    float d_out = pid->d_filtered;

    /* 合成输出: P + I + D */
    pid->output = p_out + i_out + d_out;

    /* 输出限幅 */
    if (pid->output > pid->out_max) pid->output = pid->out_max;
    if (pid->output < pid->out_min) pid->output = pid->out_min;

    return pid->output;
}

/**
 * @brief 重置PID控制器内部状态
 * @param pid PID结构体指针
 */
void PositionPID_Reset(PositionPID_t *pid)
{
    if (pid == NULL) return;
    pid->err = 0.0f;
    pid->err_last = 0.0f;
    pid->err_sum = 0.0f;
    pid->d_filtered = 0.0f;
    pid->feedback_last = 0.0f;
    pid->output = 0.0f;
}
