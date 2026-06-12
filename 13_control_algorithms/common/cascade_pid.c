/**
 * @file cascade_pid.c
 * @brief 串级PID控制器实现
 */
#include "cascade_pid.h"
#include <math.h>
#include <string.h>

#define CLAMP(val, min, max) ((val) < (min) ? (min) : ((val) > (max) ? (max) : (val)))

/* 单环PID计算 */
float CascadePID_SingleLoopCalc(CascadePID_Loop_t *loop,
                                 float setpoint, float feedback,
                                 float dt)
{
    if (dt <= 0.0f) return loop->output;

    float error = setpoint - feedback;

    /* 死区处理 */
    if (fabsf(error) < loop->gain.deadband) {
        error = 0.0f;
    }

    /* 比例项 */
    float p_term = loop->gain.Kp * error;

    /* 积分项（梯形积分 + 抗积分饱和） */
    loop->integral += error * dt;
    loop->integral = CLAMP(loop->integral, -loop->gain.integral_max, loop->gain.integral_max);
    float i_term = loop->gain.Ki * loop->integral;

    /* 微分项（带一阶低通滤波） */
    float d_raw = (error - loop->prev_error) / dt;
    float alpha = loop->gain.d_filter_alpha;
    float d_filtered = alpha * loop->prev_d + (1.0f - alpha) * d_raw;
    float d_term = loop->gain.Kd * d_filtered;

    /* 输出 */
    float output = p_term + i_term + d_term;
    output = CLAMP(output, loop->gain.output_min, loop->gain.output_max);

    /* 抗积分饱和：输出饱和时回退积分 */
    if (output >= loop->gain.output_max || output <= loop->gain.output_min) {
        loop->integral -= error * dt * 0.5f;
    }

    loop->prev_error = error;
    loop->prev_d = d_filtered;
    loop->output = output;

    return output;
}

void CascadePID_Init(CascadePID_t *pid,
                     const CascadePID_Gain_t *outer_gain,
                     const CascadePID_Gain_t *inner_gain)
{
    if (pid == NULL || outer_gain == NULL || inner_gain == NULL) return;
    memset(pid, 0, sizeof(CascadePID_t));
    memcpy(&pid->outer.gain, outer_gain, sizeof(CascadePID_Gain_t));
    memcpy(&pid->inner.gain, inner_gain, sizeof(CascadePID_Gain_t));
    pid->initialized = 1;
}

float CascadePID_Calc(CascadePID_t *pid,
                      float outer_setpoint,
                      float outer_feedback,
                      float inner_feedback,
                      float dt)
{
    if (pid == NULL || !pid->initialized || dt <= 0.0f) {
        return (pid != NULL) ? pid->final_output : 0.0f;
    }

    pid->outer_setpoint = outer_setpoint;
    pid->outer_feedback = outer_feedback;

    /* 外环计算 -> 内环设定值 */
    pid->inner_setpoint = CascadePID_SingleLoopCalc(
        &pid->outer, outer_setpoint, outer_feedback, dt);

    /* 内环计算 -> 最终输出 */
    pid->inner_feedback = inner_feedback;
    pid->final_output = CascadePID_SingleLoopCalc(
        &pid->inner, pid->inner_setpoint, inner_feedback, dt);

    return pid->final_output;
}

void CascadePID_SetOuterGain(CascadePID_t *pid, const CascadePID_Gain_t *gain)
{
    if (pid == NULL || gain == NULL) return;
    memcpy(&pid->outer.gain, gain, sizeof(CascadePID_Gain_t));
}

void CascadePID_SetInnerGain(CascadePID_t *pid, const CascadePID_Gain_t *gain)
{
    if (pid == NULL || gain == NULL) return;
    memcpy(&pid->inner.gain, gain, sizeof(CascadePID_Gain_t));
}

void CascadePID_Reset(CascadePID_t *pid)
{
    if (pid == NULL) return;
    pid->outer.integral = 0.0f;
    pid->outer.prev_error = 0.0f;
    pid->outer.prev_d = 0.0f;
    pid->outer.output = 0.0f;
    pid->inner.integral = 0.0f;
    pid->inner.prev_error = 0.0f;
    pid->inner.prev_d = 0.0f;
    pid->inner.output = 0.0f;
    pid->final_output = 0.0f;
}
