/**
 * @file    pid_mspm0.c
 * @brief   PID 控制器实现
 */

#include "pid_mspm0.h"

/* ── 内部辅助 ────────────────────────────────────────────── */
static inline float clampf(float val, float limit)
{
    if (val >  limit) return  limit;
    if (val < -limit) return -limit;
    return val;
}

/* ── API ─────────────────────────────────────────────────── */

void PID_Init(PID *pid, const PID_Params *param)
{
    pid->param          = *param;
    pid->integral       = 0.0f;
    pid->prev_error     = 0.0f;
    pid->prev_prev_error= 0.0f;
    pid->output         = 0.0f;
}

void PID_Reset(PID *pid)
{
    pid->integral        = 0.0f;
    pid->prev_error      = 0.0f;
    pid->prev_prev_error = 0.0f;
    pid->output          = 0.0f;
}

/* ── 位置式 PID ──────────────────────────────────────────── */
float PID_Calc(PID *pid, float target, float actual)
{
    float error = target - actual;

    /* 死区处理 */
    if (error > -pid->param.dead_zone && error < pid->param.dead_zone) {
        error = 0.0f;
    }

    /* 积分累加 + 限幅 */
    pid->integral += error;
    pid->integral = clampf(pid->integral, pid->param.integral_max);

    /* 微分 */
    float diff = error - pid->prev_error;
    pid->prev_error = error;

    /* 输出 */
    float out = pid->param.kp * error
              + pid->param.ki * pid->integral
              + pid->param.kd * diff;

    out = clampf(out, pid->param.output_max);
    pid->output = out;
    return out;
}

/* ── 增量式 PID ──────────────────────────────────────────── */
float PID_CalcIncremental(PID *pid, float target, float actual)
{
    float error = target - actual;

    /* 增量 Δu = Kp*(e-e1) + Ki*e + Kd*(e-2*e1+e2) */
    float delta = pid->param.kp * (error - pid->prev_error)
                + pid->param.ki * error
                + pid->param.kd * (error - 2.0f * pid->prev_error + pid->prev_prev_error);

    pid->prev_prev_error = pid->prev_error;
    pid->prev_error = error;

    pid->output += delta;
    pid->output = clampf(pid->output, pid->param.output_max);

    return pid->output;
}

/* ── 参数更新 ────────────────────────────────────────────── */
void PID_SetParams(PID *pid, float kp, float ki, float kd)
{
    pid->param.kp = kp;
    pid->param.ki = ki;
    pid->param.kd = kd;
}
