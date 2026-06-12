/**
 * @file pid_tm4c.c
 * @brief PID控制器实现 (FPU硬件加速)
 *
 * 所有浮点运算由Cortex-M4F FPU硬件执行，
 * 无需软件浮点库，单周期乘法，5周期除法。
 */
#include "platform/tivaware.h"
#include "algorithm/pid_tm4c.h"

/* ======================== 实现 ======================== */

void pid_init(pid_t *pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    pid->out_max = 1000.0f;
    pid->out_min = -1000.0f;
    pid->integral_max = 500.0f;
    pid->integral_min = -500.0f;
    pid->d_filter_alpha = 0.7f;  /* 微分低通滤波系数 */

    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_d = 0.0f;
    pid->output = 0.0f;
    pid->first_run = true;
}

void pid_set_gains(pid_t *pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
}

void pid_set_output_limit(pid_t *pid, float min, float max)
{
    pid->out_min = min;
    pid->out_max = max;
}

void pid_set_integral_limit(pid_t *pid, float max)
{
    pid->integral_max = max;
    pid->integral_min = -max;
}

float pid_calc(pid_t *pid, float target, float actual)
{
    /* 误差 */
    float error = target - actual;

    /* 比例项 */
    float P = pid->Kp * error;

    /* 积分项 (带限幅) */
    pid->integral += error;
    pid->integral = CLAMP(pid->integral, pid->integral_min, pid->integral_max);
    float I = pid->Ki * pid->integral;

    /* 微分项 (带低通滤波，避免微分尖峰) */
    float D;
    if (pid->first_run) {
        D = 0.0f;
        pid->first_run = false;
    } else {
        float raw_d = error - pid->prev_error;
        /* 一阶低通滤波: D = alpha * D_prev + (1-alpha) * raw_d */
        D = pid->d_filter_alpha * pid->prev_d +
            (1.0f - pid->d_filter_alpha) * raw_d;
        pid->prev_d = D;  /* 保存滤波后的微分值（不含Kd增益） */
        D *= pid->Kd;
    }

    /* 输出 */
    float output = P + I + D;
    output = CLAMP(output, pid->out_min, pid->out_max);

    /* 保存状态 */
    pid->prev_error = error;
    pid->output = output;

    return output;
}

float pid_calc_incremental(pid_t *pid, float target, float actual)
{
    float error = target - actual;

    /* 增量式PID: Δu = Kp*(e-e1) + Ki*e + Kd*(e-2*e1+e2) */
    float delta_P = pid->Kp * (error - pid->prev_error);
    float delta_I = pid->Ki * error;
    float delta_D = 0.0f;

    if (!pid->first_run) {
        delta_D = pid->Kd * (error - 2.0f * pid->prev_error + pid->prev_d);
    } else {
        pid->first_run = false;
    }

    float delta = delta_P + delta_I + delta_D;
    pid->output += delta;
    pid->output = CLAMP(pid->output, pid->out_min, pid->out_max);

    /* 保存状态 */
    pid->prev_d = pid->prev_error;   /* e2 = e1_prev */
    pid->prev_error = error;

    return pid->output;
}

void pid_reset(pid_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_d = 0.0f;
    pid->output = 0.0f;
    pid->first_run = true;
}
