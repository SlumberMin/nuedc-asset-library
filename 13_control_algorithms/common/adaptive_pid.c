/**
 * @file adaptive_pid.c
 * @brief 自适应PID控制器实现
 * 
 * 核心思想：根据误差信号，用梯度下降法或MIT规则在线调整Kp/Ki/Kd。
 * 
 * 梯度下降法：
 *   dKp = -lr_p * e(k) * d(u)/d(Kp) ≈ -lr_p * e(k) * e(k)
 *   dKi = -lr_i * e(k) * d(u)/d(Ki) ≈ -lr_i * e(k) * sum(e)
 *   dKd = -lr_d * e(k) * d(u)/d(Kd) ≈ -lr_d * e(k) * (e(k)-e(k-1))
 * 
 * MIT规则：
 *   dK = -alpha * e(k) * (∂y/∂u) * x(k)
 *   其中 ∂y/∂u 近似用差分估计
 */

#include "adaptive_pid.h"
#include <math.h>
#include <string.h>

/* 参数限幅宏 */
#define CLAMP(val, min_val, max_val) \
    do { if ((val) < (min_val)) (val) = (min_val); \
         if ((val) > (max_val)) (val) = (max_val); } while(0)

void AdaptivePID_Init(AdaptivePID_t *pid)
{
    /* 默认参数范围 */
    pid->Kp_init = 1.0f;  pid->Kp_min = 0.01f;  pid->Kp_max = 100.0f;
    pid->Ki_init = 0.0f;  pid->Ki_min = 0.0f;    pid->Ki_max = 50.0f;
    pid->Kd_init = 0.0f;  pid->Kd_min = 0.0f;    pid->Kd_max = 10.0f;

    pid->Kp = pid->Kp_init;
    pid->Ki = pid->Ki_init;
    pid->Kd = pid->Kd_init;

    /* 默认自适应参数 */
    pid->method = ADAPTIVE_GRADIENT;
    pid->learning_rate_p = 0.01f;
    pid->learning_rate_i = 0.005f;
    pid->learning_rate_d = 0.002f;
    pid->alpha = 0.1f;
    pid->deadband = 0.01f;

    /* 输出限幅 */
    pid->out_min = -1000.0f;
    pid->out_max =  1000.0f;
    pid->integral_max = 500.0f;
    pid->dt = 0.001f; /* 默认1ms */

    /* 清零状态 */
    AdaptivePID_Reset(pid);
}

void AdaptivePID_SetMethod(AdaptivePID_t *pid, AdaptiveMethod_e method)
{
    pid->method = method;
}

void AdaptivePID_SetParamRange(AdaptivePID_t *pid,
                                float kp_min, float kp_max,
                                float ki_min, float ki_max,
                                float kd_min, float kd_max)
{
    pid->Kp_min = kp_min; pid->Kp_max = kp_max;
    pid->Ki_min = ki_min; pid->Ki_max = ki_max;
    pid->Kd_min = kd_min; pid->Kd_max = kd_max;
}

void AdaptivePID_SetLearningRate(AdaptivePID_t *pid,
                                  float lr_p, float lr_i, float lr_d)
{
    pid->learning_rate_p = lr_p;
    pid->learning_rate_i = lr_i;
    pid->learning_rate_d = lr_d;
}

/* 梯度下降法调整参数 */
static void _GradientAdapt(AdaptivePID_t *pid, float error)
{
    float de = error - pid->error_last;

    /* dJ/dKp = e * e (P项输出对Kp的偏导近似为e) */
    float dkp = -pid->learning_rate_p * error * error;

    /* dJ/dKi = e * integral */
    float dki = -pid->learning_rate_i * error * pid->integral;

    /* dJ/dKd = e * de/dt */
    float dkd = -pid->learning_rate_d * error * de;

    pid->Kp += dkp;
    pid->Ki += dki;
    pid->Kd += dkd;

    /* 限幅 */
    CLAMP(pid->Kp, pid->Kp_min, pid->Kp_max);
    CLAMP(pid->Ki, pid->Ki_min, pid->Ki_max);
    CLAMP(pid->Kd, pid->Kd_min, pid->Kd_max);
}

/* MIT规则调整参数 */
static void _MITAdapt(AdaptivePID_t *pid, float error, float plant_output)
{
    /* 估计 ∂y/∂u (雅可比) */
    float du = pid->output_last; /* 简化：用上次输出变化近似 */
    float dy = plant_output - pid->plant_output_last;
    float jacobian = (fabsf(du) > 1e-6f) ? (dy / du) : 0.0f;

    float common = -pid->alpha * error * jacobian;

    /* P项自适应 */
    pid->Kp += common * error;
    /* I项自适应 */
    pid->Ki += common * pid->integral;
    /* D项自适应 */
    pid->Kd += common * (error - pid->error_last);

    CLAMP(pid->Kp, pid->Kp_min, pid->Kp_max);
    CLAMP(pid->Ki, pid->Ki_min, pid->Ki_max);
    CLAMP(pid->Kd, pid->Kd_min, pid->Kd_max);
}

float AdaptivePID_Compute(AdaptivePID_t *pid, float target, float measurement)
{
    float error = target - measurement;

    /* 死区判断：误差很小时不自适应，避免参数漂移 */
    int do_adapt = (fabsf(error) > pid->deadband) ? 1 : 0;

    /* 积分项（带限幅） */
    pid->integral += error * pid->dt;
    CLAMP(pid->integral, -pid->integral_max, pid->integral_max);

    /* 微分项 */
    float derivative = (error - pid->error_last) / pid->dt;

    /* PID输出 */
    float output = pid->Kp * error
                 + pid->Ki * pid->integral
                 + pid->Kd * derivative;

    /* 输出限幅 */
    CLAMP(output, pid->out_min, pid->out_max);

    /* 在线自适应调整参数 */
    if (do_adapt) {
        if (pid->method == ADAPTIVE_GRADIENT) {
            _GradientAdapt(pid, error);
        } else {
            _MITAdapt(pid, error, measurement);
        }
    }

    /* 更新历史 */
    pid->error_prev = pid->error_last;
    pid->error_last = error;
    pid->plant_output_last = measurement;
    pid->output_last = output;

    return output;
}

void AdaptivePID_Reset(AdaptivePID_t *pid)
{
    pid->error = 0.0f;
    pid->error_last = 0.0f;
    pid->error_prev = 0.0f;
    pid->integral = 0.0f;
    pid->output_last = 0.0f;
    pid->plant_output_last = 0.0f;

    /* 恢复初始参数 */
    pid->Kp = pid->Kp_init;
    pid->Ki = pid->Ki_init;
    pid->Kd = pid->Kd_init;
}

void AdaptivePID_GetParams(AdaptivePID_t *pid, float *kp, float *ki, float *kd)
{
    if (kp) *kp = pid->Kp;
    if (ki) *ki = pid->Ki;
    if (kd) *kd = pid->Kd;
}
