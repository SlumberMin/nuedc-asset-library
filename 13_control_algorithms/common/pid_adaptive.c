/**
 * @file pid_adaptive_v13.c
 * @brief 自适应PID V13 实现
 */

#include "pid_adaptive_v13.h"
#include <math.h>
#include <string.h>

/* 内部宏 */
#define CLAMP(val, min_val, max_val) \
    do { if ((val) < (min_val)) (val) = (min_val); \
         else if ((val) > (max_val)) (val) = (max_val); } while(0)

#define SIGN(x) ((x) >= 0.0f ? 1.0f : -1.0f)

 /*---------------------------------------------------------------------------
  * 内部函数
  *---------------------------------------------------------------------------*/

/**
 * @brief 基于误差幅值的增益调度
 */
static void s_CalcGainSchedule(PID_AdaptiveV13_t *pid)
{
    float abs_error = fabsf(pid->error);
    const AdaptiveGainSchedule_t *gs = &pid->gain_schedule;
    int idx = 3;  /* 默认最大区间 */

    if (abs_error < gs->error_threshold[0]) {
        idx = 0;  /* 小误差区 */
    } else if (abs_error < gs->error_threshold[1]) {
        idx = 1;  /* 中误差区 */
    } else if (abs_error < gs->error_threshold[2]) {
        idx = 2;  /* 大误差区 */
    }

    float target_kp = pid->Kp * gs->kp_scale[idx];
    float target_ki = pid->Ki * gs->ki_scale[idx];
    float target_kd = pid->Kd * gs->kd_scale[idx];

    /* 参数变化速率限制 */
    float max_delta = pid->param_change_rate_max * pid->dt;

    float delta_kp = target_kp - pid->adaptive_kp;
    float delta_ki = target_ki - pid->adaptive_ki;
    float delta_kd = target_kd - pid->adaptive_kd;

    CLAMP(delta_kp, -max_delta, max_delta);
    CLAMP(delta_ki, -max_delta, max_delta);
    CLAMP(delta_kd, -max_delta, max_delta);

    pid->adaptive_kp += delta_kp;
    pid->adaptive_ki += delta_ki;
    pid->adaptive_kd += delta_kd;
}

/**
 * @brief 基于误差梯度的参数微调
 */
static void s_GradientAdaptation(PID_AdaptiveV13_t *pid)
{
    /* 计算误差梯度 */
    pid->error_gradient = (pid->error - pid->error_prev) / pid->dt;

    /* 一阶低通平滑 */
    pid->gradient_smooth = pid->gradient_alpha * pid->error_gradient
                         + (1.0f - pid->gradient_alpha) * pid->gradient_smooth;

    /* 梯度自适应: 误差快速减小时减小增益，快速增大时增大增益 */
    float error_grad_sign = SIGN(pid->error);
    float grad_sign = SIGN(pid->gradient_smooth);

    /* 误差与梯度同号 -> 误差在增大 -> 增大增益 */
    if (error_grad_sign * grad_sign > 0.0f) {
        float boost = 1.0f + 0.3f * fabsf(pid->gradient_smooth) * pid->dt;
        CLAMP(boost, 1.0f, 2.0f);
        pid->adaptive_kp *= boost;
    } else {
        float damp = 1.0f - 0.1f * fabsf(pid->gradient_smooth) * pid->dt;
        CLAMP(damp, 0.5f, 1.0f);
        pid->adaptive_kp *= damp;
    }
}

/**
 * @brief 条件积分处理
 */
static float s_ConditionalIntegral(PID_AdaptiveV13_t *pid)
{
    float abs_error = fabsf(pid->error);

    /* 积分分离 */
    if (abs_error > pid->integral_separate_threshold) {
        pid->integral_enable = 0;
    } else {
        pid->integral_enable = 1;
    }

    if (!pid->integral_enable) {
        return 0.0f;
    }

    /* 条件积分: 输出饱和且误差同向时不积分 */
    float tentative_output = pid->adaptive_kp * pid->error + pid->adaptive_ki * pid->integral;
    if ((tentative_output > pid->output_max && pid->error > 0.0f) ||
        (tentative_output < pid->output_min && pid->error < 0.0f)) {
        /* 抗饱和: 不累加积分 */
        return pid->integral;
    }

    pid->integral += pid->error * pid->dt;
    CLAMP(pid->integral, -pid->integral_max, pid->integral_max);

    return pid->integral;
}

 /*---------------------------------------------------------------------------
  * 公开接口实现
  *---------------------------------------------------------------------------*/

void PID_AdaptiveV13_Init(PID_AdaptiveV13_t *pid, float Kp, float Ki, float Kd, float dt)
{
    memset(pid, 0, sizeof(PID_AdaptiveV13_t));

    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->dt = dt > 0.0f ? dt : 0.001f;

    pid->adaptive_kp = Kp;
    pid->adaptive_ki = Ki;
    pid->adaptive_kd = Kd;

    pid->output_max = 1000.0f;
    pid->output_min = -1000.0f;
    pid->integral_max = 500.0f;
    pid->integral_separate_threshold = 100.0f;

    pid->gradient_alpha = 0.1f;
    pid->param_change_rate_max = 100.0f;

    pid->mode = ADAPTIVE_MODE_POSITION;
    pid->derivative_on_measurement = 0;
    pid->integral_enable = 1;

    /* 默认增益调度表: 各区间增益倍率均为1.0 */
    for (int i = 0; i < 3; i++) {
        pid->gain_schedule.error_threshold[i] = (float)(i + 1) * 50.0f;
    }
    for (int i = 0; i < 4; i++) {
        pid->gain_schedule.kp_scale[i] = 1.0f;
        pid->gain_schedule.ki_scale[i] = 1.0f;
        pid->gain_schedule.kd_scale[i] = 1.0f;
    }
}

void PID_AdaptiveV13_SetGainSchedule(PID_AdaptiveV13_t *pid, const AdaptiveGainSchedule_t *schedule)
{
    if (pid && schedule) {
        memcpy(&pid->gain_schedule, schedule, sizeof(AdaptiveGainSchedule_t));
    }
}

void PID_AdaptiveV13_SetMode(PID_AdaptiveV13_t *pid, AdaptiveMode_e mode)
{
    if (pid) {
        pid->mode = mode;
    }
}

void PID_AdaptiveV13_SetOutputLimit(PID_AdaptiveV13_t *pid, float min, float max)
{
    if (pid) {
        pid->output_min = min;
        pid->output_max = max;
    }
}

void PID_AdaptiveV13_SetIntegralLimit(PID_AdaptiveV13_t *pid, float max)
{
    if (pid) {
        pid->integral_max = max > 0.0f ? max : 0.0f;
    }
}

void PID_AdaptiveV13_SetIntegralSeparate(PID_AdaptiveV13_t *pid, float threshold)
{
    if (pid) {
        pid->integral_separate_threshold = threshold > 0.0f ? threshold : 0.0f;
    }
}

void PID_AdaptiveV13_SetGradientSmooth(PID_AdaptiveV13_t *pid, float alpha)
{
    if (pid) {
        CLAMP(alpha, 0.0f, 1.0f);
        pid->gradient_alpha = alpha;
    }
}

void PID_AdaptiveV13_EnableDerivativeOnMeasurement(PID_AdaptiveV13_t *pid)
{
    if (pid) {
        pid->derivative_on_measurement = 1;
    }
}

float PID_AdaptiveV13_Calculate(PID_AdaptiveV13_t *pid, float setpoint, float measurement)
{
    if (!pid) return 0.0f;

    pid->measurement = measurement;
    pid->error = setpoint - measurement;

    /* 增益调度 */
    s_CalcGainSchedule(pid);

    /* 梯度自适应 */
    s_GradientAdaptation(pid);

    /* 条件积分 */
    float integral_term = s_ConditionalIntegral(pid) * pid->adaptive_ki;

    /* 微分计算 */
    if (pid->derivative_on_measurement) {
        /* 微分先行: 微分仅作用于测量值 */
        pid->derivative = -(pid->measurement - pid->measurement_prev) / pid->dt;
    } else {
        pid->derivative = (pid->error - pid->error_prev) / pid->dt;
    }

    /* 一阶低通滤波微分 */
    static const float deriv_alpha = 0.1f;
    /* 使用简单低通, 复用gradient_smooth变量不合适, 直接使用derivative */

    float derivative_term = pid->adaptive_kd * pid->derivative;

    if (pid->mode == ADAPTIVE_MODE_POSITION) {
        /* 位置式 */
        pid->output = pid->adaptive_kp * pid->error + integral_term + derivative_term;
    } else {
        /* 增量式 */
        float delta_output = pid->adaptive_kp * (pid->error - pid->error_prev)
                           + pid->adaptive_ki * pid->error * pid->dt
                           + pid->adaptive_kd * (pid->error - 2.0f * pid->error_prev + pid->error_prev2) / pid->dt;
        pid->output += delta_output;
    }

    /* 输出限幅 */
    CLAMP(pid->output, pid->output_min, pid->output_max);

    /* 更新历史 */
    pid->error_prev2 = pid->error_prev;
    pid->error_prev = pid->error;
    pid->measurement_prev = pid->measurement;

    return pid->output;
}

void PID_AdaptiveV13_Reset(PID_AdaptiveV13_t *pid)
{
    if (!pid) return;

    pid->error = 0.0f;
    pid->error_prev = 0.0f;
    pid->error_prev2 = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->output = 0.0f;
    pid->error_gradient = 0.0f;
    pid->gradient_smooth = 0.0f;
    pid->measurement_prev = 0.0f;
    pid->integral_enable = 1;

    /* 恢复基础参数 */
    pid->adaptive_kp = pid->Kp;
    pid->adaptive_ki = pid->Ki;
    pid->adaptive_kd = pid->Kd;
}

void PID_AdaptiveV13_GetAdaptiveParams(const PID_AdaptiveV13_t *pid, float *Kp, float *Ki, float *Kd)
{
    if (!pid) return;
    if (Kp) *Kp = pid->adaptive_kp;
    if (Ki) *Ki = pid->adaptive_ki;
    if (Kd) *Kd = pid->adaptive_kd;
}
