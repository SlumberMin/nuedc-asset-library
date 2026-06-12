/**
 * @file smc_sliding_mode.c
 * @brief 滑模控制器V2实现 - 支持多种趋近律
 *
 * 控制律设计：
 * u = u_eq + u_sw
 *
 * 等效控制 u_eq：保持系统在滑模面上的连续控制
 * 切换控制 u_sw：驱使系统到达滑模面的不连续控制
 *
 * 趋近律选择直接影响抖振程度和收敛速度。
 */

#include "smc_sliding_mode.h"
#include <math.h>
#include <string.h>

#define CLAMP(val, min_val, max_val) \
    do { if ((val) < (min_val)) (val) = (min_val); \
         if ((val) > (max_val)) (val) = (max_val); } while(0)

/* 符号函数 */
static float _sign(float x)
{
    if (x > 0.0f) return 1.0f;
    if (x < 0.0f) return -1.0f;
    return 0.0f;
}

/* 饱和函数（替代符号函数，连续化） */
static float _sat(float x, float delta)
{
    if (delta < 1e-6f) delta = 1e-6f;
    if (x > delta) return 1.0f;
    if (x < -delta) return -1.0f;
    return x / delta;
}

void SMC_Init(SMC_t *ctrl)
{
    memset(ctrl, 0, sizeof(SMC_t));

    /* 默认参数 */
    ctrl->surface_type = SMC_SURFACE_LINEAR;
    ctrl->c = 10.0f;
    ctrl->alpha_s = 0.5f;
    ctrl->ki_s = 1.0f;

    ctrl->reaching_law = SMC_REACHING_EXPONENTIAL;
    ctrl->epsilon = 5.0f;
    ctrl->k_reach = 10.0f;
    ctrl->alpha_r = 0.5f;
    ctrl->delta = 0.1f;

    ctrl->epsilon_min = 0.1f;
    ctrl->epsilon_max = 20.0f;
    ctrl->adapt_rate = 0.5f;
    ctrl->epsilon_cur = 5.0f;

    ctrl->eq_gain = 1.0f;
    ctrl->out_min = -1000.0f;
    ctrl->out_max =  1000.0f;
    ctrl->dt = 0.001f;
}

void SMC_SetSurface(SMC_t *ctrl, SMCSurface_e type,
                     float c, float alpha_s, float ki_s)
{
    ctrl->surface_type = type;
    ctrl->c = c;
    ctrl->alpha_s = alpha_s;
    ctrl->ki_s = ki_s;
}

void SMC_SetReachingLaw(SMC_t *ctrl, SMCReachingLaw_e law,
                         float epsilon, float k_reach, float alpha_r)
{
    ctrl->reaching_law = law;
    ctrl->epsilon = epsilon;
    ctrl->k_reach = k_reach;
    ctrl->alpha_r = alpha_r;
}

void SMC_SetBoundaryLayer(SMC_t *ctrl, float delta)
{
    ctrl->delta = (delta > 1e-4f) ? delta : 1e-4f;
}

void SMC_SetAdaptiveParam(SMC_t *ctrl,
                           float eps_min, float eps_max, float adapt_rate)
{
    ctrl->epsilon_min = eps_min;
    ctrl->epsilon_max = eps_max;
    ctrl->adapt_rate = adapt_rate;
}

void SMC_SetEqGain(SMC_t *ctrl, float gain)
{
    ctrl->eq_gain = gain;
}

void SMC_SetLimit(SMC_t *ctrl, float out_min, float out_max)
{
    ctrl->out_min = out_min;
    ctrl->out_max = out_max;
}

/* 计算滑模面值 */
static float _ComputeSurface(SMC_t *ctrl, float error, float error_dot)
{
    float s = 0.0f;

    switch (ctrl->surface_type) {
    case SMC_SURFACE_LINEAR:
        /* s = c*e + de */
        s = ctrl->c * error + error_dot;
        break;

    case SMC_SURFACE_NONLINEAR:
        /* s = c*|e|^α*sign(e) + de */
        s = ctrl->c * powf(fabsf(error), ctrl->alpha_s) * _sign(error)
            + error_dot;
        break;

    case SMC_SURFACE_INTEGRAL:
        /* s = c*e + de + ki*∫e */
        ctrl->integral += error * ctrl->dt;
        CLAMP(ctrl->integral, -100.0f, 100.0f);
        s = ctrl->c * error + error_dot + ctrl->ki_s * ctrl->integral;
        break;
    }

    return s;
}

/* 计算趋近律（切换控制项） */
static float _ComputeReachingControl(SMC_t *ctrl, float s)
{
    float u_sw = 0.0f;

    switch (ctrl->reaching_law) {
    case SMC_REACHING_CONSTANT:
        /* 等速趋近律：-ε*sign(s) */
        u_sw = -ctrl->epsilon * _sign(s);
        break;

    case SMC_REACHING_EXPONENTIAL:
        /* 指数趋近律：-ε*sign(s) - k*s */
        u_sw = -ctrl->epsilon * _sign(s) - ctrl->k_reach * s;
        break;

    case SMC_REACHING_POWER:
        /* 幂次趋近律：-k*|s|^α*sign(s) */
        u_sw = -ctrl->k_reach * powf(fabsf(s), ctrl->alpha_r) * _sign(s);
        break;

    case SMC_REACHING_COMBINED:
        /* 组合趋近律：-ε*sign(s) - k*|s|^α*sign(s) */
        u_sw = -ctrl->epsilon * _sign(s)
             - ctrl->k_reach * powf(fabsf(s), ctrl->alpha_r) * _sign(s);
        break;

    case SMC_REACHING_ADAPTIVE:
        /* 自适应趋近律：ε根据|s|自适应调整 */
        /* dε/dt = adapt_rate * (|s| - ε) */
        ctrl->epsilon_cur += ctrl->adapt_rate * (fabsf(s) - ctrl->epsilon_cur) * ctrl->dt;
        CLAMP(ctrl->epsilon_cur, ctrl->epsilon_min, ctrl->epsilon_max);
        u_sw = -ctrl->epsilon_cur * _sign(s) - ctrl->k_reach * s;
        break;

    case SMC_REACHING_SAT:
        /* 饱和函数替代符号函数：-ε*sat(s/δ) - k*s */
        u_sw = -ctrl->epsilon * _sat(s, ctrl->delta) - ctrl->k_reach * s;
        break;
    }

    return u_sw;
}

float SMC_Compute(SMC_t *ctrl, float target, float measurement,
                   float *target_dot)
{
    float error = target - measurement;
    float error_dot;

    /* 计算误差导数 */
    if (target_dot) {
        error_dot = *target_dot - (measurement - ctrl->error_last) / ctrl->dt;
    } else {
        error_dot = (error - ctrl->error_last) / ctrl->dt;
    }

    /* 计算滑模面 */
    float s = _ComputeSurface(ctrl, error, error_dot);

    /* 等效控制：基于滑模面的连续控制 */
    float s_dot = (s - ctrl->s_last) / ctrl->dt;
    float u_eq = ctrl->eq_gain * s_dot; /* 简化等效控制 */

    /* 切换控制：基于趋近律 */
    float u_sw = _ComputeReachingControl(ctrl, s);

    /* 总控制量 */
    float u = u_eq + u_sw;

    /* 输出限幅 */
    CLAMP(u, ctrl->out_min, ctrl->out_max);

    /* 更新状态 */
    ctrl->error_last = ctrl->error;
    ctrl->error = error;
    ctrl->error_dot = error_dot;
    ctrl->s_last = ctrl->s;
    ctrl->s = s;
    ctrl->s_dot = s_dot;

    return u;
}

float SMC_GetSlidingSurface(SMC_t *ctrl)
{
    return ctrl->s;
}

void SMC_Reset(SMC_t *ctrl)
{
    ctrl->error = 0.0f;
    ctrl->error_last = 0.0f;
    ctrl->error_dot = 0.0f;
    ctrl->s = 0.0f;
    ctrl->s_last = 0.0f;
    ctrl->s_dot = 0.0f;
    ctrl->integral = 0.0f;
    ctrl->epsilon_cur = ctrl->epsilon;
}
