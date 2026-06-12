/**
 * @file    fuzzy_pid.c
 * @brief   模糊自适应PID控制器实现
 * 
 * 模糊规则表（7×7=49条规则）：
 * 
 * 误差e的模糊集：NB(负大), NM(负中), NS(负小), ZO(零), PS(正小), PM(正中), PB(正大)
 * 误差变化率ec的模糊集：同上
 * 输出：ΔKp, ΔKi, ΔKd 的调整量
 * 
 * ΔKp规则表（示例）：
 * ec\e  | NB   NM   NS   ZO   PS   PM   PB
 * ------+--------------------------------
 * NB    | PB   PB   PM   PM   PS   ZO   ZO
 * NM    | PB   PB   PM   PS   PS   ZO   NS
 * NS    | PM   PM   PM   PS   ZO   NS   NS
 * ZO    | PM   PM   PS   ZO   NS   NM   NM
 * PS    | PS   PS   ZO   NS   NS   NM   NM
 * PM    | PS   ZO   NS   NM   NM   NM   NB
 * PB    | ZO   ZO   NM   NM   NM   NB   NB
 */

#include "fuzzy_pid.h"
#include <math.h>

/* 模糊隶属度函数 */
static void FuzzyMembership(float x, float *mu_neg, float *mu_zero, float *mu_pos)
{
    /* 三角隶属度函数 */
    if(x <= -1.0f) { *mu_neg = 1.0f; *mu_zero = 0; *mu_pos = 0; }
    else if(x <= 0) { *mu_neg = -x; *mu_zero = 1+x; *mu_pos = 0; }
    else if(x <= 1.0f) { *mu_neg = 0; *mu_zero = 1-x; *mu_pos = x; }
    else { *mu_neg = 0; *mu_zero = 0; *mu_pos = 1.0f; }
}

/* ΔKp模糊规则表 */
static const float dKp_rules[7][7] = {
    { 6, 6, 5, 5, 4, 2, 2},  /* NB */
    { 6, 6, 5, 4, 4, 2, 1},  /* NM */
    { 5, 5, 5, 4, 2, 1, 1},  /* NS */
    { 5, 5, 4, 2, 1, 0, 0},  /* ZO */
    { 4, 4, 2, 1, 1, 0, 0},  /* PS */
    { 4, 2, 1, 0, 0, 0,-1},  /* PM */
    { 2, 2, 0, 0, 0,-1,-1},  /* PB */
};

/* ΔKi模糊规则表 */
static const float dKi_rules[7][7] = {
    {-6,-6,-5,-5,-4,-2,-2},
    {-6,-6,-5,-4,-4,-2, 0},
    {-5,-5,-4,-4,-2, 0, 0},
    {-5,-5,-4, 0, 2, 4, 5},
    {-4,-4,-2, 0, 4, 5, 5},
    {-2, 0, 2, 4, 5, 6, 6},
    { 0, 2, 4, 5, 6, 6, 6},
};

/* ΔKd模糊规则表 */
static const float dKd_rules[7][7] = {
    { 2, 2, 1, 1, 0,-1,-2},
    { 2, 2, 1, 1, 0,-1,-2},
    { 1, 1, 1, 0,-1,-1,-2},
    { 1, 1, 0,-1,-1,-2,-2},
    { 0, 0,-1,-1,-2,-2,-4},
    { 0,-1,-1,-2,-2,-4,-4},
    {-2,-2,-2,-2,-4,-4,-6},
};

void FuzzyPID_Init(FuzzyPID_t *fp, float kp, float ki, float kd,
                    float dKp, float dKi, float dKd,
                    float e_factor, float ec_factor,
                    float out_min, float out_max)
{
    fp->Kp0 = kp; fp->Ki0 = ki; fp->Kd0 = kd;
    fp->dKp_max = dKp; fp->dKi_max = dKi; fp->dKd_max = dKd;
    fp->e_factor = e_factor; fp->ec_factor = ec_factor;
    fp->integral = 0; fp->prev_error = 0;
    fp->filtered_d = 0; fp->d_filter_alpha = 0.3f;
    fp->out_min = out_min; fp->out_max = out_max;
    fp->integral_max = (out_max - out_min) * 0.5f;
    fp->output = 0;
}

float FuzzyPID_Calculate(FuzzyPID_t *fp, float target, float actual)
{
    float error = target - actual;
    float ec = error - fp->prev_error;
    
    /* 量化 */
    float e_fuzzy = error * fp->e_factor;
    float ec_fuzzy = ec * fp->ec_factor;
    if(e_fuzzy > 1.0f) e_fuzzy = 1.0f;
    if(e_fuzzy < -1.0f) e_fuzzy = -1.0f;
    if(ec_fuzzy > 1.0f) ec_fuzzy = 1.0f;
    if(ec_fuzzy < -1.0f) ec_fuzzy = -1.0f;
    
    /* 模糊推理（简化：直接查表+插值） */
    int e_idx = (int)((e_fuzzy + 1.0f) * 3.0f);
    int ec_idx = (int)((ec_fuzzy + 1.0f) * 3.0f);
    if(e_idx < 0) e_idx = 0; if(e_idx > 6) e_idx = 6;
    if(ec_idx < 0) ec_idx = 0; if(ec_idx > 6) ec_idx = 6;
    
    float dKp = dKp_rules[e_idx][ec_idx] * fp->dKp_max / 6.0f;
    float dKi = dKi_rules[e_idx][ec_idx] * fp->dKi_max / 6.0f;
    float dKd = dKd_rules[e_idx][ec_idx] * fp->dKd_max / 6.0f;
    
    /* 自适应PID参数 */
    float Kp = fp->Kp0 + dKp;
    float Ki = fp->Ki0 + dKi;
    float Kd = fp->Kd0 + dKd;
    if(Kp < 0) Kp = 0;
    if(Ki < 0) Ki = 0;
    if(Kd < 0) Kd = 0;
    
    /* PID计算 */
    fp->integral += error;
    if(fp->integral > fp->integral_max) fp->integral = fp->integral_max;
    if(fp->integral < -fp->integral_max) fp->integral = -fp->integral_max;
    
    float raw_d = error - fp->prev_error;
    fp->filtered_d = fp->d_filter_alpha * raw_d + (1-fp->d_filter_alpha) * fp->filtered_d;
    
    fp->output = Kp * error + Ki * fp->integral + Kd * fp->filtered_d;
    if(fp->output > fp->out_max) fp->output = fp->out_max;
    if(fp->output < fp->out_min) fp->output = fp->out_min;
    
    fp->prev_error = error;
    return fp->output;
}
