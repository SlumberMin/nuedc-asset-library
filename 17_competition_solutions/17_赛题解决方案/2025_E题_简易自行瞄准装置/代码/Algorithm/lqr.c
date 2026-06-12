/**
 * @file    lqr.c
 * @brief   LQR线性二次最优控制器实现
 * 
 * 使用方法：
 * 1. 在MATLAB中求解Riccati方程得到K矩阵
 * 2. 在STM32中使用LQR_SetGain设置K
 * 3. 每个控制周期更新状态，调用LQR_Calculate
 * 
 * MATLAB求解K的代码：
 * [K, S, e] = lqr(A, B, Q, R);
 * 其中A/B为状态空间模型，Q/R为权重矩阵
 */

#include "lqr.h"

void LQR_Init(LQR_t *lqr, uint8_t n, uint8_t m, float u_min, float u_max)
{
    lqr->n = n;
    lqr->m = m;
    lqr->u_min = u_min;
    lqr->u_max = u_max;
    lqr->output = 0;
    for(uint8_t i = 0; i < LQR_MAX_STATES; i++)
    {
        lqr->K[i] = 0;
        lqr->x[i] = 0;
    }
}

void LQR_SetGain(LQR_t *lqr, const float *K)
{
    for(uint8_t i = 0; i < lqr->n; i++)
    {
        lqr->K[i] = K[i];
    }
}

void LQR_SetState(LQR_t *lqr, uint8_t idx, float value)
{
    if(idx < lqr->n)
    {
        lqr->x[idx] = value;
    }
}

float LQR_Calculate(LQR_t *lqr)
{
    float u = 0;
    for(uint8_t i = 0; i < lqr->n; i++)
    {
        u -= lqr->K[i] * lqr->x[i];
    }
    
    if(u > lqr->u_max) u = lqr->u_max;
    if(u < lqr->u_min) u = lqr->u_min;
    
    lqr->output = u;
    return u;
}

void LQR_Reset(LQR_t *lqr)
{
    for(uint8_t i = 0; i < LQR_MAX_STATES; i++)
    {
        lqr->x[i] = 0;
    }
    lqr->output = 0;
}
