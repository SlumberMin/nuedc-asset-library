/**
 * @file repetitive_control.c
 * @brief 重复控制器实现 - 周期性扰动抑制
 */

#include "repetitive_control.h"
#include <string.h>
#include <math.h>

int RC_Init(RepetitiveCtrl_t *rc, float Ts, float f0, float Kr, float Q)
{
    if (!rc || Ts <= 0 || f0 <= 0 || Q < 0 || Q > 1.0f)
        return -1;

    rc->Ts = Ts;
    rc->f0 = f0;
    rc->Kr = Kr;
    rc->Q  = Q;

    /* 计算每周期采样点数 N = round(1/(f0*Ts)) */
    rc->N = (int32_t)(1.0f / (f0 * Ts) + 0.5f);
    if (rc->N < 1) rc->N = 1;
    if (rc->N > RC_MAX_PERIOD_SAMPLES) rc->N = RC_MAX_PERIOD_SAMPLES;

    RC_Reset(rc);
    return 0;
}

float RC_Compute(RepetitiveCtrl_t *rc, float ref, float fbk)
{
    float err = ref - fbk;
    float u_delayed, err_delayed;
    float u_out;

    /* 取出一个周期前的控制量和误差(环形缓冲) */
    u_delayed   = rc->delay_buf[rc->idx];
    err_delayed = rc->err_buf[rc->idx];  /* e(k-N): 取N个采样前的误差 */

    /* 核心递推: u(k) = Q * u(k-N) + Kr * e(k-N) */
    u_out = rc->Q * u_delayed + rc->Kr * err_delayed;

    /* 存入当前控制量和误差到环形缓冲 */
    rc->delay_buf[rc->idx] = u_out;
    rc->err_buf[rc->idx]   = err;

    /* 更新索引 */
    rc->idx++;
    if (rc->idx >= rc->N)
        rc->idx = 0;

    rc->u_prev   = u_out;
    rc->err_prev = err;

    return u_out;
}

void RC_Reset(RepetitiveCtrl_t *rc)
{
    if (!rc) return;
    memset(rc->delay_buf, 0, sizeof(rc->delay_buf));
    memset(rc->err_buf, 0, sizeof(rc->err_buf));
    rc->idx      = 0;
    rc->u_prev   = 0.0f;
    rc->err_prev = 0.0f;
}
