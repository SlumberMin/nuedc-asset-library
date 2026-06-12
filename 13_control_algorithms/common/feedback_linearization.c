/**
 * @file feedback_linearization.c
 * @brief 反馈线性化控制器实现
 * 
 * 实现仿射非线性系统的反馈线性化控制。
 * 用户需提供系统向量场f(x), g(x)和输出函数h(x)。
 */

#include "feedback_linearization.h"
#include <string.h>

int FL_Init(FeedLinCtrl_t *fl, int32_t n, int32_t rel_degree, float Ts)
{
    if (!fl || n < 1 || n > FL_MAX_STATES || rel_degree < 1 || Ts <= 0)
        return -1;

    fl->n          = n;
    fl->rel_degree = rel_degree;
    fl->Ts         = Ts;
    fl->Kp         = 10.0f;
    fl->Kd         = 5.0f;
    fl->Ki         = 0.0f;
    fl->f_func     = NULL;
    fl->g_func     = NULL;
    fl->h_func     = NULL;
    fl->user_data  = NULL;

    FL_Reset(fl);
    return 0;
}

void FL_SetModel(FeedLinCtrl_t *fl, FL_VectorFunc_t f, FL_VectorFunc_t g, FL_ScalarFunc_t h)
{
    if (!fl) return;
    fl->f_func = f;
    fl->g_func = g;
    fl->h_func = h;
}

void FL_SetPDGains(FeedLinCtrl_t *fl, float Kp, float Kd)
{
    if (!fl) return;
    fl->Kp = Kp;
    fl->Kd = Kd;
    fl->Ki = 0.0f;
}

void FL_SetPIDGains(FeedLinCtrl_t *fl, float Kp, float Kd, float Ki)
{
    if (!fl) return;
    fl->Kp = Kp;
    fl->Kd = Kd;
    fl->Ki = Ki;
}

/**
 * @brief 数值计算Lie导数 Lf*h ≈ (h(x+eps*f) - h(x)) / eps
 * 注意：实际工程中应由用户直接提供解析表达式以提高精度和效率
 */
static float FL_NumericalLieDeriv(FeedLinCtrl_t *fl, const float *x,
                                   FL_VectorFunc_t vector_field, FL_ScalarFunc_t h)
{
    float eps = 1e-4f;
    float x_temp[FL_MAX_STATES];
    float f_val[FL_MAX_STATES];

    if (!vector_field || !h) return 0.0f;

    /* 计算向量场 f(x) */
    vector_field(x, f_val, fl->user_data);

    /* x_temp = x + eps * f(x) */
    for (int i = 0; i < fl->n; i++)
        x_temp[i] = x[i] + eps * f_val[i];

    return (h(x_temp, fl->user_data) - h(x, fl->user_data)) / eps;
}

float FL_Compute(FeedLinCtrl_t *fl, const float *x, float ref)
{
    if (!fl || !x) return 0.0f;

    /* 保存状态 */
    for (int i = 0; i < fl->n; i++)
        fl->x[i] = x[i];

    float y = (fl->h_func) ? fl->h_func(x, fl->user_data) : x[0];
    float err = ref - y;

    /* 积分项 */
    fl->integral_err += err * fl->Ts;

    /* 线性化后控制律: v = Kp*err + Kd*d(err)/dt + Ki*integral */
    float d_err = (y - fl->y_prev) / fl->Ts;  /* dy/dt近似 */
    float v = fl->Kp * err - fl->Kd * d_err + fl->Ki * fl->integral_err;

    /* 反馈线性化逆变换:
     * u = (v - Lf^r*h) / (Lg*Lf^(r-1)*h)
     * 这里简化为相对阶=1的情况:
     * u = (v - Lf*h) / (Lg*h)
     */
    float Lf_h = FL_NumericalLieDeriv(fl, x, fl->f_func, fl->h_func);
    float Lg_h = FL_NumericalLieDeriv(fl, x, fl->g_func, fl->h_func);

    float u_out;
    if (Lg_h > 1e-6f || Lg_h < -1e-6f)
        u_out = (v - Lf_h) / Lg_h;
    else
        u_out = v;  /* 退化为直接线性控制 */

    fl->y_prev = y;
    return u_out;
}

void FL_Reset(FeedLinCtrl_t *fl)
{
    if (!fl) return;
    memset(fl->x, 0, sizeof(fl->x));
    fl->integral_err = 0.0f;
    fl->y_prev       = 0.0f;
}
