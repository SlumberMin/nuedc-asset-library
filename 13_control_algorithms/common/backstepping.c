/**
 * @file backstepping.c
 * @brief 反步法控制器实现 - 严格反馈非线性系统
 * 
 * 实现2~5阶严格反馈非线性系统的反步法控制。
 * 用户需提供系统的f_i(x)和g_i(x)模型函数。
 */

#include "backstepping.h"
#include <string.h>

int BS_Init(BacksteppingCtrl_t *bs, int32_t order, float Ts)
{
    if (!bs || order < 2 || order > BS_MAX_ORDER || Ts <= 0)
        return -1;

    bs->order = order;
    bs->Ts    = Ts;
    bs->user_data = NULL;

    /* 默认增益 */
    for (int i = 0; i < BS_MAX_ORDER; i++)
        bs->c[i] = 5.0f;

    memset(bs->f, 0, sizeof(bs->f));
    memset(bs->g, 0, sizeof(bs->g));

    BS_Reset(bs);
    return 0;
}

void BS_SetModel(BacksteppingCtrl_t *bs, int32_t step, BS_Func_t f_func, BS_Func_t g_func)
{
    if (!bs || step < 0 || step >= bs->order) return;
    bs->f[step] = f_func;
    bs->g[step] = g_func;
}

void BS_SetGains(BacksteppingCtrl_t *bs, const float *c)
{
    if (!bs || !c) return;
    for (int i = 0; i < bs->order; i++)
        bs->c[i] = c[i];
}

float BS_Compute(BacksteppingCtrl_t *bs, const float *x, float xd)
{
    if (!bs || !x) return 0.0f;

    int n = bs->order;

    /* 复制当前状态 */
    for (int i = 0; i < n; i++)
        bs->x[i] = x[i];

    /* 第1步: z1 = x1 - xd */
    bs->z[0] = x[0] - xd;

    /* 虚拟控制律逐步递推 */
    for (int i = 1; i < n; i++) {
        float fi = (bs->f[i-1]) ? bs->f[i-1](x, bs->user_data) : 0.0f;
        float gi = (bs->g[i-1]) ? bs->g[i-1](x, bs->user_data) : 1.0f;

        /* alpha[i-1] = (-f_i - c[i-1]*z[i-1] - z[i-2]*gi_prev + d_alpha/dt) / g_i */
        /* 简化形式：alpha[i-1] = -(f_i + c[i-1]*z[i-1]) / g_i */
        /* 需要避免除零 */
        if (gi > 1e-6f || gi < -1e-6f)
            bs->alpha[i-1] = -(fi + bs->c[i-1] * bs->z[i-1]) / gi;
        else
            bs->alpha[i-1] = 0.0f;

        /* z[i] = x[i] - alpha[i-1] */
        bs->z[i] = x[i] - bs->alpha[i-1];
    }

    /* 最终控制律u: 使最后一个误差变量z[n-1]收敛到0 */
    /* u = (-f_n - c[n-1]*z[n-1] - z[n-2]*g_{n-1}) / g_n */
    float fn = (bs->f[n-1]) ? bs->f[n-1](x, bs->user_data) : 0.0f;
    float gn = (bs->g[n-1]) ? bs->g[n-1](x, bs->user_data) : 1.0f;

    float u_out;
    if (gn > 1e-6f || gn < -1e-6f)
        u_out = -(fn + bs->c[n-1] * bs->z[n-1]) / gn;
    else
        u_out = 0.0f;

    /* 加入前一误差项的交叉耦合(提高鲁棒性) */
    if (n >= 2) {
        float g_prev = (bs->g[n-2]) ? bs->g[n-2](x, bs->user_data) : 1.0f;
        if (gn > 1e-6f || gn < -1e-6f)
            u_out -= (bs->z[n-2] * g_prev) / gn;
    }

    return u_out;
}

void BS_Reset(BacksteppingCtrl_t *bs)
{
    if (!bs) return;
    memset(bs->x, 0, sizeof(bs->x));
    memset(bs->alpha, 0, sizeof(bs->alpha));
    memset(bs->z, 0, sizeof(bs->z));
}
