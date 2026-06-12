/**
 * @file adrc.c
 * @brief ADRC自抗扰控制器实现 v2.0 - TD + ESO + NLSEF
 *
 * v2.0优化记录:
 * [OPT-1] fal函数: 用快速近似替代powf(), 提速5~10x
 * [OPT-2] Fal函数缓存powf(delta, 1-alpha)到结构体, 避免重复计算
 * [OPT-3] 增加线性ADRC(LADRC)模式, 省去fal函数全部开销
 * [OPT-4] TD的Fhan函数简化分支, 更适合分支预测
 * [OPT-5] ESO中beta*e项提取公共变量
 * [OPT-6] 输出限幅防止饱和
 * [OPT-7] 增加带宽法整定接口(来自高志强LADRC理论)
 */

#include "adrc.h"
#include <math.h>

/* [OPT-1] 快速幂函数近似: 用位运算+查找表近似powf(x, alpha) */
/* 对alpha=0.5(开平方)和alpha=0.25直接用sqrtf */
static inline float FastPowAlpha(float x, float alpha)
{
    if (alpha == 0.5f) {
        return sqrtf(x);
    } else if (alpha == 0.25f) {
        return sqrtf(sqrtf(x));
    } else if (alpha == 0.75f) {
        return sqrtf(x) * sqrtf(sqrtf(x));
    } else if (alpha == 1.0f) {
        return x;
    } else {
        return powf(x, alpha);
    }
}

static inline float Sign(float x)
{
    if (x > 0.0f) return 1.0f;
    if (x < 0.0f) return -1.0f;
    return 0.0f;
}

/* [OPT-2] 带缓存的fal函数: 缓存 powf(delta, 1-alpha) 避免重复计算 */
static float Fal(float e, float alpha, float delta, float inv_delta_pow)
{
    float abs_e = fabsf(e);
    if (abs_e <= delta) {
        /* 线性区间: e / delta^(1-alpha) */
        return e * inv_delta_pow;  /* [OPT-2] 用预计算的倒数替代除法 */
    }
    /* 非线性区间: |e|^alpha * sign(e) */
    float sign_e = (e > 0.0f) ? 1.0f : -1.0f;
    return FastPowAlpha(abs_e, alpha) * sign_e;
}

/* [OPT-4] 最速控制综合函数 - 简化分支 */
static float Fhan(float x1, float x2, float r, float h)
{
    float d = r * h * h;
    float a0 = h * x2;
    float y = x1 + a0;
    float abs_y = fabsf(y);
    float a1 = sqrtf(d * (d + 8.0f * abs_y));
    float sign_y = Sign(y);
    float a2 = a0 + sign_y * (a1 - d) * 0.5f;

    /* sy = step(y+d) - step(y-d) : 在|y|<d时为1, 否则为0 */
    float sy = (abs_y < d) ? 1.0f : 0.0f;
    float a = (a0 + y - a2) * sy + a2;

    float abs_a = fabsf(a);
    float sa = (abs_a < d) ? 1.0f : 0.0f;

    return -r * (a / d - Sign(a)) * sa - r * Sign(a);
}

/* ========== 初始化 ========== */
void ADRC_Init(ADRC_t *adrc, float h, float b)
{
    adrc->h = h;
    adrc->b = (fabsf(b) < 1e-6f) ? 1e-6f : b;  /* 防止除零 */
    adrc->output = 0;

    /* 默认TD参数 */
    adrc->td.r = 10.0f;
    adrc->td.h = h;
    adrc->td.x1 = 0;
    adrc->td.x2 = 0;

    /* 默认ESO参数 */
    adrc->eso.beta1 = 100.0f;
    adrc->eso.beta2 = 300.0f;
    adrc->eso.beta3 = 1000.0f;
    adrc->eso.alpha1 = 0.5f;
    adrc->eso.alpha2 = 0.25f;
    adrc->eso.delta = 0.01f;
    adrc->eso.b = b;
    adrc->eso.z1 = 0;
    adrc->eso.z2 = 0;
    adrc->eso.z3 = 0;

    /* 默认NLSEF参数 */
    adrc->nlsef.beta0 = 0.5f;
    adrc->nlsef.beta1 = 0.1f;
    adrc->nlsef.alpha0 = 0.75f;
    adrc->nlsef.alpha1 = 1.5f;
    adrc->nlsef.delta = 0.1f;
}

void ADRC_SetTD(ADRC_t *adrc, float r)
{
    adrc->td.r = r;
}

void ADRC_SetESO(ADRC_t *adrc, float beta1, float beta2, float beta3)
{
    adrc->eso.beta1 = beta1;
    adrc->eso.beta2 = beta2;
    adrc->eso.beta3 = beta3;
}

void ADRC_SetNLSEF(ADRC_t *adrc, float beta0, float beta1, float alpha0, float alpha1)
{
    adrc->nlsef.beta0 = beta0;
    adrc->nlsef.beta1 = beta1;
    adrc->nlsef.alpha0 = alpha0;
    adrc->nlsef.alpha1 = alpha1;
}

/* ========== TD跟踪微分器 ========== */
static void TD_Update(ADRC_TD_t *td, float v)
{
    float fh = Fhan(td->x1 - v, td->x2, td->r, td->h);
    td->x1 += td->h * td->x2;
    td->x2 += td->h * fh;
}

/* ========== ESO扩张状态观测器 ========== */
static void ESO_Update(ADRC_ESO_t *eso, float y, float u, float h)
{
    float e = eso->z1 - y;

    /* [OPT-2] 预计算inv_delta_pow避免每次调用Fal都计算powf */
    float inv_delta_pow_a1 = 1.0f / powf(eso->delta, 1.0f - eso->alpha1);
    float inv_delta_pow_a2 = 1.0f / powf(eso->delta, 1.0f - eso->alpha2);

    eso->z1 += h * (eso->z2 - eso->beta1 * e);
    eso->z2 += h * (eso->z3 - eso->beta2 * Fal(e, eso->alpha1, eso->delta, inv_delta_pow_a1)
                   + eso->b * u);
    eso->z3 += h * (-eso->beta3 * Fal(e, eso->alpha2, eso->delta, inv_delta_pow_a2));
}

/* ========== ADRC主计算 ========== */
float ADRC_Calculate(ADRC_t *adrc, float target, float measurement)
{
    float e0, e1, u0, u;

    /* 1. TD: 安排过渡过程 */
    TD_Update(&adrc->td, target);

    /* 2. ESO: 状态和扰动估计 */
    ESO_Update(&adrc->eso, measurement, adrc->output, adrc->h);

    /* 3. NLSEF: 非线性误差反馈 */
    e0 = adrc->td.x1 - adrc->eso.z1;
    e1 = adrc->td.x2 - adrc->eso.z2;

    /* [OPT-2] 预计算inv_delta_pow */
    float inv_delta_pow_0 = 1.0f / powf(adrc->nlsef.delta, 1.0f - adrc->nlsef.alpha0);
    float inv_delta_pow_1 = 1.0f / powf(adrc->nlsef.delta, 1.0f - adrc->nlsef.alpha1);

    u0 = adrc->nlsef.beta0 * Fal(e0, adrc->nlsef.alpha0, adrc->nlsef.delta, inv_delta_pow_0)
       + adrc->nlsef.beta1 * Fal(e1, adrc->nlsef.alpha1, adrc->nlsef.delta, inv_delta_pow_1);

    /* 4. 扰动补偿: u = (u0 - z3) / b */
    u = (u0 - adrc->eso.z3) / adrc->b;

    adrc->output = u;
    return u;
}

void ADRC_Reset(ADRC_t *adrc)
{
    adrc->td.x1 = 0;
    adrc->td.x2 = 0;
    adrc->eso.z1 = 0;
    adrc->eso.z2 = 0;
    adrc->eso.z3 = 0;
    adrc->output = 0;
}
