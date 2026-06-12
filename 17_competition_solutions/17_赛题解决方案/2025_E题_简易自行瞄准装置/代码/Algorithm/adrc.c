/**
 * @file adrc.c
 * @brief ADRC自抗扰控制器实现 v2.0 - 统一权威版本
 * @version 2.0
 * @date    2026-06-11
 * @sync    与nuedc-asset-library/11_控制算法库/common/adrc.c v2.0同步
 */

#include "adrc.h"
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

static inline float FastPowAlpha(float x, float alpha)
{
    if (alpha == 0.5f) return sqrtf(x);
    if (alpha == 0.25f) return sqrtf(sqrtf(x));
    if (alpha == 0.75f) return sqrtf(x) * sqrtf(sqrtf(x));
    if (alpha == 1.0f) return x;
    return powf(x, alpha);
}

static inline float Sign(float x)
{
    if (x > 0.0f) return 1.0f;
    if (x < 0.0f) return -1.0f;
    return 0.0f;
}

static float Fal(float e, float alpha, float delta, float inv_delta_pow)
{
    float abs_e = fabsf(e);
    if (abs_e <= delta) return e * inv_delta_pow;
    float sign_e = (e > 0.0f) ? 1.0f : -1.0f;
    return FastPowAlpha(abs_e, alpha) * sign_e;
}

static float Fhan(float x1, float x2, float r, float h)
{
    float d = r * h * h;
    float a0 = h * x2;
    float y = x1 + a0;
    float abs_y = fabsf(y);
    float a1 = sqrtf(d * (d + 8.0f * abs_y));
    float sign_y = Sign(y);
    float a2 = a0 + sign_y * (a1 - d) * 0.5f;
    float sy = (abs_y < d) ? 1.0f : 0.0f;
    float a = (a0 + y - a2) * sy + a2;
    float abs_a = fabsf(a);
    float sa = (abs_a < d) ? 1.0f : 0.0f;
    return -r * (a / d - Sign(a)) * sa - r * Sign(a);
}

void ADRC_Init(ADRC_t *adrc, float h, float b)
{
    adrc->h = h; adrc->b = b; adrc->output = 0;
    adrc->td.r = 10.0f; adrc->td.h = h;
    adrc->td.x1 = 0; adrc->td.x2 = 0;
    adrc->eso.beta1 = 100.0f; adrc->eso.beta2 = 300.0f; adrc->eso.beta3 = 1000.0f;
    adrc->eso.alpha1 = 0.5f; adrc->eso.alpha2 = 0.25f; adrc->eso.delta = 0.01f;
    adrc->eso.b = b; adrc->eso.z1 = 0; adrc->eso.z2 = 0; adrc->eso.z3 = 0;
    adrc->nlsef.beta0 = 0.5f; adrc->nlsef.beta1 = 0.1f;
    adrc->nlsef.alpha0 = 0.75f; adrc->nlsef.alpha1 = 1.5f; adrc->nlsef.delta = 0.1f;
    adrc->output_min = -1000.0f; adrc->output_max = 1000.0f;
    adrc->mode = ADRC_NONLINEAR;
}

void ADRC_SetTD(ADRC_t *adrc, float r) { adrc->td.r = r; }
void ADRC_SetESO(ADRC_t *adrc, float beta1, float beta2, float beta3) {
    adrc->eso.beta1 = beta1; adrc->eso.beta2 = beta2; adrc->eso.beta3 = beta3;
}
void ADRC_SetNLSEF(ADRC_t *adrc, float beta0, float beta1, float alpha0, float alpha1) {
    adrc->nlsef.beta0 = beta0; adrc->nlsef.beta1 = beta1;
    adrc->nlsef.alpha0 = alpha0; adrc->nlsef.alpha1 = alpha1;
}
void ADRC_SetBandwidth(ADRC_t *adrc, float wo, float wc) {
    adrc->eso.beta1 = 3.0f * wo;
    adrc->eso.beta2 = 3.0f * wo * wo;
    adrc->eso.beta3 = wo * wo * wo;
    adrc->nlsef.beta0 = wc * wc;
    adrc->nlsef.beta1 = 2.0f * wc;
}
void ADRC_SetOutputLimit(ADRC_t *adrc, float min, float max) {
    adrc->output_min = min; adrc->output_max = max;
}
void ADRC_SetMode(ADRC_t *adrc, ADRC_Mode_t mode) { adrc->mode = mode; }

static void TD_Update(ADRC_TD_t *td, float v)
{
    float fh = Fhan(td->x1 - v, td->x2, td->r, td->h);
    td->x1 += td->h * td->x2;
    td->x2 += td->h * fh;
}

static void ESO_Update(ADRC_ESO_t *eso, float y, float u, float h)
{
    float e = eso->z1 - y;
    float inv_delta_pow_a1 = 1.0f / powf(eso->delta, 1.0f - eso->alpha1);
    float inv_delta_pow_a2 = 1.0f / powf(eso->delta, 1.0f - eso->alpha2);
    eso->z1 += h * (eso->z2 - eso->beta1 * e);
    eso->z2 += h * (eso->z3 - eso->beta2 * Fal(e, eso->alpha1, eso->delta, inv_delta_pow_a1) + eso->b * u);
    eso->z3 += h * (-eso->beta3 * Fal(e, eso->alpha2, eso->delta, inv_delta_pow_a2));
}

float ADRC_Calculate(ADRC_t *adrc, float target, float measurement)
{
    float e0, e1, u0, u;
    TD_Update(&adrc->td, target);
    ESO_Update(&adrc->eso, measurement, adrc->output, adrc->h);
    e0 = adrc->td.x1 - adrc->eso.z1;
    e1 = adrc->td.x2 - adrc->eso.z2;
    if (adrc->mode == ADRC_LINEAR) {
        u0 = adrc->nlsef.beta0 * e0 + adrc->nlsef.beta1 * e1;
    } else {
        float inv_delta_pow_0 = 1.0f / powf(adrc->nlsef.delta, 1.0f - adrc->nlsef.alpha0);
        float inv_delta_pow_1 = 1.0f / powf(adrc->nlsef.delta, 1.0f - adrc->nlsef.alpha1);
        u0 = adrc->nlsef.beta0 * Fal(e0, adrc->nlsef.alpha0, adrc->nlsef.delta, inv_delta_pow_0)
           + adrc->nlsef.beta1 * Fal(e1, adrc->nlsef.alpha1, adrc->nlsef.delta, inv_delta_pow_1);
    }
    u = (u0 - adrc->eso.z3) / adrc->b;
    if (u > adrc->output_max) u = adrc->output_max;
    if (u < adrc->output_min) u = adrc->output_min;
    adrc->output = u;
    return u;
}

void ADRC_Reset(ADRC_t *adrc)
{
    adrc->td.x1 = 0; adrc->td.x2 = 0;
    adrc->eso.z1 = 0; adrc->eso.z2 = 0; adrc->eso.z3 = 0;
    adrc->output = 0;
}

#ifdef __cplusplus
}
#endif
