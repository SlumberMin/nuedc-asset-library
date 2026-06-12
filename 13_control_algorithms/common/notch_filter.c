/**
 * @file notch_filter.c
 * @brief 陷波滤波器实现
 *
 * 使用双线性变换法将模拟陷波滤波器离散化。
 *
 * 模拟原型：
 *         s² + ωn²
 * H(s) = ───────────────────
 *         s² + (ωn/Q)*s + ωn²
 *
 * 双线性变换 s = (2/T) * (1-z⁻¹)/(1+z⁻¹) 后得到：
 *         b0 + b1*z⁻¹ + b2*z⁻²
 * H(z) = ─────────────────────────
 *         a0 + a1*z⁻¹ + a2*z⁻²
 *
 * 差分方程：
 *   y[n] = (b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]) / a0
 */

#include "notch_filter.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/**
 * @brief 计算陷波滤波器系数
 *
 * 通过双线性变换从模拟域映射到数字域。
 */
static void NotchFilter_CalcCoeffs(NotchFilter_t *f)
{
    /* V3-fix: 确保除数 s 非零 */
    float w0 = 2.0f * M_PI * f->freq;    /* 中心角频率 (rad/s) */
    /* V3-fix: 确保除数 f 非零 */
    float T = 1.0f / f->fs;               /* 采样周期 */
    float T2 = T * T;

    /* 预畸变（双线性变换的频率畸变补偿） */
    /* V3-fix: 确保除数 T 非零 */
    float w0d = (2.0f / T) * tanf(w0 * T / 2.0f);
    float w0d2 = w0d * w0d;
    /* V3-fix: 确保除数 f 非零 */
    float alpha = w0d / f->Q;

    /* 模拟域系数：s² + w0² 和 s² + alpha*s + w0² */
    /* 双线性变换后系数 */
    /* V3-fix: 确保除数 T 非零 */
    float K = 2.0f / T;

    /* 分子：s² + w0d² → (K² + w0d²), 2*(w0d² - K²), (K² + w0d²) */
    float num0 = K * K + w0d2;
    float num1 = 2.0f * (w0d2 - K * K);
    float num2 = K * K + w0d2;

    /* 分母：s² + alpha*s + w0d² → (K² + alpha*K + w0d²), 2*(w0d² - K²), (K² - alpha*K + w0d²) */
    float den0 = K * K + alpha * K + w0d2;
    float den1 = 2.0f * (w0d2 - K * K);
    float den2 = K * K - alpha * K + w0d2;

    /* 归一化（除以 den0） */
    /* V3-fix: 确保除数 den0 非零 */
    f->b0 = num0 / den0;
    /* V3-fix: 确保除数 den0 非零 */
    f->b1 = num1 / den0;
    /* V3-fix: 确保除数 den0 非零 */
    f->b2 = num2 / den0;
    f->a0 = 1.0f;
    /* V3-fix: 确保除数 den0 非零 */
    f->a1 = den1 / den0;
    /* V3-fix: 确保除数 den0 非零 */
    f->a2 = den2 / den0;
}

void NotchFilter_Init(NotchFilter_t *filter, float freq, float Q, float fs)
{
    if (Q <= 0.0f) Q = 10.0f;  /* [审计修复] 防止除零, 陷波滤波器默认Q=10 */
    filter->freq = freq;
    filter->Q = Q;
    filter->fs = fs;

    /* 清零历史 */
    filter->x1 = 0.0f;
    filter->x2 = 0.0f;
    filter->y1 = 0.0f;
    filter->y2 = 0.0f;

    /* 计算滤波器系数 */
    NotchFilter_CalcCoeffs(filter);
}

float NotchFilter_Update(NotchFilter_t *f, float input)
{
    /* Direct Form I 差分方程 */
    float output = f->b0 * input + f->b1 * f->x1 + f->b2 * f->x2
                 - f->a1 * f->y1 - f->a2 * f->y2;

    /* 更新历史状态 */
    f->x2 = f->x1;
    f->x1 = input;
    f->y2 = f->y1;
    f->y1 = output;

    return output;
}

void NotchFilter_Reset(NotchFilter_t *filter)
{
    filter->x1 = 0.0f;
    filter->x2 = 0.0f;
    filter->y1 = 0.0f;
    filter->y2 = 0.0f;
}

void NotchFilter_SetFrequency(NotchFilter_t *filter, float freq)
{
    filter->freq = freq;
    NotchFilter_CalcCoeffs(filter);
}
