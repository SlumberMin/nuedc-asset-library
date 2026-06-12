/**
 * @file biquad_filter.c
 * @brief 二阶滤波器（Biquad）实现
 *
 * 使用 Audio EQ Cookbook 中的标准设计公式。
 * 内部采用 Direct Form II Transposed 结构，数值稳定性好。
 *
 * DF-II-Transposed 差分方程：
 *   y[n] = b0*x[n] + z1
 *   z1   = b1*x[n] - a1*y[n] + z2
 *   z2   = b2*x[n] - a2*y[n]
 */

#include "biquad_filter.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/**
 * @brief 通用系数计算（内部辅助）
 *
 * 参考：Audio EQ Cookbook (Robert Bristow-Johnson)
 * https://www.w3.org/2011/audio/audio-eq-cookbook.html
 */
static void calc_lpf(BiquadFilter_t *f)
{
    float w0 = 2.0f * M_PI * f->freq / f->fs;
    float alpha = sinf(w0) / (2.0f * f->Q);

    float cosw0 = cosf(w0);

    f->b0 = (1.0f - cosw0) / 2.0f;
    f->b1 = 1.0f - cosw0;
    f->b2 = (1.0f - cosw0) / 2.0f;
    f->a0 = 1.0f + alpha;
    f->a1 = -2.0f * cosw0;
    f->a2 = 1.0f - alpha;
}

static void calc_hpf(BiquadFilter_t *f)
{
    float w0 = 2.0f * M_PI * f->freq / f->fs;
    float alpha = sinf(w0) / (2.0f * f->Q);
    float cosw0 = cosf(w0);

    f->b0 = (1.0f + cosw0) / 2.0f;
    f->b1 = -(1.0f + cosw0);
    f->b2 = (1.0f + cosw0) / 2.0f;
    f->a0 = 1.0f + alpha;
    f->a1 = -2.0f * cosw0;
    f->a2 = 1.0f - alpha;
}

static void calc_bpf(BiquadFilter_t *f)
{
    float w0 = 2.0f * M_PI * f->freq / f->fs;
    float alpha = sinf(w0) / (2.0f * f->Q);
    float cosw0 = cosf(w0);

    f->b0 = alpha;
    f->b1 = 0.0f;
    f->b2 = -alpha;
    f->a0 = 1.0f + alpha;
    f->a1 = -2.0f * cosw0;
    f->a2 = 1.0f - alpha;
}

static void calc_notch(BiquadFilter_t *f)
{
    float w0 = 2.0f * M_PI * f->freq / f->fs;
    float alpha = sinf(w0) / (2.0f * f->Q);
    float cosw0 = cosf(w0);

    f->b0 = 1.0f;
    f->b1 = -2.0f * cosw0;
    f->b2 = 1.0f;
    f->a0 = 1.0f + alpha;
    f->a1 = -2.0f * cosw0;
    f->a2 = 1.0f - alpha;
}

static void calc_peak(BiquadFilter_t *f)
{
    float w0 = 2.0f * M_PI * f->freq / f->fs;
    float alpha = sinf(w0) / (2.0f * f->Q);
    float cosw0 = cosf(w0);
    float A = powf(10.0f, f->gain_dB / 40.0f);

    f->b0 = 1.0f + alpha * A;
    f->b1 = -2.0f * cosw0;
    f->b2 = 1.0f - alpha * A;
    f->a0 = 1.0f + alpha / A;
    f->a1 = -2.0f * cosw0;
    f->a2 = 1.0f - alpha / A;
}

/**
 * @brief 归一化系数（a0 = 1）
 */
static void normalize(BiquadFilter_t *f)
{
    f->b0 /= f->a0;
    f->b1 /= f->a0;
    f->b2 /= f->a0;
    f->a1 /= f->a0;
    f->a2 /= f->a0;
    f->a0 = 1.0f;
}

void Biquad_InitLPF(BiquadFilter_t *filter, float freq, float Q, float fs)
{
    memset(filter, 0, sizeof(BiquadFilter_t));
    if (Q <= 0.0f) Q = 0.707f;  /* [审计修复] Q不能为0, 默认Butterworth */
    filter->type = BIQUAD_LPF;
    filter->freq = freq;
    filter->Q = Q;
    filter->fs = fs;
    calc_lpf(filter);
    normalize(filter);
}

void Biquad_InitHPF(BiquadFilter_t *filter, float freq, float Q, float fs)
{
    memset(filter, 0, sizeof(BiquadFilter_t));
    if (Q <= 0.0f) Q = 0.707f;
    filter->type = BIQUAD_HPF;
    filter->freq = freq;
    filter->Q = Q;
    filter->fs = fs;
    calc_hpf(filter);
    normalize(filter);
}

void Biquad_InitBPF(BiquadFilter_t *filter, float freq, float Q, float fs)
{
    memset(filter, 0, sizeof(BiquadFilter_t));
    if (Q <= 0.0f) Q = 0.707f;
    filter->type = BIQUAD_BPF;
    filter->freq = freq;
    filter->Q = Q;
    filter->fs = fs;
    calc_bpf(filter);
    normalize(filter);
}

void Biquad_InitNotch(BiquadFilter_t *filter, float freq, float Q, float fs)
{
    memset(filter, 0, sizeof(BiquadFilter_t));
    if (Q <= 0.0f) Q = 10.0f;  /* 陷波滤波器默认Q值 */
    filter->type = BIQUAD_NOTCH;
    filter->freq = freq;
    filter->Q = Q;
    filter->fs = fs;
    calc_notch(filter);
    normalize(filter);
}

void Biquad_InitPeak(BiquadFilter_t *filter, float freq, float Q,
                      float gain_dB, float fs)
{
    memset(filter, 0, sizeof(BiquadFilter_t));
    if (Q <= 0.0f) Q = 1.0f;
    filter->type = BIQUAD_PEAK;
    filter->freq = freq;
    filter->Q = Q;
    filter->gain_dB = gain_dB;
    filter->fs = fs;
    calc_peak(filter);
    normalize(filter);
}

/**
 * @brief Direct Form II Transposed 实现
 *
 * 这种结构只用两个延迟单元，且数值稳定性优于 DF-I。
 *
 * 计算流程：
 *   y = b0*x + z1
 *   z1 = b1*x - a1*y + z2
 *   z2 = b2*x - a2*y
 */
float Biquad_Update(BiquadFilter_t *f, float input)
{
    float output = f->b0 * input + f->z1;

    f->z1 = f->b1 * input - f->a1 * output + f->z2;
    f->z2 = f->b2 * input - f->a2 * output;

    return output;
}

void Biquad_Process(BiquadFilter_t *filter, const float *input,
                    float *output, int len)
{
    for (int i = 0; i < len; i++) {
        output[i] = Biquad_Update(filter, input[i]);
    }
}

void Biquad_Reset(BiquadFilter_t *filter)
{
    filter->z1 = 0.0f;
    filter->z2 = 0.0f;
}
