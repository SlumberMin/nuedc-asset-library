/**
 * @file lead_lag.c
 * @brief 超前滞后补偿器实现
 * 使用双线性变换(s域 -> z域)进行离散化
 */

#include "lead_lag.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/**
 * 内部: 一阶超前/滞后补偿器设计
 * H(s) = K * (s + z) / (s + p)
 * 其中 z = 2*pi*fc*sqrt(alpha), p = z/alpha (超前时alpha>1)
 *
 * 双线性变换: s = (2/T) * (1-z^-1)/(1+z^-1)
 */
static void DesignFirstOrder(LeadLag_t *comp, float zero, float pole, float gain, float fs)
{
    float T = 1.0f / fs;
    float K2 = 2.0f / T;

    /* H(s) = gain * (s + zero) / (s + pole) */
    /* 分子: (s + zero) -> (K2*(1-z^-1)/(1+z^-1) + zero) */
    /* 分母: (s + pole) -> (K2*(1-z^-1)/(1+z^-1) + pole) */

    /* 离散化后的系数 */
    float b0_num = gain * (K2 + zero);
    float b1_num = gain * (-K2 + zero);
    float a0_num = K2 + pole;
    float a1_num = -K2 + pole;

    /* 归一化(a0=1) */
    comp->b0 = b0_num / a0_num;
    comp->b1 = b1_num / a0_num;
    comp->b2 = 0.0f;
    comp->a0 = 1.0f;
    comp->a1 = a1_num / a0_num;
    comp->a2 = 0.0f;
    comp->gain = gain;

    comp->x1 = comp->x2 = 0.0f;
    comp->y1 = comp->y2 = 0.0f;
}

void LeadLag_InitLead(LeadLag_t *comp, float fc, float alpha, float gain, float fs)
{
    if (comp == NULL || fc <= 0.0f || alpha <= 0.0f || fs <= 0.0f) return;
    /* 超前: alpha > 1 */
    float wc = 2.0f * M_PI * fc;
    float zero = wc / sqrtf(alpha);   /* 零点频率较低 */
    float pole = wc * sqrtf(alpha);   /* 极点频率较高 */
    DesignFirstOrder(comp, zero, pole, gain, fs);
}

void LeadLag_InitLag(LeadLag_t *comp, float fc, float alpha, float gain, float fs)
{
    if (comp == NULL || fc <= 0.0f || alpha <= 0.0f || fs <= 0.0f) return;
    /* 滞后: 0 < alpha < 1 */
    float wc = 2.0f * M_PI * fc;
    float zero = wc * sqrtf(alpha);   /* 零点频率较高 */
    float pole = wc / sqrtf(alpha);   /* 极点频率较低 */
    DesignFirstOrder(comp, zero, pole, gain, fs);
}

void LeadLag_InitLeadLag(LeadLag_t *comp,
                         float fc_lead, float alpha_lead,
                         float fc_lag,  float alpha_lag,
                         float gain, float fs)
{
    /* 超前-滞后复合: 级联两个一阶环节,用二阶IIR实现 */
    float T = 1.0f / fs;
    float K2 = 2.0f / T;

    float wc_l = 2.0f * M_PI * fc_lead;
    float z_l = wc_l / sqrtf(alpha_lead);
    float p_l = wc_l * sqrtf(alpha_lead);

    float wc_g = 2.0f * M_PI * fc_lag;
    float z_g = wc_g * sqrtf(alpha_lag);
    float p_g = wc_g / sqrtf(alpha_lag);

    /* 两个一阶级联 -> 二阶: H(s) = K*(s+z_l)*(s+z_g) / ((s+p_l)*(s+p_g)) */
    /* 展开分子: s^2 + (z_l+z_g)*s + z_l*z_g */
    /* 展开分母: s^2 + (p_l+p_g)*s + p_l*p_g */

    /* 双线性变换后(略去推导,直接用数值形式) */
    /* 这里简化为将两个一阶系数相乘 */
    LeadLag_t lead, lag;
    DesignFirstOrder(&lead, z_l, p_l, 1.0f, fs);
    DesignFirstOrder(&lag, z_g, p_g, gain, fs);

    /* 二阶系数 = 两个一阶卷积 */
    comp->b0 = lead.b0 * lag.b0;
    comp->b1 = lead.b0 * lag.b1 + lead.b1 * lag.b0;
    comp->b2 = lead.b1 * lag.b1;
    comp->a0 = 1.0f;
    comp->a1 = lead.a1 + lag.a1;
    comp->a2 = lead.a1 * lag.a1;
    comp->gain = gain;

    comp->x1 = comp->x2 = 0.0f;
    comp->y1 = comp->y2 = 0.0f;
}

float LeadLag_Update(LeadLag_t *comp, float input)
{
    float output = comp->b0 * input
                 + comp->b1 * comp->x1
                 + comp->b2 * comp->x2
                 - comp->a1 * comp->y1
                 - comp->a2 * comp->y2;

    comp->x2 = comp->x1;
    comp->x1 = input;
    comp->y2 = comp->y1;
    comp->y1 = output;

    return output;
}

void LeadLag_Reset(LeadLag_t *comp)
{
    comp->x1 = comp->x2 = 0.0f;
    comp->y1 = comp->y2 = 0.0f;
}
