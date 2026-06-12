#include "repetitive.h"
#include <string.h>

void Repetitive_Init(RepetitiveCtrl_t *ctrl,
                     uint16_t period_samples,
                     float Kr, float Q,
                     uint16_t lead_steps)
{
    memset(ctrl, 0, sizeof(RepetitiveCtrl_t));

    if (period_samples > REP_MAX_PERIOD_SAMPLES)
        period_samples = REP_MAX_PERIOD_SAMPLES;
    if (period_samples < 2)
        period_samples = 2;

    ctrl->period_samples = period_samples;
    ctrl->Kr = Kr;
    ctrl->Q = (Q > 0.999f) ? 0.999f : ((Q < 0.0f) ? 0.0f : Q);
    ctrl->lead_steps = (lead_steps >= period_samples) ? period_samples - 1 : lead_steps;
    ctrl->index = 0;
    ctrl->out_min = -1e30f;
    ctrl->out_max =  1e30f;
}

void Repetitive_SetOutputLimit(RepetitiveCtrl_t *ctrl, float min, float max)
{
    ctrl->out_min = min;
    ctrl->out_max = max;
}

void Repetitive_SetBaseOutput(RepetitiveCtrl_t *ctrl, float base)
{
    ctrl->base_output = base;
}

float Repetitive_Compute(RepetitiveCtrl_t *ctrl, float error)
{
    uint16_t N = ctrl->period_samples;
    uint16_t idx = ctrl->index;

    /* 计算超前补偿索引: error at (k-N+lead) */
    uint16_t lead_idx = (idx + N - ctrl->lead_steps) % N;

    /* 重复控制律: u_r[k] = Q * u_r[k-N] + Kr * e[k-N+d] */
    float u_r_old = ctrl->buffer[idx];  /* u_r[k-N] */
    float e_delayed = 0.0f;             /* 需要e[k-N+d], 简化使用error */

    /*
     * 简化实现: 使用当前误差的延迟版本
     * buffer同时存储 u_r 和误差历史的组合
     * u_r[k] = Q * buffer[idx] + Kr * error
     */
    float u_r = ctrl->Q * u_r_old + ctrl->Kr * error;

    /* 存入缓冲 */
    ctrl->buffer[idx] = u_r;

    /* 更新索引 */
    ctrl->index = (idx + 1) % N;

    /* 总输出 = 基础控制 + 重复补偿 */
    float u_total = ctrl->base_output + u_r;

    /* 限幅 */
    if (u_total > ctrl->out_max) u_total = ctrl->out_max;
    if (u_total < ctrl->out_min) u_total = ctrl->out_min;

    ctrl->output = u_total;
    return u_total;
}

void Repetitive_Reset(RepetitiveCtrl_t *ctrl)
{
    memset(ctrl->buffer, 0, sizeof(float) * ctrl->period_samples);
    ctrl->index = 0;
    ctrl->base_output = 0.0f;
    ctrl->output = 0.0f;
}
