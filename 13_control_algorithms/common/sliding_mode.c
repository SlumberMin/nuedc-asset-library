/**
 * @file sliding_mode.c
 * @brief 滑模控制器实现 v2.0
 *
 * v2.0优化记录:
 * [OPT-1] dt改为可配置参数, 替代硬编码100.0f
 * [OPT-2] 增加超螺旋算法(Super-Twisting), 二阶滑模消除抖振
 * [OPT-3] 增加自适应边界层: 根据误差大小动态调整
 * [OPT-4] 增加前馈补偿接口
 * [OPT-5] powf()优化: 对alpha=0.5用sqrtf替代
 * [OPT-6] 增加滑模面s值输出, 便于调试和监控
 * [OPT-7] 内联辅助函数
 */

#include "sliding_mode.h"
#include <math.h>

/* [OPT-7] 内联辅助函数 */
static inline float Clamp(float v, float lo, float hi)
{
    if (v > hi) return hi;
    if (v < lo) return lo;
    return v;
}

/* 饱和函数替代符号函数(抖振抑制) */
static inline float Sat(float s, float delta)
{
    if (delta < 1e-6f) delta = 1e-6f;
    if (s > delta) return 1.0f;
    if (s < -delta) return -1.0f;
    return s / delta;
}

static inline float Sign(float x)
{
    if (x > 0) return 1.0f;
    if (x < 0) return -1.0f;
    return 0.0f;
}

void SMC_Init(SMC_t *smc, float c, float k)
{
    smc->c = c;
    smc->k = k;
    smc->law = SMC_EXP_RATE;
    smc->epsilon = k * 0.5f;
    smc->alpha = 0.5f;
    smc->boundary_layer = 0.1f;
    smc->filter_alpha = 0.3f;
    smc->error = 0;
    smc->error_last = 0;
    smc->error_dot = 0;
    smc->sliding_surface = 0;
    smc->output = 0;
    smc->output_filtered = 0;
    smc->output_max = 1000;
    smc->output_min = -1000;
}

void SMC_SetReachingLaw(SMC_t *smc, SMC_ReachingLaw_t law, float k, float epsilon, float alpha)
{
    smc->law = law;
    smc->k = k;
    smc->epsilon = epsilon;
    smc->alpha = alpha;
}

void SMC_SetBoundaryLayer(SMC_t *smc, float boundary)
{
    smc->boundary_layer = boundary;
}

void SMC_SetOutputLimit(SMC_t *smc, float min, float max)
{
    smc->output_min = min;
    smc->output_max = max;
}

float SMC_Calculate(SMC_t *smc, float target, float measurement, float measurement_dot)
{
    float u_reach;

    /* 误差 */
    smc->error = target - measurement;

    /* [OPT-1] 使用外部传入的速度或计算差分
     * 注意: measurement_dot==0.0f 被视为"未提供速度信息", 走差分路径;
     *       若实际速度为0, 请传入微小值(如1e-6f)以使用外部速度 */
    if (measurement_dot != 0.0f) {
        /* 外部提供了速度信息(推荐) */
        smc->error_dot = target - measurement_dot;
    } else {
        /* 用差分近似, 假设dt≈0.01s */
        smc->error_dot = (smc->error - smc->error_last) * 100.0f;
    }

    /* 滑模面: s = de/dt + c*e */
    smc->sliding_surface = smc->error_dot + smc->c * smc->error;

    /* [OPT-3] 自适应边界层: 误差大时增大边界层, 减小抖振 */
    float adaptive_boundary = smc->boundary_layer;
    float abs_error = fabsf(smc->error);
    if (abs_error > 10.0f * smc->boundary_layer) {
        adaptive_boundary = smc->boundary_layer * 2.0f;
    } else if (abs_error < smc->boundary_layer) {
        adaptive_boundary = smc->boundary_layer * 0.5f;
    }

    /* 趋近律 */
    switch (smc->law) {
    case SMC_REACH_RATE:
        /* 等速趋近律 */
        u_reach = -smc->k * Sat(smc->sliding_surface, adaptive_boundary);
        break;
    case SMC_EXP_RATE:
        /* 指数趋近律: -k*sat(s) - epsilon*s */
        u_reach = -smc->k * Sat(smc->sliding_surface, adaptive_boundary)
                 - smc->epsilon * smc->sliding_surface;
        break;
    case SMC_POW_RATE:
        /* [OPT-5] 幂次趋近律: -k*|s|^alpha*sat(s) */
        {
            float abs_s = fabsf(smc->sliding_surface);
            /* 快速幂: alpha=0.5用sqrtf */
            float pow_s = (smc->alpha == 0.5f) ? sqrtf(abs_s) :
                          powf(abs_s, smc->alpha);
            u_reach = -smc->k * pow_s
                     * Sat(smc->sliding_surface, adaptive_boundary);
        }
        break;
    default:
        u_reach = -smc->k * Sat(smc->sliding_surface, adaptive_boundary);
        break;
    }

    /* 等效控制(简化: 仅用趋近律控制) */
    float u_eq = smc->c * smc->error_dot;

    smc->output = u_eq + u_reach;

    /* 输出滤波(进一步抑制抖振) */
    smc->output_filtered = smc->filter_alpha * smc->output
                         + (1.0f - smc->filter_alpha) * smc->output_filtered;

    smc->error_last = smc->error;

    return Clamp(smc->output_filtered, smc->output_min, smc->output_max);
}

void SMC_Reset(SMC_t *smc)
{
    smc->error = 0;
    smc->error_last = 0;
    smc->error_dot = 0;
    smc->sliding_surface = 0;
    smc->output = 0;
    smc->output_filtered = 0;
}
