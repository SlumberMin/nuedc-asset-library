/**
 * @file feedforward.c
 * @brief 前馈控制算法实现
 */
#include "feedforward.h"
#include <math.h>
#include <string.h>

/* 辅助宏 */
#define CLAMP(val, min, max) ((val) < (min) ? (min) : ((val) > (max) ? (max) : (val)))

void FeedForward_Init(FeedForward_t *ff, const FeedForward_Config_t *config)
{
    if (ff == NULL || config == NULL) return;
    memset(ff, 0, sizeof(FeedForward_t));
    memcpy(&ff->config, config, sizeof(FeedForward_Config_t));
    ff->initialized = 1;
}

float FeedForward_Calc(FeedForward_t *ff, float reference, float dt)
{
    if (!ff->initialized || dt <= 0.0f) {
        return 0.0f;
    }

    /* 估算速度: v = (ref - prev_ref) / dt */
    float velocity = (reference - ff->prev_ref) / dt;

    /* 估算加速度: a = (v - prev_v) / dt */
    float acceleration = (velocity - ff->prev_vel) / dt;

    float output = 0.0f;

    switch (ff->config.type) {
        case FF_TYPE_VELOCITY:
            output = ff->config.Kv * velocity;
            break;

        case FF_TYPE_ACCELERATION:
            output = ff->config.Kv * velocity + ff->config.Ka * acceleration;
            break;

        case FF_TYPE_CUSTOM:
            output = ff->config.Kv * velocity
                   + ff->config.Ka * acceleration
                   + ff->config.Kj * (acceleration - ff->prev_accel) / dt; /* jerk = da/dt */
            break;

        default:
            output = 0.0f;
            break;
    }

    /* 限幅 */
    output = CLAMP(output, ff->config.output_min, ff->config.output_max);

    /* 更新状态 */
    ff->prev_ref = reference;
    ff->prev_vel = velocity;
    ff->prev_accel = acceleration;  /* [审计修复] 保存加速度用于下次jerk计算 */
    ff->output = output;

    return output;
}

float FeedForward_CalcExplicit(FeedForward_t *ff, float velocity,
                                float acceleration, float jerk)
{
    if (!ff->initialized) {
        return 0.0f;
    }

    float output = 0.0f;

    switch (ff->config.type) {
        case FF_TYPE_VELOCITY:
            output = ff->config.Kv * velocity;
            break;

        case FF_TYPE_ACCELERATION:
            output = ff->config.Kv * velocity + ff->config.Ka * acceleration;
            break;

        case FF_TYPE_CUSTOM:
            output = ff->config.Kv * velocity
                   + ff->config.Ka * acceleration
                   + ff->config.Kj * jerk;
            break;

        default:
            output = 0.0f;
            break;
    }

    output = CLAMP(output, ff->config.output_min, ff->config.output_max);
    ff->output = output;
    return output;
}

float FeedForward_Lookup(const FeedForward_LUT_t *lut, float input)
{
    if (lut == NULL || lut->size < 2) {
        return 0.0f;
    }

    /* 边界检查 */
    if (input <= lut->input[0]) {
        return lut->output[0];
    }
    if (input >= lut->input[lut->size - 1]) {
        return lut->output[lut->size - 1];
    }

    /* 线性插值 */
    for (uint16_t i = 0; i < lut->size - 1; i++) {
        if (input >= lut->input[i] && input <= lut->input[i + 1]) {
            float ratio = (input - lut->input[i]) / (lut->input[i + 1] - lut->input[i]);
            return lut->output[i] + ratio * (lut->output[i + 1] - lut->output[i]);
        }
    }

    return 0.0f;
}

void FeedForward_Reset(FeedForward_t *ff)
{
    ff->prev_ref = 0.0f;
    ff->prev_vel = 0.0f;
    ff->prev_accel = 0.0f;
    ff->output = 0.0f;
}
