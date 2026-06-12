/**
 * @file input_shaper.c
 * @brief 输入整形器实现
 *
 * ZV整形器 (两脉冲):
 *   A1 = 1 / (1+K),        t1 = 0
 *   A2 = K / (1+K),         t2 = T/2
 *   其中 K = exp(-zeta*pi/sqrt(1-zeta^2)), T = 1/freq
 *
 * ZVD整形器 (三脉冲):
 *   两个ZV整形器卷积
 *
 * ZVDD整形器 (四脉冲):
 *   三个ZV整形器卷积
 *
 * EI整形器:
 *   允许指定残余振动比, 获得比ZVD更短的时滞
 */
#include "input_shaper.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ================================================================
 *  计算脉冲序列
 * ================================================================ */

/*
 * 辅助函数: 计算衰减比 K 和半周期 T_half
 */
static void calc_params(float freq_hz, float zeta, float *K, float *T_half, float *omega_d)
{
    float wn = 2.0f * M_PI * freq_hz;
    float wd = wn * sqrtf(1.0f - zeta * zeta);
    *K = expf(-zeta * M_PI / sqrtf(1.0f - zeta * zeta));
    *T_half = M_PI / wd;
    *omega_d = wd;
}

void InputShaper_ComputeZV(float freq_hz, float zeta, Impulse_t *impulses, uint8_t *count)
{
    float K, T_half, wd;
    calc_params(freq_hz, zeta, &K, &T_half, &wd);

    /* ZV: 两脉冲 */
    impulses[0].amplitude = 1.0f / (1.0f + K);
    impulses[0].time      = 0.0f;
    impulses[1].amplitude = K / (1.0f + K);
    impulses[1].time      = T_half;
    *count = 2;
}

void InputShaper_ComputeZVD(float freq_hz, float zeta, Impulse_t *impulses, uint8_t *count)
{
    float K, T_half, wd;
    calc_params(freq_hz, zeta, &K, &T_half, &wd);

    float K2 = K * K;

    /* ZVD: 三脉冲 (两个ZV卷积) */
    impulses[0].amplitude = 1.0f / (1.0f + 2.0f * K + K2);
    impulses[0].time      = 0.0f;
    impulses[1].amplitude = 2.0f * K / (1.0f + 2.0f * K + K2);
    impulses[1].time      = T_half;
    impulses[2].amplitude = K2 / (1.0f + 2.0f * K + K2);
    impulses[2].time      = 2.0f * T_half;
    *count = 3;
}

void InputShaper_ComputeZVDD(float freq_hz, float zeta, Impulse_t *impulses, uint8_t *count)
{
    float K, T_half, wd;
    calc_params(freq_hz, zeta, &K, &T_half, &wd);

    float K2 = K * K;
    float K3 = K2 * K;
    float denom = 1.0f + 3.0f * K + 3.0f * K2 + K3;

    /* ZVDD: 四脉冲 */
    impulses[0].amplitude = 1.0f / denom;
    impulses[0].time      = 0.0f;
    impulses[1].amplitude = 3.0f * K / denom;
    impulses[1].time      = T_half;
    impulses[2].amplitude = 3.0f * K2 / denom;
    impulses[2].time      = 2.0f * T_half;
    impulses[3].amplitude = K3 / denom;
    impulses[3].time      = 3.0f * T_half;
    *count = 4;
}

void InputShaper_ComputeEI(float freq_hz, float zeta, float allowed_vib,
                           Impulse_t *impulses, uint8_t *count)
{
    float K, T_half, wd;
    calc_params(freq_hz, zeta, &K, &T_half, &wd);

    float A = allowed_vib;  /* 允许的残余振动比 (0~1) */
    if (A < 0.001f) A = 0.001f;
    if (A > 0.5f)   A = 0.5f;

    float K2 = K * K;
    float denom = 1.0f + 2.0f * K * A + K2;

    /* EI: 三脉冲 */
    impulses[0].amplitude = 1.0f / denom;
    impulses[0].time      = 0.0f;
    impulses[1].amplitude = 2.0f * K * A / denom;
    impulses[1].time      = T_half;
    impulses[2].amplitude = K2 / denom;
    impulses[2].time      = 2.0f * T_half;
    *count = 3;
}

/* ================================================================
 *  初始化
 * ================================================================ */
void InputShaper_Init(InputShaper_t *shaper, InputShaperType_e type,
                      float freq_hz, float zeta, float dt,
                      float *buffer_mem, uint16_t buffer_size)
{
    memset(shaper, 0, sizeof(InputShaper_t));
    shaper->type = type;
    shaper->freq = freq_hz;
    shaper->zeta = zeta;
    shaper->dt   = (dt > 0.0f) ? dt : 0.001f;  /* [审计修复] 防止除零 */

    /* 计算脉冲序列 */
    switch (type) {
    case IS_TYPE_ZV:
        InputShaper_ComputeZV(freq_hz, zeta, shaper->impulses, &shaper->num_impulses);
        break;
    case IS_TYPE_ZVD:
        InputShaper_ComputeZVD(freq_hz, zeta, shaper->impulses, &shaper->num_impulses);
        break;
    case IS_TYPE_ZVDD:
        InputShaper_ComputeZVDD(freq_hz, zeta, shaper->impulses, &shaper->num_impulses);
        break;
    case IS_TYPE_EI:
        InputShaper_ComputeEI(freq_hz, zeta, 0.05f, shaper->impulses, &shaper->num_impulses);
        break;
    }

    /* 计算所需延迟缓冲区大小 */
    float max_time = 0.0f;
    for (uint8_t i = 0; i < shaper->num_impulses; i++) {
        if (shaper->impulses[i].time > max_time) {
            max_time = shaper->impulses[i].time;
        }
    }
    if (dt <= 0.0f) dt = 0.001f;
    shaper->delay_samples = (uint16_t)(max_time / dt) + 1;

    /* 设置缓冲区 */
    shaper->buffer = buffer_mem;
    shaper->buffer_size = buffer_size;

    /* 确保缓冲区足够大 */
    if (shaper->buffer_size < shaper->delay_samples + 1) {
        shaper->delay_samples = shaper->buffer_size - 1;
    }

    memset(shaper->buffer, 0, sizeof(float) * shaper->buffer_size);
    shaper->write_idx = 0;
    shaper->initialized = 1;
}

void InputShaper_Reset(InputShaper_t *shaper)
{
    if (shaper->buffer) {
        memset(shaper->buffer, 0, sizeof(float) * shaper->buffer_size);
    }
    shaper->write_idx = 0;
    shaper->output = 0.0f;
}

/* ================================================================
 *  整形计算
 *
 *  将当前参考值写入延迟线, 然后对各脉冲加权求和
 * ================================================================ */
float InputShaper_Update(InputShaper_t *shaper, float reference)
{
    if (!shaper->initialized) return reference;

    /* 写入当前参考值到延迟线 */
    shaper->buffer[shaper->write_idx] = reference;

    float shaped = 0.0f;

    for (uint8_t i = 0; i < shaper->num_impulses; i++) {
        /* 计算该脉冲对应的延迟采样数 */
        uint16_t delay = (uint16_t)(shaper->impulses[i].time / shaper->dt + 0.5f);

        /* 从延迟线读取 */
        int32_t read_idx = (int32_t)shaper->write_idx - (int32_t)delay;
        if (read_idx < 0) {
            read_idx += shaper->buffer_size;
        }
        shaped += shaper->impulses[i].amplitude * shaper->buffer[read_idx];
    }

    /* 更新写指针 */
    shaper->write_idx++;
    if (shaper->write_idx >= shaper->buffer_size) {
        shaper->write_idx = 0;
    }

    shaper->output = shaped;
    return shaped;
}

float InputShaper_GetOutput(InputShaper_t *shaper)
{
    return shaper->output;
}

float InputShaper_GetDelay(InputShaper_t *shaper)
{
    if (!shaper->initialized || shaper->num_impulses == 0) return 0.0f;

    float max_time = 0.0f;
    for (uint8_t i = 0; i < shaper->num_impulses; i++) {
        if (shaper->impulses[i].time > max_time) {
            max_time = shaper->impulses[i].time;
        }
    }
    return max_time;
}
