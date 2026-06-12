/**
 * @file biquad_filter.h
 * @brief 二阶滤波器（Biquad Filter）
 *
 * Biquad 是最常用的通用数字滤波器基本单元，可实现：
 *   - 低通滤波器 (LPF) - 去除高频噪声
 *   - 高通滤波器 (HPF) - 去除低频漂移
 *   - 带通滤波器 (BPF) - 提取特定频带
 *   - 带阻滤波器 (Notch) - 抑制特定频率
 *   - 峰值/搁架滤波器 - 音频均衡
 *
 * 通用传递函数：
 *         b0 + b1*z⁻¹ + b2*z⁻²
 * H(z) = ─────────────────────────
 *         a0 + a1*z⁻¹ + a2*z⁻²
 *
 * 参数整定指南：
 * ==========================
 * 对于不同类型滤波器，使用对应的初始化函数。
 *
 * 低通/高通：
 *   freq - 截止频率 (Hz)
 *   Q    - 品质因数（0.707 = Butterworth 最大平坦）
 *
 * 带通：
 *   freq - 中心频率 (Hz)
 *   Q    - 带宽反比，Q = freq / bandwidth
 *
 * 级联使用：多个 Biquad 级联可实现更高阶滤波
 */

#ifndef BIQUAD_FILTER_H
#define BIQUAD_FILTER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 滤波器类型枚举 */
typedef enum {
    BIQUAD_LPF = 0,     /* 低通滤波器 */
    BIQUAD_HPF,          /* 高通滤波器 */
    BIQUAD_BPF,          /* 带通滤波器 */
    BIQUAD_NOTCH,        /* 带阻/陷波滤波器 */
    BIQUAD_PEAK,         /* 峰值滤波器 */
    BIQUAD_LOWSHELF,     /* 低频搁架 */
    BIQUAD_HIGHSHELF     /* 高频搁架 */
} BiquadType_t;

/* Biquad 滤波器结构体 */
typedef struct {
    BiquadType_t type;  /* 滤波器类型 */

    /* 设计参数 */
    float freq;         /* 中心/截止频率 (Hz) */
    float Q;            /* 品质因数 */
    float gain_dB;      /* 增益 (dB)，用于 peak/shelf 类型 */
    float fs;           /* 采样频率 (Hz) */

    /* 滤波器系数 */
    float b0, b1, b2;  /* 分子系数 */
    float a0, a1, a2;  /* 分母系数 */

    /* Direct Form II Transposed 状态变量 */
    float z1, z2;       /* 延迟状态 */
} BiquadFilter_t;

/**
 * @brief 初始化低通滤波器
 * @param filter   滤波器指针
 * @param freq     截止频率 (Hz)
 * @param Q        品质因数 (默认 0.707 = Butterworth)
 * @param fs       采样频率 (Hz)
 */
void Biquad_InitLPF(BiquadFilter_t *filter, float freq, float Q, float fs);

/**
 * @brief 初始化高通滤波器
 */
void Biquad_InitHPF(BiquadFilter_t *filter, float freq, float Q, float fs);

/**
 * @brief 初始化带通滤波器
 */
void Biquad_InitBPF(BiquadFilter_t *filter, float freq, float Q, float fs);

/**
 * @brief 初始化带阻/陷波滤波器
 */
void Biquad_InitNotch(BiquadFilter_t *filter, float freq, float Q, float fs);

/**
 * @brief 初始化峰值滤波器
 * @param gain_dB  增益 (dB)，正值提升，负值衰减
 */
void Biquad_InitPeak(BiquadFilter_t *filter, float freq, float Q,
                      float gain_dB, float fs);

/**
 * @brief 滤波器单步计算（Direct Form II Transposed）
 *
 * DF-II-Transposed 结构数值稳定性好，适合定点实现。
 *
 * @param filter   滤波器指针
 * @param input    输入样本
 * @return 滤波后的输出
 */
float Biquad_Update(BiquadFilter_t *filter, float input);

/**
 * @brief 批量滤波
 * @param input   输入数组
 * @param output  输出数组
 * @param len     数据长度
 */
void Biquad_Process(BiquadFilter_t *filter, const float *input,
                    float *output, int len);

/**
 * @brief 重置滤波器状态
 */
void Biquad_Reset(BiquadFilter_t *filter);

#ifdef __cplusplus
}
#endif

#endif /* BIQUAD_FILTER_H */
