/**
 * @file notch_filter.h
 * @brief 陷波滤波器（Notch Filter / Band-Stop Filter）
 *
 * 陷波滤波器用于抑制特定频率的干扰，同时保留其他频率的信号。
 * 常见应用：
 *   - 抑制电机换向纹波
 *   - 消除机械共振
 *   - 滤除电源工频干扰（50Hz/60Hz）
 *   - 抑制传感器特定频率噪声
 *
 * 二阶陷波滤波器传递函数：
 *         s² + ωn²
 * H(s) = ───────────────────
 *         s² + (ωn/Q)*s + ωn²
 *
 * 参数整定指南：
 * ==========================
 * freq  - 陷波中心频率 (Hz)，即需要抑制的干扰频率
 * Q     - 品质因数，决定陷波的"窄度"
 *         Q 大 → 陷波窄，只抑制很窄的频带
 *         Q 小 → 陷波宽，抑制较宽的频带
 *         典型值：1 ~ 10
 *
 * 常用设置：
 *   50Hz 工频干扰：freq=50, Q=10
 *   电机换向纹波：根据电机极对数和转速计算频率
 *   机械共振：通过扫频实验确定共振频率
 */

#ifndef NOTCH_FILTER_H
#define NOTCH_FILTER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 陷波滤波器结构体 */
typedef struct {
    /* 参数 */
    float freq;     /* 中心频率 (Hz) */
    float Q;        /* 品质因数 */
    float fs;       /* 采样频率 (Hz) */

    /* 滤波器系数（双线性变换后） */
    float b0, b1, b2;  /* 分子系数 */
    float a0, a1, a2;  /* 分母系数 */

    /* 历史状态 */
    float x1, x2;      /* 输入历史 x[n-1], x[n-2] */
    float y1, y2;      /* 输出历史 y[n-1], y[n-2] */
} NotchFilter_t;

/**
 * @brief 初始化陷波滤波器
 * @param filter   滤波器指针
 * @param freq     陷波中心频率 (Hz)
 * @param Q        品质因数
 * @param fs       采样频率 (Hz)
 */
void NotchFilter_Init(NotchFilter_t *filter, float freq, float Q, float fs);

/**
 * @brief 陷波滤波器单步计算
 * @param filter   滤波器指针
 * @param input    输入样本
 * @return 滤波后的输出
 */
float NotchFilter_Update(NotchFilter_t *filter, float input);

/**
 * @brief 重置滤波器状态
 */
void NotchFilter_Reset(NotchFilter_t *filter);

/**
 * @brief 运行时更新陷波频率（自适应陷波）
 * @param filter   滤波器指针
 * @param freq     新的中心频率 (Hz)
 */
void NotchFilter_SetFrequency(NotchFilter_t *filter, float freq);

#ifdef __cplusplus
}
#endif

#endif /* NOTCH_FILTER_H */
