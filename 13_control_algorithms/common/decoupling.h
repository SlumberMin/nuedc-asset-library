/**
 * @file decoupling.h
 * @brief 解耦控制算法(前馈解耦 + 反馈解耦)
 *
 * 适用场景:
 *   - 多轴运动控制(龙门、XY平台)
 *   - 三相逆变器dq轴解耦
 *   - 双电机同步控制
 *   - 多变量强耦合系统
 *
 * 解耦策略:
 *   1. 前馈解耦: 基于耦合模型计算补偿量, 直接叠加到控制输出
 *   2. 反馈解耦: 基于其他通道的输出实时补偿
 *   3. 混合解耦: 前馈+反馈组合使用
 */

#ifndef __DECOUPLING_H
#define __DECOUPLING_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 最大支持通道数 */
#define DECOUPLE_MAX_CHANNELS  4

typedef enum {
    DECOUPLE_TYPE_FEEDFORWARD = 0,  /* 前馈解耦 */
    DECOUPLE_TYPE_FEEDBACK,         /* 反馈解耦 */
    DECOUPLE_TYPE_HYBRID            /* 混合解耦 */
} DecoupleType_t;

/**
 * @brief 解耦控制器结构体
 *
 * 耦合模型: y_i = sum_j(K_ij * u_j), i != j 为耦合项
 * 前馈解耦: u_i_ff = -sum_j(K_ij / K_ii) * r_j
 * 反馈解耦: u_i_fb = -sum_j(D_ij) * y_j
 */
typedef struct {
    uint8_t n_channels;                                 /* 通道数 */
    DecoupleType_t type;                                /* 解耦类型 */

    /* 耦合系数矩阵 K[i][j]: 通道j对通道i的耦合系数 */
    float K[DECOUPLE_MAX_CHANNELS][DECOUPLE_MAX_CHANNELS];

    /* 解耦矩阵 D[i][j]: 用于反馈解耦 */
    float D[DECOUPLE_MAX_CHANNELS][DECOUPLE_MAX_CHANNELS];

    /* 前馈解耦增益 */
    float Kf[DECOUPLE_MAX_CHANNELS][DECOUPLE_MAX_CHANNELS];

    /* 滤波系数(0~1), 用于平滑解耦输出 */
    float filter_alpha;

    /* 解耦输出 */
    float decouple_out[DECOUPLE_MAX_CHANNELS];

    /* 内部状态 */
    float prev_decouple_out[DECOUPLE_MAX_CHANNELS];
} Decoupling_t;

/**
 * @brief 初始化解耦控制器
 * @param dec       解耦结构体
 * @param n         通道数(2~4)
 * @param type      解耦类型
 */
void Decoupling_Init(Decoupling_t *dec, uint8_t n, DecoupleType_t type);

/**
 * @brief 设置耦合系数矩阵
 * @param dec       解耦结构体
 * @param K_data    n*n维耦合矩阵(行优先)
 */
void Decoupling_SetCouplingMatrix(Decoupling_t *dec, const float *K_data);

/**
 * @brief 设置反馈解耦矩阵
 * @param dec       解耦结构体
 * @param D_data    n*n维解耦矩阵(行优先)
 */
void Decoupling_SetFeedbackMatrix(Decoupling_t *dec, const float *D_data);

/**
 * @brief 设置前馈解耦增益
 * @param dec       解耦结构体
 * @param Kf_data   n*n维前馈增益矩阵(行优先)
 */
void Decoupling_SetFeedforwardMatrix(Decoupling_t *dec, const float *Kf_data);

/**
 * @brief 设置滤波系数
 * @param dec       解耦结构体
 * @param alpha     滤波系数(0=无滤波, 1=全滤波), 默认0.9
 */
void Decoupling_SetFilter(Decoupling_t *dec, float alpha);

/**
 * @brief 前馈解耦更新
 * @param dec       解耦结构体
 * @param ref       各通道参考输入[n]
 * @param out       解耦补偿输出[n]
 */
void Decoupling_FeedforwardUpdate(Decoupling_t *dec, const float *ref, float *out);

/**
 * @brief 反馈解耦更新
 * @param dec       解耦结构体
 * @param feedback  各通道反馈输出[n]
 * @param out       解耦补偿输出[n]
 */
void Decoupling_FeedbackUpdate(Decoupling_t *dec, const float *feedback, float *out);

/**
 * @brief 混合解耦更新(前馈+反馈)
 * @param dec       解耦结构体
 * @param ref       各通道参考输入[n]
 * @param feedback  各通道反馈输出[n]
 * @param out       解耦补偿输出[n]
 */
void Decoupling_HybridUpdate(Decoupling_t *dec, const float *ref,
                              const float *feedback, float *out);

/**
 * @brief 获取指定通道的解耦补偿量
 */
float Decoupling_GetOutput(Decoupling_t *dec, uint8_t channel);

/**
 * @brief 重置解耦控制器状态
 */
void Decoupling_Reset(Decoupling_t *dec);

#ifdef __cplusplus
}
#endif

#endif /* __DECOUPLING_H */
