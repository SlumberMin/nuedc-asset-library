/**
 * @file adaptive_kalman.h
 * @brief 自适应卡尔曼滤波器 - Sage-Husa自适应算法
 * @version 1.0
 * @date 2026-06-11
 *
 * 自适应卡尔曼滤波器 (Adaptive Kalman Filter) 通过在线调整
 * 过程噪声协方差Q和测量噪声协方差R, 适应系统特性变化。
 *
 * 核心算法:
 *   1. 基于新息序列自适应调整R矩阵
 *   2. 基于残差自适应调整Q矩阵
 *   3. Sage-Husa自适应算法 (递推贝叶斯估计)
 *
 * Sage-Husa算法:
 *   R(k) = R(k-1) + d_R * (v(k)*v(k)^T - R(k-1))
 *   Q(k) = Q(k-1) + d_Q * (K(k)*v(k)*v(k)^T*K(k)^T - Q(k-1))
 *
 *   其中:
 *   - v(k) = y(k) - H*x(k|k-1) 是新息 (预测误差)
 *   - d_R, d_Q 是遗忘因子 (0 < d < 1)
 *
 * 特点:
 *   - 不需要预知精确的Q和R
 *   - 自动跟踪系统特性变化
 *   - 适合非平稳系统、时变系统
 *   - 适合电赛中传感器噪声特性变化的场景
 *
 * 推荐应用:
 *   - 传感器融合 (IMU, 编码器, 光电等)
 *   - 姿态估计 (互补滤波增强)
 *   - 电机状态观测
 *   - 噪声特性未知或时变的场合
 */

#ifndef __ADAPTIVE_KALMAN_H
#define __ADAPTIVE_KALMAN_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 最大矩阵维度 (支持2D~4D状态) */
#define AK_MAX_DIM  4

/* ========== 矩阵结构 (小规模, 嵌入式友好) ========== */
typedef struct {
    float data[AK_MAX_DIM][AK_MAX_DIM];   /* 矩阵数据 */
    uint8_t rows;                           /* 行数 */
    uint8_t cols;                           /* 列数 */
} AK_Matrix_t;

/* ========== 自适应卡尔曼滤波器 ========== */
typedef struct {
    /* 系统维度 */
    uint8_t n;              /* 状态维度 */
    uint8_t m;              /* 测量维度 */

    /* 系统矩阵 */
    AK_Matrix_t F;          /* 状态转移矩阵 (n×n) */
    AK_Matrix_t H;          /* 测量矩阵 (m×n) */
    AK_Matrix_t Q;          /* 过程噪声协方差 (n×n) */
    AK_Matrix_t R;          /* 测量噪声协方差 (m×m) */

    /* 状态估计 */
    AK_Matrix_t x;          /* 状态向量 (n×1) */
    AK_Matrix_t P;          /* 状态协方差 (n×n) */

    /* 卡尔曼增益 */
    AK_Matrix_t K;          /* 卡尔曼增益 (n×m) */

    /* 自适应参数 */
    float d_q;              /* Q矩阵遗忘因子 (0.01~0.1) */
    float d_r;              /* R矩阵遗忘因子 (0.01~0.1) */

    /* 新息序列 (用于自适应) */
    AK_Matrix_t v;          /* 新息: v = y - H*x (m×1) */
    float innovation_norm;  /* 新息范数 (标量, 用于监测) */

    /* 自适应标志 */
    uint8_t adaptive_q;     /* 是否自适应调整Q (0: 否, 1: 是) */
    uint8_t adaptive_r;     /* 是否自适应调整R (0: 否, 1: 是) */

    /* 内部辅助矩阵 (避免动态分配) */
    AK_Matrix_t temp_nn1;   /* n×n临时矩阵 */
    AK_Matrix_t temp_nn2;   /* n×n临时矩阵 */
    AK_Matrix_t temp_mm1;   /* m×m临时矩阵 */
    AK_Matrix_t temp_nm1;   /* n×m临时矩阵 */
    AK_Matrix_t temp_n1;    /* n×1临时向量 */
    AK_Matrix_t temp_m1;    /* m×1临时向量 */
} AdaptiveKalman_t;

/* ========== 初始化接口 ========== */

/**
 * @brief 初始化自适应卡尔曼滤波器
 * @param ak 自适应卡尔曼结构体指针
 * @param n 状态维度 (1~4)
 * @param m 测量维度 (1~4)
 */
void AdaptiveKalman_Init(AdaptiveKalman_t *ak, uint8_t n, uint8_t m);

/**
 * @brief 设置系统矩阵 (状态转移矩阵F)
 * @param ak 自适应卡尔曼结构体指针
 * @param F 数据指针 (n×n, 行主序)
 */
void AdaptiveKalman_SetF(AdaptiveKalman_t *ak, const float *F);

/**
 * @brief 设置测量矩阵H
 * @param ak 自适应卡尔曼结构体指针
 * @param H 数据指针 (m×n, 行主序)
 */
void AdaptiveKalman_SetH(AdaptiveKalman_t *ak, const float *H);

/**
 * @brief 设置过程噪声协方差Q (初始值)
 * @param ak 自适应卡尔曼结构体指针
 * @param Q 数据指针 (n×n, 行主序, 对角矩阵)
 */
void AdaptiveKalman_SetQ(AdaptiveKalman_t *ak, const float *Q);

/**
 * @brief 设置测量噪声协方差R (初始值)
 * @param ak 自适应卡尔曼结构体指针
 * @param R 数据指针 (m×m, 行主序, 对角矩阵)
 */
void AdaptiveKalman_SetR(AdaptiveKalman_t *ak, const float *R);

/**
 * @brief 设置初始状态估计
 * @param ak 自适应卡尔曼结构体指针
 * @param x0 初始状态 (n×1)
 * @param P0 初始协方差 (n×n, 对角矩阵)
 */
void AdaptiveKalman_SetInitialState(AdaptiveKalman_t *ak, const float *x0, const float *P0);

/* ========== 自适应参数 ========== */

/**
 * @brief 设置自适应遗忘因子
 * @param ak 自适应卡尔曼结构体指针
 * @param d_q Q矩阵遗忘因子 (0.01~0.1, 推荐0.05)
 * @param d_r R矩阵遗忘因子 (0.01~0.1, 推荐0.05)
 *
 * 遗忘因子含义:
 *   - d越小 → 跟踪越慢, 但更稳定
 *   - d越大 → 跟踪越快, 但可能振荡
 *   - d=0 → 禁用自适应
 */
void AdaptiveKalman_SetForgettingFactor(AdaptiveKalman_t *ak, float d_q, float d_r);

/**
 * @brief 启用/禁用自适应调整
 * @param ak 自适应卡尔曼结构体指针
 * @param adaptive_q 是否自适应Q
 * @param adaptive_r 是否自适应R
 */
void AdaptiveKalman_SetAdaptiveMode(AdaptiveKalman_t *ak, uint8_t adaptive_q, uint8_t adaptive_r);

/* ========== 核心计算 ========== */

/**
 * @brief 自适应卡尔曼滤波计算 (Sage-Husa算法)
 * @param ak 自适应卡尔曼结构体指针
 * @param measurement 测量值数组 (m×1)
 * @return 估计状态指针 (n×1)
 *
 * 算法步骤:
 *   1. 预测:
 *      x(k|k-1) = F * x(k-1|k-1)
 *      P(k|k-1) = F * P(k-1|k-1) * F^T + Q
 *
 *   2. 更新:
 *      v(k) = y(k) - H * x(k|k-1)  (新息)
 *      S(k) = H * P(k|k-1) * H^T + R  (新息协方差)
 *      K(k) = P(k|k-1) * H^T * S(k)^{-1}  (卡尔曼增益)
 *      x(k|k) = x(k|k-1) + K(k) * v(k)
 *      P(k|k) = (I - K(k)*H) * P(k|k-1)
 *
 *   3. 自适应调整 (Sage-Husa):
 *      d_R(k) = (1 - alpha) * (1 - alpha^k)  (变遗忘因子)
 *      R(k) = R(k-1) + d_R * (v*v^T - R)
 *      Q(k) = Q(k-1) + d_Q * (K*v*v^T*K^T - Q)
 */
const float* AdaptiveKalman_Calculate(AdaptiveKalman_t *ak, const float *measurement);

/**
 * @brief 简化计算接口 (标量版本, 适用于1D系统)
 * @param ak 自适应卡尔曼结构体指针
 * @param measurement 测量值 (标量)
 * @return 估计状态 (标量)
 */
float AdaptiveKalman_CalculateScalar(AdaptiveKalman_t *ak, float measurement);

/**
 * @brief 重置卡尔曼滤波器状态
 * @param ak 自适应卡尔曼结构体指针
 * @param x0 重置后的状态 (可为NULL则保持当前)
 */
void AdaptiveKalman_Reset(AdaptiveKalman_t *ak, const float *x0);

/* ========== 状态获取 ========== */

/**
 * @brief 获取估计状态
 * @param ak 自适应卡尔曼结构体指针
 * @return 状态向量指针 (n×1)
 */
const float* AdaptiveKalman_GetState(AdaptiveKalman_t *ak);

/**
 * @brief 获取卡尔曼增益
 * @param ak 自适应卡尔曼结构体指针
 * @return 增益矩阵指针 (n×m)
 */
const float* AdaptiveKalman_GetGain(AdaptiveKalman_t *ak);

/**
 * @brief 获取当前Q矩阵
 */
const float* AdaptiveKalman_GetQ(AdaptiveKalman_t *ak);

/**
 * @brief 获取当前R矩阵
 */
const float* AdaptiveKalman_GetR(AdaptiveKalman_t *ak);

/**
 * @brief 获取新息范数 (用于监测滤波器健康状态)
 * @return 新息范数 (应接近0)
 *
 * 诊断:
 *   - 新息范数持续偏大 → Q太小或R太大
 *   - 新息范数持续偏小 → Q太大或R太小
 *   - 新息范数应白噪声化 (无相关性)
 */
static inline float AdaptiveKalman_GetInnovationNorm(AdaptiveKalman_t *ak)
{
    return ak->innovation_norm;
}

/* ========== 便捷初始化 (典型场景) ========== */

/**
 * @brief 一维自适应卡尔曼 (速度估计, 仅位置测量)
 * @param ak 自适应卡尔曼结构体指针
 * @param dt 采样步长 (s)
 * @param q 过程噪声标准差
 * @param r 测量噪声标准差
 */
void AdaptiveKalman_Init1D_PositionVelocity(AdaptiveKalman_t *ak, float dt,
                                            float q, float r);

/**
 * @brief 二维自适应卡尔曼 (位置+速度, 仅位置测量)
 * @param ak 自适应卡尔曼结构体指针
 * @param dt 采样步长 (s)
 * @param q_accel 加速度噪声标准差
 * @param r_pos 位置测量噪声标准差
 */
void AdaptiveKalman_Init2D_PositionVelocity(AdaptiveKalman_t *ak, float dt,
                                            float q_accel, float r_pos);

/**
 * @brief 三维自适应卡尔曼 (姿态估计, 仅陀螺仪+加速度计)
 * @param ak 自适应卡尔曼结构体指针
 * @param dt 采样步长 (s)
 */
void AdaptiveKalman_Init3D_Attitude(AdaptiveKalman_t *ak, float dt);

#ifdef __cplusplus
}
#endif

#endif /* __ADAPTIVE_KALMAN_H */
