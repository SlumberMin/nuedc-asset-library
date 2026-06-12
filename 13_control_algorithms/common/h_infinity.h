/**
 * @file h_infinity.h
 * @brief H∞鲁棒控制器 - 简化版（基于状态空间的H∞控制）
 * @version 1.0
 * @date 2026-06-11
 *
 * H∞控制核心思想：最小化闭环传递函数的H∞范数（即最坏情况增益），
 * 使系统在存在模型不确定性和外部干扰时仍能保持鲁棒性能。
 *
 * 本实现采用简化H∞次优控制（γ迭代+Riccati方程），适用于电赛中
 * 对鲁棒性要求较高的场合（如倒立摆、平衡车、伺服系统等）。
 */

#ifndef __H_INFINITY_H
#define __H_INFINITY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 最大状态维数（嵌入式友好，预分配内存） */
#define HINF_MAX_STATES   8

/* H∞控制器工作模式 */
typedef enum {
    HINF_MODE_STATIC_GAIN = 0,  /* 静态状态反馈 u = -K*x */
    HINF_MODE_OUTPUT_FEEDBACK    /* 输出反馈（含观测器） */
} HInfMode_e;

/* H∞控制器结构体 */
typedef struct {
    /* 系统维数 */
    uint8_t n;          /* 状态维数 */
    uint8_t m;          /* 输入维数 */
    uint8_t p;          /* 输出维数 */

    /* 状态空间矩阵（行主序，一维数组存储） */
    float A[HINF_MAX_STATES * HINF_MAX_STATES];   /* 系统矩阵 n×n */
    float B[HINF_MAX_STATES * HINF_MAX_STATES];   /* 输入矩阵 n×m */
    float C[HINF_MAX_STATES * HINF_MAX_STATES];   /* 输出矩阵 p×n */

    /* 加权矩阵 */
    float Q[HINF_MAX_STATES * HINF_MAX_STATES];   /* 状态加权 n×n（半正定） */
    float R[HINF_MAX_STATES * HINF_MAX_STATES];   /* 输入加权 m×m（正定） */

    /* H∞增益矩阵K（计算结果） */
    float K[HINF_MAX_STATES * HINF_MAX_STATES];   /* m×n 状态反馈增益 */

    /* γ参数（H∞性能指标上界） */
    float gamma;        /* 当前γ值 */
    float gamma_min;    /* γ搜索下界 */
    float gamma_max;    /* γ搜索上界 */
    float gamma_tol;    /* γ收敛容差 */

    /* Riccati方程求解参数 */
    float P[HINF_MAX_STATES * HINF_MAX_STATES];   /* Riccati方程解 n×n */
    uint16_t riccati_max_iter;   /* 最大迭代次数 */
    float riccati_tol;           /* 收敛容差 */

    /* 干扰衰减矩阵B2（n×m_d，干扰输入） */
    float B2[HINF_MAX_STATES * HINF_MAX_STATES];
    uint8_t m_d;        /* 干扰输入维数 */

    /* 内部状态 */
    float x[HINF_MAX_STATES];    /* 状态向量 */
    float u[HINF_MAX_STATES];    /* 控制输出 */

    /* 模式 */
    HInfMode_e mode;
} HInf_t;

/**
 * @brief 初始化H∞控制器
 * @param ctrl 控制器结构体指针
 * @param n 状态维数
 * @param m 输入维数
 * @param p 输出维数
 */
void HInf_Init(HInf_t *ctrl, uint8_t n, uint8_t m, uint8_t p);

/**
 * @brief 设置系统状态空间矩阵 (A, B, C)
 */
void HInf_SetPlant(HInf_t *ctrl,
                    const float *A, const float *B, const float *C);

/**
 * @brief 设置加权矩阵 (Q, R)
 * @param Q 状态加权矩阵 n×n（半正定）
 * @param R 输入加权矩阵 m×m（正定）
 */
void HInf_SetWeight(HInf_t *ctrl, const float *Q, const float *R);

/**
 * @brief 设置干扰输入矩阵B2
 */
void HInf_SetDisturbance(HInf_t *ctrl, const float *B2, uint8_t m_d);

/**
 * @brief 求解代数Riccati方程并计算H∞增益K
 * @param gamma γ值（0表示自动搜索最小γ）
 * @return 0=成功, -1=Riccati不收敛, -2=γ过大
 */
int HInf_Solve(HInf_t *ctrl, float gamma);

/**
 * @brief H∞控制器计算一步
 * @param ctrl 控制器结构体指针
 * @param x_ref 参考状态向量（NULL表示调节问题）
 * @param x_meas 当前测量状态
 * @return 控制输出指针（内部存储）
 */
float* HInf_Compute(HInf_t *ctrl, const float *x_ref, const float *x_meas);

/**
 * @brief 获取状态反馈增益K
 */
void HInf_GetGain(HInf_t *ctrl, float *K_out);

/**
 * @brief 复位内部状态
 */
void HInf_Reset(HInf_t *ctrl);

#ifdef __cplusplus
}
#endif

#endif /* __H_INFINITY_H */
