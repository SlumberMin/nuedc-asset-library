/**
 * @file adrc.h
 * @brief ADRC自抗扰控制器 - TD + ESO + NLSEF
 * @version 1.0
 * @date 2026-06-10
 * 
 * 特点: 不依赖精确模型, 自动估计和补偿扰动
 * 应用: 电机控制、飞行器姿态、高精度位置控制
 */

#ifndef __ADRC_H
#define __ADRC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 跟踪微分器(TD) ========== */
typedef struct {
    float r;       /* 速度因子, 越大跟踪越快 */
    float h;       /* 采样步长 */
    float x1;      /* 跟踪信号 */
    float x2;      /* 跟踪信号的微分 */
} ADRC_TD_t;

/* ========== 扩张状态观测器(ESO) ========== */
typedef struct {
    float beta1, beta2, beta3;  /* 观测器增益 */
    float alpha1, alpha2;       /* 非线性因子 */
    float delta;                /* 线性区间宽度 */
    float b;                    /* 补偿系数 */
    float z1, z2, z3;          /* 状态估计: z1=y, z2=ẏ, z3=总扰动 */
} ADRC_ESO_t;

/* ========== 非线性状态误差反馈(NLSEF) ========== */
typedef struct {
    float beta0, beta1;         /* 误差反馈增益 */
    float alpha0, alpha1;       /* 非线性因子(0~1) */
    float delta;                /* 线性区间 */
} ADRC_NLSEF_t;

/* ========== ADRC主结构体 ========== */
typedef struct {
    ADRC_TD_t td;
    ADRC_ESO_t eso;
    ADRC_NLSEF_t nlsef;
    float h;        /* 采样步长 */
    float b;        /* 系统增益估计 */
    float output;
} ADRC_t;

/* ========== 接口 ========== */

/**
 * @brief 初始化ADRC
 * @param adrc ADRC结构体
 * @param h 采样步长(s)
 * @param b 系统增益估计
 */
void ADRC_Init(ADRC_t *adrc, float h, float b);

/**
 * @brief 设置TD参数
 * @param r 速度因子(推荐5~100)
 */
void ADRC_SetTD(ADRC_t *adrc, float r);

/**
 * @brief 设置ESO参数
 * @param beta1 观测器增益1(推荐10~100)
 * @param beta2 观测器增益2
 * @param beta3 观测器增益3
 */
void ADRC_SetESO(ADRC_t *adrc, float beta1, float beta2, float beta3);

/**
 * @brief 设置NLSEF参数
 */
void ADRC_SetNLSEF(ADRC_t *adrc, float beta0, float beta1, float alpha0, float alpha1);

/**
 * @brief ADRC计算
 * @param target 目标值
 * @param measurement 测量值
 * @return 控制输出
 */
float ADRC_Calculate(ADRC_t *adrc, float target, float measurement);

/**
 * @brief 重置ADRC
 */
void ADRC_Reset(ADRC_t *adrc);

#ifdef __cplusplus
}
#endif

#endif /* __ADRC_H */
