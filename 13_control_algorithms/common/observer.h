/**
 * @file observer.h
 * @brief 状态观测器 - 龙伯格(Luenberger)全阶观测器
 *
 * 观测器方程(连续):
 *   x_hat_dot = A*x_hat + B*u + L*(y - C*x_hat)
 *
 * 离散化(前向欧拉):
 *   x_hat[k+1] = (A-LC)*x_hat[k] + B*u[k] + L*y[k]
 *
 * 使用场景:
 *   - 速度/加速度估计(无传感器)
 *   - 扭矩/负载估计
 *   - 系统状态重构
 */

#ifndef OBSERVER_H
#define OBSERVER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 最大支持状态维度 */
#define OBS_MAX_STATES  8
#define OBS_MAX_OUTPUTS 4
#define OBS_MAX_INPUTS  4

typedef struct {
    uint8_t n;       /* 状态维度 */
    uint8_t p;       /* 输出维度 */
    uint8_t m;       /* 输入维度 */

    float A[OBS_MAX_STATES][OBS_MAX_STATES];   /* 系统矩阵 */
    float B[OBS_MAX_STATES][OBS_MAX_INPUTS];    /* 输入矩阵 */
    float C[OBS_MAX_OUTPUTS][OBS_MAX_STATES];   /* 输出矩阵 */
    float L[OBS_MAX_STATES][OBS_MAX_OUTPUTS];   /* 观测器增益矩阵 */

    float x_hat[OBS_MAX_STATES];                /* 状态估计 */
    float dt;                                    /* 采样周期 */
} LuenbergerObserver_t;

/**
 * @brief 初始化观测器
 * @param obs    观测器结构体
 * @param n      状态维度
 * @param p      输出维度
 * @param m      输入维度
 * @param dt     采样周期
 */
void Observer_Init(LuenbergerObserver_t *obs, uint8_t n, uint8_t p, uint8_t m, float dt);

/**
 * @brief 设置系统矩阵 A (n x n)
 */
void Observer_SetA(LuenbergerObserver_t *obs, const float *A_data);

/**
 * @brief 设置输入矩阵 B (n x m)
 */
void Observer_SetB(LuenbergerObserver_t *obs, const float *B_data);

/**
 * @brief 设置输出矩阵 C (p x n)
 */
void Observer_SetC(LuenbergerObserver_t *obs, const float *C_data);

/**
 * @brief 设置观测器增益 L (n x p)
 * L 的选取应使 (A - L*C) 的极点位于期望位置
 */
void Observer_SetL(LuenbergerObserver_t *obs, const float *L_data);

/**
 * @brief 观测器一步更新
 * @param u  输入向量 (长度 m)
 * @param y  测量输出向量 (长度 p)
 */
void Observer_Update(LuenbergerObserver_t *obs, const float *u, const float *y);

/**
 * @brief 获取估计状态
 * @param x_hat  输出估计状态 (长度 n)
 */
void Observer_GetState(const LuenbergerObserver_t *obs, float *x_hat);

/**
 * @brief 设置初始状态估计
 */
void Observer_SetInitialState(LuenbergerObserver_t *obs, const float *x0);

/**
 * @brief 一阶速度观测器(简化接口)
 * 从位置测量估计速度,等效于一阶龙伯格观测器
 */
typedef struct {
    float x1_hat;   /* 位置估计 */
    float x2_hat;   /* 速度估计 */
    float L1, L2;   /* 观测器增益 */
    float dt;
} VelocityObserver_t;

void VelocityObserver_Init(VelocityObserver_t *vo, float L1, float L2, float dt);
float VelocityObserver_Update(VelocityObserver_t *vo, float position_meas);
float VelocityObserver_GetVelocity(const VelocityObserver_t *vo);

/**
 * @brief 扭矩/负载观测器(简化接口)
 */
typedef struct {
    float omega_hat;   /* 角速度估计 */
    float tau_hat;     /* 扭矩估计 */
    float J;           /* 转动惯量 */
    float L1, L2;      /* 观测器增益 */
    float dt;
} TorqueObserver_t;

void TorqueObserver_Init(TorqueObserver_t *to, float J, float L1, float L2, float dt);
void TorqueObserver_Update(TorqueObserver_t *to, float omega_meas, float u_voltage);
float TorqueObserver_GetTorque(const TorqueObserver_t *to);

#ifdef __cplusplus
}
#endif

#endif /* OBSERVER_H */
