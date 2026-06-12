/**
 * @file deadbeat.c
 * @brief 无差拍(Deadbeat)控制器实现
 * @details 无差拍控制器是一种离散时间最优控制器, 目标是在有限拍数内
 *          将系统状态驱动到期望值。适用于数字控制的快速响应场景。
 *
 *          支持一阶和二阶系统, 通过极点配置将闭环极点置于原点。
 *          控制律: u = -K*x + Kr*r
 */

#include "deadbeat.h"
#include <string.h>
#include <math.h>

/**
 * @brief 初始化无差拍控制器(通用)
 * @param ctrl 控制器结构体指针
 * @param n 状态维度
 */
void Deadbeat_Init(DeadbeatCtrl_t *ctrl, uint8_t n)
{
    if (ctrl == NULL) return;
    memset(ctrl, 0, sizeof(DeadbeatCtrl_t));
    ctrl->n = (n > DB_STATE_DIM) ? DB_STATE_DIM : n;
    ctrl->out_min = -1e30f;
    ctrl->out_max =  1e30f;
    ctrl->Kr = 1.0f;
}

/**
 * @brief 二阶系统无差拍初始化
 * @param ctrl 控制器结构体指针
 * @param Ts 采样时间(秒)
 * @param tau 系统时间常数(秒)
 * @param gain 系统稳态增益
 *
 * @details 模型: G(z) = gain * (1 - e^(-Ts/tau)) / (z - e^(-Ts/tau))
 *          对于典型一阶惯性环节, 离散化:
 *          A = [[a]], B = [[b]], C = [[1]]
 *          其中 a = exp(-Ts/tau), b = gain*(1-a)
 */
void Deadbeat_Init2nd(DeadbeatCtrl_t *ctrl, float Ts, float tau, float gain)
{
    if (ctrl == NULL) return;
    /* 防除零保护 */
    if (fabsf(tau) < 1e-6f) tau = 1e-6f;
    if (Ts <= 0.0f) Ts = 0.001f;

    float a = expf(-Ts / tau);
    float b = gain * (1.0f - a);

    Deadbeat_Init(ctrl, 1);

    /* 离散状态空间模型: x[k+1] = a*x[k] + b*u[k], y = x[k] */
    ctrl->A[0][0] = a;
    ctrl->B[0][0] = b;
    ctrl->C[0][0] = 1.0f;

    /* 无差拍增益: 将极点配置到原点 → K = a/b */
    if (fabsf(b) < 1e-6f) b = 1e-6f;  /* 防除零 */
    float K_val = a / b;
    ctrl->K[0][0] = K_val;
    /* 前馈增益: Kr = 1/(b + a*K) */
    ctrl->Kr = 1.0f / (b + a * K_val);
}

/**
 * @brief 设置系统模型矩阵
 * @param ctrl 控制器结构体指针
 * @param A 状态矩阵数据(行优先 n×n)
 * @param B 输入矩阵数据(长度n)
 * @param C 输出矩阵数据(长度n)
 */
void Deadbeat_SetModel(DeadbeatCtrl_t *ctrl,
                        const float *A, const float *B, const float *C)
{
    if (ctrl == NULL || A == NULL || B == NULL || C == NULL) return;
    uint8_t n = ctrl->n;
    for (uint8_t i = 0; i < n; i++) {
        for (uint8_t j = 0; j < n; j++) {
            ctrl->A[i][j] = A[i * n + j];
        }
        ctrl->B[i][0] = B[i];
    }
    for (uint8_t j = 0; j < n; j++) {
        ctrl->C[0][j] = C[j];
    }
}

/**
 * @brief 设置控制器增益
 * @param ctrl 控制器结构体指针
 * @param K 状态反馈增益数组(长度n)
 * @param Kr 前馈增益
 */
void Deadbeat_SetGains(DeadbeatCtrl_t *ctrl, const float *K, float Kr)
{
    if (ctrl == NULL || K == NULL) return;
    for (uint8_t j = 0; j < ctrl->n; j++) {
        ctrl->K[0][j] = K[j];
    }
    ctrl->Kr = Kr;
}

/**
 * @brief 设置输出限幅
 * @param ctrl 控制器结构体指针
 * @param min 输出下限
 * @param max 输出上限
 */
void Deadbeat_SetOutputLimit(DeadbeatCtrl_t *ctrl, float min, float max)
{
    if (ctrl == NULL) return;
    ctrl->out_min = min;
    ctrl->out_max = max;
}

/**
 * @brief 执行一步无差拍控制计算
 * @param ctrl 控制器结构体指针
 * @param setpoint 设定值
 * @param feedback 反馈测量值
 * @return 控制器输出
 *
 * @details 计算流程:
 *          1. 输出校正: 用测量值修正状态估计
 *          2. 控制律: u = -K*x + Kr*r
 *          3. 状态更新: x = A*x + B*u
 */
float Deadbeat_Compute(DeadbeatCtrl_t *ctrl, float setpoint, float feedback)
{
    if (ctrl == NULL) return 0.0f;

    uint8_t n = ctrl->n;

    /* 状态校正: 利用输出误差修正估计状态 */
    float y_hat = 0.0f;
    for (uint8_t j = 0; j < n; j++) {
        y_hat += ctrl->C[0][j] * ctrl->x[j];
    }
    float correction = feedback - y_hat;
    ctrl->x[0] += correction;  /* 简单输出校正 */

    /* 计算控制量: u = -K*x + Kr*r */
    float u = ctrl->Kr * setpoint;
    for (uint8_t j = 0; j < n; j++) {
        u -= ctrl->K[0][j] * ctrl->x[j];
    }

    /* 输出限幅 */
    if (u > ctrl->out_max) u = ctrl->out_max;
    if (u < ctrl->out_min) u = ctrl->out_min;

    /* 状态更新: x_new = A*x + B*u */
    float x_new[DB_STATE_DIM];
    for (uint8_t i = 0; i < n; i++) {
        x_new[i] = 0.0f;
        for (uint8_t j = 0; j < n; j++) {
            x_new[i] += ctrl->A[i][j] * ctrl->x[j];
        }
        x_new[i] += ctrl->B[i][0] * u;
    }
    /* 写回状态 */
    for (uint8_t i = 0; i < n; i++) {
        ctrl->x[i] = x_new[i];
    }

    ctrl->u_last = u;
    ctrl->output = u;
    return u;
}

/**
 * @brief 独立的观测器更新步骤
 * @param ctrl 控制器结构体指针
 * @param y_meas 测量输出值
 */
void Deadbeat_UpdateObserver(DeadbeatCtrl_t *ctrl, float y_meas)
{
    if (ctrl == NULL) return;
    /* 简单输出误差校正观测器 */
    float y_hat = 0.0f;
    for (uint8_t j = 0; j < ctrl->n; j++) {
        y_hat += ctrl->C[0][j] * ctrl->x[j];
    }
    float err = y_meas - y_hat;
    ctrl->x[0] += err;
}

/**
 * @brief 重置无差拍控制器状态
 * @param ctrl 控制器结构体指针
 */
void Deadbeat_Reset(DeadbeatCtrl_t *ctrl)
{
    if (ctrl == NULL) return;
    memset(ctrl->x, 0, sizeof(float) * DB_STATE_DIM);
    ctrl->u_last = 0.0f;
    ctrl->output = 0.0f;
}
