#ifndef __DEADBEAT_H
#define __DEADBEAT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 无差拍控制器 (Deadbeat Control)
 * @note  基于离散状态空间模型, 在有限拍数(通常1~2个采样周期)内
 *        使输出精确跟踪设定值。适用于数字控制逆变器、电源等。
 *
 *        离散模型: x[k+1] = A*x[k] + B*u[k], y[k] = C*x[k]
 *        控制律:   u[k] = -K*x[k] + Kr*r[k+1]
 *        通过极点配置将闭环极点置于原点, 实现最少拍响应。
 */

#define DB_STATE_DIM   4   /* 最大状态维数, 可按需调整 */
#define DB_INPUT_DIM   1
#define DB_OUTPUT_DIM  1

typedef struct {
    uint8_t n;  /* 实际状态维数 */

    /* 系统矩阵 (离散) */
    float A[DB_STATE_DIM][DB_STATE_DIM];
    float B[DB_STATE_DIM][DB_INPUT_DIM];
    float C[DB_OUTPUT_DIM][DB_STATE_DIM];

    /* 状态反馈增益 */
    float K[DB_INPUT_DIM][DB_STATE_DIM];
    /* 前馈增益 */
    float Kr;

    /* 状态估计 */
    float x[DB_STATE_DIM];

    /* 输出限幅 */
    float out_min;
    float out_max;

    /* 上一次输出 */
    float u_last;

    float output;
} DeadbeatCtrl_t;

/**
 * @brief 初始化无差拍控制器 (二阶系统简化版)
 * @param Ts  采样周期(s)
 * @param tau 系统时间常数(s)
 * @param gain 系统直流增益
 */
void Deadbeat_Init2nd(DeadbeatCtrl_t *ctrl, float Ts, float tau, float gain);

/**
 * @brief 初始化无差拍控制器 (通用版, 手动设置A/B/C/K)
 */
void Deadbeat_Init(DeadbeatCtrl_t *ctrl, uint8_t n);

/**
 * @brief 设置系统矩阵
 */
void Deadbeat_SetModel(DeadbeatCtrl_t *ctrl,
                        const float *A, const float *B, const float *C);

/**
 * @brief 设置反馈增益和前馈增益
 */
void Deadbeat_SetGains(DeadbeatCtrl_t *ctrl, const float *K, float Kr);

/**
 * @brief 设置输出限幅
 */
void Deadbeat_SetOutputLimit(DeadbeatCtrl_t *ctrl, float min, float max);

/**
 * @brief 计算无差拍控制输出
 * @param setpoint 设定值
 * @param feedback 当前输出反馈
 * @return 控制量
 */
float Deadbeat_Compute(DeadbeatCtrl_t *ctrl, float setpoint, float feedback);

/**
 * @brief 重置状态
 */
void Deadbeat_Reset(DeadbeatCtrl_t *ctrl);

/**
 * @brief 更新状态观测 (用于降阶观测器或全阶观测器场景)
 */
void Deadbeat_UpdateObserver(DeadbeatCtrl_t *ctrl, float y_meas);

#ifdef __cplusplus
}
#endif

#endif /* __DEADBEAT_H */
