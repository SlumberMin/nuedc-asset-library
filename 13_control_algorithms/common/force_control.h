/**
 * @file force_control.h
 * @brief 力控制算法库 - 阻抗控制与导纳控制
 * @version 1.0
 * @date 2026-06
 *
 * 支持:
 *   - 阻抗控制 (Impedance Control): 设定运动→输出力
 *   - 导纳控制 (Admittance Control): 测量力→输出运动
 *   - 力/位混合控制
 */

#ifndef FORCE_CONTROL_H
#define FORCE_CONTROL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== 阻抗控制 ======================== */

/** 阻抗控制器参数 (二阶: M*x'' + B*x' + K*x = F) */
typedef struct {
    float M;        /* 惯性参数 (kg) */
    float B;        /* 阻尼参数 (N·s/m) */
    float K;        /* 刚度参数 (N/m) */
    float dt;       /* 采样周期 (s) */
    /* 内部状态 */
    float x;        /* 当前位移 (m) */
    float dx;       /* 当前速度 (m/s) */
    float F_out;    /* 输出力 (N) */
} ImpedanceCtrl_t;

/** 初始化阻抗控制器 */
void Impedance_Init(ImpedanceCtrl_t *ctrl, float M, float B, float K, float dt);

/** 阻抗控制器更新
 *  @param x_des   期望位移 (m)
 *  @param dx_des  期望速度 (m/s)
 *  @param F_ext   外部干扰力 (N)
 *  @return 输出力 (N)
 */
float Impedance_Update(ImpedanceCtrl_t *ctrl, float x_des, float dx_des, float F_ext);

/** 重置阻抗控制器状态 */
void Impedance_Reset(ImpedanceCtrl_t *ctrl);

/* ======================== 导纳控制 ======================== */

/** 导纳控制器参数 */
typedef struct {
    float Md;       /* 期望惯性 (kg) */
    float Bd;       /* 期望阻尼 (N·s/m) */
    float Kd;       /* 期望刚度 (N/m) */
    float dt;       /* 采样周期 (s) */
    /* 内部状态 */
    float x_cmd;    /* 输出位置修正 (m) */
    float dx_cmd;   /* 输出速度修正 (m/s) */
    float F_err;    /* 力误差积分 */
} AdmittanceCtrl_t;

/** 初始化导纳控制器 */
void Admittance_Init(AdmittanceCtrl_t *ctrl, float Md, float Bd, float Kd, float dt);

/** 导纳控制器更新
 *  @param F_des   期望接触力 (N)
 *  @param F_meas  实际测量力 (N)
 *  @return 位置修正量 (m)
 */
float Admittance_Update(AdmittanceCtrl_t *ctrl, float F_des, float F_meas);

/** 重置导纳控制器状态 */
void Admittance_Reset(AdmittanceCtrl_t *ctrl);

/* ======================== 力/位混合控制 ======================== */

/** 选择矩阵对角元素 (1=力控, 0=位控) */
typedef struct {
    float s[6];     /* 最多6维选择向量 (x,y,z,rx,ry,rz) */
} SelectionMatrix_t;

/** 力/位混合控制器 */
typedef struct {
    SelectionMatrix_t S;    /* 选择矩阵 */
    float Kp_pos;           /* 位置环比例增益 */
    float Kp_force;         /* 力环比例增益 */
    float Ki_force;         /* 力环积分增益 */
    float dt;
    float force_integral;   /* 力误差积分 */
    float max_force;        /* 最大输出力限幅 (N) */
} HybridForcePosCtrl_t;

/** 初始化力/位混合控制器 */
void Hybrid_Init(HybridForcePosCtrl_t *ctrl, float Kp_pos, float Kp_force,
                 float Ki_force, float dt, float max_force);

/** 设置选择矩阵 (单轴简化版, dim=0~5) */
void Hybrid_SetSelectionAxis(HybridForcePosCtrl_t *ctrl, uint8_t dim, float s_val);

/** 力/位混合控制器更新 (单轴)
 *  @param pos_des    期望位置
 *  @param pos_meas   实际位置
 *  @param force_des  期望力
 *  @param force_meas 实际力
 *  @return 控制输出
 */
float Hybrid_UpdateSingleAxis(HybridForcePosCtrl_t *ctrl, float pos_des,
                               float pos_meas, float force_des, float force_meas);

#ifdef __cplusplus
}
#endif

#endif /* FORCE_CONTROL_H */
