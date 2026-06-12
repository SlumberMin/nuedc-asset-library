/**
 * @file    advanced_pid.h
 * @brief   高级PID控制器 — ADRC / LQR / SMC 统一接口
 *
 * 基于电赛资产库算法简化移植，适合MSPM0G3507运行。
 * 统一接口：Init → Update → Reset，参数可在线调整。
 */

#ifndef __ADVANCED_PID_H
#define __ADVANCED_PID_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ═══════════════════════════════════════════════════════════════
 *  算法类型枚举
 * ═══════════════════════════════════════════════════════════════ */
typedef enum {
    CTRL_ALGO_ADRC = 0,   /* 自抗扰控制 */
    CTRL_ALGO_LQR,        /* 线性二次调节器 */
    CTRL_ALGO_SMC,        /* 滑模控制 */
    CTRL_ALGO_COUNT
} CtrlAlgoType;

/* ═══════════════════════════════════════════════════════════════
 *  ADRC — 自抗扰控制 (简化嵌入式版)
 *  TD + ESO + 非线性反馈
 * ═══════════════════════════════════════════════════════════════ */
typedef struct {
    /* TD 跟踪微分器 */
    float r0;            /* 速度因子 */
    float h0;            /* 滤波因子 */
    float v1, v2;        /* TD输出：跟踪值、微分值 */

    /* ESO 扩张状态观测器 */
    float beta01, beta02, beta03;  /* ESO增益 */
    float z1, z2, z3;    /* 位置估计、速度估计、扰动估计 */

    /* NLSEF 非线性反馈 */
    float kp, kd;        /* 比例、微分增益 */
    float b0;            /* 系统增益 */
    float delta;         /* fal函数线性区间 */

    /* 运行时 */
    float dt;            /* 采样周期 */
    float u;             /* 当前输出 */
    float u_max;         /* 输出限幅 */
} ADRC_t;

void  ADRC_Init(ADRC_t *ctrl, float r0, float h0, float b0,
                float omega_c, float omega_o, float delta, float dt);
float ADRC_Update(ADRC_t *ctrl, float ref, float y);
void  ADRC_Reset(ADRC_t *ctrl);
void  ADRC_SetOutputLimit(ADRC_t *ctrl, float max);

/* ── 在线参数调整 ──────────────────────────────── */
void  ADRC_SetOmegaC(ADRC_t *ctrl, float omega_c);
void  ADRC_SetOmegaO(ADRC_t *ctrl, float omega_o);
void  ADRC_SetB0(ADRC_t *ctrl, float b0);

/* ═══════════════════════════════════════════════════════════════
 *  LQR — 线性二次调节器 (2阶简化版)
 *  u = -K*x, K由离散Riccati方程离线求解
 * ═══════════════════════════════════════════════════════════════ */
typedef struct {
    /* 系统矩阵 2x2 */
    float A[2][2];
    float B[2];

    /* 权重矩阵 */
    float Q[2][2];
    float R;

    /* 反馈增益 */
    float K[2];

    /* Riccati解 */
    float P[2][2];

    /* 运行时 */
    float u_max;
    float dt;
} LQR_t;

void  LQR_Init(LQR_t *ctrl, float dt);
void  LQR_SetSystem(LQR_t *ctrl, float a11, float a12, float a21, float a22,
                    float b1, float b2);
void  LQR_SetWeight(LQR_t *ctrl, float q1, float q2, float r);
int   LQR_SolveRiccati(LQR_t *ctrl, int max_iter);
float LQR_Update(LQR_t *ctrl, float x1, float x2);
void  LQR_SetOutputLimit(LQR_t *ctrl, float max);
void  LQR_Reset(LQR_t *ctrl);

/* ── 在线参数调整 ──────────────────────────────── */
void  LQR_SetWeightOnline(LQR_t *ctrl, float q1, float q2, float r);

/* ═══════════════════════════════════════════════════════════════
 *  SMC — 滑模控制 (指数趋近律简化版)
 *  s = e_dot + c*e,  u = u_eq + u_sw
 * ═══════════════════════════════════════════════════════════════ */
typedef struct {
    float c;             /* 滑模面参数 */
    float eps;           /* 切换增益 */
    float k;             /* 指数趋近增益 */
    float phi;           /* 边界层厚度 */
    float u_max;         /* 输出限幅 */
    float dt;
} SMC_t;

void  SMC_Init(SMC_t *ctrl, float c, float eps, float k, float phi, float dt);
float SMC_Update(SMC_t *ctrl, float e, float e_dot, float u_eq);
void  SMC_Reset(SMC_t *ctrl);
void  SMC_SetOutputLimit(SMC_t *ctrl, float max);

/* ── 在线参数调整 ──────────────────────────────── */
void  SMC_SetC(SMC_t *ctrl, float c);
void  SMC_SetEps(SMC_t *ctrl, float eps);
void  SMC_SetK(SMC_t *ctrl, float k);

/* ═══════════════════════════════════════════════════════════════
 *  统一控制器封装
 * ═══════════════════════════════════════════════════════════════ */
typedef struct {
    CtrlAlgoType type;
    union {
        ADRC_t adrc;
        LQR_t  lqr;
        SMC_t  smc;
    } algo;
} AdvCtrl_t;

/**
 * @brief 初始化统一控制器
 * @param ctrl   控制器实例
 * @param type   算法类型
 * @param dt     采样周期(秒)
 * @param params 算法参数数组，不同算法含义不同：
 *   ADRC: [r0, h0, b0, omega_c, omega_o, delta]
 *   LQR:  [a11, a12, a21, a22, b1, b2, q1, q2, r]
 *   SMC:  [c, eps, k, phi]
 */
void  AdvCtrl_Init(AdvCtrl_t *ctrl, CtrlAlgoType type, float dt, const float *params);

/**
 * @brief 更新控制输出
 * @param ref  ADRC:目标值 y; LQR:x1; SMC:误差e
 * @param y    ADRC:实际输出; LQR:x2; SMC:误差微分e_dot
 * @param aux  ADRC:未使用; LQR:未使用; SMC:等效控制u_eq
 * @return 控制输出
 */
float AdvCtrl_Update(AdvCtrl_t *ctrl, float ref, float y, float aux);

void  AdvCtrl_Reset(AdvCtrl_t *ctrl);
void  AdvCtrl_SetOutputLimit(AdvCtrl_t *ctrl, float max);

#ifdef __cplusplus
}
#endif

#endif /* __ADVANCED_PID_H */

/* ═══════════════════════════════════════════════════════════════
 *  PID — 标准PID控制器 (供示例程序和简单控制场景使用)
 * ═══════════════════════════════════════════════════════════════ */
#ifndef __ADVANCED_PID_H_PID
#define __ADVANCED_PID_H_PID

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;
    float ki;
    float kd;
    float output_min;
    float output_max;
    float integral_max;
    float dead_zone;
} PID_Param;

typedef struct {
    PID_Param param;
    float integral;
    float prev_error;
    float output;
} PID_Controller;

void  PID_Init(PID_Controller *pid, const PID_Param *param);
float PID_Calc(PID_Controller *pid, float ref, float feedback);
void  PID_Reset(PID_Controller *pid);
void  PID_SetKp(PID_Controller *pid, float kp);
void  PID_SetKi(PID_Controller *pid, float ki);
void  PID_SetKd(PID_Controller *pid, float kd);
float PID_GetOutput(const PID_Controller *pid);

#ifdef __cplusplus
}
#endif

#endif /* __ADVANCED_PID_H_PID */
