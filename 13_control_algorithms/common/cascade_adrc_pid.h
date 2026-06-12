/**
 * @file cascade_adrc_pid.h
 * @brief ADRC+PID级联控制器
 * 
 * 外环使用PID控制，内环使用自抗扰控制(ADRC/ESO)的级联结构。
 * 结合了PID的工程实用性与ADRC的强鲁棒性。
 * 
 * 结构：
 *   外环PID: 位置/速度误差 -> 内环参考输入
 *   内环ADRC: 扩张状态观测器(ESO) + 非线性状态误差反馈(NLSEF)
 * 
 * ESO: 估计系统状态和总扰动(含未建模动态+外部扰动)
 *   z1_dot = z2 + β1*(y - z1)
 *   z2_dot = z3 + β2*(y - z1) + b0*u
 *   z3_dot = β3*(y - z1)
 *   u = (u0 - z3) / b0  (扰动补偿)
 * 
 * 典型应用：电机调速系统、飞行器姿态控制、工业过程控制
 * 
 * 参数整定指南：
 *   外环PID：
 *     - Kp_outer: 从小(0.5)开始调大
 *     - Ki_outer: 消除稳态误差
 *     - Kd_outer: 抑制超调
 *   内环ADRC：
 *     - β1, β2, β3: ESO增益，通常β1=3ωo, β2=3ωo², β3=ωo³
 *     - ωo: 观测器带宽，比闭环带宽大3~5倍
 *     - b0: 控制增益估计，与系统实际b接近
 *     - Kp_inner, Kd_inner: 内环PD增益
 */

#ifndef CASCADE_ADRC_PID_H
#define CASCADE_ADRC_PID_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* === 外环PID参数 === */
    float Kp_outer;     /* 外环比例增益 */
    float Ki_outer;     /* 外环积分增益 */
    float Kd_outer;     /* 外环微分增益 */
    float dt_outer;     /* 外环采样周期(s) */
    float out_min;      /* 外环输出限幅(最小) */
    float out_max;      /* 外环输出限幅(最大) */

    /* === 内环ADRC参数 === */
    float Kp_inner;     /* 内环比例增益 */
    float Kd_inner;     /* 内环微分增益 */
    float b0;           /* 控制增益估计 */
    float Ts;           /* 内环采样周期(s) */
    float u_min;        /* 内环输出限幅(最小) */
    float u_max;        /* 内环输出限幅(最大) */

    /* === ESO参数 === */
    float beta1;        /* ESO增益1 */
    float beta2;        /* ESO增益2 */
    float beta3;        /* ESO增益3 */

    /* === 外环运行时变量 === */
    float outer_integral;
    float outer_err_prev;
    float outer_ref;     /* 外环参考(期望位置/速度) */

    /* === 内环运行时变量 === */
    float inner_ref;     /* 内环参考(外环输出) */
    float inner_err_prev;

    /* === ESO状态 === */
    float z1;           /* 状态1估计(输出) */
    float z2;           /* 状态2估计(速度) */
    float z3;           /* 状态3估计(总扰动) */
    float u_prev;       /* 上一时刻控制量 */
} CascadeAdrcPid_t;

/**
 * @brief 初始化级联ADRC+PID控制器
 * @param ctrl 控制器句柄
 * @param Ts   内环采样周期(s)
 * @param dt   外环采样周期(s)，通常为Ts的整数倍
 * @return 0=成功
 */
int CAP_Init(CascadeAdrcPid_t *ctrl, float Ts, float dt);

/**
 * @brief 设置外环PID参数
 */
void CAP_SetOuterPID(CascadeAdrcPid_t *ctrl, float Kp, float Ki, float Kd);

/**
 * @brief 设置内环ADRC参数
 * @param Kp    内环比例增益
 * @param Kd    内环微分增益
 * @param b0    控制增益估计
 * @param omega_o ESO带宽(rad/s)
 */
void CAP_SetInnerADRC(CascadeAdrcPid_t *ctrl, float Kp, float Kd, float b0, float omega_o);

/**
 * @brief 设置输出限幅
 * @param out_min/max 外环输出限幅
 * @param u_min/max   内环输出限幅(最终控制量)
 */
void CAP_SetLimits(CascadeAdrcPid_t *ctrl,
                   float out_min, float out_max,
                   float u_min, float u_max);

/**
 * @brief 级联ADRC+PID控制计算
 * @param ctrl      控制器句柄
 * @param outer_ref 外环参考值(期望位置/速度)
 * @param outer_fbk 外环反馈值(实际位置/速度)
 * @param inner_fbk 内环反馈值(实际电流/力矩等)
 * @return 最终控制输出(如PWM占空比)
 */
float CAP_Compute(CascadeAdrcPid_t *ctrl, float outer_ref, float outer_fbk, float inner_fbk);

/**
 * @brief 仅执行内环ADRC计算(外环不更新)
 * @param ref 内环参考值
 * @param fbk 内环反馈值
 * @return 控制输出
 */
float CAP_InnerLoop(CascadeAdrcPid_t *ctrl, float ref, float fbk);

/**
 * @brief 重置控制器状态
 */
void CAP_Reset(CascadeAdrcPid_t *ctrl);

#ifdef __cplusplus
}
#endif

#endif /* CASCADE_ADRC_PID_H */
