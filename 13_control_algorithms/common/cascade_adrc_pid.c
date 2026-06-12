/**
 * @file cascade_adrc_pid.c
 * @brief ADRC+PID级联控制器实现
 * 
 * 外环PID生成内环参考，内环ADRC(ESO+NLSEF)实现鲁棒控制。
 */

#include "cascade_adrc_pid.h"
#include <string.h>

/* 工具函数：限幅 */
static float cap_clamp(float val, float min, float max)
{
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

int CAP_Init(CascadeAdrcPid_t *ctrl, float Ts, float dt)
{
    if (!ctrl || Ts <= 0 || dt <= 0)
        return -1;

    ctrl->Ts       = Ts;
    ctrl->dt_outer = dt;

    /* 外环PID默认值 */
    ctrl->Kp_outer = 1.0f;
    ctrl->Ki_outer = 0.1f;
    ctrl->Kd_outer = 0.0f;
    ctrl->out_min  = -1000.0f;
    ctrl->out_max  =  1000.0f;

    /* 内环ADRC默认值 */
    ctrl->Kp_inner = 10.0f;
    ctrl->Kd_inner = 1.0f;
    ctrl->b0       = 1.0f;
    ctrl->u_min    = -1000.0f;
    ctrl->u_max    =  1000.0f;

    /* ESO默认增益(对应ωo=100 rad/s) */
    float omega_o = 100.0f;
    ctrl->beta1 = 3.0f * omega_o;
    ctrl->beta2 = 3.0f * omega_o * omega_o;
    ctrl->beta3 = omega_o * omega_o * omega_o;

    CAP_Reset(ctrl);
    return 0;
}

void CAP_SetOuterPID(CascadeAdrcPid_t *ctrl, float Kp, float Ki, float Kd)
{
    if (!ctrl) return;
    ctrl->Kp_outer = Kp;
    ctrl->Ki_outer = Ki;
    ctrl->Kd_outer = Kd;
}

void CAP_SetInnerADRC(CascadeAdrcPid_t *ctrl, float Kp, float Kd, float b0, float omega_o)
{
    if (!ctrl) return;
    ctrl->Kp_inner = Kp;
    ctrl->Kd_inner = Kd;
    ctrl->b0       = b0;
    /* 根据观测器带宽计算ESO增益 */
    ctrl->beta1 = 3.0f * omega_o;
    ctrl->beta2 = 3.0f * omega_o * omega_o;
    ctrl->beta3 = omega_o * omega_o * omega_o;
}

void CAP_SetLimits(CascadeAdrcPid_t *ctrl,
                   float out_min, float out_max,
                   float u_min, float u_max)
{
    if (!ctrl) return;
    ctrl->out_min = out_min;
    ctrl->out_max = out_max;
    ctrl->u_min   = u_min;
    ctrl->u_max   = u_max;
}

/**
 * @brief 外环PID计算
 * @return 内环参考值
 */
static float CAP_OuterPID(CascadeAdrcPid_t *ctrl, float ref, float fbk)
{
    float err = ref - fbk;
    float dt  = ctrl->dt_outer;

    /* 积分项(带抗饱和) */
    ctrl->outer_integral += err * dt;
    /* 积分限幅 */
    float integral_limit = ctrl->out_max / (ctrl->Ki_outer > 1e-6f ? ctrl->Ki_outer : 1.0f);
    ctrl->outer_integral = cap_clamp(ctrl->outer_integral, -integral_limit, integral_limit);

    /* 微分项 */
    float derivative = (err - ctrl->outer_err_prev) / dt;

    /* PID输出 */
    float output = ctrl->Kp_outer * err
                 + ctrl->Ki_outer * ctrl->outer_integral
                 + ctrl->Kd_outer * derivative;

    ctrl->outer_err_prev = err;
    output = cap_clamp(output, ctrl->out_min, ctrl->out_max);

    return output;
}

/**
 * @brief ESO(扩张状态观测器)更新
 * 
 * 估计状态z1(输出)、z2(速度)、z3(总扰动)
 * 使用前向欧拉离散化
 */
static void CAP_ESOUpdate(CascadeAdrcPid_t *ctrl, float y, float u)
{
    float Ts = ctrl->Ts;

    /* 观测误差 */
    float e = y - ctrl->z1;

    /* ESO状态更新(前向欧拉) */
    /* z1_dot = z2 + beta1*e */
    /* z2_dot = z3 + beta2*e + b0*u */
    /* z3_dot = beta3*e */
    float z1_new = ctrl->z1 + Ts * (ctrl->z2 + ctrl->beta1 * e);
    float z2_new = ctrl->z2 + Ts * (ctrl->z3 + ctrl->beta2 * e + ctrl->b0 * u);
    float z3_new = ctrl->z3 + Ts * (ctrl->beta3 * e);

    ctrl->z1 = z1_new;
    ctrl->z2 = z2_new;
    ctrl->z3 = z3_new;
}

/**
 * @brief 内环ADRC计算(ESO + NLSEF + 扰动补偿)
 */
static float CAP_InnerADRC(CascadeAdrcPid_t *ctrl, float ref, float fbk)
{
    /* 误差 */
    float e1 = ref - ctrl->z1;
    float e2 = 0.0f - ctrl->z2;  /* 期望速度为0 */

    /* 非线性状态误差反馈(NLSEF): 线性PD形式 */
    float u0 = ctrl->Kp_inner * e1 + ctrl->Kd_inner * e2;

    /* 扰动补偿: u = (u0 - z3) / b0 */
    float u;
    if (ctrl->b0 > 1e-6f || ctrl->b0 < -1e-6f)
        u = (u0 - ctrl->z3) / ctrl->b0;
    else
        u = u0;

    /* 输出限幅 */
    u = cap_clamp(u, ctrl->u_min, ctrl->u_max);

    /* 更新ESO(使用当前控制量和反馈) */
    CAP_ESOUpdate(ctrl, fbk, u);

    ctrl->u_prev = u;
    return u;
}

float CAP_Compute(CascadeAdrcPid_t *ctrl, float outer_ref, float outer_fbk, float inner_fbk)
{
    if (!ctrl) return 0.0f;

    /* 外环PID: 生成内环参考 */
    float inner_ref = CAP_OuterPID(ctrl, outer_ref, outer_fbk);

    /* 内环ADRC: 跟踪内环参考 */
    return CAP_InnerADRC(ctrl, inner_ref, inner_fbk);
}

float CAP_InnerLoop(CascadeAdrcPid_t *ctrl, float ref, float fbk)
{
    if (!ctrl) return 0.0f;
    return CAP_InnerADRC(ctrl, ref, fbk);
}

void CAP_Reset(CascadeAdrcPid_t *ctrl)
{
    if (!ctrl) return;

    ctrl->outer_integral  = 0.0f;
    ctrl->outer_err_prev  = 0.0f;
    ctrl->outer_ref       = 0.0f;
    ctrl->inner_ref       = 0.0f;
    ctrl->inner_err_prev  = 0.0f;

    ctrl->z1     = 0.0f;
    ctrl->z2     = 0.0f;
    ctrl->z3     = 0.0f;
    ctrl->u_prev = 0.0f;
}
