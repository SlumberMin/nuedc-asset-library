/**
 * @file force_control.c
 * @brief 力控制算法实现
 */

#include "force_control.h"
#include <math.h>

/* ======================== 工具函数 ======================== */

static float clampf(float val, float min_val, float max_val)
{
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

/* ======================== 阻抗控制实现 ======================== */

void Impedance_Init(ImpedanceCtrl_t *ctrl, float M, float B, float K, float dt)
{
    ctrl->M = M;
    ctrl->B = B;
    ctrl->K = K;
    ctrl->dt = dt;
    ctrl->x = 0.0f;
    ctrl->dx = 0.0f;
    ctrl->F_out = 0.0f;
}

float Impedance_Update(ImpedanceCtrl_t *ctrl, float x_des, float dx_des, float F_ext)
{
    /* 阻抗关系: F = M*(x_des'' - x'') + B*(x_des' - x') + K*(x_des - x)
     * 简化为: F_out = K*(x_des - x) + B*(dx_des - dx) + F_ext
     * 用欧拉法积分更新内部状态 */
    float x_err = x_des - ctrl->x;
    float dx_err = dx_des - ctrl->dx;

    /* 计算期望加速度 */
    float ddx = (F_ext - ctrl->B * ctrl->dx - ctrl->K * x_err) / ctrl->M;

    /* 更新速度和位置 (欧拉前向) */
    ctrl->dx += ddx * ctrl->dt;
    ctrl->x += ctrl->dx * ctrl->dt;

    /* 输出力 = 阻抗模型计算 */
    ctrl->F_out = ctrl->K * x_err + ctrl->B * dx_err + F_ext;

    return ctrl->F_out;
}

void Impedance_Reset(ImpedanceCtrl_t *ctrl)
{
    ctrl->x = 0.0f;
    ctrl->dx = 0.0f;
    ctrl->F_out = 0.0f;
}

/* ======================== 导纳控制实现 ======================== */

void Admittance_Init(AdmittanceCtrl_t *ctrl, float Md, float Bd, float Kd, float dt)
{
    ctrl->Md = Md;
    ctrl->Bd = Bd;
    ctrl->Kd = Kd;
    ctrl->dt = dt;
    ctrl->x_cmd = 0.0f;
    ctrl->dx_cmd = 0.0f;
    ctrl->F_err = 0.0f;
}

float Admittance_Update(AdmittanceCtrl_t *ctrl, float F_des, float F_meas)
{
    /* 导纳关系: Md*x'' + Bd*x' + Kd*x = F_des - F_meas
     * 求解位置修正 x_cmd */
    float F_err = F_des - F_meas;

    /* 加速度: a = (F_err - Bd*dx_cmd - Kd*x_cmd) / Md */
    float ddx = (F_err - ctrl->Bd * ctrl->dx_cmd - ctrl->Kd * ctrl->x_cmd) / ctrl->Md;

    /* 梯形积分 (更稳定) */
    float dx_new = ctrl->dx_cmd + ddx * ctrl->dt;
    ctrl->x_cmd += 0.5f * (ctrl->dx_cmd + dx_new) * ctrl->dt;
    ctrl->dx_cmd = dx_new;

    return ctrl->x_cmd;
}

void Admittance_Reset(AdmittanceCtrl_t *ctrl)
{
    ctrl->x_cmd = 0.0f;
    ctrl->dx_cmd = 0.0f;
    ctrl->F_err = 0.0f;
}

/* ======================== 力/位混合控制实现 ======================== */

void Hybrid_Init(HybridForcePosCtrl_t *ctrl, float Kp_pos, float Kp_force,
                 float Ki_force, float dt, float max_force)
{
    ctrl->Kp_pos = Kp_pos;
    ctrl->Kp_force = Kp_force;
    ctrl->Ki_force = Ki_force;
    ctrl->dt = dt;
    ctrl->max_force = max_force;
    ctrl->force_integral = 0.0f;
    for (int i = 0; i < 6; i++) {
        ctrl->S.s[i] = 0.0f;
    }
}

void Hybrid_SetSelectionAxis(HybridForcePosCtrl_t *ctrl, uint8_t dim, float s_val)
{
    if (dim < 6) {
        ctrl->S.s[dim] = (s_val != 0.0f) ? 1.0f : 0.0f;
    }
}

float Hybrid_UpdateSingleAxis(HybridForcePosCtrl_t *ctrl, float pos_des,
                               float pos_meas, float force_des, float force_meas)
{
    /* s=1: 力控模式; s=0: 位控模式 */
    /* 第0维简化处理 */
    float s = ctrl->S.s[0];

    /* 位置控制部分 */
    float pos_err = pos_des - pos_meas;
    float u_pos = ctrl->Kp_pos * pos_err;

    /* 力控制部分 (PI) */
    float force_err = force_des - force_meas;
    ctrl->force_integral += force_err * ctrl->dt;
    float u_force = ctrl->Kp_force * force_err + ctrl->Ki_force * ctrl->force_integral;
    u_force = clampf(u_force, -ctrl->max_force, ctrl->max_force);

    /* 混合: output = (1-s)*u_pos + s*u_force */
    float output = (1.0f - s) * u_pos + s * u_force;

    return output;
}
