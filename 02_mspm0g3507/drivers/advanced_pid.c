/**
 * @file    advanced_pid.c
 * @brief   高级PID控制器 — ADRC / LQR / SMC 实现
 *
 * 基于电赛资产库算法移植，适配MSPM0G3507 (Cortex-M0+, 无FPU硬加速)
 * 使用math.h的浮点运算，编译时需链接 -lm
 */

#include "advanced_pid.h"
#include <math.h>
#include <string.h>

/* ═══════════════════════════════════════════════════════════════
 *  ADRC — 自抗扰控制实现
 *
 *  原理：TD安排过渡过程 + ESO估计总扰动 + NLSEF非线性反馈补偿
 *  参考：韩京清《自抗扰控制技术》
 * ═══════════════════════════════════════════════════════════════ */

/**
 * @brief fal函数 — 非线性映射，ADRC核心
 * @param e      误差输入
 * @param alpha  幂次指数 (0<alpha<1)
 * @param delta  线性区间阈值
 */
static float fal_func(float e, float alpha, float delta)
{
    float ae = fabsf(e);
    if (ae > delta) {
        /* 幂函数区 */
        float sign = (e > 0.0f) ? 1.0f : -1.0f;
        float val = 1.0f;
        /* 使用powf计算 |e|^alpha */
        if (ae > 0.0001f) {
            val = powf(ae, alpha);
        }
        return val * sign;
    } else {
        /* 线性区，避免除零 */
        float d = (delta > 0.0001f) ? delta : 0.0001f;
        return e / powf(d, 1.0f - alpha);
    }
}

/**
 * @brief fhan函数 — 最速综合函数（离散最速跟踪微分器）
 */
static float fhan_func(float x1, float x2, float r, float h)
{
    /* BugFix: h=0时d=0, d0=0, y=x1, a0=sqrt(0)=0, 分支y/h除零
     * 添加下限保护 */
    if (h < 1e-6f) h = 1e-6f;
    float d  = r * h;
    float d0 = d * h;
    float y  = x1 + h * x2;
    float a0 = sqrtf(d * d + 8.0f * r * fabsf(y));
    float a;

    if (fabsf(y) > d0) {
        a = x2 + (a0 - d) * 0.5f * ((y > 0.0f) ? 1.0f : -1.0f);
    } else {
        a = x2 + y / h;
    }

    if (fabsf(a) > d) {
        return -r * ((a > 0.0f) ? 1.0f : -1.0f);
    } else {
        return -r * a / d;
    }
}

void ADRC_Init(ADRC_t *ctrl, float r0, float h0, float b0,
               float omega_c, float omega_o, float delta, float dt)
{
    /* TD参数 */
    ctrl->r0 = r0;
    ctrl->h0 = h0;

    /* ESO参数 — 带宽法整定 */
    ctrl->beta01 = 3.0f * omega_o;
    ctrl->beta02 = 3.0f * omega_o * omega_o;
    ctrl->beta03 = omega_o * omega_o * omega_o;

    /* 控制器参数 */
    ctrl->b0    = b0;
    ctrl->delta = delta;
    ctrl->dt    = dt;

    /* NLSEF增益 */
    ctrl->kp = omega_c * omega_c;
    ctrl->kd = 2.0f * omega_c;

    /* 状态清零 */
    ctrl->v1 = 0.0f;  ctrl->v2 = 0.0f;
    ctrl->z1 = 0.0f;  ctrl->z2 = 0.0f;  ctrl->z3 = 0.0f;
    ctrl->u  = 0.0f;
    ctrl->u_max = 1000.0f;
}

float ADRC_Update(ADRC_t *ctrl, float ref, float y)
{
    float dt = ctrl->dt;

    /* ── 1. 跟踪微分器 TD ────────────────────── */
    float fh = fhan_func(ctrl->v1 - ref, ctrl->v2, ctrl->r0, ctrl->h0);
    ctrl->v1 += ctrl->h0 * ctrl->v2;
    ctrl->v2 += ctrl->h0 * fh;

    /* ── 2. 扩张状态观测器 ESO ────────────────── */
    {
        float e_obs = ctrl->z1 - y;
        float fe1 = fal_func(e_obs, 0.5f, ctrl->delta);
        float fe2 = fal_func(e_obs, 0.25f, ctrl->delta);

        ctrl->z1 += dt * (ctrl->z2 - ctrl->beta01 * e_obs);
        ctrl->z2 += dt * (ctrl->z3 - ctrl->beta02 * fe1 + ctrl->b0 * ctrl->u);
        ctrl->z3 += dt * (-ctrl->beta03 * fe2);
    }

    /* ── 3. 误差计算 ─────────────────────────── */
    float e1 = ctrl->v1 - ctrl->z1;   /* 跟踪误差 */
    float e2 = ctrl->v2 - ctrl->z2;   /* 微分误差 */

    /* ── 4. 非线性状态误差反馈 NLSEF ─────────── */
    float u0 = ctrl->kp * fal_func(e1, 0.5f, ctrl->delta)
             + ctrl->kd * fal_func(e2, 0.25f, ctrl->delta);

    /* ── 5. 扰动补偿 ────────────────────────── */
    /* BugFix: b0为0时导致除零，添加下限保护 */
    float b0 = ctrl->b0;
    if (fabsf(b0) < 1e-6f) b0 = (b0 >= 0.0f) ? 1e-6f : -1e-6f;
    float u = (u0 - ctrl->z3) / b0;

    /* 输出限幅 */
    if (u >  ctrl->u_max) u =  ctrl->u_max;
    if (u < -ctrl->u_max) u = -ctrl->u_max;

    ctrl->u = u;
    return u;
}

void ADRC_Reset(ADRC_t *ctrl)
{
    ctrl->v1 = 0.0f;  ctrl->v2 = 0.0f;
    ctrl->z1 = 0.0f;  ctrl->z2 = 0.0f;  ctrl->z3 = 0.0f;
    ctrl->u  = 0.0f;
}

void ADRC_SetOutputLimit(ADRC_t *ctrl, float max)
{
    ctrl->u_max = max;
}

void ADRC_SetOmegaC(ADRC_t *ctrl, float omega_c)
{
    ctrl->kp = omega_c * omega_c;
    ctrl->kd = 2.0f * omega_c;
}

void ADRC_SetOmegaO(ADRC_t *ctrl, float omega_o)
{
    ctrl->beta01 = 3.0f * omega_o;
    ctrl->beta02 = 3.0f * omega_o * omega_o;
    ctrl->beta03 = omega_o * omega_o * omega_o;
}

void ADRC_SetB0(ADRC_t *ctrl, float b0)
{
    ctrl->b0 = b0;
}

/* ═══════════════════════════════════════════════════════════════
 *  LQR — 线性二次调节器实现 (2阶简化版)
 *
 *  原理：最小化 J = Σ[x^T*Q*x + u^T*R*u]
 *  最优控制律 u = -K*x
 *  使用迭代法求解离散代数Riccati方程(DARE)
 * ═══════════════════════════════════════════════════════════════ */

void LQR_Init(LQR_t *ctrl, float dt)
{
    memset(ctrl, 0, sizeof(LQR_t));
    ctrl->dt    = dt;
    ctrl->u_max = 1000.0f;
    /* 默认单位阵 */
    ctrl->Q[0][0] = 1.0f;
    ctrl->Q[1][1] = 1.0f;
    ctrl->R = 1.0f;
}

void LQR_SetSystem(LQR_t *ctrl,
                   float a11, float a12, float a21, float a22,
                   float b1, float b2)
{
    ctrl->A[0][0] = a11;  ctrl->A[0][1] = a12;
    ctrl->A[1][0] = a21;  ctrl->A[1][1] = a22;
    ctrl->B[0] = b1;
    ctrl->B[1] = b2;
}

void LQR_SetWeight(LQR_t *ctrl, float q1, float q2, float r)
{
    ctrl->Q[0][0] = q1;
    ctrl->Q[0][1] = 0.0f;
    ctrl->Q[1][0] = 0.0f;
    ctrl->Q[1][1] = q2;
    ctrl->R = r;
}

/**
 * @brief 迭代求解2阶DARE并计算增益K
 *        P = A^T*P*A - A^T*P*B*(R+B^T*P*B)^{-1}*B^T*P*A + Q
 *        K = (R+B^T*P*B)^{-1} * B^T*P*A
 */
int LQR_SolveRiccati(LQR_t *ctrl, int max_iter)
{
    float (*A)[2] = ctrl->A;
    float *B = ctrl->B;
    float (*Q)[2] = ctrl->Q;
    float R = ctrl->R;
    float P[2][2], Pn[2][2];

    /* 初始化 P = Q */
    P[0][0] = Q[0][0];  P[0][1] = Q[0][1];
    P[1][0] = Q[1][0];  P[1][1] = Q[1][1];

    for (int iter = 0; iter < max_iter; iter++) {
        /* BTP = B^T*P */
        float BTP[2];
        BTP[0] = B[0]*P[0][0] + B[1]*P[1][0];
        BTP[1] = B[0]*P[0][1] + B[1]*P[1][1];

        /* ATPA = A^T*P*A */
        float tmp[2][2];
        tmp[0][0] = P[0][0]*A[0][0] + P[0][1]*A[1][0];
        tmp[0][1] = P[0][0]*A[0][1] + P[0][1]*A[1][1];
        tmp[1][0] = P[1][0]*A[0][0] + P[1][1]*A[1][0];
        tmp[1][1] = P[1][0]*A[0][1] + P[1][1]*A[1][1];

        float ATPA[2][2];
        ATPA[0][0] = A[0][0]*tmp[0][0] + A[1][0]*tmp[1][0];
        ATPA[0][1] = A[0][0]*tmp[0][1] + A[1][0]*tmp[1][1];
        ATPA[1][0] = A[0][1]*tmp[0][0] + A[1][1]*tmp[1][0];
        ATPA[1][1] = A[0][1]*tmp[0][1] + A[1][1]*tmp[1][1];

        /* BTPB = B^T*P*B + R */
        float BTPB = BTP[0]*B[0] + BTP[1]*B[1] + R;
        /* BugFix: BTPB可能为0（当B全零且R=0时），导致除零
         * 添加下限保护 */
        if (fabsf(BTPB) < 1e-10f) BTPB = (BTPB >= 0.0f) ? 1e-10f : -1e-10f;
        float Sinv = 1.0f / BTPB;

        /* ATPB = A^T*P*B */
        float ATPB[2];
        float PB0 = P[0][0]*B[0] + P[0][1]*B[1];
        float PB1 = P[1][0]*B[0] + P[1][1]*B[1];
        ATPB[0] = A[0][0]*PB0 + A[1][0]*PB1;
        ATPB[1] = A[0][1]*PB0 + A[1][1]*PB1;

        /* BTPA = B^T*P*A */
        float BTPA[2];
        BTPA[0] = BTP[0]*A[0][0] + BTP[1]*A[1][0];
        BTPA[1] = BTP[0]*A[0][1] + BTP[1]*A[1][1];

        /* corr = ATPB * Sinv * BTPA */
        float corr[2][2];
        corr[0][0] = ATPB[0] * Sinv * BTPA[0];
        corr[0][1] = ATPB[0] * Sinv * BTPA[1];
        corr[1][0] = ATPB[1] * Sinv * BTPA[0];
        corr[1][1] = ATPB[1] * Sinv * BTPA[1];

        /* Pn = ATPA - corr + Q */
        Pn[0][0] = ATPA[0][0] - corr[0][0] + Q[0][0];
        Pn[0][1] = ATPA[0][1] - corr[0][1] + Q[0][1];
        Pn[1][0] = ATPA[1][0] - corr[1][0] + Q[1][0];
        Pn[1][1] = ATPA[1][1] - corr[1][1] + Q[1][1];

        /* 收敛判断 */
        float diff = fabsf(Pn[0][0]-P[0][0]) + fabsf(Pn[0][1]-P[0][1])
                   + fabsf(Pn[1][0]-P[1][0]) + fabsf(Pn[1][1]-P[1][1]);

        P[0][0] = Pn[0][0]; P[0][1] = Pn[0][1];
        P[1][0] = Pn[1][0]; P[1][1] = Pn[1][1];

        if (diff < 1e-6f) break;
    }

    /* 计算最优增益 K = Sinv * B^T*P*A */
    {
        float BTP[2];
        BTP[0] = B[0]*P[0][0] + B[1]*P[1][0];
        BTP[1] = B[0]*P[0][1] + B[1]*P[1][1];

        float BTPB = BTP[0]*B[0] + BTP[1]*B[1] + R;
        if (fabsf(BTPB) < 1e-10f) BTPB = (BTPB >= 0.0f) ? 1e-10f : -1e-10f;
        float Sinv = 1.0f / BTPB;

        float BTPA[2];
        BTPA[0] = BTP[0]*A[0][0] + BTP[1]*A[1][0];
        BTPA[1] = BTP[0]*A[0][1] + BTP[1]*A[1][1];

        ctrl->K[0] = Sinv * BTPA[0];
        ctrl->K[1] = Sinv * BTPA[1];
    }

    ctrl->P[0][0] = P[0][0]; ctrl->P[0][1] = P[0][1];
    ctrl->P[1][0] = P[1][0]; ctrl->P[1][1] = P[1][1];
    return 1;
}

float LQR_Update(LQR_t *ctrl, float x1, float x2)
{
    /* u = -K*x */
    float u = -(ctrl->K[0] * x1 + ctrl->K[1] * x2);

    /* 输出限幅 */
    if (u >  ctrl->u_max) u =  ctrl->u_max;
    if (u < -ctrl->u_max) u = -ctrl->u_max;

    return u;
}

void LQR_SetOutputLimit(LQR_t *ctrl, float max)
{
    ctrl->u_max = max;
}

void LQR_Reset(LQR_t *ctrl)
{
    /* 增益矩阵不变，无需重置 */
    (void)ctrl;
}

void LQR_SetWeightOnline(LQR_t *ctrl, float q1, float q2, float r)
{
    ctrl->Q[0][0] = q1;
    ctrl->Q[1][1] = q2;
    ctrl->R = r;
    /* 重新求解Riccati方程 */
    LQR_SolveRiccati(ctrl, 50);
}

/* ═══════════════════════════════════════════════════════════════
 *  SMC — 滑模控制实现 (指数趋近律)
 *
 *  原理：设计滑模面 s = e_dot + c*e
 *  控制律：u = u_eq + u_sw
 *  u_sw = eps * sat(s/phi) + k * s  (指数趋近律)
 *  使用边界层法减小抖振
 * ═══════════════════════════════════════════════════════════════ */

/**
 * @brief sat函数 — 带边界层的符号函数
 *        代替sign(s)以减小抖振
 */
static float sat_func(float s, float phi)
{
    if (phi <= 0.0f) {
        return (s > 0.0f) ? 1.0f : ((s < 0.0f) ? -1.0f : 0.0f);
    }
    if (s >  phi) return  1.0f;
    if (s < -phi) return -1.0f;
    return s / phi;
}

void SMC_Init(SMC_t *ctrl, float c, float eps, float k, float phi, float dt)
{
    ctrl->c     = c;
    ctrl->eps   = eps;
    ctrl->k     = k;
    ctrl->phi   = phi;
    ctrl->dt    = dt;
    ctrl->u_max = 1000.0f;
}

float SMC_Update(SMC_t *ctrl, float e, float e_dot, float u_eq)
{
    /* 滑模面 s = e_dot + c * e */
    float s = e_dot + ctrl->c * e;

    /* 指数趋近律: u_sw = eps * sat(s/phi) + k * s */
    float u_sw = ctrl->eps * sat_func(s, ctrl->phi) + ctrl->k * s;

    float u = u_eq + u_sw;

    /* 输出限幅 */
    if (u >  ctrl->u_max) u =  ctrl->u_max;
    if (u < -ctrl->u_max) u = -ctrl->u_max;

    return u;
}

void SMC_Reset(SMC_t *ctrl)
{
    (void)ctrl;
}

void SMC_SetOutputLimit(SMC_t *ctrl, float max)
{
    ctrl->u_max = max;
}

void SMC_SetC(SMC_t *ctrl, float c)
{
    ctrl->c = c;
}

void SMC_SetEps(SMC_t *ctrl, float eps)
{
    ctrl->eps = eps;
}

void SMC_SetK(SMC_t *ctrl, float k)
{
    ctrl->k = k;
}

/* ═══════════════════════════════════════════════════════════════
 *  统一接口 AdvCtrl_t
 * ═══════════════════════════════════════════════════════════════ */

void AdvCtrl_Init(AdvCtrl_t *ctrl, CtrlAlgoType type, float dt, const float *p)
{
    ctrl->type = type;

    switch (type) {
    case CTRL_ALGO_ADRC:
        /* p = [r0, h0, b0, omega_c, omega_o, delta] */
        ADRC_Init(&ctrl->algo.adrc, p[0], p[1], p[2], p[3], p[4], p[5], dt);
        break;

    case CTRL_ALGO_LQR:
        /* p = [a11, a12, a21, a22, b1, b2, q1, q2, r] */
        LQR_Init(&ctrl->algo.lqr, dt);
        LQR_SetSystem(&ctrl->algo.lqr, p[0], p[1], p[2], p[3], p[4], p[5]);
        LQR_SetWeight(&ctrl->algo.lqr, p[6], p[7], p[8]);
        LQR_SolveRiccati(&ctrl->algo.lqr, 100);
        break;

    case CTRL_ALGO_SMC:
        /* p = [c, eps, k, phi] */
        SMC_Init(&ctrl->algo.smc, p[0], p[1], p[2], p[3], dt);
        break;

    default:
        break;
    }
}

float AdvCtrl_Update(AdvCtrl_t *ctrl, float ref, float y, float aux)
{
    switch (ctrl->type) {
    case CTRL_ALGO_ADRC:
        return ADRC_Update(&ctrl->algo.adrc, ref, y);
    case CTRL_ALGO_LQR:
        return LQR_Update(&ctrl->algo.lqr, ref, y);
    case CTRL_ALGO_SMC:
        return SMC_Update(&ctrl->algo.smc, ref, y, aux);
    default:
        return 0.0f;
    }
}

void AdvCtrl_Reset(AdvCtrl_t *ctrl)
{
    switch (ctrl->type) {
    case CTRL_ALGO_ADRC: ADRC_Reset(&ctrl->algo.adrc); break;
    case CTRL_ALGO_LQR:  LQR_Reset(&ctrl->algo.lqr);   break;
    case CTRL_ALGO_SMC:  SMC_Reset(&ctrl->algo.smc);    break;
    default: break;
    }
}

void AdvCtrl_SetOutputLimit(AdvCtrl_t *ctrl, float max)
{
    switch (ctrl->type) {
    case CTRL_ALGO_ADRC: ADRC_SetOutputLimit(&ctrl->algo.adrc, max); break;
    case CTRL_ALGO_LQR:  LQR_SetOutputLimit(&ctrl->algo.lqr, max);  break;
    case CTRL_ALGO_SMC:  SMC_SetOutputLimit(&ctrl->algo.smc, max);  break;
    default: break;
    }
}
