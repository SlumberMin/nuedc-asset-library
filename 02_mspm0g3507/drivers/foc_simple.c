/**
 * @file    foc_simple.c
 * @brief   简化FOC算法实现
 * @note    基于机器人竞赛优秀方案，适配MSPM0G3507
 */

#include "foc_simple.h"

#ifndef M_PI
#define M_PI 3.14159265358979f
#endif

#define SQRT3          1.73205080756888f
#define SQRT3_OVER_2   0.86602540378444f
#define ONE_OVER_SQRT3 0.57735026918963f

/* ---------- 初始化 ---------- */

void FOC_Init(FOC_HandleTypeDef *hfoc)
{
    memset(hfoc, 0, sizeof(FOC_HandleTypeDef));

    hfoc->pi_d.kp = 0.5f;
    hfoc->pi_d.ki = 0.01f;
    hfoc->pi_d.output_limit = 12.0f;

    hfoc->pi_q.kp = 0.5f;
    hfoc->pi_q.ki = 0.01f;
    hfoc->pi_q.output_limit = 12.0f;

    hfoc->pi_speed.kp = 0.1f;
    hfoc->pi_speed.ki = 0.001f;
    hfoc->pi_speed.output_limit = 5.0f;

    hfoc->v_bus = FOC_VBUS_DEFAULT;
}

/* ---------- Clark变换 ---------- */

void FOC_ClarkeTransform(float i_a, float i_b, float i_c,
                         float *i_alpha, float *i_beta)
{
    *i_alpha = (2.0f / 3.0f) * (i_a - 0.5f * i_b - 0.5f * i_c);
    *i_beta  = (2.0f / 3.0f) * (SQRT3_OVER_2 * i_b - SQRT3_OVER_2 * i_c);
}

/* ---------- Park变换 ---------- */

void FOC_ParkTransform(float i_alpha, float i_beta, float angle,
                       float *i_d, float *i_q)
{
    float ca = cosf(angle);
    float sa = sinf(angle);
    *i_d =  i_alpha * ca + i_beta * sa;
    *i_q = -i_alpha * sa + i_beta * ca;
}

/* ---------- 反Park变换 ---------- */

void FOC_InverseParkTransform(float v_d, float v_q, float angle,
                              float *v_alpha, float *v_beta)
{
    float ca = cosf(angle);
    float sa = sinf(angle);
    *v_alpha = v_d * ca - v_q * sa;
    *v_beta  = v_d * sa + v_q * ca;
}

/* ---------- PI控制器 ---------- */

float FOC_PIController(float ref, float meas, float kp, float ki,
                       float limit, float *integral)
{
    float error = ref - meas;
    float p_term = kp * error;

    *integral += ki * error;
    if (*integral >  limit) *integral =  limit;
    if (*integral < -limit) *integral = -limit;

    float output = p_term + *integral;
    if (output >  limit) output =  limit;
    if (output < -limit) output = -limit;

    return output;
}

/* ---------- SVPWM ---------- */

void FOC_SVPWM(float v_alpha, float v_beta, float v_bus,
               float *duty_a, float *duty_b, float *duty_c)
{
    /* BugFix: v_bus为0时导致除零 (v_a/v_bus)
     * v_bus过小时也可能导致占空比计算异常 */
    if (v_bus < 0.1f) {
        *duty_a = 0.5f;
        *duty_b = 0.5f;
        *duty_c = 0.5f;
        return;
    }

    /* 限幅：电压矢量不超过六边形内切圆 */
    float v_max = v_bus * ONE_OVER_SQRT3;
    float v_mag = sqrtf(v_alpha * v_alpha + v_beta * v_beta);
    if (v_mag > v_max) {
        v_alpha = v_alpha * v_max / v_mag;
        v_beta  = v_beta  * v_max / v_mag;
    }

    /* 反Clark：α-β → 三相电压 */
    float v_a = v_alpha;
    float v_b = -0.5f * v_alpha + SQRT3_OVER_2 * v_beta;
    float v_c = -0.5f * v_alpha - SQRT3_OVER_2 * v_beta;

    /* 占空比 = V/Vbus + 0.5（中心对齐） */
    *duty_a = v_a / v_bus + 0.5f;
    *duty_b = v_b / v_bus + 0.5f;
    *duty_c = v_c / v_bus + 0.5f;

    /* 限幅 */
    if (*duty_a > FOC_MAX_DUTY) *duty_a = FOC_MAX_DUTY;
    if (*duty_a < 0.0f)         *duty_a = 0.0f;
    if (*duty_b > FOC_MAX_DUTY) *duty_b = FOC_MAX_DUTY;
    if (*duty_b < 0.0f)         *duty_b = 0.0f;
    if (*duty_c > FOC_MAX_DUTY) *duty_c = FOC_MAX_DUTY;
    if (*duty_c < 0.0f)         *duty_c = 0.0f;
}

/* ---------- FOC主控制循环 ---------- */

void FOC_Update(FOC_HandleTypeDef *hfoc,
                float i_a, float i_b, float i_c, float angle)
{
    hfoc->i_a   = i_a;
    hfoc->i_b   = i_b;
    hfoc->i_c   = i_c;
    hfoc->angle = angle;

    /* Clark */
    FOC_ClarkeTransform(i_a, i_b, i_c, &hfoc->i_alpha, &hfoc->i_beta);

    /* Park */
    FOC_ParkTransform(hfoc->i_alpha, hfoc->i_beta, angle,
                      &hfoc->i_d, &hfoc->i_q);

    /* 速度环（外环）→ q轴电流参考 */
    hfoc->i_q_ref = FOC_PIController(hfoc->speed_ref, hfoc->speed,
                                     hfoc->pi_speed.kp, hfoc->pi_speed.ki,
                                     hfoc->pi_speed.output_limit,
                                     &hfoc->pi_speed.integral);

    /* d轴电流环（内环） */
    hfoc->v_d = FOC_PIController(hfoc->i_d_ref, hfoc->i_d,
                                 hfoc->pi_d.kp, hfoc->pi_d.ki,
                                 hfoc->pi_d.output_limit,
                                 &hfoc->pi_d.integral);

    /* q轴电流环（内环） */
    hfoc->v_q = FOC_PIController(hfoc->i_q_ref, hfoc->i_q,
                                 hfoc->pi_q.kp, hfoc->pi_q.ki,
                                 hfoc->pi_q.output_limit,
                                 &hfoc->pi_q.integral);

    /* 反Park */
    FOC_InverseParkTransform(hfoc->v_d, hfoc->v_q, angle,
                             &hfoc->v_alpha, &hfoc->v_beta);

    /* SVPWM */
    FOC_SVPWM(hfoc->v_alpha, hfoc->v_beta, hfoc->v_bus,
              &hfoc->duty_a, &hfoc->duty_b, &hfoc->duty_c);
}

/* ---------- 辅助函数 ---------- */

void FOC_SetPI(FOC_HandleTypeDef *hfoc, float d_kp, float d_ki,
               float q_kp, float q_ki, float spd_kp, float spd_ki)
{
    hfoc->pi_d.kp     = d_kp;
    hfoc->pi_d.ki     = d_ki;
    hfoc->pi_q.kp     = q_kp;
    hfoc->pi_q.ki     = q_ki;
    hfoc->pi_speed.kp = spd_kp;
    hfoc->pi_speed.ki = spd_ki;
}

void FOC_SetSpeedRef(FOC_HandleTypeDef *hfoc, float rpm)
{
    hfoc->speed_ref = rpm;
}
