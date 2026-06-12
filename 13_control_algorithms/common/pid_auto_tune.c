/**
 * @file pid_auto_tune.c
 * @brief PID参数自动整定实现 - 继电反馈法 + ZN阶跃响应法
 */
#include "pid_auto_tune.h"
#include <math.h>
#include <string.h>

#define CLAMP(val, lo, hi) ((val) < (lo) ? (lo) : ((val) > (hi) ? (hi) : (val)))

/* ================================================================
 *  继电反馈法 (Relay Feedback / Astrom-Hagglund)
 * ================================================================ */

void RelayAutoTune_Init(RelayAutoTune_t *at, float relay_amp, float hysteresis,
                        float offset, float dt, float timeout, AutoTuneRule_e rule)
{
    memset(at, 0, sizeof(RelayAutoTune_t));
    at->relay_amplitude  = fabsf(relay_amp);
    at->relay_hysteresis = fabsf(hysteresis);
    at->output_offset    = offset;
    at->dt               = dt;
    at->timeout          = timeout;
    at->rule             = rule;
    at->state            = AT_STATE_RELAY_WAIT;
    at->relay_output     = offset + relay_amp;
    at->peak_max         = -1e30f;
    at->peak_min         =  1e30f;
}

float RelayAutoTune_Step(RelayAutoTune_t *at, float measurement)
{
    if (at->state == AT_STATE_DONE || at->state == AT_STATE_ERROR) {
        return at->output_offset;
    }

    at->timer += at->dt;

    /* 超时检查 */
    if (at->timer > at->timeout) {
        at->state = AT_STATE_ERROR;
        return at->output_offset;
    }

    /* 继电滞环逻辑 */
    if (measurement > at->relay_hysteresis) {
        at->relay_output = at->output_offset - at->relay_amplitude;
    } else if (measurement < -at->relay_hysteresis) {
        at->relay_output = at->output_offset + at->relay_amplitude;
    }

    /* 检测过零点(上升沿)来计算周期和振幅 */
    if (at->prev_input < 0.0f && measurement >= 0.0f) {
        /* 过零 */
        if (at->rising_edge && at->zero_cross_time > 0.0f) {
            float period = at->timer - at->zero_cross_time;
            float amplitude = (at->peak_max - at->peak_min) / 2.0f;

            if (period > 0.0f && amplitude > 1e-6f) {
                at->period_sum  += period;
                at->amplitude_sum += amplitude;
                at->period_count++;
            }
        }
        at->zero_cross_time = at->timer;
        at->rising_edge = 1;
        at->peak_max = measurement;
        at->peak_min = measurement;
    } else {
        if (measurement > at->peak_max) at->peak_max = measurement;
        if (measurement < at->peak_min) at->peak_min = measurement;
    }

    at->prev_input = measurement;

    /* 至少测量4个完整周期 */
    if (at->period_count >= 4) {
        float Tu = at->period_sum / at->period_count;
        float a  = at->amplitude_sum / at->period_count;
        float d  = at->relay_amplitude;

        /* 临界增益: Ku = 4*d / (pi*a) */
        float Ku = (4.0f * d) / (3.14159265f * a);

        at->result.Ku = Ku;
        at->result.Tu = Tu;

        /* 根据规则计算PID参数 */
        AutoTune_ComputeZN(Ku, Tu, at->rule, &at->result);

        at->state = AT_STATE_DONE;
    }

    return at->relay_output;
}

uint8_t RelayAutoTune_IsDone(RelayAutoTune_t *at)
{
    return (at->state == AT_STATE_DONE) ? 1 : 0;
}

const AutoTuneResult_t* RelayAutoTune_GetResult(RelayAutoTune_t *at)
{
    return &at->result;
}

void RelayAutoTune_Reset(RelayAutoTune_t *at)
{
    AutoTuneRule_e rule = at->rule;
    float amp = at->relay_amplitude;
    float hyst = at->relay_hysteresis;
    float off = at->output_offset;
    float dt = at->dt;
    float to = at->timeout;
    RelayAutoTune_Init(at, amp, hyst, off, dt, to, rule);
}

/* ================================================================
 *  阶跃响应法 (Ziegler-Nichols Step Response / Cohen-Coon)
 * ================================================================ */

void StepAutoTune_Init(StepAutoTune_t *at, float step_amp, float dt,
                       float timeout, AutoTuneMethod_e method, AutoTuneRule_e rule)
{
    memset(at, 0, sizeof(StepAutoTune_t));
    at->step_amplitude = step_amp;
    at->dt             = dt;
    at->timeout        = timeout;
    at->method         = method;
    at->rule           = rule;
    at->state          = AT_STATE_STEP_WAIT;
    at->baseline       = 0.0f;
    at->found_283      = 0;
    at->found_632      = 0;
}

float StepAutoTune_Step(StepAutoTune_t *at, float measurement)
{
    if (at->state == AT_STATE_DONE || at->state == AT_STATE_ERROR) {
        return at->baseline;
    }

    at->timer += at->dt;

    if (at->timer > at->timeout) {
        at->state = AT_STATE_ERROR;
        return at->baseline;
    }

    float output;

    switch (at->state) {
    case AT_STATE_STEP_WAIT:
        /* 记录基线后施加阶跃 */
        at->baseline = measurement;
        at->start_time = at->timer;
        at->threshold_283 = at->baseline + 0.283f * at->step_amplitude;
        at->threshold_632 = at->baseline + 0.632f * at->step_amplitude;
        at->state = AT_STATE_STEP_MEASURE;
        output = at->baseline + at->step_amplitude;
        break;

    case AT_STATE_STEP_MEASURE:
        output = at->baseline + at->step_amplitude;

        /* 检测28.3%和63.2%响应时间 */
        if (!at->found_283 && measurement >= at->threshold_283) {
            at->t283 = at->timer - at->start_time;
            at->found_283 = 1;
        }
        if (!at->found_632 && measurement >= at->threshold_632) {
            at->t632 = at->timer - at->start_time;
            at->found_632 = 1;
        }

        /* 检测稳态 (变化率 < 1%) */
        if (at->found_632) {
            float y_norm = (measurement - at->baseline) / at->step_amplitude;
            if (y_norm >= 0.99f) {
                at->steady_state = measurement;
                float K_proc = (at->steady_state - at->baseline) / at->step_amplitude;

                /* 使用Smith切线法估算 L 和 T */
                float T_est = 1.5f * (at->t632 - at->t283);
                float L_est = at->t632 - T_est;
                if (L_est < 0.0f) L_est = 0.0f;

                at->result.K_proc = K_proc;
                at->result.L = L_est;
                at->result.T = T_est;

                if (at->method == AT_METHOD_COHEN_COON) {
                    AutoTune_ComputeCohenCoon(K_proc, L_est, T_est, &at->result);
                } else {
                    /* ZN阶跃响应法 */
                    if (L_est > 1e-10f && K_proc > 1e-10f) {
                        float Ku = T_est / (K_proc * L_est);
                        float Tu = 3.33f * L_est;  /* 近似临界周期 */
                        at->result.Ku = Ku;
                        at->result.Tu = Tu;
                        AutoTune_ComputeZN(Ku, Tu, at->rule, &at->result);
                    }
                }

                at->state = AT_STATE_DONE;
            }
        }
        break;

    default:
        output = at->baseline;
        break;
    }

    return output;
}

uint8_t StepAutoTune_IsDone(StepAutoTune_t *at)
{
    return (at->state == AT_STATE_DONE) ? 1 : 0;
}

const AutoTuneResult_t* StepAutoTune_GetResult(StepAutoTune_t *at)
{
    return &at->result;
}

void StepAutoTune_Reset(StepAutoTune_t *at)
{
    float amp = at->step_amplitude;
    float dt = at->dt;
    float to = at->timeout;
    AutoTuneMethod_e method = at->method;
    AutoTuneRule_e rule = at->rule;
    StepAutoTune_Init(at, amp, dt, to, method, rule);
}

/* ================================================================
 *  ZN规则查表计算
 *  根据 Ku, Tu 和所选规则计算 Kp, Ki, Kd
 * ================================================================ */
void AutoTune_ComputeZN(float Ku, float Tu, AutoTuneRule_e rule, AutoTuneResult_t *result)
{
    float kp, ti, td;

    switch (rule) {
    case AT_RULE_PESSEN:
        /* Pessen Integral Rule: Kp=0.7Ku, Ti=0.4Tu, Td=0.15Tu */
        kp = 0.70f * Ku;
        ti = 0.40f * Tu;
        td = 0.15f * Tu;
        break;
    case AT_RULE_SOME_OVERSHOOT:
        /* Some Overshoot: Kp=0.33Ku, Ti=0.5Tu, Td=0.33Tu */
        kp = 0.33f * Ku;
        ti = 0.50f * Tu;
        td = 0.33f * Tu;
        break;
    case AT_RULE_NO_OVERSHOOT:
        /* No Overshoot: Kp=0.2Ku, Ti=0.5Tu, Td=0.33Tu */
        kp = 0.20f * Ku;
        ti = 0.50f * Tu;
        td = 0.33f * Tu;
        break;
    case AT_RULE_CLASSIC:
    default:
        /* 经典ZN: Kp=0.6Ku, Ti=0.5Tu, Td=0.125Tu */
        kp = 0.60f * Ku;
        ti = 0.50f * Tu;
        td = 0.125f * Tu;
        break;
    }

    result->kp = kp;
    result->ki = (ti > 0.0f) ? (kp / ti) : 0.0f;
    result->kd = kp * td;
}

/* ================================================================
 *  Cohen-Coon整定公式
 * ================================================================ */
void AutoTune_ComputeCohenCoon(float K, float L, float T, AutoTuneResult_t *result)
{
    if (K <= 0.0f || L <= 0.0f || T <= 0.0f) {
        result->kp = 0.0f;
        result->ki = 0.0f;
        result->kd = 0.0f;
        return;
    }

    float tau = L / T;  /* 归一化滞后 θ/τ (dead time / time constant) */

    /* Cohen-Coon 公式 (修正版 2026-06-12):
     if (fabsf(tau) < 1e-6f) tau = 1e-6f;
     * Kp = (1/K) * (T/L + 1/3) = (1/K) * (1/tau + 1/3)
     * Ti = L * (32 + 6*tau) / (13 + 8*tau)
     * Td = 4*L / (11 + 2*tau)
     * 参考: Cohen & Coon, "Theoretical Considerations of Retarded Control" (1953)
     */
    if (fabsf(tau) < 1e-6f) tau = 1e-6f;  /* 防除零 */
    float kp = (1.0f / K) * (1.0f / tau + 1.0f / 3.0f);
    float ti = L * (32.0f + 6.0f * tau) / (13.0f + 8.0f * tau);
    float td = 4.0f * L / (11.0f + 2.0f * tau);

    result->kp = kp;
    result->ki = (ti > 0.0f) ? (kp / ti) : 0.0f;
    result->kd = kp * td;
}
