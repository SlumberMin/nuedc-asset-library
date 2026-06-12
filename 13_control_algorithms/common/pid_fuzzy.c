/**
 * @file pid_fuzzy_v13.c
 * @brief 模糊PID V13 实现
 */

#include "pid_fuzzy_v13.h"
#include <math.h>
#include <string.h>

/* 内部宏 */
#define CLAMP(val, min_val, max_val) \
    do { if ((val) < (min_val)) (val) = (min_val); \
         else if ((val) > (max_val)) (val) = (max_val); } while(0)

 /*---------------------------------------------------------------------------
  * 默认7x7模糊规则表
  *---------------------------------------------------------------------------*/

/*
 * 规则说明: 行=e(误差), 列=ec(误差变化率)
 * 等级: NB=0, NM=1, NS=2, ZO=3, PS=4, PM=5, PB=6
 *
 * delta_Kp 规则: 误差大时增大Kp, 误差小时减小Kp
 */
static const int8_t s_default_rule_Kp[FUZZY_LEVELS][FUZZY_LEVELS] = {
/*         NB   NM   NS   ZO   PS   PM   PB  */
/* NB */ {  6,   6,   5,   4,   3,   2,   1 },
/* NM */ {  6,   6,   5,   3,   2,   1,   0 },
/* NS */ {  5,   5,   4,   2,   1,   0,  -1 },
/* ZO */ {  4,   3,   2,   0,  -2,  -3,  -4 },
/* PS */ {  1,   0,  -1,  -2,  -4,  -5,  -5 },
/* PM */ {  0,  -1,  -2,  -3,  -5,  -6,  -6 },
/* PB */ { -1,  -2,  -3,  -4,  -5,  -6,  -6 }
};

/*
 * delta_Ki 规则: 误差大时减小Ki(积分分离), 误差小时增大Ki
 */
static const int8_t s_default_rule_Ki[FUZZY_LEVELS][FUZZY_LEVELS] = {
/*         NB   NM   NS   ZO   PS   PM   PB  */
/* NB */ { -6,  -6,  -5,  -4,  -3,  -2,  -1 },
/* NM */ { -6,  -5,  -4,  -3,  -2,  -1,   0 },
/* NS */ { -5,  -4,  -3,  -1,   0,   1,   2 },
/* ZO */ { -4,  -3,  -1,   0,   1,   3,   4 },
/* PS */ { -2,  -1,   0,   1,   3,   4,   5 },
/* PM */ {  0,   1,   2,   3,   4,   5,   6 },
/* PB */ {  1,   2,   3,   4,   5,   6,   6 }
};

/*
 * delta_Kd 规则: 变化率大时减小Kd防止振荡, 变化率小时增大Kd
 */
static const int8_t s_default_rule_Kd[FUZZY_LEVELS][FUZZY_LEVELS] = {
/*         NB   NM   NS   ZO   PS   PM   PB  */
/* NB */ { -6,  -5,  -4,  -3,  -2,  -1,   0 },
/* NM */ { -5,  -4,  -3,  -2,  -1,   0,   1 },
/* NS */ { -4,  -3,  -2,  -1,   0,   1,   2 },
/* ZO */ { -3,  -2,  -1,   0,   1,   2,   3 },
/* PS */ { -2,  -1,   0,   1,   2,   3,   4 },
/* PM */ { -1,   0,   1,   2,   3,   4,   5 },
/* PB */ {  0,   1,   2,   3,   4,   5,   6 }
};

 /*---------------------------------------------------------------------------
  * 内部函数
  *---------------------------------------------------------------------------*/

/**
 * @brief 三角隶属函数计算
 * @param x 输入值
 * @param mf 隶属函数参数
 * @return 隶属度 [0, 1]
 */
static float s_TriMF(float x, const TriMF_t *mf)
{
    float dist = fabsf(x - mf->center);
    if (dist >= mf->width) return 0.0f;
    return 1.0f - dist / mf->width;
}

/**
 * @brief 计算模糊输出(重心法解模糊)
 * @param fuzzified_e 误差各等级隶属度
 * @param fuzzified_ec 误差变化率各等级隶属度
 * @param rule_table 规则表
 * @return 解模糊后的输出 [-6, 6]
 */
static float s_Defuzzify(const float *fuzzified_e, const float *fuzzified_ec,
                          const FuzzyRuleTable_t rule_table)
{
    float numerator = 0.0f;
    float denominator = 0.0f;

    for (int i = 0; i < FUZZY_LEVELS; i++) {
        for (int j = 0; j < FUZZY_LEVELS; j++) {
            /* 取小运算 (AND) */
            float fire_strength = fuzzified_e[i] < fuzzified_ec[j]
                                ? fuzzified_e[i] : fuzzified_ec[j];

            if (fire_strength > 0.001f) {
                /* 规则输出值: 表中存储的是-6到6的等级, 映射到论域 */
                float output_value = (float)rule_table[i][j];
                numerator += fire_strength * output_value;
                denominator += fire_strength;
            }
        }
    }

    if (denominator < 0.001f) return 0.0f;
    return numerator / denominator;  /* 范围约 [-6, 6] */
}

/**
 * @brief 模糊推理过程
 */
static void s_FuzzyInference(PID_FuzzyV13_t *pid, float *out_dKp, float *out_dKi, float *out_dKd)
{
    /* 归一化到论域 [-6, 6] */
    float e_norm = pid->error * pid->e_quantize;
    float ec_norm = pid->error_rate * pid->ec_quantize;

    CLAMP(e_norm, -6.0f, 6.0f);
    CLAMP(ec_norm, -6.0f, 6.0f);

    /* 计算各等级隶属度 */
    float fuzzified_e[FUZZY_LEVELS];
    float fuzzified_ec[FUZZY_LEVELS];

    for (int i = 0; i < FUZZY_LEVELS; i++) {
        fuzzified_e[i] = s_TriMF(e_norm, &pid->mf[i]);
        fuzzified_ec[i] = s_TriMF(ec_norm, &pid->mf[i]);
    }

    /* 解模糊 */
    float raw_dKp = s_Defuzzify(fuzzified_e, fuzzified_ec, pid->rule_Kp);
    float raw_dKi = s_Defuzzify(fuzzified_e, fuzzified_ec, pid->rule_Ki);
    float raw_dKd = s_Defuzzify(fuzzified_e, fuzzified_ec, pid->rule_Kd);

    /* 映射到实际范围 */
    *out_dKp = raw_dKp / 6.0f * pid->delta_Kp_range;
    *out_dKi = raw_dKi / 6.0f * pid->delta_Ki_range;
    *out_dKd = raw_dKd / 6.0f * pid->delta_Kd_range;
}

 /*---------------------------------------------------------------------------
  * 公开接口实现
  *---------------------------------------------------------------------------*/

void PID_FuzzyV13_Init(PID_FuzzyV13_t *pid, float Kp, float Ki, float Kd, float dt)
{
    memset(pid, 0, sizeof(PID_FuzzyV13_t));

    pid->Kp_base = Kp;
    pid->Ki_base = Ki;
    pid->Kd_base = Kd;
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->dt = dt > 0.0f ? dt : 0.001f;

    /* 默认输出范围 */
    pid->delta_Kp_range = Kp * 0.5f;
    pid->delta_Ki_range = Ki * 0.5f;
    pid->delta_Kd_range = Kd * 0.5f;

    /* 默认量化因子 (使得满量程误差归一化到6) */
    pid->e_quantize = 0.06f;
    pid->ec_quantize = 0.06f;

    /* 输出限幅 */
    pid->output_max = 1000.0f;
    pid->output_min = -1000.0f;
    pid->integral_max = 500.0f;
    pid->integral_separate_threshold = 100.0f;
    pid->integral_enable = 1;

    /* 初始化三角隶属函数: 7个等级均匀分布在[-6, 6] */
    for (int i = 0; i < FUZZY_LEVELS; i++) {
        pid->mf[i].center = -6.0f + (float)i * 2.0f;  /* -6, -4, -2, 0, 2, 4, 6 */
        pid->mf[i].width = 2.5f;  /* 略大于间距以保证覆盖 */
    }

    /* 加载默认规则表 */
    memcpy(pid->rule_Kp, s_default_rule_Kp, sizeof(FuzzyRuleTable_t));
    memcpy(pid->rule_Ki, s_default_rule_Ki, sizeof(FuzzyRuleTable_t));
    memcpy(pid->rule_Kd, s_default_rule_Kd, sizeof(FuzzyRuleTable_t));
}

void PID_FuzzyV13_SetDeltaRange(PID_FuzzyV13_t *pid, float dKp, float dKi, float dKd)
{
    if (pid) {
        pid->delta_Kp_range = dKp;
        pid->delta_Ki_range = dKi;
        pid->delta_Kd_range = dKd;
    }
}

void PID_FuzzyV13_SetQuantizeFactor(PID_FuzzyV13_t *pid, float e_factor, float ec_factor)
{
    if (pid) {
        pid->e_quantize = e_factor;
        pid->ec_quantize = ec_factor;
    }
}

void PID_FuzzyV13_SetOutputLimit(PID_FuzzyV13_t *pid, float min, float max)
{
    if (pid) {
        pid->output_min = min;
        pid->output_max = max;
    }
}

void PID_FuzzyV13_SetIntegralLimit(PID_FuzzyV13_t *pid, float max)
{
    if (pid) {
        pid->integral_max = max > 0.0f ? max : 0.0f;
    }
}

void PID_FuzzyV13_SetIntegralSeparate(PID_FuzzyV13_t *pid, float threshold)
{
    if (pid) {
        pid->integral_separate_threshold = threshold > 0.0f ? threshold : 0.0f;
    }
}

void PID_FuzzyV13_SetRuleKp(PID_FuzzyV13_t *pid, const FuzzyRuleTable_t table)
{
    if (pid && table) {
        memcpy(pid->rule_Kp, table, sizeof(FuzzyRuleTable_t));
    }
}

void PID_FuzzyV13_SetRuleKi(PID_FuzzyV13_t *pid, const FuzzyRuleTable_t table)
{
    if (pid && table) {
        memcpy(pid->rule_Ki, table, sizeof(FuzzyRuleTable_t));
    }
}

void PID_FuzzyV13_SetRuleKd(PID_FuzzyV13_t *pid, const FuzzyRuleTable_t table)
{
    if (pid && table) {
        memcpy(pid->rule_Kd, table, sizeof(FuzzyRuleTable_t));
    }
}

void PID_FuzzyV13_EnableDerivativeOnMeasurement(PID_FuzzyV13_t *pid)
{
    if (pid) {
        pid->derivative_on_measurement = 1;
    }
}

float PID_FuzzyV13_Calculate(PID_FuzzyV13_t *pid, float setpoint, float measurement)
{
    if (!pid) return 0.0f;

    pid->measurement = measurement;
    float error_new = setpoint - measurement;

    /* 误差变化率 */
    pid->error_rate = (error_new - pid->error) / pid->dt;
    pid->error = error_new;

    /* 模糊推理: 计算参数调整量 */
    float dKp, dKi, dKd;
    s_FuzzyInference(pid, &dKp, &dKi, &dKd);

    /* 更新PID参数 */
    pid->Kp = pid->Kp_base + dKp;
    pid->Ki = pid->Ki_base + dKi;
    pid->Kd = pid->Kd_base + dKd;

    /* 参数非负保护 */
    if (pid->Kp < 0.0f) pid->Kp = 0.0f;
    if (pid->Ki < 0.0f) pid->Ki = 0.0f;
    if (pid->Kd < 0.0f) pid->Kd = 0.0f;

    /* 保存调试信息 */
    pid->last_delta_Kp = dKp;
    pid->last_delta_Ki = dKi;
    pid->last_delta_Kd = dKd;

    /* 积分 */
    float abs_error = fabsf(pid->error);
    if (abs_error > pid->integral_separate_threshold) {
        pid->integral_enable = 0;
    } else {
        pid->integral_enable = 1;
    }

    if (pid->integral_enable) {
        pid->integral += pid->error * pid->dt;
        CLAMP(pid->integral, -pid->integral_max, pid->integral_max);
    }

    /* 微分 */
    if (pid->derivative_on_measurement) {
        pid->derivative = -(pid->measurement - pid->measurement_prev) / pid->dt;
    } else {
        pid->derivative = pid->error_rate;
    }

    /* PID输出 */
    pid->output = pid->Kp * pid->error
                + pid->Ki * pid->integral
                + pid->Kd * pid->derivative;

    CLAMP(pid->output, pid->output_min, pid->output_max);

    /* 更新历史 */
    pid->error_prev = pid->error;
    pid->measurement_prev = pid->measurement;

    return pid->output;
}

void PID_FuzzyV13_Reset(PID_FuzzyV13_t *pid)
{
    if (!pid) return;

    pid->error = 0.0f;
    pid->error_prev = 0.0f;
    pid->error_rate = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->output = 0.0f;
    pid->measurement_prev = 0.0f;
    pid->integral_enable = 1;

    pid->Kp = pid->Kp_base;
    pid->Ki = pid->Ki_base;
    pid->Kd = pid->Kd_base;
}

void PID_FuzzyV13_GetAdaptiveParams(const PID_FuzzyV13_t *pid, float *Kp, float *Ki, float *Kd)
{
    if (!pid) return;
    if (Kp) *Kp = pid->Kp;
    if (Ki) *Ki = pid->Ki;
    if (Kd) *Kd = pid->Kd;
}
