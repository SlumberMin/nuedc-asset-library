/**
 * @file fuzzy_pid.c
 * @brief 模糊PID控制器实现 - 7×7规则表, 在线自整定Kp/Ki/Kd
 */

#include "fuzzy_pid.h"
#include <math.h>

/* ========== 默认7×7规则表 ========== */
/* 
 * 规则表说明:
 * 行 = 误差e (NB→PB), 列 = 误差变化率ec (NB→PB)
 * 值 = 输出调整等级 (-3 ~ +3)
 * 
 * Kp规则: 误差大时增大Kp加快响应, 误差小时减小Kp避免超调
 */
static const int8_t default_rule_kp[7][7] = {
    /*  ec:  NB    NM    NS    ZO    PS    PM    PB  */
    /*e:NB*/ { PB,   PB,   PM,   PM,   PS,   ZO,   ZO  },
    /*e:NM*/ { PB,   PB,   PM,   PS,   PS,   ZO,   NS  },
    /*e:NS*/ { PM,   PM,   PM,   PS,   ZO,   NS,   NS  },
    /*e:ZO*/ { PM,   PM,   PS,   ZO,   NS,   NM,   NM  },
    /*e:PS*/ { PS,   PS,   ZO,   NS,   NS,   NM,   NM  },
    /*e:PM*/ { PS,   ZO,   NS,   NM,   NM,   NM,   NB  },
    /*e:PB*/ { ZO,   ZO,   NM,   NM,   NM,   NB,   NB  },
};

/* Ki规则: 误差小时增大Ki消除稳态误差, 误差大时减小Ki防止积分饱和 */
static const int8_t default_rule_ki[7][7] = {
    /*  ec:  NB    NM    NS    ZO    PS    PM    PB  */
    /*e:NB*/ { NB,   NB,   NM,   NM,   NS,   ZO,   ZO  },
    /*e:NM*/ { NB,   NB,   NM,   NS,   NS,   ZO,   ZO  },
    /*e:NS*/ { NB,   NM,   NS,   NS,   ZO,   PS,   PS  },
    /*e:ZO*/ { NM,   NM,   NS,   ZO,   PS,   PM,   PM  },
    /*e:PS*/ { NM,   NS,   ZO,   PS,   PS,   PM,   PB  },
    /*e:PM*/ { ZO,   ZO,   PS,   PS,   PM,   PB,   PB  },
    /*e:PB*/ { ZO,   ZO,   PS,   PM,   PM,   PB,   PB  },
};

/* Kd规则: 误差变化快时增大Kd抑制, 误差变化慢时减小Kd避免噪声放大 */
static const int8_t default_rule_kd[7][7] = {
    /*  ec:  NB    NM    NS    ZO    PS    PM    PB  */
    /*e:NB*/ { PS,   NS,   NB,   NB,   NB,   NM,   PS  },
    /*e:NM*/ { PS,   NS,   NB,   NM,   NM,   NS,   ZO  },
    /*e:NS*/ { ZO,   NS,   NM,   NM,   NS,   NS,   ZO  },
    /*e:ZO*/ { ZO,   NS,   NS,   NS,   NS,   NS,   ZO  },
    /*e:PS*/ { ZO,   ZO,   ZO,   ZO,   ZO,   ZO,   ZO  },
    /*e:PM*/ { PB,   NS,   PS,   PS,   PS,   PS,   PB  },
    /*e:PB*/ { PB,   PM,   PM,   PM,   PS,   PS,   PB  },
};

/* ========== 初始化 ========== */
void FuzzyPID_Init(FuzzyPID_t *fuzzy, float kp, float ki, float kd)
{
    fuzzy->kp_base = kp;
    fuzzy->ki_base = ki;
    fuzzy->kd_base = kd;
    fuzzy->kp = kp;
    fuzzy->ki = ki;
    fuzzy->kd = kd;
    fuzzy->target = 0;
    fuzzy->output = 0;
    
    fuzzy->delta_kp_max = kp * 0.5f;
    fuzzy->delta_ki_max = ki * 0.5f;
    fuzzy->delta_kd_max = kd * 0.5f;
    
    fuzzy->e_scale = 1.0f;
    fuzzy->ec_scale = 1.0f;
    
    fuzzy->error = 0;
    fuzzy->error_last = 0;
    fuzzy->error_dot = 0;
    fuzzy->integral = 0;
    fuzzy->output_last = 0;
    
    fuzzy->output_max = 1000;
    fuzzy->output_min = -1000;
    fuzzy->integral_max = 500;
    fuzzy->derivative_filter = 0.3f;
    
    FuzzyPID_SetDefaultRules(fuzzy);
}

void FuzzyPID_SetDefaultRules(FuzzyPID_t *fuzzy)
{
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            fuzzy->rule_kp[i][j] = default_rule_kp[i][j];
            fuzzy->rule_ki[i][j] = default_rule_ki[i][j];
            fuzzy->rule_kd[i][j] = default_rule_kd[i][j];
        }
    }
}

void FuzzyPID_SetDeltaRange(FuzzyPID_t *fuzzy, float dkp, float dki, float dkd)
{
    fuzzy->delta_kp_max = dkp;
    fuzzy->delta_ki_max = dki;
    fuzzy->delta_kd_max = dkd;
}

void FuzzyPID_SetScale(FuzzyPID_t *fuzzy, float e_scale, float ec_scale)
{
    fuzzy->e_scale = e_scale;
    fuzzy->ec_scale = ec_scale;
}

void FuzzyPID_SetTarget(FuzzyPID_t *fuzzy, float target)
{
    fuzzy->target = target;
}

void FuzzyPID_SetOutputLimit(FuzzyPID_t *fuzzy, float min, float max)
{
    fuzzy->output_min = min;
    fuzzy->output_max = max;
}

/* ========== 隶属度函数 ========== */
/* 三角隶属度函数: 返回x在模糊集idx的隶属度 */
static float TriMF(float x, float center, float width)
{
    float dist = fabsf(x - center);
    if (dist >= width) return 0;
    return 1.0f - dist / width;
}

/* 量化并计算隶属度 */
static void Fuzzify(float value, float scale, float *membership, int8_t *indices, int8_t *count)
{
    /* 将值量化到[-3, 3]范围 */
    float x = value * scale;
    if (x > 3.0f) x = 3.0f;
    if (x < -3.0f) x = -3.0f;
    
    /* 模糊集中心: -3,-2,-1,0,1,2,3 */
    float centers[7] = {-3.0f, -2.0f, -1.0f, 0.0f, 1.0f, 2.0f, 3.0f};
    float width = 1.5f; /* 三角形宽度 */
    
    *count = 0;
    for (int i = 0; i < 7; i++) {
        float m = TriMF(x, centers[i], width);
        if (m > 0.01f) {
            membership[*count] = m;
            indices[*count] = i;
            (*count)++;
        }
    }
}

/* ========== 模糊推理 ========== */
static float FuzzyDefuzzify(const int8_t rule_table[7][7], 
                             float e_membership[], int8_t e_idx[], int8_t e_count,
                             float ec_membership[], int8_t ec_idx[], int8_t ec_count,
                             float delta_max)
{
    float numerator = 0;
    float denominator = 0;
    
    for (int i = 0; i < e_count; i++) {
        for (int j = 0; j < ec_count; j++) {
            /* 取小运算(AND) */
            float w = (e_membership[i] < ec_membership[j]) ? e_membership[i] : ec_membership[j];
            
            /* 规则输出值: 将[-3,3]映射到[-delta_max, delta_max] */
            float rule_output = rule_table[e_idx[i]][ec_idx[j]] * delta_max / 3.0f;
            
            numerator += w * rule_output;
            denominator += w;
        }
    }
    
    if (denominator < 0.0001f) return 0;
    return numerator / denominator;
}

/* ========== 模糊PID计算 ========== */
float FuzzyPID_Calculate(FuzzyPID_t *fuzzy, float measurement)
{
    float e_membership[4], ec_membership[4];
    int8_t e_idx[4], ec_idx[4];
    int8_t e_count, ec_count;
    float delta_kp, delta_ki, delta_kd;
    
    /* 计算误差和误差变化率 */
    fuzzy->error = fuzzy->target - measurement;
    fuzzy->error_dot = fuzzy->error - fuzzy->error_last;
    
    /* 模糊化 */
    Fuzzify(fuzzy->error, fuzzy->e_scale, e_membership, e_idx, &e_count);
    Fuzzify(fuzzy->error_dot, fuzzy->ec_scale, ec_membership, ec_idx, &ec_count);
    
    /* 模糊推理 + 解模糊 */
    delta_kp = FuzzyDefuzzify(fuzzy->rule_kp, e_membership, e_idx, e_count,
                               ec_membership, ec_idx, ec_count, fuzzy->delta_kp_max);
    delta_ki = FuzzyDefuzzify(fuzzy->rule_ki, e_membership, e_idx, e_count,
                               ec_membership, ec_idx, ec_count, fuzzy->delta_ki_max);
    delta_kd = FuzzyDefuzzify(fuzzy->rule_kd, e_membership, e_idx, e_count,
                               ec_membership, ec_idx, ec_count, fuzzy->delta_kd_max);
    
    /* 在线调整PID参数 */
    fuzzy->kp = fuzzy->kp_base + delta_kp;
    fuzzy->ki = fuzzy->ki_base + delta_ki;
    fuzzy->kd = fuzzy->kd_base + delta_kd;
    
    /* 参数非负保护 */
    if (fuzzy->kp < 0) fuzzy->kp = 0;
    if (fuzzy->ki < 0) fuzzy->ki = 0;
    if (fuzzy->kd < 0) fuzzy->kd = 0;
    
    /* 位置式PID计算 */
    float p_term = fuzzy->kp * fuzzy->error;
    
    fuzzy->integral += fuzzy->error;
    if (fuzzy->integral > fuzzy->integral_max) fuzzy->integral = fuzzy->integral_max;
    if (fuzzy->integral < -fuzzy->integral_max) fuzzy->integral = -fuzzy->integral_max;
    float i_term = fuzzy->ki * fuzzy->integral;
    
    float d_raw = fuzzy->kd * fuzzy->error_dot;
    fuzzy->derivative_filter = 0.3f * d_raw + 0.7f * fuzzy->derivative_filter;
    float d_term = fuzzy->derivative_filter;
    
    fuzzy->error_last = fuzzy->error;
    
    /* 输出限幅 */
    fuzzy->output = p_term + i_term + d_term;
    if (fuzzy->output > fuzzy->output_max) fuzzy->output = fuzzy->output_max;
    if (fuzzy->output < fuzzy->output_min) fuzzy->output = fuzzy->output_min;
    
    return fuzzy->output;
}

void FuzzyPID_Reset(FuzzyPID_t *fuzzy)
{
    fuzzy->error = 0;
    fuzzy->error_last = 0;
    fuzzy->error_dot = 0;
    fuzzy->integral = 0;
    fuzzy->output = 0;
    fuzzy->kp = fuzzy->kp_base;
    fuzzy->ki = fuzzy->ki_base;
    fuzzy->kd = fuzzy->kd_base;
}

void FuzzyPID_GetParams(const FuzzyPID_t *fuzzy, float *kp, float *ki, float *kd)
{
    if (kp) *kp = fuzzy->kp;
    if (ki) *ki = fuzzy->ki;
    if (kd) *kd = fuzzy->kd;
}
