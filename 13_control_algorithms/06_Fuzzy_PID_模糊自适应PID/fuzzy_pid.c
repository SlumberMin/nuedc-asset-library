#include "fuzzy_pid.h"
#include <string.h>

/*
 * 模糊自适应PID实现
 * 7x7模糊规则表，三角形隶属函数
 */

/* 模糊规则表 (7x7)
 * 行: e 从NB到PB
 * 列: ec从NB到PB
 * 值: -3,-2,-1,0,1,2,3 对应NB,NM,NS,ZO,PS,PM,PB
 */

/* ΔKp规则表 */
static const int rule_Kp[FUZZY_N][FUZZY_N] = {
    /*ec: NB  NM  NS  ZO  PS  PM  PB */
    { 3,  3,  2,  2,  1,  0,  0},  /* e=NB */
    { 3,  3,  2,  1,  1,  0, -1},  /* e=NM */
    { 2,  2,  2,  1,  0, -1, -1},  /* e=NS */
    { 2,  2,  1,  0, -1, -2, -2},  /* e=ZO */
    { 1,  1,  0, -1, -1, -2, -2},  /* e=PS */
    { 1,  0, -1, -2, -2, -2, -3},  /* e=PM */
    { 0,  0, -2, -2, -2, -3, -3}   /* e=PB */
};

/* ΔKi规则表 */
static const int rule_Ki[FUZZY_N][FUZZY_N] = {
    {-3, -3, -2, -2, -1,  0,  0},
    {-3, -3, -2, -1, -1,  0,  0},
    {-2, -2, -1, -1,  0,  1,  1},
    {-2, -2, -1,  0,  1,  2,  2},
    {-1, -1,  0,  1,  1,  2,  2},
    {-1,  0,  1,  2,  2,  3,  3},
    { 0,  0,  2,  2,  2,  3,  3}
};

/* ΔKd规则表 */
static const int rule_Kd[FUZZY_N][FUZZY_N] = {
    { 3,  1,  0,  0,  0,  2,  3},
    { 3,  1,  0, -1,  0,  1,  2},
    { 2,  1,  0, -1,  0,  1,  2},
    { 2,  1,  0,  0,  0,  1,  2},
    { 2,  2,  1,  0,  0,  1,  2},
    { 3,  2,  1,  0,  0,  1,  2},
    { 3,  3,  2,  1,  0,  2,  3}
};

void FuzzyPID_Init(FuzzyPID_t *fpid, float ke, float kec, 
                    float Kp, float Ki, float Kd,
                    float k_up, float k_ui, float k_ud)
{
    memset(fpid, 0, sizeof(FuzzyPID_t));
    fpid->ke = ke; fpid->kec = kec;
    fpid->Kp = Kp; fpid->Ki = Ki; fpid->Kd = Kd;
    fpid->k_up = k_up; fpid->k_ui = k_ui; fpid->k_ud = k_ud;
    fpid->u_max = 100;
    
    /* 隶属函数中心: NB(-3), NM(-2), NS(-1), ZO(0), PS(1), PM(2), PB(3) */
    for (int i = 0; i < FUZZY_N; i++)
        fpid->mf[i] = (float)(i - 3);
}

void FuzzyPID_SetOutputLimit(FuzzyPID_t *fpid, float max) { fpid->u_max = max; }

/* 三角形隶属函数（内部使用） */
static float trimf(float x, float a, float b, float c)
{
    if (x <= a || x >= c) return 0;
    if (x <= b) return (x - a) / (b - a);
    return (c - x) / (c - b);
}

/* 梯形隶属函数（用于边界隶属函数，半开放） */
static float trapf_left(float x, float a, float b)
{
    /* 左开放梯形：x<=a时为1，x>=b时为0 */
    if (x <= a) return 1.0f;
    if (x >= b) return 0.0f;
    return (b - x) / (b - a);
}

static float trapf_right(float x, float a, float b)
{
    /* 右开放梯形：x<=a时为0，x>=b时为1 */
    if (x <= a) return 0.0f;
    if (x >= b) return 1.0f;
    return (x - a) / (b - a);
}

/* 模糊推理 + 解模糊(重心法) */
static float fuzzy_infer(const int rule[FUZZY_N][FUZZY_N], float e_fuzzy, float ec_fuzzy, 
                          float *mf, int mf_size)
{
    float w[FUZZY_N][FUZZY_N];  /* 激活强度 */
    float num = 0, den = 0;
    
    for (int i = 0; i < mf_size; i++) {
        /* e的隶属度 */
        float mu_e;
        if (i == 0) {
            mu_e = trapf_left(e_fuzzy, mf[i], mf[i]+1); /* NB: 左开放梯形 */
        } else if (i == mf_size-1) {
            mu_e = trapf_right(e_fuzzy, mf[i]-1, mf[i]); /* PB: 右开放梯形 */
        } else {
            mu_e = trimf(e_fuzzy, mf[i]-1, mf[i], mf[i]+1);
        }
        
        for (int j = 0; j < mf_size; j++) {
            /* ec的隶属度 */
            float mu_ec;
            if (j == 0) {
                mu_ec = trapf_left(ec_fuzzy, mf[j], mf[j]+1); /* NB: 左开放梯形 */
            } else if (j == mf_size-1) {
                mu_ec = trapf_right(ec_fuzzy, mf[j]-1, mf[j]); /* PB: 右开放梯形 */
            } else {
                mu_ec = trimf(ec_fuzzy, mf[j]-1, mf[j], mf[j]+1);
            }
            
            /* 取小运算 */
            w[i][j] = (mu_e < mu_ec) ? mu_e : mu_ec;
            
            /* 输出中心值 */
            float out_center = (float)rule[i][j];  /* 整数中心 */
            num += w[i][j] * out_center;
            den += w[i][j];
        }
    }
    
    return (den > 0.001f) ? num / den : 0;
}

float FuzzyPID_Update(FuzzyPID_t *fpid, float ref, float y)
{
    /* 计算误差 */
    fpid->error_last = fpid->error;
    fpid->error = ref - y;
    float delta_e = fpid->error - fpid->error_last;
    fpid->error_sum += fpid->error;
    
    /* 量化输入到[-3,3] */
    float e_fuzzy = fpid->ke * fpid->error;
    if (e_fuzzy > 3) e_fuzzy = 3;
    if (e_fuzzy < -3) e_fuzzy = -3;
    
    float ec_fuzzy = fpid->kec * delta_e;
    if (ec_fuzzy > 3) ec_fuzzy = 3;
    if (ec_fuzzy < -3) ec_fuzzy = -3;
    
    /* 模糊推理得到ΔKp, ΔKi, ΔKd */
    float dKp = fpid->k_up * fuzzy_infer(rule_Kp, e_fuzzy, ec_fuzzy, fpid->mf, FUZZY_N);
    float dKi = fpid->k_ui * fuzzy_infer(rule_Ki, e_fuzzy, ec_fuzzy, fpid->mf, FUZZY_N);
    float dKd = fpid->k_ud * fuzzy_infer(rule_Kd, e_fuzzy, ec_fuzzy, fpid->mf, FUZZY_N);
    
    /* 更新PID参数 */
    float Kp = fpid->Kp + dKp;
    float Ki = fpid->Ki + dKi;
    float Kd = fpid->Kd + dKd;
    
    /* 参数约束 */
    if (Kp < 0) Kp = 0;
    if (Ki < 0) Ki = 0;
    if (Kd < 0) Kd = 0;
    
    /* PID输出 */
    fpid->u = Kp * fpid->error + Ki * fpid->error_sum + Kd * delta_e;
    
    /* 输出限幅 */
    if (fpid->u > fpid->u_max) {
        fpid->u = fpid->u_max;
        /* 积分抗饱和：当输出饱和时冻结积分累加 */
        fpid->error_sum -= fpid->error;
    }
    if (fpid->u < -fpid->u_max) {
        fpid->u = -fpid->u_max;
        fpid->error_sum -= fpid->error;
    }
    
    return fpid->u;
}
