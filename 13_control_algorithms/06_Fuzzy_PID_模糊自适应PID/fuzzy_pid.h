#ifndef FUZZY_PID_H
#define FUZZY_PID_H

/*
 * 模糊自适应PID (Fuzzy Adaptive PID)
 * 
 * 原理：利用模糊推理系统根据误差e和误差变化率ec在线调整ΔKp,ΔKi,ΔKd
 * 
 * 模糊规则表：7x7规则库
 * 输入变量：e, ec (均归一化到[-3,3])
 * 输出变量：ΔKp, ΔKi, ΔKd
 * 隶属函数：三角形
 * 
 * 适用场景：非线性系统、模型不确定系统
 * 参数整定：调整量化因子ke,kec和比例因子kup,kui,kud
 */

#define FUZZY_N 7  /* 语言变量个数: NB,NM,NS,ZO,PS,PM,PB */

typedef struct {
    /* 量化因子 */
    float ke;      /* 误差量化因子 */
    float kec;     /* 误差变化率量化因子 */
    
    /* 比例因子 */
    float k_up;    /* ΔKp比例因子 */
    float k_ui;    /* ΔKi比例因子 */
    float k_ud;    /* ΔKd比例因子 */
    
    /* PID基础参数 */
    float Kp, Ki, Kd;
    
    /* PID状态 */
    float error;
    float error_last;
    float error_sum;
    float u;
    float u_max;
    
    /* 隶属函数参数(三角形中心点) */
    float mf[FUZZY_N];
} FuzzyPID_t;

void FuzzyPID_Init(FuzzyPID_t *fpid, float ke, float kec, 
                    float Kp, float Ki, float Kd,
                    float k_up, float k_ui, float k_ud);
float FuzzyPID_Update(FuzzyPID_t *fpid, float ref, float y);
void FuzzyPID_SetOutputLimit(FuzzyPID_t *fpid, float max);

#endif
