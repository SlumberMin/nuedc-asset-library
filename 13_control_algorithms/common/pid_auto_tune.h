/**
 * @file pid_auto_tune.h
 * @brief PID参数自动整定模块 - 支持Ziegler-Nichols法和继电反馈法(Relay Feedback)
 *
 * 方法:
 *   1. ZN法 (Ziegler-Nichols): 基于临界增益Ku和临界周期Tu
 *   2. 继电反馈法: 利用继电环节产生的极限环振荡获取临界参数
 *   3. 改进ZN法: 基于一阶+纯滞后模型的阶跃响应整定
 *
 * 使用流程:
 *   1. 初始化 AutoTune_t 结构体
 *   2. 在控制循环中调用对应的 Tune_Step() 函数
 *   3. 当 AutoTune_IsDone() 返回1后,读取结果
 */

#ifndef PID_AUTO_TUNE_H
#define PID_AUTO_TUNE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 整定方法枚举 ========== */
typedef enum {
    AT_METHOD_RELAY,        /* 继电反馈法 */
    AT_METHOD_ZN_STEP,      /* ZN阶跃响应法 */
    AT_METHOD_COHEN_COON    /* Cohen-Coon法 */
} AutoTuneMethod_e;

/* ========== 整定规则枚举 ========== */
typedef enum {
    AT_RULE_CLASSIC,        /* 经典ZN规则 */
    AT_RULE_PESSEN,         /* Pessen积分规则 */
    AT_RULE_SOME_OVERSHOOT, /* 一些超调规则 */
    AT_RULE_NO_OVERSHOOT    /* 无超调规则 */
} AutoTuneRule_e;

/* ========== 整定状态枚举 ========== */
typedef enum {
    AT_STATE_IDLE,
    AT_STATE_RELAY_WAIT,    /* 继电法: 等待进入极限环 */
    AT_STATE_RELAY_MEASURE, /* 继电法: 测量振荡参数 */
    AT_STATE_STEP_WAIT,     /* 阶跃法: 等待响应 */
    AT_STATE_STEP_MEASURE,  /* 阶跃法: 测量响应参数 */
    AT_STATE_DONE,
    AT_STATE_ERROR
} AutoTuneState_e;

/* ========== 整定结果 ========== */
typedef struct {
    float kp;               /* 整定后的比例增益 */
    float ki;               /* 整定后的积分增益 */
    float kd;               /* 整定后的微分增益 */
    float Ku;               /* 临界增益 (继电法) */
    float Tu;               /* 临界周期 (继电法) */
    float K_proc;           /* 过程增益 (阶跃法) */
    float L;                /* 纯滞后时间 (阶跃法) */
    float T;                /* 时间常数 (阶跃法) */
} AutoTuneResult_t;

/* ========== 继电反馈法参数 ========== */
typedef struct {
    float relay_amplitude;  /* 继电幅值 */
    float relay_hysteresis; /* 继电滞环宽度 */
    float output_offset;    /* 输出偏置 */
    float dt;               /* 采样周期 */
    float timeout;          /* 超时时间 */
    AutoTuneRule_e rule;    /* 整定规则 */

    /* 内部状态 */
    AutoTuneState_e state;
    float timer;
    float relay_output;
    float peak_max;
    float peak_min;
    float prev_input;
    int32_t peak_count;
    float peak_times[8];    /* 记录峰值时刻 */
    float zero_cross_time;  /* 过零时刻 */
    uint8_t rising_edge;
    float amplitude_sum;
    float period_sum;
    int32_t period_count;
    AutoTuneResult_t result;
} RelayAutoTune_t;

/* ========== 阶跃响应法参数 ========== */
typedef struct {
    float step_amplitude;   /* 阶跃幅值 */
    float dt;               /* 采样周期 */
    float timeout;          /* 超时时间 */
    AutoTuneMethod_e method;/* ZN_STEP 或 COHEN_COON */
    AutoTuneRule_e rule;    /* 整定规则(ZN法有效) */

    /* 内部状态 */
    AutoTuneState_e state;
    float timer;
    float baseline;         /* 基线值 */
    float steady_state;     /* 稳态值 */
    float t632;             /* 63.2%响应时间 */
    float t283;             /* 28.3%响应时间 */
    float threshold_632;
    float threshold_283;
    uint8_t found_283;
    uint8_t found_632;
    float start_time;
    AutoTuneResult_t result;
} StepAutoTune_t;

/* ========== 继电反馈法 API ========== */
void RelayAutoTune_Init(RelayAutoTune_t *at, float relay_amp, float hysteresis,
                        float offset, float dt, float timeout, AutoTuneRule_e rule);
float RelayAutoTune_Step(RelayAutoTune_t *at, float measurement);
uint8_t RelayAutoTune_IsDone(RelayAutoTune_t *at);
const AutoTuneResult_t* RelayAutoTune_GetResult(RelayAutoTune_t *at);
void RelayAutoTune_Reset(RelayAutoTune_t *at);

/* ========== 阶跃响应法 API ========== */
void StepAutoTune_Init(StepAutoTune_t *at, float step_amp, float dt,
                       float timeout, AutoTuneMethod_e method, AutoTuneRule_e rule);
float StepAutoTune_Step(StepAutoTune_t *at, float measurement);
uint8_t StepAutoTune_IsDone(StepAutoTune_t *at);
const AutoTuneResult_t* StepAutoTune_GetResult(StepAutoTune_t *at);
void StepAutoTune_Reset(StepAutoTune_t *at);

/* ========== 工具函数 ========== */
void AutoTune_ComputeZN(float Ku, float Tu, AutoTuneRule_e rule, AutoTuneResult_t *result);
void AutoTune_ComputeCohenCoon(float K, float L, float T, AutoTuneResult_t *result);

#ifdef __cplusplus
}
#endif

#endif /* PID_AUTO_TUNE_H */
