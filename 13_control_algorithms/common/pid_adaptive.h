/**
 * @file pid_adaptive_v13.h
 * @brief 自适应PID V13 - 基于误差梯度的参数自整定PID
 *
 * V13特性:
 * - 误差梯度感知: 根据误差变化率实时调整PID参数
 * - 多模式切换: 支持位置式/增量式切换
 * - 抗积分饱和: 条件积分 + 积分分离
 * - 微分先行: 避免设定值突变引起的微分冲击
 * - 增益调度: 基于误差幅值的分段增益
 */

#ifndef PID_ADAPTIVE_V13_H
#define PID_ADAPTIVE_V13_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 自适应模式 */
typedef enum {
    ADAPTIVE_MODE_POSITION = 0,  /* 位置式 */
    ADAPTIVE_MODE_INCREMENTAL    /* 增量式 */
} AdaptiveMode_e;

/* PID参数自适应区间 */
typedef struct {
    float error_threshold[3];    /* 误差阈值: [小, 中, 大] */
    float kp_scale[4];           /* 各区间Kp比例系数 */
    float ki_scale[4];           /* 各区间Ki比例系数 */
    float kd_scale[4];           /* 各区间Kd比例系数 */
} AdaptiveGainSchedule_t;

/* PID自适应V13句柄 */
typedef struct {
    /* 基础参数 */
    float Kp;
    float Ki;
    float Kd;

    /* 运行时状态 */
    float error;
    float error_prev;
    float error_prev2;
    float integral;
    float derivative;
    float output;

    /* 自适应参数 */
    float adaptive_kp;
    float adaptive_ki;
    float adaptive_kd;

    /* 误差梯度 */
    float error_gradient;
    float gradient_smooth;       /* 平滑后的梯度 */
    float gradient_alpha;        /* 梯度平滑系数 */

    /* 增益调度表 */
    AdaptiveGainSchedule_t gain_schedule;

    /* 限幅 */
    float output_max;
    float output_min;
    float integral_max;

    /* 积分分离 */
    float integral_separate_threshold;  /* 积分分离阈值 */
    uint8_t integral_enable;            /* 积分使能标志 */

    /* 微分先行 */
    uint8_t derivative_on_measurement;  /* 微分作用于测量值 */
    float measurement;
    float measurement_prev;

    /* 模式 */
    AdaptiveMode_e mode;

    /* 步长 */
    float dt;

    /* 自适应速率限制 */
    float param_change_rate_max;  /* 参数最大变化速率 */
} PID_AdaptiveV13_t;

/**
 * @brief 初始化自适应PID V13
 * @param pid PID句柄指针
 * @param Kp 比例系数
 * @param Ki 积分系数
 * @param Kd 微分系数
 * @param dt 控制周期(秒)
 */
void PID_AdaptiveV13_Init(PID_AdaptiveV13_t *pid, float Kp, float Ki, float Kd, float dt);

/**
 * @brief 配置增益调度表
 */
void PID_AdaptiveV13_SetGainSchedule(PID_AdaptiveV13_t *pid, const AdaptiveGainSchedule_t *schedule);

/**
 * @brief 设置自适应模式
 */
void PID_AdaptiveV13_SetMode(PID_AdaptiveV13_t *pid, AdaptiveMode_e mode);

/**
 * @brief 设置输出限幅
 */
void PID_AdaptiveV13_SetOutputLimit(PID_AdaptiveV13_t *pid, float min, float max);

/**
 * @brief 设置积分限幅
 */
void PID_AdaptiveV13_SetIntegralLimit(PID_AdaptiveV13_t *pid, float max);

/**
 * @brief 设置积分分离阈值
 */
void PID_AdaptiveV13_SetIntegralSeparate(PID_AdaptiveV13_t *pid, float threshold);

/**
 * @brief 设置梯度平滑系数
 */
void PID_AdaptiveV13_SetGradientSmooth(PID_AdaptiveV13_t *pid, float alpha);

/**
 * @brief 启用微分先行
 */
void PID_AdaptiveV13_EnableDerivativeOnMeasurement(PID_AdaptiveV13_t *pid);

/**
 * @brief 自适应PID V13计算
 * @param pid PID句柄
 * @param setpoint 设定值
 * @param measurement 测量值
 * @return 控制输出
 */
float PID_AdaptiveV13_Calculate(PID_AdaptiveV13_t *pid, float setpoint, float measurement);

/**
 * @brief 复位PID状态
 */
void PID_AdaptiveV13_Reset(PID_AdaptiveV13_t *pid);

/**
 * @brief 获取当前自适应后的参数
 */
void PID_AdaptiveV13_GetAdaptiveParams(const PID_AdaptiveV13_t *pid, float *Kp, float *Ki, float *Kd);

#ifdef __cplusplus
}
#endif

#endif /* PID_ADAPTIVE_V13_H */
