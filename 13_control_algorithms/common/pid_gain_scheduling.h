/**
 * @file pid_gain_scheduling.h
 * @brief 增益调度PID V2 - 线性插值法
 *
 * 与 pid_scheduled（硬切换）不同，本模块在相邻工况区间之间
 * 对PID参数进行线性插值，实现平滑过渡，避免切换时的输出跳变。
 *
 * 原理:
 *   给定 N 个标定点 (v[i], kp[i], ki[i], kd[i])，
 *   对于调度变量 v 落在 v[i] ~ v[i+1] 之间时:
 *     t = (v - v[i]) / (v[i+1] - v[i])
 *     kp_eff = kp[i] + t * (kp[i+1] - kp[i])
 *   ki, kd 同理。超出范围则钳位到边界值。
 */

#ifndef PID_GAIN_SCHEDULING_H
#define PID_GAIN_SCHEDULING_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PID_GS_MAX_POINTS  16

/**
 * @brief 增益调度PID V2 控制器句柄
 */
typedef struct {
    /* 调度表: 每个标定点对应一组PID参数 */
    float sched_value[PID_GS_MAX_POINTS];   /**< 调度变量标定值(递增) */
    float kp_table[PID_GS_MAX_POINTS];
    float ki_table[PID_GS_MAX_POINTS];
    float kd_table[PID_GS_MAX_POINTS];
    uint8_t num_points;

    /* 输出限幅(全局) */
    float out_min;
    float out_max;

    /* PID内部状态 */
    float integral;
    float prev_error;
    float prev_derivative;   /**< 一阶滤波后的微分值 */
    float derivative_alpha;  /**< 微分滤波系数 (0~1, 1=不滤波) */
    float output;
    float dt;

    /* 抗积分饱和 */
    uint8_t anti_windup;
} PID_GainSched_t;

/**
 * @brief  初始化
 */
void PID_GS_Init(PID_GainSched_t *pid, float dt);

/**
 * @brief  设置全局输出限幅
 */
void PID_GS_SetOutputLimit(PID_GainSched_t *pid, float out_min, float out_max);

/**
 * @brief  设置微分低通滤波系数
 * @param  alpha  0~1, 1=不滤波, 越小滤波越强
 */
void PID_GS_SetDerivFilter(PID_GainSched_t *pid, float alpha);

/**
 * @brief  使能抗积分饱和
 */
void PID_GS_EnableAntiWindup(PID_GainSched_t *pid, uint8_t enable);

/**
 * @brief  添加标定点（调度值必须递增）
 * @return 0=成功, -1=表已满或非递增
 */
int PID_GS_AddPoint(PID_GainSched_t *pid,
                    float sched_val, float kp, float ki, float kd);

/**
 * @brief  增益调度PID计算
 * @param  pid       控制器句柄
 * @param  setpoint  目标值
 * @param  feedback  反馈值
 * @param  sched_var 调度变量
 * @return 控制输出
 */
float PID_GS_Update(PID_GainSched_t *pid,
                    float setpoint, float feedback,
                    float sched_var);

/**
 * @brief  获取当前插值后的有效参数（用于调试）
 */
void PID_GS_GetEffectiveParams(const PID_GainSched_t *pid, float sched_var,
                               float *kp_eff, float *ki_eff, float *kd_eff);

/**
 * @brief  重置
 */
void PID_GS_Reset(PID_GainSched_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* PID_GAIN_SCHEDULING_H */
