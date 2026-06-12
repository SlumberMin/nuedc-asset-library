/**
 * @file pid_scheduled.h
 * @brief 增益调度PID控制器 - 按工况切换参数
 *
 * 将工作范围划分为多个区间，每个区间使用独立的PID参数。
 * 适用于系统特性随工作点显著变化的场合（如电机不同转速段、
 * 不同负载条件等）。
 *
 * 使用方法:
 *   1. 定义工况区间数组（阈值 + 对应PID参数）
 *   2. 调用 PID_Scheduled_Init 初始化
 *   3. 每个控制周期调用 PID_Scheduled_Update，传入调度变量
 */

#ifndef PID_SCHEDULED_H
#define PID_SCHEDULED_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 最大支持的工况区间数 */
#define PID_SCHEDULED_MAX_REGIONS  8

/**
 * @brief 单个工况区间的PID参数
 */
typedef struct {
    float kp;          /**< 比例增益 */
    float ki;          /**< 积分增益 */
    float kd;          /**< 微分增益 */
    float out_min;     /**< 输出下限 */
    float out_max;     /**< 输出上限 */
} PID_Scheduled_Params_t;

/**
 * @brief 工况区间定义
 * @note  区间 i 满足: threshold[i] <= 调度变量 < threshold[i+1]
 *        最后一个区间: 调度变量 >= threshold[num_regions-1]
 */
typedef struct {
    float threshold;              /**< 区间起始阈值（递增排列） */
    PID_Scheduled_Params_t params; /**< 该区间的PID参数 */
} PID_Scheduled_Region_t;

/**
 * @brief 增益调度PID控制器句柄
 */
typedef struct {
    /* 工况区间表 */
    PID_Scheduled_Region_t regions[PID_SCHEDULED_MAX_REGIONS];
    uint8_t num_regions;

    /* 当前激活的区间索引 */
    uint8_t active_region;

    /* PID内部状态（每个区间独立，切换时重置积分） */
    float integral;
    float prev_error;
    float output;

    /* 配置 */
    float dt;                    /**< 控制周期(s) */
    uint8_t reset_on_switch;     /**< 切换区间时是否重置积分 */

    /* 抗积分饱和 */
    uint8_t anti_windup;
} PID_Scheduled_t;

/**
 * @brief  初始化增益调度PID
 * @param  pid  控制器句柄
 * @param  dt   控制周期(秒)
 */
void PID_Scheduled_Init(PID_Scheduled_t *pid, float dt);

/**
 * @brief  添加工况区间
 * @param  pid        控制器句柄
 * @param  threshold  调度变量阈值（区间起始值）
 * @param  kp,ki,kd   PID参数
 * @param  out_min    输出下限
 * @param  out_max    输出上限
 * @return 0=成功, -1=区间已满
 */
int PID_Scheduled_AddRegion(PID_Scheduled_t *pid, float threshold,
                            float kp, float ki, float kd,
                            float out_min, float out_max);

/**
 * @brief  设置区间切换时是否重置积分
 */
void PID_Scheduled_SetResetOnSwitch(PID_Scheduled_t *pid, uint8_t enable);

/**
 * @brief  使能抗积分饱和
 */
void PID_Scheduled_EnableAntiWindup(PID_Scheduled_t *pid, uint8_t enable);

/**
 * @brief  增益调度PID计算
 * @param  pid       控制器句柄
 * @param  setpoint  目标值
 * @param  feedback  反馈值
 * @param  sched_var 调度变量（用于选择工况区间）
 * @return 控制输出
 */
float PID_Scheduled_Update(PID_Scheduled_t *pid,
                           float setpoint, float feedback,
                           float sched_var);

/**
 * @brief  获取当前激活的区间索引
 */
uint8_t PID_Scheduled_GetActiveRegion(const PID_Scheduled_t *pid);

/**
 * @brief  重置PID内部状态
 */
void PID_Scheduled_Reset(PID_Scheduled_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* PID_SCHEDULED_H */
