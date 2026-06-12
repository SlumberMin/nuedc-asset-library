/**
 * @file gain_scheduling_pid.h
 * @brief 增益调度PID控制器 (Gain Scheduling PID)
 *
 * 增益调度PID根据系统工作状态（调度变量）自动切换PID参数，
 * 适用于工作点变化较大的非线性系统。
 *
 * 原理：
 *   在不同工况下，系统特性差异大，单一PID参数难以兼顾。
 *   增益调度预先为多个工况点设计好PID参数，运行时根据
 *   调度变量实时切换或插值。
 *
 * 调度变量示例：
 *   - 电机转速（不同转速下模型不同）
 *   - 系统负载（不同负载下惯量不同）
 *   - 温度（材料特性随温度变化）
 *   - 飞行高度/速度（气动参数变化）
 *
 * 参数整定指南：
 * ==========================
 * 1. 确定调度变量：选择与系统特性最相关的物理量
 * 2. 划分工况区间：根据调度变量范围划分 3~8 个工况点
 * 3. 逐点整定PID：在每个工况点用常规方法整定PID
 * 4. 确定切换方式：
 *    - 硬切换：调度变量进入某区间直接切换参数
 *    - 软切换：参数线性插值，过渡更平滑
 * 5. 测试验证：在各工况间切换观察是否有抖动
 *
 * 适用场景：
 * - 电机全速域控制
 * - 无人机不同飞行模态
 * - 温控系统不同温度段
 * - 负载变化大的伺服系统
 */

#ifndef GAIN_SCHEDULING_PID_H
#define GAIN_SCHEDULING_PID_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 最大工况点数量 */
#define GS_PID_MAX_POINTS   8

/* 切换模式 */
typedef enum {
    GS_MODE_HARD = 0,   /* 硬切换：直接跳变 */
    GS_MODE_SOFT         /* 软切换：线性插值 */
} GS_SwitchMode_t;

/* 单个工况点的 PID 参数 */
typedef struct {
    float sv;           /* 调度变量值 */
    float Kp;           /* 比例增益 */
    float Ki;           /* 积分增益 */
    float Kd;           /* 微分增益 */
} GS_Point_t;

/* 增益调度 PID 控制器结构体 */
typedef struct {
    /* 工况点表 */
    GS_Point_t points[GS_PID_MAX_POINTS];
    int point_count;                /* 已配置的工况点数 */

    /* 切换模式 */
    GS_SwitchMode_t mode;

    /* 当前使用的 PID 参数（插值后） */
    float Kp, Ki, Kd;

    /* PID 内部状态 */
    float integral;                 /* 积分累积 */
    float last_error;               /* 上一次误差（用于微分） */
    float last_feedback;            /* 上一次反馈值（用于微分先行） */

    /* 输出限幅 */
    float u_max;
    float u_min;

    /* 积分限幅（抗积分饱和） */
    float integral_max;

    /* 微分滤波系数（一阶低通滤波） */
    float d_filter_alpha;           /* 0~1, 越小滤波越强 */

    /* 微分项滤波后的值 */
    float d_filtered;

    /* 采样周期 */
    float dt;

    /* 控制模式 */
    uint8_t use_derivative_on_pv;   /* 1=微分先行, 0=标准微分 */
} GainSchedPID_t;

/**
 * @brief 初始化增益调度 PID
 * @param pid      控制器指针
 * @param dt       采样周期 (s)
 * @param u_max    输出上限
 * @param u_min    输出下限
 * @param mode     切换模式 (硬切换/软切换)
 */
void GSPID_Init(GainSchedPID_t *pid, float dt, float u_max, float u_min,
                GS_SwitchMode_t mode);

/**
 * @brief 添加工况点（按调度变量升序添加）
 * @param sv  调度变量值
 * @param Kp  比例增益
 * @param Ki  积分增益
 * @param Kd  微分增益
 * @return 0=成功, -1=已满
 */
int GSPID_AddPoint(GainSchedPID_t *pid, float sv, float Kp, float Ki, float Kd);

/**
 * @brief 增益调度 PID 计算
 * @param pid        控制器指针
 * @param setpoint   设定值
 * @param feedback   反馈值
 * @param sv         调度变量（当前工况）
 * @return 控制输出
 */
float GSPID_Update(GainSchedPID_t *pid, float setpoint, float feedback, float sv);

/**
 * @brief 重置控制器状态
 */
void GSPID_Reset(GainSchedPID_t *pid);

/**
 * @brief 设置积分限幅
 */
void GSPID_SetIntegralLimit(GainSchedPID_t *pid, float limit);

/**
 * @brief 设置微分滤波系数
 * @param alpha  滤波系数 (0~1), 越小滤波越强，典型值 0.1~0.3
 */
void GSPID_SetDerivativeFilter(GainSchedPID_t *pid, float alpha);

/**
 * @brief 启用微分先行模式（对反馈微分而非误差微分）
 */
void GSPID_SetDerivativeOnPV(GainSchedPID_t *pid, uint8_t enable);

/**
 * @brief 获取当前使用的 PID 参数
 */
void GSPID_GetCurrentParams(const GainSchedPID_t *pid, float *Kp, float *Ki, float *Kd);

#ifdef __cplusplus
}
#endif

#endif /* GAIN_SCHEDULING_PID_H */
