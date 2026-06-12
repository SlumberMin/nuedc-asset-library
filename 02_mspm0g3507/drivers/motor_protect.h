/**
 * @file    motor_protect.h
 * @brief   电机保护模块（过流/过温/堵转检测）
 * @note    基于机器人竞赛优秀方案，适配MSPM0G3507
 */

#ifndef MOTOR_PROTECT_H
#define MOTOR_PROTECT_H

#include <stdint.h>
#include <stdbool.h>

/* 保护标志位 */
#define MOTOR_PROT_NONE         0x00
#define MOTOR_PROT_OVERCURRENT  0x01    /* 过流 */
#define MOTOR_PROT_OVERTEMP     0x02    /* 过温 */
#define MOTOR_PROT_STALL        0x04    /* 堵转 */
#define MOTOR_PROT_ALL          0x07

/* 电机状态 */
typedef enum {
    MOTOR_STATE_IDLE = 0,
    MOTOR_STATE_RUNNING,
    MOTOR_STATE_PROTECTED,
    MOTOR_STATE_COOLDOWN
} motor_state_t;

/* 保护参数 */
typedef struct {
    float current_limit;        /* 过流阈值 (A) */
    float temp_limit;           /* 过温阈值 (°C) */
    float stall_speed_thresh;   /* 堵转速度阈值 (RPM) */
    float stall_current_thresh; /* 堵转电流阈值 (A) */
    uint32_t stall_time_ms;     /* 堵转持续时间阈值 (ms) */
    uint32_t cooldown_ms;       /* 冷却时间 (ms) */
} motor_prot_param_t;

/* 保护句柄 */
typedef struct {
    motor_state_t state;        /* 当前状态 */
    uint8_t       fault_flag;   /* 故障标志 */

    /* 实时数据 */
    float current_a;            /* 相电流 (A) */
    float temperature_c;        /* 温度 (°C) */
    float speed_rpm;            /* 转速 (RPM) */

    /* 堵转检测 */
    uint32_t stall_counter_ms;  /* 低速高电流持续计时 */
    uint32_t cooldown_counter;  /* 冷却计时 */

    /* 保护参数 */
    motor_prot_param_t param;

    /* 统计 */
    uint32_t fault_count;       /* 累计故障次数 */
} MotorProtect_HandleTypeDef;

/* 初始化 */
void MotorProt_Init(MotorProtect_HandleTypeDef *hmp);

/* 设置保护参数 */
void MotorProt_SetParam(MotorProtect_HandleTypeDef *hmp,
                        float current_limit, float temp_limit,
                        float stall_speed_thresh, float stall_current_thresh,
                        uint32_t stall_time_ms, uint32_t cooldown_ms);

/* 周期性更新（建议10ms调用一次） */
void MotorProt_Update(MotorProtect_HandleTypeDef *hmp,
                      float current_a, float temperature_c, float speed_rpm);

/* 获取当前状态 */
motor_state_t MotorProt_GetState(MotorProtect_HandleTypeDef *hmp);

/* 获取故障标志 */
uint8_t MotorProt_GetFault(MotorProtect_HandleTypeDef *hmp);

/* 清除故障（手动恢复） */
void MotorProt_ClearFault(MotorProtect_HandleTypeDef *hmp);

/* 是否允许输出 */
bool MotorProt_IsOutputEnabled(MotorProtect_HandleTypeDef *hmp);

/* 获取故障描述字符串 */
const char* MotorProt_GetFaultStr(MotorProtect_HandleTypeDef *hmp);

#endif /* MOTOR_PROTECT_H */
