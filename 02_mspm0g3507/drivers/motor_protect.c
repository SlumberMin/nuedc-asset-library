/**
 * @file    motor_protect.c
 * @brief   电机保护模块实现
 * @note    基于机器人竞赛优秀方案，适配MSPM0G3507
 */

#include "motor_protect.h"

/* ---------- 初始化 ---------- */

void MotorProt_Init(MotorProtect_HandleTypeDef *hmp)
{
    /* 清零 */
    hmp->state           = MOTOR_STATE_IDLE;
    hmp->fault_flag      = MOTOR_PROT_NONE;
    hmp->current_a       = 0.0f;
    hmp->temperature_c   = 25.0f;
    hmp->speed_rpm       = 0.0f;
    hmp->stall_counter_ms = 0;
    hmp->cooldown_counter = 0;
    hmp->fault_count     = 0;

    /* 默认保护参数 */
    hmp->param.current_limit        = 5.0f;    /* 5A */
    hmp->param.temp_limit           = 80.0f;   /* 80°C */
    hmp->param.stall_speed_thresh   = 50.0f;   /* 50 RPM */
    hmp->param.stall_current_thresh = 3.0f;    /* 3A */
    hmp->param.stall_time_ms        = 500;     /* 500ms */
    hmp->param.cooldown_ms          = 3000;    /* 3s */
}

/* ---------- 设置保护参数 ---------- */

void MotorProt_SetParam(MotorProtect_HandleTypeDef *hmp,
                        float current_limit, float temp_limit,
                        float stall_speed_thresh, float stall_current_thresh,
                        uint32_t stall_time_ms, uint32_t cooldown_ms)
{
    hmp->param.current_limit        = current_limit;
    hmp->param.temp_limit           = temp_limit;
    hmp->param.stall_speed_thresh   = stall_speed_thresh;
    hmp->param.stall_current_thresh = stall_current_thresh;
    hmp->param.stall_time_ms        = stall_time_ms;
    hmp->param.cooldown_ms          = cooldown_ms;
}

/* ---------- 周期性更新 ---------- */

void MotorProt_Update(MotorProtect_HandleTypeDef *hmp,
                      float current_a, float temperature_c, float speed_rpm)
{
    /* 更新实时数据 */
    hmp->current_a     = current_a;
    hmp->temperature_c = temperature_c;
    hmp->speed_rpm     = speed_rpm;

    /* 冷却状态处理 */
    if (hmp->state == MOTOR_STATE_COOLDOWN) {
        hmp->cooldown_counter += 10;  /* 假设10ms调用周期 */
        if (hmp->cooldown_counter >= hmp->param.cooldown_ms) {
            hmp->cooldown_counter = 0;
            hmp->fault_flag = MOTOR_PROT_NONE;
            hmp->state = MOTOR_STATE_IDLE;
        }
        return;
    }

    /* 已保护状态不重复检测 */
    if (hmp->state == MOTOR_STATE_PROTECTED) {
        return;
    }

    /* 过流检测 */
    float abs_current = current_a;
    if (abs_current < 0.0f) abs_current = -abs_current;
    if (abs_current > hmp->param.current_limit) {
        hmp->fault_flag |= MOTOR_PROT_OVERCURRENT;
        hmp->fault_count++;
        hmp->cooldown_counter = 0;
        hmp->state = MOTOR_STATE_COOLDOWN;
        return;
    }

    /* 过温检测 */
    if (temperature_c > hmp->param.temp_limit) {
        hmp->fault_flag |= MOTOR_PROT_OVERTEMP;
        hmp->fault_count++;
        hmp->cooldown_counter = 0;
        hmp->state = MOTOR_STATE_COOLDOWN;
        return;
    }

    /* 堵转检测：低速 + 高电流持续一段时间 */
    float abs_speed = speed_rpm;
    if (abs_speed < 0.0f) abs_speed = -abs_speed;
    if (abs_speed < hmp->param.stall_speed_thresh &&
        abs_current > hmp->param.stall_current_thresh) {
        hmp->stall_counter_ms += 10;  /* 10ms累加 */
        if (hmp->stall_counter_ms >= hmp->param.stall_time_ms) {
            hmp->fault_flag |= MOTOR_PROT_STALL;
            hmp->fault_count++;
            hmp->stall_counter_ms = 0;
            hmp->cooldown_counter = 0;
            hmp->state = MOTOR_STATE_COOLDOWN;
            return;
        }
    } else {
        hmp->stall_counter_ms = 0;  /* 条件不满足，重置计时 */
    }

    /* 正常运行 */
    hmp->state = MOTOR_STATE_RUNNING;
}

/* ---------- 查询接口 ---------- */

motor_state_t MotorProt_GetState(MotorProtect_HandleTypeDef *hmp)
{
    return hmp->state;
}

uint8_t MotorProt_GetFault(MotorProtect_HandleTypeDef *hmp)
{
    return hmp->fault_flag;
}

void MotorProt_ClearFault(MotorProtect_HandleTypeDef *hmp)
{
    hmp->fault_flag       = MOTOR_PROT_NONE;
    hmp->stall_counter_ms = 0;
    hmp->cooldown_counter = 0;
    hmp->state            = MOTOR_STATE_IDLE;
}

bool MotorProt_IsOutputEnabled(MotorProtect_HandleTypeDef *hmp)
{
    return (hmp->state == MOTOR_STATE_RUNNING || hmp->state == MOTOR_STATE_IDLE);
}

const char* MotorProt_GetFaultStr(MotorProtect_HandleTypeDef *hmp)
{
    if (hmp->fault_flag & MOTOR_PROT_OVERCURRENT) return "OVERCURRENT";
    if (hmp->fault_flag & MOTOR_PROT_OVERTEMP)    return "OVERTEMP";
    if (hmp->fault_flag & MOTOR_PROT_STALL)       return "STALL";
    return "NONE";
}
