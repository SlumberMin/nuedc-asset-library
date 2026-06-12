/**
 * @file pid_scheduled.c
 * @brief 增益调度PID控制器实现 - 按工况切换参数
 */

#include "pid_scheduled.h"

/* ---------- 内部: 查找调度变量所属区间 ---------- */
static uint8_t find_region(const PID_Scheduled_t *pid, float sched_var)
{
    /* 从最后一个区间向前搜索，找到 threshold <= sched_var 的区间 */
    for (int i = (int)pid->num_regions - 1; i >= 0; i--) {
        if (sched_var >= pid->regions[i].threshold) {
            return (uint8_t)i;
        }
    }
    /* 低于所有阈值，使用第0个区间 */
    return 0;
}

/* ============================================================ */

void PID_Scheduled_Init(PID_Scheduled_t *pid, float dt)
{
    pid->num_regions = 0;
    pid->active_region = 0;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output = 0.0f;
    pid->dt = dt;
    pid->reset_on_switch = 1;   /* 默认切换时重置积分 */
    pid->anti_windup = 0;
}

int PID_Scheduled_AddRegion(PID_Scheduled_t *pid, float threshold,
                            float kp, float ki, float kd,
                            float out_min, float out_max)
{
    if (pid->num_regions >= PID_SCHEDULED_MAX_REGIONS) {
        return -1;
    }

    uint8_t idx = pid->num_regions;
    pid->regions[idx].threshold = threshold;
    pid->regions[idx].params.kp = kp;
    pid->regions[idx].params.ki = ki;
    pid->regions[idx].params.kd = kd;
    pid->regions[idx].params.out_min = out_min;
    pid->regions[idx].params.out_max = out_max;
    pid->num_regions++;

    return 0;
}

void PID_Scheduled_SetResetOnSwitch(PID_Scheduled_t *pid, uint8_t enable)
{
    pid->reset_on_switch = enable;
}

void PID_Scheduled_EnableAntiWindup(PID_Scheduled_t *pid, uint8_t enable)
{
    pid->anti_windup = enable;
}

float PID_Scheduled_Update(PID_Scheduled_t *pid,
                           float setpoint, float feedback,
                           float sched_var)
{
    if (pid->num_regions == 0) {
        return 0.0f;
    }

    /* 1. 查找当前工况区间 */
    uint8_t new_region = find_region(pid, sched_var);

    /* 2. 区间切换时重置积分（可选） */
    if (new_region != pid->active_region) {
        if (pid->reset_on_switch) {
            pid->integral = 0.0f;
        }
        pid->active_region = new_region;
    }

    /* 3. 获取当前区间参数 */
    const PID_Scheduled_Params_t *p = &pid->regions[pid->active_region].params;

    /* 4. PID计算 */
    float error = setpoint - feedback;

    /* 积分项 */
    pid->integral += error * pid->dt;

    /* 微分项（对误差微分） */
    float derivative = (error - pid->prev_error) / pid->dt;
    pid->prev_error = error;

    /* 合成输出 */
    float output = p->kp * error + p->ki * pid->integral + p->kd * derivative;

    /* 5. 输出限幅 */
    if (output > p->out_max) {
        output = p->out_max;
        if (pid->anti_windup) {
            /* 回退积分，防止饱和 */
            pid->integral -= error * pid->dt;
        }
    } else if (output < p->out_min) {
        output = p->out_min;
        if (pid->anti_windup) {
            pid->integral -= error * pid->dt;
        }
    }

    pid->output = output;
    return output;
}

uint8_t PID_Scheduled_GetActiveRegion(const PID_Scheduled_t *pid)
{
    return pid->active_region;
}

void PID_Scheduled_Reset(PID_Scheduled_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output = 0.0f;
}
