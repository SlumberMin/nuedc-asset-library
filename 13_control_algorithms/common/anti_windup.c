/**
 * @file anti_windup.c
 * @brief 抗积分饱和模块实现
 */

#include "anti_windup.h"

float AntiWindup_Clampf(float val, float min, float max)
{
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

/* ========== 条件积分 ========== */

void PID_Conditional_Init(PID_Conditional_t *pid, float kp, float ki, float kd,
                          float out_min, float out_max, float dt)
{
    if (pid == NULL) return;
    pid->kp = kp;  pid->ki = ki;  pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->out_min = out_min;  pid->out_max = out_max;
    pid->dt = dt;
}

float PID_Conditional_Calc(PID_Conditional_t *pid, float setpoint, float measurement)
{
    if (pid == NULL || pid->dt <= 0.0f) return 0.0f;
    float error = setpoint - measurement;
    float p_out = pid->kp * error;

    /* 条件积分: 仅当输出未饱和 或 误差方向使积分减小时才积分 */
    float unconstrained = p_out + pid->ki * pid->integral + pid->kd * (error - pid->prev_error) / pid->dt;
    if ((unconstrained < pid->out_max && pid->ki * error > 0) ||
        (unconstrained > pid->out_min && pid->ki * error < 0) ||
        (unconstrained >= pid->out_min && unconstrained <= pid->out_max))
    {
        pid->integral += error * pid->dt;
    }

    float i_out = pid->ki * pid->integral;
    float d_out = pid->kd * (error - pid->prev_error) / pid->dt;
    float output = p_out + i_out + d_out;

    pid->prev_error = error;
    return AntiWindup_Clampf(output, pid->out_min, pid->out_max);
}

/* ========== 反计算抗饱和 ========== */

void PID_BackCalc_Init(PID_BackCalc_t *pid, float kp, float ki, float kd,
                       float kb, float out_min, float out_max, float dt)
{
    pid->kp = kp;  pid->ki = ki;  pid->kd = kd;
    pid->kb = kb;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->out_min = out_min;  pid->out_max = out_max;
    pid->dt = dt;
}

float PID_BackCalc_Calc(PID_BackCalc_t *pid, float setpoint, float measurement)
{
    if (pid == NULL || pid->dt <= 0.0f) return 0.0f;
    float error = setpoint - measurement;

    float p_out = pid->kp * error;
    float i_out = pid->ki * pid->integral;
    float d_out = pid->kd * (error - pid->prev_error) / pid->dt;
    float unsaturated = p_out + i_out + d_out;
    float output = AntiWindup_Clampf(unsaturated, pid->out_min, pid->out_max);

    /* 反计算: 积分项 += (Ki*e + Kb*(output - unsaturated)) * dt */
    pid->integral += (pid->ki * error + pid->kb * (output - unsaturated)) * pid->dt;
    pid->prev_error = error;

    /* 重新计算输出(积分项已修正) */
    i_out = pid->ki * pid->integral;
    output = p_out + i_out + d_out;
    return AntiWindup_Clampf(output, pid->out_min, pid->out_max);
}

/* ========== 积分限幅 ========== */

void PID_Clamp_Init(PID_Clamp_t *pid, float kp, float ki, float kd,
                    float int_min, float int_max,
                    float out_min, float out_max, float dt)
{
    pid->kp = kp;  pid->ki = ki;  pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->int_min = int_min;  pid->int_max = int_max;
    pid->out_min = out_min;  pid->out_max = out_max;
    pid->dt = dt;
}

float PID_Clamp_Calc(PID_Clamp_t *pid, float setpoint, float measurement)
{
    if (pid == NULL || pid->dt <= 0.0f) return 0.0f;
    float error = setpoint - measurement;

    pid->integral += error * pid->dt;
    pid->integral = AntiWindup_Clampf(pid->integral, pid->int_min, pid->int_max);

    float p_out = pid->kp * error;
    float i_out = pid->ki * pid->integral;
    float d_out = pid->kd * (error - pid->prev_error) / pid->dt;
    float output = p_out + i_out + d_out;

    pid->prev_error = error;
    return AntiWindup_Clampf(output, pid->out_min, pid->out_max);
}

/* ========== 增量式PID ========== */

void PID_Incremental_Init(PID_Incremental_t *pid, float kp, float ki, float kd,
                          float out_min, float out_max, float dt)
{
    pid->kp = kp;  pid->ki = ki;  pid->kd = kd;
    pid->prev_error = 0.0f;
    pid->prev_prev_error = 0.0f;
    pid->output = 0.0f;
    pid->out_min = out_min;  pid->out_max = out_max;
    pid->dt = dt;
}

float PID_Incremental_Calc(PID_Incremental_t *pid, float setpoint, float measurement)
{
    if (pid == NULL || pid->dt <= 0.0f) return 0.0f;
    float error = setpoint - measurement;

    /* 增量公式: Δu = Kp*(e-e1) + Ki*e*dt + Kd*(e-2*e1+e2)/dt */
    float delta = pid->kp * (error - pid->prev_error)
                + pid->ki * error * pid->dt
                + pid->kd * (error - 2.0f * pid->prev_error + pid->prev_prev_error) / pid->dt;

    pid->output += delta;
    pid->output = AntiWindup_Clampf(pid->output, pid->out_min, pid->out_max);

    pid->prev_prev_error = pid->prev_error;
    pid->prev_error = error;

    return pid->output;
}
