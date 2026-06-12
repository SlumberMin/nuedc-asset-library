#include "trajectory_generator.h"
#include <math.h>
#include <string.h>

/* 内部辅助函数：限幅 */
static float clampf(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

void Trajectory_Init(TrajectoryGenerator_t *gen, float dt)
{
    memset(gen, 0, sizeof(TrajectoryGenerator_t));
    gen->dt = dt;
    gen->is_initialized = 1;
}

/* 计算梯形曲线的各段时间 */
static void calculate_trapezoidal_times(TrajectoryGenerator_t *gen)
{
    float distance = fabsf(gen->config.target_position - gen->initial_position);
    float max_vel = gen->config.max_velocity;
    float max_accel = gen->config.max_acceleration;
    
    /* 计算加速段时间 */
    if (max_accel < 1e-6f) max_accel = 1e-6f;  /* V2审计: 防除零 */
    gen->ta = max_vel / max_accel;
    
    /* 计算加速段距离 */
    float accel_distance = 0.5f * max_accel * gen->ta * gen->ta;
    
    if (2.0f * accel_distance >= distance) {
        /* 三角形曲线：未达到最大速度 */
        if (max_accel < 1e-6f) max_accel = 1e-6f;  /* V2审计: 防除零 */
        gen->ta = sqrtf(distance / max_accel);
        gen->tc = 0.0f;
        gen->td = gen->ta;
    } else {
        /* 完整梯形曲线 */
        float cruise_distance = distance - 2.0f * accel_distance;
        if (max_vel < 1e-6f) max_vel = 1e-6f;  /* V2审计: 防除零 */
        gen->tc = cruise_distance / max_vel;
        gen->td = gen->ta;
    }
    
    gen->total_time = gen->ta + gen->tc + gen->td;
}

/* 计算S曲线的各段时间 */
static void calculate_scurve_times(TrajectoryGenerator_t *gen)
{
    float distance = fabsf(gen->config.target_position - gen->initial_position);
    float max_vel = gen->config.max_velocity;
    float max_accel = gen->config.max_acceleration;
    float max_jerk = gen->config.max_jerk;
    
    /* 计算加加速度段时间 */
    if (max_jerk <= 0) { gen->total_time = 0; return; }
    gen->t_jerk = max_accel / max_jerk;
    
    /* 加速段：加加速度 + 匀加速 + 减加速度 */
    if (max_accel < 1e-6f) max_accel = 1e-6f;  /* V2审计: 防除零 */
    float accel_time = max_vel / max_accel;
    gen->t_accel_const = accel_time - gen->t_jerk;
    if (gen->t_accel_const < 0) {
        gen->t_accel_const = 0;
        gen->t_jerk = sqrtf(max_vel / max_jerk);
    }
    
    /* 加速段总时间 */
    float ta_total = 2.0f * gen->t_jerk + gen->t_accel_const;
    
    /* 加速段距离 */
    float accel_distance = 0.5f * max_vel * ta_total;
    
    if (2.0f * accel_distance >= distance) {
        /* 三角形S曲线 */
        gen->t_accel_const = 0;
        gen->t_jerk = powf(distance / (2.0f * max_jerk), 1.0f/3.0f);
        gen->t_cruise_const = 0;
        gen->t_decel_const = 0;
        gen->t_jerk_decel = gen->t_jerk;
    } else {
        /* 完整S曲线 */
        float cruise_distance = distance - 2.0f * accel_distance;
        if (max_vel < 1e-6f) max_vel = 1e-6f;  /* V2审计: 防除零 */
        gen->t_cruise_const = cruise_distance / max_vel;
        gen->t_decel_const = gen->t_accel_const;
        gen->t_jerk_decel = gen->t_jerk;
    }
    
    gen->total_time = 2.0f * gen->t_jerk + gen->t_accel_const + 
                     gen->t_cruise_const + 
                     2.0f * gen->t_jerk_decel + gen->t_decel_const;
}

void Trajectory_SetTrapezoidal(TrajectoryGenerator_t *gen,
                               float start_pos, float target_pos,
                               float max_vel, float max_accel)
{
    gen->config.type = TRAJ_TRAPEZOIDAL;
    gen->config.target_position = target_pos;
    gen->config.max_velocity = fabsf(max_vel);
    if (gen->config.max_velocity < 1e-6f) gen->config.max_velocity = 1e-6f;
    gen->config.max_acceleration = fabsf(max_accel);
    if (gen->config.max_acceleration < 1e-6f) gen->config.max_acceleration = 1e-6f;
    
    gen->initial_position = start_pos;
    gen->current_time = 0.0f;
    
    calculate_trapezoidal_times(gen);
}

void Trajectory_SetSCurve(TrajectoryGenerator_t *gen,
                          float start_pos, float target_pos,
                          float max_vel, float max_accel, float max_jerk)
{
    gen->config.type = TRAJ_S_CURVE;
    gen->config.target_position = target_pos;
    gen->config.max_velocity = fabsf(max_vel);
    if (gen->config.max_velocity < 1e-6f) gen->config.max_velocity = 1e-6f;
    gen->config.max_acceleration = fabsf(max_accel);
    if (gen->config.max_acceleration < 1e-6f) gen->config.max_acceleration = 1e-6f;
    gen->config.max_jerk = fabsf(max_jerk);
    if (gen->config.max_jerk < 1e-6f) gen->config.max_jerk = 1e-6f;
    
    gen->initial_position = start_pos;
    gen->current_time = 0.0f;
    
    calculate_scurve_times(gen);
}

void Trajectory_SetPolynomial(TrajectoryGenerator_t *gen,
                              float start_pos, float target_pos,
                              float start_vel, float target_vel,
                              float start_accel, float target_accel,
                              float total_time, TrajectoryType_t type)
{
    gen->config.type = type;
    gen->config.target_position = target_pos;
    gen->initial_position = start_pos;
    gen->total_time = total_time;
    gen->current_time = 0.0f;
    
    /* 存储多项式系数在config中(简化处理) */
    gen->config.max_velocity = start_vel;
    gen->config.max_acceleration = target_vel;
    gen->config.max_jerk = start_accel;
    gen->config.jerk_time = target_accel;
}

/* 梯形曲线计算 */
static TrajectoryOutput_t calculate_trapezoidal(TrajectoryGenerator_t *gen)
{
    TrajectoryOutput_t output = {0};
    float t = gen->current_time;
    float direction = (gen->config.target_position > gen->initial_position) ? 1.0f : -1.0f;
    float max_vel = gen->config.max_velocity;
    float max_accel = gen->config.max_acceleration;
    
    if (t <= gen->ta) {
        /* 加速段 */
        output.acceleration = max_accel * direction;
        output.velocity = max_accel * t * direction;
        output.position = gen->initial_position + 0.5f * max_accel * t * t * direction;
        output.state = TRAJ_STATE_ACCELERATING;
    } else if (t <= gen->ta + gen->tc) {
        /* 匀速段 */
        float t_cruise = t - gen->ta;
        output.acceleration = 0.0f;
        output.velocity = max_vel * direction;
        output.position = gen->initial_position + 
                         (0.5f * max_accel * gen->ta * gen->ta + max_vel * t_cruise) * direction;
        output.state = TRAJ_STATE_CRUISING;
    } else if (t <= gen->total_time) {
        /* 减速段 */
        float t_decel = t - gen->ta - gen->tc;
        output.acceleration = -max_accel * direction;
        output.velocity = (max_vel - max_accel * t_decel) * direction;
        
        float accel_dist = 0.5f * max_accel * gen->ta * gen->ta;
        float cruise_dist = max_vel * gen->tc;
        float decel_dist = max_vel * t_decel - 0.5f * max_accel * t_decel * t_decel;
        
        output.position = gen->initial_position + (accel_dist + cruise_dist + decel_dist) * direction;
        output.state = TRAJ_STATE_DECELERATING;
    } else {
        /* 完成 */
        output.position = gen->config.target_position;
        output.velocity = 0.0f;
        output.acceleration = 0.0f;
        output.is_complete = 1;
        output.state = TRAJ_STATE_IDLE;
    }
    
    return output;
}

/* S曲线计算(简化版) */
static TrajectoryOutput_t calculate_scurve(TrajectoryGenerator_t *gen)
{
    TrajectoryOutput_t output = {0};
    float t = gen->current_time;
    float direction = (gen->config.target_position > gen->initial_position) ? 1.0f : -1.0f;
    
    if (t >= gen->total_time) {
        output.position = gen->config.target_position;
        output.velocity = 0.0f;
        output.acceleration = 0.0f;
        output.is_complete = 1;
        output.state = TRAJ_STATE_IDLE;
        return output;
    }
    
    /* 使用梯形曲线近似S曲线(完整S曲线实现较复杂) */
    float normalized_time = t / gen->total_time;
    
    /* 五次多项式平滑过渡 */
    float s = 10.0f * powf(normalized_time, 3) - 
              15.0f * powf(normalized_time, 4) + 
              6.0f * powf(normalized_time, 5);
    
    float s_dot = (30.0f * powf(normalized_time, 2) - 
                   60.0f * powf(normalized_time, 3) + 
                   30.0f * powf(normalized_time, 4)) / gen->total_time;
    
    float s_ddot = (60.0f * normalized_time - 
                    180.0f * powf(normalized_time, 2) + 
                    120.0f * powf(normalized_time, 3)) / (gen->total_time * gen->total_time);
    
    float distance = gen->config.target_position - gen->initial_position;
    
    output.position = gen->initial_position + distance * s;
    output.velocity = distance * s_dot;
    output.acceleration = distance * s_ddot;
    output.jerk = 0.0f; /* 简化处理 */
    
    /* 状态判断 */
    if (normalized_time < 0.2f) {
        output.state = TRAJ_STATE_JERK_ACCEL;
    } else if (normalized_time < 0.5f) {
        output.state = TRAJ_STATE_ACCELERATING;
    } else if (normalized_time < 0.8f) {
        output.state = TRAJ_STATE_DECELERATING;
    } else {
        output.state = TRAJ_STATE_JERK_DECEL;
    }
    
    return output;
}

/* 五次多项式计算 */
static TrajectoryOutput_t calculate_polynomial_5th(TrajectoryGenerator_t *gen)
{
    TrajectoryOutput_t output = {0};
    float t = gen->current_time;
    float T = gen->total_time;
    
    if (t >= T) {
        output.position = gen->config.target_position;
        output.velocity = 0.0f;
        output.acceleration = 0.0f;
        output.is_complete = 1;
        output.state = TRAJ_STATE_IDLE;
        return output;
    }
    
    float p0 = gen->initial_position;
    float p1 = gen->config.target_position;
    float v0 = gen->config.max_velocity;     /* 起始速度 */
    float v1 = gen->config.max_acceleration; /* 目标速度 */
    float a0 = gen->config.max_jerk;         /* 起始加速度 */
    float a1 = gen->config.jerk_time;        /* 目标加速度 */
    
    /* 五次多项式系数求解 */
    float c0 = p0;
    float c1 = v0;
    float c2 = a0 / 2.0f;
    float c3 = (20.0f*(p1-p0) - (8.0f*v1+12.0f*v0)*T - (3.0f*a0-a1)*T*T) / (2.0f*T*T*T);
    float c4 = (-30.0f*(p1-p0) + (14.0f*v1+16.0f*v0)*T + (3.0f*a0-2.0f*a1)*T*T) / (2.0f*T*T*T*T);
    float c5 = (12.0f*(p1-p0) - 6.0f*(v1+v0)*T + (a1-a0)*T*T) / (2.0f*T*T*T*T*T);
    
    output.position = c0 + c1*t + c2*t*t + c3*t*t*t + c4*t*t*t*t + c5*t*t*t*t*t;
    output.velocity = c1 + 2*c2*t + 3*c3*t*t + 4*c4*t*t*t + 5*c5*t*t*t*t;
    output.acceleration = 2*c2 + 6*c3*t + 12*c4*t*t + 20*c5*t*t*t;
    output.jerk = 6*c3 + 24*c4*t + 60*c5*t*t;
    output.state = TRAJ_STATE_CRUISING;
    
    return output;
}

TrajectoryOutput_t Trajectory_Calculate(TrajectoryGenerator_t *gen)
{
    TrajectoryOutput_t output = {0};
    
    if (!gen->is_initialized) {
        output.is_complete = 1;
        return output;
    }
    
    switch (gen->config.type) {
        case TRAJ_TRAPEZOIDAL:
            output = calculate_trapezoidal(gen);
            break;
        case TRAJ_S_CURVE:
            output = calculate_scurve(gen);
            break;
        case TRAJ_POLYNOMIAL_5TH:
            output = calculate_polynomial_5th(gen);
            break;
        default:
            output = calculate_trapezoidal(gen);
            break;
    }
    
    /* 计算进度 */
    if (gen->total_time > 0) {
        output.progress = clampf(gen->current_time / gen->total_time, 0.0f, 1.0f);
    } else {
        output.progress = 1.0f;
    }
    
    /* 更新时间 */
    gen->current_time += gen->dt;
    
    return output;
}

void Trajectory_Reset(TrajectoryGenerator_t *gen)
{
    gen->current_time = 0.0f;
}

uint8_t Trajectory_IsComplete(const TrajectoryGenerator_t *gen)
{
    return (gen->current_time >= gen->total_time) ? 1 : 0;
}

float Trajectory_GetTotalTime(const TrajectoryGenerator_t *gen)
{
    return gen->total_time;
}

float Trajectory_GetProgress(const TrajectoryGenerator_t *gen)
{
    if (gen->total_time <= 0) return 1.0f;
    return clampf(gen->current_time / gen->total_time, 0.0f, 1.0f);
}