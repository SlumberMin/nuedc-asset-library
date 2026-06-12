#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 轨迹类型枚举
 */
typedef enum {
    TRAJ_TRAPEZOIDAL,           // 梯形速度曲线
    TRAJ_S_CURVE,               // S曲线(7段式)
    TRAJ_POLYNOMIAL_3RD,        // 三次多项式
    TRAJ_POLYNOMIAL_5TH         // 五次多项式
} TrajectoryType_t;

/**
 * @brief 轨迹状态枚举
 */
typedef enum {
    TRAJ_STATE_IDLE,            // 空闲
    TRAJ_STATE_ACCELERATING,    // 加速段
    TRAJ_STATE_CRUISING,        // 匀速段
    TRAJ_STATE_DECELERATING,    // 减速段
    TRAJ_STATE_JERK_ACCEL,      // S曲线：加加速度段
    TRAJ_STATE_JERK_DECEL       // S曲线：减加速度段
} TrajectoryState_t;

/**
 * @brief 轨迹生成器配置结构体
 */
typedef struct {
    TrajectoryType_t type;      // 轨迹类型
    
    /* 运动参数 */
    float target_position;      // 目标位置
    float max_velocity;         // 最大速度
    float max_acceleration;     // 最大加速度
    float max_jerk;             // 最大加加速度(S曲线专用)
    
    /* S曲线专用参数 */
    float jerk_time;            // 加加速度段时间
    float accel_time;           // 匀加速段时间
    float cruise_time;          // 匀速段时间
} TrajectoryConfig_t;

/**
 * @brief 轨迹输出结构体
 */
typedef struct {
    float position;             // 当前目标位置
    float velocity;             // 当前目标速度
    float acceleration;         // 当前目标加速度
    float jerk;                 // 当前目标加加速度
    TrajectoryState_t state;    // 当前轨迹状态
    float progress;             // 进度(0-1)
    uint8_t is_complete;        // 是否完成
} TrajectoryOutput_t;

/**
 * @brief 轨迹生成器主结构体
 */
typedef struct {
    TrajectoryConfig_t config;  // 配置参数
    
    /* 内部状态 */
    float initial_position;     // 起始位置
    float current_time;         // 当前时间
    float total_time;           // 总运动时间
    
    /* 梯形曲线参数 */
    float ta;                   // 加速段时间
    float tc;                   // 匀速段时间
    float td;                   // 减速段时间
    
    /* S曲线参数 */
    float t_jerk;               // 加加速度段时间
    float t_accel_const;        // 匀加速段时间
    float t_cruise_const;       // 匀速段时间
    float t_decel_const;        // 匀减速段时间
    float t_jerk_decel;         // 减加速度段时间
    
    /* 采样参数 */
    float dt;                   // 采样周期(秒)
    uint8_t is_initialized;     // 是否已初始化
} TrajectoryGenerator_t;

/**
 * @brief 初始化轨迹生成器
 * @param gen 轨迹生成器指针
 * @param dt 采样周期(秒)
 */
void Trajectory_Init(TrajectoryGenerator_t *gen, float dt);

/**
 * @brief 配置梯形轨迹
 * @param gen 轨迹生成器指针
 * @param start_pos 起始位置
 * @param target_pos 目标位置
 * @param max_vel 最大速度
 * @param max_accel 最大加速度
 */
void Trajectory_SetTrapezoidal(TrajectoryGenerator_t *gen, 
                               float start_pos, float target_pos,
                               float max_vel, float max_accel);

/**
 * @brief 配置S曲线轨迹
 * @param gen 轨迹生成器指针
 * @param start_pos 起始位置
 * @param target_pos 目标位置
 * @param max_vel 最大速度
 * @param max_accel 最大加速度
 * @param max_jerk 最大加加速度
 */
void Trajectory_SetSCurve(TrajectoryGenerator_t *gen,
                          float start_pos, float target_pos,
                          float max_vel, float max_accel, float max_jerk);

/**
 * @brief 配置多项式轨迹
 * @param gen 轨迹生成器指针
 * @param start_pos 起始位置
 * @param target_pos 目标位置
 * @param start_vel 起始速度
 * @param target_vel 目标速度
 * @param start_accel 起始加速度
 * @param target_accel 目标加速度
 * @param total_time 总运动时间
 * @param type 轨迹类型(POLYNOMIAL_3RD 或 POLYNOMIAL_5TH)
 */
void Trajectory_SetPolynomial(TrajectoryGenerator_t *gen,
                              float start_pos, float target_pos,
                              float start_vel, float target_vel,
                              float start_accel, float target_accel,
                              float total_time, TrajectoryType_t type);

/**
 * @brief 计算下一时刻的轨迹点
 * @param gen 轨迹生成器指针
 * @return 轨迹输出(位置、速度、加速度等)
 */
TrajectoryOutput_t Trajectory_Calculate(TrajectoryGenerator_t *gen);

/**
 * @brief 重置轨迹生成器
 * @param gen 轨迹生成器指针
 */
void Trajectory_Reset(TrajectoryGenerator_t *gen);

/**
 * @brief 检查轨迹是否完成
 * @param gen 轨迹生成器指针
 * @return 1表示完成，0表示进行中
 */
uint8_t Trajectory_IsComplete(const TrajectoryGenerator_t *gen);

/**
 * @brief 获取轨迹总时长
 * @param gen 轨迹生成器指针
 * @return 总时长(秒)
 */
float Trajectory_GetTotalTime(const TrajectoryGenerator_t *gen);

/**
 * @brief 获取当前进度
 * @param gen 轨迹生成器指针
 * @return 进度(0-1)
 */
float Trajectory_GetProgress(const TrajectoryGenerator_t *gen);

#ifdef __cplusplus
}
#endif

#endif /* TRAJECTORY_GENERATOR_H */