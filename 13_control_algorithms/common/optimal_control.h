/**
 * @file optimal_control.h
 * @brief 最优控制器 (动态规划简化版)
 *
 * 基于离散动态规划(Discrete Dynamic Programming)的最优控制器,
 * 适用于有限状态/控制空间的离散最优控制问题。
 *
 * 核心思想:
 *   对于离散系统 x(k+1) = f(x(k), u(k))
 *   最小化代价函数 J = Σ [l(x(k), u(k))] + l_f(x(N))
 *   通过Bellman方程逆向递推求解最优控制策略:
 *   V*(x,k) = min_u { l(x,u) + V*(f(x,u), k+1) }
 *
 * 适用场景:
 *   - 离散状态空间的最优决策
 *   - 小车路径规划
 *   - 能量最优控制
 *   - 时间最优控制
 *   - 嵌入式资源受限环境下的离线策略计算
 */

#ifndef OPTIMAL_CONTROL_H
#define OPTIMAL_CONTROL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 状态/控制空间网格大小限制 */
#define OC_MAX_STATES       64      /* 最大状态网格数 */
#define OC_MAX_CONTROLS     32      /* 最大控制网格数 */
#define OC_MAX_DIM          4       /* 最大状态维度 */
#define OC_MAX_STEPS        50      /* 最大时间步数 */
#define OC_INFINITE_COST    1e12f

/**
 * @brief 状态转移函数 x' = f(x, u)
 * @param x      当前状态 [dim]
 * @param u      控制输入 [u_dim]
 * @param x_next 下一状态输出 [dim]
 * @param dim    状态维度
 * @param u_dim  控制维度
 * @param dt     时间步长
 * @param params 用户参数
 */
typedef void (*oc_dynamics_t)(const float *x, const float *u, float *x_next,
                               uint8_t dim, uint8_t u_dim, float dt, void *params);

/**
 * @brief 阶段代价函数 l(x, u)
 * @param x      当前状态
 * @param u      控制输入
 * @param dim    状态维度
 * @param u_dim  控制维度
 * @param params 用户参数
 * @return 代价值(非负)
 */
typedef float (*oc_stage_cost_t)(const float *x, const float *u,
                                  uint8_t dim, uint8_t u_dim, void *params);

/**
 * @brief 终端代价函数 l_f(x)
 */
typedef float (*oc_terminal_cost_t)(const float *x, uint8_t dim, void *params);

/**
 * @brief 最优控制器配置
 */
typedef struct {
    uint8_t dim;                        /* 状态维度 */
    uint8_t u_dim;                      /* 控制维度 (通常=1) */
    uint16_t N;                         /* 时间步数(规划时域) */
    float dt;                           /* 时间步长 */

    oc_dynamics_t dynamics;             /* 状态转移函数 */
    oc_stage_cost_t stage_cost;         /* 阶段代价 */
    oc_terminal_cost_t terminal_cost;   /* 终端代价 */

    /* 状态空间网格 [dim][grid_size[i]] */
    uint8_t grid_size[OC_MAX_DIM];      /* 各维度网格数 */
    float grid_min[OC_MAX_DIM];         /* 各维度下界 */
    float grid_max[OC_MAX_DIM];         /* 各维度上界 */

    /* 控制空间网格 */
    uint8_t u_grid_size;                /* 控制网格数 */
    float u_min;                        /* 控制下界 */
    float u_max;                        /* 控制上界 */

    void *params;                       /* 传递给代价/动力学函数的参数 */
} oc_config_t;

/**
 * @brief 最优控制器状态
 */
typedef struct {
    oc_config_t config;

    /* 值函数表 V[k][grid_index]
       grid_index = i0 + i1*grid_size[0] + i2*grid_size[0]*grid_size[1] + ... */
    float *value_table[OC_MAX_STEPS + 1];

    /* 最优控制策略 u*[k][grid_index] */
    float *policy_table[OC_MAX_STEPS];

    /* 网格坐标工作空间 */
    float grid_points[OC_MAX_DIM][OC_MAX_STATES];

    uint8_t solved;                     /* 是否已求解 */
    uint32_t total_grid_points;         /* 总网格点数 */
} oc_controller_t;

/**
 * @brief 初始化最优控制器
 * @param ctrl   控制器指针
 * @param config 配置
 * @return 0=成功, -1=错误
 */
int oc_init(oc_controller_t *ctrl, const oc_config_t *config);

/**
 * @brief 释放控制器资源
 */
void oc_destroy(oc_controller_t *ctrl);

/**
 * @brief 求解最优控制问题 (逆向动态规划)
 *        从终端时间N逆向递推到初始时间0
 * @param ctrl 控制器
 * @return 0=成功
 */
int oc_solve(oc_controller_t *ctrl);

/**
 * @brief 根据当前状态查询最优控制
 *        使用多线性插值从策略表中查值
 * @param ctrl 控制器
 * @param x    当前状态 [dim]
 * @param step 当前时间步 (0~N-1)
 * @return 最优控制量
 */
float oc_get_control(oc_controller_t *ctrl, const float *x, uint16_t step);

/**
 * @brief 查询某状态的最优值函数
 */
float oc_get_value(oc_controller_t *ctrl, const float *x, uint16_t step);

/**
 * @brief 前向仿真: 从初始状态开始,使用最优策略驱动系统
 * @param ctrl      控制器
 * @param x0        初始状态 [dim]
 * @param x_traj    输出状态轨迹 [N+1][dim]
 * @param u_traj    输出控制轨迹 [N]
 * @param cost_out  输出总代价
 * @return 0=成功
 */
int oc_simulate(oc_controller_t *ctrl, const float *x0,
                 float x_traj[][OC_MAX_DIM], float u_traj[], float *cost_out);

/* ====== 常用预置代价函数 ====== */

/**
 * @brief 二次代价 l(x,u) = x'Qx + u'Ru
 */
typedef struct {
    float Q[OC_MAX_DIM];    /* 状态权重 (对角矩阵) */
    float R;                /* 控制权重 (标量) */
    float x_ref[OC_MAX_DIM]; /* 参考状态 */
} oc_quadratic_cost_params_t;

float oc_quadratic_stage_cost(const float *x, const float *u,
                               uint8_t dim, uint8_t u_dim, void *params);
float oc_quadratic_terminal_cost(const float *x, uint8_t dim, void *params);

/**
 * @brief 时间最优代价 l(x,u) = dt, l_f = 0
 */
typedef struct {
    float x_target[OC_MAX_DIM];
    float tolerance;         /* 目标容差 */
    float penalty;           /* 未到达目标的惩罚 */
} oc_time_optimal_params_t;

float oc_time_optimal_stage_cost(const float *x, const float *u,
                                  uint8_t dim, uint8_t u_dim, void *params);
float oc_time_optimal_terminal_cost(const float *x, uint8_t dim, void *params);

/* ====== 常用预置动力学模型 ====== */

/**
 * @brief 双积分器 x' = [x₁ + x₂*dt, x₂ + u*dt]
 */
typedef struct {
    float dt_override;  /* 0表示使用config中的dt */
} oc_double_integrator_params_t;

void oc_double_integrator_dynamics(const float *x, const float *u, float *x_next,
                                    uint8_t dim, uint8_t u_dim, float dt, void *params);

/**
 * @brief 离散一阶系统 x' = a*x + b*u
 */
typedef struct {
    float a;
    float b;
} oc_discrete_first_order_params_t;

void oc_discrete_first_order_dynamics(const float *x, const float *u, float *x_next,
                                       uint8_t dim, uint8_t u_dim, float dt, void *params);

#ifdef __cplusplus
}
#endif

#endif /* OPTIMAL_CONTROL_H */
