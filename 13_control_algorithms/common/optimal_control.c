/**
 * @file optimal_control.c
 * @brief 最优控制器实现 (动态规划简化版)
 *
 * 使用表格型动态规划(Tabular Dynamic Programming)求解
 * 离散时间有限状态/控制空间的最优控制问题。
 */

#include "optimal_control.h"
#include <stdlib.h>
#include <math.h>
#include <string.h>

/* ========== 内部辅助函数 ========== */

/**
 * @brief 计算总网格点数
 */
static uint32_t compute_total_grid(const oc_config_t *cfg)
{
    uint32_t total = 1;
    for (uint8_t d = 0; d < cfg->dim; d++) {
        if (cfg->grid_size[d] == 0) return 0;
        total *= cfg->grid_size[d];
    }
    return total;
}

/**
 * @brief 将多维网格索引转为线性索引
 */
static uint32_t grid_index(const uint8_t *indices, const uint8_t *grid_size, uint8_t dim)
{
    uint32_t idx = 0;
    uint32_t stride = 1;
    for (uint8_t d = 0; d < dim; d++) {
        idx += indices[d] * stride;
        stride *= grid_size[d];
    }
    return idx;
}

/**
 * @brief 线性索引转多维网格索引
 */
static void grid_unindex(uint32_t idx, const uint8_t *grid_size, uint8_t dim, uint8_t *indices)
{
    for (uint8_t d = 0; d < dim; d++) {
        indices[d] = idx % grid_size[d];
        idx /= grid_size[d];
    }
}

/**
 * @brief 从网格索引获取实际状态值
 */
static void get_state(const oc_controller_t *ctrl, const uint8_t *indices, float *x)
{
    for (uint8_t d = 0; d < ctrl->config.dim; d++) {
        x[d] = ctrl->grid_points[d][indices[d]];
    }
}

/**
 * @brief 从控制索引获取实际控制值
 */
static float get_control(const oc_controller_t *ctrl, uint8_t u_idx)
{
    const oc_config_t *cfg = &ctrl->config;
    if (cfg->u_grid_size <= 1) return cfg->u_min;
    float step = (cfg->u_max - cfg->u_min) / (float)(cfg->u_grid_size - 1);
    return cfg->u_min + u_idx * step;
}

/**
 * @brief 多线性插值: 在网格上查询值函数
 *        支持1D~4D
 */
static float interpolate_value(const oc_controller_t *ctrl, const float *x, uint16_t step)
{
    const oc_config_t *cfg = &ctrl->config;
    const uint8_t dim = cfg->dim;

    if (dim == 1) {
        /* 1D线性插值 */
        float x0 = x[0];
        if (x0 <= ctrl->grid_points[0][0]) return ctrl->value_table[step][0];
        if (x0 >= ctrl->grid_points[0][cfg->grid_size[0] - 1])
            return ctrl->value_table[step][cfg->grid_size[0] - 1];

        /* 找到区间 */
        for (uint8_t i = 0; i < cfg->grid_size[0] - 1; i++) {
            if (x0 >= ctrl->grid_points[0][i] && x0 <= ctrl->grid_points[0][i + 1]) {
                float alpha = (x0 - ctrl->grid_points[0][i])
                            / (ctrl->grid_points[0][i + 1] - ctrl->grid_points[0][i]);
                return ctrl->value_table[step][i] * (1.0f - alpha)
                     + ctrl->value_table[step][i + 1] * alpha;
            }
        }
        return ctrl->value_table[step][0];

    } else if (dim == 2) {
        /* 2D双线性插值 */
        float x0 = x[0], x1 = x[1];

        /* Clamp */
        x0 = fmaxf(ctrl->grid_points[0][0], fminf(x0, ctrl->grid_points[0][cfg->grid_size[0] - 1]));
        x1 = fmaxf(ctrl->grid_points[1][0], fminf(x1, ctrl->grid_points[1][cfg->grid_size[1] - 1]));

        /* 找到网格区间 */
        uint8_t i0 = 0, i1 = 0;
        for (uint8_t i = 0; i < cfg->grid_size[0] - 1; i++) {
            if (x0 <= ctrl->grid_points[0][i + 1]) { i0 = i; break; }
        }
        for (uint8_t i = 0; i < cfg->grid_size[1] - 1; i++) {
            if (x1 <= ctrl->grid_points[1][i + 1]) { i1 = i; break; }
        }

        float dx0 = ctrl->grid_points[0][i0 + 1] - ctrl->grid_points[0][i0];
        float dx1 = ctrl->grid_points[1][i1 + 1] - ctrl->grid_points[1][i1];

        float a0 = (dx0 > 1e-10f) ? (x0 - ctrl->grid_points[0][i0]) / dx0 : 0.0f;
        float a1 = (dx1 > 1e-10f) ? (x1 - ctrl->grid_points[1][i1]) / dx1 : 0.0f;

        a0 = fmaxf(0.0f, fminf(a0, 1.0f));
        a1 = fmaxf(0.0f, fminf(a1, 1.0f));

        /* 四个角点的值 */
        uint8_t idx[2];
        float v[2][2];
        for (uint8_t di = 0; di < 2; di++) {
            for (uint8_t dj = 0; dj < 2; dj++) {
                idx[0] = i0 + di;
                idx[1] = i1 + dj;
                uint32_t li = grid_index(idx, cfg->grid_size, dim);
                v[di][dj] = ctrl->value_table[step][li];
            }
        }

        float v0 = v[0][0] * (1 - a1) + v[0][1] * a1;
        float v1 = v[1][0] * (1 - a1) + v[1][1] * a1;
        return v0 * (1 - a0) + v1 * a0;

    } else {
        /* 通用: 最近邻插值(简化) */
        uint8_t indices[OC_MAX_DIM];
        for (uint8_t d = 0; d < dim; d++) {
            float xd = x[d];
            xd = fmaxf(ctrl->grid_points[d][0], fminf(xd, ctrl->grid_points[d][cfg->grid_size[d] - 1]));
            indices[d] = 0;
            for (uint8_t i = 0; i < cfg->grid_size[d] - 1; i++) {
                if (xd <= ctrl->grid_points[d][i + 1]) {
                    indices[d] = (xd - ctrl->grid_points[d][i]
                                < ctrl->grid_points[d][i + 1] - xd) ? i : (i + 1);
                    break;
                }
            }
        }
        uint32_t li = grid_index(indices, cfg->grid_size, dim);
        return ctrl->value_table[step][li];
    }
}

/**
 * @brief 多线性插值查询策略(最优控制)
 */
static float interpolate_policy(const oc_controller_t *ctrl, const float *x, uint16_t step)
{
    /* 与值函数插值类似,这里简化为最近邻 */
    const oc_config_t *cfg = &ctrl->config;
    const uint8_t dim = cfg->dim;
    uint8_t indices[OC_MAX_DIM];

    for (uint8_t d = 0; d < dim; d++) {
        float xd = fmaxf(ctrl->grid_points[d][0], fminf(x[d], ctrl->grid_points[d][cfg->grid_size[d] - 1]));
        indices[d] = 0;
        float best_dist = fabsf(xd - ctrl->grid_points[d][0]);
        for (uint8_t i = 1; i < cfg->grid_size[d]; i++) {
            float dist = fabsf(xd - ctrl->grid_points[d][i]);
            if (dist < best_dist) {
                best_dist = dist;
                indices[d] = i;
            }
        }
    }

    uint32_t li = grid_index(indices, cfg->grid_size, dim);
    return ctrl->policy_table[step][li];
}

/* ========== 公共接口实现 ========== */

int oc_init(oc_controller_t *ctrl, const oc_config_t *config)
{
    if (!ctrl || !config) return -1;
    if (config->dim == 0 || config->dim > OC_MAX_DIM) return -1;
    if (config->N == 0 || config->N > OC_MAX_STEPS) return -1;
    if (!config->dynamics || !config->stage_cost) return -1;

    memset(ctrl, 0, sizeof(oc_controller_t));
    ctrl->config = *config;

    /* 生成网格点 */
    for (uint8_t d = 0; d < config->dim; d++) {
        if (config->grid_size[d] < 2) return -1;
        float step = (config->grid_max[d] - config->grid_min[d]) / (float)(config->grid_size[d] - 1);
        for (uint8_t i = 0; i < config->grid_size[d]; i++) {
            ctrl->grid_points[d][i] = config->grid_min[d] + i * step;
        }
    }

    /* 计算总网格点数 */
    ctrl->total_grid_points = compute_total_grid(config);
    if (ctrl->total_grid_points == 0 || ctrl->total_grid_points > OC_MAX_STATES * OC_MAX_STATES * OC_MAX_STATES) {
        return -1;
    }

    /* 分配值函数表和策略表 */
    for (uint16_t k = 0; k <= config->N; k++) {
        ctrl->value_table[k] = (float *)calloc(ctrl->total_grid_points, sizeof(float));
        if (!ctrl->value_table[k]) {
            oc_destroy(ctrl);
            return -1;
        }
        if (k < config->N) {
            ctrl->policy_table[k] = (float *)calloc(ctrl->total_grid_points, sizeof(float));
            if (!ctrl->policy_table[k]) {
                oc_destroy(ctrl);
                return -1;
            }
        }
    }

    return 0;
}

void oc_destroy(oc_controller_t *ctrl)
{
    if (!ctrl) return;
    for (uint16_t k = 0; k <= ctrl->config.N; k++) {
        if (ctrl->value_table[k]) {
            free(ctrl->value_table[k]);
            ctrl->value_table[k] = NULL;
        }
        if (k < ctrl->config.N && ctrl->policy_table[k]) {
            free(ctrl->policy_table[k]);
            ctrl->policy_table[k] = NULL;
        }
    }
    ctrl->solved = 0;
}

int oc_solve(oc_controller_t *ctrl)
{
    if (!ctrl) return -1;
    const oc_config_t *cfg = &ctrl->config;
    const uint8_t dim = cfg->dim;
    const uint8_t u_dim = cfg->u_dim;
    const uint16_t N = cfg->N;

    /* Step 1: 终端代价 V[N][i] = l_f(x_i) */
    uint8_t indices[OC_MAX_DIM];
    float x[OC_MAX_DIM];

    for (uint32_t idx = 0; idx < ctrl->total_grid_points; idx++) {
        grid_unindex(idx, cfg->grid_size, dim, indices);
        get_state(ctrl, indices, x);

        if (cfg->terminal_cost) {
            ctrl->value_table[N][idx] = cfg->terminal_cost(x, dim, cfg->params);
        } else {
            ctrl->value_table[N][idx] = 0.0f;
        }
    }

    /* Step 2: 逆向递推 k = N-1, N-2, ..., 0 */
    for (int16_t k = N - 1; k >= 0; k--) {
        for (uint32_t idx = 0; idx < ctrl->total_grid_points; idx++) {
            grid_unindex(idx, cfg->grid_size, dim, indices);
            get_state(ctrl, indices, x);

            float min_cost = OC_INFINITE_COST;
            float best_u = 0.0f;

            /* 枚举所有控制量 */
            for (uint8_t ui = 0; ui < cfg->u_grid_size; ui++) {
                float u = get_control(ctrl, ui);

                /* 状态转移 */
                float x_next[OC_MAX_DIM];
                cfg->dynamics(x, &u, x_next, dim, u_dim, cfg->dt, cfg->params);

                /* 阶段代价 */
                float stage = cfg->stage_cost(x, &u, dim, u_dim, cfg->params);

                /* 下一时刻值函数 (插值) */
                float V_next = interpolate_value(ctrl, x_next, k + 1);

                float total_cost = stage + V_next;

                if (total_cost < min_cost) {
                    min_cost = total_cost;
                    best_u = u;
                }
            }

            ctrl->value_table[k][idx] = min_cost;
            ctrl->policy_table[k][idx] = best_u;
        }
    }

    ctrl->solved = 1;
    return 0;
}

float oc_get_control(oc_controller_t *ctrl, const float *x, uint16_t step)
{
    if (!ctrl || !ctrl->solved) return 0.0f;
    if (step >= ctrl->config.N) return 0.0f;
    return interpolate_policy(ctrl, x, step);
}

float oc_get_value(oc_controller_t *ctrl, const float *x, uint16_t step)
{
    if (!ctrl || !ctrl->solved) return 0.0f;
    if (step > ctrl->config.N) return 0.0f;
    return interpolate_value(ctrl, x, step);
}

int oc_simulate(oc_controller_t *ctrl, const float *x0,
                 float x_traj[][OC_MAX_DIM], float u_traj[], float *cost_out)
{
    if (!ctrl || !ctrl->solved || !x0) return -1;

    const oc_config_t *cfg = &ctrl->config;
    const uint8_t dim = cfg->dim;
    const uint16_t N = cfg->N;
    float total_cost = 0.0f;

    /* 初始状态 */
    for (uint8_t d = 0; d < dim; d++) {
        x_traj[0][d] = x0[d];
    }

    for (uint16_t k = 0; k < N; k++) {
        /* 查询最优控制 */
        float u = oc_get_control(ctrl, x_traj[k], k);
        u_traj[k] = u;

        /* 累积代价 */
        float stage = cfg->stage_cost(x_traj[k], &u, dim, cfg->u_dim, cfg->params);
        total_cost += stage;

        /* 状态转移 */
        cfg->dynamics(x_traj[k], &u, x_traj[k + 1], dim, cfg->u_dim, cfg->dt, cfg->params);
    }

    /* 加上终端代价 */
    if (cfg->terminal_cost) {
        total_cost += cfg->terminal_cost(x_traj[N], dim, cfg->params);
    }

    if (cost_out) *cost_out = total_cost;
    return 0;
}

/* ========== 预置代价函数 ========== */

float oc_quadratic_stage_cost(const float *x, const float *u,
                               uint8_t dim, uint8_t u_dim, void *params)
{
    oc_quadratic_cost_params_t *p = (oc_quadratic_cost_params_t *)params;
    if (!p) return 0.0f;

    float cost = 0.0f;
    for (uint8_t d = 0; d < dim; d++) {
        float err = x[d] - p->x_ref[d];
        cost += p->Q[d] * err * err;
    }
    for (uint8_t d = 0; d < u_dim; d++) {
        cost += p->R * u[d] * u[d];
    }
    return cost;
}

float oc_quadratic_terminal_cost(const float *x, uint8_t dim, void *params)
{
    oc_quadratic_cost_params_t *p = (oc_quadratic_cost_params_t *)params;
    if (!p) return 0.0f;

    /* 终端代价为阶段代价的N倍(加大终端惩罚) */
    float cost = 0.0f;
    for (uint8_t d = 0; d < dim; d++) {
        float err = x[d] - p->x_ref[d];
        cost += p->Q[d] * 10.0f * err * err;
    }
    return cost;
}

float oc_time_optimal_stage_cost(const float *x, const float *u,
                                  uint8_t dim, uint8_t u_dim, void *params)
{
    (void)x; (void)u; (void)dim; (void)u_dim; (void)params;
    return 1.0f; /* 每步代价为1,最小化步数即时间最优 */
}

float oc_time_optimal_terminal_cost(const float *x, uint8_t dim, void *params)
{
    oc_time_optimal_params_t *p = (oc_time_optimal_params_t *)params;
    if (!p) return 0.0f;

    /* 检查是否到达目标 */
    float dist = 0.0f;
    for (uint8_t d = 0; d < dim; d++) {
        float err = x[d] - p->x_target[d];
        dist += err * err;
    }
    dist = sqrtf(dist);

    if (dist <= p->tolerance) {
        return 0.0f; /* 到达目标,无终端代价 */
    } else {
        return p->penalty; /* 未到达,大惩罚 */
    }
}

/* ========== 预置动力学模型 ========== */

void oc_double_integrator_dynamics(const float *x, const float *u, float *x_next,
                                    uint8_t dim, uint8_t u_dim, float dt, void *params)
{
    (void)u_dim;
    oc_double_integrator_params_t *p = (oc_double_integrator_params_t *)params;
    float h = (p && p->dt_override > 0) ? p->dt_override : dt;

    /* 双积分器: x₁' = x₁ + x₂*dt, x₂' = x₂ + u*dt */
    x_next[0] = x[0] + x[1] * h;
    x_next[1] = x[1] + u[0] * h;

    (void)dim;
}

void oc_discrete_first_order_dynamics(const float *x, const float *u, float *x_next,
                                       uint8_t dim, uint8_t u_dim, float dt, void *params)
{
    oc_discrete_first_order_params_t *p = (oc_discrete_first_order_params_t *)params;
    (void)dim; (void)u_dim; (void)dt;

    x_next[0] = p->a * x[0] + p->b * u[0];
}
