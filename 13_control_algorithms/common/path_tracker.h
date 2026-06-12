#ifndef PATH_TRACKER_H
#define PATH_TRACKER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 路径跟踪模式 */
typedef enum {
    PT_MODE_PURE_PURSUIT = 0,   /* Pure Pursuit 几何追踪 */
    PT_MODE_STANLEY     = 1    /* Stanley 控制器(预留) */
} pt_mode_t;

/* 2D路径点 */
typedef struct {
    float x;
    float y;
} pt_waypoint_t;

/* 车辆状态 */
typedef struct {
    float x;        /* 位置x(m) */
    float y;        /* 位置y(m) */
    float yaw;      /* 航向角(rad) */
    float v;        /* 纵向速度(m/s) */
} pt_state_t;

/* 路径跟踪控制输出 */
typedef struct {
    float steering_angle;   /* 转向角(rad) */
    float target_speed;     /* 目标速度(m/s) */
    float curvature;        /* 期望曲率(1/m) */
    float cross_track_err;  /* 横向误差(m) */
    float heading_err;      /* 航向误差(rad) */
    float lookahead_dist;   /* 前视距离(m) */
    uint16_t target_idx;    /* 目标点索引 */
} pt_output_t;

/* Pure Pursuit 配置 */
typedef struct {
    float lookahead_base;      /* 基础前视距离(m) */
    float lookahead_speed_coeff;/* 前视距离速度系数 */
    float wheelbase;           /* 轴距(m) */
    float max_steering;        /* 最大转向角(rad) */
    float target_speed;        /* 目标巡航速度(m/s) */
    float max_accel;           /* 最大加速度(m/s^2) */
} pt_pp_config_t;

/* 路径跟踪控制器 */
typedef struct {
    pt_mode_t mode;
    pt_pp_config_t pp_cfg;
    pt_output_t output;
    /* 路径数据(外部管理或内部拷贝) */
    const pt_waypoint_t *path;
    uint16_t path_len;
    uint16_t last_nearest_idx;  /* 上次最近点索引(加速搜索) */
} pt_tracker_t;

/**
 * @brief 初始化Pure Pursuit路径跟踪器
 */
int pt_pure_pursuit_init(pt_tracker_t *tracker, const pt_pp_config_t *cfg,
                          const pt_waypoint_t *path, uint16_t path_len);

/**
 * @brief 更新路径跟踪控制
 * @param tracker 跟踪器
 * @param state   当前车辆状态
 * @param dt      时间步长(s)
 * @return 控制输出
 */
const pt_output_t *pt_update(pt_tracker_t *tracker, const pt_state_t *state, float dt);

/**
 * @brief 获取最近路径点索引
 */
uint16_t pt_find_nearest(const pt_tracker_t *tracker, const pt_state_t *state);

/**
 * @brief 计算横向跟踪误差(带符号)
 */
float pt_calc_cross_track_error(const pt_tracker_t *tracker, const pt_state_t *state);

/**
 * @brief 检查是否到达终点
 */
int pt_is_finished(const pt_tracker_t *tracker, const pt_state_t *state, float threshold);

/**
 * @brief 重置跟踪器状态
 */
void pt_reset(pt_tracker_t *tracker);

#ifdef __cplusplus
}
#endif

#endif /* PATH_TRACKER_H */
