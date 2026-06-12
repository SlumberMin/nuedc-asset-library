#include "path_tracker.h"
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static float wrap_angle(float a) {
    while (a >  M_PI) a -= 2.0f * M_PI;
    while (a < -M_PI) a += 2.0f * M_PI;
    return a;
}

static float dist2d(float x1, float y1, float x2, float y2) {
    float dx = x2 - x1, dy = y2 - y1;
    return sqrtf(dx * dx + dy * dy);
}

/* ---------- 初始化 ---------- */

int pt_pure_pursuit_init(pt_tracker_t *tracker, const pt_pp_config_t *cfg,
                          const pt_waypoint_t *path, uint16_t path_len) {
    if (!tracker || !cfg || !path || path_len < 2) return -1;
    memset(tracker, 0, sizeof(*tracker));
    tracker->mode = PT_MODE_PURE_PURSUIT;
    tracker->pp_cfg = *cfg;
    tracker->path = path;
    tracker->path_len = path_len;
    tracker->last_nearest_idx = 0;
    return 0;
}

/* ---------- 最近点搜索 ---------- */

uint16_t pt_find_nearest(const pt_tracker_t *tracker, const pt_state_t *state) {
    float min_d = 1e12f;
    uint16_t best = tracker->last_nearest_idx;
    /* 从上次索引开始搜索(路径单调前进假设) */
    uint16_t start = tracker->last_nearest_idx;
    uint16_t len = tracker->path_len;
    for (uint16_t i = 0; i < len; i++) {
        uint16_t idx = (start + i) % len;
        float d = dist2d(state->x, state->y,
                         tracker->path[idx].x, tracker->path[idx].y);
        if (d < min_d) {
            min_d = d;
            best = idx;
        }
    }
    return best;
}

/* ---------- 横向误差 ---------- */

float pt_calc_cross_track_error(const pt_tracker_t *tracker, const pt_state_t *state) {
    if (!tracker || !state || tracker->path_len < 2) return 0.0f;
    uint16_t near_idx = pt_find_nearest(tracker, state);
    uint16_t next_idx = (near_idx + 1) % tracker->path_len;

    float ax = tracker->path[next_idx].x - tracker->path[near_idx].x;
    float ay = tracker->path[next_idx].y - tracker->path[near_idx].y;
    float bx = state->x - tracker->path[near_idx].x;
    float by = state->y - tracker->path[near_idx].y;

    /* 叉积 / 路径长度 = 带符号横向误差 */
    float path_len = sqrtf(ax * ax + ay * ay);
    if (path_len < 1e-6f) return 0.0f;
    return (ax * by - ay * bx) / path_len;
}

/* ---------- Pure Pursuit 核心 ---------- */

static uint16_t find_lookahead_point(const pt_tracker_t *tracker,
                                      const pt_state_t *state,
                                      float Ld, float *out_x, float *out_y) {
    uint16_t nearest = pt_find_nearest(tracker, state);
    uint16_t len = tracker->path_len;

    /* 从最近点往后搜索第一个距离 >= Ld 的点 */
    for (uint16_t i = nearest; i < len; i++) {
        float d = dist2d(state->x, state->y,
                         tracker->path[i].x, tracker->path[i].y);
        if (d >= Ld) {
            *out_x = tracker->path[i].x;
            *out_y = tracker->path[i].y;
            return i;
        }
    }
    /* 没找到，取终点 */
    *out_x = tracker->path[len - 1].x;
    *out_y = tracker->path[len - 1].y;
    return len - 1;
}

/* ---------- 更新 ---------- */

const pt_output_t *pt_update(pt_tracker_t *tracker, const pt_state_t *state, float dt) {
    if (!tracker || !state) return NULL;
    (void)dt;
    const pt_pp_config_t *cfg = &tracker->pp_cfg;
    pt_output_t *out = &tracker->output;

    /* 1. 计算前视距离(速度自适应) */
    float Ld = cfg->lookahead_base + cfg->lookahead_speed_coeff * fabsf(state->v);
    Ld = clampf(Ld, 0.3f, 10.0f);
    out->lookahead_dist = Ld;

    /* 2. 找到前视目标点 */
    float target_x, target_y;
    out->target_idx = find_lookahead_point(tracker, state, Ld, &target_x, &target_y);

    /* 3. 将目标点转换到车辆坐标系 */
    float dx = target_x - state->x;
    float dy = target_y - state->y;
    float cos_y = cosf(state->yaw);
    float sin_y = sinf(state->yaw);
    float local_x =  cos_y * dx + sin_y * dy;
    float local_y = -sin_y * dx + cos_y * dy;

    /* 4. Pure Pursuit 曲率公式: kappa = 2*local_y / Ld^2 */
    float curvature = 0.0f;
    if (Ld > 1e-3f) {
        curvature = 2.0f * local_y / (Ld * Ld);
    }
    out->curvature = curvature;

    /* 5. 转向角: delta = atan(wheelbase * kappa) */
    float steering = atanf(cfg->wheelbase * curvature);
    out->steering_angle = clampf(steering, -cfg->max_steering, cfg->max_steering);

    /* 6. 误差 */
    out->cross_track_err = pt_calc_cross_track_error(tracker, state);
    float path_heading = atan2f(dy, dx);
    out->heading_err = wrap_angle(path_heading - state->yaw);

    /* 7. 速度控制(弯道减速) */
    float speed_factor = 1.0f - 0.5f * clampf(fabsf(curvature) * 5.0f, 0.0f, 0.8f);
    out->target_speed = cfg->target_speed * speed_factor;

    /* 更新最近点缓存 */
    tracker->last_nearest_idx = pt_find_nearest(tracker, state);

    return out;
}

int pt_is_finished(const pt_tracker_t *tracker, const pt_state_t *state, float threshold) {
    if (!tracker || !state || tracker->path_len == 0) return 0;
    uint16_t last = tracker->path_len - 1;
    return dist2d(state->x, state->y,
                  tracker->path[last].x, tracker->path[last].y) < threshold;
}

void pt_reset(pt_tracker_t *tracker) {
    if (tracker) tracker->last_nearest_idx = 0;
}
