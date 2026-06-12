#include "visual_servo.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ---------- 内部辅助 ---------- */

static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* 构建交互矩阵(图像雅可比矩阵)的单行(2x6) */
/* 对特征点(u,v)在深度Z处: */
/* L = [ -fx/Z,  0,    u/Z,    uv/fx,     -(fx+u^2/fx), v  ] */
/*     [  0,    -fy/Z, v/Z,   (fy+v^2/fy), -uv/fy,     -u  ] */
static void compute_interaction_row(float u, float v, float fx, float fy,
                                     float cx, float cy, float Z,
                                     float L[2][6]) {
    float ui = u - cx;
    float vi = v - cy;
    L[0][0] = -fx / Z;   L[0][1] = 0.0f;       L[0][2] = ui / Z;
    L[0][3] = ui * vi / fx;
    L[0][4] = -(fx + ui * ui / fx);
    L[0][5] = vi;

    L[1][0] = 0.0f;       L[1][1] = -fy / Z;   L[1][2] = vi / Z;
    L[1][3] = (fy + vi * vi / fy);
    L[1][4] = -ui * vi / fy;
    L[1][5] = -ui;
}

/* ---------- 初始化 ---------- */

int vs_ibvs_init(vs_controller_t *ctrl, const vs_ibvs_config_t *cfg,
                  const vs_point2d_t *desired, uint8_t max_feat) {
    if (!ctrl || !cfg || !desired || max_feat == 0) return -1;
    memset(ctrl, 0, sizeof(*ctrl));
    ctrl->mode = VS_MODE_IBVS;
    ctrl->ibvs_cfg = *cfg;
    ctrl->ibvs_cfg.max_features = max_feat;

    ctrl->desired_2d = (vs_point2d_t *)malloc(max_feat * sizeof(vs_point2d_t));
    ctrl->current_2d = (vs_point2d_t *)malloc(max_feat * sizeof(vs_point2d_t));
    if (!ctrl->desired_2d || !ctrl->current_2d) {
        free(ctrl->desired_2d);
        free(ctrl->current_2d);
        return -1;
    }
    memcpy(ctrl->desired_2d, desired, max_feat * sizeof(vs_point2d_t));
    return 0;
}

int vs_pbvs_init(vs_controller_t *ctrl, const vs_pbvs_config_t *cfg,
                  const vs_pose3d_t *desired_pose) {
    if (!ctrl || !cfg || !desired_pose) return -1;
    memset(ctrl, 0, sizeof(*ctrl));
    ctrl->mode = VS_MODE_PBVS;
    ctrl->pbvs_cfg = *cfg;
    ctrl->desired_pose = *desired_pose;
    return 0;
}

/* ---------- IBVS更新 ---------- */

const float *vs_ibvs_update(vs_controller_t *ctrl,
                             const vs_point2d_t *current, float dt) {
    if (!ctrl || !current) return NULL;
    (void)dt;
    uint8_t n = ctrl->ibvs_cfg.max_features;
    const vs_ibvs_config_t *cfg = &ctrl->ibvs_cfg;

    memcpy(ctrl->current_2d, current, n * sizeof(vs_point2d_t));

    /* 组装 L矩阵(2n x 6) 和 误差向量e(2n x 1) */
    /* 使用伪逆 Ls = (L^T L)^-1 L^T, 这里用简化的增益*误差求和 */
    float e_sum[6] = {0};
    float lambda = cfg->lambda;
    float Z = cfg->depth_est;

    for (uint8_t i = 0; i < n; i++) {
        float eu = current[i].u - ctrl->desired_2d[i].u;
        float ev = current[i].v - ctrl->desired_2d[i].v;
        float L[2][6];
        compute_interaction_row(current[i].u, current[i].v,
                                cfg->cam.fx, cfg->cam.fy,
                                cfg->cam.cx, cfg->cam.cy, Z, L);
        /* 累加 L^T * e 到 6维向量 */
        for (int k = 0; k < 6; k++) {
            e_sum[k] += L[0][k] * eu + L[1][k] * ev;
        }
    }

    /* 简化: v = -lambda * sum(L^T * e) / n */
    for (int k = 0; k < 6; k++) {
        ctrl->velocity_cmd[k] = -lambda * e_sum[k] / (float)n;
        ctrl->velocity_cmd[k] = clampf(ctrl->velocity_cmd[k], -2.0f, 2.0f);
    }
    return ctrl->velocity_cmd;
}

/* ---------- PBVS更新 ---------- */

const float *vs_pbvs_update(vs_controller_t *ctrl,
                             const vs_pose3d_t *current_pose, float dt) {
    if (!ctrl || !current_pose) return NULL;
    (void)dt;

    ctrl->current_pose = *current_pose;
    const vs_pose3d_t *des = &ctrl->desired_pose;
    float lp = ctrl->pbvs_cfg.lambda_pos;
    float lr = ctrl->pbvs_cfg.lambda_rot;

    /* 位置误差 */
    float ex = current_pose->x - des->x;
    float ey = current_pose->y - des->y;
    float ez = current_pose->z - des->z;

    /* 姿态误差(简化: 直接用欧拉角差) */
    float er = current_pose->roll  - des->roll;
    float ep = current_pose->pitch - des->pitch;
    float eya = current_pose->yaw  - des->yaw;
    /* 角度归一化到 [-pi, pi] */
    while (er > M_PI) er -= 2*M_PI; while (er < -M_PI) er += 2*M_PI;
    while (ep > M_PI) ep -= 2*M_PI; while (ep < -M_PI) ep += 2*M_PI;
    while (eya > M_PI) eya -= 2*M_PI; while (eya < -M_PI) eya += 2*M_PI;

    ctrl->velocity_cmd[0] = clampf(-lp * ex, -2.0f, 2.0f);
    ctrl->velocity_cmd[1] = clampf(-lp * ey, -2.0f, 2.0f);
    ctrl->velocity_cmd[2] = clampf(-lp * ez, -2.0f, 2.0f);
    ctrl->velocity_cmd[3] = clampf(-lr * er, -2.0f, 2.0f);
    ctrl->velocity_cmd[4] = clampf(-lr * ep, -2.0f, 2.0f);
    ctrl->velocity_cmd[5] = clampf(-lr * eya, -2.0f, 2.0f);
    return ctrl->velocity_cmd;
}

/* ---------- 收敛判断 ---------- */

float vs_get_error_norm(const vs_controller_t *ctrl) {
    if (!ctrl) return 0.0f;
    if (ctrl->mode == VS_MODE_IBVS && ctrl->desired_2d && ctrl->current_2d) {
        float sum = 0.0f;
        uint8_t n = ctrl->ibvs_cfg.max_features;
        for (uint8_t i = 0; i < n; i++) {
            float du = ctrl->current_2d[i].u - ctrl->desired_2d[i].u;
            float dv = ctrl->current_2d[i].v - ctrl->desired_2d[i].v;
            sum += du * du + dv * dv;
        }
        return sqrtf(sum / (float)n);
    }
    if (ctrl->mode == VS_MODE_PBVS) {
        float dx = ctrl->current_pose.x - ctrl->desired_pose.x;
        float dy = ctrl->current_pose.y - ctrl->desired_pose.y;
        float dz = ctrl->current_pose.z - ctrl->desired_pose.z;
        return sqrtf(dx*dx + dy*dy + dz*dz);
    }
    return 0.0f;
}

int vs_is_converged(const vs_controller_t *ctrl, float threshold) {
    return vs_get_error_norm(ctrl) < threshold;
}

void vs_deinit(vs_controller_t *ctrl) {
    if (!ctrl) return;
    free(ctrl->desired_2d);
    free(ctrl->current_2d);
    ctrl->desired_2d = NULL;
    ctrl->current_2d = NULL;
}

/* ---------- 旋转工具 ---------- */

void vs_rotation_to_euler(const float R[3][3], float *roll, float *pitch, float *yaw) {
    *pitch = asinf(clampf(-R[2][0], -1.0f, 1.0f));
    if (fabsf(cosf(*pitch)) > 1e-6f) {
        *roll = atan2f(R[2][1], R[2][2]);
        *yaw  = atan2f(R[1][0], R[0][0]);
    } else {
        *roll = atan2f(-R[0][1], R[1][1]);
        *yaw  = 0.0f;
    }
}

void vs_euler_to_rotation(float roll, float pitch, float yaw, float R[3][3]) {
    float cr = cosf(roll),  sr = sinf(roll);
    float cp = cosf(pitch), sp = sinf(pitch);
    float cy = cosf(yaw),   sy = sinf(yaw);
    R[0][0] = cy*cp;  R[0][1] = cy*sp*sr - sy*cr;  R[0][2] = cy*sp*cr + sy*sr;
    R[1][0] = sy*cp;  R[1][1] = sy*sp*sr + cy*cr;  R[1][2] = sy*sp*cr - cy*sr;
    R[2][0] = -sp;    R[2][1] = cp*sr;              R[2][2] = cp*cr;
}
