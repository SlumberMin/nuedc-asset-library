#ifndef VISUAL_SERVO_H
#define VISUAL_SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 视觉伺服模式 */
typedef enum {
    VS_MODE_IBVS = 0,  /* 基于图像的视觉伺服 */
    VS_MODE_PBVS = 1   /* 基于位置的视觉伺服 */
} vs_mode_t;

/* 2D点 */
typedef struct {
    float u;  /* 像素坐标u */
    float v;  /* 像素坐标v */
} vs_point2d_t;

/* 3D位姿 */
typedef struct {
    float x, y, z;           /* 平移(m) */
    float roll, pitch, yaw;  /* 旋转(rad) */
} vs_pose3d_t;

/* 相机内参 */
typedef struct {
    float fx, fy;   /* 焦距(像素) */
    float cx, cy;   /* 主点(像素) */
} vs_camera_intrinsics_t;

/* IBVS控制器配置 */
typedef struct {
    vs_camera_intrinsics_t cam;
    float lambda;           /* 控制增益 */
    float depth_est;        /* 估计深度(m) */
    uint8_t max_features;   /* 最大特征点数 */
} vs_ibvs_config_t;

/* PBVS控制器配置 */
typedef struct {
    float lambda_pos;       /* 位置增益 */
    float lambda_rot;       /* 旋转增益 */
} vs_pbvs_config_t;

/* 视觉伺服控制器 */
typedef struct {
    vs_mode_t mode;
    vs_ibvs_config_t ibvs_cfg;
    vs_pbvs_config_t pbvs_cfg;
    /* 运行时数据 */
    vs_point2d_t *desired_2d;   /* 期望2D特征(IBVS) */
    vs_point2d_t *current_2d;
    vs_pose3d_t desired_pose;   /* 期望位姿(PBVS) */
    vs_pose3d_t current_pose;
    float velocity_cmd[6];      /* 输出: vx vy vz wx wy wz */
} vs_controller_t;

/**
 * @brief 初始化IBVS控制器
 * @param ctrl 控制器指针
 * @param cfg  IBVS配置
 * @param desired 期望特征点数组
 * @param max_feat 最大特征点数
 * @return 0成功
 */
int vs_ibvs_init(vs_controller_t *ctrl, const vs_ibvs_config_t *cfg,
                  const vs_point2d_t *desired, uint8_t max_feat);

/**
 * @brief 初始化PBVS控制器
 * @param ctrl 控制器指针
 * @param cfg  PBVS配置
 * @param desired_pose 期望位姿
 * @return 0成功
 */
int vs_pbvs_init(vs_controller_t *ctrl, const vs_pbvs_config_t *cfg,
                  const vs_pose3d_t *desired_pose);

/**
 * @brief IBVS计算
 * @param ctrl  控制器
 * @param current 当前特征点数组
 * @param dt 时间步长(s)
 * @return 速度指令指针(vx,vy,vz,wx,wy,wz)
 */
const float *vs_ibvs_update(vs_controller_t *ctrl,
                             const vs_point2d_t *current, float dt);

/**
 * @brief PBVS计算
 * @param ctrl 控制器
 * @param current_pose 当前位姿
 * @param dt 时间步长(s)
 * @return 速度指令指针
 */
const float *vs_pbvs_update(vs_controller_t *ctrl,
                             const vs_pose3d_t *current_pose, float dt);

/**
 * @brief 获取误差范数(收敛判断)
 */
float vs_get_error_norm(const vs_controller_t *ctrl);

/**
 * @brief 判断是否收敛
 * @param threshold 收敛阈值
 */
int vs_is_converged(const vs_controller_t *ctrl, float threshold);

/**
 * @brief 释放资源
 */
void vs_deinit(vs_controller_t *ctrl);

/* 工具函数: 旋转矩阵转欧拉角 */
void vs_rotation_to_euler(const float R[3][3], float *roll, float *pitch, float *yaw);

/* 工具函数: 欧拉角转旋转矩阵 */
void vs_euler_to_rotation(float roll, float pitch, float yaw, float R[3][3]);

#ifdef __cplusplus
}
#endif

#endif /* VISUAL_SERVO_H */
