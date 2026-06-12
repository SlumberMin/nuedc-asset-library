/**
 * @file motion_control.h
 * @brief 运动控制算法库 - 点到点运动与连续路径跟踪
 * @version 1.0
 * @date 2026-06
 *
 * 支持:
 *   - 梯形速度规划 (Trapezoidal Profile)
 *   - S形速度规划 (S-curve Profile)
 *   - 三次多项式插值 (Cubic Polynomial)
 *   - 点到点运动 (Point-to-Point)
 *   - 连续路径跟踪 (Continuous Path / Linear/Circular Interpolation)
 */

#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== 速度规划 ======================== */

/** 运动状态枚举 */
typedef enum {
    MOTION_IDLE = 0,
    MOTION_ACCEL,       /* 加速阶段 */
    MOTION_CONST,       /* 匀速阶段 */
    MOTION_DECEL,       /* 减速阶段 */
    MOTION_FINISHED
} MotionState_e;

/** 梯形速度规划器 */
typedef struct {
    float q0;           /* 起始位置 */
    float qf;           /* 目标位置 */
    float v_max;        /* 最大速度 */
    float a_max;        /* 最大加速度 */
    float dt;           /* 采样周期 */
    /* 内部状态 */
    float t;            /* 当前时间 */
    float T_accel;      /* 加速段时间 */
    float T_const;      /* 匀速段时间 */
    float T_decel;      /* 减速段时间 */
    float T_total;      /* 总时间 */
    float q;            /* 当前位置 */
    float dq;           /* 当前速度 */
    MotionState_e state;
} TrapezoidalProfile_t;

/** S形速度规划器 (7段) */
typedef struct {
    float q0, qf;
    float v_max, a_max, j_max;  /* 最大速度/加速度/加加速度 */
    float dt;
    /* 内部状态 */
    float t;
    float T[7];         /* 7段时间: j1,a,j2,v,j3,d,j4 */
    float q, dq, ddq;
    MotionState_e state;
} SCurveProfile_t;

/** 三次多项式轨迹: q(t) = a0 + a1*t + a2*t^2 + a3*t^3 */
typedef struct {
    float a0, a1, a2, a3;
    float T;            /* 轨迹总时长 */
    float t;            /* 当前时间 */
    float dt;
    float q, dq;
} CubicPolyTraj_t;

/* --- 梯形规划 --- */
void Trapezoidal_Init(TrapezoidalProfile_t *p, float q0, float qf,
                       float v_max, float a_max, float dt);
/** 返回1表示规划仍在运行, 0表示完成 */
int  Trapezoidal_Update(TrapezoidalProfile_t *p);
float Trapezoidal_GetPos(const TrapezoidalProfile_t *p);
float Trapezoidal_GetVel(const TrapezoidalProfile_t *p);
void  Trapezoidal_Reset(TrapezoidalProfile_t *p);

/* --- S形规划 --- */
void SCurve_Init(SCurveProfile_t *p, float q0, float qf,
                  float v_max, float a_max, float j_max, float dt);
int  SCurve_Update(SCurveProfile_t *p);
float SCurve_GetPos(const SCurveProfile_t *p);
float SCurve_GetVel(const SCurveProfile_t *p);
void  SCurve_Reset(SCurveProfile_t *p);

/* --- 三次多项式 --- */
void CubicPoly_Init(CubicPolyTraj_t *p, float q0, float dq0,
                     float qf, float dqf, float T, float dt);
int  CubicPoly_Update(CubicPolyTraj_t *p);
float CubicPoly_GetPos(const CubicPolyTraj_t *p);
float CubicPoly_GetVel(const CubicPolyTraj_t *p);

/* ======================== 点到点运动 ======================== */

/** 规划方式 */
typedef enum {
    PROFILE_TRAPEZOIDAL = 0,
    PROFILE_S_CURVE,
    PROFILE_CUBIC_POLY
} ProfileType_e;

/** 点到点运动控制器 */
typedef struct {
    ProfileType_e profile_type;
    TrapezoidalProfile_t trap;
    SCurveProfile_t scurve;
    CubicPolyTraj_t cubic;
    /* 闭环PID (可选) */
    float Kp, Ki, Kd;
    float dt;           /* PID采样时间 */
    float err_integral;
    float err_prev;
    float output;       /* 最终控制量 */
} PTPController_t;

/** 初始化点到点控制器 (使用梯形规划) */
void PTP_Init_Trapezoidal(PTPController_t *c, float q0, float qf,
                           float v_max, float a_max, float dt,
                           float Kp, float Ki, float Kd);

/** 初始化点到点控制器 (使用S形规划) */
void PTP_Init_SCurve(PTPController_t *c, float q0, float qf,
                      float v_max, float a_max, float j_max, float dt,
                      float Kp, float Ki, float Kd);

/** 更新点到点控制器, 返回1=运行中, 0=完成 */
int  PTP_Update(PTPController_t *c, float q_meas);
float PTP_GetOutput(const PTPController_t *c);
float PTP_GetRefPos(const PTPController_t *c);

/* ======================== 连续路径插补 ======================== */

/** 2D点 */
typedef struct {
    float x, y;
} Point2D_t;

/** 3D点 */
typedef struct {
    float x, y, z;
} Point3D_t;

/** 直线插补器 (2D) */
typedef struct {
    Point2D_t start;
    Point2D_t end;
    float feed_rate;    /* 进给速率 (mm/s) */
    float dt;
    float total_len;
    float traveled;
    float ratio;        /* 0.0 ~ 1.0 */
    Point2D_t current;
    int finished;
} LinearInterp2D_t;

/** 圆弧插补器 (2D) */
typedef struct {
    Point2D_t center;
    float radius;
    float start_angle;  /* 起始角 (rad) */
    float sweep_angle;  /* 扫过角 (rad, 正=逆时针) */
    float feed_rate;
    float dt;
    float arc_len;      /* 圆弧总长 */
    float traveled;
    float angle;        /* 当前角度 */
    Point2D_t current;
    int finished;
} ArcInterp2D_t;

/* --- 直线插补 --- */
void LinearInterp2D_Init(LinearInterp2D_t *ip, Point2D_t start, Point2D_t end,
                          float feed_rate, float dt);
int  LinearInterp2D_Update(LinearInterp2D_t *ip);
Point2D_t LinearInterp2D_GetPos(const LinearInterp2D_t *ip);

/* --- 圆弧插补 --- */
void ArcInterp2D_Init(ArcInterp2D_t *ip, Point2D_t center, float radius,
                       float start_angle, float sweep_angle,
                       float feed_rate, float dt);
int  ArcInterp2D_Update(ArcInterp2D_t *ip);
Point2D_t ArcInterp2D_GetPos(const ArcInterp2D_t *ip);

#ifdef __cplusplus
}
#endif

#endif /* MOTION_CONTROL_H */
