/**
 * @file motion_control.c
 * @brief 运动控制算法实现
 * @details 提供多种运动规划和控制算法:
 *          - 梯形速度规划(Trapezoidal Profile)
 *          - S形速度规划(S-Curve Profile)
 *          - 三次多项式轨迹(Cubic Polynomial)
 *          - 点到点PID运动控制(PTP)
 *          - 2D直线插补
 *          - 2D圆弧插补
 *          适用于机器人运动控制、CNC插补等场景。
 */

#include "motion_control.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/**
 * @brief 浮点数绝对值辅助函数
 * @param x 输入值
 * @return |x|
 */
static float fabsf_local(float x) { return x < 0 ? -x : x; }

/**
 * @brief 符号函数
 * @param x 输入值
 * @return 1.0(正), -1.0(负), 0.0(零)
 */
static float signf(float x) { return x > 0 ? 1.0f : (x < 0 ? -1.0f : 0.0f); }

/* ======================== 梯形速度规划 ======================== */

/**
 * @brief 初始化梯形速度规划器
 * @param p 规划器结构体指针
 * @param q0 起始位置
 * @param qf 目标位置
 * @param v_max 最大速度
 * @param a_max 最大加速度
 * @param dt 采样时间间隔(秒)
 *
 * @details 自动计算加速段、匀速段、减速段时间。
 *          若距离不足以加速到v_max, 则使用三角形规划。
 */
void Trapezoidal_Init(TrapezoidalProfile_t *p, float q0, float qf,
                       float v_max, float a_max, float dt)
{
    if (p == NULL) return;
    if (dt <= 0.0f) dt = 0.001f;

    p->q0 = q0;
    p->qf = qf;
    p->v_max = fabsf_local(v_max);
    p->a_max = fabsf_local(a_max);
    p->dt = dt;
    p->t = 0.0f;
    p->q = q0;
    p->dq = 0.0f;
    p->state = MOTION_ACCEL;

    float dist = fabsf_local(qf - q0);
    float dir = signf(qf - q0);

    /* 计算是否能达到最大速度 */
    float t_acc = p->v_max / p->a_max;
    float d_acc = 0.5f * p->a_max * t_acc * t_acc;

    if (2.0f * d_acc > dist) {
        /* 三角形规划: 距离不够达到v_max */
        t_acc = sqrtf(dist / p->a_max);
        p->T_accel = t_acc;
        p->T_const = 0.0f;
        p->T_decel = t_acc;
    } else {
        /* 梯形规划: 有匀速段 */
        p->T_accel = t_acc;
        p->T_decel = t_acc;
        float d_const = dist - 2.0f * d_acc;
        p->T_const = d_const / p->v_max;
    }
    p->T_total = p->T_accel + p->T_const + p->T_decel;
    p->v_max *= dir;  /* 恢复方向符号 */
    p->a_max *= dir;
}

/**
 * @brief 梯形速度规划器一步更新
 * @param p 规划器结构体指针
 * @return 1=运动中, 0=已完成
 */
int Trapezoidal_Update(TrapezoidalProfile_t *p)
{
    if (p == NULL) return 0;
    if (p->state == MOTION_FINISHED) return 0;

    float dir = signf(p->qf - p->q0);
    float t = p->t;

    if (t < p->T_accel) {
        /* 加速段: v = a*t, q = q0 + 0.5*a*t^2 */
        p->dq = p->a_max * t;
        p->q = p->q0 + 0.5f * p->a_max * t * t;
        p->state = MOTION_ACCEL;
    } else if (t < p->T_accel + p->T_const) {
        /* 匀速段: v = v_peak, q = q0 + d_acc + v_peak*(t-t_acc) */
        float tc = t - p->T_accel;
        float v_peak = p->a_max * p->T_accel;
        p->dq = v_peak;
        float d_acc = 0.5f * p->a_max * p->T_accel * p->T_accel;
        p->q = p->q0 + d_acc + v_peak * tc;
        p->state = MOTION_CONST;
    } else if (t < p->T_total) {
        /* 减速段 */
        float td = t - p->T_accel - p->T_const;
        float v_peak = p->a_max * p->T_accel;
        p->dq = v_peak - p->a_max * td;
        float d_acc = 0.5f * p->a_max * p->T_accel * p->T_accel;
        p->q = p->q0 + d_acc + v_peak * p->T_const
               + v_peak * td - 0.5f * p->a_max * td * td;
        p->state = MOTION_DECEL;
    } else {
        /* 运动完成 */
        p->q = p->qf;
        p->dq = 0.0f;
        p->state = MOTION_FINISHED;
        return 0;
    }

    p->t += p->dt;
    return 1;
}

/** @brief 获取当前位置 */
float Trapezoidal_GetPos(const TrapezoidalProfile_t *p) { return p ? p->q : 0.0f; }
/** @brief 获取当前速度 */
float Trapezoidal_GetVel(const TrapezoidalProfile_t *p) { return p ? p->dq : 0.0f; }

/**
 * @brief 重置梯形规划器到起始位置
 * @param p 规划器结构体指针
 */
void Trapezoidal_Reset(TrapezoidalProfile_t *p)
{
    if (p == NULL) return;
    p->t = 0.0f;
    p->q = p->q0;
    p->dq = 0.0f;
    p->state = MOTION_ACCEL;
}

/* ======================== S形速度规划 (简化版) ======================== */

/**
 * @brief 初始化S形速度规划器
 * @param p 规划器结构体指针
 * @param q0 起始位置
 * @param qf 目标位置
 * @param v_max 最大速度
 * @param a_max 最大加速度
 * @param j_max 最大加加速度(jerk)
 * @param dt 采样时间间隔(秒)
 *
 * @details S形规划通过限制加加速度(jerk)实现加速度的平滑过渡,
 *          共7个阶段: 加加速→恒加速→减加速→匀速→加减速→恒减速→减减速
 */
void SCurve_Init(SCurveProfile_t *p, float q0, float qf,
                  float v_max, float a_max, float j_max, float dt)
{
    if (p == NULL) return;
    if (dt <= 0.0f) dt = 0.001f;

    p->q0 = q0;
    p->qf = qf;
    p->v_max = fabsf_local(v_max);
    p->a_max = fabsf_local(a_max);
    p->j_max = fabsf_local(j_max);
    p->dt = dt;
    p->t = 0.0f;
    p->q = q0;
    p->dq = 0.0f;
    p->ddq = 0.0f;
    p->state = MOTION_ACCEL;

    /* 计算各段时间 */
    float Ta = p->a_max / p->j_max;  /* 加加速度段时间 */
    float Tj = p->v_max / p->a_max;  /* 加速段总时间 */
    p->T[0] = Ta;           /* 加加速 */
    p->T[1] = Tj - Ta;      /* 恒加速 */
    p->T[2] = Ta;           /* 减加速 */
    p->T[3] = 0.5f;         /* 匀速段(占位, 实际由距离决定) */
    p->T[4] = Ta;           /* 加减速 */
    p->T[5] = Tj - Ta;      /* 恒减速 */
    p->T[6] = Ta;           /* 减减速 */

    /* 修正匀速段时间 */
    float dist = fabsf_local(qf - q0);
    float d_accel = p->v_max * (p->T[0] + p->T[1] + p->T[2]);
    if (d_accel * 2 > dist) {
        /* 距离不足, 降低最大速度 */
        p->v_max = sqrtf(dist * p->a_max);
    }
    float d_const = dist - 2.0f * d_accel;
    if (d_const < 0) d_const = 0;
    p->T[3] = (p->v_max > 0.001f) ? d_const / p->v_max : 0;
}

/**
 * @brief S形速度规划器一步更新
 * @param p 规划器结构体指针
 * @return 1=运动中, 0=已完成
 */
int SCurve_Update(SCurveProfile_t *p)
{
    if (p == NULL) return 0;
    if (p->state == MOTION_FINISHED) return 0;

    float dir = signf(p->qf - p->q0);

    /* 计算各段累计时间 */
    float cumT[8];
    cumT[0] = 0;
    for (int i = 0; i < 7; i++) cumT[i+1] = cumT[i] + p->T[i];

    float t = p->t;
    float dt_local;

    /* 根据当前时间判断所处阶段 */
    if (t < cumT[1]) {
        /* 段1: 加加速(jerk>0, 加速度线性增加) */
        dt_local = t;
        p->ddq = p->j_max * dt_local;
        p->state = MOTION_ACCEL;
    } else if (t < cumT[2]) {
        /* 段2: 恒加速(加速度保持最大) */
        p->ddq = p->a_max;
        p->state = MOTION_ACCEL;
    } else if (t < cumT[3]) {
        /* 段3: 减加速(jerk<0, 加速度线性减小到0) */
        dt_local = cumT[3] - t;
        p->ddq = p->a_max - p->j_max * (cumT[3] - cumT[2] - dt_local);
        p->state = MOTION_ACCEL;
    } else if (t < cumT[4]) {
        /* 段4: 匀速 */
        p->ddq = 0;
        p->state = MOTION_CONST;
    } else if (t < cumT[5]) {
        /* 段5: 加减速 */
        dt_local = t - cumT[4];
        p->ddq = -p->j_max * dt_local;
        p->state = MOTION_DECEL;
    } else if (t < cumT[6]) {
        /* 段6: 恒减速 */
        p->ddq = -p->a_max;
        p->state = MOTION_DECEL;
    } else if (t < cumT[7]) {
        /* 段7: 减减速 */
        dt_local = cumT[7] - t;
        p->ddq = -p->a_max + p->j_max * (cumT[7] - cumT[6] - dt_local);
        p->state = MOTION_DECEL;
    } else {
        /* 运动完成 */
        p->q = p->qf;
        p->dq = 0.0f;
        p->ddq = 0.0f;
        p->state = MOTION_FINISHED;
        return 0;
    }

    /* 数值积分更新速度和位置 */
    p->dq += p->ddq * p->dt * dir;
    p->q += p->dq * p->dt;
    p->t += p->dt;
    return 1;
}

/** @brief 获取S形规划器当前位置 */
float SCurve_GetPos(const SCurveProfile_t *p) { return p ? p->q : 0.0f; }
/** @brief 获取S形规划器当前速度 */
float SCurve_GetVel(const SCurveProfile_t *p) { return p ? p->dq : 0.0f; }

/**
 * @brief 重置S形规划器
 * @param p 规划器结构体指针
 */
void SCurve_Reset(SCurveProfile_t *p)
{
    if (p == NULL) return;
    p->t = 0.0f;
    p->q = p->q0;
    p->dq = 0.0f;
    p->ddq = 0.0f;
    p->state = MOTION_ACCEL;
}

/* ======================== 三次多项式 ======================== */

/**
 * @brief 初始化三次多项式轨迹规划器
 * @param p 规划器结构体指针
 * @param q0 起始位置
 * @param dq0 起始速度
 * @param qf 目标位置
 * @param dqf 目标速度
 * @param T 总运动时间(秒)
 * @param dt 采样时间间隔(秒)
 *
 * @details 三次多项式: q(t) = a0 + a1*t + a2*t^2 + a3*t^3
 *          由边界条件 q(0)=q0, q'(0)=dq0, q(T)=qf, q'(T)=dqf 确定系数
 */
void CubicPoly_Init(CubicPolyTraj_t *p, float q0, float dq0,
                     float qf, float dqf, float T, float dt)
{
    if (p == NULL) return;
    if (T <= 0.0f) T = 1.0f;
    if (dt <= 0.0f) dt = 0.001f;

    p->T = T;
    p->dt = dt;
    p->t = 0.0f;

    /* 求解三次多项式系数 */
    p->a0 = q0;
    p->a1 = dq0;
    float T2 = T * T;
    float T3 = T2 * T;
    p->a2 = (3.0f * (qf - q0) - (2.0f * dq0 + dqf) * T) / T2;
    p->a3 = (2.0f * (q0 - qf) + (dq0 + dqf) * T) / T3;

    p->q = q0;
    p->dq = dq0;
}

/**
 * @brief 三次多项式轨迹一步更新
 * @param p 规划器结构体指针
 * @return 1=运动中, 0=已完成
 */
int CubicPoly_Update(CubicPolyTraj_t *p)
{
    if (p == NULL) return 0;

    if (p->t > p->T) {
        p->t = p->T;
        return 0;
    }
    float t = p->t;
    float t2 = t * t;
    float t3 = t2 * t;
    /* 位置和速度的解析计算 */
    p->q = p->a0 + p->a1 * t + p->a2 * t2 + p->a3 * t3;
    p->dq = p->a1 + 2.0f * p->a2 * t + 3.0f * p->a3 * t2;
    p->t += p->dt;
    return (p->t <= p->T) ? 1 : 0;
}

/** @brief 获取三次多项式轨迹当前位置 */
float CubicPoly_GetPos(const CubicPolyTraj_t *p) { return p ? p->q : 0.0f; }
/** @brief 获取三次多项式轨迹当前速度 */
float CubicPoly_GetVel(const CubicPolyTraj_t *p) { return p ? p->dq : 0.0f; }

/* ======================== 点到点运动 ======================== */

/**
 * @brief PID控制器公共初始化
 * @param c PTP控制器指针
 * @param Kp 比例增益
 * @param Ki 积分增益
 * @param Kd 微分增益
 */
static void pid_init_common(PTPController_t *c, float Kp, float Ki, float Kd)
{
    c->Kp = Kp;
    c->Ki = Ki;
    c->Kd = Kd;
    c->err_integral = 0.0f;
    c->err_prev = 0.0f;
    c->output = 0.0f;
}

/**
 * @brief 初始化PTP控制器(梯形规划)
 * @param c PTP控制器指针
 * @param q0 起始位置
 * @param qf 目标位置
 * @param v_max 最大速度
 * @param a_max 最大加速度
 * @param dt 采样时间
 * @param Kp 比例增益
 * @param Ki 积分增益
 * @param Kd 微分增益
 */
void PTP_Init_Trapezoidal(PTPController_t *c, float q0, float qf,
                           float v_max, float a_max, float dt,
                           float Kp, float Ki, float Kd)
{
    if (c == NULL) return;
    c->profile_type = PROFILE_TRAPEZOIDAL;
    Trapezoidal_Init(&c->trap, q0, qf, v_max, a_max, dt);
    pid_init_common(c, Kp, Ki, Kd);
    c->dt = dt;
}

/**
 * @brief 初始化PTP控制器(S形规划)
 * @param c PTP控制器指针
 * @param q0 起始位置
 * @param qf 目标位置
 * @param v_max 最大速度
 * @param a_max 最大加速度
 * @param j_max 最大加加速度
 * @param dt 采样时间
 * @param Kp 比例增益
 * @param Ki 积分增益
 * @param Kd 微分增益
 */
void PTP_Init_SCurve(PTPController_t *c, float q0, float qf,
                      float v_max, float a_max, float j_max, float dt,
                      float Kp, float Ki, float Kd)
{
    if (c == NULL) return;
    c->profile_type = PROFILE_S_CURVE;
    SCurve_Init(&c->scurve, q0, qf, v_max, a_max, j_max, dt);
    pid_init_common(c, Kp, Ki, Kd);
    c->dt = dt;
}

/**
 * @brief PTP控制器一步更新
 * @param c PTP控制器指针
 * @param q_meas 当前实际位置
 * @return 1=运动中, 0=已完成
 */
int PTP_Update(PTPController_t *c, float q_meas)
{
    if (c == NULL) return 0;

    float q_ref;
    int running = 1;

    /* 根据规划类型获取参考位置 */
    switch (c->profile_type) {
    case PROFILE_TRAPEZOIDAL:
        running = Trapezoidal_Update(&c->trap);
        q_ref = Trapezoidal_GetPos(&c->trap);
        break;
    case PROFILE_S_CURVE:
        running = SCurve_Update(&c->scurve);
        q_ref = SCurve_GetPos(&c->scurve);
        break;
    default:
        q_ref = 0;
        break;
    }

    /* PID闭环跟踪 */
    float err = q_ref - q_meas;
    c->err_integral += err * c->dt;
    float d_err = (err - c->err_prev) / c->dt;
    c->err_prev = err;
    c->output = c->Kp * err + c->Ki * c->err_integral + c->Kd * d_err;

    return running;
}

/** @brief 获取PTP控制器输出 */
float PTP_GetOutput(const PTPController_t *c) { return c ? c->output : 0.0f; }

/**
 * @brief 获取PTP参考位置
 * @param c PTP控制器指针
 * @return 参考位置值
 */
float PTP_GetRefPos(const PTPController_t *c)
{
    if (c == NULL) return 0.0f;
    switch (c->profile_type) {
    case PROFILE_TRAPEZOIDAL: return Trapezoidal_GetPos(&c->trap);
    case PROFILE_S_CURVE:     return SCurve_GetPos(&c->scurve);
    default: return 0;
    }
}

/* ======================== 直线插补 ======================== */

/**
 * @brief 初始化2D直线插补器
 * @param ip 插补器结构体指针
 * @param start 起始点坐标
 * @param end 终止点坐标
 * @param feed_rate 进给速率(单位/秒)
 * @param dt 采样时间间隔(秒)
 */
void LinearInterp2D_Init(LinearInterp2D_t *ip, Point2D_t start, Point2D_t end,
                          float feed_rate, float dt)
{
    if (ip == NULL) return;
    ip->start = start;
    ip->end = end;
    ip->feed_rate = feed_rate;
    ip->dt = dt;
    ip->traveled = 0.0f;
    ip->finished = 0;

    /* 计算总路径长度 */
    float dx = end.x - start.x;
    float dy = end.y - start.y;
    ip->total_len = sqrtf(dx * dx + dy * dy);
    ip->ratio = 0.0f;
    ip->current = start;
}

/**
 * @brief 2D直线插补器一步更新
 * @param ip 插补器结构体指针
 * @return 1=运动中, 0=已完成
 */
int LinearInterp2D_Update(LinearInterp2D_t *ip)
{
    if (ip == NULL) return 0;
    if (ip->finished) return 0;

    /* 按进给速率推进 */
    ip->traveled += ip->feed_rate * ip->dt;
    if (ip->traveled >= ip->total_len) {
        /* 到达终点 */
        ip->traveled = ip->total_len;
        ip->current = ip->end;
        ip->ratio = 1.0f;
        ip->finished = 1;
        return 0;
    }

    /* 线性插值计算当前位置 */
    ip->ratio = ip->traveled / ip->total_len;
    ip->current.x = ip->start.x + ip->ratio * (ip->end.x - ip->start.x);
    ip->current.y = ip->start.y + ip->ratio * (ip->end.y - ip->start.y);
    return 1;
}

/** @brief 获取直线插补器当前位置 */
Point2D_t LinearInterp2D_GetPos(const LinearInterp2D_t *ip) { return ip ? ip->current : (Point2D_t){0,0}; }

/* ======================== 圆弧插补 ======================== */

/**
 * @brief 初始化2D圆弧插补器
 * @param ip 插补器结构体指针
 * @param center 圆心坐标
 * @param radius 圆弧半径
 * @param start_angle 起始角度(弧度)
 * @param sweep_angle 扫过角度(弧度), 正=逆时针, 负=顺时针
 * @param feed_rate 进给速率(单位/秒)
 * @param dt 采样时间间隔(秒)
 */
void ArcInterp2D_Init(ArcInterp2D_t *ip, Point2D_t center, float radius,
                       float start_angle, float sweep_angle,
                       float feed_rate, float dt)
{
    if (ip == NULL) return;
    ip->center = center;
    ip->radius = radius;
    ip->start_angle = start_angle;
    ip->sweep_angle = sweep_angle;
    ip->feed_rate = feed_rate;
    ip->dt = dt;
    /* 圆弧长度 = |角度| * 半径 */
    ip->arc_len = fabsf_local(sweep_angle) * radius;
    ip->traveled = 0.0f;
    ip->angle = start_angle;
    ip->finished = 0;
    /* 初始位置 */
    ip->current.x = center.x + radius * cosf(start_angle);
    ip->current.y = center.y + radius * sinf(start_angle);
}

/**
 * @brief 2D圆弧插补器一步更新
 * @param ip 插补器结构体指针
 * @return 1=运动中, 0=已完成
 */
int ArcInterp2D_Update(ArcInterp2D_t *ip)
{
    if (ip == NULL) return 0;
    if (ip->finished) return 0;

    /* 按进给速率推进 */
    ip->traveled += ip->feed_rate * ip->dt;
    if (ip->traveled >= ip->arc_len) {
        /* 到达终点 */
        ip->angle = ip->start_angle + ip->sweep_angle;
        ip->current.x = ip->center.x + ip->radius * cosf(ip->angle);
        ip->current.y = ip->center.y + ip->radius * sinf(ip->angle);
        ip->finished = 1;
        return 0;
    }

    /* 按比例插值角度 */
    float ratio = ip->traveled / ip->arc_len;
    ip->angle = ip->start_angle + ratio * ip->sweep_angle;
    ip->current.x = ip->center.x + ip->radius * cosf(ip->angle);
    ip->current.y = ip->center.y + ip->radius * sinf(ip->angle);
    return 1;
}

/** @brief 获取圆弧插补器当前位置 */
Point2D_t ArcInterp2D_GetPos(const ArcInterp2D_t *ip) { return ip ? ip->current : (Point2D_t){0,0}; }
