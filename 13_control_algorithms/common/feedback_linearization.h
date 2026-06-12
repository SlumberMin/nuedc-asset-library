/**
 * @file feedback_linearization.h
 * @brief 反馈线性化控制器 - 非线性系统精确线性化
 * 
 * 反馈线性化(Feedback Linearization)通过非线性状态变换和反馈
 * 将非线性系统精确转化为线性系统，再用线性控制方法设计。
 * 
 * 适用系统形式(仿射非线性系统)：
 *   ẋ = f(x) + g(x)u
 *   y = h(x)
 * 
 * 通过选择输出y的Lie导数直到出现控制输入u，得到相对阶r，
 * 然后令 u = (v - Lf^r*h) / (Lg*Lf^(r-1)*h)，使系统线性化为 y^(r) = v。
 * 
 * 典型应用：机器人控制、飞行器控制、化工过程控制等
 * 
 * 参数整定指南：
 *   - Kp, Kd, Ki：线性化后PD/PID增益
 *   - 系统相对阶r决定使用PD还是PID
 *   - 需要精确的系统模型，模型误差会影响性能
 */

#ifndef FEEDBACK_LINEARIZATION_H
#define FEEDBACK_LINEARIZATION_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 最大状态维数 */
#define FL_MAX_STATES  8

/* 系统模型函数类型 */
typedef float (*FL_ScalarFunc_t)(const float *x, void *user_data);
/* 向量场函数: 输出到y数组 */
typedef void (*FL_VectorFunc_t)(const float *x, float *y, void *user_data);

typedef struct {
    /* --- 配置参数 --- */
    float   Ts;        /* 采样周期(s) */
    int32_t n;         /* 状态维数 */
    int32_t rel_degree;/* 相对阶 */

    /* 线性化后PD控制器增益 */
    float Kp;          /* 比例增益 */
    float Kd;          /* 微分增益 */
    float Ki;          /* 积分增益(可选) */

    /* 系统模型 */
    FL_VectorFunc_t f_func;  /* f(x) 向量场 */
    FL_VectorFunc_t g_func;  /* g(x) 向量场 */
    FL_ScalarFunc_t h_func;  /* h(x) 输出函数 */
    void *user_data;

    /* --- 运行时变量 --- */
    float x[FL_MAX_STATES];     /* 当前状态 */
    float integral_err;          /* 积分误差 */
    float y_prev;                /* 上一时刻输出 */
} FeedLinCtrl_t;

/**
 * @brief 初始化反馈线性化控制器
 * @param fl         控制器句柄
 * @param n          状态维数
 * @param rel_degree 相对阶(通常1或2)
 * @param Ts         采样周期(s)
 * @return 0=成功
 */
int FL_Init(FeedLinCtrl_t *fl, int32_t n, int32_t rel_degree, float Ts);

/**
 * @brief 设置系统模型
 */
void FL_SetModel(FeedLinCtrl_t *fl, FL_VectorFunc_t f, FL_VectorFunc_t g, FL_ScalarFunc_t h);

/**
 * @brief 设置线性化后控制器增益(PD)
 */
void FL_SetPDGains(FeedLinCtrl_t *fl, float Kp, float Kd);

/**
 * @brief 设置线性化后控制器增益(PID)
 */
void FL_SetPIDGains(FeedLinCtrl_t *fl, float Kp, float Kd, float Ki);

/**
 * @brief 反馈线性化控制计算
 * @param fl  控制器句柄
 * @param x   当前状态数组
 * @param ref 期望输出
 * @return 控制量u
 */
float FL_Compute(FeedLinCtrl_t *fl, const float *x, float ref);

/**
 * @brief 重置控制器
 */
void FL_Reset(FeedLinCtrl_t *fl);

#ifdef __cplusplus
}
#endif

#endif /* FEEDBACK_LINEARIZATION_H */
