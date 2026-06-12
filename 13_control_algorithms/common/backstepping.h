/**
 * @file backstepping.h
 * @brief 反步法控制器 - 严格反馈非线性系统
 * 
 * 反步法(Backstepping)适用于严格反馈形式的非线性系统：
 *   ẋ₁ = f₁(x₁) + g₁(x₁)x₂
 *   ẋ₂ = f₂(x₁,x₂) + g₂(x₁,x₂)x₃
 *   ...
 *   ẋₙ = fₙ(x) + gₙ(x)u
 * 
 * 通过逐步构造Lyapunov函数和虚拟控制律，保证系统全局渐近稳定。
 * 
 * 典型应用：电机速度/位置控制、无人机姿态控制、机械臂控制等
 * 
 * 参数整定指南：
 *   - c[i]：每步正定增益，越大收敛越快，通常1~20
 *   - 设计时需已知系统模型函数f(x), g(x)
 *   - 建议从小增益开始逐步调大
 */

#ifndef BACKSTEPPING_H
#define BACKSTEPPING_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 最大支持5阶系统 */
#define BS_MAX_ORDER  5

/* 虚拟控制函数指针类型 */
/* x[]: 系统状态, 返回值: f(x) 或 g(x) */
typedef float (*BS_Func_t)(const float *x, void *user_data);

typedef struct {
    /* --- 配置参数 --- */
    int32_t order;                    /* 系统阶数 */
    float   c[BS_MAX_ORDER];          /* 每步正定增益 */
    float   Ts;                       /* 采样周期(s) */
    
    /* 系统模型函数指针数组 */
    BS_Func_t f[BS_MAX_ORDER];        /* f_i(x) */
    BS_Func_t g[BS_MAX_ORDER];        /* g_i(x) */
    void     *user_data;              /* 用户数据 */
    
    /* --- 运行时变量 --- */
    float x[BS_MAX_ORDER];            /* 当前状态 */
    float alpha[BS_MAX_ORDER - 1];    /* 虚拟控制律 */
    float z[BS_MAX_ORDER];            /* 误差变量 */
} BacksteppingCtrl_t;

/**
 * @brief 初始化反步法控制器
 * @param bs      控制器句柄
 * @param order   系统阶数(2~5)
 * @param Ts      采样周期(s)
 * @return 0=成功
 */
int BS_Init(BacksteppingCtrl_t *bs, int32_t order, float Ts);

/**
 * @brief 设置系统模型函数
 * @param step   第几步(0~order-1)
 * @param f_func f_i(x)
 * @param g_func g_i(x)
 */
void BS_SetModel(BacksteppingCtrl_t *bs, int32_t step, BS_Func_t f_func, BS_Func_t g_func);

/**
 * @brief 设置每步增益
 */
void BS_SetGains(BacksteppingCtrl_t *bs, const float *c);

/**
 * @brief 反步法控制计算
 * @param bs    控制器句柄
 * @param x     当前状态数组(order个元素)
 * @param xd    期望状态(标量，最终跟踪目标)
 * @return 控制输出u
 */
float BS_Compute(BacksteppingCtrl_t *bs, const float *x, float xd);

/**
 * @brief 重置控制器
 */
void BS_Reset(BacksteppingCtrl_t *bs);

#ifdef __cplusplus
}
#endif

#endif /* BACKSTEPPING_H */
