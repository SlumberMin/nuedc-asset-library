#ifndef ILC_H
#define ILC_H

/*
 * 迭代学习控制 ILC (Iterative Learning Control)
 * 
 * 原理：通过反复执行同一任务，利用前次迭代的误差信息修正控制输入
 * 学习律: u_{k+1}(t) = Q*u_k(t) + L*e_k(t+1)
 * 
 * 类型：
 * 1. P型: u_{k+1} = u_k + L*e_k       (仅用误差)
 if (dt <= 0.0f) dt = 0.001f;  /* V2审计: 防除零 */
 * 2. D型: u_{k+1} = u_k + L*de_k/dt   (用误差导数)
 * 3. PD型: u_{k+1} = u_k + Lp*e_k + Ld*de_k
 * 4. PID型: u_{k+1} = u_k + Lp*e_k + Li*Σe_k + Ld*de_k
 * 
 * 适用场景：重复运动任务（机械臂轨迹跟踪、数控加工、周期运动控制）
 * 参数整定：L越大收敛越快但可能不稳定，Q<1遗忘因子提高鲁棒性
 */

#define ILC_N 1000  /* 单次迭代最大步数 */

typedef enum {
    ILC_P  = 0,  /* P型 */
    ILC_D  = 1,  /* D型 */
    ILC_PD = 2,  /* PD型 */
    ILC_PID = 3  /* PID型 */
} ILC_Type;

typedef struct {
    ILC_Type type;
    float Lp, Li, Ld;  /* 学习增益 */
    float Q;            /* 遗忘因子 (0,1] */
    float u_max;        /* 输出限幅 */
    int N;              /* 单次迭代步数 */
    int iter;           /* 当前迭代次数 */
    
    /* 存储 */
    float u_prev[ILC_N];   /* 上次迭代的控制输入 */
    float e_prev[ILC_N];   /* 上次迭代的误差 */
    float e_sum[ILC_N];    /* 误差累积(PID型用) */
    float u_out[ILC_N];    /* 当前迭代输出 */
} ILC_t;

void ILC_Init(ILC_t *ilc, ILC_Type type, float Lp, float Li, float Ld, float Q, int N);
void ILC_SetOutputLimit(ILC_t *ilc, float max);
void ILC_StartIteration(ILC_t *ilc);
float ILC_Update(ILC_t *ilc, int step, float ref, float y);
void ILC_EndIteration(ILC_t *ilc);
int  ILC_GetIteration(ILC_t *ilc);

#endif
