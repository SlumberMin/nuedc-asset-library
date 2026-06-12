#ifndef LQR_H
#define LQR_H

/*
 * 线性二次调节器 LQR (Linear Quadratic Regulator)
 * 
 * 原理：最小化代价函数 J = Σ[x^T*Q*x + u^T*R*u]
 * 最优控制律: u = -K*x, K = R^{-1}*B^T*P
 * P满足代数Riccati方程: A^T*P + P*A - P*B*R^{-1}*B^T*P + Q = 0
 * 
 * 适用场景：倒立摆、电机控制、航天器姿态
 * 参数整定：Q对角线越大对应状态响应越快，R越大控制越小
 */

#define LQR_NX 4  /* 状态维数（最大支持4阶） */
#define LQR_NU 1  /* 控制维数 */

typedef struct {
    float A[LQR_NX][LQR_NX];
    float B[LQR_NX][LQR_NU];
    float Q[LQR_NX][LQR_NX];
    float R[LQR_NU][LQR_NU];
    float K[LQR_NU][LQR_NX];  /* 反馈增益 */
    float P[LQR_NX][LQR_NX];  /* Riccati解 */
    int nx, nu;
    float u_max;
} LQR_t;

void LQR_Init(LQR_t *lqr, int nx, int nu);
void LQR_SetSystem(LQR_t *lqr, float A[][LQR_NX], float B[][LQR_NU]);
void LQR_SetWeight(LQR_t *lqr, float Q_diag[], float R_diag[]);
int  LQR_SolveRiccati(LQR_t *lqr, int max_iter);
float LQR_Update(LQR_t *lqr, float x[]);
void LQR_SetOutputLimit(LQR_t *lqr, float max);

#endif
