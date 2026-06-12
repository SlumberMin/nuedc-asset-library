#include "lqr.h"
#include <string.h>
#include <math.h>

/*
 * LQR核心实现
 * 使用迭代法求解离散代数Riccati方程(DARE)
 * P = A^T*P*A - A^T*P*B*(R+B^T*P*B)^{-1}*B^T*P*A + Q
 */

static void mat_mul(float *C, float *A, float *B, int m, int n, int p)
{
    for (int i = 0; i < m; i++)
        for (int j = 0; j < p; j++) {
            C[i*p+j] = 0;
            for (int k = 0; k < n; k++)
                C[i*p+j] += A[i*n+k] * B[k*p+j];
        }
}

static void mat_trans(float *AT, float *A, int m, int n)
{
    for (int i = 0; i < m; i++)
        for (int j = 0; j < n; j++)
            AT[j*m+i] = A[i*n+j];
}

static void mat_add(float *C, float *A, float *B, int m, int n, float sa, float sb)
{
    for (int i = 0; i < m*n; i++) C[i] = sa*A[i] + sb*B[i];
}

void LQR_Init(LQR_t *lqr, int nx, int nu)
{
    memset(lqr, 0, sizeof(LQR_t));
    lqr->nx = nx; lqr->nu = nu;
    lqr->u_max = 100;
}

void LQR_SetSystem(LQR_t *lqr, float A[][LQR_NX], float B[][LQR_NU])
{
    for (int i = 0; i < lqr->nx; i++)
        for (int j = 0; j < lqr->nx; j++)
            lqr->A[i][j] = A[i][j];
    for (int i = 0; i < lqr->nx; i++)
        for (int j = 0; j < lqr->nu; j++)
            lqr->B[i][j] = B[i][j];
}

void LQR_SetWeight(LQR_t *lqr, float Q_diag[], float R_diag[])
{
    memset(lqr->Q, 0, sizeof(lqr->Q));
    for (int i = 0; i < lqr->nx; i++) lqr->Q[i][i] = Q_diag[i];
    memset(lqr->R, 0, sizeof(lqr->R));
    for (int i = 0; i < lqr->nu; i++) lqr->R[i][i] = R_diag[i];
}

/*
 * 迭代法求解DARE: P_{k+1} = A^T*P_k*A + Q - A^T*P_k*B*(R+B^T*P_k*B)^{-1}*B^T*P_k*A
 * 适用于小规模系统(nx<=4)
 */
int LQR_SolveRiccati(LQR_t *lqr, int max_iter)
{
    int n = lqr->nx;
    float P[LQR_NX][LQR_NX], Pn[LQR_NX][LQR_NX];
    float AT[LQR_NX][LQR_NX], BT[LQR_NU][LQR_NX];
    float PB[LQR_NX][LQR_NU], BTP[LQR_NU][LQR_NX], ATPA[LQR_NX][LQR_NX];
    float ATPB[LQR_NX][LQR_NU], BTPB[LQR_NU][LQR_NU];
    float Sinv[LQR_NU][LQR_NU], Ktmp[LQR_NU][LQR_NX];
    
    mat_trans(&AT[0][0], &lqr->A[0][0], n, n);
    mat_trans(&BT[0][0], &lqr->B[0][0], n, lqr->nu);
    
    /* 初始化P=Q */
    memcpy(P, lqr->Q, sizeof(P));
    
    for (int iter = 0; iter < max_iter; iter++) {
        /* PB = P*B */
        mat_mul(&PB[0][0], &P[0][0], &lqr->B[0][0], n, n, lqr->nu);
        /* BTP = B^T*P */
        mat_mul(&BTP[0][0], &BT[0][0], &P[0][0], lqr->nu, n, n);
        /* ATPA = A^T*P*A */
        mat_mul(&ATPA[0][0], &AT[0][0], &P[0][0], n, n, n);
        float tmp[LQR_NX][LQR_NX];
        mat_mul(&tmp[0][0], &ATPA[0][0], &lqr->A[0][0], n, n, n);
        memcpy(ATPA, tmp, sizeof(ATPA));
        
        /* BTPB = B^T*P*B + R */
        mat_mul(&BTPB[0][0], &BT[0][0], &PB[0][0], lqr->nu, n, lqr->nu);
        for (int i = 0; i < lqr->nu; i++)
            BTPB[i][i] += lqr->R[i][i];
        
        /* Sinv = (R+B^T*P*B)^{-1}
         * 注意：此处仅对对角元素取倒数，仅当S矩阵为对角阵时正确。
         * 对于nu=1（单输入），B^T*P*B为标量，此简化等价于精确求逆。
         * 对于nu>1且S非对角的情况，需实现通用矩阵求逆（如LU分解）。
         * 当前实现假设R为对角且B^T*P*B近似对角（适合大多数嵌入式单输入场景）。 */
        for (int i = 0; i < lqr->nu; i++)
            Sinv[i][i] = 1.0f / BTPB[i][i];
        
        /* ATPB = A^T*P*B */
        mat_mul(&ATPB[0][0], &AT[0][0], &PB[0][0], n, n, lqr->nu);
        
        /* Ktmp = Sinv * B^T*P*A */
        float BTPA[LQR_NU][LQR_NX];
        mat_mul(&BTPA[0][0], &BTP[0][0], &lqr->A[0][0], lqr->nu, n, n);
        mat_mul(&Ktmp[0][0], &Sinv[0][0], &BTPA[0][0], lqr->nu, lqr->nu, n);
        
        /* Pn = ATPA - ATPB*Sinv*BTPA + Q */
        float corr[LQR_NX][LQR_NX];
        mat_mul(&corr[0][0], &ATPB[0][0], &Ktmp[0][0], n, lqr->nu, n);
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                Pn[i][j] = ATPA[i][j] - corr[i][j] + lqr->Q[i][j];
        
        /* 收敛判断 */
        float diff = 0;
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                diff += fabsf(Pn[i][j] - P[i][j]);
        
        memcpy(P, Pn, sizeof(P));
        if (diff < 1e-6f) break;
    }
    
    /* 计算最优增益 K = (R+B^T*P*B)^{-1}*B^T*P*A */
    mat_mul(&PB[0][0], &P[0][0], &lqr->B[0][0], n, n, lqr->nu);
    mat_mul(&BTP[0][0], &BT[0][0], &P[0][0], lqr->nu, n, n);
    mat_mul(&BTPB[0][0], &BT[0][0], &PB[0][0], lqr->nu, n, lqr->nu);
    for (int i = 0; i < lqr->nu; i++) BTPB[i][i] += lqr->R[i][i];
    for (int i = 0; i < lqr->nu; i++)
        Sinv[i][i] = 1.0f / BTPB[i][i];
    
    float BTPA[LQR_NU][LQR_NX];
    mat_mul(&BTPA[0][0], &BTP[0][0], &lqr->A[0][0], lqr->nu, n, n);
    mat_mul(&lqr->K[0][0], &Sinv[0][0], &BTPA[0][0], lqr->nu, lqr->nu, n);
    
    memcpy(lqr->P, P, sizeof(P));
    return 1;
}

float LQR_Update(LQR_t *lqr, float x[])
{
    float u = 0;
    for (int j = 0; j < lqr->nx; j++)
        u -= lqr->K[0][j] * x[j];
    
    if (u > lqr->u_max) u = lqr->u_max;
    if (u < -lqr->u_max) u = -lqr->u_max;
    return u;
}

void LQR_SetOutputLimit(LQR_t *lqr, float max) { lqr->u_max = max; }
