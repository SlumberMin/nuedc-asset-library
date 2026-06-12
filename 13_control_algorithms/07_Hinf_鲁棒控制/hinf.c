#include "hinf.h"
#include <string.h>
#include <math.h>

/*
 * H∞鲁棒控制简化实现
 * 
 * 使用迭代Riccati方程求解次优H∞控制器
 * P*A + A^T*P - P*(B*R^{-1}*B^T - gamma^{-2}*B1*B1^T)*P + Q = 0
 * K = R^{-1}*B^T*P
 */

void HInf_Init(HInf_t *hinf, int nx, float gamma)
{
    memset(hinf, 0, sizeof(HInf_t));
    hinf->nx = nx;
    hinf->gamma = gamma;
    hinf->rho = 1.0f / (gamma * gamma);
    hinf->u_max = 100;
}

void HInf_SetSystem(HInf_t *hinf, float A[][HINF_NX], float B[], float B1[], float C1[])
{
    int n = hinf->nx;
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++)
            hinf->A[i][j] = A[i][j];
        hinf->B[i] = B[i];
        hinf->B1[i] = B1[i];
        hinf->C1[i] = C1[i];
    }
}

void HInf_SetOutputLimit(HInf_t *hinf, float max) { hinf->u_max = max; }

/*
 * 迭代求解修改的Riccati方程
 * X*A + A^T*X - X*(B*B^T - rho*B1*B1^T)*X + C1^T*C1 = 0
 * 
 * 离散迭代: X_{k+1} = A^T*X_k*A + Q - A^T*X_k*(B*B^T - rho*B1*B1^T)*X_k*A
 *            / (I + (B*B^T - rho*B1*B1^T)*X_k)
 */
int HInf_Solve(HInf_t *hinf, int max_iter)
{
    int n = hinf->nx;
    float X[HINF_NX][HINF_NX], Xn[HINF_NX][HINF_NX];
    float AT[HINF_NX][HINF_NX];
    float Q[HINF_NX][HINF_NX];
    
    /* AT = A^T */
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            AT[i][j] = hinf->A[j][i];
    
    /* Q = C1^T * C1 */
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            Q[i][j] = hinf->C1[i] * hinf->C1[j];
    
    /* BB = B*B^T - rho*B1*B1^T */
    float BB[HINF_NX][HINF_NX];
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            BB[i][j] = hinf->B[i]*hinf->B[j] - hinf->rho*hinf->B1[i]*hinf->B1[j];
    
    /* 初始化 X = Q */
    memcpy(X, Q, sizeof(X));
    
    float gamma2 = hinf->gamma * hinf->gamma;

    for (int iter = 0; iter < max_iter; iter++) {
        /* A^T*X*A */
        float AXA[HINF_NX][HINF_NX];
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++) {
                AXA[i][j] = 0;
                for (int k = 0; k < n; k++)
                    for (int l = 0; l < n; l++)
                        AXA[i][j] += AT[i][k] * X[k][l] * hinf->A[l][j];
            }
        
        /* X*BB*X */
        float XBB[HINF_NX][HINF_NX], XBBX[HINF_NX][HINF_NX];
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++) {
                XBB[i][j] = 0;
                for (int k = 0; k < n; k++)
                    XBB[i][j] += X[i][k] * BB[k][j];
            }
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++) {
                XBBX[i][j] = 0;
                for (int k = 0; k < n; k++)
                    XBBX[i][j] += XBB[i][k] * X[k][j];
            }
        
        /* Xn = AXA + C1^T*C1 - A^T*X*(B*B^T - X/gamma^2)*X*A */
        float diff = 0;
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++) {
                /* subtract X/gamma^2 term for H∞ robustness */
                float X_gamma2 = X[i][j] / gamma2;
                Xn[i][j] = AXA[i][j] + Q[i][j] - XBBX[i][j] + X_gamma2 * X[i][j];
                diff += fabsf(Xn[i][j] - X[i][j]);
            }
        
        memcpy(X, Xn, sizeof(X));
        if (diff < 1e-6f) break;
    }
    
    /* K = B^T * X */
    for (int j = 0; j < n; j++) {
        hinf->K[j] = 0;
        for (int i = 0; i < n; i++)
            hinf->K[j] += hinf->B[i] * X[i][j];
    }
    
    return 1;
}

float HInf_Update(HInf_t *hinf, float x[])
{
    float u = 0;
    for (int j = 0; j < hinf->nx; j++)
        u -= hinf->K[j] * x[j];
    
    if (u > hinf->u_max) u = hinf->u_max;
    if (u < -hinf->u_max) u = -hinf->u_max;
    return u;
}
