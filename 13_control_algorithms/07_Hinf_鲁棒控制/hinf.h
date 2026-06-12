#ifndef HINF_H
#define HINF_H

/*
 * 鲁棒控制 H∞ (H-infinity Control) 简化实现
 * 
 * 原理：最小化闭环传递函数的H∞范数，保证系统在最坏扰动下的鲁棒性
 * 
 * 简化实现：基于Riccati方程的状态反馈H∞控制器
 * 对于系统 x' = Ax + B1*w + B2*u
 * 性能输出 z = C1*x + D12*u
 * 
 * 求解Riccati不等式得到反馈增益K使||Tzw||∞ < gamma
 * 
 * 适用场景：存在未建模动态、参数不确定性的系统
 * 参数整定：gamma越小鲁棒性越强但保守性越大，通常gamma=1~5
 */

#define HINF_NX 4  /* 最大状态维数 */

typedef struct {
    float A[HINF_NX][HINF_NX];
    float B[HINF_NX];        /* 控制输入矩阵 */
    float B1[HINF_NX];       /* 扰动输入矩阵 */
    float C1[HINF_NX];       /* 性能输出矩阵 */
    float K[HINF_NX];        /* 反馈增益 */
    float gamma;             /* 性能指标 */
    float rho;               /* 正则化参数 */
    int nx;
    float u_max;
} HInf_t;

void HInf_Init(HInf_t *hinf, int nx, float gamma);
void HInf_SetSystem(HInf_t *hinf, float A[][HINF_NX], float B[], float B1[], float C1[]);
int  HInf_Solve(HInf_t *hinf, int max_iter);
float HInf_Update(HInf_t *hinf, float x[]);
void HInf_SetOutputLimit(HInf_t *hinf, float max);

#endif
